// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xiaomi "MCA" ADSP charger/battery property transport (Xiaomi Pad 8 Pro)
 *
 * The piano ADSP firmware does not speak the standard Qualcomm battmgr
 * protocol.  It exposes the charger ICs and the ADC of the sub-PMIC as a flat
 * property space on the PMIC_RTR_ADSP_APPS glink channel, owner 0x800a (the
 * same owner battmgr uses), with a request/response format of its own:
 *
 *   request  { owner, type = 1, opcode (1 = read, 2 = write), property, seq,
 *              data[256] }
 *   response { owner, type = 1, opcode, property, retcode, seq, data[256] }
 *
 * Property ids are 32 bits wide; bits 31:16 select a class (0: charger and
 * battery, 1: wireless, 2: USB and PD).
 *
 * This driver replaces qcom_battmgr on this board.  By default only reads
 * are issued and charging limits stay with the ADSP firmware defaults (float
 * voltage 4350 mV, charge current 3000 mA, JEITA window 0-55 degC).  With
 * charge_policy=1 it raises the input current limit by charger type, the
 * only property it ever writes (see piano_mca_write_u32()).  The battery itself
 * is reported by the two bq27z561 fuel gauges; this driver adds the USB
 * input (presence from the bus voltage, bus current, input current limit and
 * the charger type detected by the ADSP).
 */

#include <linux/auxiliary_bus.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>

#define MCA_DATA_LEN		256
#define MCA_OP_READ		1
#define MCA_OP_WRITE		2

/* Property ids recovered from the stock charger stack */
#define MCA_PROP_ICL		0x2008	/* input current limit, mA */
#define MCA_PROP_PACK_TBAT	0x202a	/* degC */
#define MCA_PROP_BUS_VOLT	0x20002	/* uV */
#define MCA_PROP_BUS_CURR	0x20004	/* uA */
#define MCA_PROP_REAL_TYPE	0x2000c	/* enum mca_real_type */

/* Up to seven source PDOs, as decoded by the ADSP */
#define MCA_PROP_PDOS		0x21011	/* struct piano_mca_pdo[] */
#define MCA_PD_MAX_PDOS		7

struct piano_mca_pdo {
	__le32 min_mv;
	__le32 max_mv;
	__le32 max_ma;
};

/*
 * Charger types as named by the stock sysfs "real_type" table; 13 and 16
 * are unnamed there, 17-19 are wireless.
 */
enum mca_real_type {
	MCA_TYPE_UNKNOWN,
	MCA_TYPE_SDP,
	MCA_TYPE_CDP,
	MCA_TYPE_DCP,
	MCA_TYPE_FLOAT,
	MCA_TYPE_HVDCP,
	MCA_TYPE_HVDCP_3,
	MCA_TYPE_HVDCP_3_B,
	MCA_TYPE_HVDCP_3P5,
	MCA_TYPE_C,
	MCA_TYPE_PD,
	MCA_TYPE_PD_VERIFY,
	MCA_TYPE_PD_PPS,
	MCA_TYPE_ACA = 14,
	MCA_TYPE_DCP_B,
	MCA_TYPE_MAX = 20,
};

static const char * const mca_real_type_names[MCA_TYPE_MAX] = {
	[MCA_TYPE_UNKNOWN]	= "Unknown",
	[MCA_TYPE_SDP]		= "SDP",
	[MCA_TYPE_CDP]		= "CDP",
	[MCA_TYPE_DCP]		= "DCP",
	[MCA_TYPE_FLOAT]	= "USB_FLOAT",
	[MCA_TYPE_HVDCP]	= "HVDCP",
	[MCA_TYPE_HVDCP_3]	= "HVDCP_3",
	[MCA_TYPE_HVDCP_3_B]	= "HVDCP_3_B",
	[MCA_TYPE_HVDCP_3P5]	= "HVDCP_3P5",
	[MCA_TYPE_C]		= "C",
	[MCA_TYPE_PD]		= "PD",
	[MCA_TYPE_PD_VERIFY]	= "PD_PPS",
	[MCA_TYPE_PD_PPS]	= "PD_PPS",
	[MCA_TYPE_ACA]		= "ACA",
	[MCA_TYPE_DCP_B]	= "DCP",
	[17]			= "BPP",
	[18]			= "EPP",
	[19]			= "HPP",
};

/* VBUS above this means a source is attached (USB default is 5 V) */
#define MCA_VBUS_ONLINE_UV	4000000
#define MCA_POLL_MS		3000

/* Input current limits (mA) set by charge_policy, from the stock tables */
#define MCA_ICL_MIN_MA		100	/* also the ADSP power-on value */
#define MCA_ICL_MAX_MA		1500
#define MCA_ICL_DEFAULT_MA	500
/* Battery temperature window (degC) outside which the limit is not raised */
#define MCA_POLICY_TBAT_MIN	15
#define MCA_POLICY_TBAT_MAX	45

static bool charge_policy;
module_param(charge_policy, bool, 0444);
MODULE_PARM_DESC(charge_policy,
		 "Raise the USB input current limit by charger type (default: off, ADSP defaults)");

struct mca_req {
	__le32 owner;
	__le32 type;
	__le32 opcode;
	__le32 property;
	__le32 seq;
	u8 data[MCA_DATA_LEN];
} __packed;

struct mca_resp {
	__le32 owner;
	__le32 type;
	__le32 opcode;
	__le32 property;
	__le32 retcode;
	__le32 seq;
	u8 data[MCA_DATA_LEN];
} __packed;

struct piano_mca {
	struct device *dev;
	struct pmic_glink_client *client;
	struct mutex lock;
	struct completion ack;
	u32 seq;
	bool service_up;
	int retcode;
	u8 data[MCA_DATA_LEN];
	struct power_supply *usb;
	struct delayed_work poll;
	bool online;
	int vbus_uv;
	int ibus_ua;
	int icl_ma;
	u32 real_type;
	int icl_set_ma;		/* last limit written by charge_policy, 0 if none */
	struct dentry *dbg;
	u32 dbg_prop;
	u8 dbg_data[MCA_DATA_LEN];
	int dbg_ret;
};

static void piano_mca_callback(const void *data, size_t len, void *priv)
{
	struct piano_mca *mca = priv;
	const struct mca_resp *resp = data;

	if (le32_to_cpu(resp->type) == PMIC_GLINK_NOTIFY) {
		/* notification: id at +0xc, payload from +0x10 */
		dev_dbg(mca->dev, "notify %#x len %zu\n",
			le32_to_cpu(resp->property), len);
		mod_delayed_work(system_percpu_wq, &mca->poll, 0);
		return;
	}

	if (len != sizeof(*resp)) {
		dev_warn(mca->dev, "unexpected response length %zu\n", len);
		return;
	}

	if (le32_to_cpu(resp->seq) != mca->seq) {
		dev_dbg(mca->dev, "stale response seq %u != %u\n",
			le32_to_cpu(resp->seq), mca->seq);
		return;
	}

	mca->retcode = le32_to_cpu(resp->retcode);
	if (!mca->retcode && le32_to_cpu(resp->opcode) == MCA_OP_READ)
		memcpy(mca->data, resp->data, MCA_DATA_LEN);
	complete(&mca->ack);
}

static void piano_mca_pdr_notify(void *priv, int state)
{
	struct piano_mca *mca = priv;

	mca->service_up = state == SERVREG_SERVICE_STATE_UP;
	/* A restarted ADSP is back on its defaults */
	mca->icl_set_ma = 0;
	mod_delayed_work(system_percpu_wq, &mca->poll, 0);
}

/*
 * One request/response exchange.  A read copies up to 256 bytes of the reply
 * to @buf; a write sends @len bytes of @buf.
 */
static int piano_mca_xfer(struct piano_mca *mca, u32 opcode, u32 prop,
			  void *buf, size_t len)
{
	struct mca_req req = {
		.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.opcode = cpu_to_le32(opcode),
		.property = cpu_to_le32(prop),
	};
	int ret;

	if (len > MCA_DATA_LEN)
		return -EINVAL;
	if (opcode == MCA_OP_WRITE)
		memcpy(req.data, buf, len);

	mutex_lock(&mca->lock);
	if (!mca->service_up) {
		ret = -ENODEV;
		goto out;
	}

	mca->seq++;
	req.seq = cpu_to_le32(mca->seq);
	reinit_completion(&mca->ack);
	mca->retcode = -ETIMEDOUT;

	ret = pmic_glink_send(mca->client, &req, sizeof(req));
	if (ret < 0)
		goto out;

	if (!wait_for_completion_timeout(&mca->ack, msecs_to_jiffies(1000))) {
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = mca->retcode ? -EIO : 0;
	if (!ret && opcode == MCA_OP_READ)
		memcpy(buf, mca->data, len);
out:
	mutex_unlock(&mca->lock);
	return ret;
}

/* Read a property; up to 256 bytes are copied to @buf. */
static int piano_mca_read(struct piano_mca *mca, u32 prop, void *buf, size_t len)
{
	return piano_mca_xfer(mca, MCA_OP_READ, prop, buf, len);
}

/*
 * The only write path.  Every property it accepts is listed here with its
 * allowed range; anything else (float voltage, charge current, ship mode,
 * boost, ...) is refused, whatever the caller.
 */
static int piano_mca_write_u32(struct piano_mca *mca, u32 prop, u32 val)
{
	__le32 data = cpu_to_le32(val);

	switch (prop) {
	case MCA_PROP_ICL:
		if (val < MCA_ICL_MIN_MA || val > MCA_ICL_MAX_MA)
			return -ERANGE;
		break;
	default:
		return -EPERM;
	}

	return piano_mca_xfer(mca, MCA_OP_WRITE, prop, &data, sizeof(data));
}

static u32 piano_mca_read_u32(struct piano_mca *mca, u32 prop)
{
	__le32 val;

	if (piano_mca_read(mca, prop, &val, sizeof(val)))
		return 0;

	return le32_to_cpu(val);
}

static const char *piano_mca_type_name(u32 type)
{
	return type < MCA_TYPE_MAX ? mca_real_type_names[type] : NULL;
}

static u32 piano_mca_policy_icl(u32 real_type)
{
	switch (real_type) {
	case MCA_TYPE_CDP:
		return 900;
	/* QC and PD sources stay at 5 V, so they count as DCPs here */
	case MCA_TYPE_DCP:
	case MCA_TYPE_DCP_B:
	case MCA_TYPE_HVDCP:
	case MCA_TYPE_HVDCP_3:
	case MCA_TYPE_HVDCP_3_B:
	case MCA_TYPE_HVDCP_3P5:
	case MCA_TYPE_PD:
	case MCA_TYPE_PD_VERIFY:
	case MCA_TYPE_PD_PPS:
		return 1500;
	default:
		return MCA_ICL_DEFAULT_MA;
	}
}

/*
 * Current a PD source offers at 5 V: the lowest maximum among the PDOs and
 * APDOs whose range covers 5 V (some PPS sources list only a 5-11 V APDO),
 * or the default if there is none.
 */
static u32 piano_mca_pd_5v_ma(struct piano_mca *mca)
{
	struct piano_mca_pdo pdos[MCA_PD_MAX_PDOS];
	u32 ma = U32_MAX;
	int i;

	if (piano_mca_read(mca, MCA_PROP_PDOS, pdos, sizeof(pdos)))
		return MCA_ICL_DEFAULT_MA;

	for (i = 0; i < MCA_PD_MAX_PDOS; i++) {
		if (le32_to_cpu(pdos[i].min_mv) <= 5000 &&
		    le32_to_cpu(pdos[i].max_mv) >= 5000)
			ma = min(ma, le32_to_cpu(pdos[i].max_ma));
	}

	return ma && ma != U32_MAX ? ma : MCA_ICL_DEFAULT_MA;
}

/*
 * The ADSP keeps the last limit written for the next source, so put its
 * power-on value back on detach and unbind.  Retried by the next poll if
 * the write fails.
 */
static void piano_mca_reset_icl(struct piano_mca *mca)
{
	if (!piano_mca_write_u32(mca, MCA_PROP_ICL, MCA_ICL_MIN_MA))
		mca->icl_set_ma = 0;
}

/*
 * charge_policy: raise the input current limit to what the detected charger
 * type allows, never above what a PD source advertises at 5 V, and only
 * within the battery temperature window.  Voltage, charge current and JEITA
 * stay with the ADSP, and its AICL still backs the limit off if the source
 * sags.
 */
static void piano_mca_apply_policy(struct piano_mca *mca)
{
	u32 target = piano_mca_policy_icl(mca->real_type);
	int tbat = (s32)piano_mca_read_u32(mca, MCA_PROP_PACK_TBAT);
	int ret;

	switch (mca->real_type) {
	case MCA_TYPE_PD:
	case MCA_TYPE_PD_VERIFY:
	case MCA_TYPE_PD_PPS:
		target = clamp(piano_mca_pd_5v_ma(mca), MCA_ICL_MIN_MA, target);
		break;
	}
	if (tbat < MCA_POLICY_TBAT_MIN || tbat > MCA_POLICY_TBAT_MAX)
		target = MCA_ICL_DEFAULT_MA;

	if (target == mca->icl_set_ma)
		return;

	ret = piano_mca_write_u32(mca, MCA_PROP_ICL, target);
	if (ret) {
		dev_warn(mca->dev, "failed to set input limit %u mA: %d\n",
			 target, ret);
		return;
	}

	dev_info(mca->dev, "%s: input limit %u mA (battery %d degC)\n",
		 piano_mca_type_name(mca->real_type) ?: "Unknown", target, tbat);
	mca->icl_set_ma = target;
}

static void piano_mca_poll(struct work_struct *work)
{
	struct piano_mca *mca = container_of(work, struct piano_mca, poll.work);
	bool online, changed;
	u32 real_type = MCA_TYPE_UNKNOWN;

	mca->vbus_uv = piano_mca_read_u32(mca, MCA_PROP_BUS_VOLT);
	online = mca->vbus_uv >= MCA_VBUS_ONLINE_UV;

	/* The rest only means something with a source attached */
	if (online) {
		mca->ibus_ua = piano_mca_read_u32(mca, MCA_PROP_BUS_CURR);
		mca->icl_ma = piano_mca_read_u32(mca, MCA_PROP_ICL);
		real_type = piano_mca_read_u32(mca, MCA_PROP_REAL_TYPE);
	} else {
		mca->ibus_ua = 0;
		mca->icl_ma = 0;
	}

	changed = online != mca->online || real_type != mca->real_type;
	mca->online = online;
	mca->real_type = real_type;

	if (!online) {
		if (mca->icl_set_ma)
			piano_mca_reset_icl(mca);
	} else if (charge_policy) {
		piano_mca_apply_policy(mca);
	}
	if (changed)
		power_supply_changed(mca->usb);

	if (mca->service_up)
		schedule_delayed_work(&mca->poll, msecs_to_jiffies(MCA_POLL_MS));
}

static enum power_supply_usb_type piano_mca_usb_type(u32 real_type)
{
	switch (real_type) {
	case MCA_TYPE_SDP:
		return POWER_SUPPLY_USB_TYPE_SDP;
	case MCA_TYPE_CDP:
		return POWER_SUPPLY_USB_TYPE_CDP;
	/* QC adapters are DCPs that can be asked for a higher voltage */
	case MCA_TYPE_DCP:
	case MCA_TYPE_DCP_B:
	case MCA_TYPE_HVDCP:
	case MCA_TYPE_HVDCP_3:
	case MCA_TYPE_HVDCP_3_B:
	case MCA_TYPE_HVDCP_3P5:
		return POWER_SUPPLY_USB_TYPE_DCP;
	case MCA_TYPE_ACA:
		return POWER_SUPPLY_USB_TYPE_ACA;
	case MCA_TYPE_C:
		return POWER_SUPPLY_USB_TYPE_C;
	case MCA_TYPE_PD:
		return POWER_SUPPLY_USB_TYPE_PD;
	case MCA_TYPE_PD_VERIFY:
	case MCA_TYPE_PD_PPS:
		return POWER_SUPPLY_USB_TYPE_PD_PPS;
	default:
		return POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}
}

static int piano_mca_usb_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct piano_mca *mca = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mca->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = mca->online ? mca->vbus_uv : 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = mca->ibus_ua;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = mca->icl_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = piano_mca_usb_type(mca->real_type);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property piano_mca_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc piano_mca_usb_desc = {
	.name = "piano-mca-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA) |
		     BIT(POWER_SUPPLY_USB_TYPE_C) |
		     BIT(POWER_SUPPLY_USB_TYPE_PD) |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS),
	.properties = piano_mca_usb_props,
	.num_properties = ARRAY_SIZE(piano_mca_usb_props),
	.get_property = piano_mca_usb_get_property,
};

/* The ADSP's own name of the charger type, finer than usb_type */
static ssize_t charger_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct piano_mca *mca = power_supply_get_drvdata(to_power_supply(dev));
	u32 type = mca->online ? mca->real_type : MCA_TYPE_UNKNOWN;
	const char *name = piano_mca_type_name(type);

	if (!name)
		return sysfs_emit(buf, "Unknown (%u)\n", type);

	return sysfs_emit(buf, "%s\n", name);
}
static DEVICE_ATTR_RO(charger_type);

static struct attribute *piano_mca_usb_attrs[] = {
	&dev_attr_charger_type.attr,
	NULL,
};
ATTRIBUTE_GROUPS(piano_mca_usb);

/* The gauges re-read their status as soon as the input changes */
static char *piano_mca_supplied_to[] = { "bq27z561-0", "bq27z561-1" };

static void piano_mca_stop(void *data)
{
	struct piano_mca *mca = data;

	cancel_delayed_work_sync(&mca->poll);
	if (mca->icl_set_ma && mca->service_up)
		piano_mca_reset_icl(mca);
}

/* debugfs: write a property id to "prop", read the raw reply from "data" */
static ssize_t piano_mca_dbg_prop_write(struct file *file, const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct piano_mca *mca = file->private_data;
	char kbuf[16];
	u32 prop;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	if (kstrtou32(strim(kbuf), 0, &prop))
		return -EINVAL;

	mca->dbg_prop = prop;
	mca->dbg_ret = piano_mca_read(mca, prop, mca->dbg_data, MCA_DATA_LEN);
	return count;
}

static ssize_t piano_mca_dbg_data_read(struct file *file, char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct piano_mca *mca = file->private_data;
	char line[8 + 4 * 32 + 64];
	int n;

	n = scnprintf(line, sizeof(line), "prop=%#x ret=%d data=%*phN\n",
		      mca->dbg_prop, mca->dbg_ret, 32, mca->dbg_data);
	return simple_read_from_buffer(ubuf, count, ppos, line, n);
}

/*
 * Read-only properties dumped by debugfs "summary", for calibrating the
 * charger types.  The UVDM properties are left out on purpose.
 */
static const struct {
	u32 prop;
	u8 len;
	const char *name;
} piano_mca_dbg_summary_props[] = {
	{ 0x2001, 4, "chg_status" },
	{ 0x2002, 4, "chg_type" },
	{ 0x2006, 4, "term_curr" },
	{ 0x2007, 4, "term_volt" },
	{ 0x2008, 4, "input_curr_limit" },
	{ 0x200c, 4, "vsys" },
	{ 0x2015, 4, "aicl_cont_thd" },
	{ 0x2026, 4, "pack_vbat" },
	{ 0x2027, 4, "pack_ibat" },
	{ 0x2029, 4, "aicl_status" },
	{ 0x202a, 4, "pack_tbat" },
	{ 0x20001, 4, "usb_type" },
	{ 0x20002, 4, "bus_volt" },
	{ 0x20003, 4, "usb_sns_volt" },
	{ 0x20004, 4, "bus_curr" },
	{ 0x2000c, 4, "real_type" },
	{ 0x21002, 4, "verify_process" },
	{ 0x2100c, 4, "data_role" },
	{ 0x2100d, 4, "pd_state" },
	{ 0x2100e, 4, "adapter_id" },
	{ 0x2100f, 4, "adapter_svid" },
	{ MCA_PROP_PDOS, sizeof(struct piano_mca_pdo) * MCA_PD_MAX_PDOS, "pdos" },
	{ 0x21013, 4, "pps_max_curr" },
	{ 0x21014, 4, "apdo_max" },
	{ 0x21015, 4, "typec_mode" },
	{ 0x21016, 4, "cc_orientation" },
	{ 0x21019, 4, "pps_status" },
	{ 0x2101e, 1, "has_dp" },
	{ 0x21022, 4, "snk_src_mode" },
	{ 0x21025, 4, "pps_ptf" },
	{ 0x21026, 1, "suspend_support" },
};

static int piano_mca_dbg_summary_show(struct seq_file *s, void *unused)
{
	struct piano_mca *mca = s->private;
	__le32 val[3 * MCA_PD_MAX_PDOS];
	struct piano_mca_pdo *pdo = (void *)val;
	int i, j, ret;

	for (i = 0; i < ARRAY_SIZE(piano_mca_dbg_summary_props); i++) {
		u32 prop = piano_mca_dbg_summary_props[i].prop;
		u8 len = piano_mca_dbg_summary_props[i].len;

		memset(val, 0, sizeof(val));
		ret = piano_mca_read(mca, prop, val, len);
		seq_printf(s, "%#07x %-17s", prop,
			   piano_mca_dbg_summary_props[i].name);
		if (ret) {
			seq_printf(s, " error %d\n", ret);
			continue;
		}
		if (prop == MCA_PROP_PDOS) {
			/* slot:min-max mV/max mA, empty slots skipped */
			bool any = false;

			for (j = 0; j < MCA_PD_MAX_PDOS; j++) {
				if (!pdo[j].max_mv)
					continue;
				seq_printf(s, " %d:%u-%u/%u", j,
					   le32_to_cpu(pdo[j].min_mv),
					   le32_to_cpu(pdo[j].max_mv),
					   le32_to_cpu(pdo[j].max_ma));
				any = true;
			}
			if (!any)
				seq_puts(s, " none");
		} else {
			for (j = 0; j < DIV_ROUND_UP(len, 4); j++)
				seq_printf(s, " %u", le32_to_cpu(val[j]));
		}
		seq_putc(s, '\n');
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(piano_mca_dbg_summary);

static const struct file_operations piano_mca_dbg_prop_fops = {
	.open = simple_open,
	.write = piano_mca_dbg_prop_write,
};

static const struct file_operations piano_mca_dbg_data_fops = {
	.open = simple_open,
	.read = piano_mca_dbg_data_read,
};

static void piano_mca_dbg_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int piano_mca_probe(struct auxiliary_device *adev,
			   const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct power_supply_config psy_cfg = {};
	struct piano_mca *mca;
	int ret;

	mca = devm_kzalloc(dev, sizeof(*mca), GFP_KERNEL);
	if (!mca)
		return -ENOMEM;

	mca->dev = dev;
	mutex_init(&mca->lock);
	init_completion(&mca->ack);
	INIT_DELAYED_WORK(&mca->poll, piano_mca_poll);

	/* Allocated first so it outlives the poll work on unbind */
	mca->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						   piano_mca_callback,
						   piano_mca_pdr_notify, mca);
	if (IS_ERR(mca->client))
		return PTR_ERR(mca->client);

	psy_cfg.drv_data = mca;
	psy_cfg.supplied_to = piano_mca_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(piano_mca_supplied_to);
	psy_cfg.attr_grp = piano_mca_usb_groups;
	mca->usb = devm_power_supply_register(dev, &piano_mca_usb_desc, &psy_cfg);
	if (IS_ERR(mca->usb))
		return PTR_ERR(mca->usb);

	ret = devm_add_action_or_reset(dev, piano_mca_stop, mca);
	if (ret)
		return ret;

	mca->dbg = debugfs_create_dir("piano_mca", NULL);
	debugfs_create_file("prop", 0200, mca->dbg, mca, &piano_mca_dbg_prop_fops);
	debugfs_create_file("data", 0400, mca->dbg, mca, &piano_mca_dbg_data_fops);
	debugfs_create_file("summary", 0400, mca->dbg, mca,
			    &piano_mca_dbg_summary_fops);
	devm_add_action_or_reset(dev, piano_mca_dbg_remove, mca->dbg);

	pmic_glink_client_register(mca->client);
	return 0;
}

static const struct auxiliary_device_id piano_mca_id_table[] = {
	{ .name = "pmic_glink.power-supply", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, piano_mca_id_table);

static struct auxiliary_driver piano_mca_driver = {
	.name = "piano_mca",
	.probe = piano_mca_probe,
	.id_table = piano_mca_id_table,
};
module_auxiliary_driver(piano_mca_driver);

MODULE_DESCRIPTION("Xiaomi MCA ADSP property transport (piano)");
MODULE_LICENSE("GPL");
