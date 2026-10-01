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
 * This driver replaces qcom_battmgr on this board.  Only reads are issued.
 * The battery itself is reported by the two bq27z561 fuel gauges; this driver
 * adds the USB input, polled from the bus voltage of the sub-PMIC ADC.
 */

#include <linux/auxiliary_bus.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>

#define MCA_DATA_LEN		256
#define MCA_OP_READ		1

/* Property ids recovered from the stock charger stack */
#define MCA_PROP_USB_TYPE_A	0x0001
#define MCA_PROP_BUS_CURR	0x0004
#define MCA_PROP_USB_TYPE	0x000c
#define MCA_PROP_CHG_STATUS	0x2001
#define MCA_PROP_CHG_TYPE	0x2002
#define MCA_PROP_PACK_VBAT	0x2026
#define MCA_PROP_PACK_IBAT	0x2027
#define MCA_PROP_PACK_TBAT	0x202a
#define MCA_PROP_BUS_VOLT	0x20002	/* uV */

/* VBUS above this means a source is attached (USB default is 5 V) */
#define MCA_VBUS_ONLINE_UV	4000000
#define MCA_POLL_MS		3000

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
		mod_delayed_work(system_wq, &mca->poll, 0);
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
	mod_delayed_work(system_wq, &mca->poll, 0);
}

/* Read a property; up to 256 bytes are copied to @buf. */
static int piano_mca_read(struct piano_mca *mca, u32 prop, void *buf, size_t len)
{
	struct mca_req req = {
		.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.opcode = cpu_to_le32(MCA_OP_READ),
		.property = cpu_to_le32(prop),
	};
	int ret;

	if (len > MCA_DATA_LEN)
		return -EINVAL;

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
	if (!ret)
		memcpy(buf, mca->data, len);
out:
	mutex_unlock(&mca->lock);
	return ret;
}

static void piano_mca_poll(struct work_struct *work)
{
	struct piano_mca *mca = container_of(work, struct piano_mca, poll.work);
	__le32 vbus;
	bool online;

	if (piano_mca_read(mca, MCA_PROP_BUS_VOLT, &vbus, sizeof(vbus)))
		vbus = 0;

	mca->vbus_uv = le32_to_cpu(vbus);
	online = mca->vbus_uv >= MCA_VBUS_ONLINE_UV;
	if (online != mca->online) {
		mca->online = online;
		power_supply_changed(mca->usb);
	}

	if (mca->service_up)
		schedule_delayed_work(&mca->poll, msecs_to_jiffies(MCA_POLL_MS));
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
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property piano_mca_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc piano_mca_usb_desc = {
	.name = "piano-mca-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.properties = piano_mca_usb_props,
	.num_properties = ARRAY_SIZE(piano_mca_usb_props),
	.get_property = piano_mca_usb_get_property,
};

/* The gauges re-read their status as soon as the input changes */
static char *piano_mca_supplied_to[] = { "bq27z561-0", "bq27z561-1" };

static void piano_mca_cancel_poll(void *data)
{
	cancel_delayed_work_sync(data);
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
	mca->usb = devm_power_supply_register(dev, &piano_mca_usb_desc, &psy_cfg);
	if (IS_ERR(mca->usb))
		return PTR_ERR(mca->usb);

	ret = devm_add_action_or_reset(dev, piano_mca_cancel_poll, &mca->poll);
	if (ret)
		return ret;

	mca->dbg = debugfs_create_dir("piano_mca", NULL);
	debugfs_create_file("prop", 0200, mca->dbg, mca, &piano_mca_dbg_prop_fops);
	debugfs_create_file("data", 0400, mca->dbg, mca, &piano_mca_dbg_data_fops);
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
