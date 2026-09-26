// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Linaro Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/pwrseq/consumer.h>
#include <linux/pwrseq/provider.h>
#include <linux/pwrseq/qcom_wcn.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qcom_aoss.h>
#include <linux/string.h>
#include <linux/types.h>

struct pwrseq_qcom_wcn_vreg_cfg {
	unsigned int min_uV;
	unsigned int max_uV;
	unsigned int load_uA;
};

struct pwrseq_qcom_wcn_pdata {
	const char *const *vregs;
	const struct pwrseq_qcom_wcn_vreg_cfg *vreg_cfgs;
	size_t num_vregs;
	unsigned int pwup_delay_ms;
	unsigned int gpio_enable_delay_ms;
	unsigned int bt_reset_delay_ms;
	unsigned int bt_wlan_off_delay_ms;
	unsigned int bt_power_off_delay_ms;
	const struct pwrseq_target_data **targets;
	bool has_vddio; /* separate VDD IO regulator */
	int (*match)(struct pwrseq_device *pwrseq, struct device *dev);
};

struct pwrseq_qcom_wcn_map {
	const char *from;
	const char *to;
};

struct pwrseq_qcom_wcn_ctx {
	struct pwrseq_device *pwrseq;
	struct device *dev;
	struct device_node *of_node;
	const struct pwrseq_qcom_wcn_pdata *pdata;
	struct regulator_bulk_data *regs;
	struct regulator *vddio;
	struct gpio_desc *bt_gpio;
	struct gpio_desc *wlan_gpio;
	struct gpio_desc *xo_clk_gpio;
	struct clk *clk;
	struct qmp *qmp;
	/* Serializes shared WLAN/BT PDC register updates. */
	struct mutex pdc_lock;
	struct pwrseq_qcom_wcn_map *vreg_pdc_map;
	size_t num_vreg_pdc_map;
	struct pwrseq_qcom_wcn_map *pmu_vreg_map;
	size_t num_pmu_vreg_map;
	unsigned long last_gpio_enable_jf;
};

#define PWRSEQ_QCOM_WCN_QMP_MSG_LEN	64

static int
pwrseq_qcom_wcn_parse_map(struct pwrseq_qcom_wcn_ctx *ctx,
			  const char *property,
			  struct pwrseq_qcom_wcn_map **map,
			  size_t *num_map)
{
	struct device_node *np = ctx->of_node;
	const char *value;
	char *entry;
	char *sep;
	int count;
	int ret;
	int i, j;

	if (!of_property_present(np, property))
		return 0;

	count = of_property_count_strings(np, property);
	if (count < 1)
		return dev_err_probe(ctx->dev, count < 0 ? count : -EINVAL,
				     "%s must contain from:to entries\n",
				     property);

	*num_map = count;
	*map = devm_kcalloc(ctx->dev, *num_map, sizeof(**map), GFP_KERNEL);
	if (!*map)
		return -ENOMEM;

	for (i = 0; i < *num_map; i++) {
		ret = of_property_read_string_index(np, property, i, &value);
		if (ret)
			return ret;

		entry = devm_kstrdup(ctx->dev, value, GFP_KERNEL);
		if (!entry)
			return -ENOMEM;

		sep = strchr(entry, ':');
		if (!sep || sep == entry || !sep[1] || strchr(sep + 1, ':'))
			return dev_err_probe(ctx->dev, -EINVAL,
					     "%s entry %s must be exactly from:to\n",
					     property, value);

		*sep = '\0';
		(*map)[i].from = entry;
		(*map)[i].to = sep + 1;

		for (j = 0; j < i; j++) {
			if (!strcmp((*map)[j].from, (*map)[i].from))
				return dev_err_probe(ctx->dev, -EINVAL,
						     "%s contains duplicate key %s\n",
						     property, (*map)[i].from);
		}
	}

	return 0;
}

static const char *
pwrseq_qcom_wcn_map_find(const struct pwrseq_qcom_wcn_map *map,
			 size_t num_map, const char *key)
{
	size_t i;

	for (i = 0; i < num_map; i++) {
		if (!strcmp(map[i].from, key))
			return map[i].to;
	}

	return NULL;
}

static int pwrseq_qcom_wcn_validate_maps(struct pwrseq_qcom_wcn_ctx *ctx)
{
	char msg[PWRSEQ_QCOM_WCN_QMP_MSG_LEN];
	const char *pdc;
	int len;
	size_t i;

	for (i = 0; i < ctx->num_vreg_pdc_map; i++) {
		pdc = ctx->vreg_pdc_map[i].to;
		if (strcmp(pdc, "rf") && strcmp(pdc, "bb"))
			return dev_err_probe(ctx->dev, -EINVAL,
					     "Invalid PDC domain %s for rail %s\n",
					     pdc, ctx->vreg_pdc_map[i].from);

		len = snprintf(msg, sizeof(msg),
			       "{class: wlan_pdc, ss: %s, res: %s.v, dwnval: %u}",
			       pdc, ctx->vreg_pdc_map[i].from, U32_MAX);
		if (len >= sizeof(msg))
			return dev_err_probe(ctx->dev, -E2BIG,
					     "PDC mapping for %s exceeds the AOSS message limit\n",
					     ctx->vreg_pdc_map[i].from);
	}

	for (i = 0; i < ctx->num_pmu_vreg_map; i++) {
		if (strnlen(ctx->pmu_vreg_map[i].from,
			    PWRSEQ_QCOM_WCN_PMU_PIN_NAME_LEN + 1) >
		    PWRSEQ_QCOM_WCN_PMU_PIN_NAME_LEN)
			return dev_err_probe(ctx->dev, -E2BIG,
					     "PMU pin name %s exceeds the QMI limit\n",
					     ctx->pmu_vreg_map[i].from);

		pdc = pwrseq_qcom_wcn_map_find(ctx->vreg_pdc_map,
					       ctx->num_vreg_pdc_map,
					       ctx->pmu_vreg_map[i].to);
		if (!pdc)
			return dev_err_probe(ctx->dev, -EINVAL,
					     "PMU pin %s maps to unknown rail %s\n",
					     ctx->pmu_vreg_map[i].from,
					     ctx->pmu_vreg_map[i].to);
	}

	return 0;
}

static int
pwrseq_qcom_wcn_qmp_send(struct pwrseq_qcom_wcn_ctx *ctx, const char *msg)
{
	int ret;

	lockdep_assert_held(&ctx->pdc_lock);

	dev_dbg(ctx->dev, "AOSS QMP tx: %s\n", msg);
	ret = qmp_send(ctx->qmp, "%s", msg);
	if (ret)
		dev_err(ctx->dev, "AOSS QMP result=%d: %s\n", ret, msg);
	else
		dev_dbg(ctx->dev, "AOSS QMP result=0: %s\n", msg);

	return ret;
}

static void pwrseq_qcom_wcn_qmp_put(void *data)
{
	struct pwrseq_qcom_wcn_ctx *ctx = data;

	qmp_put(ctx->qmp);
}

static int pwrseq_qcom_wcn_pdc_init(struct pwrseq_qcom_wcn_ctx *ctx)
{
	struct device_node *np = ctx->of_node;
	const char *msg;
	int count = 0;
	int i, ret;

	if (!of_property_present(np, "qcom,pdc-init-table") &&
	    !ctx->num_vreg_pdc_map && !ctx->num_pmu_vreg_map)
		return 0;

	ctx->qmp = qmp_get(ctx->dev);
	if (IS_ERR(ctx->qmp))
		return dev_err_probe(ctx->dev, PTR_ERR(ctx->qmp),
				     "Failed to acquire the AOSS QMP channel\n");

	ret = devm_add_action_or_reset(ctx->dev, pwrseq_qcom_wcn_qmp_put, ctx);
	if (ret)
		return ret;

	if (!of_property_present(np, "qcom,pdc-init-table"))
		return 0;

	count = of_property_count_strings(np, "qcom,pdc-init-table");
	if (count < 0)
		return dev_err_probe(ctx->dev, count,
				     "Invalid WLAN PDC initialization table\n");

	mutex_lock(&ctx->pdc_lock);
	for (i = 0; i < count; i++) {
		ret = of_property_read_string_index(np, "qcom,pdc-init-table",
						    i, &msg);
		if (ret)
			break;

		ret = pwrseq_qcom_wcn_qmp_send(ctx, msg);
		if (ret)
			break;
	}
	mutex_unlock(&ctx->pdc_lock);
	if (ret)
		return ret;

	dev_dbg(ctx->dev, "Configured WLAN PDC with %d AOSS messages\n", count);

	return 0;
}

static void pwrseq_qcom_wcn_ensure_gpio_delay(struct pwrseq_qcom_wcn_ctx *ctx)
{
	unsigned long diff_jiffies;
	unsigned int diff_msecs;

	if (!ctx->pdata->gpio_enable_delay_ms)
		return;

	diff_jiffies = jiffies - ctx->last_gpio_enable_jf;
	diff_msecs = jiffies_to_msecs(diff_jiffies);

	if (diff_msecs < ctx->pdata->gpio_enable_delay_ms)
		msleep(ctx->pdata->gpio_enable_delay_ms - diff_msecs);
}

static int pwrseq_qcom_wcn_vddio_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_enable(ctx->vddio);
}

static int pwrseq_qcom_wcn_vddio_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_disable(ctx->vddio);
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_vddio_unit_data = {
	.name = "vddio-enable",
	.enable = pwrseq_qcom_wcn_vddio_enable,
	.disable = pwrseq_qcom_wcn_vddio_disable,
};

static int pwrseq_qcom_wcn_vregs_unvote(struct pwrseq_qcom_wcn_ctx *ctx,
					size_t count)
{
	const struct pwrseq_qcom_wcn_vreg_cfg *cfg;
	int first_err = 0;
	int ret;

	if (!ctx->pdata->vreg_cfgs)
		return 0;

	while (count--) {
		cfg = &ctx->pdata->vreg_cfgs[count];

		if (cfg->load_uA) {
			ret = regulator_set_load(ctx->regs[count].consumer, 0);
			if (ret < 0 && !first_err)
				first_err = ret;
		}

		if (cfg->min_uV) {
			ret = regulator_set_voltage(ctx->regs[count].consumer, 0,
						    cfg->max_uV);
			if (ret && !first_err)
				first_err = ret;
		}
	}

	return first_err;
}

static int pwrseq_qcom_wcn_vregs_configure(struct pwrseq_qcom_wcn_ctx *ctx)
{
	const struct pwrseq_qcom_wcn_vreg_cfg *cfg;
	int voltage;
	int ret;
	int i;

	if (!ctx->pdata->vreg_cfgs)
		return 0;

	for (i = 0; i < ctx->pdata->num_vregs; i++) {
		cfg = &ctx->pdata->vreg_cfgs[i];

		if (cfg->min_uV) {
			ret = regulator_set_voltage(ctx->regs[i].consumer,
						    cfg->min_uV, cfg->max_uV);
			if (ret) {
				dev_err(ctx->dev,
					"Failed to set %s voltage to %u-%u uV: %d\n",
					ctx->regs[i].supply, cfg->min_uV,
					cfg->max_uV, ret);
				goto err_unvote;
			}
		}

		if (cfg->load_uA) {
			ret = regulator_set_load(ctx->regs[i].consumer,
						 cfg->load_uA);
			if (ret < 0) {
				dev_err(ctx->dev,
					"Failed to set %s load to %u uA: %d\n",
					ctx->regs[i].supply, cfg->load_uA, ret);
				goto err_unvote;
			}
		}

		voltage = regulator_get_voltage(ctx->regs[i].consumer);
		dev_dbg(ctx->dev,
			"Voted %s voltage=%d uV range=%u-%u uV load=%u uA\n",
			 ctx->regs[i].supply, voltage, cfg->min_uV,
			 cfg->max_uV, cfg->load_uA);
	}

	return 0;

err_unvote:
	pwrseq_qcom_wcn_vregs_unvote(ctx, i + 1);
	return ret;
}

static int pwrseq_qcom_wcn_vregs_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);
	int ret;
	int i;

	ret = pwrseq_qcom_wcn_vregs_configure(ctx);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(ctx->pdata->num_vregs, ctx->regs);
	if (ret) {
		pwrseq_qcom_wcn_vregs_unvote(ctx, ctx->pdata->num_vregs);
		return ret;
	}

	if (ctx->pdata->vreg_cfgs) {
		for (i = 0; i < ctx->pdata->num_vregs; i++)
			dev_dbg(ctx->dev, "Enabled %s at %d uV\n",
				ctx->regs[i].supply,
				 regulator_get_voltage(ctx->regs[i].consumer));
	}

	return 0;
}

static int pwrseq_qcom_wcn_vregs_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);
	int ret;
	int unvote_ret;

	ret = regulator_bulk_disable(ctx->pdata->num_vregs, ctx->regs);
	unvote_ret = pwrseq_qcom_wcn_vregs_unvote(ctx,
						  ctx->pdata->num_vregs);

	return ret ? ret : unvote_ret;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_vregs_unit_data = {
	.name = "regulators-enable",
	.enable = pwrseq_qcom_wcn_vregs_enable,
	.disable = pwrseq_qcom_wcn_vregs_disable,
};

static int pwrseq_qcom_wcn_clk_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return clk_prepare_enable(ctx->clk);
}

static int pwrseq_qcom_wcn_clk_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	clk_disable_unprepare(ctx->clk);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_clk_unit_data = {
	.name = "clock-enable",
	.enable = pwrseq_qcom_wcn_clk_enable,
	.disable = pwrseq_qcom_wcn_clk_disable,
};

static const struct pwrseq_unit_data *pwrseq_qcom_wcn3990_unit_deps[] = {
	&pwrseq_qcom_wcn_vddio_unit_data,
	&pwrseq_qcom_wcn_vregs_unit_data,
	NULL,
};

static const struct pwrseq_unit_data pwrseq_qcom_wcn3990_unit_data = {
	.name = "clock-enable",
	.deps = pwrseq_qcom_wcn3990_unit_deps,
	.enable = pwrseq_qcom_wcn_clk_enable,
	.disable = pwrseq_qcom_wcn_clk_disable,
};

static const struct pwrseq_unit_data *pwrseq_qcom_wcn_unit_deps[] = {
	&pwrseq_qcom_wcn_vregs_unit_data,
	&pwrseq_qcom_wcn_clk_unit_data,
	NULL
};

static int pwrseq_qcom_wcn6855_clk_assert(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	if (!ctx->xo_clk_gpio)
		return 0;

	msleep(1);

	gpiod_set_value_cansleep(ctx->xo_clk_gpio, 1);
	usleep_range(100, 200);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn6855_xo_clk_assert = {
	.name = "xo-clk-assert",
	.enable = pwrseq_qcom_wcn6855_clk_assert,
};

static const struct pwrseq_unit_data *pwrseq_qcom_wcn6855_unit_deps[] = {
	&pwrseq_qcom_wcn_vregs_unit_data,
	&pwrseq_qcom_wcn_clk_unit_data,
	&pwrseq_qcom_wcn6855_xo_clk_assert,
	NULL
};

static int pwrseq_qcom_wcn_bt_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);
	int wlan_state;

	/*
	 * Some WCN platforms require a fresh reset interval after the shared
	 * regulators have been voted.  This callback runs after its regulator
	 * dependencies, so hold BT_EN low here rather than relying on the value
	 * selected when the GPIO was first acquired at probe time.
	 */
	if (ctx->pdata->bt_reset_delay_ms) {
		gpiod_set_value_cansleep(ctx->bt_gpio, 0);
		dev_dbg(ctx->dev,
			"Holding BT_EN low for %u ms after regulator enable\n",
			 ctx->pdata->bt_reset_delay_ms);
		msleep(ctx->pdata->bt_reset_delay_ms);
	}

	/*
	 * When WLAN_EN is low, allow the shared AON output to fully discharge
	 * before releasing BT_EN.  WLAN may already be owned by another consumer,
	 * so observe its current level without changing it.
	 */
	if (ctx->pdata->bt_wlan_off_delay_ms && ctx->wlan_gpio) {
		wlan_state = gpiod_get_value_cansleep(ctx->wlan_gpio);
		if (wlan_state < 0) {
			dev_warn(ctx->dev,
				 "Failed to read WLAN_EN before BT_EN release: %d\n",
				 wlan_state);
		} else if (!wlan_state) {
			dev_dbg(ctx->dev,
				"Observed WLAN_EN=0 before BT_EN release\n");
			dev_dbg(ctx->dev,
				"Waiting %u ms for WLAN-off AON discharge\n",
				 ctx->pdata->bt_wlan_off_delay_ms);
			msleep(ctx->pdata->bt_wlan_off_delay_ms);
		} else {
			dev_dbg(ctx->dev,
				"Observed WLAN_EN=1 before BT_EN release\n");
		}
	}

	pwrseq_qcom_wcn_ensure_gpio_delay(ctx);
	gpiod_set_value_cansleep(ctx->bt_gpio, 1);
	if (ctx->pdata->bt_reset_delay_ms)
		dev_dbg(ctx->dev, "Released BT_EN after reset sequence\n");
	ctx->last_gpio_enable_jf = jiffies;

	return 0;
}

static int pwrseq_qcom_wcn_bt_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	gpiod_set_value_cansleep(ctx->bt_gpio, 0);
	if (ctx->pdata->bt_power_off_delay_ms) {
		dev_dbg(ctx->dev,
			"Holding BT_EN low for %u ms before regulator disable\n",
			 ctx->pdata->bt_power_off_delay_ms);
		msleep(ctx->pdata->bt_power_off_delay_ms);
	}

	return 0;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_bt_unit_data = {
	.name = "bluetooth-enable",
	.deps = pwrseq_qcom_wcn_unit_deps,
	.enable = pwrseq_qcom_wcn_bt_enable,
	.disable = pwrseq_qcom_wcn_bt_disable,
};

static const struct pwrseq_unit_data pwrseq_qcom_wcn6855_bt_unit_data = {
	.name = "bluetooth-enable",
	.deps = pwrseq_qcom_wcn6855_unit_deps,
	.enable = pwrseq_qcom_wcn_bt_enable,
	.disable = pwrseq_qcom_wcn_bt_disable,
};

static int pwrseq_qcom_wcn_wlan_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	pwrseq_qcom_wcn_ensure_gpio_delay(ctx);
	gpiod_set_value_cansleep(ctx->wlan_gpio, 1);
	ctx->last_gpio_enable_jf = jiffies;

	return 0;
}

static int pwrseq_qcom_wcn_wlan_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	gpiod_set_value_cansleep(ctx->wlan_gpio, 0);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_wlan_unit_data = {
	.name = "wlan-enable",
	.deps = pwrseq_qcom_wcn_unit_deps,
	.enable = pwrseq_qcom_wcn_wlan_enable,
	.disable = pwrseq_qcom_wcn_wlan_disable,
};

static const struct pwrseq_unit_data pwrseq_qcom_wcn6855_wlan_unit_data = {
	.name = "wlan-enable",
	.deps = pwrseq_qcom_wcn6855_unit_deps,
	.enable = pwrseq_qcom_wcn_wlan_enable,
	.disable = pwrseq_qcom_wcn_wlan_disable,
};

static int pwrseq_qcom_wcn_pwup_delay(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	if (ctx->pdata->pwup_delay_ms)
		msleep(ctx->pdata->pwup_delay_ms);

	return 0;
}

static int pwrseq_qcom_wcn6855_xo_clk_deassert(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	if (ctx->xo_clk_gpio) {
		usleep_range(2000, 5000);
		gpiod_set_value_cansleep(ctx->xo_clk_gpio, 0);
	}

	return pwrseq_qcom_wcn_pwup_delay(pwrseq);
}

static const struct pwrseq_target_data pwrseq_qcom_wcn_bt_target_data = {
	.name = "bluetooth",
	.unit = &pwrseq_qcom_wcn_bt_unit_data,
	.post_enable = pwrseq_qcom_wcn_pwup_delay,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn_wlan_target_data = {
	.name = "wlan",
	.unit = &pwrseq_qcom_wcn_wlan_unit_data,
	.post_enable = pwrseq_qcom_wcn_pwup_delay,
};

/* There are no separate BT and WLAN enablement pins */
static const struct pwrseq_target_data pwrseq_qcom_wcn3990_bt_target_data = {
	.name = "bluetooth",
	.unit = &pwrseq_qcom_wcn3990_unit_data,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn3990_wlan_target_data = {
	.name = "wlan",
	.unit = &pwrseq_qcom_wcn3990_unit_data,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn6855_bt_target_data = {
	.name = "bluetooth",
	.unit = &pwrseq_qcom_wcn6855_bt_unit_data,
	.post_enable = pwrseq_qcom_wcn6855_xo_clk_deassert,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn6855_wlan_target_data = {
	.name = "wlan",
	.unit = &pwrseq_qcom_wcn6855_wlan_unit_data,
	.post_enable = pwrseq_qcom_wcn6855_xo_clk_deassert,
};

static const struct pwrseq_target_data *pwrseq_qcom_wcn_targets[] = {
	&pwrseq_qcom_wcn_bt_target_data,
	&pwrseq_qcom_wcn_wlan_target_data,
	NULL
};

static const struct pwrseq_target_data *pwrseq_qcom_wcn3990_targets[] = {
	&pwrseq_qcom_wcn3990_bt_target_data,
	&pwrseq_qcom_wcn3990_wlan_target_data,
	NULL
};

static const struct pwrseq_target_data *pwrseq_qcom_wcn6855_targets[] = {
	&pwrseq_qcom_wcn6855_bt_target_data,
	&pwrseq_qcom_wcn6855_wlan_target_data,
	NULL
};

static const char *const pwrseq_qca6390_vregs[] = {
	"vddio",
	"vddaon",
	"vddpmu",
	"vddrfa0p95",
	"vddrfa1p3",
	"vddrfa1p9",
	"vddpcie1p3",
	"vddpcie1p9",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_qca6390_of_data = {
	.vregs = pwrseq_qca6390_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_qca6390_vregs),
	.pwup_delay_ms = 60,
	.gpio_enable_delay_ms = 100,
	.targets = pwrseq_qcom_wcn_targets,
};

static const char *const pwrseq_wcn3990_vregs[] = {
	/* vddio is handled separately */
	"vddxo",
	"vddrf",
	"vddch0",
	"vddch1",
};

static int pwrseq_qcom_wcn3990_match(struct pwrseq_device *pwrseq,
				     struct device *dev);

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn3990_of_data = {
	.vregs = pwrseq_wcn3990_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn3990_vregs),
	.pwup_delay_ms = 50,
	.targets = pwrseq_qcom_wcn3990_targets,
	.has_vddio = true,
	.match = pwrseq_qcom_wcn3990_match,
};

static const char *const pwrseq_wcn6750_vregs[] = {
	"vddaon",
	"vddasd",
	"vddpmu",
	"vddrfa0p8",
	"vddrfa1p2",
	"vddrfa1p7",
	"vddrfa2p2",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn6750_of_data = {
	.vregs = pwrseq_wcn6750_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn6750_vregs),
	.pwup_delay_ms = 50,
	.gpio_enable_delay_ms = 5,
	.targets = pwrseq_qcom_wcn_targets,
};

static const char *const pwrseq_wcn6855_vregs[] = {
	"vddio",
	"vddaon",
	"vddpmu",
	"vddpmumx",
	"vddpmucx",
	"vddrfa0p95",
	"vddrfa1p3",
	"vddrfa1p9",
	"vddpcie1p3",
	"vddpcie1p9",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn6855_of_data = {
	.vregs = pwrseq_wcn6855_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn6855_vregs),
	.pwup_delay_ms = 50,
	.gpio_enable_delay_ms = 5,
	.targets = pwrseq_qcom_wcn6855_targets,
};

static const char *const pwrseq_wcn7850_vregs[] = {
	"vdd",
	"vddio",
	"vddio1p2",
	"vddaon",
	"vdddig",
	"vddrfa1p2",
	"vddrfa1p8",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn7850_of_data = {
	.vregs = pwrseq_wcn7850_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn7850_vregs),
	.pwup_delay_ms = 50,
	.targets = pwrseq_qcom_wcn_targets,
};

/*
 * Lenovo's downstream Peach CNSS node votes these exact external PMIC
 * ranges before WLAN_EN.  Merely enabling the variable RPMh regulators leaves
 * S4D/S5F/S7I at their lowest constraints (500/500/1224 mV), which is below
 * the WCN7861 AON, digital and RFA requirements.  These votes are independent
 * of the firmware's stock-compatible, non-fatal TME result.
 */
static const struct pwrseq_qcom_wcn_vreg_cfg pwrseq_wcn7861_vreg_cfgs[] = {
	/* vdd (same S5F rail as vdddig) */
	{ 876000, 1000000, 0 },
	/* vddio */
	{ 1800000, 1800000, 30000 },
	/* vddio1p2 */
	{ 1200000, 1200000, 30000 },
	/* vddaon (S4D) */
	{ 876000, 1036000, 0 },
	/* vdddig (S5F) */
	{ 876000, 1000000, 0 },
	/* vddrfa1p2 (S7I) */
	{ 1312000, 1340000, 0 },
	/* vddrfa1p8 (S3G) */
	{ 1860000, 2000000, 0 },
};

static_assert(ARRAY_SIZE(pwrseq_wcn7861_vreg_cfgs) ==
	      ARRAY_SIZE(pwrseq_wcn7850_vregs));

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn7861_of_data = {
	.vregs = pwrseq_wcn7850_vregs,
	.vreg_cfgs = pwrseq_wcn7861_vreg_cfgs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn7850_vregs),
	/* Match the Peach btpower reset/AON sequence around BT_EN. */
	.bt_reset_delay_ms = 50,
	.bt_wlan_off_delay_ms = 100,
	.bt_power_off_delay_ms = 100,
	.pwup_delay_ms = 50,
	.targets = pwrseq_qcom_wcn_targets,
};

static int pwrseq_qcom_wcn_match_regulator(struct pwrseq_device *pwrseq,
					   struct device *dev,
					   const char *name)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);
	struct device_node *dev_node = dev->of_node;

	/*
	 * The PMU supplies power to the Bluetooth and WLAN modules. both
	 * consume the PMU AON output so check the presence of the
	 * 'vddaon-supply' property and whether it leads us to the right
	 * device.
	 */
	if (!of_property_present(dev_node, name))
		return PWRSEQ_NO_MATCH;

	struct device_node *reg_node __free(device_node) =
			of_parse_phandle(dev_node, name, 0);
	if (!reg_node)
		return PWRSEQ_NO_MATCH;

	/*
	 * `reg_node` is the PMU AON regulator, its parent is the `regulators`
	 * node and finally its grandparent is the PMU device node that we're
	 * looking for.
	 */
	if (!reg_node->parent || !reg_node->parent->parent ||
	    reg_node->parent->parent != ctx->of_node)
		return PWRSEQ_NO_MATCH;

	return PWRSEQ_MATCH_OK;
}

static int pwrseq_qcom_wcn_match(struct pwrseq_device *pwrseq,
				 struct device *dev)
{
	return pwrseq_qcom_wcn_match_regulator(pwrseq, dev, "vddaon-supply");
}

static int pwrseq_qcom_wcn3990_match(struct pwrseq_device *pwrseq,
				     struct device *dev)
{
	int ret;

	/* BT device */
	ret = pwrseq_qcom_wcn_match_regulator(pwrseq, dev, "vddio-supply");
	if (ret == PWRSEQ_MATCH_OK)
		return ret;

	/* WiFi device match */
	return pwrseq_qcom_wcn_match_regulator(pwrseq, dev, "vdd-1.8-xo-supply");
}

struct pwrseq_qcom_wcn_pdc_vote {
	u32 wake_volt;
	u32 sleep_volt;
	bool wake_valid;
	bool sleep_valid;
};

static struct platform_driver pwrseq_qcom_wcn_driver;

static int pwrseq_qcom_wcn_adjust_voltage(u32 voltage, u32 ir_drop,
					  u32 *adjusted)
{
	u32 rounded;

	if (check_add_overflow(voltage, 3U, &rounded))
		return -ERANGE;

	rounded &= ~3U;
	if (rounded < 16U)
		return -ERANGE;

	if (check_add_overflow(rounded - 16U, ir_drop, adjusted))
		return -ERANGE;

	return 0;
}

static int
pwrseq_qcom_wcn_vote_index(struct pwrseq_qcom_wcn_ctx *ctx, const char *rail)
{
	size_t i;

	for (i = 0; i < ctx->num_vreg_pdc_map; i++) {
		if (!strcmp(ctx->vreg_pdc_map[i].from, rail))
			return i;
	}

	return -ENOENT;
}

static int
pwrseq_qcom_wcn_send_voltage(struct pwrseq_qcom_wcn_ctx *ctx,
			     const char *rail, const char *pdc,
			     const char *sequence, u32 voltage)
{
	char msg[PWRSEQ_QCOM_WCN_QMP_MSG_LEN];
	int len;

	len = snprintf(msg, sizeof(msg),
		       "{class: wlan_pdc, ss: %s, res: %s.v, %s: %u}",
		       pdc, rail, sequence, voltage);
	if (len >= sizeof(msg))
		return -E2BIG;

	return pwrseq_qcom_wcn_qmp_send(ctx, msg);
}

static int
pwrseq_qcom_wcn_apply_ol_cpr(struct pwrseq_qcom_wcn_ctx *ctx,
			     const struct pwrseq_qcom_wcn_pmu_param *params,
			     size_t num_params)
{
	struct pwrseq_qcom_wcn_pdc_vote *votes;
	char pin[PWRSEQ_QCOM_WCN_PMU_PIN_NAME_LEN + 1];
	const char *rail;
	const char *pdc;
	u32 wake_volt;
	u32 sleep_volt;
	size_t valid = 0;
	size_t mapped = 0;
	size_t unknown = 0;
	int index;
	int ret;
	size_t i;

	votes = kcalloc(ctx->num_vreg_pdc_map, sizeof(*votes), GFP_KERNEL);
	if (!votes)
		return -ENOMEM;

	for (i = 0; i < num_params; i++) {
		memcpy(pin, params[i].pin_name,
		       PWRSEQ_QCOM_WCN_PMU_PIN_NAME_LEN);
		pin[PWRSEQ_QCOM_WCN_PMU_PIN_NAME_LEN] = '\0';

		if (!pin[0]) {
			dev_err(ctx->dev, "OL-CPR[%zu] has an empty PMU pin name\n", i);
			ret = -EINVAL;
			goto out_free;
		}

		if (!params[i].wake_volt_valid &&
		    !params[i].sleep_volt_valid) {
			dev_dbg(ctx->dev,
				"OL-CPR[%zu] pin=%s has no valid voltage\n",
				 i, pin);
			continue;
		}
		valid++;

		rail = pwrseq_qcom_wcn_map_find(ctx->pmu_vreg_map,
						ctx->num_pmu_vreg_map, pin);
		if (!rail) {
			dev_warn(ctx->dev,
				 "OL-CPR[%zu] pin=%s has no platform mapping (wake_valid=%u wake=%u sleep_valid=%u sleep=%u)\n",
				 i, pin, params[i].wake_volt_valid,
				 params[i].wake_volt, params[i].sleep_volt_valid,
				 params[i].sleep_volt);
			unknown++;
			continue;
		}

		index = pwrseq_qcom_wcn_vote_index(ctx, rail);
		if (index < 0) {
			ret = index;
			goto out_free;
		}

		pdc = ctx->vreg_pdc_map[index].to;
		wake_volt = 0;
		sleep_volt = 0;

		if (params[i].wake_volt_valid) {
			ret = pwrseq_qcom_wcn_adjust_voltage(params[i].wake_volt,
							     30U, &wake_volt);
			if (ret) {
				dev_err(ctx->dev,
					"OL-CPR[%zu] invalid wake voltage %u for %s: %d\n",
					i, params[i].wake_volt, pin, ret);
				goto out_free;
			}

			votes[index].wake_volt = max(votes[index].wake_volt,
						     wake_volt);
			votes[index].wake_valid = true;
		}

		if (params[i].sleep_volt_valid) {
			ret = pwrseq_qcom_wcn_adjust_voltage(params[i].sleep_volt,
							     10U, &sleep_volt);
			if (ret) {
				dev_err(ctx->dev,
					"OL-CPR[%zu] invalid sleep voltage %u for %s: %d\n",
					i, params[i].sleep_volt, pin, ret);
				goto out_free;
			}

			votes[index].sleep_volt = max(votes[index].sleep_volt,
						      sleep_volt);
			votes[index].sleep_valid = true;
		}

		dev_dbg(ctx->dev,
			"OL-CPR[%zu] pin=%s rail=%s ss=%s wake_valid=%u wake=%u upval=%u sleep_valid=%u sleep=%u dwnval=%u\n",
			 i, pin, rail, pdc, params[i].wake_volt_valid,
			 params[i].wake_volt, wake_volt,
			 params[i].sleep_volt_valid, params[i].sleep_volt,
			 sleep_volt);
		mapped++;
	}

	if (!valid) {
		dev_dbg(ctx->dev, "OL-CPR contains no requested voltage votes\n");
		ret = 0;
		goto out_free;
	}

	if (!mapped) {
		ret = -ENODATA;
		goto out_free;
	}

	for (i = 0; i < ctx->num_vreg_pdc_map; i++) {
		rail = ctx->vreg_pdc_map[i].from;
		pdc = ctx->vreg_pdc_map[i].to;

		dev_dbg(ctx->dev,
			"OL-CPR aggregate rail=%s ss=%s up_valid=%u upval=%u down_valid=%u dwnval=%u\n",
			 rail, pdc, votes[i].wake_valid, votes[i].wake_volt,
			 votes[i].sleep_valid, votes[i].sleep_volt);

		if (votes[i].wake_valid && votes[i].wake_volt) {
			ret = pwrseq_qcom_wcn_send_voltage(ctx, rail, pdc,
							   "upval",
							    votes[i].wake_volt);
			if (ret)
				goto out_free;
		}

		if (votes[i].sleep_valid && votes[i].sleep_volt) {
			ret = pwrseq_qcom_wcn_send_voltage(ctx, rail, pdc,
							   "dwnval",
							    votes[i].sleep_volt);
			if (ret)
				goto out_free;
		}
	}

	dev_dbg(ctx->dev,
		"Applied OL-CPR votes from %zu mapped PMU entries (%zu unmapped)\n",
		 mapped, unknown);
	ret = 0;

out_free:
	kfree(votes);
	return ret;
}

int pwrseq_qcom_wcn_set_ol_cpr(struct device *consumer,
			       const struct pwrseq_qcom_wcn_pmu_param *params,
			       size_t num_params)
{
	struct pwrseq_qcom_wcn_ctx *ctx;
	struct pwrseq_desc *desc;
	struct device *provider;
	int ret;

	if (!consumer || !params || !num_params ||
	    num_params > PWRSEQ_QCOM_WCN_PMU_MAX_PARAMS)
		return -EINVAL;

	desc = pwrseq_get(consumer, "wlan");
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	provider = pwrseq_to_device(desc);
	if (!provider || !provider->parent ||
	    provider->parent->driver != &pwrseq_qcom_wcn_driver.driver) {
		ret = -EOPNOTSUPP;
		goto out_put;
	}

	ctx = dev_get_drvdata(provider);
	if (!ctx || ctx->pdata != &pwrseq_wcn7861_of_data || !ctx->qmp ||
	    !ctx->num_vreg_pdc_map ||
	    !ctx->num_pmu_vreg_map) {
		ret = -ENODEV;
		goto out_put;
	}

	/* Reapplying the same PDC TCS values during recovery is idempotent. */
	mutex_lock(&ctx->pdc_lock);
	ret = pwrseq_qcom_wcn_apply_ol_cpr(ctx, params, num_params);
	mutex_unlock(&ctx->pdc_lock);

out_put:
	pwrseq_put(desc);
	return ret;
}
EXPORT_SYMBOL_GPL(pwrseq_qcom_wcn_set_ol_cpr);

static int pwrseq_qcom_wcn_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwrseq_qcom_wcn_ctx *ctx;
	struct pwrseq_config config;
	int i, ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->of_node = dev->of_node;
	mutex_init(&ctx->pdc_lock);

	ctx->pdata = device_get_match_data(dev);
	if (!ctx->pdata)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to obtain platform data\n");

	ret = pwrseq_qcom_wcn_parse_map(ctx, "qcom,vreg-pdc-map",
					&ctx->vreg_pdc_map,
				       &ctx->num_vreg_pdc_map);
	if (ret)
		return ret;

	ret = pwrseq_qcom_wcn_parse_map(ctx, "qcom,pmu-vreg-map",
					&ctx->pmu_vreg_map,
				       &ctx->num_pmu_vreg_map);
	if (ret)
		return ret;

	if (!!ctx->num_vreg_pdc_map != !!ctx->num_pmu_vreg_map)
		return dev_err_probe(dev, -EINVAL,
				     "Both PDC and PMU mapping tables are required\n");

	ret = pwrseq_qcom_wcn_validate_maps(ctx);
	if (ret)
		return ret;

	/*
	 * Newer WCN devices require the host AOSS to configure their PDC
	 * resources before any external supply or WLAN_EN vote is made.
	 */
	ret = pwrseq_qcom_wcn_pdc_init(ctx);
	if (ret)
		return ret;

	ctx->regs = devm_kcalloc(dev, ctx->pdata->num_vregs,
				 sizeof(*ctx->regs), GFP_KERNEL);
	if (!ctx->regs)
		return -ENOMEM;

	for (i = 0; i < ctx->pdata->num_vregs; i++)
		ctx->regs[i].supply = ctx->pdata->vregs[i];

	ret = devm_regulator_bulk_get(dev, ctx->pdata->num_vregs, ctx->regs);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to get all regulators\n");

	if (ctx->pdata->has_vddio) {
		ctx->vddio = devm_regulator_get(dev, "vddio");
		if (IS_ERR(ctx->vddio))
			return dev_err_probe(dev, PTR_ERR(ctx->vddio), "Failed to get VDDIO\n");
	}

	dev_info(dev, "piano-dbg: PMU requesting bt-enable GPIO\n");
	ctx->bt_gpio = devm_gpiod_get_optional(dev, "bt-enable", GPIOD_OUT_LOW);
	dev_info(dev, "piano-dbg: PMU bt-enable result=%ld present=%d\n",
		 IS_ERR(ctx->bt_gpio) ? PTR_ERR(ctx->bt_gpio) : 0L,
		 !IS_ERR_OR_NULL(ctx->bt_gpio));
	if (IS_ERR(ctx->bt_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->bt_gpio),
				     "Failed to get the Bluetooth enable GPIO\n");

	/*
	 * FIXME: This should actually be GPIOD_OUT_LOW, but doing so would
	 * cause the WLAN power to be toggled, resulting in PCIe link down.
	 * Since the PCIe controller driver is not handling link down currently,
	 * the device becomes unusable. So we need to keep this workaround until
	 * the link down handling is implemented in the controller driver.
	 */
	ctx->wlan_gpio = devm_gpiod_get_optional(dev, "wlan-enable",
						 GPIOD_ASIS);
	if (IS_ERR(ctx->wlan_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->wlan_gpio),
				     "Failed to get the WLAN enable GPIO\n");
	dev_info(dev, "piano-dbg: PMU wlan-enable GPIO acquired\n");

	ctx->xo_clk_gpio = devm_gpiod_get_optional(dev, "xo-clk",
						   GPIOD_OUT_LOW);
	if (IS_ERR(ctx->xo_clk_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->xo_clk_gpio),
				     "Failed to get the XO_CLK GPIO\n");

	/*
	 * Set direction to output but keep the current value in order to not
	 * disable the WLAN module accidentally if it's already powered on.
	 */
	gpiod_direction_output(ctx->wlan_gpio,
			       gpiod_get_value_cansleep(ctx->wlan_gpio));

	ctx->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(ctx->clk))
		return dev_err_probe(dev, PTR_ERR(ctx->clk),
				     "Failed to get the reference clock\n");

	memset(&config, 0, sizeof(config));

	config.parent = dev;
	config.owner = THIS_MODULE;
	config.drvdata = ctx;
	config.match = ctx->pdata->match ? : pwrseq_qcom_wcn_match;
	config.targets = ctx->pdata->targets;

	dev_info(dev, "piano-dbg: PMU registering power sequencer\n");
	ctx->pwrseq = devm_pwrseq_device_register(dev, &config);
	if (IS_ERR(ctx->pwrseq))
		return dev_err_probe(dev, PTR_ERR(ctx->pwrseq),
				     "Failed to register the power sequencer\n");

	dev_info(dev, "piano-dbg: PMU probe complete\n");
	return 0;
}

static const struct of_device_id pwrseq_qcom_wcn_of_match[] = {
	{
		.compatible = "qcom,wcn3950-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3988-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3990-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3991-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3998-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,qca6390-pmu",
		.data = &pwrseq_qca6390_of_data,
	},
	{
		.compatible = "qcom,wcn6855-pmu",
		.data = &pwrseq_wcn6855_of_data,
	},
	{
		.compatible = "qcom,wcn7850-pmu",
		.data = &pwrseq_wcn7850_of_data,
	},
	{
		.compatible = "qcom,wcn7861-pmu",
		.data = &pwrseq_wcn7861_of_data,
	},
	{
		.compatible = "qcom,wcn6750-pmu",
		.data = &pwrseq_wcn6750_of_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pwrseq_qcom_wcn_of_match);

static struct platform_driver pwrseq_qcom_wcn_driver = {
	.driver = {
		.name = "pwrseq-qcom_wcn",
		.of_match_table = pwrseq_qcom_wcn_of_match,
	},
	.probe = pwrseq_qcom_wcn_probe,
};
module_platform_driver(pwrseq_qcom_wcn_driver);

MODULE_AUTHOR("Bartosz Golaszewski <bartosz.golaszewski@linaro.org>");
MODULE_DESCRIPTION("Qualcomm WCN PMU power sequencing driver");
MODULE_LICENSE("GPL");
