/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __POWER_SEQUENCING_QCOM_WCN_H__
#define __POWER_SEQUENCING_QCOM_WCN_H__

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

#define PWRSEQ_QCOM_WCN_PMU_PIN_NAME_LEN	32
#define PWRSEQ_QCOM_WCN_PMU_MAX_PARAMS		16

struct pwrseq_qcom_wcn_pmu_param {
	u8 pin_name[PWRSEQ_QCOM_WCN_PMU_PIN_NAME_LEN];
	u32 wake_volt_valid;
	u32 wake_volt;
	u32 sleep_volt_valid;
	u32 sleep_volt;
};

#if IS_REACHABLE(CONFIG_POWER_SEQUENCING_QCOM_WCN)
int pwrseq_qcom_wcn_set_ol_cpr(struct device *consumer,
			       const struct pwrseq_qcom_wcn_pmu_param *params,
			       size_t num_params);
#else
static inline int
pwrseq_qcom_wcn_set_ol_cpr(struct device *consumer,
			   const struct pwrseq_qcom_wcn_pmu_param *params,
			   size_t num_params)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* __POWER_SEQUENCING_QCOM_WCN_H__ */
