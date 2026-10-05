/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-isp-debug.h
 *
 * Bring-up aid for the SM8750 TFE pixel path.
 */
#ifndef QC_MSM_CAMSS_ISP_DEBUG_H
#define QC_MSM_CAMSS_ISP_DEBUG_H

#include <linux/types.h>

enum camss_isp_block {
	CAMSS_ISP_VFE,
	CAMSS_ISP_CSID,
	CAMSS_ISP_BLOCKS
};

void camss_isp_apply_script(enum camss_isp_block which, void __iomem *base);
void camss_isp_set_live(enum camss_isp_block which, void __iomem *base);
void camss_isp_debug_init(void);
void camss_isp_debug_exit(void);

#endif /* QC_MSM_CAMSS_ISP_DEBUG_H */
