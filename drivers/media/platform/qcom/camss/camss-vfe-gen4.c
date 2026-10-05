// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-gen4.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module gen4
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-isp-debug.h"
#include "camss-vfe.h"

#define IS_VFE_980(vfe)		((vfe)->camss->res->version == CAMSS_8750)

#define BUS_REG_BASE_980	(vfe_is_lite(vfe) ? 0x200 : 0x800)
#define BUS_REG_BASE_1080	(vfe_is_lite(vfe) ? 0x800 : 0x1000)
#define BUS_REG_BASE \
	    (IS_VFE_980(vfe) ? BUS_REG_BASE_980 : BUS_REG_BASE_1080)

#define VFE_BUS_WM_CGC_OVERRIDE			(BUS_REG_BASE + 0x08)
#define		WM_CGC_OVERRIDE_ALL			(0x7FFFFFF)

#define VFE_BUS_WM_TEST_BUS_CTRL		(BUS_REG_BASE + 0x128)

#define VFE_BUS_WM_CFG(n)			(BUS_REG_BASE + 0x500 + (n) * 0x100)
#define		WM_CFG_EN				BIT(0)
#define		WM_VIR_FRM_EN				BIT(1)
#define		WM_CFG_MODE				BIT(16)
#define VFE_BUS_WM_IMAGE_ADDR(n)		(BUS_REG_BASE + 0x504 + (n) * 0x100)
#define VFE_BUS_WM_FRAME_INCR(n)		(BUS_REG_BASE + 0x508 + (n) * 0x100)
#define VFE_BUS_WM_IMAGE_CFG_0(n)		(BUS_REG_BASE + 0x50C + (n) * 0x100)
#define		WM_IMAGE_CFG_0_DEFAULT_WIDTH		(0xFFFF)
#define VFE_BUS_WM_IMAGE_CFG_1(n)		(BUS_REG_BASE + 0x510 + (n) * 0x100)
#define VFE_BUS_WM_IMAGE_CFG_2(n)		(BUS_REG_BASE + 0x514 + (n) * 0x100)
#define		WM_IMAGE_CFG_2_DEFAULT_STRIDE		(0xFFFF)
#define VFE_BUS_WM_PACKER_CFG(n)		(BUS_REG_BASE + 0x518 + (n) * 0x100)

#define VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(n)	(BUS_REG_BASE + 0x530 + (n) * 0x100)
#define VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(n)	(BUS_REG_BASE + 0x534 + (n) * 0x100)

/* VFE lite has no such registers */
#define VFE_BUS_WM_FRAMEDROP_PERIOD(n)		(BUS_REG_BASE + 0x538 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PATTERN(n)		(BUS_REG_BASE + 0x53C + (n) * 0x100)

#define VFE_BUS_WM_MMU_PREFETCH_CFG(n)		(BUS_REG_BASE + 0x560 + (n) * 0x100)
#define VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(n)	(BUS_REG_BASE + 0x564 + (n) * 0x100)
#define VFE_BUS_WM_CTXT_CFG(n)			(BUS_REG_BASE + 0x578 + (n) * 0x100)
#define VFE_BUS_CTXT_SEL			(BUS_REG_BASE + 0x124)

#define		WM_PACKER_PLAIN_8_LSB_MSB_10		3

/*
 * IFE write master client IDs
 *
 * VIDEO_FULL			0
 * VIDEO_DC4_Y			1
 * VIDEO_DC4_C			2
 * VIDEO_DC16_Y			3
 * VIDEO_DC16_C			4
 * DISPLAY_DS2_Y		5
 * DISPLAY_DS2_C		6
 * FD_Y				7
 * FD_C				8
 * RAW_OUT(1080)/IR_OUT(980)	9
 * STATS_AEC_BG			10
 * STATS_AEC_BHIST		11
 * STATS_TINTLESS_BG		12
 * STATS_AWB_BG			13
 * STATS_AWB_BFW		14
 * STATS_AF_BHIST		15
 * STATS_ALSC_BG		16
 * STATS_FLICKER_BAYERRS	17
 * STATS_TMC_BHIST		18
 * PDAF_0			19
 * PDAF_1			20
 * PDAF_2			21
 * PDAF_3			22
 * RDI0				23
 * RDI1				24
 * RDI2				25
 * RDI3				26
 * RDI4				27
 *
 * IFE Lite write master client IDs
 *
 * RDI0			0
 * RDI1			1
 * RDI2			2
 * RDI3			3
 * GAMMA		4
 * STATES_BE		5
 */
#define RDI_WM(n) ((vfe_is_lite(vfe) ? 0x0 : 0x17) + (n))
#define FD_Y_WM		7
#define FD_C_WM		8

/*
 * TFE 980 pixel pipeline (IPP): demux -> ... -> white balance -> BDS2/demosaic
 * -> colour correct -> GTM -> gamma LUT -> colour transform (RGB to YCbCr)
 * -> MN downscaler -> crop/round/clamp -> FD_Y/FD_C write masters.
 *
 * The IQ modules sit in the TFE Bayer clock domain (TFE_n_BAYER clocks).
 * They are not documented; register layouts come from the CamX
 * hardware-setting code and were checked against sensor test patterns.
 * Each module keeps per-hardware-context register banks, selected by its
 * CTXT_SEL register (module base + 0x1f4).  Modules not listed stay disabled
 * (bypassed).
 *
 * The Bayer stage carries 14 bit data (10 bit sensor data << 4), but the
 * stages after the demosaic clamp to 10 bits, so white balance scales the
 * data down by 16.  Its offsets subtract the sensor black level before the
 * gains.  The colour stages see the channels in (G, B, R) order.  The gamma
 * LUT then maps the linear 10 bit data to sRGB, the colour transform
 * (with the colour correction matrix folded in) produces full range BT.601
 * YCbCr and the FD crop/round/clamp blocks round
 * that to 8 bits.  The white balance and digital gains come from the PIX
 * subdev controls; exposure and white balance control is left to userspace.
 */
struct vfe_reg_val {
	u32 reg;
	u32 val;
};

#define TFE_CTXT_SEL_SINGLE		0x400

/*
 * Sensor black level (64 at 10 bits) in Bayer stage units, and the Q10 white
 * balance gain that maps the remaining range to 10 bits: 1023 / 959 / 16
 */
#define TFE_BLACK_LEVEL			(64 << 4)
#define TFE_WB_GAIN_UNITY		68
#define TFE_WB_GAIN_MAX			0x7fff
#define TFE_WB_GAIN_G			0xe068
#define TFE_WB_GAIN_B			0xe06c
#define TFE_WB_GAIN_R			0xe070

static const struct vfe_reg_val vfe_pix_980_defaults[] = {
	/* demux: enabled, plain Bayer */
	{ 0xd060, 0x1 },
	/* white balance: black level offsets (gains: vfe_isp_update()) */
	{ 0xe1f4, TFE_CTXT_SEL_SINGLE },
	{ 0xe074, TFE_BLACK_LEVEL },
	{ 0xe078, TFE_BLACK_LEVEL },
	{ 0xe07c, TFE_BLACK_LEVEL },
	{ 0xe080, 0x0 },
	{ 0xe084, 0x0 },
	{ 0xe088, 0x0 },
	{ 0xe060, 0x1 },
	/* BDS2 + demosaic: demosaic only (BDS2 off below 7008 pixels) */
	{ 0x78f4, 0x1 },	/* ctxt sel */
	{ 0x7794, 0x100 },
	{ 0x78a0, 0x4001 },
	{ 0x78a4, 0x80 },
	{ 0x78a8, 0x80 },
	{ 0x78ac, 0x66 },
	{ 0x78e4, 0x1 },
	{ 0x7760, 0x1 },
	/*
	 * colour space transform, RGB to YCbCr (BT.601 full range), Q10
	 * coefficients in 13 bit two's complement; per output channel: input
	 * coefficients (G | B << 16, R), output offset, input offset, clamps
	 */
	{ 0x6ef4, TFE_CTXT_SEL_SINGLE },
	/* coefficients of all rows: vfe_isp_update() (CCM, saturation) */
	{ 0x6d70, 0x0 },
	{ 0x6d74, 0x0 },
	{ 0x6d78, 0x0 },
	{ 0x6d7c, 0x3ff },
	{ 0x6d88, 0x200 },
	{ 0x6d8c, 0x0 },
	{ 0x6d90, 0x0 },
	{ 0x6d94, 0x3ff },
	{ 0x6da0, 0x200 },
	{ 0x6da4, 0x0 },
	{ 0x6da8, 0x0 },
	{ 0x6dac, 0x3ff },
	{ 0x6d60, 0x1 },
};

static u32 vfe_isp_wb_gain(u32 gain, u32 digital_gain)
{
	u64 val = (u64)TFE_WB_GAIN_UNITY * gain * digital_gain;

	return min_t(u64, DIV_ROUND_CLOSEST_ULL(val, 1024 * 1024),
		     TFE_WB_GAIN_MAX);
}

/*
 * Colour transform rows (Y, Cb, Cr), each at TFE_CST_ROW(i): G | B << 16,
 * then R. BT.601 full range, Q10, in (R, G, B) order.
 */
#define TFE_CST_ROW(i)			(0x6d68 + 0x18 * (i))

static const s16 vfe_pix_cst_bt601[3][3] = {
	{ 306, 601, 117 },	/* Y:  0.299 R, 0.587 G, 0.114 B */
	{ -173, -339, 512 },	/* Cb: -0.169 R, -0.331 G, 0.5 B */
	{ 512, -429, -83 },	/* Cr: 0.5 R, -0.419 G, -0.081 B */
};

static u32 vfe_pix_cst_coef(s32 coef)
{
	return clamp(coef, -4096, 4095) & GENMASK(12, 0);
}

/*
 * The CST runs after the gamma LUT; the colour correction matrix is folded
 * into it (BT.601 x CCM), an approximation of a linear-light correction.
 * Saturation scales the chroma rows.
 */
static void vfe_pix_cst_config(struct vfe_device *vfe,
			       const struct vfe_isp_params *isp)
{
	static const s32 identity[9] = { 1024, 0, 0, 0, 1024, 0, 0, 0, 1024 };
	const s32 *ccm = memchr_inv(isp->ccm, 0, sizeof(isp->ccm)) ?
			 isp->ccm : identity;
	unsigned int i, j, k;

	for (i = 0; i < 3; i++) {
		s32 c[3];

		for (k = 0; k < 3; k++) {
			s64 v = 0;

			for (j = 0; j < 3; j++)
				v += (s64)vfe_pix_cst_bt601[i][j] * ccm[3 * j + k];
			if (i)
				v *= isp->saturation;
			else
				v *= 256;
			c[k] = div_s64(v + (v < 0 ? -1 : 1) * (1 << 17), 1 << 18);
		}
		/* (R, G, B) -> (G, B, R) inputs */
		writel(vfe_pix_cst_coef(c[2]) << 16 | vfe_pix_cst_coef(c[1]),
		       vfe->base + TFE_CST_ROW(i));
		writel(vfe_pix_cst_coef(c[0]), vfe->base + TFE_CST_ROW(i) + 4);
	}
}

static void vfe_pix_glut_load(struct vfe_device *vfe, u32 contrast);

/* settings behind the PIX line controls */
static void vfe_isp_update(struct vfe_device *vfe, struct vfe_line *line)
{
	struct vfe_isp_params *isp = &line->isp;

	writel(vfe_isp_wb_gain(1024, isp->digital_gain),
	       vfe->base + TFE_WB_GAIN_G);
	writel(vfe_isp_wb_gain(isp->blue_gain, isp->digital_gain),
	       vfe->base + TFE_WB_GAIN_B);
	writel(vfe_isp_wb_gain(isp->red_gain, isp->digital_gain),
	       vfe->base + TFE_WB_GAIN_R);

	vfe_pix_cst_config(vfe, isp);

	if (isp->lut_contrast != (s32)isp->contrast) {
		vfe_pix_glut_load(vfe, isp->contrast);
		isp->lut_contrast = isp->contrast;
	}
}

/*
 * Gamma LUT: 64 linearly interpolated segments per colour channel over the
 * 10 bit input, written through the module DMI port.  Each entry holds the
 * segment start value and its delta to the next one.
 */
#define TFE_GLUT_DMI_ADDR		0x6b08
#define TFE_GLUT_DMI_SEL		0x6b0c
#define TFE_GLUT_DMI_DATA		0x6b14
#define TFE_GLUT_BANK_SEL0		0x6b58
#define TFE_GLUT_BANK_SEL1		0x6b5c
#define TFE_GLUT_EN			0x6b60
#define TFE_GLUT_CTXT_SEL		0x6cf4
#define TFE_GLUT_ENTRIES		64

/* sRGB transfer function, sampled at the segment boundaries */
static const u16 vfe_pix_980_srgb[TFE_GLUT_ENTRIES + 1] = {
	   0,  135,  199,  245,  284,  317,  346,  373,  398,  421,
	 442,  462,  481,  499,  517,  534,  550,  565,  580,  595,
	 609,  622,  636,  649,  661,  674,  686,  697,  709,  720,
	 731,  742,  753,  763,  773,  783,  793,  803,  813,  822,
	 831,  841,  850,  859,  867,  876,  885,  893,  901,  910,
	 918,  926,  934,  942,  950,  957,  965,  973,  980,  987,
	 995, 1002, 1009, 1016, 1023,
};

/*
 * Tone curve: the sRGB curve blended towards a smoothstep S curve of
 * itself, by contrast / 256, for deeper shadows and brighter highlights.
 */
static u32 vfe_pix_tone(unsigned int i, u32 contrast)
{
	s64 x = vfe_pix_980_srgb[i];
	s64 sc = div_s64(x * x * (3 * 1023 - 2 * x), 1023 * 1023);

	return x + div_s64((sc - x) * contrast, 256);
}

static void vfe_pix_glut_load(struct vfe_device *vfe, u32 contrast)
{
	u32 pts[TFE_GLUT_ENTRIES + 1];
	unsigned int sel, i;

	for (i = 0; i <= TFE_GLUT_ENTRIES; i++)
		pts[i] = vfe_pix_tone(i, contrast);

	writel(TFE_CTXT_SEL_SINGLE, vfe->base + TFE_GLUT_CTXT_SEL);

	/* one LUT per colour channel: 1 = G, 2 = B, 3 = R */
	for (sel = 1; sel <= 3; sel++) {
		writel(sel, vfe->base + TFE_GLUT_DMI_SEL);
		writel(0, vfe->base + TFE_GLUT_DMI_ADDR);
		for (i = 0; i < TFE_GLUT_ENTRIES; i++)
			writel(pts[i] | ((pts[i + 1] - pts[i]) & 0x3ff) << 10,
			       vfe->base + TFE_GLUT_DMI_DATA);
	}
	writel(0, vfe->base + TFE_GLUT_DMI_SEL);
	writel(0, vfe->base + TFE_GLUT_DMI_ADDR);

	writel(0, vfe->base + TFE_GLUT_BANK_SEL0);
	writel(0, vfe->base + TFE_GLUT_BANK_SEL1);
	writel(1, vfe->base + TFE_GLUT_EN);
}

/*
 * MN downscalers (Y and C): the sink compose rectangle.  The C scaler also
 * does the 4:2:0 subsampling, at twice the luma ratio.  Phase steps are
 * input/output ratios in Q21; the top two bits pick the filter range.
 */
#define TFE_MNDS_Y			0x6f00
#define TFE_MNDS_C			0x7100
#define TFE_MNDS_EN(b)			((b) + 0x60)
#define TFE_MNDS_CFG(b)			((b) + 0x64)
#define		TFE_MNDS_CFG_H_EN		BIT(9)
#define		TFE_MNDS_CFG_V_EN		BIT(10)
#define TFE_MNDS_IN_SIZE(b)		((b) + 0x68)
#define TFE_MNDS_H_STEP(b)		((b) + 0x6c)
#define TFE_MNDS_H_PHASE(b)		((b) + 0x70)
#define TFE_MNDS_V_STEP(b)		((b) + 0x74)
#define TFE_MNDS_V_PHASE(b)		((b) + 0x78)
#define TFE_MNDS_PAD0(b)		((b) + 0x7c)
#define TFE_MNDS_PAD1(b)		((b) + 0x80)
#define TFE_MNDS_ROUND(b)		((b) + 0x84)
#define TFE_MNDS_STEP_SHIFT		21

static u32 vfe_pix_mnds_step(u32 in, u32 out)
{
	u64 step = div_u64((u64)in << TFE_MNDS_STEP_SHIFT, out);
	u32 range;

	if (step <= 16ULL << TFE_MNDS_STEP_SHIFT)
		range = 3;
	else if (step <= 32ULL << TFE_MNDS_STEP_SHIFT)
		range = 2;
	else if (step <= 64ULL << TFE_MNDS_STEP_SHIFT)
		range = 1;
	else
		range = 0;

	return range << 30 | (step & GENMASK(28, 0));
}

static void vfe_pix_mnds_config(struct vfe_device *vfe, u32 base, u32 in_w,
				u32 in_h, u32 out_w, u32 out_h, bool enable)
{
	void __iomem *b = vfe->base;

	if (!enable) {
		writel(0, b + TFE_MNDS_EN(base));
		return;
	}

	writel(TFE_MNDS_CFG_H_EN | TFE_MNDS_CFG_V_EN, b + TFE_MNDS_CFG(base));
	writel((in_w - 1) << 16 | (in_h - 1), b + TFE_MNDS_IN_SIZE(base));
	writel(vfe_pix_mnds_step(in_w, out_w), b + TFE_MNDS_H_STEP(base));
	writel(0, b + TFE_MNDS_H_PHASE(base));
	writel(vfe_pix_mnds_step(in_h, out_h), b + TFE_MNDS_V_STEP(base));
	writel(0, b + TFE_MNDS_V_PHASE(base));
	writel(0, b + TFE_MNDS_PAD0(base));
	writel(0, b + TFE_MNDS_PAD1(base));
	writel(0, b + TFE_MNDS_ROUND(base));
	writel(1, b + TFE_MNDS_EN(base));
}

static void vfe_pix_scaler_config(struct vfe_device *vfe,
				  struct vfe_line *line)
{
	const struct v4l2_mbus_framefmt *in = &line->fmt[MSM_VFE_PAD_SINK];
	const struct v4l2_rect *out = &line->compose;
	bool scale = out->width != in->width || out->height != in->height;

	vfe_pix_mnds_config(vfe, TFE_MNDS_Y, in->width, in->height,
			    out->width, out->height, scale);
	vfe_pix_mnds_config(vfe, TFE_MNDS_C, in->width, in->height,
			    out->width / 2, out->height / 2, scale);
}

/*
 * FD crop/round/clamp: the source crop rectangle (in the scaled frame), 10
 * to 8 bit rounding.  Y and C blocks, C at half resolution in both
 * directions for NV12.  Ranges are first << 16 | last.
 */
#define TFE_CRC_FD_Y			0x7300
#define TFE_CRC_FD_C			0x7500
#define TFE_CRC_EN(b)			((b) + 0x60)
#define		TFE_CRC_Y_EN			0xe01
#define		TFE_CRC_C_EN			0x3e01
#define TFE_CRC_LINES(b)		((b) + 0x68)
#define TFE_CRC_PIXELS(b)		((b) + 0x6c)
#define TFE_CRC_MIN0(b)			((b) + 0x70)
#define TFE_CRC_SHIFT0(b)		((b) + 0x74)
#define TFE_CRC_MIN1(b)			((b) + 0x78)
#define TFE_CRC_SHIFT1(b)		((b) + 0x7c)
#define TFE_CRC_MAX0(b)			((b) + 0x88)
#define TFE_CRC_MAX1(b)			((b) + 0x8c)
#define		TFE_CRC_SHIFT_10_TO_8		(2 << 3)

static u32 vfe_pix_crc_range(u32 first, u32 len)
{
	return first << 16 | (first + len - 1);
}

static void vfe_pix_crc_config(struct vfe_device *vfe,
			       const struct v4l2_rect *r)
{
	void __iomem *base = vfe->base;

	writel(vfe_pix_crc_range(r->top, r->height),
	       base + TFE_CRC_LINES(TFE_CRC_FD_Y));
	writel(vfe_pix_crc_range(r->left, r->width),
	       base + TFE_CRC_PIXELS(TFE_CRC_FD_Y));
	writel(0, base + TFE_CRC_MIN0(TFE_CRC_FD_Y));
	writel(TFE_CRC_SHIFT_10_TO_8, base + TFE_CRC_SHIFT0(TFE_CRC_FD_Y));
	writel(0xff, base + TFE_CRC_MAX0(TFE_CRC_FD_Y));
	writel(TFE_CRC_Y_EN, base + TFE_CRC_EN(TFE_CRC_FD_Y));

	writel(vfe_pix_crc_range(r->top / 2, r->height / 2),
	       base + TFE_CRC_LINES(TFE_CRC_FD_C));
	writel(vfe_pix_crc_range(r->left / 2, r->width / 2),
	       base + TFE_CRC_PIXELS(TFE_CRC_FD_C));
	writel(0, base + TFE_CRC_MIN0(TFE_CRC_FD_C));
	writel(TFE_CRC_SHIFT_10_TO_8, base + TFE_CRC_SHIFT0(TFE_CRC_FD_C));
	writel(0, base + TFE_CRC_MIN1(TFE_CRC_FD_C));
	writel(TFE_CRC_SHIFT_10_TO_8, base + TFE_CRC_SHIFT1(TFE_CRC_FD_C));
	writel(0xff, base + TFE_CRC_MAX0(TFE_CRC_FD_C));
	writel(0xff, base + TFE_CRC_MAX1(TFE_CRC_FD_C));
	writel(TFE_CRC_C_EN, base + TFE_CRC_EN(TFE_CRC_FD_C));
}

static void vfe_pix_wm_start(struct vfe_device *vfe, u8 wm, u32 width,
			     u32 height, u32 stride)
{
	writel(0, vfe->base + VFE_BUS_WM_IMAGE_CFG_1(wm));
	writel((height << 16) | width, vfe->base + VFE_BUS_WM_IMAGE_CFG_0(wm));
	writel(stride, vfe->base + VFE_BUS_WM_IMAGE_CFG_2(wm));
	writel(stride * height >> 8, vfe->base + VFE_BUS_WM_FRAME_INCR(wm));
	writel(WM_PACKER_PLAIN_8_LSB_MSB_10, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));

	writel(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
	writel(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));
	writel(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
	writel(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));

	writel(1, vfe->base + VFE_BUS_WM_MMU_PREFETCH_CFG(wm));
	writel(0xFFFFFFFF, vfe->base + VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(wm));

	/* hardware context 0 only */
	writel(BIT(0), vfe->base + VFE_BUS_WM_CTXT_CFG(wm));

	/* line based mode */
	writel(WM_CFG_EN, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_pix_start(struct vfe_device *vfe, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	unsigned int i;

	writel(WM_CGC_OVERRIDE_ALL, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);
	writel(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);
	writel(0, vfe->base + VFE_BUS_CTXT_SEL);

	for (i = 0; i < ARRAY_SIZE(vfe_pix_980_defaults); i++)
		writel(vfe_pix_980_defaults[i].val,
		       vfe->base + vfe_pix_980_defaults[i].reg);
	line->isp.lut_contrast = -1;
	vfe_isp_update(vfe, line);
	vfe_pix_scaler_config(vfe, line);
	vfe_pix_crc_config(vfe, &line->crop);
	camss_isp_apply_script(CAMSS_ISP_VFE, vfe->base);

	vfe_pix_wm_start(vfe, FD_Y_WM, pix->width, pix->height, stride);
	vfe_pix_wm_start(vfe, FD_C_WM, pix->width, pix->height / 2, stride);

	camss_isp_set_live(CAMSS_ISP_VFE, vfe->base);
}

static void vfe_wm_start(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;

	if (line->id == VFE_LINE_PIX) {
		vfe_pix_start(vfe, line);
		return;
	}

	wm = RDI_WM(wm);

	/* no clock gating at bus input */
	writel(WM_CGC_OVERRIDE_ALL, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);

	writel(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);

	writel(ALIGN(pix->plane_fmt[0].bytesperline, 16) * pix->height >> 8,
	       vfe->base + VFE_BUS_WM_FRAME_INCR(wm));
	writel((WM_IMAGE_CFG_0_DEFAULT_WIDTH & 0xFFFF),
	       vfe->base + VFE_BUS_WM_IMAGE_CFG_0(wm));
	writel(WM_IMAGE_CFG_2_DEFAULT_STRIDE,
	       vfe->base + VFE_BUS_WM_IMAGE_CFG_2(wm));
	writel(0, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));

	/* no dropped frames, one irq per frame */
	if (!vfe_is_lite(vfe)) {
		writel(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
		writel(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));
	}

	writel(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
	writel(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));

	writel(1, vfe->base + VFE_BUS_WM_MMU_PREFETCH_CFG(wm));
	writel(0xFFFFFFFF, vfe->base + VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(wm));

	writel(WM_CFG_EN | WM_CFG_MODE, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_wm_stop(struct vfe_device *vfe, u8 wm)
{
	if (wm == VFE_LINE_PIX && !vfe_is_lite(vfe)) {
		camss_isp_set_live(CAMSS_ISP_VFE, NULL);
		writel(0, vfe->base + VFE_BUS_WM_CFG(FD_Y_WM));
		writel(0, vfe->base + VFE_BUS_WM_CFG(FD_C_WM));
		return;
	}

	wm = RDI_WM(wm);
	writel(0, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_wm_update(struct vfe_device *vfe, u8 wm, u32 addr,
			  struct vfe_line *line)
{
	if (line->id == VFE_LINE_PIX) {
		struct v4l2_pix_format_mplane *pix =
			&line->video_out.active_fmt.fmt.pix_mp;
		u32 c_addr = addr + pix->plane_fmt[0].bytesperline * pix->height;

		writel(addr >> 8, vfe->base + VFE_BUS_WM_IMAGE_ADDR(FD_Y_WM));
		writel(c_addr >> 8, vfe->base + VFE_BUS_WM_IMAGE_ADDR(FD_C_WM));
		return;
	}

	wm = RDI_WM(wm);
	writel(addr >> 8, vfe->base + VFE_BUS_WM_IMAGE_ADDR(wm));

	dev_dbg(vfe->camss->dev, "wm:%d, image buf addr:0x%x\n", wm, addr);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, false);
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, true);
}

static const struct camss_video_ops vfe_video_ops_gen4 = {
	.queue_buffer = vfe_queue_buffer_v2,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->video_ops = vfe_video_ops_gen4;
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	vfe_isr_reset_ack(vfe);
}

static irqreturn_t vfe_isr(int irq, void *dev)
{
	/* nop */
	return IRQ_HANDLED;
}

static int vfe_halt(struct vfe_device *vfe)
{
	/* rely on vfe_disable_output() to stop the VFE */
	return 0;
}

const struct vfe_hw_ops vfe_ops_gen4 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr = vfe_isr,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_disable,
	.vfe_enable = vfe_enable_v2,
	.vfe_halt = vfe_halt,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_buf_done = vfe_buf_done,
	.vfe_wm_update = vfe_wm_update,
	.vfe_isp_update = vfe_isp_update,
};
