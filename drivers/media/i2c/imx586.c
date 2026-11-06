// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Sony IMX586 CMOS image sensor.
 *
 * Minimal single-mode driver (4000x3000, RAW10, 4 MIPI data lanes),
 * written for use with the Qualcomm CAMSS receiver on SM8150.
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX586_WIDTH			4000
#define IMX586_HEIGHT			3000
#define IMX586_NUM_DATA_LANES		4

#define IMX586_XCLK_FREQ		24000000
#define IMX586_LINK_FREQ		400000000
/* 4 lanes x 2 bits per link clock / 10 bits per pixel */
#define IMX586_PIXEL_RATE		(IMX586_LINK_FREQ * 2 / 10 * IMX586_NUM_DATA_LANES)

/* Frame/line length totals programmed by the mode table. */
#define IMX586_HTS_DEFAULT		8976	/* 0x2310 */
#define IMX586_HTS_MAX			0xffff
#define IMX586_VTS_DEFAULT		3064	/* 0x0bf8 */
#define IMX586_VTS_MAX			0x7fff

#define IMX586_EXPOSURE_MIN		2
#define IMX586_EXPOSURE_MAX		3062
#define IMX586_EXPOSURE_DEFAULT		3020

#define IMX586_GAIN_MIN			0x10
#define IMX586_GAIN_MAX			0x400
#define IMX586_GAIN_DEFAULT		0x70
#define IMX586_GAIN_MASK		0x3ff

#define IMX586_CHIP_ID			0x0586

#define IMX586_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX586_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX586_MODE_STANDBY		0x00
#define IMX586_MODE_STREAMING		0x01
#define IMX586_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX586_REG_GAIN			CCI_REG16(0x0204)
#define IMX586_REG_VTS			CCI_REG16(0x0340)
#define IMX586_REG_HTS			CCI_REG16(0x0342)

static const char *const imx586_supply_names[] = {
	"avdd",
	"dovdd",
	"dvdd",
};

/*
 * Global register settings, taken verbatim from the vendor
 * imx586_linear_10bit_global_regs[] table (24 MHz XVCLK).
 */
static const struct cci_reg_sequence imx586_global_regs[] = {
	/* External clock setting */
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	/* Register version */
	{ CCI_REG8(0x3c7e), 0x01 },
	{ CCI_REG8(0x3c7f), 0x08 },
	/* Signaling mode setting */
	{ CCI_REG8(0x0111), 0x02 },
	/* Global setting */
	{ CCI_REG8(0x380c), 0x00 },
	{ CCI_REG8(0x3c00), 0x10 },
	{ CCI_REG8(0x3c01), 0x10 },
	{ CCI_REG8(0x3c02), 0x10 },
	{ CCI_REG8(0x3c03), 0x10 },
	{ CCI_REG8(0x3c04), 0x10 },
	{ CCI_REG8(0x3c05), 0x01 },
	{ CCI_REG8(0x3c06), 0x00 },
	{ CCI_REG8(0x3c07), 0x00 },
	{ CCI_REG8(0x3c08), 0x03 },
	{ CCI_REG8(0x3c09), 0xff },
	{ CCI_REG8(0x3c0a), 0x01 },
	{ CCI_REG8(0x3c0b), 0x00 },
	{ CCI_REG8(0x3c0c), 0x00 },
	{ CCI_REG8(0x3c0d), 0x03 },
	{ CCI_REG8(0x3c0e), 0xff },
	{ CCI_REG8(0x3c0f), 0x20 },
	{ CCI_REG8(0x3f88), 0x00 },
	{ CCI_REG8(0x3f8e), 0x00 },
	{ CCI_REG8(0x5282), 0x01 },
	{ CCI_REG8(0x9004), 0x14 },
	{ CCI_REG8(0x9200), 0xf4 },
	{ CCI_REG8(0x9201), 0xa7 },
	{ CCI_REG8(0x9202), 0xf4 },
	{ CCI_REG8(0x9203), 0xaa },
	{ CCI_REG8(0x9204), 0xf4 },
	{ CCI_REG8(0x9205), 0xad },
	{ CCI_REG8(0x9206), 0xf4 },
	{ CCI_REG8(0x9207), 0xb0 },
	{ CCI_REG8(0x9208), 0xf4 },
	{ CCI_REG8(0x9209), 0xb3 },
	{ CCI_REG8(0x920a), 0xb7 },
	{ CCI_REG8(0x920b), 0x34 },
	{ CCI_REG8(0x920c), 0xb7 },
	{ CCI_REG8(0x920d), 0x36 },
	{ CCI_REG8(0x920e), 0xb7 },
	{ CCI_REG8(0x920f), 0x37 },
	{ CCI_REG8(0x9210), 0xb7 },
	{ CCI_REG8(0x9211), 0x38 },
	{ CCI_REG8(0x9212), 0xb7 },
	{ CCI_REG8(0x9213), 0x39 },
	{ CCI_REG8(0x9214), 0xb7 },
	{ CCI_REG8(0x9215), 0x3a },
	{ CCI_REG8(0x9216), 0xb7 },
	{ CCI_REG8(0x9217), 0x3c },
	{ CCI_REG8(0x9218), 0xb7 },
	{ CCI_REG8(0x9219), 0x3d },
	{ CCI_REG8(0x921a), 0xb7 },
	{ CCI_REG8(0x921b), 0x3e },
	{ CCI_REG8(0x921c), 0xb7 },
	{ CCI_REG8(0x921d), 0x3f },
	{ CCI_REG8(0x921e), 0x77 },
	{ CCI_REG8(0x921f), 0x77 },
	{ CCI_REG8(0x9222), 0xc4 },
	{ CCI_REG8(0x9223), 0x4b },
	{ CCI_REG8(0x9224), 0xc4 },
	{ CCI_REG8(0x9225), 0x4c },
	{ CCI_REG8(0x9226), 0xc4 },
	{ CCI_REG8(0x9227), 0x4d },
	{ CCI_REG8(0x9810), 0x14 },
	{ CCI_REG8(0x9814), 0x14 },
	{ CCI_REG8(0x99b2), 0x20 },
	{ CCI_REG8(0x99b3), 0x0f },
	{ CCI_REG8(0x99b4), 0x0f },
	{ CCI_REG8(0x99b5), 0x0f },
	{ CCI_REG8(0x99b6), 0x0f },
	{ CCI_REG8(0x99e4), 0x0f },
	{ CCI_REG8(0x99e5), 0x0f },
	{ CCI_REG8(0x99e6), 0x0f },
	{ CCI_REG8(0x99e7), 0x0f },
	{ CCI_REG8(0x99e8), 0x0f },
	{ CCI_REG8(0x99e9), 0x0f },
	{ CCI_REG8(0x99ea), 0x0f },
	{ CCI_REG8(0x99eb), 0x0f },
	{ CCI_REG8(0x99ec), 0x0f },
	{ CCI_REG8(0x99ed), 0x0f },
	{ CCI_REG8(0xa569), 0x06 },
	{ CCI_REG8(0xa679), 0x20 },
	{ CCI_REG8(0xc020), 0x01 },
	{ CCI_REG8(0xc61d), 0x00 },
	{ CCI_REG8(0xc625), 0x00 },
	{ CCI_REG8(0xc638), 0x03 },
	{ CCI_REG8(0xc63b), 0x01 },
	{ CCI_REG8(0xe286), 0x31 },
	{ CCI_REG8(0xe2a6), 0x32 },
	{ CCI_REG8(0xe2c6), 0x33 },
	{ CCI_REG8(0xbcf1), 0x00 },
	/* Image quality adjustment setting */
	{ CCI_REG8(0x9852), 0x00 },
	{ CCI_REG8(0x9954), 0x0f },
	{ CCI_REG8(0xa7ad), 0x01 },
	{ CCI_REG8(0xa7cb), 0x01 },
	{ CCI_REG8(0xae09), 0xff },
	{ CCI_REG8(0xae0a), 0xff },
	{ CCI_REG8(0xae12), 0x58 },
	{ CCI_REG8(0xae13), 0x58 },
	{ CCI_REG8(0xae15), 0x10 },
	{ CCI_REG8(0xae16), 0x10 },
	{ CCI_REG8(0xaf05), 0x48 },
	{ CCI_REG8(0xb07c), 0x02 },
};

/*
 * 4000x3000, 10-bit, 4-lane, 30 fps mode, taken verbatim from the vendor
 * imx586_linear_10bit_4000x3000_30fps_nopd_regs[] table.
 */
static const struct cci_reg_sequence imx586_mode_4000x3000_regs[] = {
	/* MIPI output setting */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	/* Line length PCK setting: HTS = 8976 */
	{ CCI_REG8(0x0342), 0x23 },
	{ CCI_REG8(0x0343), 0x10 },
	/* Frame length lines setting: VTS = 3064 */
	{ CCI_REG8(0x0340), 0x0b },
	{ CCI_REG8(0x0341), 0xf8 },
	/* ROI setting */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0x3f },
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0x6f },
	/* Mode setting */
	{ CCI_REG8(0x0220), 0x62 },
	{ CCI_REG8(0x0222), 0x01 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3140), 0x00 },
	{ CCI_REG8(0x3246), 0x81 },
	{ CCI_REG8(0x3247), 0x81 },
	{ CCI_REG8(0x3f15), 0x00 },
	/* Digital crop & scaling */
	{ CCI_REG8(0x0401), 0x00 },
	{ CCI_REG8(0x0404), 0x00 },
	{ CCI_REG8(0x0405), 0x10 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0f },
	{ CCI_REG8(0x040d), 0xa0 },
	{ CCI_REG8(0x040e), 0x0b },
	{ CCI_REG8(0x040f), 0xb8 },
	/* Output size setting */
	{ CCI_REG8(0x034c), 0x0f },
	{ CCI_REG8(0x034d), 0xa0 },
	{ CCI_REG8(0x034e), 0x0b },
	{ CCI_REG8(0x034f), 0xb8 },
	/* Clock setting */
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x04 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x58 },
	{ CCI_REG8(0x030b), 0x02 },
	{ CCI_REG8(0x030d), 0x03 },
	{ CCI_REG8(0x030e), 0x01 },
	{ CCI_REG8(0x030f), 0x1f },
	{ CCI_REG8(0x0310), 0x01 },
	/* Other setting */
	{ CCI_REG8(0x3620), 0x00 },
	{ CCI_REG8(0x3621), 0x00 },
	{ CCI_REG8(0x3c11), 0x04 },
	{ CCI_REG8(0x3c12), 0x03 },
	{ CCI_REG8(0x3c13), 0x2d },
	{ CCI_REG8(0x3f0c), 0x00 },
	{ CCI_REG8(0x3f14), 0x00 },
	{ CCI_REG8(0x3f80), 0x01 },
	{ CCI_REG8(0x3f81), 0x90 },
	{ CCI_REG8(0x3f8c), 0x00 },
	{ CCI_REG8(0x3f8d), 0x14 },
	{ CCI_REG8(0x3ff8), 0x01 },
	{ CCI_REG8(0x3ff9), 0x2a },
	{ CCI_REG8(0x3ffe), 0x00 },
	{ CCI_REG8(0x3fff), 0x6c },
	/* Integration setting */
	{ CCI_REG8(0x0202), 0x0b },
	{ CCI_REG8(0x0203), 0xc4 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x3fe0), 0x01 },
	{ CCI_REG8(0x3fe1), 0xf4 },
	/* Gain setting */
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x70 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x70 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0210), 0x01 },
	{ CCI_REG8(0x0211), 0x00 },
	{ CCI_REG8(0x0212), 0x01 },
	{ CCI_REG8(0x0213), 0x00 },
	{ CCI_REG8(0x0214), 0x01 },
	{ CCI_REG8(0x0215), 0x00 },
	{ CCI_REG8(0x3fe2), 0x00 },
	{ CCI_REG8(0x3fe3), 0x70 },
	{ CCI_REG8(0x3fe4), 0x01 },
	{ CCI_REG8(0x3fe5), 0x00 },
	/* PDAF type1 setting */
	{ CCI_REG8(0x3e20), 0x01 },
	{ CCI_REG8(0x3e37), 0x01 },
};

struct imx586 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx586_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;

	struct mutex mutex;
	bool streaming;
};

static inline struct imx586 *to_imx586(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx586, sd);
}

static int imx586_power_on(struct imx586 *sensor)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret)
		return ret;

	ret = clk_set_rate(sensor->xclk, IMX586_XCLK_FREQ);
	if (ret)
		goto err_regulators;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret)
		goto err_regulators;

	gpiod_set_value_cansleep(sensor->reset, 0);
	/* The sensor needs 8192 XVCLK cycles (about 0.34 ms) after reset. */
	usleep_range(10000, 12000);

	return 0;

err_regulators:
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);

	return ret;
}

static void imx586_power_off(struct imx586 *sensor)
{
	gpiod_set_value_cansleep(sensor->reset, 1);
	clk_disable_unprepare(sensor->xclk);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

static int imx586_start_stream(struct imx586 *sensor)
{
	u64 chip_id;
	int ret;

	ret = cci_read(sensor->regmap, IMX586_REG_CHIP_ID, &chip_id, NULL);
	if (ret)
		return ret;

	if (chip_id != IMX586_CHIP_ID)
		dev_warn(sensor->dev, "unexpected chip ID 0x%04llx\n", chip_id);

	ret = cci_multi_reg_write(sensor->regmap, imx586_global_regs,
				  ARRAY_SIZE(imx586_global_regs), NULL);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(sensor->regmap, imx586_mode_4000x3000_regs,
				  ARRAY_SIZE(imx586_mode_4000x3000_regs), NULL);
	if (ret)
		return ret;

	ret = v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		return ret;

	return cci_write(sensor->regmap, IMX586_REG_MODE_SELECT,
			 IMX586_MODE_STREAMING, NULL);
}

static int imx586_stop_stream(struct imx586 *sensor)
{
	int ret;

	ret = cci_write(sensor->regmap, IMX586_REG_MODE_SELECT,
			IMX586_MODE_STANDBY, NULL);

	return ret;
}

static int imx586_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx586 *sensor = to_imx586(sd);
	int ret = 0;

	mutex_lock(&sensor->mutex);

	if (sensor->streaming == enable)
		goto out;

	if (enable) {
		ret = imx586_power_on(sensor);
		if (ret)
			goto out;

		/* Let imx586_s_ctrl() program the controls from now on. */
		sensor->streaming = true;

		ret = imx586_start_stream(sensor);
		if (ret) {
			sensor->streaming = false;
			imx586_power_off(sensor);
			goto out;
		}
	} else {
		sensor->streaming = false;
		ret = imx586_stop_stream(sensor);
		imx586_power_off(sensor);
	}

out:
	mutex_unlock(&sensor->mutex);

	return ret;
}

/*
 * The VBLANK and HBLANK controls carry the raw VTS (0x0340) and HTS (0x0342)
 * values rather than blanking line counts, matching the register defaults
 * used by the mode table. EXPOSURE and ANALOGUE_GAIN likewise carry the raw
 * register codes written to 0x0202 and 0x0204.
 */
static int imx586_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx586 *sensor = container_of(ctrl->handler, struct imx586,
					     ctrls);

	/* Register values are (re)applied by imx586_start_stream(). */
	if (!sensor->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		return cci_write(sensor->regmap, IMX586_REG_EXPOSURE,
				 ctrl->val, NULL);
	case V4L2_CID_ANALOGUE_GAIN:
		return cci_update_bits(sensor->regmap, IMX586_REG_GAIN,
				       IMX586_GAIN_MASK, ctrl->val, NULL);
	case V4L2_CID_VBLANK:
		return cci_write(sensor->regmap, IMX586_REG_VTS,
				 ctrl->val, NULL);
	case V4L2_CID_HBLANK:
		return cci_write(sensor->regmap, IMX586_REG_HTS,
				 ctrl->val, NULL);
	default:
		return 0;
	}
}

static const struct v4l2_ctrl_ops imx586_ctrl_ops = {
	.s_ctrl = imx586_s_ctrl,
};

static const s64 imx586_link_freq_menu[] = {
	IMX586_LINK_FREQ,
};

static int imx586_ctrls_init(struct imx586 *sensor)
{
	int ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 8);

	sensor->link_freq =
		v4l2_ctrl_new_int_menu(&sensor->ctrls, &imx586_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(imx586_link_freq_menu) - 1, 0,
				       imx586_link_freq_menu);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->pixel_rate =
		v4l2_ctrl_new_std(&sensor->ctrls, &imx586_ctrl_ops,
				  V4L2_CID_PIXEL_RATE, IMX586_PIXEL_RATE,
				  IMX586_PIXEL_RATE, 1, IMX586_PIXEL_RATE);
	if (sensor->pixel_rate)
		sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->hblank =
		v4l2_ctrl_new_std(&sensor->ctrls, &imx586_ctrl_ops,
				  V4L2_CID_HBLANK, IMX586_HTS_DEFAULT,
				  IMX586_HTS_MAX, 1, IMX586_HTS_DEFAULT);

	sensor->vblank =
		v4l2_ctrl_new_std(&sensor->ctrls, &imx586_ctrl_ops,
				  V4L2_CID_VBLANK, IMX586_VTS_DEFAULT,
				  IMX586_VTS_MAX, 1, IMX586_VTS_DEFAULT);

	sensor->exposure =
		v4l2_ctrl_new_std(&sensor->ctrls, &imx586_ctrl_ops,
				  V4L2_CID_EXPOSURE, IMX586_EXPOSURE_MIN,
				  IMX586_EXPOSURE_MAX, 1,
				  IMX586_EXPOSURE_DEFAULT);

	sensor->gain =
		v4l2_ctrl_new_std(&sensor->ctrls, &imx586_ctrl_ops,
				  V4L2_CID_ANALOGUE_GAIN, IMX586_GAIN_MIN,
				  IMX586_GAIN_MAX, 1, IMX586_GAIN_DEFAULT);

	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		dev_err_probe(sensor->dev, ret, "failed to add controls\n");
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	sensor->sd.ctrl_handler = &sensor->ctrls;

	return 0;
}

static int imx586_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx586_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index != 0 || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = IMX586_WIDTH;
	fse->max_width = IMX586_WIDTH;
	fse->min_height = IMX586_HEIGHT;
	fse->max_height = IMX586_HEIGHT;

	return 0;
}

static int imx586_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index != 0)
		return -EINVAL;

	fie->width = IMX586_WIDTH;
	fie->height = IMX586_HEIGHT;
	fie->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fie->interval.numerator = 1;
	fie->interval.denominator = 30;

	return 0;
}

static void imx586_fill_format(struct v4l2_mbus_framefmt *format)
{
	format->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	format->width = IMX586_WIDTH;
	format->height = IMX586_HEIGHT;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx586_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	/* Single supported mode: always report the full 4000x3000 frame. */
	imx586_fill_format(format);

	fmt->format = *format;

	return 0;
}

static int imx586_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX586_WIDTH;
		sel->r.height = IMX586_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx586_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	imx586_fill_format(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops imx586_video_ops = {
	.s_stream = imx586_s_stream,
};

static const struct v4l2_subdev_pad_ops imx586_pad_ops = {
	.enum_mbus_code = imx586_enum_mbus_code,
	.enum_frame_size = imx586_enum_frame_size,
	.enum_frame_interval = imx586_enum_frame_interval,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx586_set_fmt,
	.get_selection = imx586_get_selection,
};

static const struct v4l2_subdev_ops imx586_subdev_ops = {
	.video = &imx586_video_ops,
	.pad = &imx586_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx586_internal_ops = {
	.init_state = imx586_init_state,
};

static int imx586_parse_endpoint(struct imx586 *sensor)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned int i, num_lanes;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!ep)
		return dev_err_probe(sensor->dev, -ENXIO,
				     "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to parse endpoint\n");

	num_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
	if (num_lanes > IMX586_NUM_DATA_LANES) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "%u data lanes exceeds the supported %u\n",
				    num_lanes, IMX586_NUM_DATA_LANES);
		goto out_free;
	}

	if (num_lanes != IMX586_NUM_DATA_LANES)
		dev_warn(sensor->dev,
			 "%u data lanes in firmware, driver only supports %u\n",
			 num_lanes, IMX586_NUM_DATA_LANES);

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_warn(sensor->dev,
			 "no link-frequencies in firmware, assuming %u Hz\n",
			 IMX586_LINK_FREQ);
	} else {
		for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
			if (bus_cfg.link_frequencies[i] == IMX586_LINK_FREQ)
				break;
		}

		if (i == bus_cfg.nr_of_link_frequencies)
			dev_warn(sensor->dev,
				 "no %u Hz link frequency in firmware\n",
				 IMX586_LINK_FREQ);
	}

	ret = 0;

out_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx586_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx586 *sensor;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;
	mutex_init(&sensor->mutex);

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = imx586_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk))
		return dev_err_probe(dev, PTR_ERR(sensor->xclk),
				     "failed to get xclk\n");

	ret = imx586_parse_endpoint(sensor);
	if (ret)
		return ret;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "failed to init CCI regmap\n");

	v4l2_i2c_subdev_init(&sensor->sd, client, &imx586_subdev_ops);
	sensor->sd.internal_ops = &imx586_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = imx586_ctrls_init(sensor);
	if (ret)
		return ret;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_ctrls;

	sensor->sd.state_lock = sensor->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_entity;

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_subdev;

	return 0;

err_subdev:
	v4l2_subdev_cleanup(&sensor->sd);
err_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);

	return ret;
}

static void imx586_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *sensor = to_imx586(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	mutex_destroy(&sensor->mutex);
}

static const struct of_device_id imx586_of_match[] = {
	{ .compatible = "sony,imx586" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx586_of_match);

static const struct i2c_device_id imx586_id[] = {
	{ "imx586", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, imx586_id);

static struct i2c_driver imx586_driver = {
	.driver = {
		.name = "imx586",
		.of_match_table = imx586_of_match,
	},
	.probe = imx586_probe,
	.remove = imx586_remove,
	.id_table = imx586_id,
};

module_i2c_driver(imx586_driver);

MODULE_DESCRIPTION("Sony IMX586 image sensor driver");
MODULE_LICENSE("GPL");
