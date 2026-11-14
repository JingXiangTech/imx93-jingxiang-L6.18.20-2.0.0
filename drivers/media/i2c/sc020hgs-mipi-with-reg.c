// SPDX-License-Identifier: GPL-2.0
/*
 * sc020.c - SmartSens SC020HGS 全局快门 RAW Mono sensor 驱动（M1 骨架）
 *
 * 芯片: SC020HGS (SmartSens) 全局快门 RAW Mono
 * 模式: 400x400 @ 240fps, RAW10 (MIPI DT 0x2B), 1 lane @ 648Mbps
 * 总线: I2C 7bit 0x30, 16bit 寄存器地址 + 8bit 数据
 * 时钟: EXTCLK 24MHz, 内部像素时钟 FPCLK 64.8MHz
 *
 * 所有寄存器出处: SC020HGS 数据手册 V0.7 / 设计应用指南 V1.0
 * 逐条映射见: SC020/SC020驱动方案_v0.1.md §3.1（节号均指手册）
 *
 * 骨架范围(M1): probe + chip ID 校验 + 上下电时序 + s_stream(0x2100)
 *               + 格式写死(Y10_1X10/400x400) + TEST_PATTERN/EXPOSURE ctrl
 *               + 初始化表全量 141 条（10bit 套, doce/Cleaned_0x04_*.txt）
 * 未含(M2+): runtime_pm、DTS 完整 regulator/clock 绑定
 * 注: 0x3031/0x3037/HTS/VTS/窗口等寄存器不在列表——手册默认值即 400x400@240fps
 *     RAW10 配置, SmartSens 工具只写与默认不同的寄存器, 属正常。
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define Enable_DEBUG
#ifdef Enable_DEBUG
#define dbg(...) printk(__VA_ARGS__)
#else
#define dbg(...)
#endif

/* ------------------------------------------------------------------
 * 寄存器定义（手册 V0.7）
 * ------------------------------------------------------------------ */

/* 系统（手册 1.3） */
#define SC020_REG_SW_RESET		0x2103	/* bit0: 软复位, 持续150ns (1.3.4, 列表首条) */
#define SC020_REG_SLEEP			0x2100	/* bit0: 1=唤醒出流 / 0=睡眠 (1.3.3, 列表尾条) */
						/* 注: 手册无 0x0100 流开关记载, M1 上板实测 */

/* 探测（手册 1.5 表 1-5） */
#define SC020_REG_CHIP_ID_HI		0x3107	/* 默认 0x3A */
#define SC020_REG_CHIP_ID_LO		0x3108	/* 默认 0x7A */
#define SC020_CHIP_ID			0x3a7a

/* MIPI 输出（手册 1.6 表 1-6/1-7） */
#define SC020_REG_MIPI_BIT_MODE		0x3031	/* bit[4:0]: 0xA=RAW10 / 0x8=RAW8（10bit 列表=0x0A） */
#define SC020_REG_PHY_BIT_MODE		0x3037	/* bit[6:5]: 1=10bit PHY（10bit 列表含） */

/* 曝光（手册 2.5.1 表 2-5）: {3e00[3:0], 3e01[7:0], 3e02[7:4]} 单位=行 */
#define SC020_REG_EXP_HI		0x3e00
#define SC020_REG_EXP_MID		0x3e01
#define SC020_REG_EXP_LO		0x3e02	/* V0.7 收窄: 仅 bit[7:4] 有效 */
#define SC020_EXP_MIN_LINES		4	/* 手册明示最小 4 行 */
#define SC020_EXP_MARGIN_LINES		9	/* 手册明示 max = VTS - 9 */

/* Group Hold（手册 2.6 表 2-8）: AE 原子更新用（本骨架未启, M5） */
#define SC020_REG_GROUP_HOLD		0x3812	/* 0x00 开始 / 0x30 释放 */

/* 测试图案（手册 2.9 表 2-12） */
#define SC020_REG_TEST_PATTERN		0x4501	/* bit3: 1=灰度渐变 */

/* ------------------------------------------------------------------
 * 写死模式: 400x400 @ 240fps（方案 §2.2, 列表实证）
 * ------------------------------------------------------------------ */
#define SC020_WIDTH			400
#define SC020_HEIGHT			400
#define SC020_HTS			600	/* 0x320C/D: 400 有效 + 200 消隐 */
#define SC020_VTS			450	/* 0x320E/F: 400 有效 + 50 消隐 */

#define SC020HGS_DATA_LANES 1

#ifdef SUPPORT_10BIT
	#define SC020HGS_DEFAULT_CODE MEDIA_BUS_FMT_Y10_1X10
	/* 帧率 = FPCLK / (HTS x VTS) = 64.8M / 270000 = 240fps */
	// #define SC020_PIXEL_RATE		64800000ULL
	/* V4L2 link freq = 每 lane 线速率的一半(DDR): 648Mbps / 2 */
	// #define SC020_LINK_FREQ_648MBPS		324000000ULL
	// #define SC020_LINK_FREQ		324000000ULL
	#define SC020_PIXEL_RATE (SC020_LINK_FREQ *2 * SC020HGS_DATA_LANES) / 10
	#define RAW_STR "RAW10"
#else
	#define SC020HGS_DEFAULT_CODE MEDIA_BUS_FMT_SBGGR8_1X8
	#define RAW_STR "RAW8"
	// #define SC020_PIXEL_RATE 52800000ULL
	#define SC020_LINK_FREQ 264000000ULL
	#define SC020_PIXEL_RATE (SC020_LINK_FREQ *2 * SC020HGS_DATA_LANES) / 8
#endif

static const s64 sc020_link_freqs[] = {
	SC020_LINK_FREQ,
};

#define SC020_LINK_FREQ_IDX		0

/* 初始化表（全量 141 条, M1 已转换）
 * 来源: doce 10bit 套 Cleaned_0x04_..._10bit_648Mbps_400x400_240fps.txt
 *       (SmartSens 工具 V00P03_20250305)
 * 已核对: 首条=软复位 0x2103 / 尾条=唤醒 0x2100,0x01;
 *       HTS=0x0258(600) VTS=0x01c2(450) -> 240fps;
 *       0x3031=0x0A(RAW10)/0x3037 10bit PHY; 0x0100 不在列表(维持 0x2100 路线);
 *       0x3e02 曝光位宽按手册 V0.7 语义(仅 bit[7:4] 有效)。
 */
static const struct sc020_regval {
	u16 reg;
	u8 val;
};

static struct sc020_regval sc020hgs_24M_1_lan_8bit_400X400_240FPS_mipi[] = {
	{0x2103, 0x01},
	{0x2100, 0x00},
	{0x36e9, 0x80},
	{0x300a, 0x40},
	{0x301f, 0x09},
	{0x3031, 0x08},
	{0x3033, 0xa8},
	{0x3037, 0x00},
	{0x320e, 0x01},
	{0x320f, 0xca},
	{0x3250, 0x03},
	{0x3303, 0x28},
	{0x3304, 0x40},
	{0x3305, 0x00},
	{0x3306, 0x40},
	{0x3309, 0x40},
	{0x330a, 0x00},
	{0x330b, 0x80},
	{0x3314, 0xc8},
	{0x3317, 0x98},
	{0x331f, 0x02},
	{0x3320, 0xf1},
	{0x3363, 0x9f},
	{0x3385, 0x31},
	{0x3387, 0x31},
	{0x33ad, 0x38},
	{0x33b5, 0x08},
	{0x33b6, 0x18},
	{0x33b8, 0x28},
	{0x33ba, 0x28},
	{0x33ef, 0x04},
	{0x33f8, 0x00},
	{0x33fa, 0x00},
	{0x341c, 0x10},
	{0x341e, 0x20},
	{0x341f, 0x20},
	{0x3420, 0x08},
	{0x3421, 0x18},
	{0x3422, 0x38},
	{0x3424, 0x28},
	{0x3426, 0x28},
	{0x3428, 0x28},
	{0x3435, 0x07},
	{0x3436, 0x01},
	{0x3437, 0x78},
	{0x3438, 0x01},
	{0x3439, 0x03},
	{0x343a, 0x00},
	{0x343b, 0xd7},
	{0x343c, 0x00},
	{0x343d, 0xca},
	{0x349c, 0x01},
	{0x34f3, 0x08},
	{0x34f4, 0x18},
	{0x34f5, 0x38},
	{0x3618, 0x92},
	{0x3619, 0x01},
	{0x3630, 0x46},
	{0x3631, 0x76},
	{0x3632, 0x1c},
	{0x3633, 0xa6},
	{0x3634, 0x90},
	{0x3635, 0x36},
	{0x3637, 0x0c},
	{0x3638, 0x04},
	{0x3670, 0x72},
	{0x3671, 0x62},
	{0x3672, 0x64},
	{0x3673, 0x08},
	{0x3674, 0x18},
	{0x3675, 0x74},
	{0x3676, 0x74},
	{0x3677, 0x74},
	{0x3678, 0x08},
	{0x3679, 0x18},
	{0x368f, 0x0c},
	{0x3690, 0x0c},
	{0x3691, 0x0c},
	{0x3692, 0x08},
	{0x3693, 0x18},
	{0x3699, 0xc4},
	{0x369a, 0xc5},
	{0x369b, 0xc3},
	{0x369c, 0xc4},
	{0x369d, 0xce},
	{0x369e, 0x08},
	{0x369f, 0x18},
	{0x36a0, 0x38},
	{0x36a1, 0x78},
	{0x36ea, 0x0b},
	{0x36eb, 0x04},
	{0x36ec, 0x43},
	{0x36ed, 0x96},
	{0x3721, 0x8b},
	{0x3723, 0xb1},
	{0x3726, 0x80},
	{0x3727, 0x18},
	{0x391f, 0x41},
	{0x3e00, 0x00},
	{0x3e01, 0x1c},
	{0x3e02, 0x10},
	{0x3e03, 0x0b},
	{0x4330, 0x1a},
	{0x4331, 0x02},
	{0x4350, 0x02},
	{0x435c, 0x0d},
	{0x4360, 0x1f},
	{0x4361, 0x00},
	{0x4362, 0x80},
	{0x4363, 0x00},
	{0x4364, 0x78},
	{0x4365, 0x18},
	{0x4366, 0x38},
	{0x4367, 0x00},
	{0x4368, 0x40},
	{0x4369, 0x00},
	{0x436a, 0x40},
	{0x436b, 0x18},
	{0x436c, 0x38},
	{0x436d, 0x78},
	{0x4370, 0x02},
	{0x4371, 0x02},
	{0x4372, 0x08},
	{0x4373, 0x18},
	{0x437a, 0x00},
	{0x437b, 0x38},
	{0x4380, 0x02},
	{0x4381, 0x02},
	{0x4382, 0x00},
	{0x4383, 0x08},
	{0x4384, 0x18},
	{0x4385, 0x38},
	{0x440e, 0x02},
	{0x4509, 0x1c},
	{0x4819, 0x08},
	{0x481b, 0x05},
	{0x481d, 0x12},
	{0x481f, 0x04},
	{0x4821, 0x0a},
	{0x4823, 0x04},
	{0x4825, 0x04},
	{0x4827, 0x04},
	{0x4829, 0x07},
	{0x36e9, 0x20},
	{0x2100, 0x01},
};
#define sc020_init_table sc020hgs_24M_1_lan_8bit_400X400_240FPS_mipi

/* ------------------------------------------------------------------
 * 驱动私有结构
 * ------------------------------------------------------------------ */
struct sc020 {
	struct i2c_client *client;
	struct regmap *regmap;
	struct clk *inclk;		/* EXTCLK 24MHz */
	struct gpio_desc *reset;	/* XSHUTDN, DTS 标 active-low */
	struct regulator *avdd;		/* 2.8V (禁 DCDC 直供) */
	struct regulator *dovdd;	/* 1.8V, 与主控 IO 同域 */
	struct regulator *dvdd;		/* 1.2V */

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_mbus_framefmt fmt;	/* active 格式（写死单模式） */
	bool powered;
};

static inline struct sc020 *to_sc020(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc020, sd);
}

static const struct regmap_config sc020_regmap_config = {
	.reg_bits		= 16,
	.val_bits		= 8,
	.max_register		= 0xffff,
	.reg_format_endian	= REGMAP_ENDIAN_BIG,
};

/* ------------------------------------------------------------------
 * 寄存器读写 / 初始化表
 * ------------------------------------------------------------------ */
static int sc020_write_reg(struct sc020 *sc020, u16 reg, u8 val)
{
	int ret;

	ret = regmap_write(sc020->regmap, reg, val);
	if (ret)
		dev_err(&sc020->client->dev, "write 0x%04x failed: %d\n", reg, ret);
	return ret;
}

static int sc020_read_reg(struct sc020 *sc020, u16 reg, unsigned int *val)
{
	return regmap_read(sc020->regmap, reg, val);
}

static int sc020_load_init_table(struct sc020 *sc020)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(sc020_init_table); i++) {
		ret = sc020_write_reg(sc020, sc020_init_table[i].reg,
				      sc020_init_table[i].val);
		if (ret)
			return ret;
		/* 软复位需 150ns, 逐条给足裕量 */
		fsleep(1000);
	}
	return 0;
}

/* ------------------------------------------------------------------
 * 上下电时序（方案 §3.1.1 / 手册图 1-3、指南图 1-1/1-2）
 *
 * 上电: EXTCLK 先有效(须上电前就绪) -> DOVDD ->T1>=0ms-> DVDD
 *       ->T2>=0ms-> AVDD ->T3>=0ms-> XSHUTDN 释放 ->T4>=4ms-> I2C
 * 下电: XSHUTDN 拉低 -> >=6 个 EXTCLK -> 断电断钟
 *       铁律: DOVDD 掉电后 XSHUTDN/EXTCLK/SDA/SCL 不得保持高
 * ------------------------------------------------------------------ */
static int sc020_power_on(struct sc020 *sc020)
{
	struct device *dev = &sc020->client->dev;
	int ret;

	if (sc020->powered)
		return 0;

	/* EXTCLK 必须先于/伴随全程上电有效 */
	ret = clk_prepare_enable(sc020->inclk);
	if (ret) {
		dev_err(dev, "enable inclk failed: %d\n", ret);
		return ret;
	}

	if (!IS_ERR_OR_NULL(sc020->dovdd)) {
		ret = regulator_enable(sc020->dovdd);
		if (ret)
			goto err_clk;
		fsleep(1000);		/* T1 >= 0ms, 取 1ms */
	}
	if (!IS_ERR_OR_NULL(sc020->dvdd)) {
		ret = regulator_enable(sc020->dvdd);
		if (ret)
			goto err_dovdd;
		fsleep(1000);		/* T2 >= 0ms, 取 1ms */
	}
	if (!IS_ERR_OR_NULL(sc020->avdd)) {
		ret = regulator_enable(sc020->avdd);
		if (ret)
			goto err_dvdd;
		fsleep(1000);		/* T3 >= 0ms, 取 1ms */
	}

	gpiod_set_value_cansleep(sc020->reset, 1);
	fsleep(10000);

	gpiod_set_value_cansleep(sc020->reset, 0);
	fsleep(5000);
	// /* XSHUTDN 释放（active-low: 写 0 = 物理拉高 = 释放复位） */
	// gpiod_set_value_cansleep(sc020->reset, 0);

	// /* T4 >= 4ms 后才允许首次 I2C, 取 5ms 裕量 */
	// fsleep(5000);

	sc020->powered = true;
	return 0;

err_dvdd:
	if (!IS_ERR_OR_NULL(sc020->dvdd))
		regulator_disable(sc020->dvdd);
err_dovdd:
	if (!IS_ERR_OR_NULL(sc020->dovdd))
		regulator_disable(sc020->dovdd);
err_clk:
	clk_disable_unprepare(sc020->inclk);
	dev_err(dev, "power on failed: %d\n", ret);
	return ret;
}

static void sc020_power_off(struct sc020 *sc020)
{
	if (!sc020->powered)
		return;

	/* 先写睡眠再断电, 干净收尾 */
	sc020_write_reg(sc020, SC020_REG_SLEEP, 0x00);

	/* XSHUTDN 拉低（active-low: 写 1 = 物理低 = 复位保持） */
	gpiod_set_value_cansleep(sc020->reset, 1);
	/* >= 6 个 EXTCLK (24M 时仅 250ns), 取 1ms 裕量 */
	fsleep(1000);

	if (!IS_ERR_OR_NULL(sc020->avdd))
		regulator_disable(sc020->avdd);
	if (!IS_ERR_OR_NULL(sc020->dvdd))
		regulator_disable(sc020->dvdd);
	if (!IS_ERR_OR_NULL(sc020->dovdd))
		regulator_disable(sc020->dovdd);
	clk_disable_unprepare(sc020->inclk);

	sc020->powered = false;
}

/* ------------------------------------------------------------------
 * V4L2 subdev ops: 流开关 / 格式
 * ------------------------------------------------------------------ */
static int sc020_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc020 *sc020 = to_sc020(sd);
	int ret;

	if (enable) {
		ret = sc020_power_on(sc020);
		if (ret)
			return ret;

		ret = sc020_load_init_table(sc020);
		if (ret)
			goto err_power;

		/* 睡眠寄存器即流开关: 1=唤醒出流（方案 §3 条4, 0x2100 实证路线） */
		ret = sc020_write_reg(sc020, SC020_REG_SLEEP, 0x01);
		if (ret)
			goto err_power;
		return 0;

err_power:
		sc020_power_off(sc020);
		return ret;
	}

	sc020_write_reg(sc020, SC020_REG_SLEEP, 0x00);
	sc020_power_off(sc020);
	return 0;
}

static int sc020_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct sc020 *sc020 = to_sc020(sd);
	struct v4l2_mbus_framefmt *mf;

	/* 单模式写死: 仅 Y10_1X10 / 400x400, 其余钳位回填（方案 §3 条5） */
	format->format.code = SC020HGS_DEFAULT_CODE;
	format->format.width = SC020_WIDTH;
	format->format.height = SC020_HEIGHT;
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_RAW;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		mf = v4l2_subdev_state_get_format(state, format->pad);
	else
		mf = &sc020->fmt;

	*mf = format->format;
	return 0;
}

static int sc020_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct sc020 *sc020 = to_sc020(sd);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		format->format = *v4l2_subdev_state_get_format(state, format->pad);
	else
		format->format = sc020->fmt;

	return 0;
}

static int sc020_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = SC020HGS_DEFAULT_CODE;
	return 0;
}

static int sc020_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->index)
		return -EINVAL;
	if (fsize->code != SC020HGS_DEFAULT_CODE)
		return -EINVAL;

	fsize->min_width = SC020_WIDTH;
	fsize->max_width = SC020_WIDTH;
	fsize->min_height = SC020_HEIGHT;
	fsize->max_height = SC020_HEIGHT;
	return 0;
}

static const struct v4l2_subdev_video_ops sc020_video_ops = {
	.s_stream = sc020_s_stream,
};

static const struct v4l2_subdev_pad_ops sc020_pad_ops = {
	.enum_mbus_code		= sc020_enum_mbus_code,
	.enum_frame_size	= sc020_enum_frame_size,
	.get_fmt		= sc020_get_fmt,
	.set_fmt		= sc020_set_fmt,
};

static const struct v4l2_subdev_ops sc020_subdev_ops = {
	.video	= &sc020_video_ops,
	.pad	= &sc020_pad_ops,
};

/* ------------------------------------------------------------------
 * V4L2 controls
 * ------------------------------------------------------------------ */
static int sc020_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc020 *sc020 = container_of(ctrl->handler, struct sc020, ctrls);
	unsigned int val;
	int ret;

	/* 只在已上电时写硬件（未上电时双保险直接跳过） */
	if (!sc020->powered)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		val = ctrl->val;
		/* {3e00[3:0], 3e01[7:0], 3e02[7:4]} (表 2-5; 3e02 仅高 4 位, V0.7) */
		ret  = sc020_write_reg(sc020, SC020_REG_EXP_HI, (val >> 12) & 0x0f);
		ret |= sc020_write_reg(sc020, SC020_REG_EXP_MID, (val >> 4) & 0xff);
		ret |= sc020_write_reg(sc020, SC020_REG_EXP_LO, (val << 4) & 0xf0);
		return ret;

	case V4L2_CID_TEST_PATTERN:
		/* 0x4501 bit3: 1=灰度渐变 (表 2-12), 读改写保其余位 */
		ret = sc020_read_reg(sc020, SC020_REG_TEST_PATTERN, &val);
		if (ret)
			return ret;
		if (ctrl->val)
			val |= 0x08;
		else
			val &= ~0x08;
		return sc020_write_reg(sc020, SC020_REG_TEST_PATTERN, (u8)val);

	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops sc020_ctrl_ops = {
	.s_ctrl = sc020_s_ctrl,
};

static const char * const sc020_test_pattern_menu[] = {
	"Disabled",
	"Grayscale Gradient",
};

static int sc020_init_controls(struct sc020 *sc020)
{
	struct v4l2_ctrl_handler *hdl = &sc020->ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *tp;

	v4l2_ctrl_handler_init(hdl, 4);

	link_freq = v4l2_ctrl_new_int_menu(hdl, &sc020_ctrl_ops,
					   V4L2_CID_LINK_FREQ,
					   ARRAY_SIZE(sc020_link_freqs) - 1,
					   SC020_LINK_FREQ_IDX,
					   sc020_link_freqs);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = v4l2_ctrl_new_std(hdl, &sc020_ctrl_ops,
				       V4L2_CID_PIXEL_RATE,
				       0, SC020_PIXEL_RATE, 1,
				       SC020_PIXEL_RATE);
	if (pixel_rate)
		pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* 曝光: 4 ~ VTS-9 行 (表 2-5); 默认 100 行约 0.93ms */
	exposure = v4l2_ctrl_new_std(hdl, &sc020_ctrl_ops,
				     V4L2_CID_EXPOSURE,
				     SC020_EXP_MIN_LINES,
				     SC020_VTS - SC020_EXP_MARGIN_LINES,
				     1, 100);

	/* 测试图案菜单: 0=关闭 1=灰度渐变 (表 2-12) */
	tp = v4l2_ctrl_new_std_menu_items(hdl, &sc020_ctrl_ops,
					  V4L2_CID_TEST_PATTERN,
					  ARRAY_SIZE(sc020_test_pattern_menu) - 1,
					  0, 0, sc020_test_pattern_menu);

	(void)exposure;
	(void)tp;

	if (hdl->error) {
		v4l2_ctrl_handler_free(hdl);
		return hdl->error;
	}

	sc020->sd.ctrl_handler = hdl;
	return 0;
}

/* ------------------------------------------------------------------
 * probe / remove
 * ------------------------------------------------------------------ */
static int sc020_power_get_resources(struct sc020 *sc020)
{
	struct device *dev = &sc020->client->dev;
	bool reg_err;

	sc020->inclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sc020->inclk))
		return dev_err_probe(dev, PTR_ERR(sc020->inclk),
				     "get EXTCLK failed\n");

	/* XSHUTDN: probe 阶段保持复位态 (active-low 逻辑 1) */
	sc020->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sc020->reset))
		return dev_err_probe(dev, PTR_ERR(sc020->reset),
				     "get reset GPIO failed\n");

	/* 三路供电全部 optional: DTS 未绑定时按常电模组处理 */
	sc020->dovdd = devm_regulator_get_optional(dev, "dovdd");
	sc020->avdd = devm_regulator_get_optional(dev, "avdd");
	sc020->dvdd = devm_regulator_get_optional(dev, "dvdd");
	if (IS_ERR(sc020->dovdd) && PTR_ERR(sc020->dovdd) == -ENODEV)
		sc020->dovdd = NULL;
	if (IS_ERR(sc020->avdd) && PTR_ERR(sc020->avdd) == -ENODEV)
		sc020->avdd = NULL;
	if (IS_ERR(sc020->dvdd) && PTR_ERR(sc020->dvdd) == -ENODEV)
		sc020->dvdd = NULL;

	reg_err = IS_ERR(sc020->dovdd) || IS_ERR(sc020->avdd) ||
		  IS_ERR(sc020->dvdd);
	if (reg_err)
		return dev_err_probe(dev, -EIO, "get regulator failed\n");

	return 0;
}

static int sc020_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc020 *sc020;
	struct v4l2_subdev *sd;
	unsigned int id_hi = 0, id_lo = 0;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks I2C_FUNC_I2C\n");

	sc020 = devm_kzalloc(dev, sizeof(*sc020), GFP_KERNEL);
	if (!sc020)
		return -ENOMEM;

	sc020->client = client;
	sc020->powered = false;

	sc020->regmap = devm_regmap_init_i2c(client, &sc020_regmap_config);
	if (IS_ERR(sc020->regmap))
		return dev_err_probe(dev, PTR_ERR(sc020->regmap),
				     "init regmap failed\n");

	ret = sc020_power_get_resources(sc020);
	if (ret)
		return ret;

	/* 上电读 chip ID: 0x3107=0x3A / 0x3108=0x7A -> 0x3A7A (表 1-5) */
	ret = sc020_power_on(sc020);
	if (ret)
		return ret;

	ret  = sc020_read_reg(sc020, SC020_REG_CHIP_ID_HI, &id_hi);
	ret |= sc020_read_reg(sc020, SC020_REG_CHIP_ID_LO, &id_lo);
	if (ret)
		goto err_power_off;

	if (((id_hi << 8) | id_lo) != SC020_CHIP_ID) {
		dev_err(dev, "chip ID mismatch: 0x%02x%02x (expect 0x%04x)\n",
			id_hi, id_lo, SC020_CHIP_ID);
		ret = -ENXIO;
		goto err_power_off;
	}
	dev_info(dev, "SC020HGS detected, chip ID 0x%02x%02x\n", id_hi, id_lo);

	sc020_power_off(sc020);

	/* v4l2 subdev 装配 */
	sd = &sc020->sd;
	v4l2_i2c_subdev_init(sd, client, &sc020_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	sc020->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &sc020->pad);
	if (ret)
		return ret;

	/* 默认 active 格式: Y10_1X10 / 400x400 */
	sc020->fmt.code = SC020HGS_DEFAULT_CODE;
	sc020->fmt.width = SC020_WIDTH;
	sc020->fmt.height = SC020_HEIGHT;
	sc020->fmt.field = V4L2_FIELD_NONE;
	sc020->fmt.colorspace = V4L2_COLORSPACE_RAW;

	ret = sc020_init_controls(sc020);
	if (ret)
		goto err_entity;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_ctrls;

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret)
		goto err_cleanup;

	dev_info(dev, "sc020 probe done: 400x400@240fps %s 1-lane\n", RAW_STR);
	return 0;

err_cleanup:
	v4l2_subdev_cleanup(sd);
err_ctrls:
	v4l2_ctrl_handler_free(&sc020->ctrls);
err_entity:
	media_entity_cleanup(&sd->entity);
	return ret;

err_power_off:
	sc020_power_off(sc020);
	return ret;
}

static void sc020_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc020 *sc020 = to_sc020(sd);

	/* 骨架简化: 假设 remove 前应用已停流; M2 起补 s_stream(0) 兜底 */
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	v4l2_ctrl_handler_free(&sc020->ctrls);
	media_entity_cleanup(&sd->entity);
	sc020_power_off(sc020);
}

static const struct of_device_id sc020_of_match[] = {
	{ .compatible = "smartsens,sc020hgs" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sc020_of_match);

static const struct i2c_device_id sc020_id[] = {
	{ "sc020hgs", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, sc020_id);

static struct i2c_driver sc020_driver = {
	.driver = {
		.name = "sc020hgs",
		.of_match_table = sc020_of_match,
	},
	.probe = sc020_probe,
	.remove = sc020_remove,
	.id_table = sc020_id,
};
module_i2c_driver(sc020_driver);

MODULE_AUTHOR("SC020 project");
MODULE_DESCRIPTION("SmartSens SC020HGS global-shutter RAW Mono sensor driver");
MODULE_LICENSE("GPL");




