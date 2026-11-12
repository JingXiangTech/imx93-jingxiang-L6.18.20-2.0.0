// Driver for the sc020hgs external camera ISP from smart sense
// reference from ap1302.c
// 2026 11 10 Jingxiang tech

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/media-entity.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define Enable_DEBUG
#ifdef Enable_DEBUG
#define dbg(...) printk(__VA_ARGS__)
#else
#define dbg(...)
#endif

#define SC020_WIDTH			400
#define SC020_HEIGHT		400
#define SC020_HTS			600	/* 0x320C/D: 400 有效 + 200 消隐 */
#define SC020_VTS			450	/* 0x320E/F: 400 有效 + 50 消隐 */

#ifdef SUPPORT_10BIT
#define SC020HGS_DEFAULT_CODE MEDIA_BUS_FMT_Y10_1X10
/* 帧率 = FPCLK / (HTS x VTS) = 64.8M / 270000 = 240fps */
#define SC020_PIXEL_RATE		64800000ULL
/* V4L2 link freq = 每 lane 线速率的一半(DDR): 648Mbps / 2 */
// #define SC020_LINK_FREQ_648MBPS		324000000ULL
#define SC020_LINK_FREQ		324000000ULL
#else
#define SC020HGS_DEFAULT_CODE MEDIA_BUS_FMT_SBGGR8_1X8
#define SC020_PIXEL_RATE 52800000ULL
#define SC020_LINK_FREQ 264000000ULL
#endif

#define SC020_REG_CHIP_ID_HI 0x3107 // default val is 0x3a
#define SC020_REG_CHIP_ID_LO 0x3108 // default val is 0x7a
#define SC020_CHIP_ID 0x3a7a




static const s64 sc020_link_freqs[] = {
	// SC020_LINK_FREQ_648MBPS,
	SC020_LINK_FREQ,
};

#define SC020_LINK_FREQ_IDX		0

struct sc020_device {
	struct i2c_client *client;
	struct regmap *regmap;
	struct clk *clock;
	struct gpio_desc *reset_gpio;
	// struct regulator *avdd;
	// struct regulator *dovdd;
	// struct regulator *dvdd;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_mbus_framefmt fmt;

	// unsigned int index;

	// struct device_node *of_node;
	struct device *dev; // will assigned by clinet->dev
	// unsigned int num_supplies;
	// struct regulator_bulk_data *supplies;

	// struct v4l2_subdev sd;
	// struct media_pad pad;
	bool powered;
};

static const struct regmap_config sc020_regmap_config = {
	.reg_bits		= 16,
	.val_bits		= 8,
	.max_register		= 0xffff,
	.reg_format_endian	= REGMAP_ENDIAN_BIG,
};

static inline struct sc020_device *to_sc020(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc020_device, sd);
}

// static const struct sc020_format_info supported_video_formats[] = {
// 	{
// 		.code = MEDIA_BUS_FMT_UYVY8_1X16,
// 		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF
// 			 | AP1302_PREVIEW_OUT_FMT_FST_YUV_422,
// 		.data_type = MIPI_CSI2_DT_YUV422_8B,
// 	}, {
// 		.code = MEDIA_BUS_FMT_UYYVYY8_0_5X24,
// 		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF
// 			 | AP1302_PREVIEW_OUT_FMT_FST_YUV_420,
// 		.data_type = MIPI_CSI2_DT_YUV420_8B,
// 	},
// };
static int sc020_write_reg(struct sc020_device *sc020, u16 reg, u8 val)
{
	int ret;

	ret = regmap_write(sc020->regmap, reg, val);
	if (ret)
		dev_err(&sc020->client->dev, "write 0x%04x failed: %d\n", reg, ret);
	return ret;
}

static int sc020_read_reg(struct sc020_device *sc020, u16 reg, unsigned int *val)
{
	dbg("sc020_read_reg: reg=0x%04x\n", reg);
	return regmap_read(sc020->regmap, reg, val);
}

static int sc020_power_get_resources(struct sc020_device *sc020)
{
	struct device *dev = sc020->dev;
	// bool reg_err;
	dbg("sc020_power_get_resources");
	sc020->clock = devm_clk_get(dev, NULL);
	if (IS_ERR(sc020->clock))
		return dev_err_probe(dev, PTR_ERR(sc020->clock),
				     "get ext clock failed\n");

	sc020->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sc020->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sc020->reset_gpio), "get reset GPIO failed\n");

	// sc020->dovdd = devm_regulator_get_optional(dev, "dovdd");
	// sc020->avdd = devm_regulator_get_optional(dev, "avdd");
	// sc020->dvdd = devm_regulator_get_optional(dev, "dvdd");
	// if (IS_ERR(sc020->dovdd) && PTR_ERR(sc020->dovdd) == -ENODEV)
	// 	sc020->dovdd = NULL;
	// if (IS_ERR(sc020->avdd) && PTR_ERR(sc020->avdd) == -ENODEV)
	// 	sc020->avdd = NULL;
	// if (IS_ERR(sc020->dvdd) && PTR_ERR(sc020->dvdd) == -ENODEV)
	// 	sc020->dvdd = NULL;

	// reg_err = IS_ERR(sc020->dovdd) || IS_ERR(sc020->avdd) ||
	// 	  IS_ERR(sc020->dvdd);
	// if (reg_err)
	// 	return dev_err_probe(dev, -EIO, "get regulator failed\n");

	return 0;
}

#if 0
static int sc020_parse_of(struct sc020_device *sc020)
{
	struct device_node *sensors;
	struct device_node *node;
	struct fwnode_handle *ep;
	unsigned int num_sensors = 0;
	const char *model;
	unsigned int i;
	int ret;

	/* Clock */
	sc020->clock = devm_clk_get(sc020->dev, NULL);
	if (IS_ERR(sc020->clock)) {
		dev_err(sc020->dev, "Failed to get clock: %ld\n",
			PTR_ERR(sc020->clock));
		return PTR_ERR(sc020->clock);
	}

	/* GPIOs */
	sc020->reset_gpio = devm_gpiod_get(sc020->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sc020->reset_gpio)) {
		dev_err(sc020->dev, "Can't get reset GPIO: %ld\n", PTR_ERR(sc020->reset_gpio));
		return PTR_ERR(sc020->reset_gpio);
	}

	// ap1302->standby_gpio = devm_gpiod_get_optional(ap1302->dev, "standby",
	// 					       GPIOD_OUT_LOW);
	// if (IS_ERR(ap1302->standby_gpio)) {
	// 	dev_err(ap1302->dev, "Can't get standby GPIO: %ld\n",
	// 		PTR_ERR(ap1302->standby_gpio));
	// 	return PTR_ERR(ap1302->standby_gpio);
	// }

	// ap1302->isp_en_gpio = devm_gpiod_get_optional(ap1302->dev, "isp_en",
	// 					       GPIOD_OUT_HIGH);
	// if (IS_ERR(ap1302->isp_en_gpio)) {
	// 	dev_err(ap1302->dev, "Can't get ISP enable GPIO: %ld\n",
	// 		PTR_ERR(ap1302->isp_en_gpio));
	// 	return PTR_ERR(ap1302->isp_en_gpio);
	// }
	// gpiod_set_value_cansleep(ap1302->isp_en_gpio, 1);

	// has issue?
	// ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(ap1302->dev),
	// 				     AP1302_PAD_SOURCE, 0,
	// 				     FWNODE_GRAPH_ENDPOINT_NEXT);
	// if (!ep) {
	// 	dev_err(ap1302->dev, "no sink port found");
	// 	return -EINVAL;
	// }

	ap1302->bus_cfg.bus_type = V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &ap1302->bus_cfg);
	if (ret < 0) {
		dev_err(ap1302->dev, "Failed to parse bus configuration\n");
		return ret;
	}

	/* Sensors */
	sensors = of_get_child_by_name(dev_of_node(ap1302->dev), "sensors");
	if (!sensors) {
		dev_err(ap1302->dev, "'sensors' child node not found\n");
		return -EINVAL;
	}

	// ret = of_property_read_string(sensors, "onnn,model", &model);
	// if (ret < 0) {
	// 	/*
	// 	 * If no sensor is connected, we can still support operation
	// 	 * with the test pattern generator.
	// 	 */
	// 	ap1302->sensor_info = &ap1302_sensor_info_tpg;
	// 	ap1302->width_factor = 1;
	// 	ret = 0;
	// 	goto done;
	// }

	// for (i = 0; i < ARRAY_SIZE(ap1302_sensor_info); ++i) {
	// 	const struct ap1302_sensor_info *info =
	// 		&ap1302_sensor_info[i];

	// 	if (!strcmp(info->model, model)) {
	// 		ap1302->sensor_info = info;
	// 		break;
	// 	}
	// }

	// if (!ap1302->sensor_info) {
	// 	dev_warn(ap1302->dev, "Unsupported sensor model %s\n", model);
	// 	ret = -EINVAL;
	// 	goto done;
	// }

	// for_each_child_of_node(sensors, node) {
	// 	if (of_node_name_eq(node, "sensor")) {
	// 		if (!ap1302_sensor_parse_of(ap1302, node))
	// 			num_sensors++;
	// 	}
	// }

	// if (!num_sensors) {
	// 	dev_err(ap1302->dev, "No sensor found\n");
	// 	ret = -EINVAL;
	// 	goto done;
	// }

	// ap1302->width_factor = num_sensors;

// done:
// 	of_node_put(sensors);
	return ret;
}
#endif

static int sc020_s_stream(struct v4l2_subdev *sd, int enable)
{
// 	struct sc020 *sc020 = to_sc020(sd);
// 	int ret;

// 	if (enable) {
// 		ret = sc020_power_on(sc020);
// 		if (ret)
// 			return ret;

// 		ret = sc020_load_init_table(sc020);
// 		if (ret)
// 			goto err_power;

// 		/* 睡眠寄存器即流开关: 1=唤醒出流（方案 §3 条4, 0x2100 实证路线） */
// 		ret = sc020_write_reg(sc020, SC020_REG_SLEEP, 0x01);
// 		if (ret)
// 			goto err_power;
// 		return 0;

// err_power:
// 		sc020_power_off(sc020);
// 		return ret;
// 	}

// 	sc020_write_reg(sc020, SC020_REG_SLEEP, 0x00);
// 	sc020_power_off(sc020);
	return 0;
}

static int sc020_power_on(struct sc020_device *sc020)
{
	struct device *dev = sc020->dev;
	int ret;
	dbg("sc020_power_on");
	if (sc020->powered)
		return 0;

	ret = clk_prepare_enable(sc020->clock);
	if (ret) {
		dev_err(dev, "enable clock failed: %d\n", ret);
		return ret;
	}
	// test
	gpiod_set_value_cansleep(sc020->reset_gpio, 1);
	fsleep(5000);

	gpiod_set_value_cansleep(sc020->reset_gpio, 0);
	fsleep(5000);
	sc020->powered = true;
	return 0;
}

static void sc020_power_off(struct sc020_device *sc020)
{
	dbg("sc020_power_off");
	if (!sc020->powered)
		return;

	// sc020_write_reg(sc020, SC020_REG_SLEEP, 0x00);

	// gpiod_set_value_cansleep(sc020->reset_gpio, 1);

	// fsleep(1000);

	// if (!IS_ERR_OR_NULL(sc020->avdd))
	// 	regulator_disable(sc020->avdd);
	// if (!IS_ERR_OR_NULL(sc020->dvdd))
	// 	regulator_disable(sc020->dvdd);
	// if (!IS_ERR_OR_NULL(sc020->dovdd))
	// 	regulator_disable(sc020->dovdd);
	// clk_disable_unprepare(sc020->inclk);

	sc020->powered = false;
}

static int sc020_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct sc020_device *sc020 = to_sc020(sd);
	struct v4l2_mbus_framefmt *mf;

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
	struct sc020_device *sc020 = to_sc020(sd);

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

static int sc020_s_ctrl(struct v4l2_ctrl *ctrl)
{
	// struct sc020 *sc020 = container_of(ctrl->handler, struct sc020, ctrls);
	// unsigned int val;
	// int ret;
	dbg("sc020_s_ctrl");
	// if (!sc020->powered)
	// 	return 0;

	// switch (ctrl->id) {
	// case V4L2_CID_EXPOSURE:
	// 	val = ctrl->val;
	// 	/* {3e00[3:0], 3e01[7:0], 3e02[7:4]} (表 2-5; 3e02 仅高 4 位, V0.7) */
	// 	ret  = sc020_write_reg(sc020, SC020_REG_EXP_HI, (val >> 12) & 0x0f);
	// 	ret |= sc020_write_reg(sc020, SC020_REG_EXP_MID, (val >> 4) & 0xff);
	// 	ret |= sc020_write_reg(sc020, SC020_REG_EXP_LO, (val << 4) & 0xf0);
	// 	return ret;

	// case V4L2_CID_TEST_PATTERN:
	// 	/* 0x4501 bit3: 1=灰度渐变 (表 2-12), 读改写保其余位 */
	// 	ret = sc020_read_reg(sc020, SC020_REG_TEST_PATTERN, &val);
	// 	if (ret)
	// 		return ret;
	// 	if (ctrl->val)
	// 		val |= 0x08;
	// 	else
	// 		val &= ~0x08;
	// 	return sc020_write_reg(sc020, SC020_REG_TEST_PATTERN, (u8)val);

	// default:
	// 	return -EINVAL;
	// }
	return 0;
}

static const struct v4l2_ctrl_ops sc020_ctrl_ops = {
	.s_ctrl = sc020_s_ctrl,
};

static int sc020_init_controls(struct sc020_device *sc020)
{
	struct v4l2_ctrl_handler *ctrl_hdl = &sc020->ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	// struct v4l2_ctrl *exposure;
	// struct v4l2_ctrl *tp;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdl, 2);
	if (ret)
		return ret;

	link_freq = v4l2_ctrl_new_int_menu(ctrl_hdl, &sc020_ctrl_ops,
					   V4L2_CID_LINK_FREQ,
					   ARRAY_SIZE(sc020_link_freqs) - 1,
					   SC020_LINK_FREQ_IDX,
					   sc020_link_freqs);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = v4l2_ctrl_new_std(ctrl_hdl, &sc020_ctrl_ops,
				       V4L2_CID_PIXEL_RATE,
				       0, SC020_PIXEL_RATE, 1,
				       SC020_PIXEL_RATE);
	if (pixel_rate)
		pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// // set exposure
	// exposure = v4l2_ctrl_new_std(ctrl_hdl, &sc020_ctrl_ops,
	// 			     V4L2_CID_EXPOSURE,
	// 			     SC020_EXP_MIN_LINES,
	// 			     SC020_VTS - SC020_EXP_MARGIN_LINES,
	// 			     1, 100);

	// // set test pattern
	// tp = v4l2_ctrl_new_std_menu(ctrl_hdl, &sc020_ctrl_ops,
	// 			    V4L2_CID_TEST_PATTERN,
	// 			    1, 0, 0);

	// (void)exposure;
	// (void)tp;

	if (ctrl_hdl->error) {
		ret = ctrl_hdl->error;
		v4l2_ctrl_handler_free(ctrl_hdl);
		return ret;
	}

	sc020->sd.ctrl_handler = ctrl_hdl;

	return 0;
}

static int sc020_probe(struct i2c_client *client)
{
	struct sc020_device *sc020;
	struct v4l2_subdev *sd;
	unsigned int id_hi = 0, id_lo = 0;
	int ret;

	dbg("sc020_probe");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(&client->dev, -EOPNOTSUPP,
				     "adapter lacks I2C_FUNC_I2C\n");

	sc020 = devm_kzalloc(&client->dev, sizeof(*sc020), GFP_KERNEL);
	if (!sc020)
		return -ENOMEM;

    sc020->dev = &client->dev;
	sc020->client = client;
	sc020->powered = false;

	sc020->regmap = devm_regmap_init_i2c(client, &sc020_regmap_config);
	if (IS_ERR(sc020->regmap))
		return dev_err_probe(sc020->dev, PTR_ERR(sc020->regmap),
				     "init regmap failed\n");

	ret = sc020_power_get_resources(sc020);
	if (ret)
		return ret;

	// Get chip ID: 0x3107=0x3A / 0x3108=0x7A -> 0x3A7A (Table 1-5)
	ret = sc020_power_on(sc020);
	if (ret)
		return ret;

	ret  = sc020_read_reg(sc020, SC020_REG_CHIP_ID_HI, &id_hi);
	dbg("sc020_probe: chip ID hi=0x%02x\n", id_hi);
	ret |= sc020_read_reg(sc020, SC020_REG_CHIP_ID_LO, &id_lo);
	dbg("sc020_probe: chip ID lo=0x%02x\n", id_lo);
	if (ret)
		goto err_power_off;

	if (((id_hi << 8) | id_lo) != SC020_CHIP_ID) {
		dev_err(sc020->dev, "chip ID mismatch: 0x%02x%02x (expect 0x%04x)\n",
			id_hi, id_lo, SC020_CHIP_ID);
		ret = -ENXIO;
		goto err_power_off;
	}
	dev_info(sc020->dev, "SC020HGS detected, chip ID 0x%02x%02x\n", id_hi, id_lo);
	dbg("sc020_probe: chip ID 0x%02x%02x\n", id_hi, id_lo);
	// sc020_power_off(sc020);

	/* v4l2 subdev 装配 */
	sd = &sc020->sd;
	v4l2_i2c_subdev_init(sd, client, &sc020_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	sc020->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &sc020->pad);
	if (ret)
		return ret;

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

	dev_info(sc020->dev, "sc020 probe done: 400x400@240fps RAW10 1-lane\n");
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
	struct sc020_device *sc020 = to_sc020(sd);

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

MODULE_AUTHOR("Jingxiang Tech <support@jingxiang.com>");
MODULE_DESCRIPTION("SmartSens SC020HGS global-shutter RAW Mono sensor driver");
MODULE_LICENSE("GPL");