// SPDX-License-Identifier: GPL-2.0
/*
 * Lontium LT7911EXC USB-C/DisplayPort to MIPI CSI-2 bridge
 *
 * The bridge runs its own firmware from flash: it trains the DisplayPort
 * link, converts the video to CSI-2 YUV422 8-bit and reports the detected
 * timings and the audio sample rate through a register page. The host reads
 * that page on the interrupt the firmware raises when the video or the
 * audio appears or disappears.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/v4l2-dv-timings.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

#include <uapi/linux/lt7911exc.h>

#define LT7911EXC_PAGE_SELECT		0xff

#define REG_FW_VERSION			CCI_REG24(0xe081)
#define REG_INT_TYPE			CCI_REG8(0xe084)
#define  INT_VIDEO_OFF			0x00
#define  INT_VIDEO_READY		0x01
#define  INT_AUDIO_OFF			0x02
#define  INT_AUDIO_READY		0x03
#define REG_PIXEL_CLOCK			CCI_REG24(0xe085)	/* kHz */
#define REG_H_TOTAL			CCI_REG16(0xe088)
#define REG_V_TOTAL			CCI_REG16(0xe08a)
#define REG_H_ACTIVE			CCI_REG16(0xe08c)
#define REG_V_ACTIVE			CCI_REG16(0xe08e)
#define REG_AUDIO_RATE			CCI_REG8(0xe0ad)	/* kHz */

#define LT7911EXC_LANES			4

static const struct v4l2_dv_timings_cap lt7911exc_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with CLANG */
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(160, 3840, 120, 2160, 25000000, 600000000,
			     V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			     V4L2_DV_BT_STD_CVT,
			     V4L2_DV_BT_CAP_PROGRESSIVE |
			     V4L2_DV_BT_CAP_CUSTOM |
			     V4L2_DV_BT_CAP_REDUCED_BLANKING)
};

static const struct regmap_range_cfg lt7911exc_ranges[] = {
	{
		.name = "register_range",
		.range_min = 0,
		.range_max = 0xffff,
		.selector_reg = LT7911EXC_PAGE_SELECT,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt7911exc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = lt7911exc_ranges,
	.num_ranges = ARRAY_SIZE(lt7911exc_ranges),
};

static const struct v4l2_ctrl_config lt7911exc_ctrl_audio_rate = {
	.id = LT7911EXC_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt7911exc_ctrl_audio_present = {
	.id = LT7911EXC_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

struct lt7911exc {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *audio_rate;
	struct v4l2_ctrl *audio_present;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	/* Detected input timings; zero width when there is no signal. */
	struct v4l2_dv_timings detected;
	struct v4l2_dv_timings timings;
};

static const struct v4l2_event lt7911exc_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static struct lt7911exc *to_lt7911exc(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt7911exc, sd);
}

static void lt7911exc_read_timings(struct lt7911exc *lt)
{
	struct v4l2_bt_timings *bt = &lt->detected.bt;
	u64 pclk, htotal, vtotal, width, height;
	int ret = 0;

	cci_read(lt->regmap, REG_PIXEL_CLOCK, &pclk, &ret);
	cci_read(lt->regmap, REG_H_TOTAL, &htotal, &ret);
	cci_read(lt->regmap, REG_V_TOTAL, &vtotal, &ret);
	cci_read(lt->regmap, REG_H_ACTIVE, &width, &ret);
	cci_read(lt->regmap, REG_V_ACTIVE, &height, &ret);

	memset(&lt->detected, 0, sizeof(lt->detected));
	lt->detected.type = V4L2_DV_BT_656_1120;
	if (ret || !pclk || !width || !height || htotal < width ||
	    vtotal < height)
		return;

	bt->width = width;
	bt->height = height;
	bt->hsync = htotal - width;
	bt->vsync = vtotal - height;
	bt->pixelclock = pclk * 1000;
	bt->interlaced = V4L2_DV_PROGRESSIVE;
}

static u32 lt7911exc_fps(const struct v4l2_bt_timings *bt)
{
	u32 total = V4L2_DV_BT_FRAME_WIDTH(bt) * V4L2_DV_BT_FRAME_HEIGHT(bt);

	return total ? div_u64(bt->pixelclock, total) : 0;
}

static s64 lt7911exc_pixel_rate(const struct v4l2_bt_timings *bt)
{
	/* 16 bits per pixel spread over the data lanes. */
	return div_u64((u64)bt->width * bt->height * lt7911exc_fps(bt) * 16,
		       LT7911EXC_LANES) ?: 1;
}

/* The firmware reports the rate in whole kHz. */
static u32 lt7911exc_audio_hz(u8 khz)
{
	switch (khz) {
	case 44:
		return 44100;
	case 88:
		return 88200;
	case 176:
		return 176400;
	default:
		return khz * 1000;
	}
}

static void lt7911exc_update_audio(struct lt7911exc *lt, bool present)
{
	u64 khz = 0;
	u32 rate = 0;

	if (present && !cci_read(lt->regmap, REG_AUDIO_RATE, &khz, NULL))
		rate = lt7911exc_audio_hz(khz);

	v4l2_ctrl_s_ctrl(lt->audio_rate, rate);
	v4l2_ctrl_s_ctrl(lt->audio_present, rate != 0);
}

static void lt7911exc_fill_format(struct lt7911exc *lt,
				  struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = lt->detected.bt.width;
	fmt->height = lt->detected.bt.height;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_REC709;
}

static int lt7911exc_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *fmt)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	s64 rate;

	lt7911exc_fill_format(lt, &fmt->format);
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	rate = lt7911exc_pixel_rate(&lt->detected.bt);
	__v4l2_ctrl_modify_range(lt->pixel_rate, rate, rate, 1, rate);

	return 0;
}

static int lt7911exc_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}

static int lt7911exc_s_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (!v4l2_valid_dv_timings(timings, &lt7911exc_timings_cap, NULL, NULL))
		return -ERANGE;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	/* The firmware follows the source: only the detected mode is valid. */
	if (!v4l2_match_dv_timings(&lt->detected, timings, 0, false))
		ret = -EINVAL;
	else
		lt->timings = *timings;
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int lt7911exc_g_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	*timings = lt->timings;
	v4l2_subdev_unlock_state(state);

	return 0;
}

static int lt7911exc_query_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				      struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	*timings = lt->detected;
	if (!timings->bt.width)
		ret = -ENOLINK;
	else if (!v4l2_valid_dv_timings(timings, &lt7911exc_timings_cap,
					NULL, NULL))
		ret = -ERANGE;
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int lt7911exc_enum_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &lt7911exc_timings_cap,
					NULL, NULL);
}

static int lt7911exc_dv_timings_cap(struct v4l2_subdev *sd,
				    struct v4l2_dv_timings_cap *cap)
{
	*cap = lt7911exc_timings_cap;

	return 0;
}

static int lt7911exc_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				     struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = LT7911EXC_LANES;

	return 0;
}

static int lt7911exc_enable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state, u32 pad,
				    u64 streams_mask)
{
	return 0;
}

static int lt7911exc_disable_streams(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state, u32 pad,
				     u64 streams_mask)
{
	return 0;
}

static int lt7911exc_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = state ? V4L2_SUBDEV_FORMAT_TRY :
				 V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	return lt7911exc_set_format(sd, state, &fmt);
}

static const struct v4l2_subdev_core_ops lt7911exc_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops lt7911exc_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops lt7911exc_pad_ops = {
	.set_fmt = lt7911exc_set_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.enum_mbus_code = lt7911exc_enum_mbus_code,
	.enable_streams = lt7911exc_enable_streams,
	.disable_streams = lt7911exc_disable_streams,
	.s_dv_timings = lt7911exc_s_dv_timings,
	.g_dv_timings = lt7911exc_g_dv_timings,
	.query_dv_timings = lt7911exc_query_dv_timings,
	.enum_dv_timings = lt7911exc_enum_dv_timings,
	.dv_timings_cap = lt7911exc_dv_timings_cap,
	.get_mbus_config = lt7911exc_get_mbus_config,
};

static const struct v4l2_subdev_ops lt7911exc_subdev_ops = {
	.core = &lt7911exc_core_ops,
	.video = &lt7911exc_video_ops,
	.pad = &lt7911exc_pad_ops,
};

static const struct v4l2_subdev_internal_ops lt7911exc_internal_ops = {
	.init_state = lt7911exc_init_state,
};

static const struct media_entity_operations lt7911exc_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static void lt7911exc_update(struct lt7911exc *lt)
{
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_ACTIVE };
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(&lt->sd);
	lt7911exc_read_timings(lt);
	lt->timings = lt->detected;
	lt7911exc_set_format(&lt->sd, state, &fmt);
	v4l2_subdev_unlock_state(state);

	dev_info(lt->sd.dev, "input %ux%u@%u\n", lt->detected.bt.width,
		 lt->detected.bt.height, lt7911exc_fps(&lt->detected.bt));
	v4l2_subdev_notify_event(&lt->sd, &lt7911exc_ev_source_change);
}

static irqreturn_t lt7911exc_irq(int irq, void *data)
{
	struct lt7911exc *lt = data;
	u64 type;
	int ret;

	ret = cci_read(lt->regmap, REG_INT_TYPE, &type, NULL);
	if (ret)
		return IRQ_NONE;

	switch (type) {
	case INT_VIDEO_OFF:
	case INT_VIDEO_READY:
		lt7911exc_update(lt);
		break;
	case INT_AUDIO_OFF:
		lt7911exc_update_audio(lt, false);
		break;
	case INT_AUDIO_READY:
		lt7911exc_update_audio(lt, true);
		break;
	}

	return IRQ_HANDLED;
}

static int lt7911exc_check_endpoint(struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!fwnode)
		return dev_err_probe(dev, -EINVAL, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return dev_err_probe(dev, ret, "invalid endpoint\n");
	if (ep.bus.mipi_csi2.num_data_lanes != LT7911EXC_LANES)
		return dev_err_probe(dev, -EINVAL,
				     "only 4 data lanes are supported\n");

	return 0;
}

static int lt7911exc_init_controls(struct lt7911exc *lt)
{
	struct v4l2_ctrl_handler *hdl = &lt->ctrl_handler;
	s64 rate = lt7911exc_pixel_rate(&lt->detected.bt);

	v4l2_ctrl_handler_init(hdl, 3);
	lt->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, 1,
					   INT_MAX, 1, rate);
	lt->audio_rate = v4l2_ctrl_new_custom(hdl, &lt7911exc_ctrl_audio_rate,
					      NULL);
	lt->audio_present = v4l2_ctrl_new_custom(hdl,
						 &lt7911exc_ctrl_audio_present,
						 NULL);
	if (hdl->error) {
		int ret = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	lt->sd.ctrl_handler = hdl;

	/* The interrupt for audio that started before probe has been missed. */
	lt7911exc_update_audio(lt, true);

	return 0;
}

static void lt7911exc_cleanup(void *data)
{
	struct lt7911exc *lt = data;

	v4l2_subdev_cleanup(&lt->sd);
	media_entity_cleanup(&lt->sd.entity);
	v4l2_ctrl_handler_free(&lt->ctrl_handler);
}

static int lt7911exc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lt7911exc *lt;
	u64 version;
	int ret;

	lt = devm_kzalloc(dev, sizeof(*lt), GFP_KERNEL);
	if (!lt)
		return -ENOMEM;

	lt->regmap = devm_regmap_init_i2c(client, &lt7911exc_regmap_config);
	if (IS_ERR(lt->regmap))
		return dev_err_probe(dev, PTR_ERR(lt->regmap),
				     "failed to init regmap\n");

	ret = lt7911exc_check_endpoint(dev);
	if (ret)
		return ret;

	/*
	 * Leave the bridge running: resetting it drops the DisplayPort link,
	 * and the source does not always retrain it.
	 */
	lt->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lt->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(lt->reset_gpio),
				     "failed to get reset gpio\n");

	ret = cci_read(lt->regmap, REG_FW_VERSION, &version, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "no response from the bridge\n");

	v4l2_i2c_subdev_init(&lt->sd, client, &lt7911exc_subdev_ops);
	lt->sd.internal_ops = &lt7911exc_internal_ops;
	lt->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	lt->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	lt->sd.entity.ops = &lt7911exc_entity_ops;
	lt->pad.flags = MEDIA_PAD_FL_SOURCE;

	lt7911exc_read_timings(lt);
	lt->timings = lt->detected;

	ret = lt7911exc_init_controls(lt);
	if (ret)
		return ret;

	ret = media_entity_pads_init(&lt->sd.entity, 1, &lt->pad);
	if (ret) {
		v4l2_ctrl_handler_free(&lt->ctrl_handler);
		return ret;
	}

	ret = v4l2_subdev_init_finalize(&lt->sd);
	if (ret) {
		media_entity_cleanup(&lt->sd.entity);
		v4l2_ctrl_handler_free(&lt->ctrl_handler);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, lt7911exc_cleanup, lt);
	if (ret)
		return ret;

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						lt7911exc_irq, IRQF_ONESHOT,
						dev_name(dev), lt);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request irq\n");
	}

	ret = v4l2_async_register_subdev(&lt->sd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register subdev\n");

	dev_info(dev, "firmware %06llx, input %ux%u@%u, audio %d Hz\n", version,
		 lt->detected.bt.width, lt->detected.bt.height,
		 lt7911exc_fps(&lt->detected.bt),
		 v4l2_ctrl_g_ctrl(lt->audio_rate));

	return 0;
}

static void lt7911exc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
}

static const struct of_device_id lt7911exc_of_match[] = {
	{ .compatible = "lontium,lt7911exc" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt7911exc_of_match);

static struct i2c_driver lt7911exc_driver = {
	.driver = {
		.name = "lt7911exc",
		.of_match_table = lt7911exc_of_match,
	},
	.probe = lt7911exc_probe,
	.remove = lt7911exc_remove,
};
module_i2c_driver(lt7911exc_driver);

MODULE_DESCRIPTION("Lontium LT7911EXC USB-C/DP to MIPI CSI-2 bridge driver");
MODULE_LICENSE("GPL");
