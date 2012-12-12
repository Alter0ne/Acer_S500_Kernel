/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_sensor.h"
#include "ov8830.h"
#define SENSOR_NAME "ov8830"
#define PLATFORM_DRIVER_NAME "msm_camera_ov8830"
#define ov8830_obj ov8830_##obj

DEFINE_MUTEX(ov8830_mut);
static struct msm_sensor_ctrl_t ov8830_s_ctrl;

static struct v4l2_subdev_info ov8830_subdev_info[] = {
	{
	.code = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt = 1,
	.order = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array ov8830_init_conf[] = {
	{&ov8830_recommend_settings[0],
	ARRAY_SIZE(ov8830_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ov8830_confs[] = {
	{&ov8830_snap_settings[0],
	ARRAY_SIZE(ov8830_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov8830_prev_settings[0],
	ARRAY_SIZE(ov8830_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t ov8830_dimensions[] = {
/*OV 3280x2464,24fps*/
	{
		.x_output = 0xCD0,
		.y_output = 0x9A0,
		.line_length_pclk = 0xE18,
		.frame_length_lines = 0x9C4,
		.vt_pixel_clk = 216000000,
		.op_pixel_clk = 216000000,
		.binning_factor = 1,
	},
/*OV 2592x1944,30fps*/
	{
		.x_output = 0xA20,
		.y_output = 0x798,
		.line_length_pclk = 0xE18,
		.frame_length_lines = 0x7CB,
		.vt_pixel_clk = 216000000,
		.op_pixel_clk = 216000000,
		.binning_factor = 1,
	},
};

static struct msm_camera_csid_vc_cfg ov8830_cid_cfg[] = {
	{0, CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params ov8830_csi_params = {
	.csid_params = {
		.lane_assign = 0xe4,
		.lane_cnt = 4,
		.lut_params = {
			.num_cid = 2,
			.vc_cfg = ov8830_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 4,
		.settle_cnt = 0x14,
	},
};

static struct msm_camera_csi2_params *ov8830_csi_params_array[] = {
	&ov8830_csi_params,
	&ov8830_csi_params,
};

static struct msm_sensor_output_reg_addr_t ov8830_reg_addr = {
	.x_output = 0x3808,
	.y_output = 0x380a,
	.line_length_pclk = 0x380c,
	.frame_length_lines = 0x380e,
};

static struct msm_sensor_id_info_t ov8830_id_info = {
	.sensor_id_reg_addr = 0x300A,
	.sensor_id = 0x8830,
};

static struct msm_sensor_exp_gain_info_t ov8830_exp_gain_info = {
	.coarse_int_time_addr = 0x3501,
	.global_gain_addr = 0x3508,
	.vert_offset = 6,
};

static int32_t ov8830_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);
	int_time[0] = line >> 12;
	int_time[1] = line >> 4;
	int_time[2] = line << 4;
	msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr-1,
		&int_time[0], 3);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

static const struct i2c_device_id ov8830_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ov8830_s_ctrl},
	{ }
};

static struct i2c_driver ov8830_i2c_driver = {
	.id_table = ov8830_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8830_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&ov8830_i2c_driver);
}

static struct v4l2_subdev_core_ops ov8830_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov8830_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov8830_subdev_ops = {
	.core = &ov8830_subdev_core_ops,
	.video  = &ov8830_subdev_video_ops,
};

static struct msm_sensor_fn_t ov8830_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = ov8830_write_exp_gain,
	.sensor_write_snapshot_exp_gain = ov8830_write_exp_gain,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
};

static struct msm_sensor_reg_t ov8830_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ov8830_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ov8830_start_settings),
	.stop_stream_conf = ov8830_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ov8830_stop_settings),
	.group_hold_on_conf = ov8830_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ov8830_groupon_settings),
	.group_hold_off_conf = ov8830_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(ov8830_groupoff_settings),
	.init_settings = &ov8830_init_conf[0],
	.init_size = ARRAY_SIZE(ov8830_init_conf),
	.mode_settings = &ov8830_confs[0],
	.output_settings = &ov8830_dimensions[0],
	.num_conf = ARRAY_SIZE(ov8830_confs),
};

static struct msm_sensor_ctrl_t ov8830_s_ctrl = {
	.msm_sensor_reg = &ov8830_regs,
	.sensor_i2c_client = &ov8830_sensor_i2c_client,
	.sensor_i2c_addr = 0x6C,
	.sensor_output_reg_addr = &ov8830_reg_addr,
	.sensor_id_info = &ov8830_id_info,
	.sensor_exp_gain_info = &ov8830_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csi_params = &ov8830_csi_params_array[0],
	.msm_sensor_mutex = &ov8830_mut,
	.sensor_i2c_driver = &ov8830_i2c_driver,
	.sensor_v4l2_subdev_info = ov8830_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8830_subdev_info),
	.sensor_v4l2_subdev_ops = &ov8830_subdev_ops,
	.func_tbl = &ov8830_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivision 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
