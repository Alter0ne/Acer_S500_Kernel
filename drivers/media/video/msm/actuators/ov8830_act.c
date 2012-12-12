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

#include "msm_actuator.h"
#include "msm_camera_i2c.h"

#define OV8830_TOTAL_STEPS_NEAR_TO_FAR 52
DEFINE_MUTEX(ov8830_act_mutex);
static struct msm_actuator_ctrl_t ov8830_act_t;

static struct region_params_t g_regions[] = {
	/* step_bound[0] - macro side boundary
	 * step_bound[1] - infinity side boundary
	 */
	/* Region 1 */
	{
		.step_bound = {10, 0},
		.code_per_step = 40,
	},
	/* Region 2 */
	{
		.step_bound = {52, 10},
		.code_per_step = 4,
	},
};

static uint16_t g_scenario[] = {
	/* MOVE_NEAR and MOVE_FAR dir*/
	OV8830_TOTAL_STEPS_NEAR_TO_FAR,
};

static struct damping_params_t g_damping[] = {
	/* MOVE_NEAR Dir */
	/* Scene 1 => Damping params */
	{
		.damping_step = 40,
		.damping_delay = 0,
	},
};

static struct damping_t g_damping_params[] = {
	/* MOVE_NEAR and MOVE_FAR dir */
	/* Region 1 */
	{
		.ringing_params = g_damping,
	},
	/* Region 2 */
	{
		.ringing_params = g_damping,
	},
};

static int32_t ov8830_wrapper_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, void *params)
{
	int32_t rc = -1;
	uint8_t AD5823_data[2];
	AD5823_data[0] = ((next_lens_position & 0x300) >> 8);//Reg 0x4: VCM_CODE_MSB
	AD5823_data[1] = (next_lens_position & 0xFF);//Reg 0x5: VCM_CODE_LSB

	LINFO("%s AD5823 Reg4= 0x%x;Reg5=0x%x\n",__func__,AD5823_data[0],AD5823_data[1]);
	rc = msm_camera_i2c_write_seq(&a_ctrl->i2c_client,
			     0x4, &AD5823_data[0], 2);

	if (rc < 0) {
		pr_err("%s fail\n",__func__);
	}
	return 0;
}

static int32_t ov8830_act_init_focus(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc;
	uint8_t AD5823_data[2];
	LINFO("%s called\n",__func__);

	AD5823_data[0] = 0x3F;//Reg 0x2: MODE
	AD5823_data[1] = 0x4;//Reg 0x3: VCM_MOVE_TIME

	LINFO("%s AD5823 Reg2= 0x%x;Reg3=0x%x\n",__func__,AD5823_data[0],AD5823_data[1]);
	rc = msm_camera_i2c_write_seq(&a_ctrl->i2c_client,
			     0x2, &AD5823_data[0], 2);

	if (rc < 0) {
		pr_err("%s fail\n",__func__);
	}

	/* Initialize to infinity */
	rc = a_ctrl->func_tbl.actuator_i2c_write(a_ctrl, 0x8, NULL);
	a_ctrl->curr_step_pos = 0;
	return rc;
}

static const struct i2c_device_id ov8830_act_i2c_id[] = {
	{"ov8830_act", (kernel_ulong_t)&ov8830_act_t},
	{ }
};

static int ov8830_act_config(
	void __user *argp)
{
	LINFO("%s called\n", __func__);
	return (int) msm_actuator_config(&ov8830_act_t, argp);
}

static int ov8830_i2c_add_driver_table(
	void)
{
	LINFO("%s called\n", __func__);
	return (int) msm_actuator_init_table(&ov8830_act_t);
}

static struct i2c_driver ov8830_act_i2c_driver = {
	.id_table = ov8830_act_i2c_id,
	.probe  = msm_actuator_i2c_probe,
	.remove = __exit_p(ov8830_act_i2c_remove),
	.driver = {
		.name = "ov8830_act",
	},
};

static int __init ov8830_i2c_add_driver(
	void)
{
	LINFO("%s called\n", __func__);
	return i2c_add_driver(ov8830_act_t.i2c_driver);
}

static struct v4l2_subdev_core_ops ov8830_act_subdev_core_ops;

static struct v4l2_subdev_ops ov8830_act_subdev_ops = {
	.core = &ov8830_act_subdev_core_ops,
};

static int32_t ov8830_act_create_subdevice(
	void *act_info,
	void *sdev)
{
	struct msm_actuator_info *info = (struct msm_actuator_info *)act_info;
	LINFO("%s called\n", __func__);

	return (int) msm_actuator_create_subdevice(&ov8830_act_t,
		(struct i2c_board_info const *)info->board_info,
		(struct v4l2_subdev *)sdev);
}

static int ov8830_act_power_down(void *act_info)
{
	return (int) msm_actuator_af_power_down(&ov8830_act_t);
}

static struct msm_actuator_ctrl_t ov8830_act_t = {
	.i2c_driver = &ov8830_act_i2c_driver,
	.i2c_addr = 0x18,
	.act_v4l2_subdev_ops = &ov8830_act_subdev_ops,
	.actuator_ext_ctrl = {
		.a_init_table = ov8830_i2c_add_driver_table,
		.a_create_subdevice = ov8830_act_create_subdevice,
		.a_config = ov8830_act_config,
		.a_power_down = ov8830_act_power_down,
	},

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	},

	.set_info = {
		.total_steps = OV8830_TOTAL_STEPS_NEAR_TO_FAR,
	},

	.curr_step_pos = 0,
	.curr_region_index = 0,
	.initial_code = 0x8,
	.actuator_mutex = &ov8830_act_mutex,

	.func_tbl = {
		.actuator_init_table = msm_actuator_init_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = ov8830_act_init_focus,
		.actuator_i2c_write = ov8830_wrapper_i2c_write,
	},

	.get_info = {
		.focal_length_num = 46,
		.focal_length_den = 10,
		.f_number_num = 265,
		.f_number_den = 100,
		.f_pix_num = 14,
		.f_pix_den = 10,
		.total_f_dist_num = 197681,
		.total_f_dist_den = 1000,
	},

	/* Initialize scenario */
	.ringing_scenario[MOVE_NEAR] = g_scenario,
	.scenario_size[MOVE_NEAR] = ARRAY_SIZE(g_scenario),
	.ringing_scenario[MOVE_FAR] = g_scenario,
	.scenario_size[MOVE_FAR] = ARRAY_SIZE(g_scenario),

	/* Initialize region params */
	.region_params = g_regions,
	.region_size = ARRAY_SIZE(g_regions),

	/* Initialize damping params */
	.damping[MOVE_NEAR] = g_damping_params,
	.damping[MOVE_FAR] = g_damping_params,
};

subsys_initcall(ov8830_i2c_add_driver);
MODULE_DESCRIPTION("OV8830 actuator");
MODULE_LICENSE("GPL v2");
