/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_himax.h"
#include "mdp4.h"
#include "../../../arch/arm/mach-msm/board-acer-8960.h"

//#define VENDOR_SUGGEST_COMMAND

static struct mipi_dsi_panel_platform_data *mipi_himax_pdata;

extern int acer_boot_mode;

static struct dsi_buf himax_tx_buf;
static struct dsi_buf himax_rx_buf;
static int mipi_himax_lcd_init(void);
static int cabc_is_on;
static int gpio_reset;
static int forbid_cabc;

static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
static char set_te_on[2] = {0x35, 0x00}; /* DTYPE_DCS_WRITE1 */
static char set_ext_cmd[4] = {0xb9, 0xff, 0x83, 0x92}; /* DTYPE_DCS_LWRITE */

static char set_display[13] = { /* DTYPE_DCS_LWRITE */
	0xb2, 0x0f, 0xc8, 0x05,
	0x0f, 0x08, 0x84, 0x00,
	0xff, 0x05, 0x0f, 0x04,
	0x20};

static char set_mode[2] = {0xc2, 0x08}; /*default command mode */

/* 3 lanes */
static char set_num_of_lanes[3] = {0xba, 0x12, 0x83}; /* DTYPE_DCS_LWRITE */
static char blanking_area[3] = {0xc7, 0x00, 0x40}; /* DTYPE_DCS_LWRITE */
static char set_panel[2] = {0xcc, 0x08}; /* DTYPE_DCS_WRITE1 */
static char set_eq_ltps[2] = {0xd4, 0x0c}; /* DTYPE_DCS_WRITE1 */

static char led_pwm1[2] = {0x51, 0x00};	/* DTYPE_DCS_WRITE1 */
static char led_pwm_freq[3] = {0xc9, 0x2f, 0x00};
static char set_cabc_on[2] = {0x53, 0x24};	/* turn on backlight */
static char set_cabc_mode[2] = {0x55, 0x0};	/* 0:cabc off, 1:ui mode, 2:still image, 3:moving image */
static char read_ic_cut[2] = {0xc4, 0x00}; /* DTYPE_DCS_READ */
static char backlight_off[2] = {0x53, 0x0};	/* DTYPE_DCS_WRITE1 */

static struct dsi_cmd_desc read_ic_version[] = {
	{DTYPE_DCS_READ, 1, 0, 0, 5,
		sizeof(read_ic_cut), read_ic_cut},
};
static struct dsi_cmd_desc himax_cmd_backlight_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1), led_pwm1},
};

#ifdef VENDOR_SUGGEST_COMMAND
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char set_power[14] = { /* DTYPE_DCS_LWRITE */
	0xb1, 0x7c, 0x00, 0x44,
	0xa5, 0x00, 0x0d, 0x0d,
	0x12, 0x1a, 0x3f, 0x3f,
	0x42, 0x72};
static char ic_bias[5] = {0xbf, 0x05, 0x60, 0x82, 0x00};
static char led_pwm2[2] = {0x53, 0x24}; /* DTYPE_DCS_WRITE1 */
static char led_pwm3[2] = {0x55, 0x00}; /* DTYPE_DCS_WRITE1 */
static char set_ltps[22] = { /* DTYPE_DCS_LWRITE */
	0xd5, 0x00, 0x08, 0x08,
	0x00, 0x44, 0x55, 0x66,
	0x77, 0xcc, 0xcc, 0xcc,
	0xcc, 0x00, 0x77, 0x66,
	0x55, 0x44, 0xcc, 0xcc,
	0xcc, 0xcc};

static char set_video_cyc[21] = { /* DTYPE_DCS_LWRITE */
	0xd8, 0x00, 0x00, 0x04,
	0x00, 0xa0, 0x04, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x06, 0x07, 0x07,
	0x00};
static char set_cyc[21] = { /* DTYPE_DCS_LWRITE */
	0xb4, 0x00, 0x00, 0x05,
	0x00, 0xa0, 0x05, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x06, 0x07, 0x07,
	0x00};

/* gamma set to 2.2 */
static char gamma_r[35] = {
	0xe0, 0x01, 0x10, 0x16, 0x26,
	0x23, 0x32, 0x21, 0x3b, 0x05,
	0x0b, 0x0e, 0x11, 0x14, 0x11,
	0x12, 0x16, 0x14, 0x01, 0x10,
	0x16, 0x2c, 0x2a, 0x32, 0x21,
	0x3b, 0x05, 0x0b, 0x0e, 0x11,
	0x14, 0x11, 0x12, 0x16, 0x14,
};

static char gamma_g[35] = {
	0xe1, 0x01, 0x0e, 0x14, 0x2c,
	0x2a, 0x32, 0x20, 0x3e, 0x05,
	0x0b, 0x0e, 0x11, 0x14, 0x11,
	0x12, 0x16, 0x14, 0x01, 0x0e,
	0x14, 0x2c, 0x2a, 0x32, 0x20,
	0x3e, 0x05, 0x0b, 0x0e, 0x11,
	0x14, 0x11, 0x12, 0x16, 0x14,
};

static char gamma_b[35] = {
	0xe2, 0x01, 0x0b, 0x0e, 0x30,
	0x2a, 0x32, 0x1c, 0x3c, 0x06,
	0x0b, 0x0e, 0x12, 0x14, 0x12,
	0x13, 0x15, 0x1b, 0x01, 0x0b,
	0x0e, 0x30, 0x2a, 0x32, 0x1c,
	0x3c, 0x06, 0x0b, 0x0e, 0x12,
	0x14, 0x12, 0x13, 0x15, 0x1b,
};

static struct dsi_cmd_desc himax_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 150,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_ext_cmd), set_ext_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_te_on), set_te_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_cabc_bl), set_cabc_bl},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_cabc_on), set_cabc_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_cabc_mode), set_cabc_mode},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(ic_bias), ic_bias},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_power), set_power},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_display), set_display},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_cyc), set_cyc},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_mode), set_mode},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(blanking_area), blanking_area},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_panel), set_panel},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_eq_ltps), set_eq_ltps},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_ltps), set_ltps},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_video_cyc), set_video_cyc},
#ifdef SET_GAMMA
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(gamma_r), gamma_r},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(gamma_g), gamma_g},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(gamma_b), gamma_b},
#endif
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm2), led_pwm2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(led_pwm_freq), led_pwm_freq},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm3), led_pwm3},
	{DTYPE_DCS_WRITE, 1, 0, 0, 150,
		sizeof(display_on), display_on},
};
#else
static struct dsi_cmd_desc himax_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(set_ext_cmd), set_ext_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_te_on), set_te_on},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(set_display), set_display},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_mode), set_mode},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(blanking_area), blanking_area},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_panel), set_panel},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_eq_ltps), set_eq_ltps},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(led_pwm_freq), led_pwm_freq},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(display_on), display_on},
};
#endif

static struct dsi_cmd_desc himax_cabc_on_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_cabc_on), set_cabc_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_cabc_mode), set_cabc_mode},
};

static struct dsi_cmd_desc himax_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(backlight_off), backlight_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc himax_fast_on_cmd1[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(exit_sleep), exit_sleep},
};

static struct dsi_cmd_desc himax_fast_on_cmd2[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(set_ext_cmd), set_ext_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_te_on), set_te_on},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(set_display), set_display},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_mode), set_mode},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(blanking_area), blanking_area},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_panel), set_panel},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_eq_ltps), set_eq_ltps},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(led_pwm_freq), led_pwm_freq},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(display_on), display_on},
};
static struct msm_fb_data_type *mfd_ptr;
static unsigned long panel_off_time;
static unsigned long panel_on_time;

#define PANEL_SUSPEND	0
#define PANEL_SLEEP_OUT	1
#define PANEL_INIT_DONE	2
static int panel_init_stage;
static void panel_init_part2(struct msm_fb_data_type *mfd);
extern int mipi_dsi_on_part2(struct msm_fb_data_type *mfd);
extern int mdp_on_part2(struct msm_fb_data_type *mfd);

static void wait_at_least(unsigned long start, unsigned int wait_ms)
{
	unsigned int tmp;

	tmp = jiffies_to_msecs(jiffies - start);
	if ((tmp > 0) && (tmp < wait_ms)) {
		usleep_range((wait_ms - tmp) * 1000, (wait_ms - tmp) * 1000);
	} else if (tmp < 0)
		usleep_range(wait_ms * 1000, wait_ms * 1000);
}

/* If resume reason is power button, the resume flow:
 *  msm_fb_blank_sub -> mdp_on part1 -> mipi_dsi_on part1
 *  -> panel sleep_out command -> msm_fb_blank_sub ->
 *  panel resume commands -> mipi_dsi_on_part2 -> mdp_on part2
 * */
int wait_panel_ready(struct msm_fb_data_type *mfd)
{
	if (!mfd) {
		pr_err("%s:mfd null pointer!\n", __func__);
		return 0;
	}

	if (panel_init_stage == PANEL_SLEEP_OUT) {
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		mutex_lock(&mfd->dma->ov_mutex);
		panel_init_part2(mfd);
		mipi_dsi_on_part2(mfd);
		mutex_unlock(&mfd->dma->ov_mutex);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
		mdp_on_part2(mfd);

		panel_init_stage = PANEL_INIT_DONE;
		return 1;
	}
	return 0;
}

void panel_need_wait(void)
{
	/*the fisrt frame: at least 40ms after disply_on command */
	wait_at_least(panel_on_time, 40);
}

static void panel_init_part2(struct msm_fb_data_type *mfd)
{
	/* should be called after sleep_out command at least 120ms */
	wait_at_least(panel_on_time, 120);

	mipi_dsi_cmds_tx(mfd, &himax_tx_buf, himax_fast_on_cmd2,
			ARRAY_SIZE(himax_fast_on_cmd2));

	mipi_dsi_cmd_bta_sw_trigger(); /* clean up ack_err_status */
	panel_on_time = jiffies;
	pr_info("Panel initialize part2 done!\n");

}

static int mipi_himax_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	static bool first_init = true;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mfd_ptr = mfd;
	mipi  = &mfd->panel_info.mipi;

	gpio_direction_output(gpio_reset, 0);
	if (!panel_off_time ||
		unlikely(jiffies_to_msecs(jiffies - panel_off_time) < 10)) {
		usleep_range(10 * 1000, 10 * 1000);
	}
	gpio_direction_output(gpio_reset, 1);
	usleep_range(10 * 1000, 10 * 1000);

	if (!mfd->fast_resume_enable ||
		(acer_boot_mode != NORMAL_BOOT) || first_init) {

		mipi_dsi_cmds_tx(mfd, &himax_tx_buf, himax_on_cmds,
			ARRAY_SIZE(himax_on_cmds));

		panel_on_time = jiffies;
		panel_init_stage = PANEL_INIT_DONE;
		first_init = false;

		mipi_dsi_cmd_bta_sw_trigger(); /* clean up ack_err_status */
		pr_info("Panel initialize done!\n");
	} else {
		mipi_dsi_cmds_tx(mfd, &himax_tx_buf, himax_fast_on_cmd1,
			ARRAY_SIZE(himax_fast_on_cmd1));

		panel_on_time = jiffies;
		panel_init_stage = PANEL_SLEEP_OUT;

		pr_info("Panel initialize part1 done!\n");
	}
	return 0;
}

static int mipi_himax_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd, &himax_tx_buf, himax_display_off_cmds,
			ARRAY_SIZE(himax_display_off_cmds));

	panel_init_stage = PANEL_SUSPEND;
	cabc_is_on = 0;
	gpio_direction_output(gpio_reset, 0);
	usleep_range(8000, 8000);
	panel_off_time = jiffies;
	return 0;
}

static uint32 mipi_himax_read_id(struct msm_fb_data_type *mfd)
{
	struct dsi_buf *rp, *tp;
	uint32 *lp;
	struct dsi_cmd_desc *cmd = read_ic_version;
	tp = &himax_tx_buf;
	rp = &himax_rx_buf;

	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 1);
	lp = (uint32 *)rp->data;

	pr_debug("%s: DATA=%x", __func__, *lp);

	return (*lp & 0xf00) >> 8;
}

static void turn_on_cabc(bool on, struct msm_fb_data_type *mfd)
{
	static int himax_ic_cut;

	if (!mfd) {
		pr_err("%s: get wrong parameters\n", __func__);
		return;
	}
	if (!himax_ic_cut) {
		himax_ic_cut = mipi_himax_read_id(mfd);
		pr_info("%s: himax chip IC cut:%d", __func__, himax_ic_cut);
		if (himax_ic_cut >= 5) {
			set_cabc_mode[1] = 3; //Moving Image mode
			pr_info("CABC mode: Moving mode.");
		} else {
			forbid_cabc = 1;
			pr_info("CABC mode: None.");
		}
	}

	if (on)
		mipi_dsi_cmds_tx(mfd, &himax_tx_buf, himax_cabc_on_cmds,
			ARRAY_SIZE(himax_cabc_on_cmds));
}

static void mipi_himax_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;

	mipi  = &mfd->panel_info.mipi;

	mutex_lock(&mfd->dma->bl_mutex);
	mutex_lock(&mfd->dma->ov_mutex);
	mutex_unlock(&mfd->dma->bl_mutex);
	if (mdp4_overlay_dsi_state_get() == ST_DSI_SUSPEND) {
		mutex_unlock(&mfd->dma->ov_mutex);
		return;
	}

	/* mdp4_dsi_cmd_busy_wait: will turn on dsi clock also */
	mdp4_dsi_cmd_dma_busy_wait(mfd);
	mdp4_dsi_blt_dmap_busy_wait(mfd);
	mipi_dsi_mdp_busy_wait(mfd);

	if (!cabc_is_on) {
		pr_info("[BL] bl level = %d \n",mfd->bl_level);
		turn_on_cabc(true, mfd);
		cabc_is_on = 1;
	}

	led_pwm1[1] = (unsigned char)(mfd->bl_level);
	mipi_dsi_cmds_tx(mfd, &himax_tx_buf, himax_cmd_backlight_cmds,
			ARRAY_SIZE(himax_cmd_backlight_cmds));
	mutex_unlock(&mfd->dma->ov_mutex);
	return;
}

ssize_t cabc_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (forbid_cabc) {
		pr_info("forbid cabc adjusting.");
		return count;
	}

	if ((val < 4) && (val >= 0)) {
		set_cabc_mode[1] = val;
		if (mfd_ptr) {
			pr_info("set cabc mode to %d\n", set_cabc_mode[1]);
			cabc_is_on = 0;
			mipi_himax_set_backlight(mfd_ptr);
		}
	}
	return count;
}
ssize_t cabc_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", set_cabc_mode[1]);
}
static struct device_attribute dev_info_attrs =
	__ATTR(cabc_mode, S_IRUGO | S_IWUSR, cabc_mode_show, cabc_mode_store);

static int __devinit mipi_himax_lcd_probe(struct platform_device *pdev)
{
	int ret;

	if (pdev->id == 0) {
		mipi_himax_pdata = pdev->dev.platform_data;
		if (mipi_himax_pdata->gpio) {
			gpio_reset = mipi_himax_pdata->gpio[0];
			ret = gpio_request(gpio_reset, "panel_reset");
			if (ret) {
				pr_err("%s: gpio_request failed on pin [%d] (rc=%d)\n",
						__func__, gpio_reset, ret);
				return -EFAULT;
			}
			gpio_direction_output(gpio_reset, 1);
		}

		if (sysfs_create_file(&pdev->dev.kobj, &dev_info_attrs.attr))
			pr_err("%s:create sysfs error!", __func__);
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_himax_lcd_probe,
	.driver = {
		.name   = "mipi_himax",
	},
};

static struct msm_fb_panel_data himax_panel_data = {
	.on		= mipi_himax_lcd_on,
	.off		= mipi_himax_lcd_off,
	.set_backlight = mipi_himax_set_backlight,
};

static int ch_used[3];

int mipi_himax_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_himax", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	himax_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &himax_panel_data,
		sizeof(himax_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_himax_lcd_init(void)
{
	mipi_dsi_buf_alloc(&himax_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&himax_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}
module_init(mipi_himax_lcd_init);
