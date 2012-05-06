/*
 * linux/arch/arm/mach-omap2/board-n8x0.c
 *
 * Copyright (C) 2005-2009 Nokia Corporation
 * Author: Juha Yrjola <juha.yrjola@nokia.com>
 *
 * Modified from mach-omap2/board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/omapfb.h>

#include <plat/lcd_mipid.h>
#include <plat/blizzard.h>

#include <../drivers/cbus/tahvo.h>

#define N8X0_BLIZZARD_POWERDOWN_GPIO	15

// MIPID LCD Panel

static void mipid_shutdown(struct mipid_platform_data *pdata)
{
	if (pdata->nreset_gpio != -1) {
		pr_info("shutdown LCD\n");
		gpio_set_value(pdata->nreset_gpio, 0);
		msleep(120);
	}
}

static int n8x0_get_backlight_level(struct mipid_platform_data *pdata)
{
	return tahvo_get_backlight_level();
}

static int n8x0_get_max_backlight_level(struct mipid_platform_data *pdata)
{
	return tahvo_get_max_backlight_level();
}

static void n8x0_set_backlight_level(struct mipid_platform_data *pdata, int level)
{
	tahvo_set_backlight_level(level);
}

struct mipid_platform_data n8x0_mipid_platform_data = {
	.shutdown = mipid_shutdown,
	.get_bklight_level = n8x0_get_backlight_level,
	.set_bklight_level = n8x0_set_backlight_level,
	.get_bklight_max = n8x0_get_max_backlight_level,
};

void __init n8x0_mipid_init(void)
{
	const struct omap_lcd_config *conf;
	int err;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		n8x0_mipid_platform_data.nreset_gpio = conf->nreset_gpio;
		n8x0_mipid_platform_data.data_lines = conf->data_lines;
		if (conf->nreset_gpio != -1) {
			err = gpio_request(conf->nreset_gpio, "MIPID nreset");
			if (err) {
				printk(KERN_ERR "N8x0 MIPID failed to request nreset GPIO %d\n",
				       conf->nreset_gpio);
			} else {
				err = gpio_direction_output(conf->nreset_gpio, 1);
				if (err) {
					printk(KERN_ERR "N8x0 MIPID failed to set nreset GPIO %d\n",
					       conf->nreset_gpio);
				}
			}
		}
		printk(KERN_INFO "N8x0 MIPID config loaded");
	}
	else
		printk(KERN_INFO "N8x0 MIPID config not provided");
}


// Epson Blizzard LCD Controller

static struct {
	struct clk *sys_ck;
} blizzard;

static int blizzard_get_clocks(void)
{
	blizzard.sys_ck = clk_get(0, "osc_ck");
	if (IS_ERR(blizzard.sys_ck)) {
		printk(KERN_ERR "can't get Blizzard clock\n");
		return PTR_ERR(blizzard.sys_ck);
	}
	return 0;
}

static unsigned long blizzard_get_clock_rate(struct device *dev)
{
	return clk_get_rate(blizzard.sys_ck);
}

static void blizzard_enable_clocks(int enable)
{
	if (enable)
		clk_enable(blizzard.sys_ck);
	else
		clk_disable(blizzard.sys_ck);
}

static void blizzard_power_up(struct device *dev)
{
	/* Vcore to 1.475V */
	tahvo_set_clear_reg_bits(0x07, 0, 0xf);
	msleep(10);

	blizzard_enable_clocks(1);
	gpio_set_value(N8X0_BLIZZARD_POWERDOWN_GPIO, 1);
}

static void blizzard_power_down(struct device *dev)
{
	gpio_set_value(N8X0_BLIZZARD_POWERDOWN_GPIO, 0);
	blizzard_enable_clocks(0);

	/* Vcore to 1.005V */
	tahvo_set_clear_reg_bits(0x07, 0xf, 0);
}

static struct blizzard_platform_data n8x0_blizzard_data = {
	.power_up	= blizzard_power_up,
	.power_down	= blizzard_power_down,
	.get_clock_rate	= blizzard_get_clock_rate,
	.te_connected	= 1,
};

void __init n8x0_blizzard_init(void)
{
	int r;

	r = gpio_request(N8X0_BLIZZARD_POWERDOWN_GPIO, "Blizzard pd");
	if (r < 0)
	{
		printk(KERN_ERR "Can't get N8x0 Blizzard powerdown GPIO %d\n", N8X0_BLIZZARD_POWERDOWN_GPIO);
		return;
	}
	gpio_direction_output(N8X0_BLIZZARD_POWERDOWN_GPIO, 1);

	blizzard_get_clocks();
	omapfb_set_ctrl_platform_data(&n8x0_blizzard_data);

	printk(KERN_INFO "N8x0 Blizzard initialized");
}
