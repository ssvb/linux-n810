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
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/stddef.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/usb/musb.h>
#include <sound/tlv320aic3x.h>
#include <linux/input.h>
#include <linux/i2c/lm8323.h>
#include <linux/spi/tsc2005.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/menelaus.h>
#include <mach/irqs.h>
#include <plat/mcspi.h>
#include <plat/onenand.h>
#include <plat/mmc.h>
#include <plat/serial.h>
#include <plat/cbus.h>
#include <plat/gpio-switch.h>
#include <plat/usb.h>

#include "mux.h"

static int slot1_cover_open;
static int slot2_cover_open;
static struct device *mmc_device;

/* We map the FN key as LALT to workaround an X keycode problem.
 * The XKB map needs to be adjusted to support this. */
#define MAP_FN_AS_LEFTALT

static s16 rx44_keymap[LM8323_KEYMAP_SIZE] = {
	[0x01] = KEY_Q,
	[0x02] = KEY_K,
	[0x03] = KEY_O,
	[0x04] = KEY_P,
	[0x05] = KEY_BACKSPACE,
	[0x06] = KEY_A,
	[0x07] = KEY_S,
	[0x08] = KEY_D,
	[0x09] = KEY_F,
	[0x0a] = KEY_G,
	[0x0b] = KEY_H,
	[0x0c] = KEY_J,

	[0x11] = KEY_W,
	[0x12] = KEY_F4,
	[0x13] = KEY_L,
	[0x14] = KEY_APOSTROPHE,
	[0x16] = KEY_Z,
	[0x17] = KEY_X,
	[0x18] = KEY_C,
	[0x19] = KEY_V,
	[0x1a] = KEY_B,
	[0x1b] = KEY_N,
	[0x1c] = KEY_LEFTSHIFT, /* Actually, this is both shift keys */
	[0x1f] = KEY_F7,

	[0x21] = KEY_E,
	[0x22] = KEY_SEMICOLON,
	[0x23] = KEY_MINUS,
	[0x24] = KEY_EQUAL,
#ifdef MAP_FN_AS_LEFTALT
	[0x2b] = KEY_LEFTALT,
#else
	[0x2b] = KEY_FN,
#endif
	[0x2c] = KEY_M,
	[0x2f] = KEY_F8,

	[0x31] = KEY_R,
	[0x32] = KEY_RIGHTCTRL,
	[0x34] = KEY_SPACE,
	[0x35] = KEY_COMMA,
	[0x37] = KEY_UP,
	[0x3c] = KEY_COMPOSE,
	[0x3f] = KEY_F6,

	[0x41] = KEY_T,
	[0x44] = KEY_DOT,
	[0x46] = KEY_RIGHT,
	[0x4f] = KEY_F5,
	[0x51] = KEY_Y,
	[0x53] = KEY_DOWN,
	[0x55] = KEY_ENTER,
	[0x5f] = KEY_ESC,

	[0x61] = KEY_U,
	[0x64] = KEY_LEFT,

	[0x71] = KEY_I,
	[0x75] = KEY_KPENTER,
};

static struct lm8323_platform_data lm8323_pdata = {
	.repeat		= 0, /* Repeat is handled in userspace for now. */
	.keymap		= rx44_keymap,
	.size_x		= 8,
	.size_y		= 12,
	.debounce_time	= 12,
	.active_time	= 500,

	.name		= "Internal keyboard",
	.pwm_names[0] 	= "n810::keyboard",
	.pwm_names[1] 	= "n810::cover",
};

#define OMAP_TAG_NOKIA_BT	0x4e01

struct omap_bluetooth_config {
	u8    chip_type;
	u8    bt_wakeup_gpio;
	u8    host_wakeup_gpio;
	u8    reset_gpio;
	u8    bt_uart;
	u8    bd_addr[6];
	u8    bt_sysclk;
};

static struct platform_device n8x0_bt_device = {
	.name           = "hci_h4p",
	.id             = -1,
	.num_resources  = 0,
};

void __init n8x0_bt_init(void)
{
	const struct omap_bluetooth_config *bt_config;

	bt_config = (void *) omap_get_config(OMAP_TAG_NOKIA_BT,
					     struct omap_bluetooth_config);
	n8x0_bt_device.dev.platform_data = (void *) bt_config;
	if (platform_device_register(&n8x0_bt_device) < 0)
		BUG();
}

#define	RX51_TSC2005_RESET_GPIO	94
#define	RX51_TSC2005_IRQ_GPIO	106

#ifdef CONFIG_TOUCHSCREEN_TSC2005
static struct tsc2005_platform_data tsc2005_config;
static void rx51_tsc2005_set_reset(bool enable)
{
	gpio_set_value(RX51_TSC2005_RESET_GPIO, enable);
}

static struct omap2_mcspi_device_config tsc2005_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
};
#endif

static void __init tsc2005_set_config(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
#ifdef CONFIG_TOUCHSCREEN_TSC2005
		if (strcmp(conf->panel_name, "lph8923") == 0) {
			tsc2005_config.ts_x_plate_ohm = 180;
			tsc2005_config.ts_hw_avg = 0;
			tsc2005_config.ts_ignore_last = 0;
			tsc2005_config.ts_touch_pressure = 1500;
			tsc2005_config.ts_stab_time = 100;
			tsc2005_config.ts_pressure_max = 2048;
			tsc2005_config.ts_pressure_fudge = 2;
			tsc2005_config.ts_x_max = 4096;
			tsc2005_config.ts_x_fudge = 4;
			tsc2005_config.ts_y_max = 4096;
			tsc2005_config.ts_y_fudge = 7;
			tsc2005_config.set_reset = rx51_tsc2005_set_reset;
		} else if (strcmp(conf->panel_name, "ls041y3") == 0) {
			tsc2005_config.ts_x_plate_ohm = 280;
			tsc2005_config.ts_hw_avg = 0;
			tsc2005_config.ts_ignore_last = 0;
			tsc2005_config.ts_touch_pressure = 1500;
			tsc2005_config.ts_stab_time = 1000;
			tsc2005_config.ts_pressure_max = 2048;
			tsc2005_config.ts_pressure_fudge = 2;
			tsc2005_config.ts_x_max = 4096;
			tsc2005_config.ts_x_fudge = 4;
			tsc2005_config.ts_y_max = 4096;
			tsc2005_config.ts_y_fudge = 7;
			tsc2005_config.set_reset = rx51_tsc2005_set_reset;
		} else {
			printk(KERN_ERR "Unknown panel type, set default "
			       "touchscreen configuration\n");
			tsc2005_config.ts_x_plate_ohm = 200;
			tsc2005_config.ts_stab_time = 100;
		}
#endif
	}
}

static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
};

extern struct mipid_platform_data n8x0_mipid_platform_data;

extern void n8x0_mipid_init(void);
extern void n8x0_blizzard_init(void);

struct gpio_switch_input_dev {
	struct input_dev *idev;
	unsigned int swcode;
};

static struct gpio_switch_input_dev *slide_input;
static struct gpio_switch_input_dev *kblock_input;

static void n8x0_gpio_switch_input_notify(struct gpio_switch_input_dev *gdev,
					  int state)
{
	if (gdev) {
		input_report_switch(gdev->idev, gdev->swcode, state);
		input_sync(gdev->idev);
	}
}

static void n8x0_slide_notify(void *data, int state)
{
	n8x0_gpio_switch_input_notify(slide_input, state);
}

static void n8x0_kb_lock_notify(void *data, int state)
{
	n8x0_gpio_switch_input_notify(kblock_input, state);
}

static struct gpio_switch_input_dev * __init gpioswitch_input_init(
			const char *name,
			unsigned int swcode)
{
	struct gpio_switch_input_dev *gdev;
	int err;

	gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);
	if (!gdev)
		goto error;
	gdev->swcode = swcode;

	gdev->idev = input_allocate_device();
	if (!gdev->idev)
		goto err_free;

	gdev->idev->evbit[0] = BIT_MASK(EV_SW);
	gdev->idev->swbit[BIT_WORD(swcode)] = BIT_MASK(swcode);
	gdev->idev->name = name;

	err = input_register_device(gdev->idev);
	if (err)
		goto err_free_idev;

	return gdev;

err_free_idev:
	input_free_device(gdev->idev);
err_free:
	kfree(gdev);
error:
	return NULL;
}

static int __init n8x0_gpio_switches_input_init(void)
{
	slide_input = gpioswitch_input_init("slide", SW_KEYPAD_SLIDE);
	kblock_input = gpioswitch_input_init("kb_lock", SW_LID);
	if (WARN_ON(!slide_input || !kblock_input))
		return -ENODEV;
	return 0;
}
late_initcall(n8x0_gpio_switches_input_init);

static struct omap_gpio_switch n8x0_gpio_switches[] __initdata = {
	{
		.name			= "headphone",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
	}, {
		.name			= "cam_act",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
	}, {
		.name			= "cam_turn",
		.gpio			= -1,
		.debounce_rising	= 100,
		.debounce_falling	= 100,
	}, {
		.name			= "slide",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
		.notify			= n8x0_slide_notify,
	}, {
		.name			= "kb_lock",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
		.notify			= n8x0_kb_lock_notify,
	},
};

static void __init n8x0_gpio_switches_init(void)
{
	/* The switches are actually registered through ATAG mechanism.
	 * This just updates the parameters (thus .gpio is -1) */
	omap_register_gpio_switches(n8x0_gpio_switches,
				    ARRAY_SIZE(n8x0_gpio_switches));
}

#define TUSB6010_ASYNC_CS	1
#define TUSB6010_SYNC_CS	4
#define TUSB6010_GPIO_INT	58
#define TUSB6010_GPIO_ENABLE	0
#define TUSB6010_DMACHAN	0x3f

#ifdef CONFIG_USB_MUSB_TUSB6010
/*
 * Enable or disable power to TUSB6010. When enabling, turn on 3.3 V and
 * 1.5 V voltage regulators of PM companion chip. Companion chip will then
 * provide then PGOOD signal to TUSB6010 which will release it from reset.
 */
static int tusb_set_power(int state)
{
	int i, retval = 0;

	if (state) {
		gpio_set_value(TUSB6010_GPIO_ENABLE, 1);
		msleep(1);

		/* Wait until TUSB6010 pulls INT pin down */
		i = 100;
		while (i && gpio_get_value(TUSB6010_GPIO_INT)) {
			msleep(1);
			i--;
		}

		if (!i) {
			printk(KERN_ERR "tusb: powerup failed\n");
			retval = -ENODEV;
		}
	} else {
		gpio_set_value(TUSB6010_GPIO_ENABLE, 0);
		msleep(10);
	}

	return retval;
}

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.num_eps	= 16,
	.ram_bits	= 12,
};

static struct musb_hdrc_platform_data tusb_data = {
#if defined(CONFIG_USB_MUSB_OTG)
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_PERIPHERAL)
	.mode		= MUSB_PERIPHERAL,
#else /* defined(CONFIG_USB_MUSB_HOST) */
	.mode		= MUSB_HOST,
#endif
	.set_power	= tusb_set_power,
	.min_power	= 25,	/* x2 = 50 mA drawn from VBUS as peripheral */
	.power		= 100,	/* Max 100 mA VBUS for host mode */
	.config		= &musb_config,
};

static struct omap_usb_config n8x0_omap_usb_config __initdata = {
	.otg		= 1,
	.register_host	= 1,
	.register_dev	= 1,
	.hmc_mode	= 16,
	.pins[0]	= 6,
};

static void __init n8x0_usb_init(void)
{
	int ret = 0;
	static char	announce[] __initdata = KERN_INFO "TUSB 6010\n";

	/* PM companion chip power control pin */
	ret = gpio_request(TUSB6010_GPIO_ENABLE, "TUSB6010 enable");
	if (ret != 0) {
		printk(KERN_ERR "Could not get TUSB power GPIO%i\n",
		       TUSB6010_GPIO_ENABLE);
		return;
	}
	gpio_direction_output(TUSB6010_GPIO_ENABLE, 0);

	tusb_set_power(0);

	ret = tusb6010_setup_interface(&tusb_data, TUSB6010_REFCLK_19, 2,
					TUSB6010_ASYNC_CS, TUSB6010_SYNC_CS,
					TUSB6010_GPIO_INT, TUSB6010_DMACHAN);
	if (ret != 0)
		goto err;

	omap2_usbfs_init(&n8x0_omap_usb_config);

	printk(announce);

	return;

err:
	gpio_free(TUSB6010_GPIO_ENABLE);
}
#else

static void __init n8x0_usb_init(void) {}

#endif /*CONFIG_USB_MUSB_TUSB6010 */


static struct omap2_mcspi_device_config p54spi_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
};

static struct spi_board_info n800_spi_board_info[] __initdata = {
	{
		.modalias	= "lcd_mipid",
		.bus_num	= 1,
		.chip_select	= 1,
		.max_speed_hz	= 4000000,
		.controller_data= &mipid_mcspi_config,
		.platform_data	= &n8x0_mipid_platform_data,
	},
	{
		.modalias	= "p54spi",
		.bus_num	= 2,
		.chip_select	= 0,
		.max_speed_hz   = 48000000,
		.controller_data = &p54spi_mcspi_config,
	},
	{
		.modalias	 = "tsc2005",
		.bus_num	 = 1,
		.chip_select	 = 0,
		.irq		 = OMAP_GPIO_IRQ(RX51_TSC2005_IRQ_GPIO),
		.max_speed_hz    = 6000000,
		.controller_data = &tsc2005_mcspi_config,
		.platform_data   = &tsc2005_config,
	},
};

#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
	defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)

static struct mtd_partition onenand_partitions[] = {
	{
		.name           = "bootloader",
		.offset         = 0,
		.size           = 0x20000,
		.mask_flags     = MTD_WRITEABLE,	/* Force read-only */
	},
	{
		.name           = "config",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x60000,
	},
	{
		.name           = "kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x200000,
	},
	{
		.name           = "initfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x400000,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct omap_onenand_platform_data board_onenand_data[] = {
	{
		.cs		= 0,
		.gpio_irq	= 26,
		.parts		= onenand_partitions,
		.nr_parts	= ARRAY_SIZE(onenand_partitions),
		.flags		= ONENAND_SYNC_READ,
	}
};
#endif

#if defined(CONFIG_CBUS) || defined(CONFIG_CBUS_MODULE)

static struct cbus_host_platform_data n8x0_cbus_data = {
	.clk_gpio	= 66,
	.dat_gpio	= 65,
	.sel_gpio	= 64,
};

static struct platform_device n8x0_cbus_device = {
	.name		= "cbus",
	.id		= -1,
	.dev		= {
		.platform_data = &n8x0_cbus_data,
	},
};

static struct resource retu_resource[] = {
	{
		.start	= -EINVAL, /* set later */
		.flags	= IORESOURCE_IRQ,
	},
};

static struct cbus_retu_platform_data n8x0_retu_data = {
	.irq_base	= CBUS_RETU_IRQ_BASE,
	.irq_end	= CBUS_RETU_IRQ_END,
	.devid		= CBUS_RETU_DEVICE_ID,
};

static struct platform_device retu_device = {
	.name		= "retu",
	.id		= -1,
	.resource	= retu_resource,
	.num_resources	= ARRAY_SIZE(retu_resource),
	.dev		= {
		.platform_data = &n8x0_retu_data,
	},
};

static struct resource tahvo_resource[] = {
	{
		.start	= -EINVAL, /* set later */
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device tahvo_device = {
	.name		= "tahvo",
	.id		= -1,
	.resource	= tahvo_resource,
	.num_resources	= ARRAY_SIZE(tahvo_resource),
};

static struct platform_device tahvo_usb_device = {
	.name		= "tahvo-usb",
	.id		= -1,
};

static void __init n8x0_cbus_init(void)
{
	int		ret;

	platform_device_register(&n8x0_cbus_device);

	ret = gpio_request(108, "RETU irq");
	if (ret < 0) {
		pr_err("retu: Unable to reserve IRQ GPIO\n");
		return;
	}

	ret = gpio_direction_input(108);
	if (ret < 0) {
		pr_err("retu: Unable to change gpio direction\n");
		gpio_free(108);
		return;
	}

	set_irq_type(gpio_to_irq(108), IRQ_TYPE_EDGE_RISING);
	retu_resource[0].start = gpio_to_irq(108);
	platform_device_register(&retu_device);

	ret = gpio_request(111, "TAHVO irq");
	if (ret) {
		pr_err("tahvo: Unable to reserve IRQ GPIO\n");
		gpio_free(108);
		return;
	}

	/* Set the pin as input */
	ret = gpio_direction_input(111);
	if (ret) {
		pr_err("tahvo: Unable to change direction\n");
		gpio_free(108);
		gpio_free(111);
		return;
	}

	tahvo_resource[0].start = gpio_to_irq(111);
	platform_device_register(&tahvo_device);
	platform_device_register(&tahvo_usb_device);
}

#else
static inline void __init n8x0_cbus_init(void)
{
}
#endif

#if defined(CONFIG_MENELAUS) &&						\
	(defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE))

/*
 * On both N800 and N810, only the first of the two MMC controllers is in use.
 * The two MMC slots are multiplexed via Menelaus companion chip over I2C.
 * On N800, both slots are powered via Menelaus. On N810, only one of the
 * slots is powered via Menelaus. The N810 EMMC is powered via GPIO.
 *
 * VMMC				slot 1 on both N800 and N810
 * VDCDC3_APE and VMCS2_APE	slot 2 on N800
 * GPIO23 and GPIO9		slot 2 EMMC on N810
 *
 */
#define N8X0_SLOT_SWITCH_GPIO	96
#define N810_EMMC_VSD_GPIO	23
#define N810_EMMC_VIO_GPIO	9

static int n8x0_mmc_switch_slot(struct device *dev, int slot)
{
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Choose slot %d\n", slot + 1);
#endif
	gpio_set_value(N8X0_SLOT_SWITCH_GPIO, slot);
	return 0;
}

static int n8x0_mmc_set_power_menelaus(struct device *dev, int slot,
					int power_on, int vdd)
{
	int mV;

#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d power: %s (vdd %d)\n", slot + 1,
		power_on ? "on" : "off", vdd);
#endif
	if (slot == 0) {
		if (!power_on)
			return menelaus_set_vmmc(0);
		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
		case MMC_VDD_31_32:
			mV = 3100;
			break;
		case MMC_VDD_30_31:
			mV = 3000;
			break;
		case MMC_VDD_28_29:
			mV = 2800;
			break;
		case MMC_VDD_165_195:
			mV = 1850;
			break;
		default:
			BUG();
		}
		return menelaus_set_vmmc(mV);
	} else {
		if (!power_on)
			return menelaus_set_vdcdc(3, 0);
		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
			mV = 3300;
			break;
		case MMC_VDD_30_31:
		case MMC_VDD_29_30:
			mV = 3000;
			break;
		case MMC_VDD_28_29:
		case MMC_VDD_27_28:
			mV = 2800;
			break;
		case MMC_VDD_24_25:
		case MMC_VDD_23_24:
			mV = 2400;
			break;
		case MMC_VDD_22_23:
		case MMC_VDD_21_22:
			mV = 2200;
			break;
		case MMC_VDD_20_21:
			mV = 2000;
			break;
		case MMC_VDD_165_195:
			mV = 1800;
			break;
		default:
			BUG();
		}
		return menelaus_set_vdcdc(3, mV);
	}
	return 0;
}

static void n810_set_power_emmc(struct device *dev,
					 int power_on)
{
	dev_dbg(dev, "Set EMMC power %s\n", power_on ? "on" : "off");

	if (power_on) {
		gpio_set_value(N810_EMMC_VSD_GPIO, 1);
		msleep(1);
		gpio_set_value(N810_EMMC_VIO_GPIO, 1);
		msleep(1);
	} else {
		gpio_set_value(N810_EMMC_VIO_GPIO, 0);
		msleep(50);
		gpio_set_value(N810_EMMC_VSD_GPIO, 0);
		msleep(50);
	}
}

static int n8x0_mmc_set_power(struct device *dev, int slot, int power_on,
			      int vdd)
{
	if (machine_is_nokia_n800() || slot == 0)
		return n8x0_mmc_set_power_menelaus(dev, slot, power_on, vdd);

	n810_set_power_emmc(dev, power_on);

	return 0;
}

static int n8x0_mmc_set_bus_mode(struct device *dev, int slot, int bus_mode)
{
	int r;

	dev_dbg(dev, "Set slot %d bus mode %s\n", slot + 1,
		bus_mode == MMC_BUSMODE_OPENDRAIN ? "open-drain" : "push-pull");
	BUG_ON(slot != 0 && slot != 1);
	slot++;
	switch (bus_mode) {
	case MMC_BUSMODE_OPENDRAIN:
		r = menelaus_set_mmc_opendrain(slot, 1);
		break;
	case MMC_BUSMODE_PUSHPULL:
		r = menelaus_set_mmc_opendrain(slot, 0);
		break;
	default:
		BUG();
	}
	if (r != 0 && printk_ratelimit())
		dev_err(dev, "MMC: unable to set bus mode for slot %d\n",
			slot);
	return r;
}

static int n8x0_mmc_get_cover_state(struct device *dev, int slot)
{
	slot++;
	BUG_ON(slot != 1 && slot != 2);
	if (slot == 1)
		return slot1_cover_open;
	else
		return slot2_cover_open;
}

static void n8x0_mmc_callback(void *data, u8 card_mask)
{
	int bit, *openp, index;

	if (machine_is_nokia_n800()) {
		bit = 1 << 1;
		openp = &slot2_cover_open;
		index = 1;
	} else {
		bit = 1;
		openp = &slot1_cover_open;
		index = 0;
	}

	if (card_mask & bit)
		*openp = 1;
	else
		*openp = 0;

	omap_mmc_notify_cover_event(mmc_device, index, *openp);
}

static int n8x0_mmc_late_init(struct device *dev)
{
	int r, bit, *openp;
	int vs2sel;

	mmc_device = dev;

	r = menelaus_set_slot_sel(1);
	if (r < 0)
		return r;

	if (machine_is_nokia_n800())
		vs2sel = 0;
	else
		vs2sel = 2;

	r = menelaus_set_mmc_slot(2, 0, vs2sel, 1);
	if (r < 0)
		return r;

	n8x0_mmc_set_power(dev, 0, MMC_POWER_ON, 16); /* MMC_VDD_28_29 */
	n8x0_mmc_set_power(dev, 1, MMC_POWER_ON, 16);

	r = menelaus_set_mmc_slot(1, 1, 0, 1);
	if (r < 0)
		return r;
	r = menelaus_set_mmc_slot(2, 1, vs2sel, 1);
	if (r < 0)
		return r;

	r = menelaus_get_slot_pin_states();
	if (r < 0)
		return r;

	if (machine_is_nokia_n800()) {
		bit = 1 << 1;
		openp = &slot2_cover_open;
	} else {
		bit = 1;
		openp = &slot1_cover_open;
		slot2_cover_open = 0;
	}

	/* All slot pin bits seem to be inversed until first switch change */
	if (r == 0xf || r == (0xf & ~bit))
		r = ~r;

	if (r & bit)
		*openp = 1;
	else
		*openp = 0;

	r = menelaus_register_mmc_callback(n8x0_mmc_callback, NULL);

	return r;
}

static void n8x0_mmc_shutdown(struct device *dev)
{
	int vs2sel;

	if (machine_is_nokia_n800())
		vs2sel = 0;
	else
		vs2sel = 2;

	menelaus_set_mmc_slot(1, 0, 0, 0);
	menelaus_set_mmc_slot(2, 0, vs2sel, 0);
}

static void n8x0_mmc_cleanup(struct device *dev)
{
	menelaus_unregister_mmc_callback();

	gpio_free(N8X0_SLOT_SWITCH_GPIO);

	if (machine_is_nokia_n810()) {
		gpio_free(N810_EMMC_VSD_GPIO);
		gpio_free(N810_EMMC_VIO_GPIO);
	}
}

/*
 * MMC controller1 has two slots that are multiplexed via I2C.
 * MMC controller2 is not in use.
 */
static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots			= 2,
	.switch_slot			= n8x0_mmc_switch_slot,
	.init				= n8x0_mmc_late_init,
	.cleanup			= n8x0_mmc_cleanup,
	.shutdown			= n8x0_mmc_shutdown,
	.max_freq			= 24000000,
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wires			= 4,
		.set_power		= n8x0_mmc_set_power,
		.set_bus_mode		= n8x0_mmc_set_bus_mode,
		.get_cover_state	= n8x0_mmc_get_cover_state,
		.ocr_mask		= MMC_VDD_165_195 | MMC_VDD_30_31 |
						MMC_VDD_32_33   | MMC_VDD_33_34,
		.name			= "internal",
	},
	.slots[1] = {
		.set_power		= n8x0_mmc_set_power,
		.set_bus_mode		= n8x0_mmc_set_bus_mode,
		.get_cover_state	= n8x0_mmc_get_cover_state,
		.ocr_mask		= MMC_VDD_165_195 | MMC_VDD_20_21 |
						MMC_VDD_21_22 | MMC_VDD_22_23 |
						MMC_VDD_23_24 | MMC_VDD_24_25 |
						MMC_VDD_27_28 | MMC_VDD_28_29 |
						MMC_VDD_29_30 | MMC_VDD_30_31 |
						MMC_VDD_32_33 | MMC_VDD_33_34,
		.name			= "external",
	},
};

static struct omap_mmc_platform_data *mmc_data[OMAP24XX_NR_MMC];

static void __init n8x0_mmc_init(void)

{
	int err;

	if (machine_is_nokia_n810()) {
		mmc1_data.slots[0].name = "external";

		/*
		 * Some Samsung Movinand chips do not like open-ended
		 * multi-block reads and fall to braind-dead state
		 * while doing so. Reducing the number of blocks in
		 * the transfer or delays in clock disable do not help
		 */
		mmc1_data.slots[1].name = "internal";
		mmc1_data.slots[1].ban_openended = 1;
	}

	err = gpio_request(N8X0_SLOT_SWITCH_GPIO, "MMC slot switch");
	if (err)
		return;

	gpio_direction_output(N8X0_SLOT_SWITCH_GPIO, 0);

	if (machine_is_nokia_n810()) {
		err = gpio_request(N810_EMMC_VSD_GPIO, "MMC slot 2 Vddf");
		if (err) {
			gpio_free(N8X0_SLOT_SWITCH_GPIO);
			return;
		}
		gpio_direction_output(N810_EMMC_VSD_GPIO, 0);

		err = gpio_request(N810_EMMC_VIO_GPIO, "MMC slot 2 Vdd");
		if (err) {
			gpio_free(N8X0_SLOT_SWITCH_GPIO);
			gpio_free(N810_EMMC_VSD_GPIO);
			return;
		}
		gpio_direction_output(N810_EMMC_VIO_GPIO, 0);
	}

	mmc_data[0] = &mmc1_data;
	omap2_init_mmc(mmc_data, OMAP24XX_NR_MMC);
}
#else

void __init n8x0_mmc_init(void)
{
}
#endif	/* CONFIG_MMC_OMAP */

#ifdef CONFIG_MENELAUS

static int n8x0_auto_sleep_regulators(void)
{
	u32 val;
	int ret;

	val = EN_VPLL_SLEEP | EN_VMMC_SLEEP    \
		| EN_VAUX_SLEEP | EN_VIO_SLEEP \
		| EN_VMEM_SLEEP | EN_DC3_SLEEP \
		| EN_VC_SLEEP | EN_DC2_SLEEP;

	ret = menelaus_set_regulator_sleep(1, val);
	if (ret < 0) {
		printk(KERN_ERR "Could not set regulators to sleep on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n8x0_auto_voltage_scale(void)
{
	int ret;

	ret = menelaus_set_vcore_hw(1400, 1050);
	if (ret < 0) {
		printk(KERN_ERR "Could not set VCORE voltage on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n8x0_menelaus_late_init(struct device *dev)
{
	int ret;

	ret = n8x0_auto_voltage_scale();
	if (ret < 0)
		return ret;
	ret = n8x0_auto_sleep_regulators();
	if (ret < 0)
		return ret;
	return 0;
}

#else
static int n8x0_menelaus_late_init(struct device *dev)
{
	return 0;
}
#endif

static struct menelaus_platform_data n8x0_menelaus_platform_data __initdata = {
	.late_init = n8x0_menelaus_late_init,
};

static struct i2c_board_info __initdata n8x0_i2c_board_info_1[] __initdata = {
	{
		I2C_BOARD_INFO("menelaus", 0x72),
		.irq = INT_24XX_SYS_NIRQ,
		.platform_data = &n8x0_menelaus_platform_data,
	},
};

static struct aic3x_pdata n810_aic33_data __initdata = {
	.gpio_reset = 118,
};

static struct i2c_board_info n810_i2c_board_info_2[] __initdata = {
 	{
		I2C_BOARD_INFO("lm8323", 0x45),
		.irq		= OMAP_GPIO_IRQ(109),
		.platform_data	= &lm8323_pdata,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
		.platform_data = &n810_aic33_data,
	},
};

static void __init n8x0_map_io(void)
{
	omap2_set_globals_242x();
	omap242x_map_common_io();
}

static void __init n8x0_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* I2S codec port pins for McBSP block */
	OMAP2420_MUX(EAC_AC_SCLK, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP2420_MUX(EAC_AC_FS, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP2420_MUX(EAC_AC_DIN, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP2420_MUX(EAC_AC_DOUT, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_device_pad serial2_pads[] __initdata = {
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_MUX_MODE0,
		.idle	= OMAP_MUX_MODE3	/* Mux as GPIO for idle */
	},
};

static inline void board_serial_init(void)
{
	struct omap_board_data bdata;

	bdata.flags = 0;
	bdata.pads = NULL;
	bdata.pads_cnt = 0;

	bdata.id = 0;
	omap_serial_init_port(&bdata);

	bdata.id = 1;
	omap_serial_init_port(&bdata);

	bdata.id = 2;
	bdata.pads = serial2_pads;
	bdata.pads_cnt = ARRAY_SIZE(serial2_pads);
	omap_serial_init_port(&bdata);
}

#else

static inline void board_serial_init(void)
{
	omap_serial_init();
}

#endif

static void __init n8x0_init_machine(void)
{
	omap2420_mux_init(board_mux, OMAP_PACKAGE_ZAC);
	n8x0_gpio_switches_init();
	n8x0_cbus_init();
	n8x0_bt_init();

	/* FIXME: add n810 spi devices */
	tsc2005_set_config();
	spi_register_board_info(n800_spi_board_info,
				ARRAY_SIZE(n800_spi_board_info));
	omap_register_i2c_bus(1, 400, n8x0_i2c_board_info_1,
			      ARRAY_SIZE(n8x0_i2c_board_info_1));
	omap_register_i2c_bus(2, 400, NULL, 0);
	if (machine_is_nokia_n810())
		i2c_register_board_info(2, n810_i2c_board_info_2,
					ARRAY_SIZE(n810_i2c_board_info_2));
	board_serial_init();
	n8x0_mipid_init();
	n8x0_blizzard_init();
	gpmc_onenand_init(board_onenand_data);
	n8x0_mmc_init();
	n8x0_usb_init();
}

MACHINE_START(NOKIA_N800, "Nokia N800")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= n8x0_map_io,
	.init_early	= n8x0_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(NOKIA_N810, "Nokia N810")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= n8x0_map_io,
	.init_early	= n8x0_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(NOKIA_N810_WIMAX, "Nokia N810 WiMAX")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= n8x0_map_io,
	.init_early	= n8x0_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END
