/*
 *   Generic LIPO battery charger
 *
 *   Copyright (c) 2010-2011 Michael Buesch <mb@bu3sch.de>
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation; either version 2
 *   of the License, or (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

#define DEBUG

#include "lipocharge.h"

#include <linux/slab.h>


/* Hysteresis constants */
#define CURRENT_HYST		30 /* mA */
#define VOLTAGE_HYST		10 /* mV */

/* Threshold constants */
#define FINISH_CURRENT_PERCENT	3


/* Returns the requested first-stage charge current in mA */
static inline unsigned int get_stage1_charge_current(struct lipocharge *c)
{
	/* current = (capacity * C) */
	return c->capacity * c->rate / 1000;
}

void lipocharge_init(struct lipocharge *c, struct device *dev)
{
	c->dev = dev;
	c->state = LIPO_IDLE;
}

void lipocharge_exit(struct lipocharge *c)
{
	c->state = LIPO_IDLE;
}

int lipocharge_start(struct lipocharge *c)
{
	int err;

	if (c->state != LIPO_IDLE)
		return -EBUSY;
	if (!c->set_current_pwm || !c->emergency)
		return -EINVAL;
	if (!c->top_voltage || c->top_voltage > 4200)
		return -EINVAL;

	c->active_duty_cycle = 0;
	err = c->set_current_pwm(c, c->active_duty_cycle);
	if (err)
		return err;
	c->state = LIPO_FIRST_STAGE;

	return 0;
}

void lipocharge_stop(struct lipocharge *c)
{
	if (c->state == LIPO_IDLE)
		return;
	c->state = LIPO_IDLE;
}

static int lipocharge_increase_current(struct lipocharge *c,
				       unsigned int inc_permille)
{
	int old_pwm, new_pwm;

	if (c->active_duty_cycle >= c->duty_cycle_max)
		return 0;

	old_pwm = c->active_duty_cycle;
	new_pwm = old_pwm + (c->duty_cycle_max * inc_permille / 1000);
	new_pwm = min(new_pwm, (int)c->duty_cycle_max);
	c->active_duty_cycle = new_pwm;

	dev_dbg(c->dev, "lipo: Increasing duty_cycle by "
		"%u permille (0x%02X -> 0x%02X)",
		inc_permille, old_pwm, new_pwm);

	return c->set_current_pwm(c, c->active_duty_cycle);
}

static int lipocharge_decrease_current(struct lipocharge *c,
				       unsigned int dec_permille)
{
	int old_pwm, new_pwm;

	if (c->active_duty_cycle <= 0)
		return 0;

	old_pwm = c->active_duty_cycle;
	new_pwm = old_pwm - (c->duty_cycle_max * dec_permille / 1000);
	new_pwm = max(0, new_pwm);
	c->active_duty_cycle = new_pwm;

	dev_dbg(c->dev, "lipo: Decreasing duty_cycle by "
		"%u permille (0x%02X -> 0x%02X)",
		dec_permille, old_pwm, new_pwm);

	return c->set_current_pwm(c, c->active_duty_cycle);
}

/** lipocharge_update_state - Update the charge state
 * @c: The context.
 * @voltage_mV: The measured battery voltage.
 * @current_mA: The measured charge current.
 *		negative -> drain.
 *		positive -> charge.
 * @temp_K: Battery temperature in K.
 *
 * Returns 0 on success, -1 on error.
 * Returns 1, if the charging process is finished.
 */
int lipocharge_update_state(struct lipocharge *c,
			    unsigned int voltage_mV,
			    int current_mA,
			    unsigned int temp_K)
{
	int requested_current, current_diff;
	int err;
	unsigned int permille;

	//TODO temp

restart:
	switch (c->state) {
	case LIPO_IDLE:
		dev_err(c->dev, "%s: called while idle", __func__);
		return -EINVAL;
	case LIPO_FIRST_STAGE:	/* Constant current */
//printk("GOT %u %d %u\n", voltage_mV, current_mA, temp_K);
		if (voltage_mV >= c->top_voltage) {
			/* Float voltage reached.
			 * Switch charger mode to "constant current" */
			c->state = LIPO_SECOND_STAGE;
			dev_dbg(c->dev, "Switched to second charging stage.");
			goto restart;
		}
		/* Float voltage not reached, yet.
		 * Try to get the requested constant current. */
		requested_current = get_stage1_charge_current(c);
		if (current_mA < 0)
			current_mA = 0;
		current_diff = requested_current - current_mA;
		if (abs(requested_current - current_mA) > CURRENT_HYST) {
			if (current_diff > 0) {
				/* Increase current */
				permille = current_diff * 1000 / requested_current;
				permille /= 2;
				err = lipocharge_increase_current(c, permille);
				if (err)
					return err;
			} else {
				/* Decrease current */
				permille = (-current_diff) * 1000 / requested_current;
				permille /= 2;
				err = lipocharge_decrease_current(c, permille);
				if (err)
					return err;
			}
		}
		break;
	case LIPO_SECOND_STAGE:	/* Constant voltage */
		//TODO
		break;
	}

	return 0;
}
