#ifndef LIPOCHARGE_H_
#define LIPOCHARGE_H_

#include <linux/types.h>
#include <linux/device.h>


#define LIPORATE(a,b)	(((a) * 1000) + ((b) * 100))
#define LIPORATE_p6C	LIPORATE(0,6)	/* 0.6C */

enum lipocharge_state {
	LIPO_IDLE = 0,		/* Not charging */
	LIPO_FIRST_STAGE,	/* Charging: constant current */
	LIPO_SECOND_STAGE,	/* Charging: constant voltage */
};

/** struct lipocharge - A generic LIPO charger
 *
 * @capacity: Battery capacity in mAh.
 * @rate: Charge rate.
 * @top_voltage: Fully charged voltage, in mV.
 * @duty_cycle_max: Max value for duty_cycle.
 *
 * @set_charge_current: Set the charge current PWM duty cycle.
 * @emergency: Something went wrong. Force shutdown.
 */
struct lipocharge {
	unsigned int capacity;
	unsigned int rate;
	unsigned int top_voltage;
	unsigned int duty_cycle_max;

	int (*set_current_pwm)(struct lipocharge *c, unsigned int duty_cycle);
	void (*emergency)(struct lipocharge *c);

	/* internal */
	struct device *dev;
	enum lipocharge_state state;
	unsigned int active_duty_cycle;

	//TODO implement timer to cut power after maximum charge time.
};

void lipocharge_init(struct lipocharge *c, struct device *dev);
void lipocharge_exit(struct lipocharge *c);

int lipocharge_start(struct lipocharge *c);
void lipocharge_stop(struct lipocharge *c);

int lipocharge_update_state(struct lipocharge *c,
			    unsigned int voltage_mV,
			    int current_mA,
			    unsigned int temp_K);

static inline bool lipocharge_is_charging(struct lipocharge *c)
{
	return (c->state != LIPO_IDLE);
}

#endif /* LIPOCHARGE_H_ */
