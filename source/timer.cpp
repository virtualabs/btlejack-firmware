/**
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2013 Paulo B. de Oliveira Filho <pauloborgesfilho@gmail.com>
 *  Copyright (c) 2013 Claudio Takahasi <claudio.takahasi@gmail.com>
 *  Copyright (c) 2013 João Paulo Rechi Vita <jprvita@gmail.com>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include <string.h>
#include <stdbool.h>

#include <nrf51.h>
#include <nrf51_bitfields.h>
#include "timer.h"

#define TIMER_MILLIS(v)			(v * 1000UL)	/* ms -> us */
#define TIMER_SECONDS(v)		(v * 1000000UL)	/* s -> us */

#define ENOMEM				0x01 /* Out of memory */
#define EINVAL				0x02 /* Invalid argument */
#define EALREADY			0x03 /* Operation already in progress */
#define ENOREADY			0x04 /* Not ready */
#define EBUSY				0x05 /* Device or resource busy */
#define EINTERN				0x06 /* Internal error */


#define HFCLK				16000000UL
#define TIMER_PRESCALER			4		/* 1 MHz */
#define MAX_TIMERS			3

/* The implementation of repeated timers inserts a constant drift in every
 * repetition because of the interruption context switch until initialize
 * TIMER0_IRQHandler(). The define below calculates the number of ticks until
 * the exception handler is executed. */
#define DRIFT_FIX			(1UL << (5 - TIMER_PRESCALER))

#define ROUNDED_DIV(A, B)		(((A) + ((B) / 2)) / (B))
#define POW2(e)				ROUNDED_DIV(2 << e, 2)
#define BIT(n)				(1 << n)

struct timer {
	uint32_t ticks;
	timer_cb_t cb;
	uint8_t enabled:1;
	uint8_t active:1;
	uint8_t type:1;
};

static struct timer timers[MAX_TIMERS];
static uint8_t active = 0;

static __inline uint32_t us2ticks(uint64_t us)
{
	return ROUNDED_DIV(us * HFCLK, TIMER_SECONDS(1)
						* POW2(TIMER_PRESCALER));
}

static __inline uint32_t ticks2us(uint64_t ticks)
{
	return ROUNDED_DIV(ticks * TIMER_SECONDS(1) * POW2(TIMER_PRESCALER),
									HFCLK);
}

static __inline void get_clr_set_masks(uint8_t id, uint32_t *clr, uint32_t *set)
{
	*clr = TIMER_INTENCLR_COMPARE0_Msk << id;
	*set = TIMER_INTENSET_COMPARE0_Msk << id;
}

static __inline uint32_t get_curr_ticks(void)
{
	uint32_t ticks;
	uint32_t cc3;

	cc3 = NRF_TIMER2->CC[3];
	NRF_TIMER2->TASKS_CAPTURE[3] = 1UL;
	ticks = NRF_TIMER2->CC[3];
	NRF_TIMER2->CC[3] = cc3;

	return ticks;
}

static __inline void update_cc(uint8_t id, uint32_t ticks)
{
	uint32_t clr_mask = 0;
	uint32_t set_mask = 0;

	get_clr_set_masks(id, &clr_mask, &set_mask);

	NRF_TIMER2->CC[id] = ticks;
	NRF_TIMER2->INTENSET = set_mask;
}

extern "C" {

	void TIMER2_IRQHandler(void)
	{
		uint32_t curr = get_curr_ticks();
		uint8_t id_mask = 0;
		uint8_t id;

		for (id = 0; id < MAX_TIMERS; id++) {
			if (NRF_TIMER2->EVENTS_COMPARE[id]) {
				NRF_TIMER2->EVENTS_COMPARE[id] = 0UL;

				if (timers[id].active)
					id_mask |= BIT(id);
			}
		}

		for (id = 0; id < MAX_TIMERS; id++) {
			if (id_mask & BIT(id)) {
				if (timers[id].type == TIMER_REPEATED) {
					update_cc(id, curr + timers[id].ticks
									- DRIFT_FIX);
				} else if (timers[id].type == TIMER_SINGLESHOT) {
					timers[id].active = 0;
					active--;

					if (active == 0) {
						NRF_TIMER2->TASKS_STOP = 1UL;
						NRF_TIMER2->TASKS_CLEAR = 1UL;
					}
				}

				timers[id].cb();
			}
		}
	}
}


int16_t timer_init(void)
{
	if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0UL) {
		NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
		while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0UL);
	}

	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_24Bit;
	NRF_TIMER2->PRESCALER = TIMER_PRESCALER;

	NRF_TIMER2->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk
						| TIMER_INTENCLR_COMPARE1_Msk
						| TIMER_INTENCLR_COMPARE2_Msk
						| TIMER_INTENCLR_COMPARE3_Msk;

	NVIC_SetPriority(TIMER2_IRQn, IRQ_PRIORITY_HIGH);
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);

	memset(timers, 0, sizeof(timers));

	return 0;
}

int16_t timer_create(uint8_t type)
{
	int16_t id;

	if (type != TIMER_SINGLESHOT && type != TIMER_REPEATED)
		return -EINVAL;

	for (id = 0; id < MAX_TIMERS; id++) {
		if (!timers[id].enabled)
			goto create;
	}

	return -ENOMEM;

create:
	timers[id].enabled = 1;
	timers[id].active = 0;
	timers[id].type = type;

	return id;
}

int16_t timer_destroy(int16_t id)
{
	if (id<0 || id>2)
		return -EINVAL;

	timers[id].enabled = 0;
	timers[id].active = 0;

	return 0;
}

int16_t timer_start(int16_t id, uint32_t us, timer_cb_t cb)
{
	uint32_t curr = get_curr_ticks();
	uint32_t ticks;

	if (id < 0)
		return -EINVAL;

	if (!timers[id].enabled)
		return -EINVAL;

	if (timers[id].active)
		return -EALREADY;

	ticks = us2ticks(us);

	if (ticks >= 0xFFFFFF)
		return -EINVAL;

	update_cc(id, curr + ticks);

	timers[id].active = 1;
	timers[id].ticks = ticks;
	timers[id].cb = cb;

	if (active == 0) {
		NRF_TIMER2->TASKS_START = 1UL;
	}

	active++;

	return 0;
}

int16_t timer_stop(int16_t id)
{
	uint32_t clr_mask = 0;
	uint32_t set_mask = 0;

	if (id < 0)
		return -EINVAL;

	if (!timers[id].active)
		return -EINVAL;

	get_clr_set_masks(id, &clr_mask, &set_mask);
	NRF_TIMER2->INTENCLR = clr_mask;

	timers[id].active = 0;
	active--;

	if (active == 0) {
		NRF_TIMER2->TASKS_STOP = 1UL;
		NRF_TIMER2->TASKS_CLEAR = 1UL;
	}

	return 0;
}

uint32_t timer_get_remaining_us(int16_t id)
{
	uint32_t ticks = 0;
	uint32_t curr = get_curr_ticks();

	if (!timers[id].active)
		return 0;

	if (NRF_TIMER2->CC[id] > curr)
		ticks = NRF_TIMER2->CC[id] - curr;
	else
		ticks = (0xFFFFFF - curr) + NRF_TIMER2->CC[id];

	return ticks2us(ticks);
}
