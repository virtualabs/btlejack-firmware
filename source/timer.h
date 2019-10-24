/**
 * NRF51822 accurate timer library from blessed BLE stack
 * (https://github.com/pauloborges/blessed/)
 *
 * This timer is far more accurate for a BLE stack than mbed's, but I had to
 * remove PWMOUT library compatibility from microbit mbed lib in order to use
 * NRF51822 Timer2.
 **/

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

#define IRQ_PRIORITY_HIGHEST		0
#define IRQ_PRIORITY_HIGH		1
#define IRQ_PRIORITY_MEDIUM		2
#define IRQ_PRIORITY_LOW		3

#define TIMER_SINGLESHOT		0
#define TIMER_REPEATED			1

#define UNUSED(symbol)			((void) symbol)

typedef void (*timer_cb_t) (void);

int16_t timer_init(void);
int16_t timer_create(uint8_t type);
int16_t timer_destroy(int16_t id);
int16_t timer_start(int16_t id, uint32_t us, timer_cb_t cb);
void timer_start_no_cb(void);
int16_t timer_stop(int16_t id);
void timer_stop_no_cb(void);
uint32_t timer_get_remaining_us(int16_t id);
