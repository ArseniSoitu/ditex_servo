/*
 * SystemTimer.cpp
 *
 *  Created on: Nov 14, 2017
 *      Author: Arseniy Soytu
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include "drivers/SystemTimer.h"

#define RELOAD_VALUE 800

volatile uint64_t tick = 0;

/*
 *
 * 8MHz - base clock
 * 8000000 / 800 = 100 uS for every timer isr
 *
 */
void SystemTimerInit(void)
{
	rcc_periph_clock_enable(RCC_TIM2);

	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_reset(TIM2);
	timer_set_prescaler(TIM2, 0);
	timer_set_period(TIM2, RELOAD_VALUE - 1);
	timer_enable_preload(TIM2);
	timer_enable_counter(TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_generate_event(TIM2, TIM_EGR_UG);
	timer_enable_irq(TIM2, TIM_DIER_UIE);
}

/*
 *
 * tick += 100 (uS)
 *
 */
void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_UIF)) {

		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_UIF);

		tick += (uint64_t)(1000000 / (8000000 / RELOAD_VALUE));
	}
}

uint64_t GetCurrentTime(void)
{
	uint64_t roughTick;
	uint32_t timerTicks;
	uint64_t currentTick;

	do {
		roughTick = tick;
		timerTicks = timer_get_counter(TIM2);
		currentTick = (1000000ULL * timerTicks) / 8000000ULL;
	} while (roughTick != tick);

	return roughTick + (uint64_t)currentTick;
}
