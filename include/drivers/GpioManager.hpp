/*
 * GpioManager.hpp
 *
 *  Created on: Jan 10, 2018
 *      Author: Arseniy Soytu
 */

#ifndef INCLUDE_DRIVERS_GPIOMANAGER_HPP_
#define INCLUDE_DRIVERS_GPIOMANAGER_HPP_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

class GpioManager {
public:
	~GpioManager() = default;

	static uint32_t gpioToRcc(uint32_t gpio)
	{
		uint32_t gpioRcc;
		gpioRcc = (gpio == GPIOA) ? RCC_GPIOA :
				(gpio == GPIOB) ? RCC_GPIOB :
				(gpio == GPIOC) ? RCC_GPIOC :
				(gpio == GPIOD) ? RCC_GPIOD :
				(gpio == GPIOE) ? RCC_GPIOE :
				(gpio == GPIOF) ? RCC_GPIOF :
				(gpio == GPIOG) ? RCC_GPIOG : 0;
		return gpioRcc;
	}
};



#endif /* INCLUDE_DRIVERS_GPIOMANAGER_HPP_ */
