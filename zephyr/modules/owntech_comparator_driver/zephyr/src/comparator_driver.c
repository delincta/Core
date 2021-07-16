/*
 * Copyright (c) 2021 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @author  Clément Foucher <clement.foucher@laas.fr>
 */


// Zephyr
#include <zephyr.h>

// STM32 LL
#include <stm32_ll_comp.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>

// Current file header
#include "comparator_driver.h"


void comparator_init()
{
	_comparator_gpio_init();
	_comparator_comp1_init();
	_comparator_comp3_init();
}


static void _comparator_gpio_init()
{
	// TODO : use Zephyr gpio_pin_configure

	/*
	COMP1 GPIO Configuration
		PA1       ------> COMP1_INP
		PB8-BOOT0 ------> COMP1_OUT
	COMP3 GPIO Configuration
		PC1       ------> COMP3_INP
		PB15      ------> COMP3_OUT
	*/

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

	// Pin A.1 (cmp 1)
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);

	// Pin C.1 (cmp 3)
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
}

static void _comparator_comp1_init()
{
	LL_COMP_ConfigInputs(COMP1, LL_COMP_INPUT_MINUS_DAC1_CH1, LL_COMP_INPUT_PLUS_IO1);
	LL_COMP_SetInputHysteresis(COMP1, LL_COMP_HYSTERESIS_NONE);
	LL_COMP_SetOutputPolarity(COMP1, LL_COMP_OUTPUTPOL_NONINVERTED);
	LL_COMP_SetOutputBlankingSource(COMP1, LL_COMP_BLANKINGSRC_NONE);

	k_busy_wait(LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US);

	LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_21);
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_21);

	LL_COMP_Enable(COMP1);
}

static void _comparator_comp3_init()
{
	LL_COMP_ConfigInputs(COMP3, LL_COMP_INPUT_MINUS_DAC3_CH1, LL_COMP_INPUT_PLUS_IO2);
	LL_COMP_SetInputHysteresis(COMP3, LL_COMP_HYSTERESIS_NONE);
	LL_COMP_SetOutputPolarity(COMP3, LL_COMP_OUTPUTPOL_NONINVERTED);
	LL_COMP_SetOutputBlankingSource(COMP3, LL_COMP_BLANKINGSRC_NONE);

	k_busy_wait(LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US);

	LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_29);
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_29);

	LL_COMP_Enable(COMP3);
}
