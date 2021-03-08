/****************************************************************************
 * boards/arm/stm32h7/portenta-h7/src/stm32_clockconfig.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>
#include <stdint.h>

#include "portenta-h7.h"

#include "stm32_gpio.h"
#include "stm32_rcc.h"

#if defined(CONFIG_STM32H7_CUSTOM_CLOCKCONFIG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   The Portena H7 board has an external oscillator, but it must be turned on
 *   by pulling the OSCEN pin high. The oscillator is 25MHz despite any
 *   documentation stating otherwise - which at the time of writing, the
 *   documentation states it is 27MHz. This has been confirmed by the
 *   Arduino staff.
 *
 ****************************************************************************/

void stm32_board_clockconfig(void)
{
    /* We are in the context of stm32_clockconfig(), after the RCC block has
     * been reset. The peripheral clocks are all off, but we need to turn on
     * the oscillator via pin PH1 before we can get the system running. We must
     * enable the peripheral clock for port H now and set the pin.
     */

    uint32_t regval;
    regval = getreg32(STM32_RCC_AHB4ENR);
    regval |= RCC_AHB4ENR_GPIOHEN;
    putreg32(regval, STM32_RCC_AHB4ENR);
    stm32_configgpio(GPIO_OSCEN);

    stm32_stdclockconfig();
}

#endif /* CONFIG_STM32H7_CUSTOM_CLOCKCONFIG */
