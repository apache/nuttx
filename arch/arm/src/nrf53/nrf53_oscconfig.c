/****************************************************************************
 * arch/arm/src/nrf53/nrf53_oscconfig.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf53_oscconfig.h"
#include "nrf53_gpio.h"
#include "hardware/nrf53_osc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NRF53_NETCORE
#  error Oscillators configuration availalbe only for the App core
#endif

/* LFXO pins */

#define LFXO_XL1_GPIO_PIN  (GPIO_MCUSEL_PERIP | GPIO_PORT0 | GPIO_PIN(0))
#define LFXO_XL2_GPIO_PIN  (GPIO_MCUSEL_PERIP | GPIO_PORT0 | GPIO_PIN(1))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_oscconfig
 ****************************************************************************/

void nrf53_oscconfig(void)
{
#ifdef CONFIG_NRF53_OSCILLATOR_LFXO
  /* Configure LFXO pins */

  nrf53_gpio_config(LFXO_XL1_GPIO_PIN);
  nrf53_gpio_config(LFXO_XL2_GPIO_PIN);

  /* Configure internal capacitors for LFXO */

  putreg32(BOARD_OSC_XOSC32KI_INTCAP, NRF53_OSC_XOSC32KI_INTCAP);
#endif

#ifdef CONFIG_NRF53_HFCLK_XTAL
#  warning TODO: missing HFCLK XTAL oscillator config
#endif
}
