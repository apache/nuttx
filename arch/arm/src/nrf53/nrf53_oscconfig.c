/****************************************************************************
 * arch/arm/src/nrf53/nrf53_oscconfig.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf53_oscconfig.h"
#include "nrf53_gpio.h"
#include "hardware/nrf53_osc.h"
#include "hardware/nrf53_ficr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NRF53_NETCORE
#  error Oscillators configuration available only for the App core
#endif

/* LFXO pins */

#define LFXO_XL1_GPIO_PIN  (GPIO_MCUSEL_PERIP | GPIO_PORT0 | GPIO_PIN(0))
#define LFXO_XL2_GPIO_PIN  (GPIO_MCUSEL_PERIP | GPIO_PORT0 | GPIO_PIN(1))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_NRF53_HFCLK_XTAL) && defined(BOARD_OSC_XOSC32MCAPS_CAP)
static void nrf53_hfxo_intcap(void)
{
  uint32_t slope_field;
  uint32_t slope_mask;
  uint32_t slope_sign;
  int32_t  slope;
  uint32_t offset;
  uint32_t capvalue;
  uint32_t trim;

  trim = getreg32(NRF53_FICR_XOSC32MTRIM);

  slope_field = (trim & FICR_XOSC32MTRIM_OFFSET_MASK);
  slope_mask = FICR_XOSC32MTRIM_OFFSET_MASK;
  slope_sign = (slope_mask - (slope_mask >> 1));
  slope = (int32_t)(slope_field ^ slope_sign) - (int32_t)slope_sign;

  offset = ((trim & FICR_XOSC32MTRIM_SLOPE_MASK) >>
            FICR_XOSC32MTRIM_SLOPE_SHIFT);

  /* CAPVALUE = (((FICR->XOSC32MTRIM.SLOPE+56)*(CAPACITANCE*2-14))
   *            +((FICR->XOSC32MTRIM.OFFSET-8)<<4)+32)>>6;
   */

  capvalue = ((slope + 56) * ((BOARD_OSC_XOSC32MCAPS_CAP * 2) - 14)
              + ((offset - 8) << 4) + 32) >> 6;

  putreg32(OSC_XOSC32MCAPS_ENABLE | capvalue, NRF53_OSC_XOSC32MCAPS);
}
#endif

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

#if defined(CONFIG_NRF53_HFCLK_XTAL) && defined(BOARD_OSC_XOSC32MCAPS_CAP)
  /* Configure internal capacitors for HFXO */

  nrf53_hfxo_intcap();
#endif
}
