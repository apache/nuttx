/****************************************************************************
 * boards/arm/tiva/lm3s6432-s2e/src/lm_boot.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "tiva_gpio.h"
#include "lm3s6432-s2e.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_TIVA_UART1) && defined(CONFIG_TIVA_SSI0)
#  error Only one of UART1 and SSI0 can be enabled on this board.
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_boardinitialize
 *
 * Description:
 *   All Stellaris architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void tiva_boardinitialize(void)
{
  /* Configure SPI chip selects if
   * 1) SSI is not disabled, and
   * 2) the weak function
   * lm_ssidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_TIVA_SSI0)
  if (lm_ssidev_initialize)
    {
      lm_ssidev_initialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Configure serial transceiver */

  tiva_configgpio(XCVR_INV_GPIO);
  tiva_configgpio(XCVR_ENA_GPIO);
  tiva_configgpio(XCVR_ON_GPIO);
  tiva_configgpio(XCVR_OFF_GPIO);
}
