/****************************************************************************
 * boards/arm/lpc214x/mcu123-lpc214x/src/lpc2148_leds.c
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
#include <nuttx/board.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* P1.16-P1.23 control LEDS 1-8 */

#define LEDBIT(led)     (0x00010000 << (led))
#define ALLLEDS         (0x00ff0000)

#ifdef CONFIG_LPC214x_FIO
#  define putled(v,r)    putreg32((v),(LPC214X_FIO1_BASE+(r)))
#  define CLRLEDS        putled(ALLLEDS,LPC214X_FIO_SET_OFFSET)

#  define LED_SET_OFFSET LPC214X_FIO_SET_OFFSET
#  define LED_CLR_OFFSET LPC214X_FIO_CLR_OFFSET
#  define LED_DIR_OFFSET LPC214X_FIO_DIR_OFFSET

#else
#  define putled(v,r)    putreg32((v),(LPC214X_GPIO1_BASE+(r)))
#  define CLRLEDS        putled(ALLLEDS,LPC214X_GPIO_SET_OFFSET)

#  define LED_SET_OFFSET LPC214X_GPIO_SET_OFFSET
#  define LED_CLR_OFFSET LPC214X_GPIO_CLR_OFFSET
#  define LED_DIR_OFFSET LPC214X_GPIO_DIR_OFFSET
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* Initialize GIOs P1.16-P1.23 */

  putled(ALLLEDS, LED_DIR_OFFSET);
  putled(ALLLEDS, LED_SET_OFFSET);
  putled(LEDBIT(0), LED_CLR_OFFSET);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  putled(LEDBIT(led), LED_CLR_OFFSET);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  putled(LEDBIT(led), LED_SET_OFFSET);
}
#endif /* CONFIG_ARCH_LEDS */
