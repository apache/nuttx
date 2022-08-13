/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_boot.c
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
#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_start.h"
#include "sam_pck.h"
#include "samv71-xult.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAMV7 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ****************************************************************************/

void sam_boardinitialize(void)
{
#if defined(CONFIG_SCHED_TICKLESS) || defined(CONFIG_TIMER)
  uint32_t frequency;
  uint32_t actual;

  /* If Tickless mode is selected then enabled PCK6 as a possible clock
   * source for the timer/counters.  The ideal frequency could be:
   *
   *  frequency = 1,000,000 / CONFIG_USEC_PER_TICK
   *
   * The main crystal is selected as the frequency source.  The maximum
   * prescaler value is 256 so the minimum frequency is 46,875 Hz which
   * corresponds to a period of 21.3 microseconds.  A value of
   * CONFIG_USEC_PER_TICK=20, or 50KHz, would give an exact solution with
   * a divider of 240.
   */

  frequency = USEC_PER_SEC / CONFIG_USEC_PER_TICK;

#if defined(CONFIG_TIMER)
  /* Timer driver needs at least microseconds resolution */

  if (frequency < USEC_PER_SEC)
    {
      frequency = USEC_PER_SEC;
    }
#endif

  DEBUGASSERT(frequency >= (BOARD_MAINOSC_FREQUENCY / 256));

  actual = sam_pck_configure(PCK6, PCKSRC_MAINCK, frequency);

  /* We expect to achieve this frequency exactly */

  DEBUGASSERT(actual == frequency);
  UNUSED(actual);

  /* Enable PCK6 */

  sam_pck_enable(PCK6, true);
#endif

#ifdef CONFIG_SAMV7_SDRAMC
  /* Configure SDRAM if it has been enabled in the NuttX configuration.
   * Here we assume, of course, that we are not running out of SDRAM.
   */

  sam_sdram_config();
#endif

#ifdef CONFIG_SAMV7_SPI
  /* Configure SPI chip selects if SPI has been enabled */

  sam_spidev_initialize();
#endif

#ifdef HAVE_USB
  /* Setup USB-related GPIO pins for the SAMV71-XULT board. */

  sam_usbinitialize();
#endif

#ifdef HAVE_NETWORK
  /* Configure board resources to support networking if the 1) networking is
   * enabled, and 2) the EMAC module is enabled
   */

  sam_netinitialize();
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  sam_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
