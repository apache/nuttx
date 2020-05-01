/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_boot.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "arm_arch.h"
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
#ifdef CONFIG_SCHED_TICKLESS
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
 *   function called board_late_initialize().  board_late_initialize() will be
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
