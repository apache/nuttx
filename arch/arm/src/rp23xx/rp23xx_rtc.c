/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_rtc.c
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
 *
 * The RP2350 has no dedicated RTC block (the RP2040 one was removed).  This
 * driver uses the POWMAN always-on timer instead: a 64-bit millisecond
 * counter, clocked from the low-power oscillator so it keeps running across
 * warm resets.  It backs the NuttX system clock through the simple up_rtc_*
 * interface (seconds resolution).
 *
 * Reference: the POWMAN "Always-on Timer" in the Power chapter of the RP2350
 * datasheet (https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
 * and the pico-sdk hardware_powman library
 * (https://github.com/raspberrypi/pico-sdk, src/rp2_common/hardware_powman),
 * whose powman_timer_start_lposc()/set_ms()/get_ms() this mirrors.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/rp23xx_powman.h"

#ifdef CONFIG_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Every POWMAN write must carry this password in the top 16 bits, otherwise
 * it is ignored (and flagged in BADPASSWD).  Only the low 16 bits are data,
 * which is why the 64-bit time is split across four registers.
 */

#define POWMAN_PASSWORD   0x5afe0000

/* Atomic set/clear aliases of a bus register (RP2350 memory map). */

#define POWMAN_SET_ALIAS  0x2000
#define POWMAN_CLR_ALIAS  0x3000

/* Nominal low-power oscillator frequency. */

#define LPOSC_FREQ_HZ     32768

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Variable determines the state of the RTC module; set once the counter is
 * running so the rest of the system knows the RTC is available.
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void powman_write(uint32_t reg, uint32_t value)
{
  putreg32(POWMAN_PASSWORD | (value & 0xffff), reg);
}

static inline void powman_setbits(uint32_t reg, uint32_t bits)
{
  putreg32(POWMAN_PASSWORD | bits, reg + POWMAN_SET_ALIAS);
}

static inline void powman_clrbits(uint32_t reg, uint32_t bits)
{
  putreg32(POWMAN_PASSWORD | bits, reg + POWMAN_CLR_ALIAS);
}

/* Read the 64-bit millisecond counter, guarding against a carry from the low
 * word to the high word between the two reads.
 */

static uint64_t powman_get_ms(void)
{
  uint32_t hi = getreg32(RP23XX_POWMAN_READ_TIME_UPPER);
  uint32_t lo;

  for (; ; )
    {
      uint32_t next;

      lo   = getreg32(RP23XX_POWMAN_READ_TIME_LOWER);
      next = getreg32(RP23XX_POWMAN_READ_TIME_UPPER);
      if (hi == next)
        {
          break;
        }

      hi = next;
    }

  return ((uint64_t)hi << 32) | lo;
}

/* The counter must be stopped while its value is (re)loaded. */

static void powman_set_ms(uint64_t ms)
{
  bool running =
    (getreg32(RP23XX_POWMAN_TIMER) & RP23XX_POWMAN_TIMER_RUN) != 0;

  if (running)
    {
      powman_clrbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_RUN);
    }

  powman_write(RP23XX_POWMAN_SET_TIME_15TO0,  (uint32_t)(ms));
  powman_write(RP23XX_POWMAN_SET_TIME_31TO16, (uint32_t)(ms >> 16));
  powman_write(RP23XX_POWMAN_SET_TIME_47TO32, (uint32_t)(ms >> 32));
  powman_write(RP23XX_POWMAN_SET_TIME_63TO48, (uint32_t)(ms >> 48));

  if (running)
    {
      powman_setbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_RUN);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the POWMAN always-on timer.  If it is not already running
 *   (e.g. left running by the bootrom), source it from the low-power
 *   oscillator at a 1 kHz tick and start it.  Preserves the current value
 *   when it is already running.
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  if ((getreg32(RP23XX_POWMAN_TIMER) & RP23XX_POWMAN_TIMER_RUN) == 0)
    {
      uint32_t khz_int  = LPOSC_FREQ_HZ / 1000;
      uint32_t khz_frac = (LPOSC_FREQ_HZ % 1000) * 65536 / 1000;
      int timeout       = 1000000;

      powman_write(RP23XX_POWMAN_LPOSC_FREQ_KHZ_INT, khz_int);
      powman_write(RP23XX_POWMAN_LPOSC_FREQ_KHZ_FRAC, khz_frac);
      powman_setbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_USE_LPOSC);
      powman_setbits(RP23XX_POWMAN_TIMER, RP23XX_POWMAN_TIMER_RUN);

      /* Wait for the timer to accept the low-power oscillator source, but do
       * not spin forever if it never does.
       */

      while ((getreg32(RP23XX_POWMAN_TIMER) &
              RP23XX_POWMAN_TIMER_USING_LPOSC) == 0 && --timeout > 0)
        {
        }
    }

  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.
 *
 ****************************************************************************/

time_t up_rtc_time(void)
{
  return (time_t)(powman_get_ms() / 1000);
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time value.
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  irqstate_t flags;
  uint64_t ms;

  ms = (uint64_t)tp->tv_sec * 1000 + tp->tv_nsec / 1000000;

  flags = enter_critical_section();
  powman_set_ms(ms);
  leave_critical_section(flags);

  return OK;
}

#endif /* CONFIG_RTC */
