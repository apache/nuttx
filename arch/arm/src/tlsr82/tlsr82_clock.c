/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_clock.c
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

#include <nuttx/irq.h>
#include <assert.h>
#include <debug.h>

#include "tlsr82_clock.h"
#include "tlsr82_analog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCU_CORE_B87          1
#define SYSCLK_RC_CLOCK_EN    0

#define CLOCK_SYS_CLOCK_HZ    CPU_CLK

#if (CLOCK_SYS_CLOCK_HZ == 12000000)
#  define SYS_CLK             CLK_SYS_12M_CRYSTAL
#elif (CLOCK_SYS_CLOCK_HZ == 16000000)
#  define SYS_CLK             CLK_SYS_16M_CRYSTAL
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
#  define SYS_CLK             CLK_SYS_24M_CRYSTAL
#elif ((CLOCK_SYS_CLOCK_HZ == 32000000) && (MCU_CORE_B85 || MCU_CORE_B87))
#  define SYS_CLK             CLK_SYS_32M_CRYSTAL
#elif ((CLOCK_SYS_CLOCK_HZ == 48000000) && (MCU_CORE_B85 || MCU_CORE_B87))
#  define SYS_CLK             CLK_SYS_48M_CRYSTAL
#else
#  error "sys clock error or undefined"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void clock_rc24m_init(void)
{
  uint8_t cap;

  tlsr82_analog_write(0xc8, 0x80);

  tlsr82_analog_write(0x30, tlsr82_analog_read(0x30) | BIT(7));

  tlsr82_analog_write(0xc7, 0x0e);
  tlsr82_analog_write(0xc7, 0x0f);
  while ((tlsr82_analog_read(0xcf) & 0x80) == 0);

  /* Write 24m cap into manual register */

  cap = tlsr82_analog_read(0xcb);
  tlsr82_analog_write(0x33, cap);

  tlsr82_analog_write(0x30, tlsr82_analog_read(0x30) & (~BIT(7)));

  tlsr82_analog_write(0xc7, 0x0e);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tlsr82_clock_init(void)
{
  CLK_SYS_SEL = (uint8_t)SYS_CLK;

#if (SYSCLK_RC_CLOCK_EN)
  if (SYS_CLK < SYS_CLK_RC_THRES)
    {
      clock_rc_set(SYS_CLK);
    }
#endif

  clock_rc24m_init();
}
