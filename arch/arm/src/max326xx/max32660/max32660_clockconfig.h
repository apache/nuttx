/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <arch/board/board.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock sources.  Values match the CLKSEL field of the GCR_CLK_CTRL
 * register
 */

enum clock_source_e
{
  CLKSRC_HFIO  = 0, /* High frequency internal oscillator */
  CLKSRC_8KHZ  = 3, /* 8kHz Internal Ultra-Low Power Nano-Ring Oscillator */
  CLKSRC_32KHZ = 6  /* 32.768kHz External Crystal Oscillator */
};

/* This structure can be used to define a clock configuration.
 *
 * Fhfio     Determined by Output Voltage Range.
 * Fsysosc   Determined by source clock selection.
 * Fsysclk = Fsysclk / (2^psc)
 * Fpclk   = Fsysclk / 2
 */

struct clock_setup_s
{
  uint8_t ovr;      /* Output voltage range for internal regulator */
  uint8_t clksrc;   /* See enum clock_source_e.  Determines Fsysosc */
  uint8_t psc;      /* System Oscillator Prescaler.  Derives Fsysclk */
  bool hfio;        /* True: Enable the High frequency internal oscillator. */
#ifdef BOARD_HAVE_X32K
  bool x32k;        /* True: Enable the 32.768KHz ext crystal oscillator. */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX32660_MAX32660_CLOCKCONFIG_H */
