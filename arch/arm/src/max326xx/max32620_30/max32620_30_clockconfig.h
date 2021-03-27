/****************************************************************************
 * arch/arm/src/max326xx/max32620_30/max32620_30_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX_32620_30_MAX32620_30_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_MAX326XX_MAX_32620_30_MAX32620_30_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock sources */

enum clock_source_e
{
  CLKSRC_HFIO = 0,  /* Internal 96MHz Relaxation Oscillator */
  CLKSRC_4KHZ,      /* Internal 4MHz RC Oscillator */
  CLKSRC_44KHZ,     /* Internal 44MHz Relaxation Oscillator (Crypto Oscillator) */
  CLKSRC_32KHZ      /* RTC 32768Hz Crystal Oscillator */
};

/* This structure can be used to define a clock configuration. */

struct clock_setup_s
{
  uint8_t clksrc;   /* See enum clock_source_e */
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

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MAX326XX_MAX_32620_30_MAX32620_30_CLOCKCONFIG_H */
