/****************************************************************************
 * arch/avr/src/avrdx/iodefs/avr128da28.h
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

#ifndef __ARCH_AVR_SRC_AVRDX_IODEFS_AVR128DA28_H
#define __ARCH_AVR_SRC_AVRDX_IODEFS_AVR128DA28_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <avr/io.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_IODEFS_H
#  error "Do not include this file directly, use avrdx_iodefs.h instead"
#endif

/* CLKCTRL.OSCHFCTRLA */

#define CLKCTRL_FRQSEL_GM ( CLKCTRL_FRQSEL_0_bm | CLKCTRL_FRQSEL_1_bm | \
                            CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_3_bm )

#define CLKCTRL_FRQSEL_1M_GC (0)
#define CLKCTRL_FRQSEL_2M_GC (CLKCTRL_FRQSEL_0_bm)
#define CLKCTRL_FRQSEL_3M_GC (CLKCTRL_FRQSEL_1_bm)
#define CLKCTRL_FRQSEL_4M_GC (CLKCTRL_FRQSEL_1_bm | CLKCTRL_FRQSEL_0_bm)
#define CLKCTRL_FRQSEL_8M_GC (CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_0_bm)
#define CLKCTRL_FRQSEL_12M_GC (CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_1_bm)
#define CLKCTRL_FRQSEL_16M_GC (CLKCTRL_FRQSEL_2_bm | CLKCTRL_FRQSEL_1_bm | \
                               CLKCTRL_FRQSEL_0_bm )
#define CLKCTRL_FRQSEL_20M_GC (CLKCTRL_FRQSEL_3_bm)
#define CLKCTRL_FRQSEL_24M_GC (CLKCTRL_FRQSEL_3_bm | CLKCTRL_FRQSEL_0_bm)

/* RTC.CTRLA */

#define RTC_PRESCALER_GM ( RTC_PRESCALER_0_bm | RTC_PRESCALER_1_bm | \
                           RTC_PRESCALER_2_bm | RTC_PRESCALER_3_bm )

#define RTC_PRESCALER_DIV2_GC (RTC_PRESCALER_0_bm)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVRDX_AVRDX_H */
