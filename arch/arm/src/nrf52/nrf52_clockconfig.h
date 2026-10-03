/****************************************************************************
 * arch/arm/src/nrf52/nrf52_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_NRF52_NRF52_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frequencies of internal clocks */

#define NRF52_SYSTEM_CLOCK     64000000  /* Default System clock value */
#define NRF52_RTC_CLOCK           32768  /* RTC oscillator 32 kHz output
                                          * (32k_clk)
                                          */

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_clockconfig
 *
 * Description:
 *   Called to initialize the NRF52xxx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 ****************************************************************************/

void nrf52_clockconfig(void);

/****************************************************************************
 * Name: nrf52_clock_hfxo_request
 *
 * Description:
 *   Request the HFCLK crystal oscillator.  The oscillator is started on the
 *   first request; the caller must wait for it to stabilize.
 *
 ****************************************************************************/

bool nrf52_clock_hfxo_request(void);

/****************************************************************************
 * Name: nrf52_clock_hfxo_release
 *
 * Description:
 *   Release the HFCLK crystal oscillator.  The oscillator is stopped when
 *   the last request is released.
 *
 ****************************************************************************/

bool nrf52_clock_hfxo_release(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_NRF52_NRF52_CLOCKCONFIG_H */
