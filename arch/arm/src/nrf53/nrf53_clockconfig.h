/****************************************************************************
 * arch/arm/src/nrf53/nrf53_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_NRF53_NRF53_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frequencies of internal clocks */

#define NRF53_SYSTEM_CLOCK     64000000  /* Default System clock value */
#define NRF53_RTC_CLOCK           32768  /* RTC oscillator 32 kHz output
                                          * (32k_clk)
                                          */

/* HFCLK192M configuration */

#ifdef CONFIG_NRF53_USE_HFCLK192M
#  if defined(CONFIG_NRF53_HFCLK192M_192)
#    define NRF53_PCLK192M_FREQ    192000000
#  elif defined(CONFIG_NRF53_HFCLK192M_96)
#    define NRF53_PCLK192M_FREQ    96000000
#  elif defined(CONFIG_NRF53_HFCLK192M_48)
#    define NRF53_PCLK192M_FREQ    48000000
#  else
#    error Invalid configuration
#  endif
#else
#  define NRF53_PCLK192M_FREQ    0
#endif

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_clockconfig
 *
 * Description:
 *   Called to initialize the NRF53xxx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 ****************************************************************************/

void nrf53_clockconfig(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_NRF53_NRF53_CLOCKCONFIG_H */
