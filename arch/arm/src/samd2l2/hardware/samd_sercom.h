/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_sercom.h
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

/* References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J–SAM–12/2013
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_SERCOM_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_SERCOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Two generic clocks are used by the SERCOM: GCLK_SERCOMx_CORE and
 * GCLK_SERCOMx_SLOW.  The core clock (GCLK_SERCOMx_CORE) is required to
 * clock the SERCOM while operating as a master, while the slow clock
 * (GCLK_SERCOM_SLOW) is only required for certain functions.
 * SERCOM modules must share the same slow GCLK channel ID.
 *
 * The baud-rate generator runs off the GCLK_SERCOMx_CORE clock
 * (or, optionally, external clock).
 *
 * These definitions must match the GCLK_CLKCTRL_ID_* values defined in
 * samd_gclk.c.
 */

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define SERCOM_GCLK_ID_SLOW          12
#  define SERCOM_GCLK_ID_CORE(n)       (13+(n))
#    define SERCOM0_GCLK_ID_CORE       13
#    define SERCOM1_GCLK_ID_CORE       14
#    define SERCOM2_GCLK_ID_CORE       15
#    define SERCOM3_GCLK_ID_CORE       16
#    define SERCOM4_GCLK_ID_CORE       17
#    define SERCOM5_GCLK_ID_CORE       18
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SERCOM_GCLK_ID_SLOW          19
#  define SERCOM_GCLK_ID_CORE(n)       (20+(n))
#    define SERCOM0_GCLK_ID_CORE       20
#    define SERCOM1_GCLK_ID_CORE       21
#    define SERCOM2_GCLK_ID_CORE       22
#    define SERCOM3_GCLK_ID_CORE       23
#    define SERCOM4_GCLK_ID_CORE       24
#    define SERCOM5_GCLK_ID_CORE       25
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_SERCOM_H */
