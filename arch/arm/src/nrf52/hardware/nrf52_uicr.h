/***************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_uicr.h
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
 ***************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_UICR_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_UICR_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Configuration ***********************************************************/

#if defined(CONFIG_ARCH_CHIP_NRF52832)
#  undef  HAVE_UICR_DEBUGCTRL
#  undef  HAVE_UICR_REGOUT0
#elif defined(CONFIG_ARCH_CHIP_NRF52833)
#  define HAVE_UICR_DEBUGCTRL
#  define HAVE_UICR_REGOUT0
#elif defined(CONFIG_ARCH_CHIP_NRF52840)
#  define HAVE_UICR_DEBUGCTRL
#  define HAVE_UICR_REGOUT0
#else
#  error Unknown NRF52 chip !
#endif

/* UICR Register Offsets ***************************************************/

#define NRF52_UICR_NRFFW_OFFSET(x)       (0x014 + ((x) * 0x4)) /* Reserved for Nordic firmware design */
#define NRF52_UICR_NRFHW_OFFSET(x)       (0x050 + ((x) * 0x4)) /* Reserved for Nordic hardware design */
#define NRF52_UICR_CUSTOMER_OFFSET(x)    (0x080 + ((x) * 0x4)) /* Reserved for customer */
#define NRF52_UICR_PSELRESET0_OFFSET     0x200                 /* Mapping of the nRESET function */
#define NRF52_UICR_PSELRESET1_OFFSET     0x204                 /* Mapping of the nRESET function */
#define NRF52_UICR_APPROTECT_OFFSET      0x208                 /* Access port protection */
#define NRF52_UICR_NFCPINS_OFFSET        0x20c                 /* Setting of pins dedicated to NFC */
#ifdef HAVE_UICR_DEBUGCTRL
#  define NRF52_UICR_DEBUGCTRL_OFFSET    0x210                 /* Setting of pins dedicated to NFC */
#endif
#ifdef HAVE_UICR_REGOUT0
#  define NRF52_UICR_REGOUT0_OFFSET      0x304                 /* GPIO reference voltage / external voltage */
#endif

/* UICR Register Addresses *************************************************/

#define NRF52_UICR_PSELRESET0            (NRF52_UICR_BASE + NRF52_UICR_PSELRESET0_OFFSET)
#define NRF52_UICR_PSELRESET1            (NRF52_UICR_BASE + NRF52_UICR_PSELRESET1_OFFSET)
#define NRF52_UICR_APPROTECT             (NRF52_UICR_BASE + NRF52_UICR_APPROTECT_OFFSET)
#define NRF52_UICR_NFCPINS               (NRF52_UICR_BASE + NRF52_UICR_NFCPINS_OFFSET)
#define NRF52_UICR_DEBUGCTRL             (NRF52_UICR_BASE + NRF52_UICR_DEBUGCTRL_OFFSET)
#define NRF52_UICR_REGOUT0               (NRF52_UICR_BASE + NRF52_UICR_REGOUT0_OFFSET)

/* UICR Register Bitfield Definitions **************************************/

/* REGOUT0 Register */

#define UICR_REGOUT0_VOUT_SHIFT          (0) /* Bits 0-2: Output voltage from REG0 regulator stage */
#define UICR_REGOUT0_VOUT_MASK           (0x3 << UICR_REGOUT0_VOUT_SHIFT)
#  define UICR_REGOUT0_VOUT_1V8          (0 << UICR_REGOUT0_VOUT_SHIFT)
#  define UICR_REGOUT0_VOUT_2V1          (1 << UICR_REGOUT0_VOUT_SHIFT)
#  define UICR_REGOUT0_VOUT_2V4          (2 << UICR_REGOUT0_VOUT_SHIFT)
#  define UICR_REGOUT0_VOUT_2V7          (3 << UICR_REGOUT0_VOUT_SHIFT)
#  define UICR_REGOUT0_VOUT_3V0          (4 << UICR_REGOUT0_VOUT_SHIFT)
#  define UICR_REGOUT0_VOUT_3V3          (5 << UICR_REGOUT0_VOUT_SHIFT)
#  define UICR_REGOUT0_VOUT_DEFAULT      (7 << UICR_REGOUT0_VOUT_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_UICR_H */
