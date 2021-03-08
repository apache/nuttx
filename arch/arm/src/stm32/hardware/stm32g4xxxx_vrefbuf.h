/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_vrefbuf.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_VREFBUF_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_VREFBUF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_VREFBUF_CSR_OFFSET          0x0000                         /* VREFBUF control and status register (CSR) */
#define STM32_VREFBUF_CCR_OFFSET          0x0004                         /* VREFBUF calibration control register (CCR) */

/* Register Addresses *******************************************************/

#define STM32_VREFBUF_CSR                 (STM32_VREFBUF_BASE + STM32_VREFBUF_CSR_OFFSET)
#define STM32_VREFBUF_CCR                 (STM32_VREFBUF_BASE + STM32_VREFBUF_CCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* VREFBUF control and status register (CSR) */

#define VREFBUF_CSR_ENVR                  (1 << 0)                       /* Bit 0: VREFBUF Mode: 0 = Use external vref, 1 = VREFBUF enable or hold */
#define VREFBUF_CSR_HIZ                   (1 << 1)                       /* Bit 1: HiZ mode: 0 = Connect VREF+ to VREFBUF; 1 = VREF+ pin HiZ */
#define VREFBUF_CSR_RESERVED2             (1 << 2)                       /* Bit 2: Reserved; keep at reset value */
#define VREFBUF_CSR_VRR                   (1 << 3)                       /* Bit 3: Voltage Reference Buffer Ready (0 = not ready; 1 = ready) */
#define VREFBUF_CSR_VRS_SHIFT             (4)                            /* Bits 4-5 */
#define VREFBUF_CSR_VRS_MASK              (0x3 << VREFBUF_CSR_VRS_SHIFT) /* Voltage Reference Scale (VRS) selection bitmask */
#  define VREFBUF_CSR_VRS_2_048V          (0x0 << VREFBUF_CSR_VRS_SHIFT) /* 00: Voltage reference set to 2.048V */
#  define VREFBUF_CSR_VRS_2_5V            (0x1 << VREFBUF_CSR_VRS_SHIFT) /* 01: Voltage reference set to 2.5V */
#  define VREFBUF_CSR_VRS_2_90V           (0x2 << VREFBUF_CSR_VRS_SHIFT) /* 10: Voltage reference set to 2.90V */
#  define VREFBUF_CSR_VRS_RESERVED        (0x3 << VREFBUF_CSR_VRS_SHIFT) /* 11: Reserved; do not use */

/* VREFBUF calibration control register (CCR) */

#define VREFBUF_CCR_TRIM_SHIFT            (0)
#define VREFBUF_CCR_TRIM_MASK             (0x3f)                         /* 6-bit unsigned trim code */

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_VREFBUF_H */
