/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_flexcomm.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_FLEXCOMM_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_FLEXCOMM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NXXX_FLEXCOMM_PSELID_OFFSET   0x0ff8  /* Peripheral Select /Flexcomm Interface ID */
#define NXXX_FLEXCOMM_PID_OFFSET      0x0ffc  /* Peripheral identification register */

/* Register addresses *******************************************************/

#define NXXX_FLEXCOMM0_PSELID         (NXXX_FLEXCOMM0_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM0_PID            (NXXX_FLEXCOMM0_BASE + NXXX_FLEXCOMM_PID_OFFSET)

#define NXXX_FLEXCOMM1_PSELID         (NXXX_FLEXCOMM1_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM1_PID            (NXXX_FLEXCOMM1_BASE + NXXX_FLEXCOMM_PID_OFFSET)

#define NXXX_FLEXCOMM2_PSELID         (NXXX_FLEXCOMM2_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM2_PID            (NXXX_FLEXCOMM2_BASE + NXXX_FLEXCOMM_PID_OFFSET)

#define NXXX_FLEXCOMM3_PSELID         (NXXX_FLEXCOMM3_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM3_PID            (NXXX_FLEXCOMM3_BASE + NXXX_FLEXCOMM_PID_OFFSET)

#define NXXX_FLEXCOMM4_PSELID         (NXXX_FLEXCOMM4_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM4_PID            (NXXX_FLEXCOMM4_BASE + NXXX_FLEXCOMM_PID_OFFSET)

#define NXXX_FLEXCOMM5_PSELID         (NXXX_FLEXCOMM5_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM5_PID            (NXXX_FLEXCOMM5_BASE + NXXX_FLEXCOMM_PID_OFFSET)

#define NXXX_FLEXCOMM6_PSELID         (NXXX_FLEXCOMM6_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM6_PID            (NXXX_FLEXCOMM6_BASE + NXXX_FLEXCOMM_PID_OFFSET)

#define NXXX_FLEXCOMM7_PSELID         (NXXX_FLEXCOMM7_BASE + NXXX_FLEXCOMM_PSELID_OFFSET)
#define NXXX_FLEXCOMM7_PID            (NXXX_FLEXCOMM7_BASE + NXXX_FLEXCOMM_PID_OFFSET)

/* Register bit definitions *************************************************/

/* Peripheral Select /Flexcomm Interface ID */

#define FLEXCOMM_PSELID_PERSEL_SHIFT   (0)       /* Bits 0-2: Peripheral Select */
#define FLEXCOMM_PSELID_PERSEL_MASK    (7 << FLEXCOMM_PSELID_PERSEL_SHIFT)
#  define FLEXCOMM_PSELID_PERSEL_NONE  (0 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* No peripheral selected */
#  define FLEXCOMM_PSELID_PERSEL_USART (1 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* USART function selected */
#  define FLEXCOMM_PSELID_PERSEL_SPI   (2 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* SPI function selected */
#  define FLEXCOMM_PSELID_PERSEL_I2C   (3 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* I2C function selected */

#define FLEXCOMM_PSELID_LOCK           (1 << 3)  /* Bit 3:  Lock the peripheral select */
#define FLEXCOMM_PSELID_USARTPRESENT   (1 << 4)  /* Bit 4:  USART present indicator */
#define FLEXCOMM_PSELID_SPIPRESENT     (1 << 5)  /* Bit 5:  SPI present indicator */
#define FLEXCOMM_PSELID_I2CPRESENT     (1 << 6)  /* Bit 6:  I2C present indicator */
#define FLEXCOMM_PSELID_ID_SHIFT       (12)      /* Bits 12-31: Flexcomm Interface ID */
#define FLEXCOMM_PSELID_ID_MASK        (0xfffff << FLEXCOMM_PSELID_ID_SHIFT)

/* Peripheral identification register */

#define FLEXCOMM_PID_MINOR_SHIFT       (8)       /* Bits 8-11:  Minor revision number */
#define FLEXCOMM_PID_MINOR_MASK        (15 << FLEXCOMM_PID_MINOR_SHIFT)
#define FLEXCOMM_PID_MAJOR_SHIFT       (12)      /* Bits 12-15: Major revision number */
#define FLEXCOMM_PID_MAJOR_MASK        (15 << FLEXCOMM_PID_MAJOR_SHIFT)
#define FLEXCOMM_PID_ID_SHIFT          (16)      /* Bits 15-31: Module ID for selected function */
#define FLEXCOMM_PID_ID_MASK           (0xffff << FLEXCOMM_PID_ID_SHIFT)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_FLEXCOMM_H */
