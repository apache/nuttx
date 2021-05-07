/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_pit.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_PIT_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_PIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMXRT_PIT_MCR_OFFSET       0x0000 /* PIT Module Control Register */
#define IMXRT_PIT_LTMR64H_OFFSET   0x00e0 /* PIT Upper Lifetime Timer Register */
#define IMXRT_PIT_LTMR64L_OFFSET   0x00e4 /* PIT Lower Lifetime Timer Register */
#define IMXRT_PIT_LDVAL0_OFFSET    0x0100 /* Timer Load Value Register */
#define IMXRT_PIT_CVAL0_OFFSET     0x0104 /* Current Timer Value Register */
#define IMXRT_PIT_TCTRL0_OFFSET    0x0108 /* Timer Control Register */
#define IMXRT_PIT_TFLG0_OFFSET     0x010c /* Timer Flag Register */
#define IMXRT_PIT_LDVAL1_OFFSET    0x0110 /* Timer Load Value Register */
#define IMXRT_PIT_CVAL1_OFFSET     0x0114 /* Current Timer Value Register */
#define IMXRT_PIT_TCTRL1_OFFSET    0x0118 /* Timer Control Register */
#define IMXRT_PIT_TFLG1_OFFSET     0x011c /* Timer Flag Register */
#define IMXRT_PIT_LDVAL2_OFFSET    0x0120 /* Timer Load Value Register */
#define IMXRT_PIT_CVAL2_OFFSET     0x0124 /* Current Timer Value Register */
#define IMXRT_PIT_TCTRL2_OFFSET    0x0128 /* Timer Control Register */
#define IMXRT_PIT_TFLG2_OFFSET     0x012c /* Timer Flag Register */
#define IMXRT_PIT_LDVAL3_OFFSET    0x0130 /* Timer Load Value Register */
#define IMXRT_PIT_CVAL3_OFFSET     0x0134 /* Current Timer Value Register */
#define IMXRT_PIT_TCTRL3_OFFSET    0x0138 /* Timer Control Register */
#define IMXRT_PIT_TFLG3_OFFSET     0x013c /* Timer Flag Register */

/* Register Addresses *******************************************************/

#define IMXRT_PIT_MCR              (IMXRT_PIT_BASE+IMXRT_PIT_MCR_OFFSET)
#define IMXRT_PIT_LTMR64H          (IMXRT_PIT_BASE+IMXRT_PIT_LTMR64H_OFFSET)
#define IMXRT_PIT_LTMR64L          (IMXRT_PIT_BASE+IMXRT_PIT_LTMR64L_OFFSET)
#define IMXRT_PIT_LDVAL0           (IMXRT_PIT_BASE+IMXRT_PIT_LDVAL0_OFFSET)
#define IMXRT_PIT_CVAL0            (IMXRT_PIT_BASE+IMXRT_PIT_CVAL0_OFFSET)
#define IMXRT_PIT_TCTRL0           (IMXRT_PIT_BASE+IMXRT_PIT_TCTRL0_OFFSET)
#define IMXRT_PIT_TFLG0            (IMXRT_PIT_BASE+IMXRT_PIT_TFLG0_OFFSET)
#define IMXRT_PIT_LDVAL1           (IMXRT_PIT_BASE+IMXRT_PIT_LDVAL1_OFFSET)
#define IMXRT_PIT_CVAL1            (IMXRT_PIT_BASE+IMXRT_PIT_CVAL1_OFFSET)
#define IMXRT_PIT_TCTRL1           (IMXRT_PIT_BASE+IMXRT_PIT_TCTRL1_OFFSET)
#define IMXRT_PIT_TFLG1            (IMXRT_PIT_BASE+IMXRT_PIT_TFLG1_OFFSET)
#define IMXRT_PIT_LDVAL2           (IMXRT_PIT_BASE+IMXRT_PIT_LDVAL2_OFFSET)
#define IMXRT_PIT_CVAL2            (IMXRT_PIT_BASE+IMXRT_PIT_CVAL2_OFFSET)
#define IMXRT_PIT_TCTRL2           (IMXRT_PIT_BASE+IMXRT_PIT_TCTRL2_OFFSET)
#define IMXRT_PIT_TFLG2            (IMXRT_PIT_BASE+IMXRT_PIT_TFLG2_OFFSET)
#define IMXRT_PIT_LDVAL3           (IMXRT_PIT_BASE+IMXRT_PIT_LDVAL3_OFFSET)
#define IMXRT_PIT_CVAL3            (IMXRT_PIT_BASE+IMXRT_PIT_CVAL3_OFFSET)
#define IMXRT_PIT_TCTRL3           (IMXRT_PIT_BASE+IMXRT_PIT_TCTRL3_OFFSET)
#define IMXRT_PIT_TFLG3            (IMXRT_PIT_BASE+IMXRT_PIT_TFLG3_OFFSET)

/* Register Bit Definitions *************************************************/

/* PIT Module Control Register */

#define PIT_MCR_FRZ                (1 << 0)  /* Bit 0:  Freeze */
#define PIT_MCR_MDIS               (1 << 1)  /* Bit 1:  Module Disable */
                                             /* Bits 2-31: Reserved */

/* Timer Load Value Register (32-bit Timer Start Value Bits) */

/* Current Timer Value Register (32-bit Current Timer Value) */

/* Timer Control Register */

#define PIT_TCTRL_TEN              (1 << 0)  /* Bit 0:  Timer Enable Bit */
#define PIT_TCTRL_TIE              (1 << 1)  /* Bit 1:  Timer Interrupt Enable Bit */
                                             /* Bits 2-31: Reserved */
#define PIT_TCTRL_CHN              (1 << 2)  /* Bit 2:  Chain Mode */
                                             /* Bits 3-31: Reserved */

/* Timer Flag Register */

#define PIT_TFLG_TIF               (1 << 0)  /* Bit 0:  Timer Interrupt Flag */
                                             /* Bits 1-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_PIT_H */
