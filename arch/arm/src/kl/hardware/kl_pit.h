/****************************************************************************
 * arch/arm/src/kl/hardware/kl_pit.h
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_PIT_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_PIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "kl_config.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

#define PIT_MCR_OFFSET       0x0000 /* PIT Module Control Register offset */
#define PIT_LTMR64H_OFFSET   0x00E0 /* PIT Upper Lifetime Timer Register offset */
#define PIT_LTMR64L_OFFSET   0x00E4 /* PIT Lower Lifetime Timer Register offset */
#define PIT_LDVAL0_OFFSET    0x0100 /* Timer Load Value Register offset */
#define PIT_CVAL0_OFFSET     0x0104 /* Current Timer Value Register offset */
#define PIT_TCTRL0_OFFSET    0x0108 /* Timer Control Register offset */
#define PIT_TFLG0_OFFSET     0x010C /* Timer Flag Register offset */
#define PIT_LDVAL1_OFFSET    0x0110 /* Timer Load Value Register offset */
#define PIT_CVAL1_OFFSET     0x0114 /* Current Timer Value Register offset */
#define PIT_TCTRL1_OFFSET    0x0118 /* Timer Control Register offset */
#define PIT_TFLG1_OFFSET     0x011C /* Timer Flag Register offset */

#define PIT_MCR             (KL_PIT_BASE + PIT_MCR_OFFSET)      /* PIT Module Control Register */
#define PIT_LTMR64H         (KL_PIT_BASE + PIT_LTMR64H_OFFSET)  /* PIT Upper Lifetime Timer Register */
#define PIT_LTMR64L         (KL_PIT_BASE + PIT_LTMR64L_OFFSET)  /* PIT Lower Lifetime Timer Register */
#define PIT_LDVAL0          (KL_PIT_BASE + PIT_LDVAL0_OFFSET)   /* Timer Load Value Register */
#define PIT_CVAL0           (KL_PIT_BASE + PIT_CVAL0_OFFSET)    /* Current Timer Value Register */
#define PIT_TCTRL0          (KL_PIT_BASE + PIT_TCTRL0_OFFSET)   /* Timer Control Register */
#define PIT_TFLG0           (KL_PIT_BASE + PIT_TFLG0_OFFSET)    /* Timer Flag Register */
#define PIT_LDVAL1          (KL_PIT_BASE + PIT_LDVAL1_OFFSET)   /* Timer Load Value Register */
#define PIT_CVAL1           (KL_PIT_BASE + PIT_CVAL1_OFFSET)    /* Current Timer Value Register */
#define PIT_TCTRL1          (KL_PIT_BASE + PIT_TCTRL1_OFFSET)   /* Timer Control Register */
#define PIT_TFLG1           (KL_PIT_BASE + PIT_TFLG1_OFFSET)    /* Timer Flag Register */

                            /* MCR Bits 31-2: Reserved */
#define PIT_MCR_MDIS        (1 << 1) /* Module Disable */
#define PIT_MCR_FRZ         (1 << 0) /* Freeze when in debug mode */

                             /* TCTRLn Bits 0-28: Reserved */
#define PIT_TCTRL_CHN       (1 << 2) /* Chain Mode */
#define PIT_TCTRL_TIE       (1 << 1) /* Timer Interrupt Enable */
#define PIT_TCTRL_TEN       (1 << 0) /* Timer Enable */

                            /* TFLGn Bits 0-30: Reserved */
#define PIT_TFLG_TIF        (1 << 0) /* Timer Interrupt Flag */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_PIT_H */
