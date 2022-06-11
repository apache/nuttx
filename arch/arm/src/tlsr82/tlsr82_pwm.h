/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_pwm.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_PWM_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include "hardware/tlsr82_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PWM_ENABLE                0x1
#define PWM_DISABLE               0x0

#define PWM_MODE_NORMAL           0x0
#define PWM_MODE_CNT              0x1
#define PWM_MODE_IR               0x3
#define PWM_MODE_IR_FIFO          0x7
#define PWM_MODE_IR_DMA_FIFO      0xF

#define PWM_INV_ENABLE            0x1
#define PWM_INV_DISABLE           0x0

#define PWM_POL_HIGH              0x0
#define PWM_POL_LOW               0x1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

int tlsr82_pwminitialize(const char *devpath, int miror);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TLSR82_TLSR82_PWM_H */
