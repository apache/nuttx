/****************************************************************************
 * arch/arm/src/stm32h7/stm32_dualcore.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_DUALCORE_H
#define __ARCH_ARM_SRC_STM32H7_STM32_DUALCORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware semaphore used for cores synchronisation */

#define CPU2_HOLD_HSEM (0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(CONFIG_ARCH_CHIP_STM32H7_CORTEXM7) && \
    defined(CONFIG_STM32H7_CORTEXM4_ENABLED)

/****************************************************************************
 * Name: stm32h7_start_cm4
 *
 * Description:
 *   Start CM4 core
 *
 ****************************************************************************/

void stm32h7_start_cm4(void);
#endif

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4
/****************************************************************************
 * Name: stm32h7_waitfor_cm7
 *
 * Description:
 *   Wait for CM7 core initialization
 *
 ****************************************************************************/

void stm32h7_waitfor_cm7(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_DUALCORE_H */
