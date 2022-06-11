/****************************************************************************
 * arch/arm/src/stm32wl5/stm32wl5_waste.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_STM32WL5_WASTE_H
#define __ARCH_ARM_SRC_STM32WL5_STM32WL5_WASTE_H

/* Waste CPU Time */

/****************************************************************************
 * Pre-processor Definitions
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

/* Waste CPU Time
 *
 * stm32wl5_waste() is the logic that will be executed when portions of
 * kernel or user-app is polling some register or similar, waiting for
 * desired status. This time is wasted away. This function offers a measure
 * of badly written piece of software or some undesired behavior. At
 * the same time this function adds to some IDLE time which portion cannot
 * be used for other purposes (yet).
 */

void stm32wl5_waste(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32WL5_STM32WL5_WASTE_H */
