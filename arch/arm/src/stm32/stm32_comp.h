/****************************************************************************
 * arch/arm/src/stm32/stm32_comp.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_COMP_H
#define __ARCH_ARM_SRC_STM32_STM32_COMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include "hardware/stm32_comp.h"

#if defined(CONFIG_STM32_HAVE_IP_COMP_V1)
#  include "stm32_comp_v1.h"
#elif defined(CONFIG_STM32_HAVE_IP_COMP_V2)
#  include "stm32_comp_v2.h"
#endif

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

/****************************************************************************
 * Name: stm32_compinitialize
 *
 * Description:
 *   Initialize the COMP.
 *
 * Input Parameters:
 *   intf - The COMP interface number.
 *
 * Returned Value:
 *   Valid COMP device structure reference on success; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the COMP block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct comp_dev_s *stm32_compinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32_STM32_COMP_H */
