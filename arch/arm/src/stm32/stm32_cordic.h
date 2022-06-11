/****************************************************************************
 * arch/arm/src/stm32/stm32_cordic.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_CORDIC_H
#define __ARCH_ARM_SRC_STM32_STM32_CORDIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cordicinitialize
 *
 * Description:
 *   Initialize a CORDIC device.  This function must be called
 *   from board-specific logic.
 *
 * Returned Value:
 *   On success, a pointer to the lower half CORDIC driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct cordic_lowerhalf_s *stm32_cordicinitialize(void);

#endif /* __ARCH_ARM_SRC_STM32_STM32_CORDIC_H */
