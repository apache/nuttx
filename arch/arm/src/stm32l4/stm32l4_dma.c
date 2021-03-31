/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_dma.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* This file is only a thin shell that includes the correct DMA
 * implementation for the selected STM32 family.  The correct file cannot be
 * selected by the make system because it needs the intelligence that only
 * exists in chip.h that can associate an STM32 part number with an STM32
 * family.
 */

#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4X5) || \
    defined(CONFIG_STM32L4_STM32L4X6)
#include "stm32l4x6xx_dma.c"
#elif defined(CONFIG_STM32L4_STM32L4XR)
#include "stm32l4xrxx_dma.c"
#else
#  error "Unsupported STM32L4 chip"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
