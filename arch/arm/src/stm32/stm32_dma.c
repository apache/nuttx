/****************************************************************************
 * arch/arm/src/stm32/stm32_dma.c
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
 * implementation for the selected STM32 IP core:
 *   - STM32 DMA IP version 1 - F0, F1, F3, G4, L0, L1, L4
 *   - STM32 DMA IP version 2 - F2, F4, F7, H7
 *
 * The STM32 DMA IPv2 differs from the STM32 DMA IPv1 primarily in that it
 * adds the concept of "streams" that are used to associate DMA sources with
 * DMA channels.
 */

#if defined(CONFIG_STM32_HAVE_IP_DMA_V1)
#  if defined(CONFIG_STM32_HAVE_DMAMUX)
#    include "stm32_dma_v1mux.c"
#  else
#    include "stm32_dma_v1.c"
#  endif
#elif defined(CONFIG_STM32_HAVE_IP_DMA_V2)
#  include "stm32_dma_v2.c"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
