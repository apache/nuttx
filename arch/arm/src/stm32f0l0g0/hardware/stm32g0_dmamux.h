/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_dmamux.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_DMAMUX_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_DMAMUX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* DMAMUX1 mapping ******************************************************************/

/* NOTE: DMAMUX1 channels 0 to 7 are connected to DMA1 channels 0 to 7 */

#define DMAMUX1_REQ_GEN0       (1)
#define DMAMUX1_REQ_GEN1       (2)
#define DMAMUX1_REQ_GEN2       (3)
#define DMAMUX1_REQ_GEN3       (4)
#define DMAMUX1_ADC1           (5)
#define DMAMUX1_AES_IN         (6)
#define DMAMUX1_AES_OUT        (7)

/* TODO: ... */

/* DMAP for DMA1 */

#define DMAMAP_DMA1_REGGEN0     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN0)
#define DMAMAP_DMA1_REGGEN1     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN1)
#define DMAMAP_DMA1_REGGEN2     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN2)
#define DMAMAP_DMA1_REGGEN3     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN3)
#define DMAMAP_DMA1_ADC1        DMAMAP_MAP(DMA1, DMAMUX1_ADC1)

/* TODO: ... */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_DMAMUX_H */
