/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_dmamux.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/* DMAMUX1 mapping ****************************************************/

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
