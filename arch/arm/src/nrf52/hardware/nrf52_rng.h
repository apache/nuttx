/************************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_rng.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Zhiqiang Li <levin.li@outlook.com>
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_RNG_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_RNG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets for RNG *********************************************************/

#define NRF52_RNG_TASKS_START_OFFSET   0x0000  /* RNG Task Start */
#define NRF52_RNG_TASKS_STOP_OFFSET    0x0004  /* RNG Task Stop */
#define NRF52_RNG_EVENTS_RDY_OFFSET    0x0100  /* RNG Eevent ValRDY */
#define NRF52_RNG_SHORTS_OFFSET        0x0200  /* RNG Short Register */
#define NRF52_RNG_INTSET_OFFSET        0x0304  /* RNG INT SET Register */
#define NRF52_RNG_INTCLR_OFFSET        0x0308  /* RNG INT CLR Register */
#define NRF52_RNG_CONFIG_OFFSET        0x0504  /* RNG CONFIG Register */
#define NRF52_RNG_VALUE_OFFSET         0x0508  /* RNG Value Register */

/* Register Addresses for RNG *******************************************************/

#define NRF52_RNG_TASKS_START          (NRF52_RNG_BASE + NRF52_RNG_TASKS_START_OFFSET)
#define NRF52_RNG_TASKS_STOP           (NRF52_RNG_BASE + NRF52_RNG_TASKS_STOP_OFFSET)
#define NRF52_RNG_EVENTS_RDY           (NRF52_RNG_BASE + NRF52_RNG_EVENTS_RDY_OFFSET)

#define NRF52_RNG_SHORTS               (NRF52_RNG_BASE + NRF52_RNG_SHORTS_OFFSET)
#define NRF52_RNG_INTSET               (NRF52_RNG_BASE + NRF52_RNG_INTSET_OFFSET)
#define NRF52_RNG_INTCLR               (NRF52_RNG_BASE + NRF52_RNG_INTCLR_OFFSET)

#define NRF52_RNG_CONFIG               (NRF52_RNG_BASE + NRF52_RNG_CONFIG_OFFSET)
#define NRF52_RNG_VALUE                (NRF52_RNG_BASE + NRF52_RNG_VALUE_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* INTSET/INTCLR Register */

#define RNG_INT_RDY                    (1 << 0) /* Bit 0: VALRDY event */

/* CONFIG Register */

#define RNG_CONFIG_DERCEN              (1 << 0) /* Bit 0: Bias correction */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_STM32_RNG_H */
