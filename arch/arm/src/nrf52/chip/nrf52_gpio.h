/************************************************************************************
 * arch/arm/src/nrf52/chip/nrf52_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52_GPIO_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "chip/nrf52_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define NRF52_GPIO_PORT0            0
#define NRF52_GPIO_NPORTS           1

/* Register offsets *****************************************************************/

#define NRF52_GPIO_OUT_OFFSET        0x0504 /* Write GPIO port */
#define NRF52_GPIO_OUTSET_OFFSET     0x0508 /* Set individual bits in GPIO port */
#define NRF52_GPIO_OUTCLR_OFFSET     0x050c /* Clear individual bits in GPIO port */
#define NRF52_GPIO_IN_OFFSET         0x0510 /* Read GPIO port */
#define NRF52_GPIO_DIR_OFFSET        0x0514 /* Direction of GPIO pins */
#define NRF52_GPIO_DIRSET_OFFSET     0x0518 /* DIR set register */
#define NRF52_GPIO_DIRCLR_OFFSET     0x051c /* DIR clear register */
#define NRF52_GPIO_LATCH_OFFSET      0x0520 /* Latch register  */
#define NRF52_GPIO_DETECTMODE_OFFSET 0x0524 /* Select between default DETECT signal behaviour and LDETECT mode */

#define NRF52_GPIO_PIN_CNF_OFFSET(n) (0x0700 + (n << 2))

/* Register addresses ***************************************************************/

#define NRF52_GPIO0_OUT              (NRF52_GPIO_P0_BASE + NRF52_GPIO_OUT_OFFSET)
#define NRF52_GPIO0_OUTSET           (NRF52_GPIO_P0_BASE + NRF52_GPIO_OUTSET_OFFSET)
#define NRF52_GPIO0_OUTCLR           (NRF52_GPIO_P0_BASE + NRF52_GPIO_OUTCLR_OFFSET)
#define NRF52_GPIO0_IN               (NRF52_GPIO_P0_BASE + NRF52_GPIO_IN_OFFSET)
#define NRF52_GPIO0_DIR              (NRF52_GPIO_P0_BASE + NRF52_GPIO_DIR_OFFSET)
#define NRF52_GPIO0_DIRSET           (NRF52_GPIO_P0_BASE + NRF52_GPIO_DIRSET_OFFSET)
#define NRF52_GPIO0_DIRCLR           (NRF52_GPIO_P0_BASE + NRF52_GPIO_DIRCLR_OFFSET)
#define NRF52_GPIO0_CNF(n)           (NRF52_GPIO_P0_BASE + NRF52_GPIO_PIN_CNF_OFFSET(n))

/* Register bit definitions *********************************************************/

#define NRF52_GPIO_CNF_PULL_SHIFT       (2)
#define NRF52_GPIO_CNF_PULL_MASK        (0x3 << NRF52_GPIO_CNF_PULL_SHIFT)
#  define NRF52_GPIO_CNF_PULL_DISABLED  (0 << NRF52_GPIO_CNF_PULL_SHIFT)
#  define NRF52_GPIO_CNF_PULL_DOWN      (1 << NRF52_GPIO_CNF_PULL_SHIFT)
#  define NRF52_GPIO_CNF_PULL_UP        (3 << NRF52_GPIO_CNF_PULL_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF52_CHIP_NRF52_GPIO_H */
