/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_gpio.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_GPIO_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NRF52_HAVE_PORT1
#  define NRF52_GPIO_NPORTS         2
#  define NRF52_GPIO_PORT0          0
#  define NRF52_GPIO_PORT1          1
#else
#  define NRF52_GPIO_PORT0          0
#  define NRF52_GPIO_NPORTS         1
#endif

#define NRF52_GPIO_NPINS            32

/* Register offsets *********************************************************/

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

/* Register addresses *******************************************************/

#define NRF52_GPIO0_OUT              (NRF52_GPIO_P0_BASE + NRF52_GPIO_OUT_OFFSET)
#define NRF52_GPIO0_OUTSET           (NRF52_GPIO_P0_BASE + NRF52_GPIO_OUTSET_OFFSET)
#define NRF52_GPIO0_OUTCLR           (NRF52_GPIO_P0_BASE + NRF52_GPIO_OUTCLR_OFFSET)
#define NRF52_GPIO0_IN               (NRF52_GPIO_P0_BASE + NRF52_GPIO_IN_OFFSET)
#define NRF52_GPIO0_DIR              (NRF52_GPIO_P0_BASE + NRF52_GPIO_DIR_OFFSET)
#define NRF52_GPIO0_DIRSET           (NRF52_GPIO_P0_BASE + NRF52_GPIO_DIRSET_OFFSET)
#define NRF52_GPIO0_DIRCLR           (NRF52_GPIO_P0_BASE + NRF52_GPIO_DIRCLR_OFFSET)
#define NRF52_GPIO0_CNF(n)           (NRF52_GPIO_P0_BASE + NRF52_GPIO_PIN_CNF_OFFSET(n))

#ifdef CONFIG_NRF52_HAVE_PORT1
#  define NRF52_GPIO1_OUT            (NRF52_GPIO_P1_BASE + NRF52_GPIO_OUT_OFFSET)
#  define NRF52_GPIO1_OUTSET         (NRF52_GPIO_P1_BASE + NRF52_GPIO_OUTSET_OFFSET)
#  define NRF52_GPIO1_OUTCLR         (NRF52_GPIO_P1_BASE + NRF52_GPIO_OUTCLR_OFFSET)
#  define NRF52_GPIO1_IN             (NRF52_GPIO_P1_BASE + NRF52_GPIO_IN_OFFSET)
#  define NRF52_GPIO1_DIR            (NRF52_GPIO_P1_BASE + NRF52_GPIO_DIR_OFFSET)
#  define NRF52_GPIO1_DIRSET         (NRF52_GPIO_P1_BASE + NRF52_GPIO_DIRSET_OFFSET)
#  define NRF52_GPIO1_DIRCLR         (NRF52_GPIO_P1_BASE + NRF52_GPIO_DIRCLR_OFFSET)
#  define NRF52_GPIO1_CNF(n)         (NRF52_GPIO_P1_BASE + NRF52_GPIO_PIN_CNF_OFFSET(n))
#endif

/* Register bit definitions *************************************************/

#define GPIO_DETECTMODE_DEFAULT         (0)
#define GPIO_DETECTMODE_LDETECT         (1)

#define GPIO_CNF_DIR                    (1 << 0) /* Bit 0: Pin direction */
#define GPIO_CNF_INPUT                  (1 << 1) /* Bit 1: Input buffer disconnect */
#define GPIO_CNF_PULL_SHIFT             (2)
#define GPIO_CNF_PULL_MASK              (0x3 << GPIO_CNF_PULL_SHIFT)
#  define GPIO_CNF_PULL_DISABLED        (0 << GPIO_CNF_PULL_SHIFT)
#  define GPIO_CNF_PULL_DOWN            (1 << GPIO_CNF_PULL_SHIFT)
#  define GPIO_CNF_PULL_UP              (3 << GPIO_CNF_PULL_SHIFT)
#define GPIO_CNF_DRIVE_SHIFT            (8)
#define GPIO_CNF_DRIVE_MASK             (0x7 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_S0S1           (0 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_H0S1           (1 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_S0H1           (2 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_H0H1           (3 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_D0S1           (4 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_D0H1           (5 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_S0D1           (6 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_H0D1           (7 << GPIO_CNF_DRIVE_SHIFT)
#define GPIO_CNF_SENSE_SHIFT            (16)
#define GPIO_CNF_SENSE_MASK             (0x3 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_DISABLED       (0 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_HIGH           (2 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_LOW            (3 << GPIO_CNF_SENSE_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_GPIO_H */
