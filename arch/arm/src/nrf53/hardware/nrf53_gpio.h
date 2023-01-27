/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_gpio.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_GPIO_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF53_GPIO_NPORTS           2
#define NRF53_GPIO_PORT0            0
#define NRF53_GPIO_PORT1            1

#define NRF53_GPIO_NPINS            32

/* Register offsets *********************************************************/

#define NRF53_GPIO_OUT_OFFSET           0x0004 /* Write GPIO port */
#define NRF53_GPIO_OUTSET_OFFSET        0x0008 /* Set individual bits in GPIO port */
#define NRF53_GPIO_OUTCLR_OFFSET        0x000c /* Clear individual bits in GPIO port */
#define NRF53_GPIO_IN_OFFSET            0x0010 /* Read GPIO port */
#define NRF53_GPIO_DIR_OFFSET           0x0014 /* Direction of GPIO pins */
#define NRF53_GPIO_DIRSET_OFFSET        0x0018 /* DIR set register */
#define NRF53_GPIO_DIRCLR_OFFSET        0x001c /* DIR clear register */
#define NRF53_GPIO_LATCH_OFFSET         0x0020 /* Latch register  */
#define NRF53_GPIO_DETECTMODE_OFFSET    0x0024 /* Select between default DETECT signal behaviour and LDETECT mode (non-secure) */
#define NRF53_GPIO_DETECTMODESEC_OFFSET 0x0024 /* Select between default DETECT signal behaviour and LDETECT mode (secure) */

#define NRF53_GPIO_PIN_CNF_OFFSET(n) (0x0200 + (n << 2))

/* Register addresses *******************************************************/

#define NRF53_GPIO0_OUT              (NRF53_GPIO_P0_BASE + NRF53_GPIO_OUT_OFFSET)
#define NRF53_GPIO0_OUTSET           (NRF53_GPIO_P0_BASE + NRF53_GPIO_OUTSET_OFFSET)
#define NRF53_GPIO0_OUTCLR           (NRF53_GPIO_P0_BASE + NRF53_GPIO_OUTCLR_OFFSET)
#define NRF53_GPIO0_IN               (NRF53_GPIO_P0_BASE + NRF53_GPIO_IN_OFFSET)
#define NRF53_GPIO0_DIR              (NRF53_GPIO_P0_BASE + NRF53_GPIO_DIR_OFFSET)
#define NRF53_GPIO0_DIRSET           (NRF53_GPIO_P0_BASE + NRF53_GPIO_DIRSET_OFFSET)
#define NRF53_GPIO0_DIRCLR           (NRF53_GPIO_P0_BASE + NRF53_GPIO_DIRCLR_OFFSET)
#define NRF53_GPIO0_CNF(n)           (NRF53_GPIO_P0_BASE + NRF53_GPIO_PIN_CNF_OFFSET(n))

#ifdef CONFIG_NRF53_HAVE_PORT1
#  define NRF53_GPIO1_OUT            (NRF53_GPIO_P1_BASE + NRF53_GPIO_OUT_OFFSET)
#  define NRF53_GPIO1_OUTSET         (NRF53_GPIO_P1_BASE + NRF53_GPIO_OUTSET_OFFSET)
#  define NRF53_GPIO1_OUTCLR         (NRF53_GPIO_P1_BASE + NRF53_GPIO_OUTCLR_OFFSET)
#  define NRF53_GPIO1_IN             (NRF53_GPIO_P1_BASE + NRF53_GPIO_IN_OFFSET)
#  define NRF53_GPIO1_DIR            (NRF53_GPIO_P1_BASE + NRF53_GPIO_DIR_OFFSET)
#  define NRF53_GPIO1_DIRSET         (NRF53_GPIO_P1_BASE + NRF53_GPIO_DIRSET_OFFSET)
#  define NRF53_GPIO1_DIRCLR         (NRF53_GPIO_P1_BASE + NRF53_GPIO_DIRCLR_OFFSET)
#  define NRF53_GPIO1_CNF(n)         (NRF53_GPIO_P1_BASE + NRF53_GPIO_PIN_CNF_OFFSET(n))
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
#define GPIO_CNF_DRIVE_MASK             (0xf << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_S0S1           (0 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_H0S1           (1 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_S0H1           (2 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_H0H1           (3 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_D0S1           (4 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_D0H1           (5 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_S0D1           (6 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_H0D1           (7 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_EOS1           (9 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_SOE1           (10 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_EOE1           (11 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_DOE1           (13 << GPIO_CNF_DRIVE_SHIFT)
#  define GPIO_CNF_DRIVE_EOD1           (15 << GPIO_CNF_DRIVE_SHIFT)
#define GPIO_CNF_SENSE_SHIFT            (16)
#define GPIO_CNF_SENSE_MASK             (0x3 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_DISABLED       (0 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_HIGH           (2 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_LOW            (3 << GPIO_CNF_SENSE_SHIFT)
#define GPIO_CNF_MCUSEL_SHIFT           (28)
#define GPIO_CNF_MCUSEL_MASK            (0x7 << GPIO_CNF_MCUSEL_SHIFT)
#  define GPIO_CNF_MCUSEL_CPUAPP        (0 << GPIO_CNF_MCUSEL_SHIFT)
#  define GPIO_CNF_MCUSEL_CPUNET        (1 << GPIO_CNF_MCUSEL_SHIFT)
#  define GPIO_CNF_MCUSEL_PERIP         (3 << GPIO_CNF_MCUSEL_SHIFT)
#  define GPIO_CNF_MCUSEL_TND           (4 << GPIO_CNF_MCUSEL_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_GPIO_H */
