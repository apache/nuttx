/****************************************************************************
 * arch/risc-v/src/ab32vg1/hardware/ab32vg1_gpio.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_GPIO_H
#define __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ab32vg1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO ports A, B, E, F (register manual, chapter 1).  Each port has the
 * same register layout; ports are 16 registers apart.
 */

#define AB32VG1_GPIOA_BASE      AB32VG1_SFR(6, 0x00)
#define AB32VG1_GPIOB_BASE      AB32VG1_SFR(6, 0x10)
#define AB32VG1_GPIOE_BASE      AB32VG1_SFR(6, 0x20)
#define AB32VG1_GPIOF_BASE      AB32VG1_SFR(6, 0x30)

#define AB32VG1_GPIO_SET_OFFSET     0x00   /* Write 1: output high */
#define AB32VG1_GPIO_CLR_OFFSET     0x04   /* Write 1: output low */
#define AB32VG1_GPIO_DATA_OFFSET    0x08   /* Pin level / output latch */
#define AB32VG1_GPIO_DIR_OFFSET     0x0c   /* 1: input, 0: output */
#define AB32VG1_GPIO_DE_OFFSET      0x10   /* 1: digital, 0: analog */
#define AB32VG1_GPIO_FEN_OFFSET     0x14   /* 1: peripheral function */
#define AB32VG1_GPIO_DRV_OFFSET     0x18   /* Drive strength */
#define AB32VG1_GPIO_PU_OFFSET      0x1c   /* 10K pull-up */
#define AB32VG1_GPIO_PD_OFFSET      0x20   /* 10K pull-down */
#define AB32VG1_GPIO_PU200K_OFFSET  0x24
#define AB32VG1_GPIO_PD200K_OFFSET  0x28
#define AB32VG1_GPIO_PU300_OFFSET   0x2c
#define AB32VG1_GPIO_PD300_OFFSET   0x30

/* Function mapping (register manual, FUNCMCON0..2).  Writing 0xf to a
 * 4-bit field clears it, writing another value selects a pin group; the
 * other fields are not affected by writing 0.
 */

#define AB32VG1_FUNCMCON0       AB32VG1_SFR(0, 0x07)
#define AB32VG1_FUNCMCON1       AB32VG1_SFR(0, 0x08)
#define AB32VG1_FUNCMCON2       AB32VG1_SFR(0, 0x09)

#define FUNCMCON0_SD0MAP_SHIFT    0
#define FUNCMCON0_SPI0MAP_SHIFT   4
#define FUNCMCON0_UT0TXMAP_SHIFT  8
#define FUNCMCON0_UT0RXMAP_SHIFT  12
#define FUNCMCON0_HSUTTXMAP_SHIFT 16
#define FUNCMCON0_HSUTRXMAP_SHIFT 20
#define FUNCMCON0_UT1TXMAP_SHIFT  24
#define FUNCMCON0_UT1RXMAP_SHIFT  28

#define FUNCMCON_CLEAR          0xf
#define FUNCMCON_G(n)           (n)        /* Map to pin group n */
#define FUNCMCON_RX_ON_TX       0x7        /* UARTx RX on its TX pin */

#endif /* __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_GPIO_H */
