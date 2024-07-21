/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_gpio.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_GPIO_H
#define __ARCH_ARM64_SRC_BCM2711_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCM_GPIO_NUM 58 /* Number of GPIO pins */

/* GPIO register offset definitions */

#define BCM_GPIO_GPFSEL0_OFFSET 0x00 /* GPIO function select 0 */
#define BCM_GPIO_GPFSEL1_OFFSET 0x04 /* GPIO function select 1 */
#define BCM_GPIO_GPFSEL2_OFFSET 0x08 /* GPIO function select 2 */
#define BCM_GPIO_GPFSEL3_OFFSET 0x0c /* GPIO function select 3 */
#define BCM_GPIO_GPFSEL4_OFFSET 0x10 /* GPIO function select 4 */
#define BCM_GPIO_GPFSEL5_OFFSET 0x14 /* GPIO function select 5 */
#define BCM_GPIO_GPSET0_OFFSET 0x1c  /* GPIO pin output set 0 */
#define BCM_GPIO_GPSET1_OFFSET 0x20  /* GPIO pin output set 1 */
#define BCM_GPIO_GPCLR0_OFFSET 0x28  /* GPIO pin output clear 0 */
#define BCM_GPIO_GPCLR1_OFFSET 0x2c  /* GPIO pin output clear 1 */
#define BCM_GPIO_GPLEV0_OFFSET 0x34  /* GPIO pin level 0 */
#define BCM_GPIO_GPLEV1_OFFSET 0x38  /* GPIO pin level 1 */

#define BCM_GPIO_GPEDS0_OFFSET 0x40  /* GPIO pin event detect status 0 */
#define BCM_GPIO_GPEDS1_OFFSET 0x44  /* GPIO pin event detect status 1 */
#define BCM_GPIO_GPREN0_OFFSET 0x4c  /* GPIO pin rise edge detect enable 0 */
#define BCM_GPIO_GPREN1_OFFSET 0x50  /* GPIO pin rise edge detect enable 1 */
#define BCM_GPIO_GPFEN0_OFFSET 0x58  /* GPIO pin fall edge detect enable 0 */
#define BCM_GPIO_GPFEN1_OFFSET 0x5c  /* GPIO pin fall edge detect enable 1 */
#define BCM_GPIO_GPHEN0_OFFSET 0x64  /* GPIO pin high detect enable 0 */
#define BCM_GPIO_GPHEN1_OFFSET 0x68  /* GPIO pin high detect enable 1 */
#define BCM_GPIO_GPLEN0_OFFSET 0x70  /* GPIO pin low detect enable 0 */
#define BCM_GPIO_GPLEN1_OFFSET 0x74  /* GPIO pin low detect enable 1 */
#define BCM_GPIO_GPAREN0_OFFSET 0x7c /* GPIO pin async rise edge detect 0 */
#define BCM_GPIO_GPAREN1_OFFSET 0x80 /* GPIO pin async rise edge detect 1 */
#define BCM_GPIO_GPAFEN0_OFFSET 0x88 /* GPIO pin async fall edge detect 0 */
#define BCM_GPIO_GPAFEN1_OFFSET 0x8c /* GPIO pin async fall edge detect 1 */

#define BCM_GPIO_PUP_PDN_REG0_OFFSET 0xe4 /* GPIO pullup/down reg 0 */
#define BCM_GPIO_PUP_PDN_REG1_OFFSET 0xe8 /* GPIO pullup/down reg 1 */
#define BCM_GPIO_PUP_PDN_REG2_OFFSET 0xec /* GPIO pullup/down reg 2 */
#define BCM_GPIO_PUP_PDN_REG3_OFFSET 0xf0 /* GPIO pullup/down reg 3 */

/* GPIO register address definitions */

#define _BCM_GP_REG(reg) (BCM_GPIO_BASEADDR + (reg))

#define BCM_GPIO_GPFSEL0 _BCM_GP_REG(BCM_GPIO_GPFSEL0_OFFSET)
#define BCM_GPIO_GPFSEL1 _BCM_GP_REG(BCM_GPIO_GPFSEL1_OFFSET)
#define BCM_GPIO_GPFSEL2 _BCM_GP_REG(BCM_GPIO_GPFSEL2_OFFSET)
#define BCM_GPIO_GPFSEL3 _BCM_GP_REG(BCM_GPIO_GPFSEL3_OFFSET)
#define BCM_GPIO_GPFSEL4 _BCM_GP_REG(BCM_GPIO_GPFSEL4_OFFSET)
#define BCM_GPIO_GPFSEL5 _BCM_GP_REG(BCM_GPIO_GPFSEL5_OFFSET)
#define BCM_GPIO_GPSET0 _BCM_GP_REG(BCM_GPIO_GPSET0_OFFSET)
#define BCM_GPIO_GPSET1 _BCM_GP_REG(BCM_GPIO_GPSET1_OFFSET)
#define BCM_GPIO_GPCLR0 _BCM_GP_REG(BCM_GPIO_GPCLR0_OFFSET)
#define BCM_GPIO_GPCLR1 _BCM_GP_REG(BCM_GPIO_GPCLR1_OFFSET)
#define BCM_GPIO_GPLEV0 _BCM_GP_REG(BCM_GPIO_GPLEV0_OFFSET)
#define BCM_GPIO_GPLEV1 _BCM_GP_REG(BCM_GPIO_GPLEV1_OFFSET)
#define BCM_GPIO_GPEDS0 _BCM_GP_REG(BCM_GPIO_GPEDS0_OFFSET)
#define BCM_GPIO_GPEDS1 _BCM_GP_REG(BCM_GPIO_GPEDS1_OFFSET)
#define BCM_GPIO_GPREN0 _BCM_GP_REG(BCM_GPIO_GPREN0_OFFSET)
#define BCM_GPIO_GPREN1 _BCM_GP_REG(BCM_GPIO_GPREN1_OFFSET)
#define BCM_GPIO_GPFEN0 _BCM_GP_REG(BCM_GPIO_GPFEN0_OFFSET)
#define BCM_GPIO_GPFEN1 _BCM_GP_REG(BCM_GPIO_GPFEN1_OFFSET)
#define BCM_GPIO_GPHEN0 _BCM_GP_REG(BCM_GPIO_GPHEN0_OFFSET)
#define BCM_GPIO_GPHEN1 _BCM_GP_REG(BCM_GPIO_GPHEN1_OFFSET)
#define BCM_GPIO_GPLEN0 _BCM_GP_REG(BCM_GPIO_GPLEN0_OFFSET)
#define BCM_GPIO_GPLEN1 _BCM_GP_REG(BCM_GPIO_GPLEN1_OFFSET)
#define BCM_GPIO_GPAREN0 _BCM_GP_REG(BCM_GPIO_GPAREN0_OFFSET)
#define BCM_GPIO_GPAREN1 _BCM_GP_REG(BCM_GPIO_GPAREN1_OFFSET)
#define BCM_GPIO_GPAFEN0 _BCM_GP_REG(BCM_GPIO_GPAFEN0_OFFSET)
#define BCM_GPIO_GPAFEN1 _BCM_GP_REG(BCM_GPIO_GPAFEN1_OFFSET)
#define BCM_GPIO_PUP_PDN_CNTRL_REG0 _BCM_GP_REG(BCM_GPIO_PUP_PDN_REG0_OFFSET)
#define BCM_GPIO_PUP_PDN_CNTRL_REG1 _BCM_GP_REG(BCM_GPIO_PUP_PDN_REG1_OFFSET)
#define BCM_GPIO_PUP_PDN_CNTRL_REG2 _BCM_GP_REG(BCM_GPIO_PUP_PDN_REG2_OFFSET)
#define BCM_GPIO_PUP_PDN_CNTRL_REG3 _BCM_GP_REG(BCM_GPIO_PUP_PDN_REG3_OFFSET)

/* GPIO register bit definitions */

#define BCM_GPIO_FS_IN 0x0   /* Pin is input */
#define BCM_GPIO_FS_OUT 0x1  /* Pin is output */
#define BCM_GPIO_FS_ALT0 0x4 /* Pin is alternate func 0 */
#define BCM_GPIO_FS_ALT1 0x5 /* Pin is alternate func 1 */
#define BCM_GPIO_FS_ALT2 0x6 /* Pin is alternate func 2 */
#define BCM_GPIO_FS_ALT3 0x7 /* Pin is alternate func 3 */
#define BCM_GPIO_FS_ALT4 0x3 /* Pin is alternate func 4 */
#define BCM_GPIO_FS_ALT5 0x2 /* Pin is alternate func 5 */

/* Any register that does not have specific bit masks listed here is omitted
 * because the control is done via a single bit, which corresponds bit N to
 * pin N. It is up to the programmer to read the data sheet to know whether
 * to use register 0, 1, ..., n.
 */

#define BCM_GPIO_NORES 0x0    /* No resistor */
#define BCM_GPIO_PULLUP 0x1   /* Pull-up resistor */
#define BCM_GPIO_PULLDOWN 0x2 /* Pull-down resistor */

/* TODO: how to encode alternative function table to preproc definitions? */

#endif // __ARCH_ARM64_SRC_BCM2711_GPIO_H
