/************************************************************************************
 * arch/arm/src/bcm2708/chip/bcm2708_gpio.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_SRC_BCM2708_CHIP_BCM2708_GPIO_H
#define __ARCH_ARM_SRC_BCM2708_CHIP_BCM2708_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip/bcm2708_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPIO Register Offsets ************************************************************/

#define BCM_GPIO_GPFSEL_INDEX(n)       ((n) / 10)
#define BCM_GPIO_GPFSEL_FIELD(n)       ((n) % 10)

#define BCM_GPIO_GPFSEL_OFFSET(n)      (0x0000 + (BCM_GPIO_GPFSEL_INDEX(n) << 2))
#  define BCM_GPIO_GPFSEL0_OFFSET      0x0000  /* GPIO Function Select 0 */
#  define BCM_GPIO_GPFSEL1_OFFSET      0x0004  /* GPIO Function Select 1 */
#  define BCM_GPIO_GPFSEL2_OFFSET      0x0008  /* GPIO Function Select 2 */
#  define BCM_GPIO_GPFSEL3_OFFSET      0x000c  /* GPIO Function Select 3 */
#  define BCM_GPIO_GPFSEL4_OFFSET      0x0010  /* GPIO Function Select 4 */
#  define BCM_GPIO_GPFSEL5_OFFSET      0x0014  /* GPIO Function Select 5 */
#define BCM_GPIO_GPSET_OFFSET(n)       (0x001c + (((n) >> 5) << 2))
#  define BCM_GPIO_GPSET0_OFFSET       0x001c  /* GPIO Pin Output Set 0 */
#  define BCM_GPIO_GPSET1_OFFSET       0x0020  /* GPIO Pin Output Set 1 */
#define BCM_GPIO_GPCLR_OFFSET(n)       (0x0028 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPCLR0_OFFSET       0x0028  /* GPIO Pin Output Clear 0 */
#  define BCM_GPIO_GPCLR1_OFFSET       0x002c  /* GPIO Pin Output Clear 1 */
#define BCM_GPIO_GPLEV_OFFSET(n)       (0x0034 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPLEV0_OFFSET       0x0034  /* GPIO Pin Level 0 */
#  define BCM_GPIO_GPLEV1_OFFSET       0x0038  /* GPIO Pin Level 1 */
#define BCM_GPIO_GPEDS_OFFSET(n)       (0x0040 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPEDS0_OFFSET       0x0040  /* GPIO Pin Event Detect Status 0 */
#  define BCM_GPIO_GPEDS1_OFFSET       0x0044  /* GPIO Pin Event Detect Status 1 */
#define BCM_GPIO_GPREN_OFFSET(n)       (0x004c + (((n) >> 5) << 2))
#  define BCM_GPIO_GPREN0_OFFSET       0x004c  /* GPIO Pin Rising Edge Detect Enable 0 */
#  define BCM_GPIO_GPREN1_OFFSET       0x0050  /* GPIO Pin Rising Edge Detect Enable 1 */
#define BCM_GPIO_GPFEN_OFFSET(n)       (0x0058 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPFEN0_OFFSET       0x0058  /* GPIO Pin Falling Edge Detect Enable 0 */
#  define BCM_GPIO_GPFEN1_OFFSET       0x005c  /* GPIO Pin Falling Edge Detect Enable 1 */
#define BCM_GPIO_GPHEN_OFFSET(n)       (0x0064 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPHEN0_OFFSET       0x0064  /* GPIO Pin High Detect Enable 0 */
#  define BCM_GPIO_GPHEN1_OFFSET       0x0068  /* GPIO Pin High Detect Enable 1 */
#define BCM_GPIO_GPLEN_OFFSET(n)       (0x0070 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPLEN0_OFFSET       0x0070  /* GPIO Pin Low Detect Enable 0 */
#  define BCM_GPIO_GPLEN1_OFFSET       0x0074  /* GPIO Pin Low Detect Enable 1 */
#define BCM_GPIO_GPAREN_OFFSET(n)      (0x007c + (((n) >> 5) << 2))
#  define BCM_GPIO_GPAREN0_OFFSET      0x007c  /* GPIO Pin Async. Rising Edge Detect 0 */
#  define BCM_GPIO_GPAREN1_OFFSET      0x0080  /* GPIO Pin Async. Rising Edge Detect 1 */
#define BCM_GPIO_GPAFEN_OFFSET(n)      (0x0088 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPAFEN0_OFFSET      0x0088  /* GPIO Pin Async. Falling Edge Detect 0 */
#  define BCM_GPIO_GPAFEN1_OFFSET      0x008c  /* GPIO Pin Async. Falling Edge Detect 1 */
#define BCM_GPIO_GPPUD_OFFSET          0x0094  /* GPIO Pin Pull-up/down Enable */
#define BCM_GPIO_GPPUDCLK_OFFSET(n)    (0x0098 + (((n) >> 5) << 2))
#  define BCM_GPIO_GPPUDCLK0_OFFSET    0x0098  /* GPIO Pin Pull-up/down Enable Clock 0 */
#  define BCM_GPIO_GPPUDCLK1_OFFSET    0x009c  /* GPIO Pin Pull-up/down Enable Clock 1 */

/* GPIO Register Addresses **********************************************************/

#define BCM_GPIO_GPFSEL(n)             (BCM_GPIO_VBASE+BCM_GPIO_GPFSEL_OFFSET(n))
#  define BCM_GPIO_GPFSEL0             (BCM_GPIO_VBASE+BCM_GPIO_GPFSEL0_OFFSET)
#  define BCM_GPIO_GPFSEL1             (BCM_GPIO_VBASE+BCM_GPIO_GPFSEL1_OFFSET)
#  define BCM_GPIO_GPFSEL2             (BCM_GPIO_VBASE+BCM_GPIO_GPFSEL2_OFFSET)
#  define BCM_GPIO_GPFSEL3             (BCM_GPIO_VBASE+BCM_GPIO_GPFSEL3_OFFSET)
#  define BCM_GPIO_GPFSEL4             (BCM_GPIO_VBASE+BCM_GPIO_GPFSEL4_OFFSET)
#  define BCM_GPIO_GPFSEL5             (BCM_GPIO_VBASE+BCM_GPIO_GPFSEL5_OFFSET)
#define BCM_GPIO_GPSET(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPSET_OFFSET(n))
#  define BCM_GPIO_GPSET0              (BCM_GPIO_VBASE+BCM_BCM_GPIO_GPSET0_OFFSET)
#  define BCM_GPIO_GPSET1              (BCM_GPIO_VBASE+BCM_BCM_GPIO_GPSET1_OFFSET)
#define BCM_GPIO_GPCLR(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPCLR_OFFSET(n))
#  define BCM_GPIO_GPCLR0              (BCM_GPIO_VBASE+BCM_GPIO_GPCLR0_OFFSET)
#  define BCM_GPIO_GPCLR1              (BCM_GPIO_VBASE+BCM_GPIO_GPCLR1_OFFSET)
#define BCM_GPIO_GPLEV(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPLEV_OFFSET(n))
#  define BCM_GPIO_GPLEV0              (BCM_GPIO_VBASE+BCM_GPIO_GPLEV0_OFFSET)
#  define BCM_GPIO_GPLEV1              (BCM_GPIO_VBASE+BCM_GPIO_GPLEV1_OFFSET)
#define BCM_GPIO_GPEDS(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPEDS_OFFSET(n))
#  define BCM_GPIO_GPEDS0              (BCM_GPIO_VBASE+BCM_GPIO_GPEDS0_OFFSET)
#  define BCM_GPIO_GPEDS1              (BCM_GPIO_VBASE+BCM_GPIO_GPEDS1_OFFSET)
#define BCM_GPIO_GPREN(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPREN_OFFSET(n))
#  define BCM_GPIO_GPREN0              (BCM_GPIO_VBASE+BCM_GPIO_GPREN0_OFFSET)
#  define BCM_GPIO_GPREN1              (BCM_GPIO_VBASE+BCM_GPIO_GPREN1_OFFSET)
#define BCM_GPIO_GPFEN(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPFEN_OFFSET(n))
#  define BCM_GPIO_GPFEN0              (BCM_GPIO_VBASE+BCM_GPIO_GPFEN0_OFFSET)
#  define BCM_GPIO_GPFEN1              (BCM_GPIO_VBASE+BCM_GPIO_GPFEN1_OFFSET)
#define BCM_GPIO_GPHEN(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPHEN_OFFSET(n))
#  define BCM_GPIO_GPHEN0              (BCM_GPIO_VBASE+BCM_GPIO_GPHEN0_OFFSET)
#  define BCM_GPIO_GPHEN1              (BCM_GPIO_VBASE+BCM_GPIO_GPHEN1_OFFSET)
#define BCM_GPIO_GPLEN(n)              (BCM_GPIO_VBASE+BCM_GPIO_GPLEN_OFFSET(n))
#  define BCM_GPIO_GPLEN0              (BCM_GPIO_VBASE+BCM_GPIO_GPLEN0_OFFSET)
#  define BCM_GPIO_GPLEN1              (BCM_GPIO_VBASE+BCM_GPIO_GPLEN1_OFFSET)
#define BCM_GPIO_GPAREN(n)             (BCM_GPIO_VBASE+BCM_GPIO_GPAREN_OFFSET(n))
#  define BCM_GPIO_GPAREN0             (BCM_GPIO_VBASE+BCM_GPIO_GPAREN0_OFFSET)
#  define BCM_GPIO_GPAREN1             (BCM_GPIO_VBASE+BCM_GPIO_GPAREN1_OFFSET)
#define BCM_GPIO_GPAFEN(n)             (BCM_GPIO_VBASE+BCM_GPIO_GPAFEN_OFFSET(n))
#  define BCM_GPIO_GPAFEN0             (BCM_GPIO_VBASE+BCM_GPIO_GPAFEN0_OFFSET)
#  define BCM_GPIO_GPAFEN1             (BCM_GPIO_VBASE+BCM_GPIO_GPAFEN1_OFFSET)
#define BCM_GPIO_GPPUD                 (BCM_GPIO_VBASE+BCM_GPIO_GPPUD_OFFSET)
#define BCM_GPIO_GPPUDCLK(n)           (BCM_GPIO_VBASE+BCM_GPIO_GPPUDCLK_OFFSET(n))
#  define BCM_GPIO_GPPUDCLK0           (BCM_GPIO_VBASE+BCM_GPIO_GPPUDCLK0_OFFSET)
#  define BCM_GPIO_GPPUDCLK1           (BCM_GPIO_VBASE+BCM_GPIO_GPPUDCLK1_OFFSET)

/* GPIO Register Bit Definitions ****************************************************/

#define BCM_GPIO_GPFSEL_FSEL_SHIFT(n)   (3 * BCM_GPIO_GPFSEL_FIELD(n))
#define BCM_GPIO_GPFSEL_FSEL_MASK(n)    (7 << BCM_GPIO_GPFSEL_FSEL_SHIFT(n))
#  define BCM_GPIO_GPFSEL_FSEL(n,f)     ((uint32_t)(f) << BCM_GPIO_GPFSEL_FSEL_SHIFT(n))
#  define BCM_GPIO_GPFSEL_FSEL_INPUT    (0)
#  define BCM_GPIO_GPFSEL_FSEL_OUTPUT   (1)
#  define BCM_GPIO_GPFSEL_FSEL_ALT0     (4)
#  define BCM_GPIO_GPFSEL_FSEL_ALT1     (5)
#  define BCM_GPIO_GPFSEL_FSEL_ALT2     (6)
#  define BCM_GPIO_GPFSEL_FSEL_ALT3     (7)
#  define BCM_GPIO_GPFSEL_FSEL_ALT4     (3)
#  define BCM_GPIO_GPFSEL_FSEL_ALT5     (2)

#define BCM_GPIO_GPFSEL_FSELx0_SHIFT    0
#define BCM_GPIO_GPFSEL_FSELx0_MASK     (7 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx0_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx0_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx1_SHIFT    3
#define BCM_GPIO_GPFSEL_FSELx1_MASK     (7 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx1_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx1_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx2_SHIFT    6
#define BCM_GPIO_GPFSEL_FSELx2_MASK     (7 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx2_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx2_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx3_SHIFT    9
#define BCM_GPIO_GPFSEL_FSELx3_MASK     (7 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx3_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx3_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx4_SHIFT    12
#define BCM_GPIO_GPFSEL_FSELx4_MASK     (7 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx4_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx4_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx5_SHIFT    15
#define BCM_GPIO_GPFSEL_FSELx5_MASK     (7 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx5_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx5_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx6_SHIFT    18
#define BCM_GPIO_GPFSEL_FSELx6_MASK     (7 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx6_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx6_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx7_SHIFT    21
#define BCM_GPIO_GPFSEL_FSELx7_MASK     (7 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx7_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx7_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx8_SHIFT    24
#define BCM_GPIO_GPFSEL_FSELx8_MASK     (7 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx8_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx8_SHIFT)
#define BCM_GPIO_GPFSEL_FSELx9_SHIFT    27
#define BCM_GPIO_GPFSEL_FSELx9_MASK     (7 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_INPUT  (0 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_OUTPUT (1 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_ALT0   (4 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_ALT1   (5 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_ALT2   (6 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_ALT3   (7 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_ALT4   (3 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)
#  define BCM_GPIO_GPFSEL_FSELx9_ALT5   (2 << BCM_GPIO_GPFSEL_FSELx9_SHIFT)

#define BCM_GPIO_GPSET_SET(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPCLR_CLR(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPLEV_LEV(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPEDS_EDS(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPREN_REN(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPFEN_FEN(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPHEN_HEN(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPLEN_LEN(n)           (1 << ((n) & 0x1f))
#define BCM_GPIO_GPAREN_AREN(n)         (1 << ((n) & 0x1f))
#define BCM_GPIO_GPAFEN_AFEN(n)         (1 << ((n) & 0x1f))

#define BCM_GPIO_GPPUD_PUD_SHIFT        0 /* bit 0-1: Pull-up/down register */
#define BCM_GPIO_GPPUD_PUD_MASK         (3 << BCM_GPIO_GPPUD_PUD_SHIFT)
#  define BCM_GPIO_GPPUD_PUD_OFF        (0 << BCM_GPIO_GPPUD_PUD_SHIFT) /* Disable Pull-up/down */
#  define BCM_GPIO_GPPUD_PUD_PD         (1 << BCM_GPIO_GPPUD_PUD_SHIFT) /* Enable Pull-down */
#  define BCM_GPIO_GPPUD_PUD_PU         (2 << BCM_GPIO_GPPUD_PUD_SHIFT) /* Enable Pull-up */

#define BCM_GPIO_GPPUD_PUDCLK_SHIFT(n)  ((n) & 0x1f)
#  define BCM_GPIO_GPPUD_PUDCLK_MASK(n) (1 << BCM_GPIO_GPPUD_PUDCLK_SHIFT(n))
#  define BCM_GPIO_GPPUD_PUDCLK(n,v)    ((uint32_t)(v) << BCM_GPIO_GPPUD_PUDCLK_SHIFT(n))

#endif /* __ARCH_ARM_SRC_BCM2708_CHIP_BCM2708_GPIO_H */
