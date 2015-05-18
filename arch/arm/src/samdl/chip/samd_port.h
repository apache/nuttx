/********************************************************************************************
 * arch/arm/src/samdl/chip/samd_port.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J–SAM–12/2013
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMDL_CHIP_SAMD_PORT_H
#define __ARCH_ARM_SRC_SAMDL_CHIP_SAMD_PORT_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD20

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* PORT register offsets ********************************************************************/

#define SAM_PORTA                 (0)
#define SAM_PORTB                 (1)

#define SAM_PORTN_OFFSET(n)       (0x0000 + ((n) << 7))
#define SAM_PORTA_OFFSET          0x0000 /* Port A register offset */
#define SAM_PORTB_OFFSET          0x0080 /* Port B register offset */

#define SAM_PORT_DIR_OFFSET       0x0000 /* Data direction register */
#define SAM_PORT_DIRCLR_OFFSET    0x0004 /* Data direction clear register */
#define SAM_PORT_DIRSET_OFFSET    0x0008 /* Data direction set register */
#define SAM_PORT_DIRTGL_OFFSET    0x000c /* Data direction toggle register */
#define SAM_PORT_OUT_OFFSET       0x0010 /* Data output value register */
#define SAM_PORT_OUTCLR_OFFSET    0x0014 /* Data output value clear register */
#define SAM_PORT_OUTSET_OFFSET    0x0018 /* Data output value set register */
#define SAM_PORT_OUTTGL_OFFSET    0x001c /* Data output value toggle register */
#define SAM_PORT_IN_OFFSET        0x0020 /* Data input value register */
#define SAM_PORT_CTRL_OFFSET      0x0024 /* Control register */
#define SAM_PORT_WRCONFIG_OFFSET  0x0028 /* Write configuration registers */

#define SAM_PORT_PMUX_OFFSET(n)   (0x0030+((n)>>1))
#define SAM_PORT_PMUX0_OFFSET     0x0030 /* Peripheral multiplexing register 0 */
#define SAM_PORT_PMUX1_OFFSET     0x0031 /* Peripheral multiplexing register 1 */
#define SAM_PORT_PMUX2_OFFSET     0x0032 /* Peripheral multiplexing register 2 */
#define SAM_PORT_PMUX3_OFFSET     0x0033 /* Peripheral multiplexing register 3 */
#define SAM_PORT_PMUX4_OFFSET     0x0034 /* Peripheral multiplexing register 4 */
#define SAM_PORT_PMUX5_OFFSET     0x0035 /* Peripheral multiplexing register 5 */
#define SAM_PORT_PMUX6_OFFSET     0x0036 /* Peripheral multiplexing register 6 */
#define SAM_PORT_PMUX7_OFFSET     0x0037 /* Peripheral multiplexing register 7 */
#define SAM_PORT_PMUX8_OFFSET     0x0038 /* Peripheral multiplexing register 8 */
#define SAM_PORT_PMUX9_OFFSET     0x0039 /* Peripheral multiplexing register 9 */
#define SAM_PORT_PMUX10_OFFSET    0x003a /* Peripheral multiplexing register 10 */
#define SAM_PORT_PMUX11_OFFSET    0x003b /* Peripheral multiplexing register 11 */
#define SAM_PORT_PMUX12_OFFSET    0x003c /* Peripheral multiplexing register 12 */
#define SAM_PORT_PMUX13_OFFSET    0x003d /* Peripheral multiplexing register 13 */
#define SAM_PORT_PMUX14_OFFSET    0x003e /* Peripheral multiplexing register 14 */
#define SAM_PORT_PMUX15_OFFSET    0x003f /* Peripheral multiplexing register 15 */

#define SAM_PORT_PINCFG_OFFSET(n) (0x0040+(n))
#define SAM_PORT_PINCFG0_OFFSET   0x0040 /* Pin configuration register 0 */
#define SAM_PORT_PINCFG1_OFFSET   0x0041 /* Pin configuration register 1 */
#define SAM_PORT_PINCFG2_OFFSET   0x0042 /* Pin configuration register 2 */
#define SAM_PORT_PINCFG3_OFFSET   0x0043 /* Pin configuration register 3 */
#define SAM_PORT_PINCFG4_OFFSET   0x0044 /* Pin configuration register 4 */
#define SAM_PORT_PINCFG5_OFFSET   0x0045 /* Pin configuration register 5 */
#define SAM_PORT_PINCFG6_OFFSET   0x0046 /* Pin configuration register 6 */
#define SAM_PORT_PINCFG7_OFFSET   0x0047 /* Pin configuration register 7 */
#define SAM_PORT_PINCFG8_OFFSET   0x0048 /* Pin configuration register 8 */
#define SAM_PORT_PINCFG9_OFFSET   0x0049 /* Pin configuration register 9 */
#define SAM_PORT_PINCFG10_OFFSET  0x004a /* Pin configuration register 10 */
#define SAM_PORT_PINCFG11_OFFSET  0x004b /* Pin configuration register 11 */
#define SAM_PORT_PINCFG12_OFFSET  0x004c /* Pin configuration register 12 */
#define SAM_PORT_PINCFG13_OFFSET  0x004d /* Pin configuration register 13 */
#define SAM_PORT_PINCFG14_OFFSET  0x004e /* Pin configuration register 14 */
#define SAM_PORT_PINCFG15_OFFSET  0x004f /* Pin configuration register 15 */
#define SAM_PORT_PINCFG16_OFFSET  0x0050 /* Pin configuration register 16 */
#define SAM_PORT_PINCFG17_OFFSET  0x0051 /* Pin configuration register 17 */
#define SAM_PORT_PINCFG18_OFFSET  0x0052 /* Pin configuration register 18 */
#define SAM_PORT_PINCFG19_OFFSET  0x0053 /* Pin configuration register 19 */
#define SAM_PORT_PINCFG20_OFFSET  0x0054 /* Pin configuration register 20 */
#define SAM_PORT_PINCFG21_OFFSET  0x0055 /* Pin configuration register 21 */
#define SAM_PORT_PINCFG22_OFFSET  0x0056 /* Pin configuration register 22 */
#define SAM_PORT_PINCFG23_OFFSET  0x0057 /* Pin configuration register 23 */
#define SAM_PORT_PINCFG24_OFFSET  0x0058 /* Pin configuration register 24 */
#define SAM_PORT_PINCFG25_OFFSET  0x0059 /* Pin configuration register 25 */
#define SAM_PORT_PINCFG26_OFFSET  0x005a /* Pin configuration register 26 */
#define SAM_PORT_PINCFG27_OFFSET  0x005b /* Pin configuration register 27 */
#define SAM_PORT_PINCFG28_OFFSET  0x005c /* Pin configuration register 28 */
#define SAM_PORT_PINCFG29_OFFSET  0x005d /* Pin configuration register 29 */
#define SAM_PORT_PINCFG30_OFFSET  0x005e /* Pin configuration register 30 */
#define SAM_PORT_PINCFG31_OFFSET  0x005f /* Pin configuration register 31 */

/* PORT register addresses ******************************************************************/

#define SAM_PORTN_BASE(n)         (SAM_PORT_BASE+SAM_PORTN_OFFSET(n))
#define SAM_PORTA_BASE            (SAM_PORT_BASE+SAM_PORTA_OFFSET)
#define SAM_PORTB_BASE            (SAM_PORT_BASE+SAM_PORTB_OFFSET)

#define SAM_PORTA_DIR             (SAM_PORTA_BASE+SAM_PORT_DIR_OFFSET)
#define SAM_PORTA_DIRCLR          (SAM_PORTA_BASE+SAM_PORT_DIRCLR_OFFSET)
#define SAM_PORTA_DIRSET          (SAM_PORTA_BASE+SAM_PORT_DIRSET_OFFSET)
#define SAM_PORTA_DIRTGL          (SAM_PORTA_BASE+SAM_PORT_DIRTGL_OFFSET)
#define SAM_PORTA_OUT             (SAM_PORTA_BASE+SAM_PORT_OUT_OFFSET)
#define SAM_PORTA_OUTCLR          (SAM_PORTA_BASE+SAM_PORT_OUTCLR_OFFSET)
#define SAM_PORTA_OUTSET          (SAM_PORTA_BASE+SAM_PORT_OUTSET_OFFSET)
#define SAM_PORTA_OUTTGL          (SAM_PORTA_BASE+SAM_PORT_OUTTGL_OFFSET)
#define SAM_PORTA_IN              (SAM_PORTA_BASE+SAM_PORT_IN_OFFSET)
#define SAM_PORTA_CTRL            (SAM_PORTA_BASE+SAM_PORT_CTRL_OFFSET)
#define SAM_PORTA_WRCONFIG        (SAM_PORTA_BASE+SAM_PORT_WRCONFIG_OFFSET)

#define SAM_PORTA_PMUX(n)         (SAM_PORTA_BASE+SAM_PORT_PMUX_OFFSET(n))
#define SAM_PORTA_PMUX0           (SAM_PORTA_BASE+SAM_PORT_PMUX0_OFFSET)
#define SAM_PORTA_PMUX1           (SAM_PORTA_BASE+SAM_PORT_PMUX1_OFFSET)
#define SAM_PORTA_PMUX2           (SAM_PORTA_BASE+SAM_PORT_PMUX2_OFFSET)
#define SAM_PORTA_PMUX3           (SAM_PORTA_BASE+SAM_PORT_PMUX3_OFFSET)
#define SAM_PORTA_PMUX4           (SAM_PORTA_BASE+SAM_PORT_PMUX4_OFFSET)
#define SAM_PORTA_PMUX5           (SAM_PORTA_BASE+SAM_PORT_PMUX5_OFFSET)
#define SAM_PORTA_PMUX6           (SAM_PORTA_BASE+SAM_PORT_PMUX6_OFFSET)
#define SAM_PORTA_PMUX7           (SAM_PORTA_BASE+SAM_PORT_PMUX7_OFFSET)
#define SAM_PORTA_PMUX8           (SAM_PORTA_BASE+SAM_PORT_PMUX8_OFFSET)
#define SAM_PORTA_PMUX9           (SAM_PORTA_BASE+SAM_PORT_PMUX9_OFFSET)
#define SAM_PORTA_PMUX10          (SAM_PORTA_BASE+SAM_PORT_PMUX10_OFFSET)
#define SAM_PORTA_PMUX11          (SAM_PORTA_BASE+SAM_PORT_PMUX11_OFFSET)
#define SAM_PORTA_PMUX12          (SAM_PORTA_BASE+SAM_PORT_PMUX12_OFFSET)
#define SAM_PORTA_PMUX13          (SAM_PORTA_BASE+SAM_PORT_PMUX13_OFFSET)
#define SAM_PORTA_PMUX14          (SAM_PORTA_BASE+SAM_PORT_PMUX14_OFFSET)
#define SAM_PORTA_PMUX15          (SAM_PORTA_BASE+SAM_PORT_PMUX15_OFFSET)

#define SAM_PORTA_PINCFG(n)       (SAM_PORTA_BASE+SAM_PORT_PINCFG_OFFSET(n))
#define SAM_PORTA_PINCFG0         (SAM_PORTA_BASE+SAM_PORT_PINCFG0_OFFSET)
#define SAM_PORTA_PINCFG1         (SAM_PORTA_BASE+SAM_PORT_PINCFG1_OFFSET)
#define SAM_PORTA_PINCFG2         (SAM_PORTA_BASE+SAM_PORT_PINCFG2_OFFSET)
#define SAM_PORTA_PINCFG3         (SAM_PORTA_BASE+SAM_PORT_PINCFG3_OFFSET)
#define SAM_PORTA_PINCFG4         (SAM_PORTA_BASE+SAM_PORT_PINCFG4_OFFSET)
#define SAM_PORTA_PINCFG5         (SAM_PORTA_BASE+SAM_PORT_PINCFG5_OFFSET)
#define SAM_PORTA_PINCFG6         (SAM_PORTA_BASE+SAM_PORT_PINCFG6_OFFSET)
#define SAM_PORTA_PINCFG7         (SAM_PORTA_BASE+SAM_PORT_PINCFG7_OFFSET)
#define SAM_PORTA_PINCFG8         (SAM_PORTA_BASE+SAM_PORT_PINCFG8_OFFSET)
#define SAM_PORTA_PINCFG9         (SAM_PORTA_BASE+SAM_PORT_PINCFG9_OFFSET)
#define SAM_PORTA_PINCFG10        (SAM_PORTA_BASE+SAM_PORT_PINCFG10_OFFSET)
#define SAM_PORTA_PINCFG11        (SAM_PORTA_BASE+SAM_PORT_PINCFG11_OFFSET)
#define SAM_PORTA_PINCFG12        (SAM_PORTA_BASE+SAM_PORT_PINCFG12_OFFSET)
#define SAM_PORTA_PINCFG13        (SAM_PORTA_BASE+SAM_PORT_PINCFG13_OFFSET)
#define SAM_PORTA_PINCFG14        (SAM_PORTA_BASE+SAM_PORT_PINCFG14_OFFSET)
#define SAM_PORTA_PINCFG15        (SAM_PORTA_BASE+SAM_PORT_PINCFG15_OFFSET)
#define SAM_PORTA_PINCFG16        (SAM_PORTA_BASE+SAM_PORT_PINCFG16_OFFSET)
#define SAM_PORTA_PINCFG17        (SAM_PORTA_BASE+SAM_PORT_PINCFG17_OFFSET)
#define SAM_PORTA_PINCFG18        (SAM_PORTA_BASE+SAM_PORT_PINCFG18_OFFSET)
#define SAM_PORTA_PINCFG19        (SAM_PORTA_BASE+SAM_PORT_PINCFG19_OFFSET)
#define SAM_PORTA_PINCFG20        (SAM_PORTA_BASE+SAM_PORT_PINCFG20_OFFSET)
#define SAM_PORTA_PINCFG21        (SAM_PORTA_BASE+SAM_PORT_PINCFG21_OFFSET)
#define SAM_PORTA_PINCFG22        (SAM_PORTA_BASE+SAM_PORT_PINCFG22_OFFSET)
#define SAM_PORTA_PINCFG23        (SAM_PORTA_BASE+SAM_PORT_PINCFG23_OFFSET)
#define SAM_PORTA_PINCFG24        (SAM_PORTA_BASE+SAM_PORT_PINCFG24_OFFSET)
#define SAM_PORTA_PINCFG25        (SAM_PORTA_BASE+SAM_PORT_PINCFG25_OFFSET)
#define SAM_PORTA_PINCFG26        (SAM_PORTA_BASE+SAM_PORT_PINCFG26_OFFSET)
#define SAM_PORTA_PINCFG27        (SAM_PORTA_BASE+SAM_PORT_PINCFG27_OFFSET)
#define SAM_PORTA_PINCFG28        (SAM_PORTA_BASE+SAM_PORT_PINCFG28_OFFSET)
#define SAM_PORTA_PINCFG29        (SAM_PORTA_BASE+SAM_PORT_PINCFG29_OFFSET)
#define SAM_PORTA_PINCFG30        (SAM_PORTA_BASE+SAM_PORT_PINCFG30_OFFSET)
#define SAM_PORTA_PINCFG31        (SAM_PORTA_BASE+SAM_PORT_PINCFG31_OFFSET)

#define SAM_PORTB_DIR             (SAM_PORTB_BASE+SAM_PORT_DIR_OFFSET)
#define SAM_PORTB_DIRCLR          (SAM_PORTB_BASE+SAM_PORT_DIRCLR_OFFSET)
#define SAM_PORTB_DIRSET          (SAM_PORTB_BASE+SAM_PORT_DIRSET_OFFSET)
#define SAM_PORTB_DIRTGL          (SAM_PORTB_BASE+SAM_PORT_DIRTGL_OFFSET)
#define SAM_PORTB_OUT             (SAM_PORTB_BASE+SAM_PORT_OUT_OFFSET)
#define SAM_PORTB_OUTCLR          (SAM_PORTB_BASE+SAM_PORT_OUTCLR_OFFSET)
#define SAM_PORTB_OUTSET          (SAM_PORTB_BASE+SAM_PORT_OUTSET_OFFSET)
#define SAM_PORTB_OUTTGL          (SAM_PORTB_BASE+SAM_PORT_OUTTGL_OFFSET)
#define SAM_PORTB_IN              (SAM_PORTB_BASE+SAM_PORT_IN_OFFSET)
#define SAM_PORTB_CTRL            (SAM_PORTB_BASE+SAM_PORT_CTRL_OFFSET)
#define SAM_PORTB_WRCONFIG        (SAM_PORTB_BASE+SAM_PORT_WRCONFIG_OFFSET)

#define SAM_PORTB_PMUX(n)         (SAM_PORTB_BASE+SAM_PORT_PMUX_OFFSET(n))
#define SAM_PORTB_PMUX0           (SAM_PORTB_BASE+SAM_PORT_PMUX0_OFFSET)
#define SAM_PORTB_PMUX1           (SAM_PORTB_BASE+SAM_PORT_PMUX1_OFFSET)
#define SAM_PORTB_PMUX2           (SAM_PORTB_BASE+SAM_PORT_PMUX2_OFFSET)
#define SAM_PORTB_PMUX3           (SAM_PORTB_BASE+SAM_PORT_PMUX3_OFFSET)
#define SAM_PORTB_PMUX4           (SAM_PORTB_BASE+SAM_PORT_PMUX4_OFFSET)
#define SAM_PORTB_PMUX5           (SAM_PORTB_BASE+SAM_PORT_PMUX5_OFFSET)
#define SAM_PORTB_PMUX6           (SAM_PORTB_BASE+SAM_PORT_PMUX6_OFFSET)
#define SAM_PORTB_PMUX7           (SAM_PORTB_BASE+SAM_PORT_PMUX7_OFFSET)
#define SAM_PORTB_PMUX8           (SAM_PORTB_BASE+SAM_PORT_PMUX8_OFFSET)
#define SAM_PORTB_PMUX9           (SAM_PORTB_BASE+SAM_PORT_PMUX9_OFFSET)
#define SAM_PORTB_PMUX10          (SAM_PORTB_BASE+SAM_PORT_PMUX10_OFFSET)
#define SAM_PORTB_PMUX11          (SAM_PORTB_BASE+SAM_PORT_PMUX11_OFFSET)
#define SAM_PORTB_PMUX12          (SAM_PORTB_BASE+SAM_PORT_PMUX12_OFFSET)
#define SAM_PORTB_PMUX13          (SAM_PORTB_BASE+SAM_PORT_PMUX13_OFFSET)
#define SAM_PORTB_PMUX14          (SAM_PORTB_BASE+SAM_PORT_PMUX14_OFFSET)
#define SAM_PORTB_PMUX15          (SAM_PORTB_BASE+SAM_PORT_PMUX15_OFFSET)

#define SAM_PORTB_PINCFG(n)       (SAM_PORTB_BASE+SAM_PORT_PINCFG_OFFSET(n))
#define SAM_PORTB_PINCFG0         (SAM_PORTB_BASE+SAM_PORT_PINCFG0_OFFSET)
#define SAM_PORTB_PINCFG1         (SAM_PORTB_BASE+SAM_PORT_PINCFG1_OFFSET)
#define SAM_PORTB_PINCFG2         (SAM_PORTB_BASE+SAM_PORT_PINCFG2_OFFSET)
#define SAM_PORTB_PINCFG3         (SAM_PORTB_BASE+SAM_PORT_PINCFG3_OFFSET)
#define SAM_PORTB_PINCFG4         (SAM_PORTB_BASE+SAM_PORT_PINCFG4_OFFSET)
#define SAM_PORTB_PINCFG5         (SAM_PORTB_BASE+SAM_PORT_PINCFG5_OFFSET)
#define SAM_PORTB_PINCFG6         (SAM_PORTB_BASE+SAM_PORT_PINCFG6_OFFSET)
#define SAM_PORTB_PINCFG7         (SAM_PORTB_BASE+SAM_PORT_PINCFG7_OFFSET)
#define SAM_PORTB_PINCFG8         (SAM_PORTB_BASE+SAM_PORT_PINCFG8_OFFSET)
#define SAM_PORTB_PINCFG9         (SAM_PORTB_BASE+SAM_PORT_PINCFG9_OFFSET)
#define SAM_PORTB_PINCFG10        (SAM_PORTB_BASE+SAM_PORT_PINCFG10_OFFSET)
#define SAM_PORTB_PINCFG11        (SAM_PORTB_BASE+SAM_PORT_PINCFG11_OFFSET)
#define SAM_PORTB_PINCFG12        (SAM_PORTB_BASE+SAM_PORT_PINCFG12_OFFSET)
#define SAM_PORTB_PINCFG13        (SAM_PORTB_BASE+SAM_PORT_PINCFG13_OFFSET)
#define SAM_PORTB_PINCFG14        (SAM_PORTB_BASE+SAM_PORT_PINCFG14_OFFSET)
#define SAM_PORTB_PINCFG15        (SAM_PORTB_BASE+SAM_PORT_PINCFG15_OFFSET)
#define SAM_PORTB_PINCFG16        (SAM_PORTB_BASE+SAM_PORT_PINCFG16_OFFSET)
#define SAM_PORTB_PINCFG17        (SAM_PORTB_BASE+SAM_PORT_PINCFG17_OFFSET)
#define SAM_PORTB_PINCFG18        (SAM_PORTB_BASE+SAM_PORT_PINCFG18_OFFSET)
#define SAM_PORTB_PINCFG19        (SAM_PORTB_BASE+SAM_PORT_PINCFG19_OFFSET)
#define SAM_PORTB_PINCFG20        (SAM_PORTB_BASE+SAM_PORT_PINCFG20_OFFSET)
#define SAM_PORTB_PINCFG21        (SAM_PORTB_BASE+SAM_PORT_PINCFG21_OFFSET)
#define SAM_PORTB_PINCFG22        (SAM_PORTB_BASE+SAM_PORT_PINCFG22_OFFSET)
#define SAM_PORTB_PINCFG23        (SAM_PORTB_BASE+SAM_PORT_PINCFG23_OFFSET)
#define SAM_PORTB_PINCFG24        (SAM_PORTB_BASE+SAM_PORT_PINCFG24_OFFSET)
#define SAM_PORTB_PINCFG25        (SAM_PORTB_BASE+SAM_PORT_PINCFG25_OFFSET)
#define SAM_PORTB_PINCFG26        (SAM_PORTB_BASE+SAM_PORT_PINCFG26_OFFSET)
#define SAM_PORTB_PINCFG27        (SAM_PORTB_BASE+SAM_PORT_PINCFG27_OFFSET)
#define SAM_PORTB_PINCFG28        (SAM_PORTB_BASE+SAM_PORT_PINCFG28_OFFSET)
#define SAM_PORTB_PINCFG29        (SAM_PORTB_BASE+SAM_PORT_PINCFG29_OFFSET)
#define SAM_PORTB_PINCFG30        (SAM_PORTB_BASE+SAM_PORT_PINCFG30_OFFSET)
#define SAM_PORTB_PINCFG31        (SAM_PORTB_BASE+SAM_PORT_PINCFG31_OFFSET)

/* PORT register bit definitions ************************************************************/

/* Data direction, data direction clear,  data direction set, and data direction toggle
 * registers
 */

#define PORT_DIR(n)               (1 << n) /* Port data n, direction, n=0-31 */

/* Data output value, data output value clear, data output value set, and data output
 * value toggle registers
 */

#define PORT_OUT(n)               (1 << n) /* Port data n output value, n=0-31 */

/* Data input value register */

#define PORT_IN(n)                (1 << n) /* Port n data input value, n=0-31 */

/* Control register */

#define PORT_CTRL(n)              (1 << n) /* Port n input sampling mode, n=0-31 */

/* Write configuration registers */

#define PORT_WRCONFIG_PINMASK_SHIFT (0) /* Bits 0-15:  Pin Mask for Multiple Pin Configuration */
#define PORT_WRCONFIG_PINMASK_MASK  (0xffff << PORT_WRCONFIG_PINMASK_SHIFT)
#  define PORT_WRCONFIG_PINMASK(n)  (1 << (PORT_WRCONFIG_PINMASK_SHIFT+(n)))
#define PORT_WRCONFIG_PMUXEN      (1 << 16) /* Bit 16: Peripheral Multiplexer Enable */
#define PORT_WRCONFIG_INEN        (1 << 17) /* Bit 17: Input Enable */
#define PORT_WRCONFIG_PULLEN      (1 << 18) /* Bit 18: Pull Enable */
#define PORT_WRCONFIG_DRVSTR      (1 << 22) /* Bit 22: Output Driver Strength Selection */
#define PORT_WRCONFIG_PMUX_SHIFT  (24)      /* Bits 24-27: Peripheral Multiplexing */
#define PORT_WRCONFIG_PMUX_MASK   (15 << PORT_WRCONFIG_PMUX_SHIFT)
#define PORT_WRCONFIG_WRPMUX      (1 << 28) /* Bit 28: Write PMUX */
#define PORT_WRCONFIG_WRPINCFG    (1 << 30) /* Bit 30: Write PINCFG */
#define PORT_WRCONFIG_HWSEL       (1 << 31) /* Bit 31: Half-Word Select */

/* Peripheral multiplexing registers */

#define PORT_PMUX_PERIPHA         0x00      /* Peripheral function A */
#define PORT_PMUX_PERIPHB         0x01      /* Peripheral function B */
#define PORT_PMUX_PERIPHC         0x02      /* Peripheral function C */
#define PORT_PMUX_PERIPHD         0x03      /* Peripheral function D */
#define PORT_PMUX_PERIPHE         0x04      /* Peripheral function E */
#define PORT_PMUX_PERIPHF         0x05      /* Peripheral function F */
#define PORT_PMUX_PERIPHG         0x06      /* Peripheral function G */
#define PORT_PMUX_PERIPHH         0x07      /* Peripheral function H */

/* Pin configuration registers */

#define PORT_PINCFG_PMUXEN        (1 << 0)  /* Bit 0: Peripheral Multiplexer Enable */
#define PORT_PINCFG_INEN          (1 << 1)  /* Bit 1: Input Enable */
#define PORT_PINCFG_PULLEN        (1 << 2)  /* Bit 2: Pull Enable */
#define PORT_PINCFG_DRVSTR        (1 << 6)  /* Bit 6: Output Driver Strength Selection */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 */
#endif /* __ARCH_ARM_SRC_SAMDL_CHIP_SAMD_PORT_H */
