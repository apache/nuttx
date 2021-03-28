/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_port.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_PORT_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_PORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PORT register offsets ****************************************************/

#define SAM_PORTA                   (0)
#define SAM_PORTB                   (1)
#define SAM_PORTC                   (2)
#define SAM_PORTD                   (3)
#define SAM_NPORTS                  (4)

#define SAM_PORTN_OFFSET(n)         (0x0000 + ((n) << 7))
#  define SAM_PORTA_OFFSET          0x0000 /* Port A register offset */
#  define SAM_PORTB_OFFSET          0x0080 /* Port B register offset */
#  define SAM_PORTC_OFFSET          0x0100 /* Port C register offset */
#  define SAM_PORTD_OFFSET          0x0180 /* Port D register offset */

#define SAM_PORT_DIR_OFFSET         0x0000 /* Data direction register */
#define SAM_PORT_DIRCLR_OFFSET      0x0004 /* Data direction clear register */
#define SAM_PORT_DIRSET_OFFSET      0x0008 /* Data direction set register */
#define SAM_PORT_DIRTGL_OFFSET      0x000c /* Data direction toggle register */
#define SAM_PORT_OUT_OFFSET         0x0010 /* Data output value register */
#define SAM_PORT_OUTCLR_OFFSET      0x0014 /* Data output value clear register */
#define SAM_PORT_OUTSET_OFFSET      0x0018 /* Data output value set register */
#define SAM_PORT_OUTTGL_OFFSET      0x001c /* Data output value toggle register */
#define SAM_PORT_IN_OFFSET          0x0020 /* Data input value register */
#define SAM_PORT_CTRL_OFFSET        0x0024 /* Control register */
#define SAM_PORT_WRCONFIG_OFFSET    0x0028 /* Write configuration register */
#define SAM_PORT_EVCTRL_OFFSET      0x002c /* Event input control register */

#define SAM_PORT_PMUX_OFFSET(n)     (0x0030+((n)>>1))
#  define SAM_PORT_PMUX0_OFFSET     0x0030 /* Peripheral multiplexing register 0 */
#  define SAM_PORT_PMUX1_OFFSET     0x0031 /* Peripheral multiplexing register 1 */
#  define SAM_PORT_PMUX2_OFFSET     0x0032 /* Peripheral multiplexing register 2 */
#  define SAM_PORT_PMUX3_OFFSET     0x0033 /* Peripheral multiplexing register 3 */
#  define SAM_PORT_PMUX4_OFFSET     0x0034 /* Peripheral multiplexing register 4 */
#  define SAM_PORT_PMUX5_OFFSET     0x0035 /* Peripheral multiplexing register 5 */
#  define SAM_PORT_PMUX6_OFFSET     0x0036 /* Peripheral multiplexing register 6 */
#  define SAM_PORT_PMUX7_OFFSET     0x0037 /* Peripheral multiplexing register 7 */
#  define SAM_PORT_PMUX8_OFFSET     0x0038 /* Peripheral multiplexing register 8 */
#  define SAM_PORT_PMUX9_OFFSET     0x0039 /* Peripheral multiplexing register 9 */
#  define SAM_PORT_PMUX10_OFFSET    0x003a /* Peripheral multiplexing register 10 */
#  define SAM_PORT_PMUX11_OFFSET    0x003b /* Peripheral multiplexing register 11 */
#  define SAM_PORT_PMUX12_OFFSET    0x003c /* Peripheral multiplexing register 12 */
#  define SAM_PORT_PMUX13_OFFSET    0x003d /* Peripheral multiplexing register 13 */
#  define SAM_PORT_PMUX14_OFFSET    0x003e /* Peripheral multiplexing register 14 */
#  define SAM_PORT_PMUX15_OFFSET    0x003f /* Peripheral multiplexing register 15 */

#define SAM_PORT_PINCFG_OFFSET(n)   (0x0040+(n))
#  define SAM_PORT_PINCFG0_OFFSET   0x0040 /* Pin configuration register 0 */
#  define SAM_PORT_PINCFG1_OFFSET   0x0041 /* Pin configuration register 1 */
#  define SAM_PORT_PINCFG2_OFFSET   0x0042 /* Pin configuration register 2 */
#  define SAM_PORT_PINCFG3_OFFSET   0x0043 /* Pin configuration register 3 */
#  define SAM_PORT_PINCFG4_OFFSET   0x0044 /* Pin configuration register 4 */
#  define SAM_PORT_PINCFG5_OFFSET   0x0045 /* Pin configuration register 5 */
#  define SAM_PORT_PINCFG6_OFFSET   0x0046 /* Pin configuration register 6 */
#  define SAM_PORT_PINCFG7_OFFSET   0x0047 /* Pin configuration register 7 */
#  define SAM_PORT_PINCFG8_OFFSET   0x0048 /* Pin configuration register 8 */
#  define SAM_PORT_PINCFG9_OFFSET   0x0049 /* Pin configuration register 9 */
#  define SAM_PORT_PINCFG10_OFFSET  0x004a /* Pin configuration register 10 */
#  define SAM_PORT_PINCFG11_OFFSET  0x004b /* Pin configuration register 11 */
#  define SAM_PORT_PINCFG12_OFFSET  0x004c /* Pin configuration register 12 */
#  define SAM_PORT_PINCFG13_OFFSET  0x004d /* Pin configuration register 13 */
#  define SAM_PORT_PINCFG14_OFFSET  0x004e /* Pin configuration register 14 */
#  define SAM_PORT_PINCFG15_OFFSET  0x004f /* Pin configuration register 15 */
#  define SAM_PORT_PINCFG16_OFFSET  0x0050 /* Pin configuration register 16 */
#  define SAM_PORT_PINCFG17_OFFSET  0x0051 /* Pin configuration register 17 */
#  define SAM_PORT_PINCFG18_OFFSET  0x0052 /* Pin configuration register 18 */
#  define SAM_PORT_PINCFG19_OFFSET  0x0053 /* Pin configuration register 19 */
#  define SAM_PORT_PINCFG20_OFFSET  0x0054 /* Pin configuration register 20 */
#  define SAM_PORT_PINCFG21_OFFSET  0x0055 /* Pin configuration register 21 */
#  define SAM_PORT_PINCFG22_OFFSET  0x0056 /* Pin configuration register 22 */
#  define SAM_PORT_PINCFG23_OFFSET  0x0057 /* Pin configuration register 23 */
#  define SAM_PORT_PINCFG24_OFFSET  0x0058 /* Pin configuration register 24 */
#  define SAM_PORT_PINCFG25_OFFSET  0x0059 /* Pin configuration register 25 */
#  define SAM_PORT_PINCFG26_OFFSET  0x005a /* Pin configuration register 26 */
#  define SAM_PORT_PINCFG27_OFFSET  0x005b /* Pin configuration register 27 */
#  define SAM_PORT_PINCFG28_OFFSET  0x005c /* Pin configuration register 28 */
#  define SAM_PORT_PINCFG29_OFFSET  0x005d /* Pin configuration register 29 */
#  define SAM_PORT_PINCFG30_OFFSET  0x005e /* Pin configuration register 30 */
#  define SAM_PORT_PINCFG31_OFFSET  0x005f /* Pin configuration register 31 */

/* PORT register addresses **************************************************/

#define SAM_PORTN_BASE(n)           (SAM_PORT_BASE + SAM_PORTN_OFFSET(n))
#  define SAM_PORTA_BASE            (SAM_PORT_BASE + SAM_PORTA_OFFSET)
#  define SAM_PORTB_BASE            (SAM_PORT_BASE + SAM_PORTB_OFFSET)
#  define SAM_PORTC_BASE            (SAM_PORT_BASE + SAM_PORTC_OFFSET)
#  define SAM_PORTD_BASE            (SAM_PORT_BASE + SAM_PORTD_OFFSET)

#define SAM_PORTA_DIR               (SAM_PORTA_BASE + SAM_PORT_DIR_OFFSET)
#define SAM_PORTA_DIRCLR            (SAM_PORTA_BASE + SAM_PORT_DIRCLR_OFFSET)
#define SAM_PORTA_DIRSET            (SAM_PORTA_BASE + SAM_PORT_DIRSET_OFFSET)
#define SAM_PORTA_DIRTGL            (SAM_PORTA_BASE + SAM_PORT_DIRTGL_OFFSET)
#define SAM_PORTA_OUT               (SAM_PORTA_BASE + SAM_PORT_OUT_OFFSET)
#define SAM_PORTA_OUTCLR            (SAM_PORTA_BASE + SAM_PORT_OUTCLR_OFFSET)
#define SAM_PORTA_OUTSET            (SAM_PORTA_BASE + SAM_PORT_OUTSET_OFFSET)
#define SAM_PORTA_OUTTGL            (SAM_PORTA_BASE + SAM_PORT_OUTTGL_OFFSET)
#define SAM_PORTA_IN                (SAM_PORTA_BASE + SAM_PORT_IN_OFFSET)
#define SAM_PORTA_CTRL              (SAM_PORTA_BASE + SAM_PORT_CTRL_OFFSET)
#define SAM_PORTA_WRCONFIG          (SAM_PORTA_BASE + SAM_PORT_WRCONFIG_OFFSET)
#define SAM_PORT_EVCTRL             (SAM_PORTA_BASE + SAM_PORT_EVCTRL_OFFSET)

#define SAM_PORTA_PMUX(n)           (SAM_PORTA_BASE + SAM_PORT_PMUX_OFFSET(n))
#  define SAM_PORTA_PMUX0           (SAM_PORTA_BASE + SAM_PORT_PMUX0_OFFSET)
#  define SAM_PORTA_PMUX1           (SAM_PORTA_BASE + SAM_PORT_PMUX1_OFFSET)
#  define SAM_PORTA_PMUX2           (SAM_PORTA_BASE + SAM_PORT_PMUX2_OFFSET)
#  define SAM_PORTA_PMUX3           (SAM_PORTA_BASE + SAM_PORT_PMUX3_OFFSET)
#  define SAM_PORTA_PMUX4           (SAM_PORTA_BASE + SAM_PORT_PMUX4_OFFSET)
#  define SAM_PORTA_PMUX5           (SAM_PORTA_BASE + SAM_PORT_PMUX5_OFFSET)
#  define SAM_PORTA_PMUX6           (SAM_PORTA_BASE + SAM_PORT_PMUX6_OFFSET)
#  define SAM_PORTA_PMUX7           (SAM_PORTA_BASE + SAM_PORT_PMUX7_OFFSET)
#  define SAM_PORTA_PMUX8           (SAM_PORTA_BASE + SAM_PORT_PMUX8_OFFSET)
#  define SAM_PORTA_PMUX9           (SAM_PORTA_BASE + SAM_PORT_PMUX9_OFFSET)
#  define SAM_PORTA_PMUX10          (SAM_PORTA_BASE + SAM_PORT_PMUX10_OFFSET)
#  define SAM_PORTA_PMUX11          (SAM_PORTA_BASE + SAM_PORT_PMUX11_OFFSET)
#  define SAM_PORTA_PMUX12          (SAM_PORTA_BASE + SAM_PORT_PMUX12_OFFSET)
#  define SAM_PORTA_PMUX13          (SAM_PORTA_BASE + SAM_PORT_PMUX13_OFFSET)
#  define SAM_PORTA_PMUX14          (SAM_PORTA_BASE + SAM_PORT_PMUX14_OFFSET)
#  define SAM_PORTA_PMUX15          (SAM_PORTA_BASE + SAM_PORT_PMUX15_OFFSET)

#define SAM_PORTA_PINCFG(n)         (SAM_PORTA_BASE + SAM_PORT_PINCFG_OFFSET(n))
#  define SAM_PORTA_PINCFG0         (SAM_PORTA_BASE + SAM_PORT_PINCFG0_OFFSET)
#  define SAM_PORTA_PINCFG1         (SAM_PORTA_BASE + SAM_PORT_PINCFG1_OFFSET)
#  define SAM_PORTA_PINCFG2         (SAM_PORTA_BASE + SAM_PORT_PINCFG2_OFFSET)
#  define SAM_PORTA_PINCFG3         (SAM_PORTA_BASE + SAM_PORT_PINCFG3_OFFSET)
#  define SAM_PORTA_PINCFG4         (SAM_PORTA_BASE + SAM_PORT_PINCFG4_OFFSET)
#  define SAM_PORTA_PINCFG5         (SAM_PORTA_BASE + SAM_PORT_PINCFG5_OFFSET)
#  define SAM_PORTA_PINCFG6         (SAM_PORTA_BASE + SAM_PORT_PINCFG6_OFFSET)
#  define SAM_PORTA_PINCFG7         (SAM_PORTA_BASE + SAM_PORT_PINCFG7_OFFSET)
#  define SAM_PORTA_PINCFG8         (SAM_PORTA_BASE + SAM_PORT_PINCFG8_OFFSET)
#  define SAM_PORTA_PINCFG9         (SAM_PORTA_BASE + SAM_PORT_PINCFG9_OFFSET)
#  define SAM_PORTA_PINCFG10        (SAM_PORTA_BASE + SAM_PORT_PINCFG10_OFFSET)
#  define SAM_PORTA_PINCFG11        (SAM_PORTA_BASE + SAM_PORT_PINCFG11_OFFSET)
#  define SAM_PORTA_PINCFG12        (SAM_PORTA_BASE + SAM_PORT_PINCFG12_OFFSET)
#  define SAM_PORTA_PINCFG13        (SAM_PORTA_BASE + SAM_PORT_PINCFG13_OFFSET)
#  define SAM_PORTA_PINCFG14        (SAM_PORTA_BASE + SAM_PORT_PINCFG14_OFFSET)
#  define SAM_PORTA_PINCFG15        (SAM_PORTA_BASE + SAM_PORT_PINCFG15_OFFSET)
#  define SAM_PORTA_PINCFG16        (SAM_PORTA_BASE + SAM_PORT_PINCFG16_OFFSET)
#  define SAM_PORTA_PINCFG17        (SAM_PORTA_BASE + SAM_PORT_PINCFG17_OFFSET)
#  define SAM_PORTA_PINCFG18        (SAM_PORTA_BASE + SAM_PORT_PINCFG18_OFFSET)
#  define SAM_PORTA_PINCFG19        (SAM_PORTA_BASE + SAM_PORT_PINCFG19_OFFSET)
#  define SAM_PORTA_PINCFG20        (SAM_PORTA_BASE + SAM_PORT_PINCFG20_OFFSET)
#  define SAM_PORTA_PINCFG21        (SAM_PORTA_BASE + SAM_PORT_PINCFG21_OFFSET)
#  define SAM_PORTA_PINCFG22        (SAM_PORTA_BASE + SAM_PORT_PINCFG22_OFFSET)
#  define SAM_PORTA_PINCFG23        (SAM_PORTA_BASE + SAM_PORT_PINCFG23_OFFSET)
#  define SAM_PORTA_PINCFG24        (SAM_PORTA_BASE + SAM_PORT_PINCFG24_OFFSET)
#  define SAM_PORTA_PINCFG25        (SAM_PORTA_BASE + SAM_PORT_PINCFG25_OFFSET)
#  define SAM_PORTA_PINCFG26        (SAM_PORTA_BASE + SAM_PORT_PINCFG26_OFFSET)
#  define SAM_PORTA_PINCFG27        (SAM_PORTA_BASE + SAM_PORT_PINCFG27_OFFSET)
#  define SAM_PORTA_PINCFG28        (SAM_PORTA_BASE + SAM_PORT_PINCFG28_OFFSET)
#  define SAM_PORTA_PINCFG29        (SAM_PORTA_BASE + SAM_PORT_PINCFG29_OFFSET)
#  define SAM_PORTA_PINCFG30        (SAM_PORTA_BASE + SAM_PORT_PINCFG30_OFFSET)
#  define SAM_PORTA_PINCFG31        (SAM_PORTA_BASE + SAM_PORT_PINCFG31_OFFSET)

#define SAM_PORTB_DIR               (SAM_PORTB_BASE + SAM_PORT_DIR_OFFSET)
#define SAM_PORTB_DIRCLR            (SAM_PORTB_BASE + SAM_PORT_DIRCLR_OFFSET)
#define SAM_PORTB_DIRSET            (SAM_PORTB_BASE + SAM_PORT_DIRSET_OFFSET)
#define SAM_PORTB_DIRTGL            (SAM_PORTB_BASE + SAM_PORT_DIRTGL_OFFSET)
#define SAM_PORTB_OUT               (SAM_PORTB_BASE + SAM_PORT_OUT_OFFSET)
#define SAM_PORTB_OUTCLR            (SAM_PORTB_BASE + SAM_PORT_OUTCLR_OFFSET)
#define SAM_PORTB_OUTSET            (SAM_PORTB_BASE + SAM_PORT_OUTSET_OFFSET)
#define SAM_PORTB_OUTTGL            (SAM_PORTB_BASE + SAM_PORT_OUTTGL_OFFSET)
#define SAM_PORTB_IN                (SAM_PORTB_BASE + SAM_PORT_IN_OFFSET)
#define SAM_PORTB_CTRL              (SAM_PORTB_BASE + SAM_PORT_CTRL_OFFSET)
#define SAM_PORTB_WRCONFIG          (SAM_PORTB_BASE + SAM_PORT_WRCONFIG_OFFSET)

#define SAM_PORTB_PMUX(n)           (SAM_PORTB_BASE + SAM_PORT_PMUX_OFFSET(n))
#  define SAM_PORTB_PMUX0           (SAM_PORTB_BASE + SAM_PORT_PMUX0_OFFSET)
#  define SAM_PORTB_PMUX1           (SAM_PORTB_BASE + SAM_PORT_PMUX1_OFFSET)
#  define SAM_PORTB_PMUX2           (SAM_PORTB_BASE + SAM_PORT_PMUX2_OFFSET)
#  define SAM_PORTB_PMUX3           (SAM_PORTB_BASE + SAM_PORT_PMUX3_OFFSET)
#  define SAM_PORTB_PMUX4           (SAM_PORTB_BASE + SAM_PORT_PMUX4_OFFSET)
#  define SAM_PORTB_PMUX5           (SAM_PORTB_BASE + SAM_PORT_PMUX5_OFFSET)
#  define SAM_PORTB_PMUX6           (SAM_PORTB_BASE + SAM_PORT_PMUX6_OFFSET)
#  define SAM_PORTB_PMUX7           (SAM_PORTB_BASE + SAM_PORT_PMUX7_OFFSET)
#  define SAM_PORTB_PMUX8           (SAM_PORTB_BASE + SAM_PORT_PMUX8_OFFSET)
#  define SAM_PORTB_PMUX9           (SAM_PORTB_BASE + SAM_PORT_PMUX9_OFFSET)
#  define SAM_PORTB_PMUX10          (SAM_PORTB_BASE + SAM_PORT_PMUX10_OFFSET)
#  define SAM_PORTB_PMUX11          (SAM_PORTB_BASE + SAM_PORT_PMUX11_OFFSET)
#  define SAM_PORTB_PMUX12          (SAM_PORTB_BASE + SAM_PORT_PMUX12_OFFSET)
#  define SAM_PORTB_PMUX13          (SAM_PORTB_BASE + SAM_PORT_PMUX13_OFFSET)
#  define SAM_PORTB_PMUX14          (SAM_PORTB_BASE + SAM_PORT_PMUX14_OFFSET)
#  define SAM_PORTB_PMUX15          (SAM_PORTB_BASE + SAM_PORT_PMUX15_OFFSET)

#define SAM_PORTB_PINCFG(n)         (SAM_PORTB_BASE + SAM_PORT_PINCFG_OFFSET(n))
#  define SAM_PORTB_PINCFG0         (SAM_PORTB_BASE + SAM_PORT_PINCFG0_OFFSET)
#  define SAM_PORTB_PINCFG1         (SAM_PORTB_BASE + SAM_PORT_PINCFG1_OFFSET)
#  define SAM_PORTB_PINCFG2         (SAM_PORTB_BASE + SAM_PORT_PINCFG2_OFFSET)
#  define SAM_PORTB_PINCFG3         (SAM_PORTB_BASE + SAM_PORT_PINCFG3_OFFSET)
#  define SAM_PORTB_PINCFG4         (SAM_PORTB_BASE + SAM_PORT_PINCFG4_OFFSET)
#  define SAM_PORTB_PINCFG5         (SAM_PORTB_BASE + SAM_PORT_PINCFG5_OFFSET)
#  define SAM_PORTB_PINCFG6         (SAM_PORTB_BASE + SAM_PORT_PINCFG6_OFFSET)
#  define SAM_PORTB_PINCFG7         (SAM_PORTB_BASE + SAM_PORT_PINCFG7_OFFSET)
#  define SAM_PORTB_PINCFG8         (SAM_PORTB_BASE + SAM_PORT_PINCFG8_OFFSET)
#  define SAM_PORTB_PINCFG9         (SAM_PORTB_BASE + SAM_PORT_PINCFG9_OFFSET)
#  define SAM_PORTB_PINCFG10        (SAM_PORTB_BASE + SAM_PORT_PINCFG10_OFFSET)
#  define SAM_PORTB_PINCFG11        (SAM_PORTB_BASE + SAM_PORT_PINCFG11_OFFSET)
#  define SAM_PORTB_PINCFG12        (SAM_PORTB_BASE + SAM_PORT_PINCFG12_OFFSET)
#  define SAM_PORTB_PINCFG13        (SAM_PORTB_BASE + SAM_PORT_PINCFG13_OFFSET)
#  define SAM_PORTB_PINCFG14        (SAM_PORTB_BASE + SAM_PORT_PINCFG14_OFFSET)
#  define SAM_PORTB_PINCFG15        (SAM_PORTB_BASE + SAM_PORT_PINCFG15_OFFSET)
#  define SAM_PORTB_PINCFG16        (SAM_PORTB_BASE + SAM_PORT_PINCFG16_OFFSET)
#  define SAM_PORTB_PINCFG17        (SAM_PORTB_BASE + SAM_PORT_PINCFG17_OFFSET)
#  define SAM_PORTB_PINCFG18        (SAM_PORTB_BASE + SAM_PORT_PINCFG18_OFFSET)
#  define SAM_PORTB_PINCFG19        (SAM_PORTB_BASE + SAM_PORT_PINCFG19_OFFSET)
#  define SAM_PORTB_PINCFG20        (SAM_PORTB_BASE + SAM_PORT_PINCFG20_OFFSET)
#  define SAM_PORTB_PINCFG21        (SAM_PORTB_BASE + SAM_PORT_PINCFG21_OFFSET)
#  define SAM_PORTB_PINCFG22        (SAM_PORTB_BASE + SAM_PORT_PINCFG22_OFFSET)
#  define SAM_PORTB_PINCFG23        (SAM_PORTB_BASE + SAM_PORT_PINCFG23_OFFSET)
#  define SAM_PORTB_PINCFG24        (SAM_PORTB_BASE + SAM_PORT_PINCFG24_OFFSET)
#  define SAM_PORTB_PINCFG25        (SAM_PORTB_BASE + SAM_PORT_PINCFG25_OFFSET)
#  define SAM_PORTB_PINCFG26        (SAM_PORTB_BASE + SAM_PORT_PINCFG26_OFFSET)
#  define SAM_PORTB_PINCFG27        (SAM_PORTB_BASE + SAM_PORT_PINCFG27_OFFSET)
#  define SAM_PORTB_PINCFG28        (SAM_PORTB_BASE + SAM_PORT_PINCFG28_OFFSET)
#  define SAM_PORTB_PINCFG29        (SAM_PORTB_BASE + SAM_PORT_PINCFG29_OFFSET)
#  define SAM_PORTB_PINCFG30        (SAM_PORTB_BASE + SAM_PORT_PINCFG30_OFFSET)
#  define SAM_PORTB_PINCFG31        (SAM_PORTB_BASE + SAM_PORT_PINCFG31_OFFSET)

#define SAM_PORTC_DIR               (SAM_PORTC_BASE + SAM_PORT_DIR_OFFSET)
#define SAM_PORTC_DIRCLR            (SAM_PORTC_BASE + SAM_PORT_DIRCLR_OFFSET)
#define SAM_PORTC_DIRSET            (SAM_PORTC_BASE + SAM_PORT_DIRSET_OFFSET)
#define SAM_PORTC_DIRTGL            (SAM_PORTC_BASE + SAM_PORT_DIRTGL_OFFSET)
#define SAM_PORTC_OUT               (SAM_PORTC_BASE + SAM_PORT_OUT_OFFSET)
#define SAM_PORTC_OUTCLR            (SAM_PORTC_BASE + SAM_PORT_OUTCLR_OFFSET)
#define SAM_PORTC_OUTSET            (SAM_PORTC_BASE + SAM_PORT_OUTSET_OFFSET)
#define SAM_PORTC_OUTTGL            (SAM_PORTC_BASE + SAM_PORT_OUTTGL_OFFSET)
#define SAM_PORTC_IN                (SAM_PORTC_BASE + SAM_PORT_IN_OFFSET)
#define SAM_PORTC_CTRL              (SAM_PORTC_BASE + SAM_PORT_CTRL_OFFSET)
#define SAM_PORTC_WRCONFIG          (SAM_PORTC_BASE + SAM_PORT_WRCONFIG_OFFSET)

#define SAM_PORTC_PMUX(n)           (SAM_PORTC_BASE + SAM_PORT_PMUX_OFFSET(n))
#  define SAM_PORTC_PMUX0           (SAM_PORTC_BASE + SAM_PORT_PMUX0_OFFSET)
#  define SAM_PORTC_PMUX1           (SAM_PORTC_BASE + SAM_PORT_PMUX1_OFFSET)
#  define SAM_PORTC_PMUX2           (SAM_PORTC_BASE + SAM_PORT_PMUX2_OFFSET)
#  define SAM_PORTC_PMUX3           (SAM_PORTC_BASE + SAM_PORT_PMUX3_OFFSET)
#  define SAM_PORTC_PMUX4           (SAM_PORTC_BASE + SAM_PORT_PMUX4_OFFSET)
#  define SAM_PORTC_PMUX5           (SAM_PORTC_BASE + SAM_PORT_PMUX5_OFFSET)
#  define SAM_PORTC_PMUX6           (SAM_PORTC_BASE + SAM_PORT_PMUX6_OFFSET)
#  define SAM_PORTC_PMUX7           (SAM_PORTC_BASE + SAM_PORT_PMUX7_OFFSET)
#  define SAM_PORTC_PMUX8           (SAM_PORTC_BASE + SAM_PORT_PMUX8_OFFSET)
#  define SAM_PORTC_PMUX9           (SAM_PORTC_BASE + SAM_PORT_PMUX9_OFFSET)
#  define SAM_PORTC_PMUX10          (SAM_PORTC_BASE + SAM_PORT_PMUX10_OFFSET)
#  define SAM_PORTC_PMUX11          (SAM_PORTC_BASE + SAM_PORT_PMUX11_OFFSET)
#  define SAM_PORTC_PMUX12          (SAM_PORTC_BASE + SAM_PORT_PMUX12_OFFSET)
#  define SAM_PORTC_PMUX13          (SAM_PORTC_BASE + SAM_PORT_PMUX13_OFFSET)
#  define SAM_PORTC_PMUX14          (SAM_PORTC_BASE + SAM_PORT_PMUX14_OFFSET)
#  define SAM_PORTC_PMUX15          (SAM_PORTC_BASE + SAM_PORT_PMUX15_OFFSET)

#define SAM_PORTC_PINCFG(n)         (SAM_PORTC_BASE + SAM_PORT_PINCFG_OFFSET(n))
#  define SAM_PORTC_PINCFG0         (SAM_PORTC_BASE + SAM_PORT_PINCFG0_OFFSET)
#  define SAM_PORTC_PINCFG1         (SAM_PORTC_BASE + SAM_PORT_PINCFG1_OFFSET)
#  define SAM_PORTC_PINCFG2         (SAM_PORTC_BASE + SAM_PORT_PINCFG2_OFFSET)
#  define SAM_PORTC_PINCFG3         (SAM_PORTC_BASE + SAM_PORT_PINCFG3_OFFSET)
#  define SAM_PORTC_PINCFG4         (SAM_PORTC_BASE + SAM_PORT_PINCFG4_OFFSET)
#  define SAM_PORTC_PINCFG5         (SAM_PORTC_BASE + SAM_PORT_PINCFG5_OFFSET)
#  define SAM_PORTC_PINCFG6         (SAM_PORTC_BASE + SAM_PORT_PINCFG6_OFFSET)
#  define SAM_PORTC_PINCFG7         (SAM_PORTC_BASE + SAM_PORT_PINCFG7_OFFSET)
#  define SAM_PORTC_PINCFG8         (SAM_PORTC_BASE + SAM_PORT_PINCFG8_OFFSET)
#  define SAM_PORTC_PINCFG9         (SAM_PORTC_BASE + SAM_PORT_PINCFG9_OFFSET)
#  define SAM_PORTC_PINCFG10        (SAM_PORTC_BASE + SAM_PORT_PINCFG10_OFFSET)
#  define SAM_PORTC_PINCFG11        (SAM_PORTC_BASE + SAM_PORT_PINCFG11_OFFSET)
#  define SAM_PORTC_PINCFG12        (SAM_PORTC_BASE + SAM_PORT_PINCFG12_OFFSET)
#  define SAM_PORTC_PINCFG13        (SAM_PORTC_BASE + SAM_PORT_PINCFG13_OFFSET)
#  define SAM_PORTC_PINCFG14        (SAM_PORTC_BASE + SAM_PORT_PINCFG14_OFFSET)
#  define SAM_PORTC_PINCFG15        (SAM_PORTC_BASE + SAM_PORT_PINCFG15_OFFSET)
#  define SAM_PORTC_PINCFG16        (SAM_PORTC_BASE + SAM_PORT_PINCFG16_OFFSET)
#  define SAM_PORTC_PINCFG17        (SAM_PORTC_BASE + SAM_PORT_PINCFG17_OFFSET)
#  define SAM_PORTC_PINCFG18        (SAM_PORTC_BASE + SAM_PORT_PINCFG18_OFFSET)
#  define SAM_PORTC_PINCFG19        (SAM_PORTC_BASE + SAM_PORT_PINCFG19_OFFSET)
#  define SAM_PORTC_PINCFG20        (SAM_PORTC_BASE + SAM_PORT_PINCFG20_OFFSET)
#  define SAM_PORTC_PINCFG21        (SAM_PORTC_BASE + SAM_PORT_PINCFG21_OFFSET)
#  define SAM_PORTC_PINCFG22        (SAM_PORTC_BASE + SAM_PORT_PINCFG22_OFFSET)
#  define SAM_PORTC_PINCFG23        (SAM_PORTC_BASE + SAM_PORT_PINCFG23_OFFSET)
#  define SAM_PORTC_PINCFG24        (SAM_PORTC_BASE + SAM_PORT_PINCFG24_OFFSET)
#  define SAM_PORTC_PINCFG25        (SAM_PORTC_BASE + SAM_PORT_PINCFG25_OFFSET)
#  define SAM_PORTC_PINCFG26        (SAM_PORTC_BASE + SAM_PORT_PINCFG26_OFFSET)
#  define SAM_PORTC_PINCFG27        (SAM_PORTC_BASE + SAM_PORT_PINCFG27_OFFSET)
#  define SAM_PORTC_PINCFG28        (SAM_PORTC_BASE + SAM_PORT_PINCFG28_OFFSET)
#  define SAM_PORTC_PINCFG29        (SAM_PORTC_BASE + SAM_PORT_PINCFG29_OFFSET)
#  define SAM_PORTC_PINCFG30        (SAM_PORTC_BASE + SAM_PORT_PINCFG30_OFFSET)
#  define SAM_PORTC_PINCFG31        (SAM_PORTC_BASE + SAM_PORT_PINCFG31_OFFSET)

#define SAM_PORTD_DIR               (SAM_PORTD_BASE + SAM_PORT_DIR_OFFSET)
#define SAM_PORTD_DIRCLR            (SAM_PORTD_BASE + SAM_PORT_DIRCLR_OFFSET)
#define SAM_PORTD_DIRSET            (SAM_PORTD_BASE + SAM_PORT_DIRSET_OFFSET)
#define SAM_PORTD_DIRTGL            (SAM_PORTD_BASE + SAM_PORT_DIRTGL_OFFSET)
#define SAM_PORTD_OUT               (SAM_PORTD_BASE + SAM_PORT_OUT_OFFSET)
#define SAM_PORTD_OUTCLR            (SAM_PORTD_BASE + SAM_PORT_OUTCLR_OFFSET)
#define SAM_PORTD_OUTSET            (SAM_PORTD_BASE + SAM_PORT_OUTSET_OFFSET)
#define SAM_PORTD_OUTTGL            (SAM_PORTD_BASE + SAM_PORT_OUTTGL_OFFSET)
#define SAM_PORTD_IN                (SAM_PORTD_BASE + SAM_PORT_IN_OFFSET)
#define SAM_PORTD_CTRL              (SAM_PORTD_BASE + SAM_PORT_CTRL_OFFSET)
#define SAM_PORTD_WRCONFIG          (SAM_PORTD_BASE + SAM_PORT_WRCONFIG_OFFSET)

#define SAM_PORTD_PMUX(n)           (SAM_PORTD_BASE + SAM_PORT_PMUX_OFFSET(n))
#  define SAM_PORTD_PMUX0           (SAM_PORTD_BASE + SAM_PORT_PMUX0_OFFSET)
#  define SAM_PORTD_PMUX1           (SAM_PORTD_BASE + SAM_PORT_PMUX1_OFFSET)
#  define SAM_PORTD_PMUX2           (SAM_PORTD_BASE + SAM_PORT_PMUX2_OFFSET)
#  define SAM_PORTD_PMUX3           (SAM_PORTD_BASE + SAM_PORT_PMUX3_OFFSET)
#  define SAM_PORTD_PMUX4           (SAM_PORTD_BASE + SAM_PORT_PMUX4_OFFSET)
#  define SAM_PORTD_PMUX5           (SAM_PORTD_BASE + SAM_PORT_PMUX5_OFFSET)
#  define SAM_PORTD_PMUX6           (SAM_PORTD_BASE + SAM_PORT_PMUX6_OFFSET)
#  define SAM_PORTD_PMUX7           (SAM_PORTD_BASE + SAM_PORT_PMUX7_OFFSET)
#  define SAM_PORTD_PMUX8           (SAM_PORTD_BASE + SAM_PORT_PMUX8_OFFSET)
#  define SAM_PORTD_PMUX9           (SAM_PORTD_BASE + SAM_PORT_PMUX9_OFFSET)
#  define SAM_PORTD_PMUX10          (SAM_PORTD_BASE + SAM_PORT_PMUX10_OFFSET)
#  define SAM_PORTD_PMUX11          (SAM_PORTD_BASE + SAM_PORT_PMUX11_OFFSET)
#  define SAM_PORTD_PMUX12          (SAM_PORTD_BASE + SAM_PORT_PMUX12_OFFSET)
#  define SAM_PORTD_PMUX13          (SAM_PORTD_BASE + SAM_PORT_PMUX13_OFFSET)
#  define SAM_PORTD_PMUX14          (SAM_PORTD_BASE + SAM_PORT_PMUX14_OFFSET)
#  define SAM_PORTD_PMUX15          (SAM_PORTD_BASE + SAM_PORT_PMUX15_OFFSET)

#define SAM_PORTD_PINCFG(n)         (SAM_PORTD_BASE + SAM_PORT_PINCFG_OFFSET(n))
#  define SAM_PORTD_PINCFG0         (SAM_PORTD_BASE + SAM_PORT_PINCFG0_OFFSET)
#  define SAM_PORTD_PINCFG1         (SAM_PORTD_BASE + SAM_PORT_PINCFG1_OFFSET)
#  define SAM_PORTD_PINCFG2         (SAM_PORTD_BASE + SAM_PORT_PINCFG2_OFFSET)
#  define SAM_PORTD_PINCFG3         (SAM_PORTD_BASE + SAM_PORT_PINCFG3_OFFSET)
#  define SAM_PORTD_PINCFG4         (SAM_PORTD_BASE + SAM_PORT_PINCFG4_OFFSET)
#  define SAM_PORTD_PINCFG5         (SAM_PORTD_BASE + SAM_PORT_PINCFG5_OFFSET)
#  define SAM_PORTD_PINCFG6         (SAM_PORTD_BASE + SAM_PORT_PINCFG6_OFFSET)
#  define SAM_PORTD_PINCFG7         (SAM_PORTD_BASE + SAM_PORT_PINCFG7_OFFSET)
#  define SAM_PORTD_PINCFG8         (SAM_PORTD_BASE + SAM_PORT_PINCFG8_OFFSET)
#  define SAM_PORTD_PINCFG9         (SAM_PORTD_BASE + SAM_PORT_PINCFG9_OFFSET)
#  define SAM_PORTD_PINCFG10        (SAM_PORTD_BASE + SAM_PORT_PINCFG10_OFFSET)
#  define SAM_PORTD_PINCFG11        (SAM_PORTD_BASE + SAM_PORT_PINCFG11_OFFSET)
#  define SAM_PORTD_PINCFG12        (SAM_PORTD_BASE + SAM_PORT_PINCFG12_OFFSET)
#  define SAM_PORTD_PINCFG13        (SAM_PORTD_BASE + SAM_PORT_PINCFG13_OFFSET)
#  define SAM_PORTD_PINCFG14        (SAM_PORTD_BASE + SAM_PORT_PINCFG14_OFFSET)
#  define SAM_PORTD_PINCFG15        (SAM_PORTD_BASE + SAM_PORT_PINCFG15_OFFSET)
#  define SAM_PORTD_PINCFG16        (SAM_PORTD_BASE + SAM_PORT_PINCFG16_OFFSET)
#  define SAM_PORTD_PINCFG17        (SAM_PORTD_BASE + SAM_PORT_PINCFG17_OFFSET)
#  define SAM_PORTD_PINCFG18        (SAM_PORTD_BASE + SAM_PORT_PINCFG18_OFFSET)
#  define SAM_PORTD_PINCFG19        (SAM_PORTD_BASE + SAM_PORT_PINCFG19_OFFSET)
#  define SAM_PORTD_PINCFG20        (SAM_PORTD_BASE + SAM_PORT_PINCFG20_OFFSET)
#  define SAM_PORTD_PINCFG21        (SAM_PORTD_BASE + SAM_PORT_PINCFG21_OFFSET)
#  define SAM_PORTD_PINCFG22        (SAM_PORTD_BASE + SAM_PORT_PINCFG22_OFFSET)
#  define SAM_PORTD_PINCFG23        (SAM_PORTD_BASE + SAM_PORT_PINCFG23_OFFSET)
#  define SAM_PORTD_PINCFG24        (SAM_PORTD_BASE + SAM_PORT_PINCFG24_OFFSET)
#  define SAM_PORTD_PINCFG25        (SAM_PORTD_BASE + SAM_PORT_PINCFG25_OFFSET)
#  define SAM_PORTD_PINCFG26        (SAM_PORTD_BASE + SAM_PORT_PINCFG26_OFFSET)
#  define SAM_PORTD_PINCFG27        (SAM_PORTD_BASE + SAM_PORT_PINCFG27_OFFSET)
#  define SAM_PORTD_PINCFG28        (SAM_PORTD_BASE + SAM_PORT_PINCFG28_OFFSET)
#  define SAM_PORTD_PINCFG29        (SAM_PORTD_BASE + SAM_PORT_PINCFG29_OFFSET)
#  define SAM_PORTD_PINCFG30        (SAM_PORTD_BASE + SAM_PORT_PINCFG30_OFFSET)
#  define SAM_PORTD_PINCFG31        (SAM_PORTD_BASE + SAM_PORT_PINCFG31_OFFSET)

/* PORT register bit definitions ********************************************/

/* Data direction, data direction clear,  data direction set, and data
 * direction toggle registers
 */

#define PORT_DIR(n)                 (1 << n) /* Port data n, direction, n=0-31 */

/* Data output value, data output value clear, data output value set, and
 * data output value toggle registers
 */

#define PORT_OUT(n)                 (1 << n) /* Port data n output value, n=0-31 */

/* Data input value register */

#define PORT_IN(n)                  (1 << n) /* Port n data input value, n=0-31 */

/* Control register */

#define PORT_CTRL(n)                (1 << n) /* Port n input sampling mode, n=0-31 */

/* Write configuration registers */

#define PORT_WRCONFIG_PINMASK_SHIFT (0) /* Bits 0-15:  Pin Mask for Multiple Pin Configuration */
#define PORT_WRCONFIG_PINMASK_MASK  (0xffff << PORT_WRCONFIG_PINMASK_SHIFT)
#  define PORT_WRCONFIG_PINMASK(n)  (1 << (PORT_WRCONFIG_PINMASK_SHIFT+(n)))
#define PORT_WRCONFIG_PMUXEN        (1 << 16) /* Bit 16: Peripheral Multiplexer Enable */
#define PORT_WRCONFIG_INEN          (1 << 17) /* Bit 17: Input Enable */
#define PORT_WRCONFIG_PULLEN        (1 << 18) /* Bit 18: Pull Enable */
#define PORT_WRCONFIG_DRVSTR        (1 << 22) /* Bit 22: Output Driver Strength Selection */
#define PORT_WRCONFIG_PMUX_SHIFT    (24)      /* Bits 24-27: Peripheral Multiplexing */
#define PORT_WRCONFIG_PMUX_MASK     (15 << PORT_WRCONFIG_PMUX_SHIFT)
#  define PORT_WRCONFIG_PMUX(n)     ((uint32_t)(n) << PORT_WRCONFIG_PMUX_SHIFT)
#define PORT_WRCONFIG_WRPMUX        (1 << 28) /* Bit 28: Write PMUX */
#define PORT_WRCONFIG_WRPINCFG      (1 << 30) /* Bit 30: Write PINCFG */
#define PORT_WRCONFIG_HWSEL         (1 << 31) /* Bit 31: Half-Word Select */

/* Event input control register */

#define PORT_EVCTRL_PID_SHIFT(n)    ((n) << 3)       /* Port event pin identifier n, n=0..3 */
#define PORT_EVCTRL_PID_MASK(n)     (31 << PORT_EVCTRL_PID_SHIFT(n))
#  define PORT_EVCTRL_PID(n,v)      ((unint32_t)(v) << PORT_EVCTRL_PID_SHIFT(n))
#define PORT_EVCTRL_EVACT_SHIFT(n)  (((n) << 3) + 5) /* Port event pin action n, n=0..3 */
#define PORT_EVCTRL_EVACT_MASK(n)   (3 << PORT_EVCTRL_EVACT_SHIFT(n))
#  define PORT_EVCTRL_EVACT(n,v)    ((unint32_t)(v) << PORT_EVCTRL_EVACT_SHIFT(n))
#define PORT_EVCTRL_PORTEI(n)       (((n) << 3) + 7) /* Port event input enable n, n=0..3 */

#define PORT_EVCTRL_PID0_SHIFT      (0)       /* Bits 0-4: Port event pin identifier 0 */
#define PORT_EVCTRL_PID0_MASK       (31 << PORT_EVCTRL_PID0_SHIFT)
#  define PORT_EVCTRL_PID0(v)       ((unint32_t)(v) << PORT_EVCTRL_PID0_SHIFT)
#define PORT_EVCTRL_EVACT0_SHIFT    (5)       /* Bits 5-6: Port event pin action 0 */
#define PORT_EVCTRL_EVACT0_MASK     (3 << PORT_EVCTRL_EVACT0_SHIFT)
#  define PORT_EVCTRL_EVACT0(v)     ((unint32_t)(v) << PORT_EVCTRL_EVACT0_SHIFT)
#define PORT_EVCTRL_PORTEI0         (1 << 7)  /* Bit 7: Port event input enable 0 */
#define PORT_EVCTRL_PID1_SHIFT      (8)       /* Bits 8-12: Port event pin identifier 1 */
#define PORT_EVCTRL_PID1_MASK       (31 << PORT_EVCTRL_PID1_SHIFT)
#  define PORT_EVCTRL_PID1(v)       ((unint32_t)(v) << PORT_EVCTRL_PID1_SHIFT)
#define PORT_EVCTRL_EVACT1_SHIFT    (13)      /* Bits 13-14: Port event pin action 1 */
#define PORT_EVCTRL_EVACT1_MASK     (3 << PORT_EVCTRL_EVACT1_SHIFT)
#  define PORT_EVCTRL_EVACT1(v)     ((unint32_t)(v) << PORT_EVCTRL_EVACT1_SHIFT)
#define PORT_EVCTRL_PORTEI1         (1 << 15) /* Bit 15: Port event input enable 1 */
#define PORT_EVCTRL_PID2_SHIFT      (16)      /* Bits 16-20: Port event pin identifier 2 */
#define PORT_EVCTRL_PID2_MASK       (31 << PORT_EVCTRL_PID2_SHIFT)
#  define PORT_EVCTRL_PID2(v)       ((unint32_t)(v) << PORT_EVCTRL_PID2_SHIFT)
#define PORT_EVCTRL_EVACT2_SHIFT    (21)      /* Bits 21-22: Port event pin action 2 */
#define PORT_EVCTRL_EVACT2_MASK     (3 << PORT_EVCTRL_EVACT2_SHIFT)
#  define PORT_EVCTRL_EVACT2(v)     ((unint32_t)(v) << PORT_EVCTRL_EVACT2_SHIFT)
#define PORT_EVCTRL_PORTEI2         (1 << 23) /* Bit 23: Port event input enable 2 */
#define PORT_EVCTRL_PID3_SHIFT      (24)      /* Bits 24-28: Port event pin identifier 3 */
#define PORT_EVCTRL_PID3_MASK       (31 << PORT_EVCTRL_PID3_SHIFT)
#  define PORT_EVCTRL_PID3(v)       ((unint32_t)(v) << PORT_EVCTRL_PID3_SHIFT)
#define PORT_EVCTRL_EVACT3_SHIFT    (29)      /* Bits 29-30: Port event pin action 3 */
#define PORT_EVCTRL_EVACT3_MASK     (3 << PORT_EVCTRL_EVACT3_SHIFT)
#  define PORT_EVCTRL_EVACT3(v)     ((unint32_t)(v) << PORT_EVCTRL_EVACT3_SHIFT)
#define PORT_EVCTRL_PORTEI3         (1 << 31) /* Bit 31: Port event input enable 3 */

/* Peripheral multiplexing registers */

#define PORT_PMUXE_SHIFT            (0)       /* Bits 0-3: Peripheral multiplexing even */
#define PORT_PMUXE_MASK             (15 << PORT_PMUXE_SHIFT)
#  define PORT_PMUXE_PERIPHA        (0  << PORT_PMUXE_SHIFT) /* Peripheral function A */
#  define PORT_PMUXE_PERIPHB        (1  << PORT_PMUXE_SHIFT) /* Peripheral function B */
#  define PORT_PMUXE_PERIPHC        (2  << PORT_PMUXE_SHIFT) /* Peripheral function C */
#  define PORT_PMUXE_PERIPHD        (3  << PORT_PMUXE_SHIFT) /* Peripheral function D */
#  define PORT_PMUXE_PERIPHE        (4  << PORT_PMUXE_SHIFT) /* Peripheral function E */
#  define PORT_PMUXE_PERIPHF        (5  << PORT_PMUXE_SHIFT) /* Peripheral function F */
#  define PORT_PMUXE_PERIPHG        (6  << PORT_PMUXE_SHIFT) /* Peripheral function G */
#  define PORT_PMUXE_PERIPHH        (7  << PORT_PMUXE_SHIFT) /* Peripheral function H */
#  define PORT_PMUXE_PERIPHI        (8  << PORT_PMUXE_SHIFT) /* Peripheral function I */
#  define PORT_PMUXE_PERIPHJ        (9  << PORT_PMUXE_SHIFT) /* Peripheral function J */
#  define PORT_PMUXE_PERIPHK        (10 << PORT_PMUXE_SHIFT) /* Peripheral function K */
#  define PORT_PMUXE_PERIPHL        (11 << PORT_PMUXE_SHIFT) /* Peripheral function L */
#  define PORT_PMUXE_PERIPHM        (12 << PORT_PMUXE_SHIFT) /* Peripheral function M */
#  define PORT_PMUXE_PERIPHN        (13 << PORT_PMUXE_SHIFT) /* Peripheral function N */

#define PORT_PMUXO_SHIFT            (4)       /* Bits 4-7: Peripheral multiplexing odd */
#define PORT_PMUXO_MASK             (15 << PORT_PMUXO_SHIFT)
#  define PORT_PMUXO_PERIPHA        (0  << PORT_PMUXO_SHIFT) /* Peripheral function A */
#  define PORT_PMUXO_PERIPHB        (1  << PORT_PMUXO_SHIFT) /* Peripheral function B */
#  define PORT_PMUXO_PERIPHC        (2  << PORT_PMUXO_SHIFT) /* Peripheral function C */
#  define PORT_PMUXO_PERIPHD        (3  << PORT_PMUXO_SHIFT) /* Peripheral function D */
#  define PORT_PMUXO_PERIPHE        (4  << PORT_PMUXO_SHIFT) /* Peripheral function E */
#  define PORT_PMUXO_PERIPHF        (5  << PORT_PMUXO_SHIFT) /* Peripheral function F */
#  define PORT_PMUXO_PERIPHG        (6  << PORT_PMUXO_SHIFT) /* Peripheral function G */
#  define PORT_PMUXO_PERIPHH        (7  << PORT_PMUXO_SHIFT) /* Peripheral function H */
#  define PORT_PMUXO_PERIPHI        (8  << PORT_PMUXO_SHIFT) /* Peripheral function I */
#  define PORT_PMUXO_PERIPHJ        (9  << PORT_PMUXO_SHIFT) /* Peripheral function J */
#  define PORT_PMUXO_PERIPHK        (10 << PORT_PMUXO_SHIFT) /* Peripheral function K */
#  define PORT_PMUXO_PERIPHL        (11 << PORT_PMUXO_SHIFT) /* Peripheral function L */
#  define PORT_PMUXO_PERIPHM        (12 << PORT_PMUXO_SHIFT) /* Peripheral function M */
#  define PORT_PMUXO_PERIPHN        (13 << PORT_PMUXO_SHIFT) /* Peripheral function N */

/* Pin configuration registers */

#define PORT_PINCFG_PMUXEN          (1 << 0)  /* Bit 0: Peripheral Multiplexer Enable */
#define PORT_PINCFG_INEN            (1 << 1)  /* Bit 1: Input Enable */
#define PORT_PINCFG_PULLEN          (1 << 2)  /* Bit 2: Pull Enable */
#define PORT_PINCFG_DRVSTR          (1 << 6)  /* Bit 6: Output Driver Strength Selection */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_PORT_H */
