/************************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_ports.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers.
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
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use with
 * Infineon's microcontrollers.  This file can be freely distributed within
 * development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_PORTS_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_PORTS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "hardware/xmc4_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

/* PORTS Registers */

#define XMC4_PORT_OUT_OFFSET        0x0000    /* Port Output Register */
#define XMC4_PORT_OMR_OFFSET        0x0004    /* Port Output Modification Register */

#define XMC4_PORT_IOCR_OFFSET(n)    (0x0010 + ((n) & ~3))
#define XMC4_PORT_IOCR0_OFFSET      0x0010    /* Port Input/Output Control Register 0 */
#define XMC4_PORT_IOCR4_OFFSET      0x0014    /* Port Input/Output Control Register 4 */
#define XMC4_PORT_IOCR8_OFFSET      0x0018    /* Port Input/Output Control Register 8 */
#define XMC4_PORT_IOCR12_OFFSET     0x001c    /* Port Input/Output Control Register 12 */

#define XMC4_PORT_IN_OFFSET         0x0024    /* Port Input Register */

#define XMC4_PORT_PDR_OFFSET(n)     (0x0040 + (((n) >> 1) & ~3))
#define XMC4_PORT_PDR0_OFFSET       0x0040    /* Port Pad Driver Mode 0 Register */
#define XMC4_PORT_PDR1_OFFSET       0x0044    /* Port Pad Driver Mode 1 Register */

#define XMC4_PORT_PDISC_OFFSET      0x0060    /* Port Pin Function Decision Control Register */
#define XMC4_PORT_PPS_OFFSET        0x0070    /* Port Pin Power Save Register */
#define XMC4_PORT_HWSEL_OFFSET      0x0074    /* Port Pin Hardware Select Register */

/* Register Addresses ****************************************************************/

#define XMC4_PORT0_OUT              (XMC4_PORT0_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT0_OMR              (XMC4_PORT0_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT0_IOCR0            (XMC4_PORT0_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT0_IOCR4            (XMC4_PORT0_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT0_IOCR8            (XMC4_PORT0_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT0_IOCR12           (XMC4_PORT0_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT0_IN               (XMC4_PORT0_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT0_PDR0             (XMC4_PORT0_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT0_PDR1             (XMC4_PORT0_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT0_PDISC            (XMC4_PORT0_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT0_PPS              (XMC4_PORT0_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT0_HWSEL            (XMC4_PORT0_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT1_OUT              (XMC4_PORT1_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT1_OMR              (XMC4_PORT1_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT1_IOCR0            (XMC4_PORT1_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT1_IOCR4            (XMC4_PORT1_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT1_IOCR8            (XMC4_PORT1_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT1_IOCR12           (XMC4_PORT1_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT1_IN               (XMC4_PORT1_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT1_PDR0             (XMC4_PORT1_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT1_PDR1             (XMC4_PORT1_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT1_PDISC            (XMC4_PORT1_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT1_PPS              (XMC4_PORT1_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT1_HWSEL            (XMC4_PORT1_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT2_OUT              (XMC4_PORT2_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT2_OMR              (XMC4_PORT2_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT2_IOCR0            (XMC4_PORT2_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT2_IOCR4            (XMC4_PORT2_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT2_IOCR8            (XMC4_PORT2_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT2_IOCR12           (XMC4_PORT2_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT2_IN               (XMC4_PORT2_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT2_PDR0             (XMC4_PORT2_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT2_PDR1             (XMC4_PORT2_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT2_PDISC            (XMC4_PORT2_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT2_PPS              (XMC4_PORT2_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT2_HWSEL            (XMC4_PORT2_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT3_OUT              (XMC4_PORT3_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT3_OMR              (XMC4_PORT3_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT3_IOCR0            (XMC4_PORT3_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT3_IOCR4            (XMC4_PORT3_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT3_IOCR8            (XMC4_PORT3_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT3_IOCR12           (XMC4_PORT3_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT3_IN               (XMC4_PORT3_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT3_PDR0             (XMC4_PORT3_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT3_PDR1             (XMC4_PORT3_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT3_PDISC            (XMC4_PORT3_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT3_PPS              (XMC4_PORT3_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT3_HWSEL            (XMC4_PORT3_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT4_OUT              (XMC4_PORT4_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT4_OMR              (XMC4_PORT4_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT4_IOCR0            (XMC4_PORT4_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT4_IOCR4            (XMC4_PORT4_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT4_IOCR8            (XMC4_PORT4_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT4_IOCR12           (XMC4_PORT4_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT4_IN               (XMC4_PORT4_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT4_PDR0             (XMC4_PORT4_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT4_PDR1             (XMC4_PORT4_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT4_PDISC            (XMC4_PORT4_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT4_PPS              (XMC4_PORT4_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT4_HWSEL            (XMC4_PORT4_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT5_OUT              (XMC4_PORT5_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT5_OMR              (XMC4_PORT5_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT5_IOCR0            (XMC4_PORT5_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT5_IOCR4            (XMC4_PORT5_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT5_IOCR8            (XMC4_PORT5_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT5_IOCR12           (XMC4_PORT5_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT5_IN               (XMC4_PORT5_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT5_PDR0             (XMC4_PORT5_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT5_PDR1             (XMC4_PORT5_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT5_PDISC            (XMC4_PORT5_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT5_PPS              (XMC4_PORT5_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT5_HWSEL            (XMC4_PORT5_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT6_OUT              (XMC4_PORT6_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT6_OMR              (XMC4_PORT6_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT6_IOCR0            (XMC4_PORT6_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT6_IOCR4            (XMC4_PORT6_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT6_IOCR8            (XMC4_PORT6_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT6_IOCR12           (XMC4_PORT6_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT6_IN               (XMC4_PORT6_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT6_PDR0             (XMC4_PORT6_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT6_PDR1             (XMC4_PORT6_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT6_PDISC            (XMC4_PORT6_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT6_PPS              (XMC4_PORT6_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT6_HWSEL            (XMC4_PORT6_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT7_OUT              (XMC4_PORT7_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT7_OMR              (XMC4_PORT7_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT7_IOCR0            (XMC4_PORT7_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT7_IOCR4            (XMC4_PORT7_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT7_IOCR8            (XMC4_PORT7_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT7_IOCR12           (XMC4_PORT7_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT7_IN               (XMC4_PORT7_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT7_PDR0             (XMC4_PORT7_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT7_PDR1             (XMC4_PORT7_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT7_PDISC            (XMC4_PORT7_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT7_PPS              (XMC4_PORT7_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT7_HWSEL            (XMC4_PORT7_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT8_OUT              (XMC4_PORT8_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT8_OMR              (XMC4_PORT8_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT8_IOCR0            (XMC4_PORT8_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT8_IOCR4            (XMC4_PORT8_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT8_IOCR8            (XMC4_PORT8_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT8_IOCR12           (XMC4_PORT8_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT8_IN               (XMC4_PORT8_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT8_PDR0             (XMC4_PORT8_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT8_PDR1             (XMC4_PORT8_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT8_PDISC            (XMC4_PORT8_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT8_PPS              (XMC4_PORT8_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT8_HWSEL            (XMC4_PORT8_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT9_OUT              (XMC4_PORT9_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT9_OMR              (XMC4_PORT9_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT9_IOCR0            (XMC4_PORT9_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT9_IOCR4            (XMC4_PORT9_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT9_IOCR8            (XMC4_PORT9_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT9_IOCR12           (XMC4_PORT9_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT9_IN               (XMC4_PORT9_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT9_PDR0             (XMC4_PORT9_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT9_PDR1             (XMC4_PORT9_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT9_PDISC            (XMC4_PORT9_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT9_PPS              (XMC4_PORT9_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT9_HWSEL            (XMC4_PORT9_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT14_OUT             (XMC4_PORT14_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT14_OMR             (XMC4_PORT14_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT14_IOCR0           (XMC4_PORT14_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT14_IOCR4           (XMC4_PORT14_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT14_IOCR8           (XMC4_PORT14_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT14_IOCR12          (XMC4_PORT14_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT14_IN              (XMC4_PORT14_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT14_PDR0            (XMC4_PORT14_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT14_PDR1            (XMC4_PORT14_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT14_PDISC           (XMC4_PORT14_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT14_PPS             (XMC4_PORT14_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT14_HWSEL           (XMC4_PORT14_BASE+XMC4_PORT_HWSEL_OFFSET)

#define XMC4_PORT15_OUT             (XMC4_PORT15_BASE+XMC4_PORT_OUT_OFFSET)
#define XMC4_PORT15_OMR             (XMC4_PORT15_BASE+XMC4_PORT_OMR_OFFSET)
#define XMC4_PORT15_IOCR0           (XMC4_PORT15_BASE+XMC4_PORT_IOCR0_OFFSET)
#define XMC4_PORT15_IOCR4           (XMC4_PORT15_BASE+XMC4_PORT_IOCR4_OFFSET)
#define XMC4_PORT15_IOCR8           (XMC4_PORT15_BASE+XMC4_PORT_IOCR8_OFFSET)
#define XMC4_PORT15_IOCR12          (XMC4_PORT15_BASE+XMC4_PORT_IOCR12_OFFSET)
#define XMC4_PORT15_IN              (XMC4_PORT15_BASE+XMC4_PORT_IN_OFFSET)
#define XMC4_PORT15_PDR0            (XMC4_PORT15_BASE+XMC4_PORT_PDR0_OFFSET)
#define XMC4_PORT15_PDR1            (XMC4_PORT15_BASE+XMC4_PORT_PDR1_OFFSET)
#define XMC4_PORT15_PDISC           (XMC4_PORT15_BASE+XMC4_PORT_PDISC_OFFSET)
#define XMC4_PORT15_PPS             (XMC4_PORT15_BASE+XMC4_PORT_PPS_OFFSET)
#define XMC4_PORT15_HWSEL           (XMC4_PORT15_BASE+XMC4_PORT_HWSEL_OFFSET)

/* Register Bit-Field Definitions **************************************************/

/* Port Output Register, , Port Input Register, Port Pin Function Decision Control
 * Register, Port Pin Power Save Register.
 */

#define PORT_PIN(n)                 (1 << (n))

/* Port Output Modification Register:
 *
 * PRx PSx Function
 * 0   0   Bit Pn_OUT.Px is not changed.
 * 0   1   Bit Pn_OUT.Px is set.
 * 1   0   Bit Pn_OUT.Px is reset.
 * 1   1   Bit Pn_OUT.Px is toggled.
 */

#define OMR_PS(n)                   (1 << (n))
#define OMR_PR(n)                   (1 << ((n) + 16))

/* Basic port input/output field values */
/* Direct Input */

#define IOCR_INPUT_NOPULL           0         /* No internal pull device active */
#define IOCR_INPUT_PULLDOWN         1         /* Internal pull-down device active */
#define IOCR_INPUT_PULLUP           2         /* Internal pull-down device active */
#define IOCR_INPUT_CONT             3         /* No internal pull device active; Pn_OUTx
                                               * continuously samples the input value */

/* Any of the above input configurations may be OR'ed with */
/* Inverted Input */

#define IOCR_INPUT_INVERT           4         /* Inverted input modifier */

/* Push-pull Output (direct input) */

#define IOCR_OUTPUT                 16        /* General-purpose output */
#define IOCR_OUTPUT_ALT1            17        /* Alternate output function 1 */
#define IOCR_OUTPUT_ALT2            18        /* Alternate output function 2 */
#define IOCR_OUTPUT_ALT3            19        /* Alternate output function 3 */
#define IOCR_OUTPUT_ALT4            20        /* Alternate output function 4 */

/* Any of the above may be OR'ed with */
/* Open drain output */

#define IOCR_OUTPUT_OPENDRAIN       8         /* Output drain output modifier */

/* Port Input/Output Control Register 0 */

#define PORT_IOCR0_PC_SHIFT(p)      (((p) << 3) + 3)
#define PORT_IOCR0_PC_MASK(p)       (31 << PORT_IOCR0_PC_SHIFT(p))
#  define PORT_IOCR0_PC(p,n)        ((uint32_t)(n) << PORT_IOCR0_PC_SHIFT(p))
#define PORT_IOCR0_PC0_SHIFT        (3)       /* Bit 3-7: Port Control for Port n Pin 0 */
#define PORT_IOCR0_PC0_MASK         (31 << PORT_IOCR0_PC0_SHIFT)
#  define PORT_IOCR0_PC0(n)         ((uint32_t)(n) << PORT_IOCR0_PC0_SHIFT)
#define PORT_IOCR0_PC1_SHIFT        (11)      /* Bit 11-15: Port Control for Port n Pin 1 */
#define PORT_IOCR0_PC1_MASK         (31 << PORT_IOCR0_PC1_SHIFT)
#  define PORT_IOCR0_PC1(n)         ((uint32_t)(n) << PORT_IOCR0_PC1_SHIFT)
#define PORT_IOCR0_PC2_SHIFT        (19)      /* Bit 19-23: Port Control for Port n Pin 2 */
#define PORT_IOCR0_PC2_MASK         (31 << PORT_IOCR0_PC2_SHIFT)
#  define PORT_IOCR0_PC2(n)         ((uint32_t)(n) << PORT_IOCR0_PC2_SHIFT)
#define PORT_IOCR0_PC3_SHIFT        (27)      /* Bit 27-31: Port Control for Port 0 Pin 3 */
#define PORT_IOCR0_PC3_MASK         (31 << PORT_IOCR0_PC3_SHIFT)
#  define PORT_IOCR0_PC3(n)         ((uint32_t)(n) << PORT_IOCR0_PC3_SHIFT)

/* Port Input/Output Control Register 4 */

#define PORT_IOCR4_PC_SHIFT(p)      ((((p) - 4) << 3) + 3)
#define PORT_IOCR4_PC_MASK(p)       (31 << PORT_IOCR4_PC_SHIFT(p))
#  define PORT_IOCR4_PC(p,n)        ((uint32_t)(n) << PORT_IOCR4_PC_SHIFT(p))
#define PORT_IOCR4_PC4_SHIFT        (3)       /* Bit 3-7: Port Control for Port n Pin 4 */
#define PORT_IOCR4_PC4_MASK         (31 << PORT_IOCR4_PC4_SHIFT)
#  define PORT_IOCR4_PC4(n)         ((uint32_t)(n) << PORT_IOCR4_PC4_SHIFT)
#define PORT_IOCR4_PC5_SHIFT        (11)      /* Bit 11-15: Port Control for Port n Pin 5 */
#define PORT_IOCR4_PC5_MASK         (31 << PORT_IOCR4_PC5_SHIFT)
#  define PORT_IOCR4_PC5(n)         ((uint32_t)(n) << PORT_IOCR4_PC5_SHIFT)
#define PORT_IOCR4_PC6_SHIFT        (19)      /* Bit 19-23: Port Control for Port n Pin 6 */
#define PORT_IOCR4_PC6_MASK         (31 << PORT_IOCR4_PC6_SHIFT)
#  define PORT_IOCR4_PC6(n)         ((uint32_t)(n) << PORT_IOCR4_PC6_SHIFT)
#define PORT_IOCR4_PC7_SHIFT        (27)      /* Bit 27-31: Port Control for Port 0 Pin 7 */
#define PORT_IOCR4_PC7_MASK         (31 << PORT_IOCR4_PC7_SHIFT)
#  define PORT_IOCR4_PC7(n)         ((uint32_t)(n) << PORT_IOCR4_PC7_SHIFT)

/* Port Input/Output Control Register 8 */

#define PORT_IOCR8_PC_SHIFT(p)      ((((p) - 8) << 3) + 3)
#define PORT_IOCR8_PC_MASK(p)       (31 << PORT_IOCR8_PC_SHIFT(p))
#  define PORT_IOCR8_PC(p,n)        ((uint32_t)(n) << PORT_IOCR8_PC_SHIFT(p))
#define PORT_IOCR8_PC8_SHIFT        (3)       /* Bit 3-7: Port Control for Port n Pin 8 */
#define PORT_IOCR8_PC8_MASK         (31 << PORT_IOCR8_PC8_SHIFT)
#  define PORT_IOCR8_PC8(n)         ((uint32_t)(n) << PORT_IOCR8_PC8_SHIFT)
#define PORT_IOCR8_PC9_SHIFT        (11)      /* Bit 11-15: Port Control for Port n Pin 9 */
#define PORT_IOCR8_PC9_MASK         (31 << PORT_IOCR8_PC9_SHIFT)
#  define PORT_IOCR8_PC9(n)         ((uint32_t)(n) << PORT_IOCR8_PC9_SHIFT)
#define PORT_IOCR8_PC10_SHIFT       (19)      /* Bit 19-23: Port Control for Port n Pin 10 */
#define PORT_IOCR8_PC10_MASK        (31 << PORT_IOCR8_PC10_SHIFT)
#  define PORT_IOCR8_PC10(n)        ((uint32_t)(n) << PORT_IOCR8_PC10_SHIFT)
#define PORT_IOCR8_PC11_SHIFT       (27)      /* Bit 17-31: Port Control for Port 0 Pin 11 */
#define PORT_IOCR8_PC11_MASK        (31 << PORT_IOCR8_PC11_SHIFT)
#  define PORT_IOCR8_PC11(n)        ((uint32_t)(n) << PORT_IOCR8_PC11_SHIFT)

/* Port Input/Output Control Register 12 */

#define PORT_IOCR12_PC_SHIFT(p)     ((((p) - 12) << 3) + 3)
#define PORT_IOCR12_PC_MASK(p)      (31 << PORT_IOCR12_PC_SHIFT(p))
#  define PORT_IOCR12_PC(p,n)       ((uint32_t)(n) << PORT_IOCR12_PC_SHIFT(p))
#define PORT_IOCR12_PC12_SHIFT      (3)       /* Bit 3-7: Port Control for Port n Pin 12 */
#define PORT_IOCR12_PC12_MASK       (31 << PORT_IOCR12_PC12_SHIFT)
#  define PORT_IOCR12_PC12(n)       ((uint32_t)(n) << PORT_IOCR12_PC12_SHIFT)
#define PORT_IOCR12_PC13_SHIFT      (11)      /* Bit 3-7: Port Control for Port n Pin 13 */
#define PORT_IOCR12_PC13_MASK       (31 << PORT_IOCR12_PC13_SHIFT)
#  define PORT_IOCR12_PC13(n)       ((uint32_t)(n) << PORT_IOCR12_PC13_SHIFT)
#define PORT_IOCR12_PC14_SHIFT      (19)      /* Bit 3-7: Port Control for Port n Pin 14 */
#define PORT_IOCR12_PC14_MASK       (31 << PORT_IOCR12_PC14_SHIFT)
#  define PORT_IOCR12_PC14(n)       ((uint32_t)(n) << PORT_IOCR12_PC14_SHIFT)
#define PORT_IOCR12_PC15_SHIFT      (27)      /* Bit 3-7: Port Control for Port 0 Pin 15 */
#define PORT_IOCR12_PC15_MASK       (31 << PORT_IOCR12_PC15_SHIFT)
#  define PORT_IOCR12_PC15(n)       ((uint32_t)(n) << PORT_IOCR12_PC15_SHIFT)

/* Pad driver field values */
/* Pad class A1: */

#define PDR_PADA1_MEDIUM           0         /* Medium driver */
#define PDR_PADA1_WEAK             1         /* Weak driver */

/* Pad class A1+: */

#define PDR_PADA1P_STRONGSOFT      0         /* Strong driver soft edge */
#define PDR_PADA1P_STRONGSLOW      1         /* Strong driver slow edge */
#define PDR_PADA1P_MEDIUM          4         /* Medium driver */
#define PDR_PADA1P_WEAK            5         /* Weak driver */

/* Pad class A2: */

#define PDR_PADA2_STRONGSHARP      0         /* Strong driver sharp edge */
#define PDR_PADA2_STRONGMEDIUM     1         /* Strong driver medium edge */
#define PDR_PADA2_STRONGSOFT       2         /* Strong driver soft edge */
#define PDR_PADA2_MEDIUM           4         /* Medium driver */
#define PDR_PADA2_WEAK             7         /* Weak driver */

/* Port Pad Driver Mode 0 Register */

#define PORT_PDR0_PD_SHIFT(p)      ((p) << 2)
#define PORT_PDR0_PD_MASK(p)       (7 << PORT_PDR0_PD_SHIFT(p))
#  define PORT_PDR0_PD(p,n)        ((uint32_t)(n) << PORT_PDR0_PD_SHIFT(p))
#define PORT_PDR0_PD0_SHIFT        (0)       /* Bit 0-2: Pad Driver Mode for Port n Pin 0 */
#define PORT_PDR0_PD0_MASK         (7 << PORT_PDR0_PD0_SHIFT)
#  define PORT_PDR0_PD0(n)         ((uint32_t)(n) << PORT_PDR0_PD0_SHIFT)
#define PORT_PDR0_PD1_SHIFT        (4)       /* Bit 4-6: Pad Driver Mode for Port n Pin 1 */
#define PORT_PDR0_PD1_MASK         (7 << PORT_PDR0_PD1_SHIFT)
#  define PORT_PDR0_PD1(n)         ((uint32_t)(n) << PORT_PDR0_PD1_SHIFT)
#define PORT_PDR0_PD2_SHIFT        (8)       /* Bit 8-10: Pad Driver Mode for Port n Pin 2 */
#define PORT_PDR0_PD2_MASK         (7 << PORT_PDR0_PD2_SHIFT)
#  define PORT_PDR0_PD2(n)         ((uint32_t)(n) << PORT_PDR0_PD2_SHIFT)
#define PORT_PDR0_PD3_SHIFT        (12)      /* Bit 12-14: Pad Driver Mode for Port n Pin 3 */
#define PORT_PDR0_PD3_MASK         (7 << PORT_PDR0_PD3_SHIFT)
#  define PORT_PDR0_PD3(n)         ((uint32_t)(n) << PORT_PDR0_PD3_SHIFT)
#define PORT_PDR0_PD4_SHIFT        (16)      /* Bit 16-18: Pad Driver Mode for Port n Pin 4 */
#define PORT_PDR0_PD4_MASK         (7 << PORT_PDR0_PD4_SHIFT)
#  define PORT_PDR0_PD4(n)         ((uint32_t)(n) << PORT_PDR0_PD4_SHIFT)
#define PORT_PDR0_PD5_SHIFT        (20)      /* Bit 20-22: Pad Driver Mode for Port n Pin 5 */
#define PORT_PDR0_PD5_MASK         (7 << PORT_PDR0_PD5_SHIFT)
#  define PORT_PDR0_PD5(n)         ((uint32_t)(n) << PORT_PDR0_PD5_SHIFT)
#define PORT_PDR0_PD6_SHIFT        (24)      /* Bit 24-26: Pad Driver Mode for Port n Pin 6 */
#define PORT_PDR0_PD6_MASK         (7 << PORT_PDR0_PD6_SHIFT)
#  define PORT_PDR0_PD6(n)         ((uint32_t)(n) << PORT_PDR0_PD6_SHIFT)
#define PORT_PDR0_PD7_SHIFT        (28)      /* Bit 28-30: Pad Driver Mode for Port n Pin 7 */
#define PORT_PDR0_PD7_MASK         (7 << PORT_PDR0_PD7_SHIFT)
#  define PORT_PDR0_PD7(n)         ((uint32_t)(n) << PORT_PDR0_PD7_SHIFT)

/* Port Pad Driver Mode 1 Register */

#define PORT_PDR1_PD_SHIFT(p)      (((p) - 8) << 2)
#define PORT_PDR1_PD_MASK(p)       (7 << PORT_PDR1_PD_SHIFT(p))
#  define PORT_PDR1_PD(p,n)        ((uint32_t)(n) << PORT_PDR1_PD_SHIFT(p))
#define PORT_PDR1_PD8_SHIFT        (0)       /* Bit 0-2: Pad Driver Mode for Port n Pin 8 */
#define PORT_PDR1_PD8_MASK         (7 << PORT_PDR1_PD8_SHIFT)
#  define PORT_PDR1_PD8(n)         ((uint32_t)(n) << PORT_PDR1_PD8_SHIFT)
#define PORT_PDR1_PD9_SHIFT        (4)       /* Bit 4-6: Pad Driver Mode for Port n Pin 9 */
#define PORT_PDR1_PD9_MASK         (7 << PORT_PDR1_PD9_SHIFT)
#  define PORT_PDR1_PD9(n)         ((uint32_t)(n) << PORT_PDR1_PD9_SHIFT)
#define PORT_PDR1_PD10_SHIFT       (8)       /* Bit 8-10: Pad Driver Mode for Port n Pin 10 */
#define PORT_PDR1_PD10_MASK        (7 << PORT_PDR1_PD10_SHIFT)
#  define PORT_PDR1_PD10(n)        ((uint32_t)(n) << PORT_PDR1_PD10_SHIFT)
#define PORT_PDR1_PD11_SHIFT       (12)      /* Bit 12-14: Pad Driver Mode for Port n Pin 11 */
#define PORT_PDR1_PD11_MASK        (7 << PORT_PDR1_PD11_SHIFT)
#  define PORT_PDR1_PD11(n)        ((uint32_t)(n) << PORT_PDR1_PD11_SHIFT)
#define PORT_PDR1_PD12_SHIFT       (16)      /* Bit 16-18: Pad Driver Mode for Port n Pin 12 */
#define PORT_PDR1_PD12_MASK        (7 << PORT_PDR1_PD12_SHIFT)
#  define PORT_PDR1_PD12(n)        ((uint32_t)(n) << PORT_PDR1_PD12_SHIFT)
#define PORT_PDR1_PD13_SHIFT       (20)      /* Bit 20-22: Pad Driver Mode for Port n Pin 13 */
#define PORT_PDR1_PD13_MASK        (7 << PORT_PDR1_PD13_SHIFT)
#  define PORT_PDR1_PD13(n)        ((uint32_t)(n) << PORT_PDR1_PD13_SHIFT)
#define PORT_PDR1_PD14_SHIFT       (24)      /* Bit 24-26: Pad Driver Mode for Port n Pin 14 */
#define PORT_PDR1_PD14_MASK        (7 << PORT_PDR1_PD14_SHIFT)
#  define PORT_PDR1_PD14(n)        ((uint32_t)(n) << PORT_PDR1_PD14_SHIFT)
#define PORT_PDR1_PD15_SHIFT       (28)      /* Bit 28-30: Pad Driver Mode for Port n Pin 15 */
#define PORT_PDR1_PD15_MASK        (7 << PORT_PDR1_PD15_SHIFT)
#  define PORT_PDR1_PD15(n)        ((uint32_t)(n) << PORT_PDR1_PD15_SHIFT)

/* Hardware select field values */

#define HWSEL_SW                   0          /* Software control only */
#define HWSEL_HW0                  1          /* HWI0/HWO0 control path can override
                                               * the software configuration */
#define HWSEL_HW1                  2          /* HWI1/HWO1 control path can override
                                               * the software configuration */

/* Port Pin Hardware Select Register */

#define PORT_HWSEL_HW_SHIFT(p)      ((p) << 1)
#define PORT_HWSEL_HW_MASK(p)       (3 << PORT_HWSEL_HW_SHIFT(p))
#  define PORT_HWSEL_HW(p,n)        ((uint32_t)(n) << PORT_HWSEL_HW_SHIFT(p))
#define PORT_HWSEL_HW0_SHIFT        (0)       /* Bit 0-1: Port n Pin 0 Hardware Select */
#define PORT_HWSEL_HW0_MASK         (3 << PORT_HWSEL_HW0_SHIFT)
#  define PORT_HWSEL_HW0(n)         ((uint32_t)(n) << PORT_HWSEL_HW0_SHIFT)
#define PORT_HWSEL_HW1_SHIFT        (2)       /* Bit 2-3: Port n Pin 1 Hardware Select */
#define PORT_HWSEL_HW1_MASK         (3 << PORT_HWSEL_HW1_SHIFT)
#  define PORT_HWSEL_HW1(n)         ((uint32_t)(n) << PORT_HWSEL_HW1_SHIFT)
#define PORT_HWSEL_HW2_SHIFT        (4)       /* Bit 4-5: Port n Pin 2 Hardware Select */
#define PORT_HWSEL_HW2_MASK         (3 << PORT_HWSEL_HW2_SHIFT)
#  define PORT_HWSEL_HW2(n)         ((uint32_t)(n) << PORT_HWSEL_HW2_SHIFT)
#define PORT_HWSEL_HW3_SHIFT        (6)       /* Bit 6-7: Port 0 Pin 3 Hardware Select */
#define PORT_HWSEL_HW3_MASK         (3 << PORT_HWSEL_HW3_SHIFT)
#  define PORT_HWSEL_HW3(n)         ((uint32_t)(n) << PORT_HWSEL_HW3_SHIFT)
#define PORT_HWSEL_HW4_SHIFT        (8)      /* Bit 8-9: Port 0 Pin 4 Hardware Select */
#define PORT_HWSEL_HW4_MASK         (3 << PORT_HWSEL_HW4_SHIFT)
#  define PORT_HWSEL_HW4(n)         ((uint32_t)(n) << PORT_HWSEL_HW4_SHIFT)
#define PORT_HWSEL_HW5_SHIFT        (10)      /* Bit 10-11: Port 0 Pin 5 Hardware Select */
#define PORT_HWSEL_HW5_MASK         (3 << PORT_HWSEL_HW5_SHIFT)
#  define PORT_HWSEL_HW5(n)         ((uint32_t)(n) << PORT_HWSEL_HW5_SHIFT)
#define PORT_HWSEL_HW6_SHIFT        (12)      /* Bit 12-13: Port 0 Pin 6 Hardware Select */
#define PORT_HWSEL_HW6_MASK         (3 << PORT_HWSEL_HW6_SHIFT)
#  define PORT_HWSEL_HW6(n)         14uint32_t)(n) << PORT_HWSEL_HW6_SHIFT)
#define PORT_HWSEL_HW7_SHIFT        (14)      /* Bit 14-15: Port 0 Pin 7 Hardware Select */
#define PORT_HWSEL_HW7_MASK         (3 << PORT_HWSEL_HW7_SHIFT)
#  define PORT_HWSEL_HW7(n)         ((uint32_t)(n) << PORT_HWSEL_HW7_SHIFT)
#define PORT_HWSEL_HW8_SHIFT        (16)      /* Bit 16-17: Port n Pin 8 Hardware Select */
#define PORT_HWSEL_HW8_MASK         (3 << PORT_HWSEL_HW8_SHIFT)
#  define PORT_HWSEL_HW8(n)         ((uint32_t)(n) << PORT_HWSEL_HW8_SHIFT)
#define PORT_HWSEL_HW9_SHIFT        (18)      /* Bit 18-19: Port n Pin 9 Hardware Select */
#define PORT_HWSEL_HW9_MASK         (3 << PORT_HWSEL_HW9_SHIFT)
#  define PORT_HWSEL_HW9(n)         ((uint32_t)(n) << PORT_HWSEL_HW9_SHIFT)
#define PORT_HWSEL_HW10_SHIFT       (20)      /* Bit 20-21: Port n Pin 10 Hardware Select */
#define PORT_HWSEL_HW10_MASK        (3 << PORT_HWSEL_HW10_SHIFT)
#  define PORT_HWSEL_HW10(n)        ((uint32_t)(n) << PORT_HWSEL_HW10_SHIFT)
#define PORT_HWSEL_HW11_SHIFT       (22)      /* Bit 22-23: Port 0 Pin 11 Hardware Select */
#define PORT_HWSEL_HW11_MASK        (3 << PORT_HWSEL_HW11_SHIFT)
#  define PORT_HWSEL_HW11(n)        ((uint32_t)(n) << PORT_HWSEL_HW11_SHIFT)
#define PORT_HWSEL_HW12_SHIFT       (24)      /* Bit 24-25: Port 0 Pin 12 Hardware Select */
#define PORT_HWSEL_HW12_MASK        (3 << PORT_HWSEL_HW12_SHIFT)
#  define PORT_HWSEL_HW12(n)        ((uint32_t)(n) << PORT_HWSEL_HW12_SHIFT)
#define PORT_HWSEL_HW13_SHIFT       (26)      /* Bit 26-27: Port 0 Pin 13 Hardware Select */
#define PORT_HWSEL_HW13_MASK        (3 << PORT_HWSEL_HW13_SHIFT)
#  define PORT_HWSEL_HW13(n)        ((uint32_t)(n) << PORT_HWSEL_HW13_SHIFT)
#define PORT_HWSEL_HW14_SHIFT       (28)      /* Bit 28-29: Port 0 Pin 14 Hardware Select */
#define PORT_HWSEL_HW14_MASK        (3 << PORT_HWSEL_HW14_SHIFT)
#  define PORT_HWSEL_HW14(n)        14uint32_t)(n) << PORT_HWSEL_HW14_SHIFT)
#define PORT_HWSEL_HW15_SHIFT       (30)      /* Bit 30-31: Port 0 Pin 15 Hardware Select */
#define PORT_HWSEL_HW15_MASK        (3 << PORT_HWSEL_HW15_SHIFT)
#  define PORT_HWSEL_HW15(n)        ((uint32_t)(n) << PORT_HWSEL_HW15_SHIFT)

#endif /* __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_PORTS_H */
