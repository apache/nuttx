/************************************************************************************
 * arch/arm/src/xmc4/chip/xmc4_ports.h
 *
 *   Copyright (C    /*2017 Gregory Nutt. All rights reserved.
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION    /*HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE    /*ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C    /*2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon    /*is supplying this software for use with
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

#ifndef __ARCH_ARM_SRC_XMC4_CHIP_XMC4_PORTS_H
#define __ARCH_ARM_SRC_XMC4_CHIP_XMC4_PORTS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

/* PORTS Registers */

#define XMC4_PORTS_OUT_OFFSET       0x0000    /* Port Output Register */
#define XMC4_PORTS_OMR_OFFSET       0x0004    /* Port Output Modification Register */
#define XMC4_PORTS_IOCR0_OFFSET     0x0010    /* Port Input/Output Control Register 0 */
#define XMC4_PORTS_IOCR4_OFFSET     0x0014    /* Port Input/Output Control Register 4 */
#define XMC4_PORTS_IOCR8_OFFSET     0x0018    /* Port Input/Output Control Register 8 */
#define XMC4_PORTS_IOCR12_OFFSET    0x001c    /* Port Input/Output Control Register 12 */
#define XMC4_PORTS_IN_OFFSET        0x0024    /* Port Input Register */
#define XMC4_PORTS_PDR0_OFFSET      0x0040    /* Port Pad Driver Mode 0 Register */
#define XMC4_PORTS_PDR1_OFFSET      0x0044    /* Port Pad Driver Mode 1 Register */
#define XMC4_PORTS_PDISC_OFFSET     0x0060    /* Port Pin Function Decision Control Register */
#define XMC4_PORTS_PPS_OFFSET       0x0070    /* Port Pin Power Save Register */
#define XMC4_PORTS_HWSEL_OFFSET     0x0074    /* Port Pin Hardware Select Register */

/* Register Addresses ****************************************************************/
#define              0x48028000
#define XMC4_PORT1_BASE             0x48028100
#define XMC4_PORT2_BASE             0x48028200
#define XMC4_PORT3_BASE             0x48028300
#define XMC4_PORT4_BASE             0x48028400
#define XMC4_PORT5_BASE             0x48028500
#define XMC4_PORT6_BASE             0x48028600
#define XMC4_PORT7_BASE             0x48028700
#define XMC4_PORT8_BASE             0x48028800
#define XMC4_PORT9_BASE             0x48028900
#define XMC4_PORT14_BASE            0x48028e00
#define XMC4_PORT15_BASE            0x48028f00

#define XMC4_PORT0_OUT              (XMC4_PORT0_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT0_OMR              (XMC4_PORT0_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT0_IOCR0            (XMC4_PORT0_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT0_IOCR4            (XMC4_PORT0_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT0_IOCR8            (XMC4_PORT0_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT0_IOCR12           (XMC4_PORT0_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT0_IN               (XMC4_PORT0_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT0_PDR0             (XMC4_PORT0_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT0_PDR1             (XMC4_PORT0_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT0_PDISC            (XMC4_PORT0_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT0_PPS              (XMC4_PORT0_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT0_HWSEL            (XMC4_PORT0_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT1_OUT              (XMC4_PORT1_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT1_OMR              (XMC4_PORT1_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT1_IOCR0            (XMC4_PORT1_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT1_IOCR4            (XMC4_PORT1_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT1_IOCR8            (XMC4_PORT1_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT1_IOCR12           (XMC4_PORT1_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT1_IN               (XMC4_PORT1_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT1_PDR0             (XMC4_PORT1_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT1_PDR1             (XMC4_PORT1_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT1_PDISC            (XMC4_PORT1_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT1_PPS              (XMC4_PORT1_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT1_HWSEL            (XMC4_PORT1_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT2_OUT              (XMC4_PORT2_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT2_OMR              (XMC4_PORT2_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT2_IOCR0            (XMC4_PORT2_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT2_IOCR4            (XMC4_PORT2_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT2_IOCR8            (XMC4_PORT2_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT2_IOCR12           (XMC4_PORT2_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT2_IN               (XMC4_PORT2_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT2_PDR0             (XMC4_PORT2_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT2_PDR1             (XMC4_PORT2_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT2_PDISC            (XMC4_PORT2_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT2_PPS              (XMC4_PORT2_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT2_HWSEL            (XMC4_PORT2_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT3_OUT              (XMC4_PORT3_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT3_OMR              (XMC4_PORT3_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT3_IOCR0            (XMC4_PORT3_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT3_IOCR4            (XMC4_PORT3_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT3_IOCR8            (XMC4_PORT3_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT3_IOCR12           (XMC4_PORT3_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT3_IN               (XMC4_PORT3_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT3_PDR0             (XMC4_PORT3_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT3_PDR1             (XMC4_PORT3_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT3_PDISC            (XMC4_PORT3_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT3_PPS              (XMC4_PORT3_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT3_HWSEL            (XMC4_PORT3_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT4_OUT              (XMC4_PORT4_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT4_OMR              (XMC4_PORT4_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT4_IOCR0            (XMC4_PORT4_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT4_IOCR4            (XMC4_PORT4_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT4_IOCR8            (XMC4_PORT4_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT4_IOCR12           (XMC4_PORT4_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT4_IN               (XMC4_PORT4_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT4_PDR0             (XMC4_PORT4_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT4_PDR1             (XMC4_PORT4_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT4_PDISC            (XMC4_PORT4_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT4_PPS              (XMC4_PORT4_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT4_HWSEL            (XMC4_PORT4_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT5_OUT              (XMC4_PORT5_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT5_OMR              (XMC4_PORT5_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT5_IOCR0            (XMC4_PORT5_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT5_IOCR4            (XMC4_PORT5_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT5_IOCR8            (XMC4_PORT5_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT5_IOCR12           (XMC4_PORT5_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT5_IN               (XMC4_PORT5_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT5_PDR0             (XMC4_PORT5_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT5_PDR1             (XMC4_PORT5_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT5_PDISC            (XMC4_PORT5_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT5_PPS              (XMC4_PORT5_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT5_HWSEL            (XMC4_PORT5_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT6_OUT              (XMC4_PORT6_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT6_OMR              (XMC4_PORT6_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT6_IOCR0            (XMC4_PORT6_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT6_IOCR4            (XMC4_PORT6_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT6_IOCR8            (XMC4_PORT6_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT6_IOCR12           (XMC4_PORT6_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT6_IN               (XMC4_PORT6_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT6_PDR0             (XMC4_PORT6_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT6_PDR1             (XMC4_PORT6_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT6_PDISC            (XMC4_PORT6_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT6_PPS              (XMC4_PORT6_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT6_HWSEL            (XMC4_PORT6_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT7_OUT              (XMC4_PORT7_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT7_OMR              (XMC4_PORT7_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT7_IOCR0            (XMC4_PORT7_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT7_IOCR4            (XMC4_PORT7_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT7_IOCR8            (XMC4_PORT7_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT7_IOCR12           (XMC4_PORT7_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT7_IN               (XMC4_PORT7_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT7_PDR0             (XMC4_PORT7_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT7_PDR1             (XMC4_PORT7_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT7_PDISC            (XMC4_PORT7_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT7_PPS              (XMC4_PORT7_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT7_HWSEL            (XMC4_PORT7_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT8_OUT              (XMC4_PORT8_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT8_OMR              (XMC4_PORT8_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT8_IOCR0            (XMC4_PORT8_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT8_IOCR4            (XMC4_PORT8_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT8_IOCR8            (XMC4_PORT8_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT8_IOCR12           (XMC4_PORT8_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT8_IN               (XMC4_PORT8_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT8_PDR0             (XMC4_PORT8_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT8_PDR1             (XMC4_PORT8_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT8_PDISC            (XMC4_PORT8_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT8_PPS              (XMC4_PORT8_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT8_HWSEL            (XMC4_PORT8_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT9_OUT              (XMC4_PORT9_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT9_OMR              (XMC4_PORT9_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT9_IOCR0            (XMC4_PORT9_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT9_IOCR4            (XMC4_PORT9_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT9_IOCR8            (XMC4_PORT9_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT9_IOCR12           (XMC4_PORT9_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT9_IN               (XMC4_PORT9_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT9_PDR0             (XMC4_PORT9_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT9_PDR1             (XMC4_PORT9_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT9_PDISC            (XMC4_PORT9_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT9_PPS              (XMC4_PORT9_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT9_HWSEL            (XMC4_PORT9_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT14_OUT             (XMC4_PORT14_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT14_OMR             (XMC4_PORT14_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT14_IOCR0           (XMC4_PORT14_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT14_IOCR4           (XMC4_PORT14_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT14_IOCR8           (XMC4_PORT14_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT14_IOCR12          (XMC4_PORT14_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT14_IN              (XMC4_PORT14_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT14_PDR0            (XMC4_PORT14_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT14_PDR1            (XMC4_PORT14_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT14_PDISC           (XMC4_PORT14_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT14_PPS             (XMC4_PORT14_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT14_HWSEL           (XMC4_PORT14_BASE+XMC4_PORTS_HWSEL_OFFSET)

#define XMC4_PORT15_OUT             (XMC4_PORT15_BASE+XMC4_PORTS_OUT_OFFSET)
#define XMC4_PORT15_OMR             (XMC4_PORT15_BASE+XMC4_PORTS_OMR_OFFSET)
#define XMC4_PORT15_IOCR0           (XMC4_PORT15_BASE+XMC4_PORTS_IOCR0_OFFSET)
#define XMC4_PORT15_IOCR4           (XMC4_PORT15_BASE+XMC4_PORTS_IOCR4_OFFSET)
#define XMC4_PORT15_IOCR8           (XMC4_PORT15_BASE+XMC4_PORTS_IOCR8_OFFSET)
#define XMC4_PORT15_IOCR12          (XMC4_PORT15_BASE+XMC4_PORTS_IOCR12_OFFSET)
#define XMC4_PORT15_IN              (XMC4_PORT15_BASE+XMC4_PORTS_IN_OFFSET)
#define XMC4_PORT15_PDR0            (XMC4_PORT15_BASE+XMC4_PORTS_PDR0_OFFSET)
#define XMC4_PORT15_PDR1            (XMC4_PORT15_BASE+XMC4_PORTS_PDR1_OFFSET)
#define XMC4_PORT15_PDISC           (XMC4_PORT15_BASE+XMC4_PORTS_PDISC_OFFSET)
#define XMC4_PORT15_PPS             (XMC4_PORT15_BASE+XMC4_PORTS_PPS_OFFSET)
#define XMC4_PORT15_HWSEL           (XMC4_PORT15_BASE+XMC4_PORTS_HWSEL_OFFSET)

/* Register Bit-Field Definitions **************************************************/

/* Port Output Register */
#define PORTS_OUT_
/* Port Output Modification Register */
#define PORTS_OMR_
/* Port Input/Output Control Register 0 */
#define PORTS_IOCR0_
/* Port Input/Output Control Register 4 */
#define PORTS_IOCR4_
/* Port Input/Output Control Register 8 */
#define PORTS_IOCR8_
/* Port Input/Output Control Register 12 */
#define PORTS_IOCR12_
/* Port Input Register */
#define PORTS_IN_
/* Port Pad Driver Mode 0 Register */
#define PORTS_PDR0_
/* Port Pad Driver Mode 1 Register */
#define PORTS_PDR1_
/* Port Pin Function Decision Control Register */
#define PORTS_PDISC_
/* Port Pin Power Save Register */
#define PORTS_PPS_
/* Port Pin Hardware Select Register */
#define PORTS_HWSEL_

#endif /* __ARCH_ARM_SRC_XMC4_CHIP_XMC4_SCU_H */
