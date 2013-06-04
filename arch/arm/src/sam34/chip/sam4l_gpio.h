/****************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_gpio.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_GPIO_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_GPIO_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PIO register offsets *****************************************************************/

#define SAM_GPIO_GPER_OFFSET       0x0000 /* GPIO Enable Register Read/Write */
#define SAM_GPIO_GPERS_OFFSET      0x0004 /* GPIO Enable Register Set */
#define SAM_GPIO_GPERC_OFFSET      0x0008 /* GPIO Enable Register Clear */
#define SAM_GPIO_GPERT_OFFSET      0x000c /* GPIO Enable Register Toggle */

/* {PMR2, PMR1, PMR0} Selected Peripheral Function
 *
 *   000 GPIO 100 D
 *   001 A    101 E
 *   010 B    110 F
 *   011 C    111 G
 *
 * NOTE:  Labeling in the data sheet is inconsistent.  In the pin multiplexing table,
 * It shows GPIO functions A-G with 000 apparently corresponding to the GPIO.  In the
 * register description, it should A-H with presumably A corresponding to 000.  Here
 * we adopt the above convention.
 */

#define SAM_GPIO_PMR0_OFFSET       0x0010 /* Peripheral Mux Register 0 Read/Write */
#define SAM_GPIO_PMR0S_OFFSET      0x0014 /* Peripheral Mux Register 0 Set */
#define SAM_GPIO_PMR0C_OFFSET      0x0018 /* Peripheral Mux Register 0 Clear */
#define SAM_GPIO_PMR0T_OFFSET      0x001c /* Peripheral Mux Register 0 Toggle */

#define SAM_GPIO_PMR1_OFFSET       0x0020 /* Peripheral Mux Register 1 Read/Write */
#define SAM_GPIO_PMR1S_OFFSET      0x0024 /* Peripheral Mux Register 1 Set */
#define SAM_GPIO_PMR1C_OFFSET      0x0028 /* Peripheral Mux Register 1 Clear */
#define SAM_GPIO_PMR1T_OFFSET      0x002c /* Peripheral Mux Register 1 Toggle */

#define SAM_GPIO_PMR2_OFFSET       0x0030 /* Peripheral Mux Register 2 Read/Write */
#define SAM_GPIO_PMR2S_OFFSET      0x0034 /* Peripheral Mux Register 2 Set */
#define SAM_GPIO_PMR2C_OFFSET      0x0038 /* Peripheral Mux Register 2 Clear */
#define SAM_GPIO_PMR2T_OFFSET      0x003c /* Peripheral Mux Register 2 Toggle */

#define SAM_GPIO_ODER_OFFSET       0x0040 /* Output Driver Enable Register Read/Write */
#define SAM_GPIO_ODERS_OFFSET      0x0044 /* Output Driver Enable Register Set */
#define SAM_GPIO_ODERC_OFFSET      0x0048 /* Output Driver Enable Register Clear */
#define SAM_GPIO_ODERT_OFFSET      0x004c /* Output Driver Enable Register Toggle */

#define SAM_GPIO_OVR_OFFSET        0x0050 /* Output Value Register Read/Write */
#define SAM_GPIO_OVRS_OFFSET       0x0054 /* Output Value Register Set */
#define SAM_GPIO_OVRC_OFFSET       0x0058 /* Output Value Register Clear */
#define SAM_GPIO_OVRT_OFFSET       0x005c /* Output Value Register Toggle */

/* Pin Value Register Read (4 registers)*/

#define SAM_GPIO_PVR_OFFSET(n)     (0x0060 + (((n) & ~31) >> 3))
#define SAM_GPIO_PVR0_OFFSET       0x0060 /* Pin Value Register 0 Read*/
#define SAM_GPIO_PVR1_OFFSET       0x0064 /* Pin Value Register 1 Read*/
#define SAM_GPIO_PVR2_OFFSET       0x0068 /* Pin Value Register 2 Read*/
#define SAM_GPIO_PVR3_OFFSET       0x006c /* Pin Value Register 3 Read*/

/* {PUER, PDER} Selected Function
 *
 *   00 Disabled
 *   01 Pull-down enabled
 *   10 Pull-up enabled
 *   11 Buskeeper enabled
 */

#define SAM_GPIO_PUER_OFFSET       0x0070 /* Pull-up Enable Register Read/Write */
#define SAM_GPIO_PUERS_OFFSET      0x0074 /* Pull-up Enable Register Set */
#define SAM_GPIO_PUERC_OFFSET      0x0078 /* Pull-up Enable Register Clear*/ 
#define SAM_GPIO_PUERT_OFFSET      0x007c /* Pull-up Enable Register Toggle */

#define SAM_GPIO_PDER_OFFSET       0x0080 /* Pull-down Enable Register Read/Write */
#define SAM_GPIO_PDERS_OFFSET      0x0084 /* Pull-down Enable Register Set */
#define SAM_GPIO_PDERC_OFFSET      0x0088 /* Pull-down Enable Register Clear */
#define SAM_GPIO_PDERT_OFFSET      0x008c /* Pull-down Enable Register Toggle */

#define SAM_GPIO_IER_OFFSET        0x0090 /* Interrupt Enable Register Read/Write */
#define SAM_GPIO_IERS_OFFSET       0x0094 /* Interrupt Enable Register Set */
#define SAM_GPIO_IERC_OFFSET       0x0098 /* Interrupt Enable Register Clear */
#define SAM_GPIO_IERT_OFFSET       0x009c /* Interrupt Enable Register Toggle */

/* {IMR1, IMR0} Interrupt Mode
 *
 *   00 Pin Change
 *   01 Rising Edge
 *   10 Falling Edge
 *   11 Reserved
 */

#define SAM_GPIO_IMR0_OFFSET       0x00a0 /* Interrupt Mode Register 0 Read/Write */
#define SAM_GPIO_IMR0S_OFFSET      0x00a4 /* Interrupt Mode Register 0 Set */
#define SAM_GPIO_IMR0C_OFFSET      0x00a8 /* Interrupt Mode Register 0 Clear */
#define SAM_GPIO_IMR0T_OFFSET      0x00ac /* Interrupt Mode Register 0 Toggle */

#define SAM_GPIO_IMR1_OFFSET       0x00b0 /* Interrupt Mode Register 1 Read/Write */
#define SAM_GPIO_IMR1S_OFFSET      0x00b4 /* Interrupt Mode Register 1 Set */
#define SAM_GPIO_IMR1C_OFFSET      0x00b8 /* Interrupt Mode Register 1 Clear */
#define SAM_GPIO_IMR1T_OFFSET      0x00bc /* Interrupt Mode Register 1 Toggle */

#define SAM_GPIO_GFER_OFFSET       0x00c0 /* Glitch Filter Enable Register Read/Write */
#define SAM_GPIO_GFERS_OFFSET      0x00c4 /* Glitch Filter Enable Register Set */
#define SAM_GPIO_GFERC_OFFSET      0x00c8 /* Glitch Filter Enable Register Clear */
#define SAM_GPIO_GFERT_OFFSET      0x00cc /* Glitch Filter Enable Register Toggle */

/* Interrupt Flag Register Read (2 registers)*/

#define SAM_GPIO_IFR_OFFSET(n)     (0x00d0 + (((n) & ~31) >> 3))
#define SAM_GPIO_IFR0_OFFSET       0x00d0 /* Interrupt Flag Register 0 Read */
#define SAM_GPIO_IFR1_OFFSET       0x00d4 /* Interrupt Flag Register 0 Read */

/* Interrupt Flag Register Clear (2 registers)*/

#define SAM_GPIO_IFRC_OFFSET(n)    (0x00d8 + (((n) & ~31) >> 3))
#define SAM_GPIO_IFRC0_OFFSET      0x00d8 /* Interrupt Flag Register 0 Clear */
#define SAM_GPIO_IFRC1_OFFSET      0x00dc /* Interrupt Flag Register 1 Clear */

/* {ODCR1, ODCR0} Interrupt Mode
 *
 *   00 Lowest drive strength
 *   01 ...
 *   10 ...
 *   11 Highest drive strength
 */

#define SAM_GPIO_ODCR0_OFFSET      0x0100 /* Output Driving Capability Register 0 Read/Write */
#define SAM_GPIO_ODCR0S_OFFSET     0x0104 /* Output Driving Capability Register 0 Set */
#define SAM_GPIO_ODCR0C_OFFSET     0x0108 /* Output Driving Capability Register 0 Clear */
#define SAM_GPIO_ODCR0T_OFFSET     0x010c /* Output Driving Capability Register 0 Toggle */

#define SAM_GPIO_ODCR1_OFFSET      0x0110 /* Output Driving Capability Register 1 Read */
#define SAM_GPIO_ODCR1S_OFFSET     0x0114 /* Output Driving Capability Register 1 Set */
#define SAM_GPIO_ODCR1C_OFFSET     0x0118 /* Output Driving Capability Register 1 Clear */
#define SAM_GPIO_ODCR1T_OFFSET     0x011c /* Output Driving Capability Register 1 Toggle */

#define SAM_GPIO_OSRR0_OFFSET      0x0130 /* Output Slew Rate Register 0 Read */
#define SAM_GPIO_OSRR0S_OFFSET     0x0134 /* Output Slew Rate Register 0 Set */
#define SAM_GPIO_OSRR0C_OFFSET     0x0138 /* Output Slew Rate Register 0 Clear */
#define SAM_GPIO_OSRR0T_OFFSET     0x013c /* Output Slew Rate Register 0 Toggle */

#define SAM_GPIO_STER_OFFSET       0x0160 /* Schmitt Trigger Enable Register Read */
#define SAM_GPIO_STERS_OFFSET      0x0164 /* Schmitt Trigger Enable Register Set */
#define SAM_GPIO_STERC_OFFSET      0x0168 /* Schmitt Trigger Enable Register Clear */
#define SAM_GPIO_STERT_OFFSET      0x016c /* Schmitt Trigger Enable Register Toggle */

#define SAM_GPIO_EVER_OFFSET       0x0180 /* Event Enable Register Read */
#define SAM_GPIO_EVERS_OFFSET      0x0184 /* Event Enable Register Set */
#define SAM_GPIO_EVERC_OFFSET      0x0188 /* Event Enable Register Clear */
#define SAM_GPIO_EVERT_OFFSET      0x018c /* Event Enable Register Toggle */

#define SAM_GPIO_PARAMETER_OFFSET  0x01f8 /* Parameter Register Read */
#define SAM_GPIO_VERSION_OFFSET    0x01fc /* Version Register Read */

/* GPIO register adresses ***************************************************************/

#define SAM_GPIO_GPER              (SAM_GPIO_BASE+SAM_GPIO_GPER_OFFSET)
#define SAM_GPIO_GPERS             (SAM_GPIO_BASE+SAM_GPIO_GPERS_OFFSET)
#define SAM_GPIO_GPERC             (SAM_GPIO_BASE+SAM_GPIO_GPERC_OFFSET)
#define SAM_GPIO_GPERT             (SAM_GPIO_BASE+SAM_GPIO_GPERT_OFFSET)

#define SAM_GPIO_PMR0              (SAM_GPIO_BASE+SAM_GPIO_PMR0_OFFSET)
#define SAM_GPIO_PMR0S             (SAM_GPIO_BASE+SAM_GPIO_PMR0S_OFFSET)
#define SAM_GPIO_PMR0C             (SAM_GPIO_BASE+SAM_GPIO_PMR0C_OFFSET)
#define SAM_GPIO_PMR0T             (SAM_GPIO_BASE+SAM_GPIO_PMR0T_OFFSET_

#define SAM_GPIO_PMR1              (SAM_GPIO_BASE+SAM_GPIO_PMR1_OFFSET)
#define SAM_GPIO_PMR1S             (SAM_GPIO_BASE+SAM_GPIO_PMR1S_OFFSET)
#define SAM_GPIO_PMR1C             (SAM_GPIO_BASE+SAM_GPIO_PMR1C_OFFSET)
#define SAM_GPIO_PMR1T             (SAM_GPIO_BASE+SAM_GPIO_PMR1T_OFFSET)

#define SAM_GPIO_PMR2              (SAM_GPIO_BASE+SAM_GPIO_PMR2_OFFSET)
#define SAM_GPIO_PMR2S             (SAM_GPIO_BASE+SAM_GPIO_PMR2S_OFFSET)
#define SAM_GPIO_PMR2C             (SAM_GPIO_BASE+SAM_GPIO_PMR2C_OFFSET)
#define SAM_GPIO_PMR2T             (SAM_GPIO_BASE+SAM_GPIO_PMR2T_OFFSET)

#define SAM_GPIO_ODER              (SAM_GPIO_BASE+SAM_GPIO_ODER_OFFSET)
#define SAM_GPIO_ODERS             (SAM_GPIO_BASE+SAM_GPIO_ODERS_OFFSET)
#define SAM_GPIO_ODERC             (SAM_GPIO_BASE+SAM_GPIO_ODERC_OFFSET)
#define SAM_GPIO_ODERT             (SAM_GPIO_BASE+SAM_GPIO_ODERT_OFFSET)

#define SAM_GPIO_OVR               (SAM_GPIO_BASE+SAM_GPIO_OVR_OFFSET)
#define SAM_GPIO_OVRS              (SAM_GPIO_BASE+SAM_GPIO_OVRS_OFFSET)
#define SAM_GPIO_OVRC              (SAM_GPIO_BASE+SAM_GPIO_OVRC_OFFSET)
#define SAM_GPIO_OVRT              (SAM_GPIO_BASE+SAM_GPIO_OVRT_OFFSET)

/* Pin Value Register Read (4 registers)*/

#define SAM_GPIO_PVR(n)            (SAM_GPIO_BASE+SAM_GPIO_PVR_OFFSET(n))
#define SAM_GPIO_PVR0              (SAM_GPIO_BASE+SAM_GPIO_PVR0_OFFSET)
#define SAM_GPIO_PVR1              (SAM_GPIO_BASE+SAM_GPIO_PVR1_OFFSET)
#define SAM_GPIO_PVR2              (SAM_GPIO_BASE+SAM_GPIO_PVR2_OFFSET)
#define SAM_GPIO_PVR3              (SAM_GPIO_BASE+SAM_GPIO_PVR3_OFFSET)

#define SAM_GPIO_PUER              (SAM_GPIO_BASE+SAM_GPIO_PUER_OFFSET)
#define SAM_GPIO_PUERS             (SAM_GPIO_BASE+SAM_GPIO_PUERS_OFFSET)
#define SAM_GPIO_PUERC             (SAM_GPIO_BASE+SAM_GPIO_PUERC_OFFSET)
#define SAM_GPIO_PUERT             (SAM_GPIO_BASE+SAM_GPIO_PUERT_OFFSET)

#define SAM_GPIO_PDER              (SAM_GPIO_BASE+SAM_GPIO_PDER_OFFSET)
#define SAM_GPIO_PDERS             (SAM_GPIO_BASE+SAM_GPIO_PDERS_OFFSET)
#define SAM_GPIO_PDERC             (SAM_GPIO_BASE+SAM_GPIO_PDERC_OFFSET)
#define SAM_GPIO_PDERT             (SAM_GPIO_BASE+SAM_GPIO_PDERT_OFFSET)

#define SAM_GPIO_IER               (SAM_GPIO_BASE+SAM_GPIO_IER_OFFSET)
#define SAM_GPIO_IERS              (SAM_GPIO_BASE+SAM_GPIO_IERS_OFFSET)
#define SAM_GPIO_IERC              (SAM_GPIO_BASE+SAM_GPIO_IERC_OFFSET)
#define SAM_GPIO_IERT              (SAM_GPIO_BASE+SAM_GPIO_IERT_OFFSET)

#define SAM_GPIO_IMR0              (SAM_GPIO_BASE+SAM_GPIO_IMR0_OFFSET)
#define SAM_GPIO_IMR0S             (SAM_GPIO_BASE+SAM_GPIO_IMR0S_OFFSET)
#define SAM_GPIO_IMR0C             (SAM_GPIO_BASE+SAM_GPIO_IMR0C_OFFSET)
#define SAM_GPIO_IMR0T             (SAM_GPIO_BASE+SAM_GPIO_IMR0T_OFFSET)

#define SAM_GPIO_IMR1              (SAM_GPIO_BASE+SAM_GPIO_IMR1_OFFSET)
#define SAM_GPIO_IMR1S             (SAM_GPIO_BASE+SAM_GPIO_IMR1S_OFFSET)
#define SAM_GPIO_IMR1C             (SAM_GPIO_BASE+SAM_GPIO_IMR1C_OFFSET)
#define SAM_GPIO_IMR1T             (SAM_GPIO_BASE+SAM_GPIO_IMR1T_OFFSET)

#define SAM_GPIO_GFER              (SAM_GPIO_BASE+SAM_GPIO_GFER_OFFSET)
#define SAM_GPIO_GFERS             (SAM_GPIO_BASE+SAM_GPIO_GFERS_OFFSET)
#define SAM_GPIO_GFERC             (SAM_GPIO_BASE+SAM_GPIO_GFERC_OFFSET)
#define SAM_GPIO_GFERT             (SAM_GPIO_BASE+SAM_GPIO_GFERT_OFFSET)

/* Interrupt Flag Register Read (2 registers)*/

#define SAM_GPIO_IFR(n)            (SAM_GPIO_BASE+SAM_GPIO_IFR_OFFSET(n))
#define SAM_GPIO_IFR0              (SAM_GPIO_BASE+SAM_GPIO_IFR0_OFFSET)
#define SAM_GPIO_IFR1              (SAM_GPIO_BASE+SAM_GPIO_IFR1_OFFSET)

/* Interrupt Flag Register Clear (2 registers)*/

#define SAM_GPIO_IFRC(n)           (SAM_GPIO_BASE+SAM_GPIO_IFRC_OFFSET(n))
#define SAM_GPIO_IFRC0             (SAM_GPIO_BASE+SAM_GPIO_IFRC0_OFFSET)
#define SAM_GPIO_IFRC1             (SAM_GPIO_BASE+SAM_GPIO_IFRC1_OFFSET)

#define SAM_GPIO_ODCR0             (SAM_GPIO_BASE+SAM_GPIO_ODCR0_OFFSET)
#define SAM_GPIO_ODCR0S            (SAM_GPIO_BASE+SAM_GPIO_ODCR0S_OFFSET)
#define SAM_GPIO_ODCR0C            (SAM_GPIO_BASE+SAM_GPIO_ODCR0C_OFFSET)
#define SAM_GPIO_ODCR0T            (SAM_GPIO_BASE+SAM_GPIO_ODCR0T_OFFSET)

#define SAM_GPIO_ODCR1             (SAM_GPIO_BASE+SAM_GPIO_ODCR1_OFFSET)
#define SAM_GPIO_ODCR1S            (SAM_GPIO_BASE+SAM_GPIO_ODCR1S_OFFSET)
#define SAM_GPIO_ODCR1C            (SAM_GPIO_BASE+SAM_GPIO_ODCR1C_OFFSET)
#define SAM_GPIO_ODCR1T            (SAM_GPIO_BASE+SAM_GPIO_ODCR1T_OFFSET)

#define SAM_GPIO_OSRR0             (SAM_GPIO_BASE+SAM_GPIO_OSRR0_OFFSET)
#define SAM_GPIO_OSRR0S            (SAM_GPIO_BASE+SAM_GPIO_OSRR0S_OFFSET)
#define SAM_GPIO_OSRR0C            (SAM_GPIO_BASE+SAM_GPIO_OSRR0C_OFFSET)
#define SAM_GPIO_OSRR0T            (SAM_GPIO_BASE+SAM_GPIO_OSRR0T_OFFSET)

#define SAM_GPIO_STER              (SAM_GPIO_BASE+SAM_GPIO_STER_OFFSET)
#define SAM_GPIO_STERS             (SAM_GPIO_BASE+SAM_GPIO_STERS_OFFSET)
#define SAM_GPIO_STERC             (SAM_GPIO_BASE+SAM_GPIO_STERC_OFFSET)
#define SAM_GPIO_STERT             (SAM_GPIO_BASE+SAM_GPIO_STERT_OFFSET)

#define SAM_GPIO_EVER              (SAM_GPIO_BASE+SAM_GPIO_EVER_OFFSET)
#define SAM_GPIO_EVERS             (SAM_GPIO_BASE+SAM_GPIO_EVERS_OFFSET)
#define SAM_GPIO_EVERC             (SAM_GPIO_BASE+SAM_GPIO_EVERC_OFFSET)
#define SAM_GPIO_EVERT             (SAM_GPIO_BASE+SAM_GPIO_EVERT_OFFSET)

#define SAM_GPIO_PARAMETER         (SAM_GPIO_BASE+SAM_GPIO_PARAMETER_OFFSET)
#define SAM_GPIO_VERSION           (SAM_GPIO_BASE+SAM_GPIO_VERSION_OFFSET)

/* GPIO register bit definitions ********************************************************/

/* Common bit definitions for all GPIO registers */

#define PIN(n)                     (1 << (n)) /* Bit n: PIO n */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_GPIO_H */
