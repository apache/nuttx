/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam4l_gpio.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_GPIO_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_GPIO_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

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
 *   000 A    100 E
 *   001 B    101 F
 *   010 C    110 G
 *   011 D    111 H
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

#define SAM_GPIO_PVR_OFFSET        0x0060 /* Pin Value Register Read */

/* {PUER, PDER} Selected Function
 *
 *   00 Disabled
 *   01 Pull-down enabled
 *   10 Pull-up enabled
 *   11 Buskeeper enabled
 */

#define SAM_GPIO_PUER_OFFSET       0x0070 /* Pull-up Enable Register Read/Write */
#define SAM_GPIO_PUERS_OFFSET      0x0074 /* Pull-up Enable Register Set */
#define SAM_GPIO_PUERC_OFFSET      0x0078 /* Pull-up Enable Register Clear */
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

#define SAM_GPIO_IFR_OFFSET        0x00d0 /* Interrupt Flag Register 0 Read */
#define SAM_GPIO_IFRC_OFFSET       0x00d8 /* Interrupt Flag Register 0 Clear */

/* {ODCR1, ODCR0} Output drive strength
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

/* GPIO port offsets and addresses ******************************************************/

#define SAM_GPIOA                  0
#define SAM_GPIOB                  1
#define SAM_GPIOC                  2

#define SAM_GPIO_PORTSIZE          0x200
#define SAM_GPION_OFFSET(n)        ((n) << 9)
#define SAM_GPION_BASE(n)          (SAM_GPIO_BASE+SAM_GPION_OFFSET(n))
#define SAM_GPIOA_BASE             SAM_GPION_BASE(SAM_GPIOA)
#define SAM_GPIOB_BASE             SAM_GPION_BASE(SAM_GPIOB)
#define SAM_GPIOC_BASE             SAM_GPION_BASE(SAM_GPIOC)

/* GPIO register addresses **************************************************************/

#define SAM_GPIO_GPER(n)           (SAM_GPION_BASE(n)+SAM_GPIO_GPER_OFFSET)
#define SAM_GPIO_GPERS(n)          (SAM_GPION_BASE(n)+SAM_GPIO_GPERS_OFFSET)
#define SAM_GPIO_GPERC(n)          (SAM_GPION_BASE(n)+SAM_GPIO_GPERC_OFFSET)
#define SAM_GPIO_GPERT(n)          (SAM_GPION_BASE(n)+SAM_GPIO_GPERT_OFFSET)

#define SAM_GPIO_PMR0(n)           (SAM_GPION_BASE(n)+SAM_GPIO_PMR0_OFFSET)
#define SAM_GPIO_PMR0S(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR0S_OFFSET)
#define SAM_GPIO_PMR0C(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR0C_OFFSET)
#define SAM_GPIO_PMR0T(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR0T_OFFSET_

#define SAM_GPIO_PMR1(n)           (SAM_GPION_BASE(n)+SAM_GPIO_PMR1_OFFSET)
#define SAM_GPIO_PMR1S(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR1S_OFFSET)
#define SAM_GPIO_PMR1C(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR1C_OFFSET)
#define SAM_GPIO_PMR1T(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR1T_OFFSET)

#define SAM_GPIO_PMR2(n)           (SAM_GPION_BASE(n)+SAM_GPIO_PMR2_OFFSET)
#define SAM_GPIO_PMR2S(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR2S_OFFSET)
#define SAM_GPIO_PMR2C(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR2C_OFFSET)
#define SAM_GPIO_PMR2T(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PMR2T_OFFSET)

#define SAM_GPIO_ODER(n)           (SAM_GPION_BASE(n)+SAM_GPIO_ODER_OFFSET)
#define SAM_GPIO_ODERS(n)          (SAM_GPION_BASE(n)+SAM_GPIO_ODERS_OFFSET)
#define SAM_GPIO_ODERC(n)          (SAM_GPION_BASE(n)+SAM_GPIO_ODERC_OFFSET)
#define SAM_GPIO_ODERT(n)          (SAM_GPION_BASE(n)+SAM_GPIO_ODERT_OFFSET)

#define SAM_GPIO_OVR(n)            (SAM_GPION_BASE(n)+SAM_GPIO_OVR_OFFSET)
#define SAM_GPIO_OVRS(n)           (SAM_GPION_BASE(n)+SAM_GPIO_OVRS_OFFSET)
#define SAM_GPIO_OVRC(n)           (SAM_GPION_BASE(n)+SAM_GPIO_OVRC_OFFSET)
#define SAM_GPIO_OVRT(n)           (SAM_GPION_BASE(n)+SAM_GPIO_OVRT_OFFSET)

#define SAM_GPIO_PVR(n)            (SAM_GPION_BASE(n)+SAM_GPIO_PVR_OFFSET)

#define SAM_GPIO_PUER(n)           (SAM_GPION_BASE(n)+SAM_GPIO_PUER_OFFSET)
#define SAM_GPIO_PUERS(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PUERS_OFFSET)
#define SAM_GPIO_PUERC(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PUERC_OFFSET)
#define SAM_GPIO_PUERT(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PUERT_OFFSET)

#define SAM_GPIO_PDER(n)           (SAM_GPION_BASE(n)+SAM_GPIO_PDER_OFFSET)
#define SAM_GPIO_PDERS(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PDERS_OFFSET)
#define SAM_GPIO_PDERC(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PDERC_OFFSET)
#define SAM_GPIO_PDERT(n)          (SAM_GPION_BASE(n)+SAM_GPIO_PDERT_OFFSET)

#define SAM_GPIO_IER(n)            (SAM_GPION_BASE(n)+SAM_GPIO_IER_OFFSET)
#define SAM_GPIO_IERS(n)           (SAM_GPION_BASE(n)+SAM_GPIO_IERS_OFFSET)
#define SAM_GPIO_IERC(n)           (SAM_GPION_BASE(n)+SAM_GPIO_IERC_OFFSET)
#define SAM_GPIO_IERT(n)           (SAM_GPION_BASE(n)+SAM_GPIO_IERT_OFFSET)

#define SAM_GPIO_IMR0(n)           (SAM_GPION_BASE(n)+SAM_GPIO_IMR0_OFFSET)
#define SAM_GPIO_IMR0S(n)          (SAM_GPION_BASE(n)+SAM_GPIO_IMR0S_OFFSET)
#define SAM_GPIO_IMR0C(n)          (SAM_GPION_BASE(n)+SAM_GPIO_IMR0C_OFFSET)
#define SAM_GPIO_IMR0T(n)          (SAM_GPION_BASE(n)+SAM_GPIO_IMR0T_OFFSET)

#define SAM_GPIO_IMR1(n)           (SAM_GPION_BASE(n)+SAM_GPIO_IMR1_OFFSET)
#define SAM_GPIO_IMR1S(n)          (SAM_GPION_BASE(n)+SAM_GPIO_IMR1S_OFFSET)
#define SAM_GPIO_IMR1C(n)          (SAM_GPION_BASE(n)+SAM_GPIO_IMR1C_OFFSET)
#define SAM_GPIO_IMR1T(n)          (SAM_GPION_BASE(n)+SAM_GPIO_IMR1T_OFFSET)

#define SAM_GPIO_GFER(n)           (SAM_GPION_BASE(n)+SAM_GPIO_GFER_OFFSET)
#define SAM_GPIO_GFERS(n)          (SAM_GPION_BASE(n)+SAM_GPIO_GFERS_OFFSET)
#define SAM_GPIO_GFERC(n)          (SAM_GPION_BASE(n)+SAM_GPIO_GFERC_OFFSET)
#define SAM_GPIO_GFERT(n)          (SAM_GPION_BASE(n)+SAM_GPIO_GFERT_OFFSET)

#define SAM_GPIO_IFR(n)            (SAM_GPION_BASE(n)+SAM_GPIO_IFR_OFFSET)
#define SAM_GPIO_IFRC(n)           (SAM_GPION_BASE(n)+SAM_GPIO_IFRC_OFFSET)

#define SAM_GPIO_ODCR0(n)          (SAM_GPION_BASE(n)+SAM_GPIO_ODCR0_OFFSET)
#define SAM_GPIO_ODCR0S(n)         (SAM_GPION_BASE(n)+SAM_GPIO_ODCR0S_OFFSET)
#define SAM_GPIO_ODCR0C(n)         (SAM_GPION_BASE(n)+SAM_GPIO_ODCR0C_OFFSET)
#define SAM_GPIO_ODCR0T(n)         (SAM_GPION_BASE(n)+SAM_GPIO_ODCR0T_OFFSET)

#define SAM_GPIO_ODCR1(n)          (SAM_GPION_BASE(n)+SAM_GPIO_ODCR1_OFFSET)
#define SAM_GPIO_ODCR1S(n)         (SAM_GPION_BASE(n)+SAM_GPIO_ODCR1S_OFFSET)
#define SAM_GPIO_ODCR1C(n)         (SAM_GPION_BASE(n)+SAM_GPIO_ODCR1C_OFFSET)
#define SAM_GPIO_ODCR1T(n)         (SAM_GPION_BASE(n)+SAM_GPIO_ODCR1T_OFFSET)

#define SAM_GPIO_OSRR0(n)          (SAM_GPION_BASE(n)+SAM_GPIO_OSRR0_OFFSET)
#define SAM_GPIO_OSRR0S(n)         (SAM_GPION_BASE(n)+SAM_GPIO_OSRR0S_OFFSET)
#define SAM_GPIO_OSRR0C(n)         (SAM_GPION_BASE(n)+SAM_GPIO_OSRR0C_OFFSET)
#define SAM_GPIO_OSRR0T(n)         (SAM_GPION_BASE(n)+SAM_GPIO_OSRR0T_OFFSET)

#define SAM_GPIO_STER(n)           (SAM_GPION_BASE(n)+SAM_GPIO_STER_OFFSET)
#define SAM_GPIO_STERS(n)          (SAM_GPION_BASE(n)+SAM_GPIO_STERS_OFFSET)
#define SAM_GPIO_STERC(n)          (SAM_GPION_BASE(n)+SAM_GPIO_STERC_OFFSET)
#define SAM_GPIO_STERT(n)          (SAM_GPION_BASE(n)+SAM_GPIO_STERT_OFFSET)

#define SAM_GPIO_EVER(n)           (SAM_GPION_BASE(n)+SAM_GPIO_EVER_OFFSET)
#define SAM_GPIO_EVERS(n)          (SAM_GPION_BASE(n)+SAM_GPIO_EVERS_OFFSET)
#define SAM_GPIO_EVERC(n)          (SAM_GPION_BASE(n)+SAM_GPIO_EVERC_OFFSET)
#define SAM_GPIO_EVERT(n)          (SAM_GPION_BASE(n)+SAM_GPIO_EVERT_OFFSET)

#define SAM_GPIO_PARAMETER(n)      (SAM_GPION_BASE(n)+SAM_GPIO_PARAMETER_OFFSET)
#define SAM_GPIO_VERSION (n)       (SAM_GPION_BASE(n)+SAM_GPIO_VERSION_OFFSET)

/* GPIO PORTA register addresses ********************************************************/

#define SAM_GPIOA_GPER             (SAM_GPIOA_BASE+SAM_GPIO_GPER_OFFSET)
#define SAM_GPIOA_GPERS            (SAM_GPIOA_BASE+SAM_GPIO_GPERS_OFFSET)
#define SAM_GPIOA_GPERC            (SAM_GPIOA_BASE+SAM_GPIO_GPERC_OFFSET)
#define SAM_GPIOA_GPERT            (SAM_GPIOA_BASE+SAM_GPIO_GPERT_OFFSET)

#define SAM_GPIOA_PMR0             (SAM_GPIOA_BASE+SAM_GPIO_PMR0_OFFSET)
#define SAM_GPIOA_PMR0S            (SAM_GPIOA_BASE+SAM_GPIO_PMR0S_OFFSET)
#define SAM_GPIOA_PMR0C            (SAM_GPIOA_BASE+SAM_GPIO_PMR0C_OFFSET)
#define SAM_GPIOA_PMR0T            (SAM_GPIOA_BASE+SAM_GPIO_PMR0T_OFFSET_

#define SAM_GPIOA_PMR1             (SAM_GPIOA_BASE+SAM_GPIO_PMR1_OFFSET)
#define SAM_GPIOA_PMR1S            (SAM_GPIOA_BASE+SAM_GPIO_PMR1S_OFFSET)
#define SAM_GPIOA_PMR1C            (SAM_GPIOA_BASE+SAM_GPIO_PMR1C_OFFSET)
#define SAM_GPIOA_PMR1T            (SAM_GPIOA_BASE+SAM_GPIO_PMR1T_OFFSET)

#define SAM_GPIOA_PMR2             (SAM_GPIOA_BASE+SAM_GPIO_PMR2_OFFSET)
#define SAM_GPIOA_PMR2S            (SAM_GPIOA_BASE+SAM_GPIO_PMR2S_OFFSET)
#define SAM_GPIOA_PMR2C            (SAM_GPIOA_BASE+SAM_GPIO_PMR2C_OFFSET)
#define SAM_GPIOA_PMR2T            (SAM_GPIOA_BASE+SAM_GPIO_PMR2T_OFFSET)

#define SAM_GPIOA_ODER             (SAM_GPIOA_BASE+SAM_GPIO_ODER_OFFSET)
#define SAM_GPIOA_ODERS            (SAM_GPIOA_BASE+SAM_GPIO_ODERS_OFFSET)
#define SAM_GPIOA_ODERC            (SAM_GPIOA_BASE+SAM_GPIO_ODERC_OFFSET)
#define SAM_GPIOA_ODERT            (SAM_GPIOA_BASE+SAM_GPIO_ODERT_OFFSET)

#define SAM_GPIOA_OVR              (SAM_GPIOA_BASE+SAM_GPIO_OVR_OFFSET)
#define SAM_GPIOA_OVRS             (SAM_GPIOA_BASE+SAM_GPIO_OVRS_OFFSET)
#define SAM_GPIOA_OVRC             (SAM_GPIOA_BASE+SAM_GPIO_OVRC_OFFSET)
#define SAM_GPIOA_OVRT             (SAM_GPIOA_BASE+SAM_GPIO_OVRT_OFFSET)

#define SAM_GPIOA_PVR              (SAM_GPIOA_BASE+SAM_GPIO_PVR_OFFSET)

#define SAM_GPIOA_PUER             (SAM_GPIOA_BASE+SAM_GPIO_PUER_OFFSET)
#define SAM_GPIOA_PUERS            (SAM_GPIOA_BASE+SAM_GPIO_PUERS_OFFSET)
#define SAM_GPIOA_PUERC            (SAM_GPIOA_BASE+SAM_GPIO_PUERC_OFFSET)
#define SAM_GPIOA_PUERT            (SAM_GPIOA_BASE+SAM_GPIO_PUERT_OFFSET)

#define SAM_GPIOA_PDER             (SAM_GPIOA_BASE+SAM_GPIO_PDER_OFFSET)
#define SAM_GPIOA_PDERS            (SAM_GPIOA_BASE+SAM_GPIO_PDERS_OFFSET)
#define SAM_GPIOA_PDERC            (SAM_GPIOA_BASE+SAM_GPIO_PDERC_OFFSET)
#define SAM_GPIOA_PDERT            (SAM_GPIOA_BASE+SAM_GPIO_PDERT_OFFSET)

#define SAM_GPIOA_IER              (SAM_GPIOA_BASE+SAM_GPIO_IER_OFFSET)
#define SAM_GPIOA_IERS             (SAM_GPIOA_BASE+SAM_GPIO_IERS_OFFSET)
#define SAM_GPIOA_IERC             (SAM_GPIOA_BASE+SAM_GPIO_IERC_OFFSET)
#define SAM_GPIOA_IERT             (SAM_GPIOA_BASE+SAM_GPIO_IERT_OFFSET)

#define SAM_GPIOA_IMR0             (SAM_GPIOA_BASE+SAM_GPIO_IMR0_OFFSET)
#define SAM_GPIOA_IMR0S            (SAM_GPIOA_BASE+SAM_GPIO_IMR0S_OFFSET)
#define SAM_GPIOA_IMR0C            (SAM_GPIOA_BASE+SAM_GPIO_IMR0C_OFFSET)
#define SAM_GPIOA_IMR0T            (SAM_GPIOA_BASE+SAM_GPIO_IMR0T_OFFSET)

#define SAM_GPIOA_IMR1             (SAM_GPIOA_BASE+SAM_GPIO_IMR1_OFFSET)
#define SAM_GPIOA_IMR1S            (SAM_GPIOA_BASE+SAM_GPIO_IMR1S_OFFSET)
#define SAM_GPIOA_IMR1C            (SAM_GPIOA_BASE+SAM_GPIO_IMR1C_OFFSET)
#define SAM_GPIOA_IMR1T            (SAM_GPIOA_BASE+SAM_GPIO_IMR1T_OFFSET)

#define SAM_GPIOA_GFER             (SAM_GPIOA_BASE+SAM_GPIO_GFER_OFFSET)
#define SAM_GPIOA_GFERS            (SAM_GPIOA_BASE+SAM_GPIO_GFERS_OFFSET)
#define SAM_GPIOA_GFERC            (SAM_GPIOA_BASE+SAM_GPIO_GFERC_OFFSET)
#define SAM_GPIOA_GFERT            (SAM_GPIOA_BASE+SAM_GPIO_GFERT_OFFSET)

#define SAM_GPIOA_IFR              (SAM_GPIOA_BASE+SAM_GPIO_IFR_OFFSET)
#define SAM_GPIOA_IFRC             (SAM_GPIOA_BASE+SAM_GPIO_IFRC_OFFSET)

#define SAM_GPIOA_ODCR0            (SAM_GPIOA_BASE+SAM_GPIO_ODCR0_OFFSET)
#define SAM_GPIOA_ODCR0S           (SAM_GPIOA_BASE+SAM_GPIO_ODCR0S_OFFSET)
#define SAM_GPIOA_ODCR0C           (SAM_GPIOA_BASE+SAM_GPIO_ODCR0C_OFFSET)
#define SAM_GPIOA_ODCR0T           (SAM_GPIOA_BASE+SAM_GPIO_ODCR0T_OFFSET)

#define SAM_GPIOA_ODCR1            (SAM_GPIOA_BASE+SAM_GPIO_ODCR1_OFFSET)
#define SAM_GPIOA_ODCR1S           (SAM_GPIOA_BASE+SAM_GPIO_ODCR1S_OFFSET)
#define SAM_GPIOA_ODCR1C           (SAM_GPIOA_BASE+SAM_GPIO_ODCR1C_OFFSET)
#define SAM_GPIOA_ODCR1T           (SAM_GPIOA_BASE+SAM_GPIO_ODCR1T_OFFSET)

#define SAM_GPIOA_OSRR0            (SAM_GPIOA_BASE+SAM_GPIO_OSRR0_OFFSET)
#define SAM_GPIOA_OSRR0S           (SAM_GPIOA_BASE+SAM_GPIO_OSRR0S_OFFSET)
#define SAM_GPIOA_OSRR0C           (SAM_GPIOA_BASE+SAM_GPIO_OSRR0C_OFFSET)
#define SAM_GPIOA_OSRR0T           (SAM_GPIOA_BASE+SAM_GPIO_OSRR0T_OFFSET)

#define SAM_GPIOA_STER             (SAM_GPIOA_BASE+SAM_GPIO_STER_OFFSET)
#define SAM_GPIOA_STERS            (SAM_GPIOA_BASE+SAM_GPIO_STERS_OFFSET)
#define SAM_GPIOA_STERC            (SAM_GPIOA_BASE+SAM_GPIO_STERC_OFFSET)
#define SAM_GPIOA_STERT            (SAM_GPIOA_BASE+SAM_GPIO_STERT_OFFSET)

#define SAM_GPIOA_EVER             (SAM_GPIOA_BASE+SAM_GPIO_EVER_OFFSET)
#define SAM_GPIOA_EVERS            (SAM_GPIOA_BASE+SAM_GPIO_EVERS_OFFSET)
#define SAM_GPIOA_EVERC            (SAM_GPIOA_BASE+SAM_GPIO_EVERC_OFFSET)
#define SAM_GPIOA_EVERT            (SAM_GPIOA_BASE+SAM_GPIO_EVERT_OFFSET)

#define SAM_GPIOA_PARAMETER        (SAM_GPIOA_BASE+SAM_GPIO_PARAMETER_OFFSET)
#define SAM_GPIOA_VERSION          (SAM_GPIOA_BASE+SAM_GPIO_VERSION_OFFSET)

/* GPIO PORTB register addresses ********************************************************/

#define SAM_GPIOB_GPER             (SAM_GPIOB_BASE+SAM_GPIO_GPER_OFFSET)
#define SAM_GPIOB_GPERS            (SAM_GPIOB_BASE+SAM_GPIO_GPERS_OFFSET)
#define SAM_GPIOB_GPERC            (SAM_GPIOB_BASE+SAM_GPIO_GPERC_OFFSET)
#define SAM_GPIOB_GPERT            (SAM_GPIOB_BASE+SAM_GPIO_GPERT_OFFSET)

#define SAM_GPIOB_PMR0             (SAM_GPIOB_BASE+SAM_GPIO_PMR0_OFFSET)
#define SAM_GPIOB_PMR0S            (SAM_GPIOB_BASE+SAM_GPIO_PMR0S_OFFSET)
#define SAM_GPIOB_PMR0C            (SAM_GPIOB_BASE+SAM_GPIO_PMR0C_OFFSET)
#define SAM_GPIOB_PMR0T            (SAM_GPIOB_BASE+SAM_GPIO_PMR0T_OFFSET_

#define SAM_GPIOB_PMR1             (SAM_GPIOB_BASE+SAM_GPIO_PMR1_OFFSET)
#define SAM_GPIOB_PMR1S            (SAM_GPIOB_BASE+SAM_GPIO_PMR1S_OFFSET)
#define SAM_GPIOB_PMR1C            (SAM_GPIOB_BASE+SAM_GPIO_PMR1C_OFFSET)
#define SAM_GPIOB_PMR1T            (SAM_GPIOB_BASE+SAM_GPIO_PMR1T_OFFSET)

#define SAM_GPIOB_PMR2             (SAM_GPIOB_BASE+SAM_GPIO_PMR2_OFFSET)
#define SAM_GPIOB_PMR2S            (SAM_GPIOB_BASE+SAM_GPIO_PMR2S_OFFSET)
#define SAM_GPIOB_PMR2C            (SAM_GPIOB_BASE+SAM_GPIO_PMR2C_OFFSET)
#define SAM_GPIOB_PMR2T            (SAM_GPIOB_BASE+SAM_GPIO_PMR2T_OFFSET)

#define SAM_GPIOB_ODER             (SAM_GPIOB_BASE+SAM_GPIO_ODER_OFFSET)
#define SAM_GPIOB_ODERS            (SAM_GPIOB_BASE+SAM_GPIO_ODERS_OFFSET)
#define SAM_GPIOB_ODERC            (SAM_GPIOB_BASE+SAM_GPIO_ODERC_OFFSET)
#define SAM_GPIOB_ODERT            (SAM_GPIOB_BASE+SAM_GPIO_ODERT_OFFSET)

#define SAM_GPIOB_OVR              (SAM_GPIOB_BASE+SAM_GPIO_OVR_OFFSET)
#define SAM_GPIOB_OVRS             (SAM_GPIOB_BASE+SAM_GPIO_OVRS_OFFSET)
#define SAM_GPIOB_OVRC             (SAM_GPIOB_BASE+SAM_GPIO_OVRC_OFFSET)
#define SAM_GPIOB_OVRT             (SAM_GPIOB_BASE+SAM_GPIO_OVRT_OFFSET)

#define SAM_GPIOB_PVR              (SAM_GPIOB_BASE+SAM_GPIO_PVR_OFFSET)

#define SAM_GPIOB_PUER             (SAM_GPIOB_BASE+SAM_GPIO_PUER_OFFSET)
#define SAM_GPIOB_PUERS            (SAM_GPIOB_BASE+SAM_GPIO_PUERS_OFFSET)
#define SAM_GPIOB_PUERC            (SAM_GPIOB_BASE+SAM_GPIO_PUERC_OFFSET)
#define SAM_GPIOB_PUERT            (SAM_GPIOB_BASE+SAM_GPIO_PUERT_OFFSET)

#define SAM_GPIOB_PDER             (SAM_GPIOB_BASE+SAM_GPIO_PDER_OFFSET)
#define SAM_GPIOB_PDERS            (SAM_GPIOB_BASE+SAM_GPIO_PDERS_OFFSET)
#define SAM_GPIOB_PDERC            (SAM_GPIOB_BASE+SAM_GPIO_PDERC_OFFSET)
#define SAM_GPIOB_PDERT            (SAM_GPIOB_BASE+SAM_GPIO_PDERT_OFFSET)

#define SAM_GPIOB_IER              (SAM_GPIOB_BASE+SAM_GPIO_IER_OFFSET)
#define SAM_GPIOB_IERS             (SAM_GPIOB_BASE+SAM_GPIO_IERS_OFFSET)
#define SAM_GPIOB_IERC             (SAM_GPIOB_BASE+SAM_GPIO_IERC_OFFSET)
#define SAM_GPIOB_IERT             (SAM_GPIOB_BASE+SAM_GPIO_IERT_OFFSET)

#define SAM_GPIOB_IMR0             (SAM_GPIOB_BASE+SAM_GPIO_IMR0_OFFSET)
#define SAM_GPIOB_IMR0S            (SAM_GPIOB_BASE+SAM_GPIO_IMR0S_OFFSET)
#define SAM_GPIOB_IMR0C            (SAM_GPIOB_BASE+SAM_GPIO_IMR0C_OFFSET)
#define SAM_GPIOB_IMR0T            (SAM_GPIOB_BASE+SAM_GPIO_IMR0T_OFFSET)

#define SAM_GPIOB_IMR1             (SAM_GPIOB_BASE+SAM_GPIO_IMR1_OFFSET)
#define SAM_GPIOB_IMR1S            (SAM_GPIOB_BASE+SAM_GPIO_IMR1S_OFFSET)
#define SAM_GPIOB_IMR1C            (SAM_GPIOB_BASE+SAM_GPIO_IMR1C_OFFSET)
#define SAM_GPIOB_IMR1T            (SAM_GPIOB_BASE+SAM_GPIO_IMR1T_OFFSET)

#define SAM_GPIOB_GFER             (SAM_GPIOB_BASE+SAM_GPIO_GFER_OFFSET)
#define SAM_GPIOB_GFERS            (SAM_GPIOB_BASE+SAM_GPIO_GFERS_OFFSET)
#define SAM_GPIOB_GFERC            (SAM_GPIOB_BASE+SAM_GPIO_GFERC_OFFSET)
#define SAM_GPIOB_GFERT            (SAM_GPIOB_BASE+SAM_GPIO_GFERT_OFFSET)

#define SAM_GPIOB_IFR              (SAM_GPIOB_BASE+SAM_GPIO_IFR_OFFSET)
#define SAM_GPIOB_IFRC             (SAM_GPIOB_BASE+SAM_GPIO_IFRC_OFFSET)

#define SAM_GPIOB_ODCR0            (SAM_GPIOB_BASE+SAM_GPIO_ODCR0_OFFSET)
#define SAM_GPIOB_ODCR0S           (SAM_GPIOB_BASE+SAM_GPIO_ODCR0S_OFFSET)
#define SAM_GPIOB_ODCR0C           (SAM_GPIOB_BASE+SAM_GPIO_ODCR0C_OFFSET)
#define SAM_GPIOB_ODCR0T           (SAM_GPIOB_BASE+SAM_GPIO_ODCR0T_OFFSET)

#define SAM_GPIOB_ODCR1            (SAM_GPIOB_BASE+SAM_GPIO_ODCR1_OFFSET)
#define SAM_GPIOB_ODCR1S           (SAM_GPIOB_BASE+SAM_GPIO_ODCR1S_OFFSET)
#define SAM_GPIOB_ODCR1C           (SAM_GPIOB_BASE+SAM_GPIO_ODCR1C_OFFSET)
#define SAM_GPIOB_ODCR1T           (SAM_GPIOB_BASE+SAM_GPIO_ODCR1T_OFFSET)

#define SAM_GPIOB_OSRR0            (SAM_GPIOB_BASE+SAM_GPIO_OSRR0_OFFSET)
#define SAM_GPIOB_OSRR0S           (SAM_GPIOB_BASE+SAM_GPIO_OSRR0S_OFFSET)
#define SAM_GPIOB_OSRR0C           (SAM_GPIOB_BASE+SAM_GPIO_OSRR0C_OFFSET)
#define SAM_GPIOB_OSRR0T           (SAM_GPIOB_BASE+SAM_GPIO_OSRR0T_OFFSET)

#define SAM_GPIOB_STER             (SAM_GPIOB_BASE+SAM_GPIO_STER_OFFSET)
#define SAM_GPIOB_STERS            (SAM_GPIOB_BASE+SAM_GPIO_STERS_OFFSET)
#define SAM_GPIOB_STERC            (SAM_GPIOB_BASE+SAM_GPIO_STERC_OFFSET)
#define SAM_GPIOB_STERT            (SAM_GPIOB_BASE+SAM_GPIO_STERT_OFFSET)

#define SAM_GPIOB_EVER             (SAM_GPIOB_BASE+SAM_GPIO_EVER_OFFSET)
#define SAM_GPIOB_EVERS            (SAM_GPIOB_BASE+SAM_GPIO_EVERS_OFFSET)
#define SAM_GPIOB_EVERC            (SAM_GPIOB_BASE+SAM_GPIO_EVERC_OFFSET)
#define SAM_GPIOB_EVERT            (SAM_GPIOB_BASE+SAM_GPIO_EVERT_OFFSET)

#define SAM_GPIOB_PARAMETER        (SAM_GPIOB_BASE+SAM_GPIO_PARAMETER_OFFSET)
#define SAM_GPIOB_VERSION          (SAM_GPIOB_BASE+SAM_GPIO_VERSION_OFFSET)

/* GPIO PORTC register addresses ********************************************************/

#define SAM_GPIOC_GPER             (SAM_GPIOC_BASE+SAM_GPIO_GPER_OFFSET)
#define SAM_GPIOC_GPERS            (SAM_GPIOC_BASE+SAM_GPIO_GPERS_OFFSET)
#define SAM_GPIOC_GPERC            (SAM_GPIOC_BASE+SAM_GPIO_GPERC_OFFSET)
#define SAM_GPIOC_GPERT            (SAM_GPIOC_BASE+SAM_GPIO_GPERT_OFFSET)

#define SAM_GPIOC_PMR0             (SAM_GPIOC_BASE+SAM_GPIO_PMR0_OFFSET)
#define SAM_GPIOC_PMR0S            (SAM_GPIOC_BASE+SAM_GPIO_PMR0S_OFFSET)
#define SAM_GPIOC_PMR0C            (SAM_GPIOC_BASE+SAM_GPIO_PMR0C_OFFSET)
#define SAM_GPIOC_PMR0T            (SAM_GPIOC_BASE+SAM_GPIO_PMR0T_OFFSET_

#define SAM_GPIOC_PMR1             (SAM_GPIOC_BASE+SAM_GPIO_PMR1_OFFSET)
#define SAM_GPIOC_PMR1S            (SAM_GPIOC_BASE+SAM_GPIO_PMR1S_OFFSET)
#define SAM_GPIOC_PMR1C            (SAM_GPIOC_BASE+SAM_GPIO_PMR1C_OFFSET)
#define SAM_GPIOC_PMR1T            (SAM_GPIOC_BASE+SAM_GPIO_PMR1T_OFFSET)

#define SAM_GPIOC_PMR2             (SAM_GPIOC_BASE+SAM_GPIO_PMR2_OFFSET)
#define SAM_GPIOC_PMR2S            (SAM_GPIOC_BASE+SAM_GPIO_PMR2S_OFFSET)
#define SAM_GPIOC_PMR2C            (SAM_GPIOC_BASE+SAM_GPIO_PMR2C_OFFSET)
#define SAM_GPIOC_PMR2T            (SAM_GPIOC_BASE+SAM_GPIO_PMR2T_OFFSET)

#define SAM_GPIOC_ODER             (SAM_GPIOC_BASE+SAM_GPIO_ODER_OFFSET)
#define SAM_GPIOC_ODERS            (SAM_GPIOC_BASE+SAM_GPIO_ODERS_OFFSET)
#define SAM_GPIOC_ODERC            (SAM_GPIOC_BASE+SAM_GPIO_ODERC_OFFSET)
#define SAM_GPIOC_ODERT            (SAM_GPIOC_BASE+SAM_GPIO_ODERT_OFFSET)

#define SAM_GPIOC_OVR              (SAM_GPIOC_BASE+SAM_GPIO_OVR_OFFSET)
#define SAM_GPIOC_OVRS             (SAM_GPIOC_BASE+SAM_GPIO_OVRS_OFFSET)
#define SAM_GPIOC_OVRC             (SAM_GPIOC_BASE+SAM_GPIO_OVRC_OFFSET)
#define SAM_GPIOC_OVRT             (SAM_GPIOC_BASE+SAM_GPIO_OVRT_OFFSET)

#define SAM_GPIOC_PVR              (SAM_GPIOC_BASE+SAM_GPIO_PVR_OFFSET)

#define SAM_GPIOC_PUER             (SAM_GPIOC_BASE+SAM_GPIO_PUER_OFFSET)
#define SAM_GPIOC_PUERS            (SAM_GPIOC_BASE+SAM_GPIO_PUERS_OFFSET)
#define SAM_GPIOC_PUERC            (SAM_GPIOC_BASE+SAM_GPIO_PUERC_OFFSET)
#define SAM_GPIOC_PUERT            (SAM_GPIOC_BASE+SAM_GPIO_PUERT_OFFSET)

#define SAM_GPIOC_PDER             (SAM_GPIOC_BASE+SAM_GPIO_PDER_OFFSET)
#define SAM_GPIOC_PDERS            (SAM_GPIOC_BASE+SAM_GPIO_PDERS_OFFSET)
#define SAM_GPIOC_PDERC            (SAM_GPIOC_BASE+SAM_GPIO_PDERC_OFFSET)
#define SAM_GPIOC_PDERT            (SAM_GPIOC_BASE+SAM_GPIO_PDERT_OFFSET)

#define SAM_GPIOC_IER              (SAM_GPIOC_BASE+SAM_GPIO_IER_OFFSET)
#define SAM_GPIOC_IERS             (SAM_GPIOC_BASE+SAM_GPIO_IERS_OFFSET)
#define SAM_GPIOC_IERC             (SAM_GPIOC_BASE+SAM_GPIO_IERC_OFFSET)
#define SAM_GPIOC_IERT             (SAM_GPIOC_BASE+SAM_GPIO_IERT_OFFSET)

#define SAM_GPIOC_IMR0             (SAM_GPIOC_BASE+SAM_GPIO_IMR0_OFFSET)
#define SAM_GPIOC_IMR0S            (SAM_GPIOC_BASE+SAM_GPIO_IMR0S_OFFSET)
#define SAM_GPIOC_IMR0C            (SAM_GPIOC_BASE+SAM_GPIO_IMR0C_OFFSET)
#define SAM_GPIOC_IMR0T            (SAM_GPIOC_BASE+SAM_GPIO_IMR0T_OFFSET)

#define SAM_GPIOC_IMR1             (SAM_GPIOC_BASE+SAM_GPIO_IMR1_OFFSET)
#define SAM_GPIOC_IMR1S            (SAM_GPIOC_BASE+SAM_GPIO_IMR1S_OFFSET)
#define SAM_GPIOC_IMR1C            (SAM_GPIOC_BASE+SAM_GPIO_IMR1C_OFFSET)
#define SAM_GPIOC_IMR1T            (SAM_GPIOC_BASE+SAM_GPIO_IMR1T_OFFSET)

#define SAM_GPIOC_GFER             (SAM_GPIOC_BASE+SAM_GPIO_GFER_OFFSET)
#define SAM_GPIOC_GFERS            (SAM_GPIOC_BASE+SAM_GPIO_GFERS_OFFSET)
#define SAM_GPIOC_GFERC            (SAM_GPIOC_BASE+SAM_GPIO_GFERC_OFFSET)
#define SAM_GPIOC_GFERT            (SAM_GPIOC_BASE+SAM_GPIO_GFERT_OFFSET)

#define SAM_GPIOC_IFR              (SAM_GPIOC_BASE+SAM_GPIO_IFR_OFFSET)
#define SAM_GPIOC_IFRC             (SAM_GPIOC_BASE+SAM_GPIO_IFRC_OFFSET)

#define SAM_GPIOC_ODCR0            (SAM_GPIOC_BASE+SAM_GPIO_ODCR0_OFFSET)
#define SAM_GPIOC_ODCR0S           (SAM_GPIOC_BASE+SAM_GPIO_ODCR0S_OFFSET)
#define SAM_GPIOC_ODCR0C           (SAM_GPIOC_BASE+SAM_GPIO_ODCR0C_OFFSET)
#define SAM_GPIOC_ODCR0T           (SAM_GPIOC_BASE+SAM_GPIO_ODCR0T_OFFSET)

#define SAM_GPIOC_ODCR1            (SAM_GPIOC_BASE+SAM_GPIO_ODCR1_OFFSET)
#define SAM_GPIOC_ODCR1S           (SAM_GPIOC_BASE+SAM_GPIO_ODCR1S_OFFSET)
#define SAM_GPIOC_ODCR1C           (SAM_GPIOC_BASE+SAM_GPIO_ODCR1C_OFFSET)
#define SAM_GPIOC_ODCR1T           (SAM_GPIOC_BASE+SAM_GPIO_ODCR1T_OFFSET)

#define SAM_GPIOC_OSRR0            (SAM_GPIOC_BASE+SAM_GPIO_OSRR0_OFFSET)
#define SAM_GPIOC_OSRR0S           (SAM_GPIOC_BASE+SAM_GPIO_OSRR0S_OFFSET)
#define SAM_GPIOC_OSRR0C           (SAM_GPIOC_BASE+SAM_GPIO_OSRR0C_OFFSET)
#define SAM_GPIOC_OSRR0T           (SAM_GPIOC_BASE+SAM_GPIO_OSRR0T_OFFSET)

#define SAM_GPIOC_STER             (SAM_GPIOC_BASE+SAM_GPIO_STER_OFFSET)
#define SAM_GPIOC_STERS            (SAM_GPIOC_BASE+SAM_GPIO_STERS_OFFSET)
#define SAM_GPIOC_STERC            (SAM_GPIOC_BASE+SAM_GPIO_STERC_OFFSET)
#define SAM_GPIOC_STERT            (SAM_GPIOC_BASE+SAM_GPIO_STERT_OFFSET)

#define SAM_GPIOC_EVER             (SAM_GPIOC_BASE+SAM_GPIO_EVER_OFFSET)
#define SAM_GPIOC_EVERS            (SAM_GPIOC_BASE+SAM_GPIO_EVERS_OFFSET)
#define SAM_GPIOC_EVERC            (SAM_GPIOC_BASE+SAM_GPIO_EVERC_OFFSET)
#define SAM_GPIOC_EVERT            (SAM_GPIOC_BASE+SAM_GPIO_EVERT_OFFSET)

#define SAM_GPIOC_PARAMETER        (SAM_GPIOC_BASE+SAM_GPIO_PARAMETER_OFFSET)
#define SAM_GPIOC_VERSION          (SAM_GPIOC_BASE+SAM_GPIO_VERSION_OFFSET)

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

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_GPIO_H */
