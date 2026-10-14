/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57l843_memorymap.h
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

/* Base addresses below are taken directly from TI's HALCoGen-generated
 * register headers for RM57L843, not assumed from TMS570
 * parity - several peripheral base addresses differ between the two
 * Hercules generations (VIM, PCR, and pinmux in particular).
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_MEMORYMAP_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map Overview */

/* 0x00000000-0x003fffff: Program FLASH (4MB) */
#define RM57_FLASH_BASE 0x00000000

/* 0x08000000-0x0807ffff: RAM (512KB) */
#define RM57_RAM_BASE 0x08000000

/* Peripheral Control Registers */

#define RM57_GIO_BASE             0xfff7bc00 /* GIO */
#define RM57_GIO_PORTA_BASE       0xfff7bc34 /* GIO Port A */
#define RM57_GIO_PORTB_BASE       0xfff7bc54 /* GIO Port B */
#define RM57_ADC1_BASE            0xfff7c000 /* MibADC1 */
#define RM57_ADC2_BASE            0xfff7c200 /* MibADC2 */
#define RM57_I2C1_BASE            0xfff7d400 /* I2C1 */
#define RM57_I2C2_BASE            0xfff7d500 /* I2C2 */
#define RM57_DCAN1_BASE           0xfff7dc00 /* DCAN1 */
#define RM57_DCAN2_BASE           0xfff7de00 /* DCAN2 */
#define RM57_DCAN3_BASE           0xfff7e000 /* DCAN3 */
#define RM57_DCAN4_BASE           0xfff7e200 /* DCAN4 */
#define RM57_SCI1_BASE            0xfff7e400 /* SCI1/LIN1 */
#define RM57_SCI1_PORT_BASE       0xfff7e440 /* SCI1 GIO port */
#define RM57_SCI3_BASE            0xfff7e500 /* SCI3 */
#define RM57_SCI3_PORT_BASE       0xfff7e540 /* SCI3 GIO port */
#define RM57_SCI2_BASE            0xfff7e600 /* SCI2 */
#define RM57_SCI2_PORT_BASE       0xfff7e640 /* SCI2 GIO port */
#define RM57_SCI4_BASE            0xfff7e700 /* SCI4 */
#define RM57_SCI4_PORT_BASE       0xfff7e740 /* SCI4 GIO port */
#define RM57_MIBSPI1_BASE         0xfff7f400 /* MibSPI1 */
#define RM57_MIBSPI2_BASE         0xfff7f600 /* MibSPI2 */
#define RM57_MIBSPI3_BASE         0xfff7f800 /* MibSPI3 */
#define RM57_MIBSPI4_BASE         0xfff7fa00 /* MibSPI4 */
#define RM57_MIBSPI5_BASE         0xfff7fc00 /* MibSPI5 */

/* MibSPI multi-buffer RAM: the transmit bank followed by the receive bank,
 * accessible only once the module is in multi-buffer mode.
 */

#define RM57_MIBSPI4_RAM_BASE     0xff060000 /* MibSPI4 buffer RAM */
#define RM57_MIBSPI2_RAM_BASE     0xff080000 /* MibSPI2 buffer RAM */
#define RM57_MIBSPI5_RAM_BASE     0xff0a0000 /* MibSPI5 buffer RAM */
#define RM57_MIBSPI3_RAM_BASE     0xff0c0000 /* MibSPI3 buffer RAM */
#define RM57_MIBSPI1_RAM_BASE     0xff0e0000 /* MibSPI1 buffer RAM */

/* DCAN message RAM.  Memory mapped only in Debug/Suspend or RAM Direct
 * Access mode; normal accesses go through the DCAN IFx register sets.
 */

#define RM57_DCAN4_RAM_BASE       0xff180000 /* DCAN4 mailbox RAM */
#define RM57_DCAN3_RAM_BASE       0xff1a0000 /* DCAN3 mailbox RAM */
#define RM57_DCAN2_RAM_BASE       0xff1c0000 /* DCAN2 mailbox RAM */
#define RM57_DCAN1_RAM_BASE       0xff1e0000 /* DCAN1 mailbox RAM */

/* MibADC conversion results RAM.  64 words per module, partitioned between
 * the Event/Group1/Group2 conversion groups by ADBNDCR.  The parity bits are
 * mapped 4KB above the RAM base and are only writable while ADPARCR.TEST is
 * set.  The look-up tables used by the enhanced channel selection mode sit
 * 8KB above the RAM base; only ADC1 implements that mode (TRM SPNU562A
 * sections 22.2.2.2.2 and 22.2.7).
 */

#define RM57_ADC2_RAM_BASE        0xff3a0000 /* ADC2 results RAM */
#define RM57_ADC2_PARRAM_BASE     0xff3a1000 /* ADC2 results RAM parity */
#define RM57_ADC1_RAM_BASE        0xff3e0000 /* ADC1 results RAM */
#define RM57_ADC1_PARRAM_BASE     0xff3e1000 /* ADC1 results RAM parity */
#define RM57_ADC1_LUT_BASE        0xff3e2000 /* ADC1 channel select LUT */

/* System Modules Control Registers and Memories */

#define RM57_DMARAM_BASE          0xfff80000 /* DMA control packet RAM */
#define RM57_VIMRAM_BASE          0xfff82000 /* VIM RAM */
#define RM57_FWRAP_BASE           0xfff87000 /* Flash Wrapper */
#define RM57_PCR1_BASE            0xffff1000 /* PCR frame 1 */

#define RM57_IOMM_BASE            0xffff1c00 /* I/O mux and control */
#define RM57_PCR3_BASE            0xfff78000 /* PCR frame 3 */

/* PCR frame 2, mirrored bus */
#define RM57_PCR2_BASE 0xfcff1000

/* System Module - Frame 2 */
#define RM57_SYS2_BASE 0xffffe100
#define RM57_PBIST_BASE           0xffffe400 /* PBIST controller */
#define RM57_STC1_BASE            0xffffe600 /* STC1: CPU LBIST */
#define RM57_STC2_BASE            0xffff0800 /* STC2: N2HET LBIST */
#define RM57_EFC_BASE             0xfff8c000 /* eFuse controller */
#define RM57_ESM_BASE             0xfffff500 /* ESM */
#define RM57_CCM_BASE             0xfffff600 /* CCM-R5F */
#define RM57_L2RAMW_BASE          0xfffff900 /* Level 2 RAM wrapper */
#define RM57_RTI1_BASE            0xfffffc00 /* RTI + DWWD */
#define RM57_VIM_BASE             0xfffffd00 /* VIM */

/* System Module - Frame 1 */
#define RM57_SYS1_BASE 0xffffff00
#define RM57_ECLK_PORT_BASE       0xffffff04 /* ECLK GIO port */
#define RM57_DMA_BASE             0xfffff000 /* DMA control registers */

/* Ethernet (EMAC / EMAC Control / MDIO).  These live under PCR2's PS[29]-
 * PS[31] segments and the CPPI RAM under PCR2's PCS[41] segment - outside
 * every other peripheral base in this file, which is why they are not
 * grouped with the PCR1-gated peripherals above.
 */

#define RM57_EMAC_CPPI_RAM_BASE   0xfc520000 /* Ethernet CPPI descriptor RAM */
#define RM57_EMAC_BASE            0xfcf78000 /* EMAC */
#define RM57_EMAC_CTRL_BASE       0xfcf78800 /* EMAC Control */
#define RM57_MDIO_BASE            0xfcf78900 /* MDIO */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_MEMORYMAP_H */
