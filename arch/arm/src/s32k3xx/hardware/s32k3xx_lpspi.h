/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_lpspi.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPSPI_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPSPI Register Offsets ***************************************************/

#define S32K3XX_LPSPI_VERID_OFFSET     (0x0000) /* Version ID Register (VERID) */
#define S32K3XX_LPSPI_PARAM_OFFSET     (0x0004) /* Parameter Register (PARAM) */
#define S32K3XX_LPSPI_CR_OFFSET        (0x0010) /* Control Register (CR) */
#define S32K3XX_LPSPI_SR_OFFSET        (0x0014) /* Status Register (SR) */
#define S32K3XX_LPSPI_IER_OFFSET       (0x0018) /* Interrupt Enable Register (IER) */
#define S32K3XX_LPSPI_DER_OFFSET       (0x001c) /* DMA Enable Register (DER) */
#define S32K3XX_LPSPI_CFGR0_OFFSET     (0x0020) /* Configuration Register 0 (CFGR0) */
#define S32K3XX_LPSPI_CFGR1_OFFSET     (0x0024) /* Configuration Register 1 (CFGR1) */
#define S32K3XX_LPSPI_DMR0_OFFSET      (0x0030) /* Data Match Register 0 (DMR0) */
#define S32K3XX_LPSPI_DMR1_OFFSET      (0x0034) /* Data Match Register 1 (DMR1) */
#define S32K3XX_LPSPI_CCR_OFFSET       (0x0040) /* Clock Configuration Register (CCR) */
#define S32K3XX_LPSPI_CCR1_OFFSET      (0x0044) /* Clock Configuration Register 1 (CCR1) */
#define S32K3XX_LPSPI_FCR_OFFSET       (0x0058) /* FIFO Control Register (FCR) */
#define S32K3XX_LPSPI_FSR_OFFSET       (0x005c) /* FIFO Status Register (FSR) */
#define S32K3XX_LPSPI_TCR_OFFSET       (0x0060) /* Transmit Command Register (TCR) */
#define S32K3XX_LPSPI_TDR_OFFSET       (0x0064) /* Transmit Data Register (TDR) */
#define S32K3XX_LPSPI_RSR_OFFSET       (0x0070) /* Receive Status Register (RSR) */
#define S32K3XX_LPSPI_RDR_OFFSET       (0x0074) /* Receive Data Register (RDR) */
#define S32K3XX_LPSPI_RDROR_OFFSET     (0x0078) /* Receive Data Read Only Register (RDROR) */
#define S32K3XX_LPSPI_TCBR_OFFSET      (0x03fc) /* Transmit Command Burst Register (TCBR) */

#define S32K3XX_LPSPI_TDBR_OFFSET(n)   (0x0400 + ((n) << 2)) /* Transmit Data Burst Register n=0..127 (TDBRn) */
#define S32K3XX_LPSPI_RDBR_OFFSET(n)   (0x0600 + ((n) << 2)) /* Receive Data Burst Register n=0..127 (RDBRn) */

/* LPSPI Register Addresses *************************************************/

#define S32K3XX_LPSPI0_VERID           (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_VERID_OFFSET)
#define S32K3XX_LPSPI0_PARAM           (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_PARAM_OFFSET)
#define S32K3XX_LPSPI0_CR              (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_CR_OFFSET)
#define S32K3XX_LPSPI0_SR              (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_SR_OFFSET)
#define S32K3XX_LPSPI0_IER             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_IER_OFFSET)
#define S32K3XX_LPSPI0_DER             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_DER_OFFSET)
#define S32K3XX_LPSPI0_CFGR0           (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_CFGR0_OFFSET)
#define S32K3XX_LPSPI0_CFGR1           (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_CFGR1_OFFSET)
#define S32K3XX_LPSPI0_DMR0            (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_DMR0_OFFSET)
#define S32K3XX_LPSPI0_DMR1            (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_DMR1_OFFSET)
#define S32K3XX_LPSPI0_CCR             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_CCR_OFFSET)
#define S32K3XX_LPSPI0_CCR1            (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_CCR1_OFFSET)
#define S32K3XX_LPSPI0_FCR             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_FCR_OFFSET)
#define S32K3XX_LPSPI0_FSR             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_FSR_OFFSET)
#define S32K3XX_LPSPI0_TCR             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_TCR_OFFSET)
#define S32K3XX_LPSPI0_TDR             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_TDR_OFFSET)
#define S32K3XX_LPSPI0_RSR             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_RSR_OFFSET)
#define S32K3XX_LPSPI0_RDR             (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_RDR_OFFSET)
#define S32K3XX_LPSPI0_RDROR           (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_RDROR_OFFSET)
#define S32K3XX_LPSPI0_TCBR            (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_TCBR_OFFSET)
#define S32K3XX_LPSPI0_TDBR(n)         (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_TDBR_OFFSET(n))
#define S32K3XX_LPSPI0_RDBR(n)         (S32K3XX_LPSPI0_BASE + S32K3XX_LPSPI_RDBR_OFFSET(n))

#define S32K3XX_LPSPI1_VERID           (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_VERID_OFFSET)
#define S32K3XX_LPSPI1_PARAM           (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_PARAM_OFFSET)
#define S32K3XX_LPSPI1_CR              (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_CR_OFFSET)
#define S32K3XX_LPSPI1_SR              (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_SR_OFFSET)
#define S32K3XX_LPSPI1_IER             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_IER_OFFSET)
#define S32K3XX_LPSPI1_DER             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_DER_OFFSET)
#define S32K3XX_LPSPI1_CFGR0           (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_CFGR0_OFFSET)
#define S32K3XX_LPSPI1_CFGR1           (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_CFGR1_OFFSET)
#define S32K3XX_LPSPI1_DMR0            (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_DMR0_OFFSET)
#define S32K3XX_LPSPI1_DMR1            (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_DMR1_OFFSET)
#define S32K3XX_LPSPI1_CCR             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_CCR_OFFSET)
#define S32K3XX_LPSPI1_CCR1            (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_CCR1_OFFSET)
#define S32K3XX_LPSPI1_FCR             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_FCR_OFFSET)
#define S32K3XX_LPSPI1_FSR             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_FSR_OFFSET)
#define S32K3XX_LPSPI1_TCR             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_TCR_OFFSET)
#define S32K3XX_LPSPI1_TDR             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_TDR_OFFSET)
#define S32K3XX_LPSPI1_RSR             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_RSR_OFFSET)
#define S32K3XX_LPSPI1_RDR             (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_RDR_OFFSET)
#define S32K3XX_LPSPI1_RDROR           (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_RDROR_OFFSET)
#define S32K3XX_LPSPI1_TCBR            (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_TCBR_OFFSET)
#define S32K3XX_LPSPI1_TDBR(n)         (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_TDBR_OFFSET(n))
#define S32K3XX_LPSPI1_RDBR(n)         (S32K3XX_LPSPI1_BASE + S32K3XX_LPSPI_RDBR_OFFSET(n))

#define S32K3XX_LPSPI2_VERID           (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_VERID_OFFSET)
#define S32K3XX_LPSPI2_PARAM           (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_PARAM_OFFSET)
#define S32K3XX_LPSPI2_CR              (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_CR_OFFSET)
#define S32K3XX_LPSPI2_SR              (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_SR_OFFSET)
#define S32K3XX_LPSPI2_IER             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_IER_OFFSET)
#define S32K3XX_LPSPI2_DER             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_DER_OFFSET)
#define S32K3XX_LPSPI2_CFGR0           (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_CFGR0_OFFSET)
#define S32K3XX_LPSPI2_CFGR1           (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_CFGR1_OFFSET)
#define S32K3XX_LPSPI2_DMR0            (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_DMR0_OFFSET)
#define S32K3XX_LPSPI2_DMR1            (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_DMR1_OFFSET)
#define S32K3XX_LPSPI2_CCR             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_CCR_OFFSET)
#define S32K3XX_LPSPI2_CCR1            (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_CCR1_OFFSET)
#define S32K3XX_LPSPI2_FCR             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_FCR_OFFSET)
#define S32K3XX_LPSPI2_FSR             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_FSR_OFFSET)
#define S32K3XX_LPSPI2_TCR             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_TCR_OFFSET)
#define S32K3XX_LPSPI2_TDR             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_TDR_OFFSET)
#define S32K3XX_LPSPI2_RSR             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_RSR_OFFSET)
#define S32K3XX_LPSPI2_RDR             (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_RDR_OFFSET)
#define S32K3XX_LPSPI2_RDROR           (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_RDROR_OFFSET)
#define S32K3XX_LPSPI2_TCBR            (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_TCBR_OFFSET)
#define S32K3XX_LPSPI2_TDBR(n)         (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_TDBR_OFFSET(n))
#define S32K3XX_LPSPI2_RDBR(n)         (S32K3XX_LPSPI2_BASE + S32K3XX_LPSPI_RDBR_OFFSET(n))

#define S32K3XX_LPSPI3_VERID           (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_VERID_OFFSET)
#define S32K3XX_LPSPI3_PARAM           (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_PARAM_OFFSET)
#define S32K3XX_LPSPI3_CR              (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_CR_OFFSET)
#define S32K3XX_LPSPI3_SR              (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_SR_OFFSET)
#define S32K3XX_LPSPI3_IER             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_IER_OFFSET)
#define S32K3XX_LPSPI3_DER             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_DER_OFFSET)
#define S32K3XX_LPSPI3_CFGR0           (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_CFGR0_OFFSET)
#define S32K3XX_LPSPI3_CFGR1           (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_CFGR1_OFFSET)
#define S32K3XX_LPSPI3_DMR0            (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_DMR0_OFFSET)
#define S32K3XX_LPSPI3_DMR1            (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_DMR1_OFFSET)
#define S32K3XX_LPSPI3_CCR             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_CCR_OFFSET)
#define S32K3XX_LPSPI3_CCR1            (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_CCR1_OFFSET)
#define S32K3XX_LPSPI3_FCR             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_FCR_OFFSET)
#define S32K3XX_LPSPI3_FSR             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_FSR_OFFSET)
#define S32K3XX_LPSPI3_TCR             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_TCR_OFFSET)
#define S32K3XX_LPSPI3_TDR             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_TDR_OFFSET)
#define S32K3XX_LPSPI3_RSR             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_RSR_OFFSET)
#define S32K3XX_LPSPI3_RDR             (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_RDR_OFFSET)
#define S32K3XX_LPSPI3_RDROR           (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_RDROR_OFFSET)
#define S32K3XX_LPSPI3_TCBR            (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_TCBR_OFFSET)
#define S32K3XX_LPSPI3_TDBR(n)         (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_TDBR_OFFSET(n))
#define S32K3XX_LPSPI3_RDBR(n)         (S32K3XX_LPSPI3_BASE + S32K3XX_LPSPI_RDBR_OFFSET(n))

#define S32K3XX_LPSPI4_VERID           (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_VERID_OFFSET)
#define S32K3XX_LPSPI4_PARAM           (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_PARAM_OFFSET)
#define S32K3XX_LPSPI4_CR              (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_CR_OFFSET)
#define S32K3XX_LPSPI4_SR              (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_SR_OFFSET)
#define S32K3XX_LPSPI4_IER             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_IER_OFFSET)
#define S32K3XX_LPSPI4_DER             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_DER_OFFSET)
#define S32K3XX_LPSPI4_CFGR0           (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_CFGR0_OFFSET)
#define S32K3XX_LPSPI4_CFGR1           (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_CFGR1_OFFSET)
#define S32K3XX_LPSPI4_DMR0            (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_DMR0_OFFSET)
#define S32K3XX_LPSPI4_DMR1            (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_DMR1_OFFSET)
#define S32K3XX_LPSPI4_CCR             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_CCR_OFFSET)
#define S32K3XX_LPSPI4_CCR1            (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_CCR1_OFFSET)
#define S32K3XX_LPSPI4_FCR             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_FCR_OFFSET)
#define S32K3XX_LPSPI4_FSR             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_FSR_OFFSET)
#define S32K3XX_LPSPI4_TCR             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_TCR_OFFSET)
#define S32K3XX_LPSPI4_TDR             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_TDR_OFFSET)
#define S32K3XX_LPSPI4_RSR             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_RSR_OFFSET)
#define S32K3XX_LPSPI4_RDR             (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_RDR_OFFSET)
#define S32K3XX_LPSPI4_RDROR           (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_RDROR_OFFSET)
#define S32K3XX_LPSPI4_TCBR            (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_TCBR_OFFSET)
#define S32K3XX_LPSPI4_TDBR(n)         (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_TDBR_OFFSET(n))
#define S32K3XX_LPSPI4_RDBR(n)         (S32K3XX_LPSPI4_BASE + S32K3XX_LPSPI_RDBR_OFFSET(n))

#define S32K3XX_LPSPI5_VERID           (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_VERID_OFFSET)
#define S32K3XX_LPSPI5_PARAM           (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_PARAM_OFFSET)
#define S32K3XX_LPSPI5_CR              (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_CR_OFFSET)
#define S32K3XX_LPSPI5_SR              (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_SR_OFFSET)
#define S32K3XX_LPSPI5_IER             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_IER_OFFSET)
#define S32K3XX_LPSPI5_DER             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_DER_OFFSET)
#define S32K3XX_LPSPI5_CFGR0           (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_CFGR0_OFFSET)
#define S32K3XX_LPSPI5_CFGR1           (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_CFGR1_OFFSET)
#define S32K3XX_LPSPI5_DMR0            (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_DMR0_OFFSET)
#define S32K3XX_LPSPI5_DMR1            (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_DMR1_OFFSET)
#define S32K3XX_LPSPI5_CCR             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_CCR_OFFSET)
#define S32K3XX_LPSPI5_CCR1            (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_CCR1_OFFSET)
#define S32K3XX_LPSPI5_FCR             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_FCR_OFFSET)
#define S32K3XX_LPSPI5_FSR             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_FSR_OFFSET)
#define S32K3XX_LPSPI5_TCR             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_TCR_OFFSET)
#define S32K3XX_LPSPI5_TDR             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_TDR_OFFSET)
#define S32K3XX_LPSPI5_RSR             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_RSR_OFFSET)
#define S32K3XX_LPSPI5_RDR             (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_RDR_OFFSET)
#define S32K3XX_LPSPI5_RDROR           (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_RDROR_OFFSET)
#define S32K3XX_LPSPI5_TCBR            (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_TCBR_OFFSET)
#define S32K3XX_LPSPI5_TDBR(n)         (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_TDBR_OFFSET(n))
#define S32K3XX_LPSPI5_RDBR(n)         (S32K3XX_LPSPI5_BASE + S32K3XX_LPSPI_RDBR_OFFSET(n))

/* LPSPI Register Bitfield Definitions **************************************/

/* Version ID Register (VERID)  */

#define LPSPI_VERID_FEATURE_SHIFT      (0)       /* Bits 0-15: Module Identification Number (FEATURE) */
#define LPSPI_VERID_FEATURE_MASK       (0xffff << LPSPI_VERID_FEATURE_SHIFT)
#define LPSPI_VERID_MINOR_SHIFT        (16)      /* Bits 16-23: Minor Version Number (MINOR) */
#define LPSPI_VERID_MINOR_MASK         (0xff << LPSPI_VERID_MINOR_SHIFT)
#define LPSPI_VERID_MAJOR_SHIFT        (24)      /* Bits 24-31: Major Version Number (MAJOR) */
#define LPSPI_VERID_MAJOR_MASK         (0xff << LPSPI_VERID_MAJOR_SHIFT)

/* Parameter Register (PARAM) */

#define LPSPI_PARAM_TXFIFO_SHIFT       (0)       /* Bits 0-7: Transmit FIFO Size (TXFIFO) */
#define LPSPI_PARAM_TXFIFO_MASK        (0xff << LPSPI_PARAM_TXFIFO_SHIFT)
#define LPSPI_PARAM_RXFIFO_SHIFT       (8)       /* Bits 8-15: Receive FIFO Size (RXFIFO) */
#define LPSPI_PARAM_RXFIFO_MASK        (0xff << LPSPI_PARAM_RXFIFO_SHIFT)
#define LPSPI_PARAM_PCSNUM_SHIFT       (16)      /* Bits 16-23: PCS Number (PCSNUM) */
#define LPSPI_PARAM_PCSNUM_MASK        (0xff << LPSPI_PARAM_PCSNUM_SHIFT)
                                                 /* Bits 24-31: Reserved */

/* Control Register (CR) */

#define LPSPI_CR_MEN                   (1 << 0)  /* Bit 0: Module Enable (MEN) */
#define LPSPI_CR_RST                   (1 << 1)  /* Bit 1: Software Reset (RST) */
                                                 /* Bit 2: Reserved */
#define LPSPI_CR_DBGEN                 (1 << 3)  /* Bit 3: Debug Enable (DBGEN) */
                                                 /* Bits 4-7: Reserved */
#define LPSPI_CR_RTF                   (1 << 8)  /* Bit 8: Reset Transmit FIFO (RTF) */
#define LPSPI_CR_RRF                   (1 << 9)  /* Bit 9: Reset Receive FIFO (RRF) */
                                                 /* Bits 10-31: Reserved */

/* Status Register (SR) */

#define LPSPI_SR_TDF                   (1 << 0)  /* Bit 0: Transmit Data Flag (TDF) */
#define LPSPI_SR_RDF                   (1 << 1)  /* Bit 1: Receive Data Flag (RDF) */
                                                 /* Bits 2-7: Reserved */
#define LPSPI_SR_WCF                   (1 << 8)  /* Bit 8: Word Complete Flag (WCF) */
#define LPSPI_SR_FCF                   (1 << 9)  /* Bit 9: Frame Complete Flag (FCF) */
#define LPSPI_SR_TCF                   (1 << 10) /* Bit 10: Transfer Complete Flag (TCF) */
#define LPSPI_SR_TEF                   (1 << 11) /* Bit 11: Transmit Error Flag (TEF) */
#define LPSPI_SR_REF                   (1 << 12) /* Bit 12: Receive Error Flag (REF) */
#define LPSPI_SR_DMF                   (1 << 13) /* Bit 13: Data Match Flag (DMF) */
                                                 /* Bits 14-23: Reserved */
#define LPSPI_SR_MBF                   (1 << 24) /* Bit 24: Module Busy Flag (MBF) */
                                                 /* Bits 25-31: Reserved */

/* Interrupt Enable Register (IER) */

#define LPSPI_IER_TDIE                 (1 << 0)  /* Bit 0: Transmit Data Interrupt Enable (TDIE) */
#define LPSPI_IER_RDIE                 (1 << 1)  /* Bit 1: Receive Data Interrupt Enable (RDIE) */
                                                 /* Bits 2-7: Reserved */
#define LPSPI_IER_WCIE                 (1 << 8)  /* Bit 8: Word Complete Interrupt Enable (WCIE) */
#define LPSPI_IER_FCIE                 (1 << 9)  /* Bit 9: Frame Complete Interrupt Enable (FCIE) */
#define LPSPI_IER_TCIE                 (1 << 10) /* Bit 10: Transfer Complete Interrupt Enable (TCIE) */
#define LPSPI_IER_TEIE                 (1 << 11) /* Bit 11: Transmit Error Interrupt Enable (TEIE) */
#define LPSPI_IER_REIE                 (1 << 12) /* Bit 12: Receive Error Interrupt Enable (REIE) */
#define LPSPI_IER_DMIE                 (1 << 13) /* Bit 13: Data Match Interrupt Enable (DMIE) */
                                                 /* Bits 14-31: Reserved */

/* DMA Enable Register (DER) */

#define LPSPI_DER_TDDE                 (1 << 0)  /* Bit 0: Transmit Data DMA Enable (TDDE) */
#define LPSPI_DER_RDDE                 (1 << 1)  /* Bit 1: Receive Data DMA Enable (RDDE) */
                                                 /* Bits 2-31: Reserved */

/* Configuration Register 0 (CFGR0) */

#define LPSPI_CFGR0_HREN               (1 << 0)  /* Bit 0: Host Request Enable (HREN) */
#define LPSPI_CFGR0_HRPOL              (1 << 1)  /* Bit 1: Host Request Polarity (HRPOL) */
#  define LPSPI_CFGR0_HRPOL_HIGH       (0 << 1)  /*        HREQ pin or input trigger is active high */
#  define LPSPI_CFGR0_HRPOL_LOW        (1 << 1)  /*        HREQ pin or input trigger is active low */
#define LPSPI_CFGR0_HRSEL              (1 << 2)  /* Bit 2: Host Request Select (HRSEL) */
#  define LPSPI_CFGR0_HRSEL_HREQ       (0 << 2)  /*        Host request input is the LPSPI_HREQ pin */
#  define LPSPI_CFGR0_HRSEL_INTR       (1 << 2)  /*        Host request input is the input trigger */
#define LPSPI_CFGR0_HRDIR              (1 << 3)  /* Bit 3: Host Request Direction (HRDIR) */
#  define LPSPI_CFGR0_HRDIR_INPUT      (0 << 3)  /*        HREQ pin is configured as input */
#  define LPSPI_CFGR0_HRDIR_OUTPUT     (1 << 3)  /*        HREQ pin is configured as output */
                                                 /* Bits 4-7: Reserved */
#define LPSPI_CFGR0_CIRFIFO            (1 << 8)  /* Bit 8: Circular FIFO Enable (CIRCFIFO) */
#define LPSPI_CFGR0_RDMO               (1 << 9)  /* Bit 9: Receive Data Match Only (RDMO) */
#  define LPSPI_CFGR0_RDMO_FIFO        (0 << 9)  /*        Received data is stored in the receive FIFO as in normal operations */
#  define LPSPI_CFGR0_RDMO_DMF         (1 << 9)  /*        Received data is discarded unless the Data Match Flag (DMF) is set */
                                                 /* Bits 10-31: Reserved */

/* Configuration Register 1 (CFGR1) */

#define LPSPI_CFGR1_MASTER             (1 << 0)  /* Bit 0: Master Mode (MASTER) */
#define LPSPI_CFGR1_SAMPLE             (1 << 1)  /* Bit 1: Sample Point (SAMPLE) */
#  define LPSPI_CFGR1_SAMPLE_SCK       (0 << 1)  /*        Input data is sampled on SCK edge */
#  define LPSPI_CFGR1_SAMPLE_DELAY     (1 << 1)  /*        Input data is sampled on delayed SCK edge */
#define LPSPI_CFGR1_AUTOPCS            (1 << 2)  /* Bit 2: Automatic PCS (AUTOPCS) */
#define LPSPI_CFGR1_NOSTALL            (1 << 3)  /* Bit 3: No Stall (NOSTALL) */
#define LPSPI_CFGR1_PARTIAL            (1 << 4)  /* Bit 4: Partial Enable (PARTIAL) */
                                                 /* Bits 5-7: Reserved */
#define LPSPI_CFGR1_PCSPOL_SHIFT       (8)       /* Bits 8-15: Peripheral Chip Select Polarity (PCSPOL) */
#define LPSPI_CFGR1_PCSPOL_MASK        (0xff << LPSPI_CFGR1_PCSPOL_SHIFT)
#  define LPSPI_CFGR1_PCSPOL_LOW(n)    (0 << (LPSPI_CFGR1_PCSPOL_SHIFT + (n))) /* The Peripheral Chip Select PCS[n] pin is active low */
#  define LPSPI_CFGR1_PCSPOL_HIGH(n)   (1 << (LPSPI_CFGR1_PCSPOL_SHIFT + (n))) /* The Peripheral Chip Select PCS[n] pin is active high */

#define LPSPI_CFGR1_MATCFG_SHIFT       (16)      /* Bits 16-18: Match Configuration (MATCFG) */
#define LPSPI_CFGR1_MATCFG_MASK        (0x07 << LPSPI_CFGR1_MATCFG_SHIFT)
#define LPSPI_CFGR1_MATCFG_DIS         (0x00 << LPSPI_CFGR1_MATCFG_SHIFT)  /* Match is disabled */

                                                 /* Bits 19-23: Reserved */
#define LPSPI_CFGR1_PINCFG_SHIFT       (24)      /* Bits 24-25: Pin Configuration (PINCFG) */
#define LPSPI_CFGR1_PINCFG_MASK        (0x03 << LPSPI_CFGR1_PINCFG_SHIFT)
#  define LPSPI_CFGR1_PINCFG_SIN_SOUT  (0x00 << LPSPI_CFGR1_PINCFG_SHIFT) /* SIN is used for input data and SOUT is used for output data */
#  define LPSPI_CFGR1_PINCFG_SIN_SIN   (0x01 << LPSPI_CFGR1_PINCFG_SHIFT) /* SIN is used for both input and output data */
#  define LPSPI_CFGR1_PINCFG_SOUT_SOUT (0x02 << LPSPI_CFGR1_PINCFG_SHIFT) /* SOUT is used for both input and output data */
#  define LPSPI_CFGR1_PINCFG_SOUT_SIN  (0x03 << LPSPI_CFGR1_PINCFG_SHIFT) /* SOUT is used for input data and SIN is used for output data */
#  define LPSPI_CFGR1_PINCFG(n)        ((n) << LPSPI_CFGR1_PINCFG_SHIFT)

#define LPSPI_CFGR1_OUTCFG             (1 << 26) /* Bit 26: Output Config (OUTCFG) */
#  define LPSPI_CFGR1_OUTCFG_RETAIN    (0 << 26) /*         Output data retains last value when chip select is negated */
#  define LPSPI_CFGR1_OUTCFG_TRISTATE  (1 << 26) /*         Output data is tristated when chip select is negated */
#define LPSPI_CFGR1_PCSCFG_SHIFT       (27)      /* Bits 27-28: Peripheral Chip Select Configuration (PCSCFG) */
#define LPSPI_CFGR1_PCSCFG_MASK        (0x03 << LPSPI_CFGR1_PCSCFG_SHIFT)
#  define LPSPI_CFGR1_PCSCFG_PCS       (0x00 << LPSPI_CFGR1_PCSCFG_SHIFT) /* PCS[2:7] are configured for chip select function */
#  define LPSPI_CFGR1_PCSCFG_4BIT      (0x01 << LPSPI_CFGR1_PCSCFG_SHIFT) /* PCS[2:3] are configured for half-duplex 4-bit transfers */
#  define LPSPI_CFGR1_PCSCFG_8BIT      (0x03 << LPSPI_CFGR1_PCSCFG_SHIFT) /* PCS[2:7] are configured for half-duplex 4-bit and 8-bit transfers */

                                                 /* Bits 29-31: Reserved */

/* Data Match Register 0 (DMR0) */

#define LPSPI_DMR0_MATCH0_SHIFT        (0)       /* Bits 0-31: Match 0 Value (MATCH0) */
#define LPSPI_DMR0_MATCH0_MASK         (0xffffffff << LPSPI_DMR0_MATCH0_SHIFT)

/* Data Match Register 0 (DMR1) */

#define LPSPI_DMR1_MATCH1_SHIFT        (0)       /* Bits 0-31: Match 1 Value (MATCH1) */
#define LPSPI_DMR1_MATCH1_MASK         (0xffffffff << LPSPI_DMR1_MATCH1_SHIFT)

/* Clock Configuration Register (CCR) */

#define LPSPI_CCR_SCKDIV_SHIFT         (0)       /* Bits 0-7: SCK Divider (SCKDIV) */
#define LPSPI_CCR_SCKDIV_MASK          (0xff << LPSPI_CCR_SCKDIV_SHIFT)
#    define LPSPI_CCR_SCKDIV(n)        (((uint32_t)(n) << LPSPI_CCR_SCKDIV_SHIFT) & LPSPI_CCR_SCKDIV_MASK)
#define LPSPI_CCR_DBT_SHIFT            (8)       /* Bits 8-15: Delay Between Transfers (DBT) */
#define LPSPI_CCR_DBT_MASK             (0xff << LPSPI_CCR_DBT_SHIFT)
#    define LPSPI_CCR_DBT(n)           (((uint32_t)(n) << LPSPI_CCR_DBT_SHIFT) & LPSPI_CCR_DBT_MASK)
#define LPSPI_CCR_PCSSCK_SHIFT         (16)      /* Bits 16-23: PCS-to-SCK Delay (PCSSCK) */
#define LPSPI_CCR_PCSSCK_MASK          (0xff << LPSPI_CCR_PCSSCK_SHIFT)
#    define LPSPI_CCR_PCSSCK(n)        (((uint32_t)(n) << LPSPI_CCR_PCSSCK_SHIFT) & LPSPI_CCR_PCSSCK_MASK)
#define LPSPI_CCR_SCKPCS_SHIFT         (24)      /* Bits 24-31: SCK-to-PCS Delay (SCKPCS) */
#define LPSPI_CCR_SCKPCS_MASK          (0xff << LPSPI_CCR_SCKPCS_SHIFT)
#    define LPSPI_CCR_SCKPCS(n)        (((uint32_t)(n) << LPSPI_CCR_SCKPCS_SHIFT) & LPSPI_CCR_SCKPCS_MASK)

/* Clock Configuration Register 1 (CCR1) */

#define LPSPI_CCR1_SCKSET_SHIFT        (0)       /* Bits 0-7: SCK Setup (SCKSET) */
#define LPSPI_CCR1_SCKSET_MASK         (0xff << LPSPI_CCR1_SCKSET_SHIFT)
#define LPSPI_CCR1_SCKHLD_SHIFT        (8)       /* Bits 8-15: SCK Hold (SCKHLD) */
#define LPSPI_CCR1_SCKHLD_MASK         (0xff << LPSPI_CCR1_SCKHLD_SHIFT)
#define LPSPI_CCR1_PCSPCS_SHIFT        (16)      /* Bits 16-23: PCS to PCS Delay (PCSPCS) */
#define LPSPI_CCR1_PCSPCS_MASK         (0xff << LPSPI_CCR1_PCSPCS_SHIFT)
#define LPSPI_CCR1_SCKSCK_SHIFT        (24)      /* Bits 24-31: SCK Inter-Frame Delay (SCKSCK) */
#define LPSPI_CCR1_SCKSCK_MASK         (0xff << LPSPI_CCR1_SCKSCK_SHIFT)

/* FIFO Control Register (FCR) */

#define LPSPI_FCR_TXWATER_SHIFT        (0)       /* Bits 0-1: Transmit FIFO Watermark (TXWATER) */
#define LPSPI_FCR_TXWATER_MASK         (0x03 << LPSPI_FCR_TXWATER_SHIFT)
#    define LPSPI_FCR_TXWATER(n)       ((uint32_t)(n) << LPSPI_FCR_TXWATER_SHIFT)
                                                 /* Bits 2-15: Reserved */
#define LPSPI_FCR_RXWATER_SHIFT        (16)      /* Bits 16-17: Receive FIFO Watermark (RXWATER) */
#define LPSPI_FCR_RXWATER_MASK         (0x03 << LPSPI_FCR_RXWATER_SHIFT)
#    define LPSPI_FCR_RXWATER(n)       ((uint32_t)(n) << LPSPI_FCR_RXWATER_SHIFT)
                                                 /* Bits 18-31: Reserved */

/* FIFO Status Register (FSR) */

#define LPSPI_FSR_TXCOUNT_SHIFT        (0)       /* Bits 0-2: Transmit FIFO Count (TXCOUNT) */
#define LPSPI_FSR_TXCOUNT_MASK         (0x07 << LPSPI_FSR_TXCOUNT_SHIFT)
                                                 /* Bits 3-15: Reserved */
#define LPSPI_FSR_RXCOUNT_SHIFT        (16)      /* Bits 16-18: Receive FIFO Count (RXCOUNT) */
#define LPSPI_FSR_RXCOUNT_MASK         (0x07 << LPSPI_FSR_RXCOUNT_SHIFT)
                                                 /* Bits 19-31: Reserved */

/* Transmit Command Register (TCR) */

#define LPSPI_TCR_FRAMESZ_SHIFT        (0)       /* Bits 0-11: Frame Size (FRAMESZ) */
#define LPSPI_TCR_FRAMESZ_MASK         (0x0fff << LPSPI_TCR_FRAMESZ_SHIFT)
#    define LPSPI_TCR_FRAMESZ(n)       ((uint32_t)(n) << LPSPI_TCR_FRAMESZ_SHIFT)
                                                 /* Bits 12-15: Reserved */
#define LPSPI_TCR_WIDTH_SHIFT          (16)      /* Bits 16-17: Transfer Width (WIDTH) */
#define LPSPI_TCR_WIDTH_MASK           (0x03 << LPSPI_TCR_WIDTH_SHIFT)
#  define LPSPI_TCR_WIDTH_1BIT         (0x00 << LPSPI_TCR_WIDTH_SHIFT) /* 1 bit transfer */
#  define LPSPI_TCR_WIDTH_2BIT         (0x01 << LPSPI_TCR_WIDTH_SHIFT) /* 2 bit transfer */
#  define LPSPI_TCR_WIDTH_4BIT         (0x02 << LPSPI_TCR_WIDTH_SHIFT) /* 4 bit transfer */
#  define LPSPI_TCR_WIDTH_8BIT         (0x03 << LPSPI_TCR_WIDTH_SHIFT) /* 8 bit transfer */

#define LPSPI_TCR_TXMSK                (1 << 18) /* Bit 18: Transmit Data Mask (TXMSK) */
#define LPSPI_TCR_RXMSK                (1 << 19) /* Bit 19: Receive Data Mask (RXMSK) */
#define LPSPI_TCR_CONTC                (1 << 20) /* Bit 20: Continuing Command (CONTC) */
#define LPSPI_TCR_CONT                 (1 << 21) /* Bit 21: Continuous Transfer (CONT) */
#define LPSPI_TCR_BYSW                 (1 << 22) /* Bit 22: Byte Swap (BYSW) */
#define LPSPI_TCR_LSBF                 (1 << 23) /* Bit 23: LSB First (LSBF) */
#  define LPSPI_TCR_MSBF               (0 << 23) /*         MSB First */
#define LPSPI_TCR_PCS_SHIFT            (24)      /* Bits 24-26: Peripheral Chip Select (PCS) */
#define LPSPI_TCR_PCS_MASK             (0x07 << LPSPI_TCR_PCS_SHIFT)
#  define LPSPI_TCR_PCS_0              (0x00 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[0] */
#  define LPSPI_TCR_PCS_1              (0x01 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[1] */
#  define LPSPI_TCR_PCS_2              (0x02 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[2] */
#  define LPSPI_TCR_PCS_3              (0x03 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[3] */
#  define LPSPI_TCR_PCS_4              (0x04 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[4] */
#  define LPSPI_TCR_PCS_5              (0x05 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[5] */
#  define LPSPI_TCR_PCS_6              (0x06 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[6] */
#  define LPSPI_TCR_PCS_7              (0x07 << LPSPI_TCR_PCS_SHIFT) /* Transfer using PCS[7] */

#define LPSPI_TCR_PRESCALE_SHIFT       (27)      /* Bits 27-29: Prescaler Value (PRESCALE) */
#define LPSPI_TCR_PRESCALE_MASK        (0x07 << LPSPI_TCR_PRESCALE_SHIFT)
#  define LPSPI_TCR_PRESCALE_DIV1      (0x00 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 1 */
#  define LPSPI_TCR_PRESCALE_DIV2      (0x01 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 2 */
#  define LPSPI_TCR_PRESCALE_DIV4      (0x02 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 4 */
#  define LPSPI_TCR_PRESCALE_DIV8      (0x03 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 8 */
#  define LPSPI_TCR_PRESCALE_DIV16     (0x04 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 16 */
#  define LPSPI_TCR_PRESCALE_DIV32     (0x05 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 32 */
#  define LPSPI_TCR_PRESCALE_DIV64     (0x06 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 64 */
#  define LPSPI_TCR_PRESCALE_DIV128    (0x07 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 128 */
#  define LPSPI_TCR_PRESCALE(n)        ((n) << LPSPI_TCR_PRESCALE_SHIFT)

#define LPSPI_TCR_CPHA                 (1 << 30) /* Bit 30: Clock Phase (CPHA) */
#  define LPSPI_TCR_CPHA_CAPTURED      (0 << 30) /*         Data is captured on the leading edge of SCK and changed on the following edge of SCK */
#  define LPSPI_TCR_CPHA_CHANGED       (1 << 30) /*         Data is changed on the leading edge of SCK and captured on the following edge of SCK */
#define LPSPI_TCR_CPOL                 (1 << 31) /* Bit 31: Clock Polarity (CPOL) */
#  define LPSPI_TCR_CPOL_LOW           (0 << 31) /*         The inactive state value of SCK is low */
#  define LPSPI_TCR_CPOL_HIGH          (1 << 31) /*         The inactive state value of SCK is high */

/* Transmit Data Register (TDR) */

#define LPSPI_TDR_DATA_SHIFT           (0)       /* Bits 0-31: Transmit Data (DATA) */
#define LPSPI_TDR_DATA_MASK            (0xffffffff << LPSPI_TDR_DATA_SHIFT)

/* Receive Status Register (RSR) */

#define LPSPI_RSR_SOF                  (1 << 0)  /* Bit 0: Start Of Frame (SOF) */
#define LPSPI_RSR_RXEMPTY              (1 << 1)  /* Bit 1: RX FIFO Empty (RXEMPTY) */
                                                 /* Bits 2-31: Reserved */

/* Receive Data Register (RDR) */

#define LPSPI_RDR_DATA_SHIFT           (0)       /* Bits 0-31: Receive Data (DATA) */
#define LPSPI_RDR_DATA_MASK            (0xffffffff << LPSPI_RDR_DATA_SHIFT)

/* Receive Data Read Only Register (RDROR) */

#define LPSPI_RDROR_DATA_SHIFT         (0)       /* Bits 0-31: Receive Data (DATA) */
#define LPSPI_RDROR_DATA_MASK          (0xffffffff << LPSPI_RDROR_DATA_SHIFT)

/* Transmit Command Burst Register (TCBR) */

#define LPSPI_TCBR_DATA_SHIFT          (0)       /* Bits 0-31: Command Data (DATA) */
#define LPSPI_TCBR_DATA_MASK           (0xffffffff << LPSPI_TCBR_DATA_SHIFT)

/* Transmit Data Burst Register (TDBR) */

#define LPSPI_TDBR_DATA_SHIFT          (0)       /* Bits 0-31: Data (DATA) */
#define LPSPI_TDBR_DATA_MASK           (0xffffffff << LPSPI_TDBR_DATA_SHIFT)

/* Receive Data Burst Register (RDBR) */

#define LPSPI_RDBR_DATA_SHIFT          (0)       /* Bits 0-31: Data (DATA) */
#define LPSPI_RDBR_DATA_MASK           (0xffffffff << LPSPI_RDBR_DATA_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPSPI_H */
