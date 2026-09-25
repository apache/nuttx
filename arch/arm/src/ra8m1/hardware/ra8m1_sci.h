/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_sci.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_SCI_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_SCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_SCI_B_RDR_OFFSET                  0x0000  /* Received Data Register (32-bits) */
#define R_SCI_B_TDR_OFFSET                  0x0004  /* Transmission Data Register (32-bits) */
#define R_SCI_B_TDR_HA_L_OFFSET             0x0004  /* Transmission Data Register (16-bits) */
#define R_SCI_B_TDR_BY_LL_OFFSET            0x0004  /* Transmission Data Register (8-bits) */
#define R_SCI_B_TDR_BY_LH_OFFSET            0x0005  /* Transmission Data Register (8-bits) */
#define R_SCI_B_CCR0_OFFSET                 0x0008  /* Common Control Register 0 (32-bits) */
#define R_SCI_B_CCR0_HA_L_OFFSET            0x0008  /* Common Control Register 0 (16-bits) */
#define R_SCI_B_CCR0_BY_LL_OFFSET           0x0008  /* Common Control Register 0 (8-bits) */
#define R_SCI_B_CCR0_BY_LH_OFFSET           0x0009  /* Common Control Register 0 (8-bits) */
#define R_SCI_B_CCR0_HA_H_OFFSET            0x000a  /* Common Control Register 0 (16-bits) */
#define R_SCI_B_CCR0_BY_HL_OFFSET           0x000a  /* Common Control Register 0 (8-bits) */
#define R_SCI_B_CCR0_BY_HH_OFFSET           0x000b  /* Common Control Register 0 (8-bits) */
#define R_SCI_B_CCR1_OFFSET                 0x000c  /* Common Control Register 1 (32-bits) */
#define R_SCI_B_CCR1_HA_L_OFFSET            0x000c  /* Common Control Register 1 (16-bits) */
#define R_SCI_B_CCR1_BY_LL_OFFSET           0x000c  /* Common Control Register 1 (8-bits) */
#define R_SCI_B_CCR1_BY_LH_OFFSET           0x000d  /* Common Control Register 1 (8-bits) */
#define R_SCI_B_CCR1_HA_H_OFFSET            0x000e  /* Common Control Register 1 (16-bits) */
#define R_SCI_B_CCR1_BY_HL_OFFSET           0x000e  /* Common Control Register 1 (8-bits) */
#define R_SCI_B_CCR1_BY_HH_OFFSET           0x000f  /* Common Control Register 1 (8-bits) */
#define R_SCI_B_CCR2_OFFSET                 0x0010  /* Common Control Register 2 (32-bits) */
#define R_SCI_B_CCR2_HA_L_OFFSET            0x0010  /* Common Control Register 2 (16-bits) */
#define R_SCI_B_CCR2_BY_LL_OFFSET           0x0010  /* Common Control Register 2 (8-bits) */
#define R_SCI_B_CCR2_BY_LH_OFFSET           0x0011  /* Common Control Register 2 (8-bits) */
#define R_SCI_B_CCR2_HA_H_OFFSET            0x0012  /* Common Control Register 2 (16-bits) */
#define R_SCI_B_CCR2_BY_HL_OFFSET           0x0012  /* Common Control Register 2 (8-bits) */
#define R_SCI_B_CCR2_BY_HH_OFFSET           0x0013  /* Common Control Register 2 (8-bits) */
#define R_SCI_B_CCR3_OFFSET                 0x0014  /* Common Control Register 3 (32-bits) */
#define R_SCI_B_CCR3_HA_L_OFFSET            0x0014  /* Common Control Register 3 (16-bits) */
#define R_SCI_B_CCR3_BY_LL_OFFSET           0x0014  /* Common Control Register 3 (8-bits) */
#define R_SCI_B_CCR3_BY_LH_OFFSET           0x0015  /* Common Control Register 3 (8-bits) */
#define R_SCI_B_CCR3_HA_H_OFFSET            0x0016  /* Common Control Register 3 (16-bits) */
#define R_SCI_B_CCR3_BY_HL_OFFSET           0x0016  /* Common Control Register 3 (8-bits) */
#define R_SCI_B_CCR3_BY_HH_OFFSET           0x0017  /* Common Control Register 3 (8-bits) */
#define R_SCI_B_CCR4_OFFSET                 0x0018  /* Common Control Register 4 (32-bits) */
#define R_SCI_B_CCR4_HA_L_OFFSET            0x0018  /* Common Control Register 4 (16-bits) */
#define R_SCI_B_CCR4_BY_LL_OFFSET           0x0018  /* Common Control Register 4 (8-bits) */
#define R_SCI_B_CCR4_BY_LH_OFFSET           0x0019  /* Common Control Register 4 (8-bits) */
#define R_SCI_B_CCR4_HA_H_OFFSET            0x001a  /* Common Control Register 4 (16-bits) */
#define R_SCI_B_CCR4_BY_HL_OFFSET           0x001a  /* Common Control Register 4 (8-bits) */
#define R_SCI_B_CCR4_BY_HH_OFFSET           0x001b  /* Common Control Register 4 (8-bits) */
#define R_SCI_B_CESR_OFFSET                 0x001c  /* Communication Enable Status Register (8-bits) */
#define R_SCI_B_HCR_OFFSET                  0x001e  /* HBS valid mode Control Register (8-bits) */
#define R_SCI_B_ICR_OFFSET                  0x0020  /* Simple-I2C Control Register (32-bits) */
#define R_SCI_B_ICR_HA_L_OFFSET             0x0020  /* Simple-I2C Control Register (16-bits) */
#define R_SCI_B_ICR_BY_LL_OFFSET            0x0020  /* Simple-I2C Control Register (8-bits) */
#define R_SCI_B_ICR_BY_LH_OFFSET            0x0021  /* Simple-I2C Control Register (8-bits) */
#define R_SCI_B_ICR_HA_H_OFFSET             0x0022  /* Simple-I2C Control Register (16-bits) */
#define R_SCI_B_ICR_BY_HL_OFFSET            0x0022  /* Simple-I2C Control Register (8-bits) */
#define R_SCI_B_FCR_OFFSET                  0x0024  /* FIFO Control Register (32-bits) */
#define R_SCI_B_FCR_HA_L_OFFSET             0x0024  /* FIFO Control Register (16-bits) */
#define R_SCI_B_FCR_BY_LL_OFFSET            0x0024  /* FIFO Control Register (8-bits) */
#define R_SCI_B_FCR_BY_LH_OFFSET            0x0025  /* FIFO Control Register (8-bits) */
#define R_SCI_B_FCR_HA_H_OFFSET             0x0026  /* FIFO Control Register (16-bits) */
#define R_SCI_B_FCR_BY_HL_OFFSET            0x0026  /* FIFO Control Register (8-bits) */
#define R_SCI_B_FCR_BY_HH_OFFSET            0x0027  /* FIFO Control Register (8-bits) */
#define R_SCI_B_MCR_OFFSET                  0x002c  /* Manchester Control Register (32-bits) */
#define R_SCI_B_MCR_HA_L_OFFSET             0x002c  /* Manchester Control Register (16-bits) */
#define R_SCI_B_MCR_BY_LL_OFFSET            0x002c  /* Manchester Control Register (8-bits) */
#define R_SCI_B_MCR_BY_LH_OFFSET            0x002d  /* Manchester Control Register (8-bits) */
#define R_SCI_B_MCR_HA_H_OFFSET             0x002e  /* Manchester Control Register (16-bits) */
#define R_SCI_B_MCR_BY_HL_OFFSET            0x002e  /* Manchester Control Register (8-bits) */
#define R_SCI_B_MCR_BY_HH_OFFSET            0x002f  /* Manchester Control Register (8-bits) */
#define R_SCI_B_DCR_OFFSET                  0x0030  /* Driver Control Register (32-bits) */
#define R_SCI_B_DCR_HA_L_OFFSET             0x0030  /* Driver Control Register (16-bits) */
#define R_SCI_B_DCR_BY_LL_OFFSET            0x0030  /* Driver Control Register (8-bits) */
#define R_SCI_B_DCR_BY_LH_OFFSET            0x0031  /* Driver Control Register (8-bits) */
#define R_SCI_B_DCR_HA_H_OFFSET             0x0032  /* Driver Control Register (16-bits) */
#define R_SCI_B_DCR_BY_HL_OFFSET            0x0032  /* Driver Control Register (8-bits) */
#define R_SCI_B_XCR0_OFFSET                 0x0034  /* Simple-LIN(SCIX) Control Register 0 (32-bits) */
#define R_SCI_B_XCR0_HA_L_OFFSET            0x0034  /* Simple-LIN(SCIX) Control Register 0 (16-bits) */
#define R_SCI_B_XCR0_BY_LL_OFFSET           0x0034  /* Simple-LIN(SCIX) Control Register 0 (8-bits) */
#define R_SCI_B_XCR0_BY_LH_OFFSET           0x0035  /* Simple-LIN(SCIX) Control Register 0 (8-bits) */
#define R_SCI_B_XCR0_HA_H_OFFSET            0x0036  /* Simple-LIN(SCIX) Control Register 0 (16-bits) */
#define R_SCI_B_XCR0_BY_HL_OFFSET           0x0036  /* Simple-LIN(SCIX) Control Register 0 (8-bits) */
#define R_SCI_B_XCR0_BY_HH_OFFSET           0x0037  /* Simple-LIN(SCIX) Control Register 0 (8-bits) */
#define R_SCI_B_XCR1_OFFSET                 0x0038  /* Simple-LIN(SCIX) Control Register 1 (32-bits) */
#define R_SCI_B_XCR1_HA_L_OFFSET            0x0038  /* Simple-LIN(SCIX) Control Register 1 (16-bits) */
#define R_SCI_B_XCR1_BY_LL_OFFSET           0x0038  /* Simple-LIN(SCIX) Control Register 1 (8-bits) */
#define R_SCI_B_XCR1_BY_LH_OFFSET           0x0039  /* Simple-LIN(SCIX) Control Register 1 (8-bits) */
#define R_SCI_B_XCR1_HA_H_OFFSET            0x003a  /* Simple-LIN(SCIX) Control Register 1 (16-bits) */
#define R_SCI_B_XCR1_BY_HL_OFFSET           0x003a  /* Simple-LIN(SCIX) Control Register 1 (8-bits) */
#define R_SCI_B_XCR1_BY_HH_OFFSET           0x003b  /* Simple-LIN(SCIX) Control Register 1 (8-bits) */
#define R_SCI_B_XCR2_OFFSET                 0x003c  /* Simple-LIN(SCIX) Control Register 2 (32-bits) */
#define R_SCI_B_XCR2_HA_L_OFFSET            0x003c  /* Simple-LIN(SCIX) Control Register 2 (16-bits) */
#define R_SCI_B_XCR2_BY_LL_OFFSET           0x003c  /* Simple-LIN(SCIX) Control Register 2 (8-bits) */
#define R_SCI_B_XCR2_BY_LH_OFFSET           0x003d  /* Simple-LIN(SCIX) Control Register 2 (8-bits) */
#define R_SCI_B_XCR2_HA_H_OFFSET            0x003e  /* Simple-LIN(SCIX) Control Register 2 (16-bits) */
#define R_SCI_B_XCR2_BY_HL_OFFSET           0x003e  /* Simple-LIN(SCIX) Control Register 2 (8-bits) */
#define R_SCI_B_XCR2_BY_HH_OFFSET           0x003f  /* Simple-LIN(SCIX) Control Register 2 (8-bits) */
#define R_SCI_B_CSR_OFFSET                  0x0048  /* Common Status Register (32-bits) */
#define R_SCI_B_ISR_OFFSET                  0x004c  /* Simple-I2C Status Register (32-bits) */
#define R_SCI_B_FRSR_OFFSET                 0x0050  /* FIFO Receive Status Register (32-bits) */
#define R_SCI_B_FTSR_OFFSET                 0x0054  /* FIFO Transmit Status Register (32-bits) */
#define R_SCI_B_MSR_OFFSET                  0x0058  /* Manchester Status Register (32-bits) */
#define R_SCI_B_XSR0_OFFSET                 0x005c  /* Simple-LIN(SCIX) Status Register 0 (32-bits) */
#define R_SCI_B_XSR1_OFFSET                 0x0060  /* Simple-LIN(SCIX) Status Register 1 (32-bits) */
#define R_SCI_B_CFCLR_OFFSET                0x0068  /* Common Flag Clear Register (32-bits) */
#define R_SCI_B_CFCLR_HA_L_OFFSET           0x0068  /* Common Flag Clear Register (16-bits) */
#define R_SCI_B_CFCLR_BY_LL_OFFSET          0x0068  /* Common Flag Clear Register (8-bits) */
#define R_SCI_B_CFCLR_HA_H_OFFSET           0x006a  /* Common Flag Clear Register (16-bits) */
#define R_SCI_B_CFCLR_BY_HL_OFFSET          0x006a  /* Common Flag Clear Register (8-bits) */
#define R_SCI_B_CFCLR_BY_HH_OFFSET          0x006b  /* Common Flag Clear Register (8-bits) */
#define R_SCI_B_ICFCLR_OFFSET               0x006c  /* Simple-I2C Flag Clear Register (32-bits) */
#define R_SCI_B_ICFCLR_HA_L_OFFSET          0x006c  /* Simple-I2C Flag Clear Register (16-bits) */
#define R_SCI_B_ICFCLR_BY_LL_OFFSET         0x006c  /* Simple-I2C Flag Clear Register (8-bits) */
#define R_SCI_B_FFCLR_OFFSET                0x0070  /* FIFO Flag Clear Register (32-bits) */
#define R_SCI_B_FFCLR_HA_L_OFFSET           0x0070  /* FIFO Flag Clear Register (16-bits) */
#define R_SCI_B_FFCLR_BY_LL_OFFSET          0x0070  /* FIFO Flag Clear Register (8-bits) */
#define R_SCI_B_MFCLR_OFFSET                0x0074  /* Manchester Flag Clear Register (32-bits) */
#define R_SCI_B_MFCLR_HA_L_OFFSET           0x0074  /* Manchester Flag Clear Register (16-bits) */
#define R_SCI_B_MFCLR_BY_LL_OFFSET          0x0074  /* Manchester Flag Clear Register (8-bits) */
#define R_SCI_B_XFCLR_OFFSET                0x0078  /* Simpe-LIN(SCIX) Flag Clear Register (32-bits) */
#define R_SCI_B_XFCLR_HA_L_OFFSET           0x0078  /* Simpe-LIN(SCIX) Flag Clear Register (16-bits) */
#define R_SCI_B_XFCLR_BY_LH_OFFSET          0x0079  /* Simpe-LIN(SCIX) Flag Clear Register (8-bits) */

/* Register Addresses *******************************************************/

/* SCI_B0 Registers */

#define R_SCI_B0_RDR                       (R_SCI_B0_BASE + R_SCI_B_RDR_OFFSET)
#define R_SCI_B0_TDR                       (R_SCI_B0_BASE + R_SCI_B_TDR_OFFSET)
#define R_SCI_B0_TDR_HA_L                  (R_SCI_B0_BASE + R_SCI_B_TDR_HA_L_OFFSET)
#define R_SCI_B0_TDR_BY_LL                 (R_SCI_B0_BASE + R_SCI_B_TDR_BY_LL_OFFSET)
#define R_SCI_B0_TDR_BY_LH                 (R_SCI_B0_BASE + R_SCI_B_TDR_BY_LH_OFFSET)
#define R_SCI_B0_CCR0                      (R_SCI_B0_BASE + R_SCI_B_CCR0_OFFSET)
#define R_SCI_B0_CCR0_HA_L                 (R_SCI_B0_BASE + R_SCI_B_CCR0_HA_L_OFFSET)
#define R_SCI_B0_CCR0_BY_LL                (R_SCI_B0_BASE + R_SCI_B_CCR0_BY_LL_OFFSET)
#define R_SCI_B0_CCR0_BY_LH                (R_SCI_B0_BASE + R_SCI_B_CCR0_BY_LH_OFFSET)
#define R_SCI_B0_CCR0_HA_H                 (R_SCI_B0_BASE + R_SCI_B_CCR0_HA_H_OFFSET)
#define R_SCI_B0_CCR0_BY_HL                (R_SCI_B0_BASE + R_SCI_B_CCR0_BY_HL_OFFSET)
#define R_SCI_B0_CCR0_BY_HH                (R_SCI_B0_BASE + R_SCI_B_CCR0_BY_HH_OFFSET)
#define R_SCI_B0_CCR1                      (R_SCI_B0_BASE + R_SCI_B_CCR1_OFFSET)
#define R_SCI_B0_CCR1_HA_L                 (R_SCI_B0_BASE + R_SCI_B_CCR1_HA_L_OFFSET)
#define R_SCI_B0_CCR1_BY_LL                (R_SCI_B0_BASE + R_SCI_B_CCR1_BY_LL_OFFSET)
#define R_SCI_B0_CCR1_BY_LH                (R_SCI_B0_BASE + R_SCI_B_CCR1_BY_LH_OFFSET)
#define R_SCI_B0_CCR1_HA_H                 (R_SCI_B0_BASE + R_SCI_B_CCR1_HA_H_OFFSET)
#define R_SCI_B0_CCR1_BY_HL                (R_SCI_B0_BASE + R_SCI_B_CCR1_BY_HL_OFFSET)
#define R_SCI_B0_CCR1_BY_HH                (R_SCI_B0_BASE + R_SCI_B_CCR1_BY_HH_OFFSET)
#define R_SCI_B0_CCR2                      (R_SCI_B0_BASE + R_SCI_B_CCR2_OFFSET)
#define R_SCI_B0_CCR2_HA_L                 (R_SCI_B0_BASE + R_SCI_B_CCR2_HA_L_OFFSET)
#define R_SCI_B0_CCR2_BY_LL                (R_SCI_B0_BASE + R_SCI_B_CCR2_BY_LL_OFFSET)
#define R_SCI_B0_CCR2_BY_LH                (R_SCI_B0_BASE + R_SCI_B_CCR2_BY_LH_OFFSET)
#define R_SCI_B0_CCR2_HA_H                 (R_SCI_B0_BASE + R_SCI_B_CCR2_HA_H_OFFSET)
#define R_SCI_B0_CCR2_BY_HL                (R_SCI_B0_BASE + R_SCI_B_CCR2_BY_HL_OFFSET)
#define R_SCI_B0_CCR2_BY_HH                (R_SCI_B0_BASE + R_SCI_B_CCR2_BY_HH_OFFSET)
#define R_SCI_B0_CCR3                      (R_SCI_B0_BASE + R_SCI_B_CCR3_OFFSET)
#define R_SCI_B0_CCR3_HA_L                 (R_SCI_B0_BASE + R_SCI_B_CCR3_HA_L_OFFSET)
#define R_SCI_B0_CCR3_BY_LL                (R_SCI_B0_BASE + R_SCI_B_CCR3_BY_LL_OFFSET)
#define R_SCI_B0_CCR3_BY_LH                (R_SCI_B0_BASE + R_SCI_B_CCR3_BY_LH_OFFSET)
#define R_SCI_B0_CCR3_HA_H                 (R_SCI_B0_BASE + R_SCI_B_CCR3_HA_H_OFFSET)
#define R_SCI_B0_CCR3_BY_HL                (R_SCI_B0_BASE + R_SCI_B_CCR3_BY_HL_OFFSET)
#define R_SCI_B0_CCR3_BY_HH                (R_SCI_B0_BASE + R_SCI_B_CCR3_BY_HH_OFFSET)
#define R_SCI_B0_CCR4                      (R_SCI_B0_BASE + R_SCI_B_CCR4_OFFSET)
#define R_SCI_B0_CCR4_HA_L                 (R_SCI_B0_BASE + R_SCI_B_CCR4_HA_L_OFFSET)
#define R_SCI_B0_CCR4_BY_LL                (R_SCI_B0_BASE + R_SCI_B_CCR4_BY_LL_OFFSET)
#define R_SCI_B0_CCR4_BY_LH                (R_SCI_B0_BASE + R_SCI_B_CCR4_BY_LH_OFFSET)
#define R_SCI_B0_CCR4_HA_H                 (R_SCI_B0_BASE + R_SCI_B_CCR4_HA_H_OFFSET)
#define R_SCI_B0_CCR4_BY_HL                (R_SCI_B0_BASE + R_SCI_B_CCR4_BY_HL_OFFSET)
#define R_SCI_B0_CCR4_BY_HH                (R_SCI_B0_BASE + R_SCI_B_CCR4_BY_HH_OFFSET)
#define R_SCI_B0_CESR                      (R_SCI_B0_BASE + R_SCI_B_CESR_OFFSET)
#define R_SCI_B0_HCR                       (R_SCI_B0_BASE + R_SCI_B_HCR_OFFSET)
#define R_SCI_B0_ICR                       (R_SCI_B0_BASE + R_SCI_B_ICR_OFFSET)
#define R_SCI_B0_ICR_HA_L                  (R_SCI_B0_BASE + R_SCI_B_ICR_HA_L_OFFSET)
#define R_SCI_B0_ICR_BY_LL                 (R_SCI_B0_BASE + R_SCI_B_ICR_BY_LL_OFFSET)
#define R_SCI_B0_ICR_BY_LH                 (R_SCI_B0_BASE + R_SCI_B_ICR_BY_LH_OFFSET)
#define R_SCI_B0_ICR_HA_H                  (R_SCI_B0_BASE + R_SCI_B_ICR_HA_H_OFFSET)
#define R_SCI_B0_ICR_BY_HL                 (R_SCI_B0_BASE + R_SCI_B_ICR_BY_HL_OFFSET)
#define R_SCI_B0_FCR                       (R_SCI_B0_BASE + R_SCI_B_FCR_OFFSET)
#define R_SCI_B0_FCR_HA_L                  (R_SCI_B0_BASE + R_SCI_B_FCR_HA_L_OFFSET)
#define R_SCI_B0_FCR_BY_LL                 (R_SCI_B0_BASE + R_SCI_B_FCR_BY_LL_OFFSET)
#define R_SCI_B0_FCR_BY_LH                 (R_SCI_B0_BASE + R_SCI_B_FCR_BY_LH_OFFSET)
#define R_SCI_B0_FCR_HA_H                  (R_SCI_B0_BASE + R_SCI_B_FCR_HA_H_OFFSET)
#define R_SCI_B0_FCR_BY_HL                 (R_SCI_B0_BASE + R_SCI_B_FCR_BY_HL_OFFSET)
#define R_SCI_B0_FCR_BY_HH                 (R_SCI_B0_BASE + R_SCI_B_FCR_BY_HH_OFFSET)
#define R_SCI_B0_MCR                       (R_SCI_B0_BASE + R_SCI_B_MCR_OFFSET)
#define R_SCI_B0_MCR_HA_L                  (R_SCI_B0_BASE + R_SCI_B_MCR_HA_L_OFFSET)
#define R_SCI_B0_MCR_BY_LL                 (R_SCI_B0_BASE + R_SCI_B_MCR_BY_LL_OFFSET)
#define R_SCI_B0_MCR_BY_LH                 (R_SCI_B0_BASE + R_SCI_B_MCR_BY_LH_OFFSET)
#define R_SCI_B0_MCR_HA_H                  (R_SCI_B0_BASE + R_SCI_B_MCR_HA_H_OFFSET)
#define R_SCI_B0_MCR_BY_HL                 (R_SCI_B0_BASE + R_SCI_B_MCR_BY_HL_OFFSET)
#define R_SCI_B0_MCR_BY_HH                 (R_SCI_B0_BASE + R_SCI_B_MCR_BY_HH_OFFSET)
#define R_SCI_B0_DCR                       (R_SCI_B0_BASE + R_SCI_B_DCR_OFFSET)
#define R_SCI_B0_DCR_HA_L                  (R_SCI_B0_BASE + R_SCI_B_DCR_HA_L_OFFSET)
#define R_SCI_B0_DCR_BY_LL                 (R_SCI_B0_BASE + R_SCI_B_DCR_BY_LL_OFFSET)
#define R_SCI_B0_DCR_BY_LH                 (R_SCI_B0_BASE + R_SCI_B_DCR_BY_LH_OFFSET)
#define R_SCI_B0_DCR_HA_H                  (R_SCI_B0_BASE + R_SCI_B_DCR_HA_H_OFFSET)
#define R_SCI_B0_DCR_BY_HL                 (R_SCI_B0_BASE + R_SCI_B_DCR_BY_HL_OFFSET)
#define R_SCI_B0_XCR0                      (R_SCI_B0_BASE + R_SCI_B_XCR0_OFFSET)
#define R_SCI_B0_XCR0_HA_L                 (R_SCI_B0_BASE + R_SCI_B_XCR0_HA_L_OFFSET)
#define R_SCI_B0_XCR0_BY_LL                (R_SCI_B0_BASE + R_SCI_B_XCR0_BY_LL_OFFSET)
#define R_SCI_B0_XCR0_BY_LH                (R_SCI_B0_BASE + R_SCI_B_XCR0_BY_LH_OFFSET)
#define R_SCI_B0_XCR0_HA_H                 (R_SCI_B0_BASE + R_SCI_B_XCR0_HA_H_OFFSET)
#define R_SCI_B0_XCR0_BY_HL                (R_SCI_B0_BASE + R_SCI_B_XCR0_BY_HL_OFFSET)
#define R_SCI_B0_XCR0_BY_HH                (R_SCI_B0_BASE + R_SCI_B_XCR0_BY_HH_OFFSET)
#define R_SCI_B0_XCR1                      (R_SCI_B0_BASE + R_SCI_B_XCR1_OFFSET)
#define R_SCI_B0_XCR1_HA_L                 (R_SCI_B0_BASE + R_SCI_B_XCR1_HA_L_OFFSET)
#define R_SCI_B0_XCR1_BY_LL                (R_SCI_B0_BASE + R_SCI_B_XCR1_BY_LL_OFFSET)
#define R_SCI_B0_XCR1_BY_LH                (R_SCI_B0_BASE + R_SCI_B_XCR1_BY_LH_OFFSET)
#define R_SCI_B0_XCR1_HA_H                 (R_SCI_B0_BASE + R_SCI_B_XCR1_HA_H_OFFSET)
#define R_SCI_B0_XCR1_BY_HL                (R_SCI_B0_BASE + R_SCI_B_XCR1_BY_HL_OFFSET)
#define R_SCI_B0_XCR1_BY_HH                (R_SCI_B0_BASE + R_SCI_B_XCR1_BY_HH_OFFSET)
#define R_SCI_B0_XCR2                      (R_SCI_B0_BASE + R_SCI_B_XCR2_OFFSET)
#define R_SCI_B0_XCR2_HA_L                 (R_SCI_B0_BASE + R_SCI_B_XCR2_HA_L_OFFSET)
#define R_SCI_B0_XCR2_BY_LL                (R_SCI_B0_BASE + R_SCI_B_XCR2_BY_LL_OFFSET)
#define R_SCI_B0_XCR2_BY_LH                (R_SCI_B0_BASE + R_SCI_B_XCR2_BY_LH_OFFSET)
#define R_SCI_B0_XCR2_HA_H                 (R_SCI_B0_BASE + R_SCI_B_XCR2_HA_H_OFFSET)
#define R_SCI_B0_XCR2_BY_HL                (R_SCI_B0_BASE + R_SCI_B_XCR2_BY_HL_OFFSET)
#define R_SCI_B0_XCR2_BY_HH                (R_SCI_B0_BASE + R_SCI_B_XCR2_BY_HH_OFFSET)
#define R_SCI_B0_CSR                       (R_SCI_B0_BASE + R_SCI_B_CSR_OFFSET)
#define R_SCI_B0_ISR                       (R_SCI_B0_BASE + R_SCI_B_ISR_OFFSET)
#define R_SCI_B0_FRSR                      (R_SCI_B0_BASE + R_SCI_B_FRSR_OFFSET)
#define R_SCI_B0_FTSR                      (R_SCI_B0_BASE + R_SCI_B_FTSR_OFFSET)
#define R_SCI_B0_MSR                       (R_SCI_B0_BASE + R_SCI_B_MSR_OFFSET)
#define R_SCI_B0_XSR0                      (R_SCI_B0_BASE + R_SCI_B_XSR0_OFFSET)
#define R_SCI_B0_XSR1                      (R_SCI_B0_BASE + R_SCI_B_XSR1_OFFSET)
#define R_SCI_B0_CFCLR                     (R_SCI_B0_BASE + R_SCI_B_CFCLR_OFFSET)
#define R_SCI_B0_CFCLR_HA_L                (R_SCI_B0_BASE + R_SCI_B_CFCLR_HA_L_OFFSET)
#define R_SCI_B0_CFCLR_BY_LL               (R_SCI_B0_BASE + R_SCI_B_CFCLR_BY_LL_OFFSET)
#define R_SCI_B0_CFCLR_HA_H                (R_SCI_B0_BASE + R_SCI_B_CFCLR_HA_H_OFFSET)
#define R_SCI_B0_CFCLR_BY_HL               (R_SCI_B0_BASE + R_SCI_B_CFCLR_BY_HL_OFFSET)
#define R_SCI_B0_CFCLR_BY_HH               (R_SCI_B0_BASE + R_SCI_B_CFCLR_BY_HH_OFFSET)
#define R_SCI_B0_ICFCLR                    (R_SCI_B0_BASE + R_SCI_B_ICFCLR_OFFSET)
#define R_SCI_B0_ICFCLR_HA_L               (R_SCI_B0_BASE + R_SCI_B_ICFCLR_HA_L_OFFSET)
#define R_SCI_B0_ICFCLR_BY_LL              (R_SCI_B0_BASE + R_SCI_B_ICFCLR_BY_LL_OFFSET)
#define R_SCI_B0_FFCLR                     (R_SCI_B0_BASE + R_SCI_B_FFCLR_OFFSET)
#define R_SCI_B0_FFCLR_HA_L                (R_SCI_B0_BASE + R_SCI_B_FFCLR_HA_L_OFFSET)
#define R_SCI_B0_FFCLR_BY_LL               (R_SCI_B0_BASE + R_SCI_B_FFCLR_BY_LL_OFFSET)
#define R_SCI_B0_MFCLR                     (R_SCI_B0_BASE + R_SCI_B_MFCLR_OFFSET)
#define R_SCI_B0_MFCLR_HA_L                (R_SCI_B0_BASE + R_SCI_B_MFCLR_HA_L_OFFSET)
#define R_SCI_B0_MFCLR_BY_LL               (R_SCI_B0_BASE + R_SCI_B_MFCLR_BY_LL_OFFSET)
#define R_SCI_B0_XFCLR                     (R_SCI_B0_BASE + R_SCI_B_XFCLR_OFFSET)
#define R_SCI_B0_XFCLR_HA_L                (R_SCI_B0_BASE + R_SCI_B_XFCLR_HA_L_OFFSET)
#define R_SCI_B0_XFCLR_BY_LH               (R_SCI_B0_BASE + R_SCI_B_XFCLR_BY_LH_OFFSET)

/* SCI_B1 Registers */

#define R_SCI_B1_RDR                       (R_SCI_B1_BASE + R_SCI_B_RDR_OFFSET)
#define R_SCI_B1_TDR                       (R_SCI_B1_BASE + R_SCI_B_TDR_OFFSET)
#define R_SCI_B1_TDR_HA_L                  (R_SCI_B1_BASE + R_SCI_B_TDR_HA_L_OFFSET)
#define R_SCI_B1_TDR_BY_LL                 (R_SCI_B1_BASE + R_SCI_B_TDR_BY_LL_OFFSET)
#define R_SCI_B1_TDR_BY_LH                 (R_SCI_B1_BASE + R_SCI_B_TDR_BY_LH_OFFSET)
#define R_SCI_B1_CCR0                      (R_SCI_B1_BASE + R_SCI_B_CCR0_OFFSET)
#define R_SCI_B1_CCR0_HA_L                 (R_SCI_B1_BASE + R_SCI_B_CCR0_HA_L_OFFSET)
#define R_SCI_B1_CCR0_BY_LL                (R_SCI_B1_BASE + R_SCI_B_CCR0_BY_LL_OFFSET)
#define R_SCI_B1_CCR0_BY_LH                (R_SCI_B1_BASE + R_SCI_B_CCR0_BY_LH_OFFSET)
#define R_SCI_B1_CCR0_HA_H                 (R_SCI_B1_BASE + R_SCI_B_CCR0_HA_H_OFFSET)
#define R_SCI_B1_CCR0_BY_HL                (R_SCI_B1_BASE + R_SCI_B_CCR0_BY_HL_OFFSET)
#define R_SCI_B1_CCR0_BY_HH                (R_SCI_B1_BASE + R_SCI_B_CCR0_BY_HH_OFFSET)
#define R_SCI_B1_CCR1                      (R_SCI_B1_BASE + R_SCI_B_CCR1_OFFSET)
#define R_SCI_B1_CCR1_HA_L                 (R_SCI_B1_BASE + R_SCI_B_CCR1_HA_L_OFFSET)
#define R_SCI_B1_CCR1_BY_LL                (R_SCI_B1_BASE + R_SCI_B_CCR1_BY_LL_OFFSET)
#define R_SCI_B1_CCR1_BY_LH                (R_SCI_B1_BASE + R_SCI_B_CCR1_BY_LH_OFFSET)
#define R_SCI_B1_CCR1_HA_H                 (R_SCI_B1_BASE + R_SCI_B_CCR1_HA_H_OFFSET)
#define R_SCI_B1_CCR1_BY_HL                (R_SCI_B1_BASE + R_SCI_B_CCR1_BY_HL_OFFSET)
#define R_SCI_B1_CCR1_BY_HH                (R_SCI_B1_BASE + R_SCI_B_CCR1_BY_HH_OFFSET)
#define R_SCI_B1_CCR2                      (R_SCI_B1_BASE + R_SCI_B_CCR2_OFFSET)
#define R_SCI_B1_CCR2_HA_L                 (R_SCI_B1_BASE + R_SCI_B_CCR2_HA_L_OFFSET)
#define R_SCI_B1_CCR2_BY_LL                (R_SCI_B1_BASE + R_SCI_B_CCR2_BY_LL_OFFSET)
#define R_SCI_B1_CCR2_BY_LH                (R_SCI_B1_BASE + R_SCI_B_CCR2_BY_LH_OFFSET)
#define R_SCI_B1_CCR2_HA_H                 (R_SCI_B1_BASE + R_SCI_B_CCR2_HA_H_OFFSET)
#define R_SCI_B1_CCR2_BY_HL                (R_SCI_B1_BASE + R_SCI_B_CCR2_BY_HL_OFFSET)
#define R_SCI_B1_CCR2_BY_HH                (R_SCI_B1_BASE + R_SCI_B_CCR2_BY_HH_OFFSET)
#define R_SCI_B1_CCR3                      (R_SCI_B1_BASE + R_SCI_B_CCR3_OFFSET)
#define R_SCI_B1_CCR3_HA_L                 (R_SCI_B1_BASE + R_SCI_B_CCR3_HA_L_OFFSET)
#define R_SCI_B1_CCR3_BY_LL                (R_SCI_B1_BASE + R_SCI_B_CCR3_BY_LL_OFFSET)
#define R_SCI_B1_CCR3_BY_LH                (R_SCI_B1_BASE + R_SCI_B_CCR3_BY_LH_OFFSET)
#define R_SCI_B1_CCR3_HA_H                 (R_SCI_B1_BASE + R_SCI_B_CCR3_HA_H_OFFSET)
#define R_SCI_B1_CCR3_BY_HL                (R_SCI_B1_BASE + R_SCI_B_CCR3_BY_HL_OFFSET)
#define R_SCI_B1_CCR3_BY_HH                (R_SCI_B1_BASE + R_SCI_B_CCR3_BY_HH_OFFSET)
#define R_SCI_B1_CCR4                      (R_SCI_B1_BASE + R_SCI_B_CCR4_OFFSET)
#define R_SCI_B1_CCR4_HA_L                 (R_SCI_B1_BASE + R_SCI_B_CCR4_HA_L_OFFSET)
#define R_SCI_B1_CCR4_BY_LL                (R_SCI_B1_BASE + R_SCI_B_CCR4_BY_LL_OFFSET)
#define R_SCI_B1_CCR4_BY_LH                (R_SCI_B1_BASE + R_SCI_B_CCR4_BY_LH_OFFSET)
#define R_SCI_B1_CCR4_HA_H                 (R_SCI_B1_BASE + R_SCI_B_CCR4_HA_H_OFFSET)
#define R_SCI_B1_CCR4_BY_HL                (R_SCI_B1_BASE + R_SCI_B_CCR4_BY_HL_OFFSET)
#define R_SCI_B1_CCR4_BY_HH                (R_SCI_B1_BASE + R_SCI_B_CCR4_BY_HH_OFFSET)
#define R_SCI_B1_CESR                      (R_SCI_B1_BASE + R_SCI_B_CESR_OFFSET)
#define R_SCI_B1_HCR                       (R_SCI_B1_BASE + R_SCI_B_HCR_OFFSET)
#define R_SCI_B1_ICR                       (R_SCI_B1_BASE + R_SCI_B_ICR_OFFSET)
#define R_SCI_B1_ICR_HA_L                  (R_SCI_B1_BASE + R_SCI_B_ICR_HA_L_OFFSET)
#define R_SCI_B1_ICR_BY_LL                 (R_SCI_B1_BASE + R_SCI_B_ICR_BY_LL_OFFSET)
#define R_SCI_B1_ICR_BY_LH                 (R_SCI_B1_BASE + R_SCI_B_ICR_BY_LH_OFFSET)
#define R_SCI_B1_ICR_HA_H                  (R_SCI_B1_BASE + R_SCI_B_ICR_HA_H_OFFSET)
#define R_SCI_B1_ICR_BY_HL                 (R_SCI_B1_BASE + R_SCI_B_ICR_BY_HL_OFFSET)
#define R_SCI_B1_FCR                       (R_SCI_B1_BASE + R_SCI_B_FCR_OFFSET)
#define R_SCI_B1_FCR_HA_L                  (R_SCI_B1_BASE + R_SCI_B_FCR_HA_L_OFFSET)
#define R_SCI_B1_FCR_BY_LL                 (R_SCI_B1_BASE + R_SCI_B_FCR_BY_LL_OFFSET)
#define R_SCI_B1_FCR_BY_LH                 (R_SCI_B1_BASE + R_SCI_B_FCR_BY_LH_OFFSET)
#define R_SCI_B1_FCR_HA_H                  (R_SCI_B1_BASE + R_SCI_B_FCR_HA_H_OFFSET)
#define R_SCI_B1_FCR_BY_HL                 (R_SCI_B1_BASE + R_SCI_B_FCR_BY_HL_OFFSET)
#define R_SCI_B1_FCR_BY_HH                 (R_SCI_B1_BASE + R_SCI_B_FCR_BY_HH_OFFSET)
#define R_SCI_B1_MCR                       (R_SCI_B1_BASE + R_SCI_B_MCR_OFFSET)
#define R_SCI_B1_MCR_HA_L                  (R_SCI_B1_BASE + R_SCI_B_MCR_HA_L_OFFSET)
#define R_SCI_B1_MCR_BY_LL                 (R_SCI_B1_BASE + R_SCI_B_MCR_BY_LL_OFFSET)
#define R_SCI_B1_MCR_BY_LH                 (R_SCI_B1_BASE + R_SCI_B_MCR_BY_LH_OFFSET)
#define R_SCI_B1_MCR_HA_H                  (R_SCI_B1_BASE + R_SCI_B_MCR_HA_H_OFFSET)
#define R_SCI_B1_MCR_BY_HL                 (R_SCI_B1_BASE + R_SCI_B_MCR_BY_HL_OFFSET)
#define R_SCI_B1_MCR_BY_HH                 (R_SCI_B1_BASE + R_SCI_B_MCR_BY_HH_OFFSET)
#define R_SCI_B1_DCR                       (R_SCI_B1_BASE + R_SCI_B_DCR_OFFSET)
#define R_SCI_B1_DCR_HA_L                  (R_SCI_B1_BASE + R_SCI_B_DCR_HA_L_OFFSET)
#define R_SCI_B1_DCR_BY_LL                 (R_SCI_B1_BASE + R_SCI_B_DCR_BY_LL_OFFSET)
#define R_SCI_B1_DCR_BY_LH                 (R_SCI_B1_BASE + R_SCI_B_DCR_BY_LH_OFFSET)
#define R_SCI_B1_DCR_HA_H                  (R_SCI_B1_BASE + R_SCI_B_DCR_HA_H_OFFSET)
#define R_SCI_B1_DCR_BY_HL                 (R_SCI_B1_BASE + R_SCI_B_DCR_BY_HL_OFFSET)
#define R_SCI_B1_XCR0                      (R_SCI_B1_BASE + R_SCI_B_XCR0_OFFSET)
#define R_SCI_B1_XCR0_HA_L                 (R_SCI_B1_BASE + R_SCI_B_XCR0_HA_L_OFFSET)
#define R_SCI_B1_XCR0_BY_LL                (R_SCI_B1_BASE + R_SCI_B_XCR0_BY_LL_OFFSET)
#define R_SCI_B1_XCR0_BY_LH                (R_SCI_B1_BASE + R_SCI_B_XCR0_BY_LH_OFFSET)
#define R_SCI_B1_XCR0_HA_H                 (R_SCI_B1_BASE + R_SCI_B_XCR0_HA_H_OFFSET)
#define R_SCI_B1_XCR0_BY_HL                (R_SCI_B1_BASE + R_SCI_B_XCR0_BY_HL_OFFSET)
#define R_SCI_B1_XCR0_BY_HH                (R_SCI_B1_BASE + R_SCI_B_XCR0_BY_HH_OFFSET)
#define R_SCI_B1_XCR1                      (R_SCI_B1_BASE + R_SCI_B_XCR1_OFFSET)
#define R_SCI_B1_XCR1_HA_L                 (R_SCI_B1_BASE + R_SCI_B_XCR1_HA_L_OFFSET)
#define R_SCI_B1_XCR1_BY_LL                (R_SCI_B1_BASE + R_SCI_B_XCR1_BY_LL_OFFSET)
#define R_SCI_B1_XCR1_BY_LH                (R_SCI_B1_BASE + R_SCI_B_XCR1_BY_LH_OFFSET)
#define R_SCI_B1_XCR1_HA_H                 (R_SCI_B1_BASE + R_SCI_B_XCR1_HA_H_OFFSET)
#define R_SCI_B1_XCR1_BY_HL                (R_SCI_B1_BASE + R_SCI_B_XCR1_BY_HL_OFFSET)
#define R_SCI_B1_XCR1_BY_HH                (R_SCI_B1_BASE + R_SCI_B_XCR1_BY_HH_OFFSET)
#define R_SCI_B1_XCR2                      (R_SCI_B1_BASE + R_SCI_B_XCR2_OFFSET)
#define R_SCI_B1_XCR2_HA_L                 (R_SCI_B1_BASE + R_SCI_B_XCR2_HA_L_OFFSET)
#define R_SCI_B1_XCR2_BY_LL                (R_SCI_B1_BASE + R_SCI_B_XCR2_BY_LL_OFFSET)
#define R_SCI_B1_XCR2_BY_LH                (R_SCI_B1_BASE + R_SCI_B_XCR2_BY_LH_OFFSET)
#define R_SCI_B1_XCR2_HA_H                 (R_SCI_B1_BASE + R_SCI_B_XCR2_HA_H_OFFSET)
#define R_SCI_B1_XCR2_BY_HL                (R_SCI_B1_BASE + R_SCI_B_XCR2_BY_HL_OFFSET)
#define R_SCI_B1_XCR2_BY_HH                (R_SCI_B1_BASE + R_SCI_B_XCR2_BY_HH_OFFSET)
#define R_SCI_B1_CSR                       (R_SCI_B1_BASE + R_SCI_B_CSR_OFFSET)
#define R_SCI_B1_ISR                       (R_SCI_B1_BASE + R_SCI_B_ISR_OFFSET)
#define R_SCI_B1_FRSR                      (R_SCI_B1_BASE + R_SCI_B_FRSR_OFFSET)
#define R_SCI_B1_FTSR                      (R_SCI_B1_BASE + R_SCI_B_FTSR_OFFSET)
#define R_SCI_B1_MSR                       (R_SCI_B1_BASE + R_SCI_B_MSR_OFFSET)
#define R_SCI_B1_XSR0                      (R_SCI_B1_BASE + R_SCI_B_XSR0_OFFSET)
#define R_SCI_B1_XSR1                      (R_SCI_B1_BASE + R_SCI_B_XSR1_OFFSET)
#define R_SCI_B1_CFCLR                     (R_SCI_B1_BASE + R_SCI_B_CFCLR_OFFSET)
#define R_SCI_B1_CFCLR_HA_L                (R_SCI_B1_BASE + R_SCI_B_CFCLR_HA_L_OFFSET)
#define R_SCI_B1_CFCLR_BY_LL               (R_SCI_B1_BASE + R_SCI_B_CFCLR_BY_LL_OFFSET)
#define R_SCI_B1_CFCLR_HA_H                (R_SCI_B1_BASE + R_SCI_B_CFCLR_HA_H_OFFSET)
#define R_SCI_B1_CFCLR_BY_HL               (R_SCI_B1_BASE + R_SCI_B_CFCLR_BY_HL_OFFSET)
#define R_SCI_B1_CFCLR_BY_HH               (R_SCI_B1_BASE + R_SCI_B_CFCLR_BY_HH_OFFSET)
#define R_SCI_B1_ICFCLR                    (R_SCI_B1_BASE + R_SCI_B_ICFCLR_OFFSET)
#define R_SCI_B1_ICFCLR_HA_L               (R_SCI_B1_BASE + R_SCI_B_ICFCLR_HA_L_OFFSET)
#define R_SCI_B1_ICFCLR_BY_LL              (R_SCI_B1_BASE + R_SCI_B_ICFCLR_BY_LL_OFFSET)
#define R_SCI_B1_FFCLR                     (R_SCI_B1_BASE + R_SCI_B_FFCLR_OFFSET)
#define R_SCI_B1_FFCLR_HA_L                (R_SCI_B1_BASE + R_SCI_B_FFCLR_HA_L_OFFSET)
#define R_SCI_B1_FFCLR_BY_LL               (R_SCI_B1_BASE + R_SCI_B_FFCLR_BY_LL_OFFSET)
#define R_SCI_B1_MFCLR                     (R_SCI_B1_BASE + R_SCI_B_MFCLR_OFFSET)
#define R_SCI_B1_MFCLR_HA_L                (R_SCI_B1_BASE + R_SCI_B_MFCLR_HA_L_OFFSET)
#define R_SCI_B1_MFCLR_BY_LL               (R_SCI_B1_BASE + R_SCI_B_MFCLR_BY_LL_OFFSET)
#define R_SCI_B1_XFCLR                     (R_SCI_B1_BASE + R_SCI_B_XFCLR_OFFSET)
#define R_SCI_B1_XFCLR_HA_L                (R_SCI_B1_BASE + R_SCI_B_XFCLR_HA_L_OFFSET)
#define R_SCI_B1_XFCLR_BY_LH               (R_SCI_B1_BASE + R_SCI_B_XFCLR_BY_LH_OFFSET)

/* SCI_B2 Registers */

#define R_SCI_B2_RDR                       (R_SCI_B2_BASE + R_SCI_B_RDR_OFFSET)
#define R_SCI_B2_TDR                       (R_SCI_B2_BASE + R_SCI_B_TDR_OFFSET)
#define R_SCI_B2_TDR_HA_L                  (R_SCI_B2_BASE + R_SCI_B_TDR_HA_L_OFFSET)
#define R_SCI_B2_TDR_BY_LL                 (R_SCI_B2_BASE + R_SCI_B_TDR_BY_LL_OFFSET)
#define R_SCI_B2_TDR_BY_LH                 (R_SCI_B2_BASE + R_SCI_B_TDR_BY_LH_OFFSET)
#define R_SCI_B2_CCR0                      (R_SCI_B2_BASE + R_SCI_B_CCR0_OFFSET)
#define R_SCI_B2_CCR0_HA_L                 (R_SCI_B2_BASE + R_SCI_B_CCR0_HA_L_OFFSET)
#define R_SCI_B2_CCR0_BY_LL                (R_SCI_B2_BASE + R_SCI_B_CCR0_BY_LL_OFFSET)
#define R_SCI_B2_CCR0_BY_LH                (R_SCI_B2_BASE + R_SCI_B_CCR0_BY_LH_OFFSET)
#define R_SCI_B2_CCR0_HA_H                 (R_SCI_B2_BASE + R_SCI_B_CCR0_HA_H_OFFSET)
#define R_SCI_B2_CCR0_BY_HL                (R_SCI_B2_BASE + R_SCI_B_CCR0_BY_HL_OFFSET)
#define R_SCI_B2_CCR0_BY_HH                (R_SCI_B2_BASE + R_SCI_B_CCR0_BY_HH_OFFSET)
#define R_SCI_B2_CCR1                      (R_SCI_B2_BASE + R_SCI_B_CCR1_OFFSET)
#define R_SCI_B2_CCR1_HA_L                 (R_SCI_B2_BASE + R_SCI_B_CCR1_HA_L_OFFSET)
#define R_SCI_B2_CCR1_BY_LL                (R_SCI_B2_BASE + R_SCI_B_CCR1_BY_LL_OFFSET)
#define R_SCI_B2_CCR1_BY_LH                (R_SCI_B2_BASE + R_SCI_B_CCR1_BY_LH_OFFSET)
#define R_SCI_B2_CCR1_HA_H                 (R_SCI_B2_BASE + R_SCI_B_CCR1_HA_H_OFFSET)
#define R_SCI_B2_CCR1_BY_HL                (R_SCI_B2_BASE + R_SCI_B_CCR1_BY_HL_OFFSET)
#define R_SCI_B2_CCR1_BY_HH                (R_SCI_B2_BASE + R_SCI_B_CCR1_BY_HH_OFFSET)
#define R_SCI_B2_CCR2                      (R_SCI_B2_BASE + R_SCI_B_CCR2_OFFSET)
#define R_SCI_B2_CCR2_HA_L                 (R_SCI_B2_BASE + R_SCI_B_CCR2_HA_L_OFFSET)
#define R_SCI_B2_CCR2_BY_LL                (R_SCI_B2_BASE + R_SCI_B_CCR2_BY_LL_OFFSET)
#define R_SCI_B2_CCR2_BY_LH                (R_SCI_B2_BASE + R_SCI_B_CCR2_BY_LH_OFFSET)
#define R_SCI_B2_CCR2_HA_H                 (R_SCI_B2_BASE + R_SCI_B_CCR2_HA_H_OFFSET)
#define R_SCI_B2_CCR2_BY_HL                (R_SCI_B2_BASE + R_SCI_B_CCR2_BY_HL_OFFSET)
#define R_SCI_B2_CCR2_BY_HH                (R_SCI_B2_BASE + R_SCI_B_CCR2_BY_HH_OFFSET)
#define R_SCI_B2_CCR3                      (R_SCI_B2_BASE + R_SCI_B_CCR3_OFFSET)
#define R_SCI_B2_CCR3_HA_L                 (R_SCI_B2_BASE + R_SCI_B_CCR3_HA_L_OFFSET)
#define R_SCI_B2_CCR3_BY_LL                (R_SCI_B2_BASE + R_SCI_B_CCR3_BY_LL_OFFSET)
#define R_SCI_B2_CCR3_BY_LH                (R_SCI_B2_BASE + R_SCI_B_CCR3_BY_LH_OFFSET)
#define R_SCI_B2_CCR3_HA_H                 (R_SCI_B2_BASE + R_SCI_B_CCR3_HA_H_OFFSET)
#define R_SCI_B2_CCR3_BY_HL                (R_SCI_B2_BASE + R_SCI_B_CCR3_BY_HL_OFFSET)
#define R_SCI_B2_CCR3_BY_HH                (R_SCI_B2_BASE + R_SCI_B_CCR3_BY_HH_OFFSET)
#define R_SCI_B2_CCR4                      (R_SCI_B2_BASE + R_SCI_B_CCR4_OFFSET)
#define R_SCI_B2_CCR4_HA_L                 (R_SCI_B2_BASE + R_SCI_B_CCR4_HA_L_OFFSET)
#define R_SCI_B2_CCR4_BY_LL                (R_SCI_B2_BASE + R_SCI_B_CCR4_BY_LL_OFFSET)
#define R_SCI_B2_CCR4_BY_LH                (R_SCI_B2_BASE + R_SCI_B_CCR4_BY_LH_OFFSET)
#define R_SCI_B2_CCR4_HA_H                 (R_SCI_B2_BASE + R_SCI_B_CCR4_HA_H_OFFSET)
#define R_SCI_B2_CCR4_BY_HL                (R_SCI_B2_BASE + R_SCI_B_CCR4_BY_HL_OFFSET)
#define R_SCI_B2_CCR4_BY_HH                (R_SCI_B2_BASE + R_SCI_B_CCR4_BY_HH_OFFSET)
#define R_SCI_B2_CESR                      (R_SCI_B2_BASE + R_SCI_B_CESR_OFFSET)
#define R_SCI_B2_HCR                       (R_SCI_B2_BASE + R_SCI_B_HCR_OFFSET)
#define R_SCI_B2_ICR                       (R_SCI_B2_BASE + R_SCI_B_ICR_OFFSET)
#define R_SCI_B2_ICR_HA_L                  (R_SCI_B2_BASE + R_SCI_B_ICR_HA_L_OFFSET)
#define R_SCI_B2_ICR_BY_LL                 (R_SCI_B2_BASE + R_SCI_B_ICR_BY_LL_OFFSET)
#define R_SCI_B2_ICR_BY_LH                 (R_SCI_B2_BASE + R_SCI_B_ICR_BY_LH_OFFSET)
#define R_SCI_B2_ICR_HA_H                  (R_SCI_B2_BASE + R_SCI_B_ICR_HA_H_OFFSET)
#define R_SCI_B2_ICR_BY_HL                 (R_SCI_B2_BASE + R_SCI_B_ICR_BY_HL_OFFSET)
#define R_SCI_B2_FCR                       (R_SCI_B2_BASE + R_SCI_B_FCR_OFFSET)
#define R_SCI_B2_FCR_HA_L                  (R_SCI_B2_BASE + R_SCI_B_FCR_HA_L_OFFSET)
#define R_SCI_B2_FCR_BY_LL                 (R_SCI_B2_BASE + R_SCI_B_FCR_BY_LL_OFFSET)
#define R_SCI_B2_FCR_BY_LH                 (R_SCI_B2_BASE + R_SCI_B_FCR_BY_LH_OFFSET)
#define R_SCI_B2_FCR_HA_H                  (R_SCI_B2_BASE + R_SCI_B_FCR_HA_H_OFFSET)
#define R_SCI_B2_FCR_BY_HL                 (R_SCI_B2_BASE + R_SCI_B_FCR_BY_HL_OFFSET)
#define R_SCI_B2_FCR_BY_HH                 (R_SCI_B2_BASE + R_SCI_B_FCR_BY_HH_OFFSET)
#define R_SCI_B2_MCR                       (R_SCI_B2_BASE + R_SCI_B_MCR_OFFSET)
#define R_SCI_B2_MCR_HA_L                  (R_SCI_B2_BASE + R_SCI_B_MCR_HA_L_OFFSET)
#define R_SCI_B2_MCR_BY_LL                 (R_SCI_B2_BASE + R_SCI_B_MCR_BY_LL_OFFSET)
#define R_SCI_B2_MCR_BY_LH                 (R_SCI_B2_BASE + R_SCI_B_MCR_BY_LH_OFFSET)
#define R_SCI_B2_MCR_HA_H                  (R_SCI_B2_BASE + R_SCI_B_MCR_HA_H_OFFSET)
#define R_SCI_B2_MCR_BY_HL                 (R_SCI_B2_BASE + R_SCI_B_MCR_BY_HL_OFFSET)
#define R_SCI_B2_MCR_BY_HH                 (R_SCI_B2_BASE + R_SCI_B_MCR_BY_HH_OFFSET)
#define R_SCI_B2_DCR                       (R_SCI_B2_BASE + R_SCI_B_DCR_OFFSET)
#define R_SCI_B2_DCR_HA_L                  (R_SCI_B2_BASE + R_SCI_B_DCR_HA_L_OFFSET)
#define R_SCI_B2_DCR_BY_LL                 (R_SCI_B2_BASE + R_SCI_B_DCR_BY_LL_OFFSET)
#define R_SCI_B2_DCR_BY_LH                 (R_SCI_B2_BASE + R_SCI_B_DCR_BY_LH_OFFSET)
#define R_SCI_B2_DCR_HA_H                  (R_SCI_B2_BASE + R_SCI_B_DCR_HA_H_OFFSET)
#define R_SCI_B2_DCR_BY_HL                 (R_SCI_B2_BASE + R_SCI_B_DCR_BY_HL_OFFSET)
#define R_SCI_B2_XCR0                      (R_SCI_B2_BASE + R_SCI_B_XCR0_OFFSET)
#define R_SCI_B2_XCR0_HA_L                 (R_SCI_B2_BASE + R_SCI_B_XCR0_HA_L_OFFSET)
#define R_SCI_B2_XCR0_BY_LL                (R_SCI_B2_BASE + R_SCI_B_XCR0_BY_LL_OFFSET)
#define R_SCI_B2_XCR0_BY_LH                (R_SCI_B2_BASE + R_SCI_B_XCR0_BY_LH_OFFSET)
#define R_SCI_B2_XCR0_HA_H                 (R_SCI_B2_BASE + R_SCI_B_XCR0_HA_H_OFFSET)
#define R_SCI_B2_XCR0_BY_HL                (R_SCI_B2_BASE + R_SCI_B_XCR0_BY_HL_OFFSET)
#define R_SCI_B2_XCR0_BY_HH                (R_SCI_B2_BASE + R_SCI_B_XCR0_BY_HH_OFFSET)
#define R_SCI_B2_XCR1                      (R_SCI_B2_BASE + R_SCI_B_XCR1_OFFSET)
#define R_SCI_B2_XCR1_HA_L                 (R_SCI_B2_BASE + R_SCI_B_XCR1_HA_L_OFFSET)
#define R_SCI_B2_XCR1_BY_LL                (R_SCI_B2_BASE + R_SCI_B_XCR1_BY_LL_OFFSET)
#define R_SCI_B2_XCR1_BY_LH                (R_SCI_B2_BASE + R_SCI_B_XCR1_BY_LH_OFFSET)
#define R_SCI_B2_XCR1_HA_H                 (R_SCI_B2_BASE + R_SCI_B_XCR1_HA_H_OFFSET)
#define R_SCI_B2_XCR1_BY_HL                (R_SCI_B2_BASE + R_SCI_B_XCR1_BY_HL_OFFSET)
#define R_SCI_B2_XCR1_BY_HH                (R_SCI_B2_BASE + R_SCI_B_XCR1_BY_HH_OFFSET)
#define R_SCI_B2_XCR2                      (R_SCI_B2_BASE + R_SCI_B_XCR2_OFFSET)
#define R_SCI_B2_XCR2_HA_L                 (R_SCI_B2_BASE + R_SCI_B_XCR2_HA_L_OFFSET)
#define R_SCI_B2_XCR2_BY_LL                (R_SCI_B2_BASE + R_SCI_B_XCR2_BY_LL_OFFSET)
#define R_SCI_B2_XCR2_BY_LH                (R_SCI_B2_BASE + R_SCI_B_XCR2_BY_LH_OFFSET)
#define R_SCI_B2_XCR2_HA_H                 (R_SCI_B2_BASE + R_SCI_B_XCR2_HA_H_OFFSET)
#define R_SCI_B2_XCR2_BY_HL                (R_SCI_B2_BASE + R_SCI_B_XCR2_BY_HL_OFFSET)
#define R_SCI_B2_XCR2_BY_HH                (R_SCI_B2_BASE + R_SCI_B_XCR2_BY_HH_OFFSET)
#define R_SCI_B2_CSR                       (R_SCI_B2_BASE + R_SCI_B_CSR_OFFSET)
#define R_SCI_B2_ISR                       (R_SCI_B2_BASE + R_SCI_B_ISR_OFFSET)
#define R_SCI_B2_FRSR                      (R_SCI_B2_BASE + R_SCI_B_FRSR_OFFSET)
#define R_SCI_B2_FTSR                      (R_SCI_B2_BASE + R_SCI_B_FTSR_OFFSET)
#define R_SCI_B2_MSR                       (R_SCI_B2_BASE + R_SCI_B_MSR_OFFSET)
#define R_SCI_B2_XSR0                      (R_SCI_B2_BASE + R_SCI_B_XSR0_OFFSET)
#define R_SCI_B2_XSR1                      (R_SCI_B2_BASE + R_SCI_B_XSR1_OFFSET)
#define R_SCI_B2_CFCLR                     (R_SCI_B2_BASE + R_SCI_B_CFCLR_OFFSET)
#define R_SCI_B2_CFCLR_HA_L                (R_SCI_B2_BASE + R_SCI_B_CFCLR_HA_L_OFFSET)
#define R_SCI_B2_CFCLR_BY_LL               (R_SCI_B2_BASE + R_SCI_B_CFCLR_BY_LL_OFFSET)
#define R_SCI_B2_CFCLR_HA_H                (R_SCI_B2_BASE + R_SCI_B_CFCLR_HA_H_OFFSET)
#define R_SCI_B2_CFCLR_BY_HL               (R_SCI_B2_BASE + R_SCI_B_CFCLR_BY_HL_OFFSET)
#define R_SCI_B2_CFCLR_BY_HH               (R_SCI_B2_BASE + R_SCI_B_CFCLR_BY_HH_OFFSET)
#define R_SCI_B2_ICFCLR                    (R_SCI_B2_BASE + R_SCI_B_ICFCLR_OFFSET)
#define R_SCI_B2_ICFCLR_HA_L               (R_SCI_B2_BASE + R_SCI_B_ICFCLR_HA_L_OFFSET)
#define R_SCI_B2_ICFCLR_BY_LL              (R_SCI_B2_BASE + R_SCI_B_ICFCLR_BY_LL_OFFSET)
#define R_SCI_B2_FFCLR                     (R_SCI_B2_BASE + R_SCI_B_FFCLR_OFFSET)
#define R_SCI_B2_FFCLR_HA_L                (R_SCI_B2_BASE + R_SCI_B_FFCLR_HA_L_OFFSET)
#define R_SCI_B2_FFCLR_BY_LL               (R_SCI_B2_BASE + R_SCI_B_FFCLR_BY_LL_OFFSET)
#define R_SCI_B2_MFCLR                     (R_SCI_B2_BASE + R_SCI_B_MFCLR_OFFSET)
#define R_SCI_B2_MFCLR_HA_L                (R_SCI_B2_BASE + R_SCI_B_MFCLR_HA_L_OFFSET)
#define R_SCI_B2_MFCLR_BY_LL               (R_SCI_B2_BASE + R_SCI_B_MFCLR_BY_LL_OFFSET)
#define R_SCI_B2_XFCLR                     (R_SCI_B2_BASE + R_SCI_B_XFCLR_OFFSET)
#define R_SCI_B2_XFCLR_HA_L                (R_SCI_B2_BASE + R_SCI_B_XFCLR_HA_L_OFFSET)
#define R_SCI_B2_XFCLR_BY_LH               (R_SCI_B2_BASE + R_SCI_B_XFCLR_BY_LH_OFFSET)

/* SCI_B3 Registers */

#define R_SCI_B3_RDR                       (R_SCI_B3_BASE + R_SCI_B_RDR_OFFSET)
#define R_SCI_B3_TDR                       (R_SCI_B3_BASE + R_SCI_B_TDR_OFFSET)
#define R_SCI_B3_TDR_HA_L                  (R_SCI_B3_BASE + R_SCI_B_TDR_HA_L_OFFSET)
#define R_SCI_B3_TDR_BY_LL                 (R_SCI_B3_BASE + R_SCI_B_TDR_BY_LL_OFFSET)
#define R_SCI_B3_TDR_BY_LH                 (R_SCI_B3_BASE + R_SCI_B_TDR_BY_LH_OFFSET)
#define R_SCI_B3_CCR0                      (R_SCI_B3_BASE + R_SCI_B_CCR0_OFFSET)
#define R_SCI_B3_CCR0_HA_L                 (R_SCI_B3_BASE + R_SCI_B_CCR0_HA_L_OFFSET)
#define R_SCI_B3_CCR0_BY_LL                (R_SCI_B3_BASE + R_SCI_B_CCR0_BY_LL_OFFSET)
#define R_SCI_B3_CCR0_BY_LH                (R_SCI_B3_BASE + R_SCI_B_CCR0_BY_LH_OFFSET)
#define R_SCI_B3_CCR0_HA_H                 (R_SCI_B3_BASE + R_SCI_B_CCR0_HA_H_OFFSET)
#define R_SCI_B3_CCR0_BY_HL                (R_SCI_B3_BASE + R_SCI_B_CCR0_BY_HL_OFFSET)
#define R_SCI_B3_CCR0_BY_HH                (R_SCI_B3_BASE + R_SCI_B_CCR0_BY_HH_OFFSET)
#define R_SCI_B3_CCR1                      (R_SCI_B3_BASE + R_SCI_B_CCR1_OFFSET)
#define R_SCI_B3_CCR1_HA_L                 (R_SCI_B3_BASE + R_SCI_B_CCR1_HA_L_OFFSET)
#define R_SCI_B3_CCR1_BY_LL                (R_SCI_B3_BASE + R_SCI_B_CCR1_BY_LL_OFFSET)
#define R_SCI_B3_CCR1_BY_LH                (R_SCI_B3_BASE + R_SCI_B_CCR1_BY_LH_OFFSET)
#define R_SCI_B3_CCR1_HA_H                 (R_SCI_B3_BASE + R_SCI_B_CCR1_HA_H_OFFSET)
#define R_SCI_B3_CCR1_BY_HL                (R_SCI_B3_BASE + R_SCI_B_CCR1_BY_HL_OFFSET)
#define R_SCI_B3_CCR1_BY_HH                (R_SCI_B3_BASE + R_SCI_B_CCR1_BY_HH_OFFSET)
#define R_SCI_B3_CCR2                      (R_SCI_B3_BASE + R_SCI_B_CCR2_OFFSET)
#define R_SCI_B3_CCR2_HA_L                 (R_SCI_B3_BASE + R_SCI_B_CCR2_HA_L_OFFSET)
#define R_SCI_B3_CCR2_BY_LL                (R_SCI_B3_BASE + R_SCI_B_CCR2_BY_LL_OFFSET)
#define R_SCI_B3_CCR2_BY_LH                (R_SCI_B3_BASE + R_SCI_B_CCR2_BY_LH_OFFSET)
#define R_SCI_B3_CCR2_HA_H                 (R_SCI_B3_BASE + R_SCI_B_CCR2_HA_H_OFFSET)
#define R_SCI_B3_CCR2_BY_HL                (R_SCI_B3_BASE + R_SCI_B_CCR2_BY_HL_OFFSET)
#define R_SCI_B3_CCR2_BY_HH                (R_SCI_B3_BASE + R_SCI_B_CCR2_BY_HH_OFFSET)
#define R_SCI_B3_CCR3                      (R_SCI_B3_BASE + R_SCI_B_CCR3_OFFSET)
#define R_SCI_B3_CCR3_HA_L                 (R_SCI_B3_BASE + R_SCI_B_CCR3_HA_L_OFFSET)
#define R_SCI_B3_CCR3_BY_LL                (R_SCI_B3_BASE + R_SCI_B_CCR3_BY_LL_OFFSET)
#define R_SCI_B3_CCR3_BY_LH                (R_SCI_B3_BASE + R_SCI_B_CCR3_BY_LH_OFFSET)
#define R_SCI_B3_CCR3_HA_H                 (R_SCI_B3_BASE + R_SCI_B_CCR3_HA_H_OFFSET)
#define R_SCI_B3_CCR3_BY_HL                (R_SCI_B3_BASE + R_SCI_B_CCR3_BY_HL_OFFSET)
#define R_SCI_B3_CCR3_BY_HH                (R_SCI_B3_BASE + R_SCI_B_CCR3_BY_HH_OFFSET)
#define R_SCI_B3_CCR4                      (R_SCI_B3_BASE + R_SCI_B_CCR4_OFFSET)
#define R_SCI_B3_CCR4_HA_L                 (R_SCI_B3_BASE + R_SCI_B_CCR4_HA_L_OFFSET)
#define R_SCI_B3_CCR4_BY_LL                (R_SCI_B3_BASE + R_SCI_B_CCR4_BY_LL_OFFSET)
#define R_SCI_B3_CCR4_BY_LH                (R_SCI_B3_BASE + R_SCI_B_CCR4_BY_LH_OFFSET)
#define R_SCI_B3_CCR4_HA_H                 (R_SCI_B3_BASE + R_SCI_B_CCR4_HA_H_OFFSET)
#define R_SCI_B3_CCR4_BY_HL                (R_SCI_B3_BASE + R_SCI_B_CCR4_BY_HL_OFFSET)
#define R_SCI_B3_CCR4_BY_HH                (R_SCI_B3_BASE + R_SCI_B_CCR4_BY_HH_OFFSET)
#define R_SCI_B3_CESR                      (R_SCI_B3_BASE + R_SCI_B_CESR_OFFSET)
#define R_SCI_B3_HCR                       (R_SCI_B3_BASE + R_SCI_B_HCR_OFFSET)
#define R_SCI_B3_ICR                       (R_SCI_B3_BASE + R_SCI_B_ICR_OFFSET)
#define R_SCI_B3_ICR_HA_L                  (R_SCI_B3_BASE + R_SCI_B_ICR_HA_L_OFFSET)
#define R_SCI_B3_ICR_BY_LL                 (R_SCI_B3_BASE + R_SCI_B_ICR_BY_LL_OFFSET)
#define R_SCI_B3_ICR_BY_LH                 (R_SCI_B3_BASE + R_SCI_B_ICR_BY_LH_OFFSET)
#define R_SCI_B3_ICR_HA_H                  (R_SCI_B3_BASE + R_SCI_B_ICR_HA_H_OFFSET)
#define R_SCI_B3_ICR_BY_HL                 (R_SCI_B3_BASE + R_SCI_B_ICR_BY_HL_OFFSET)
#define R_SCI_B3_FCR                       (R_SCI_B3_BASE + R_SCI_B_FCR_OFFSET)
#define R_SCI_B3_FCR_HA_L                  (R_SCI_B3_BASE + R_SCI_B_FCR_HA_L_OFFSET)
#define R_SCI_B3_FCR_BY_LL                 (R_SCI_B3_BASE + R_SCI_B_FCR_BY_LL_OFFSET)
#define R_SCI_B3_FCR_BY_LH                 (R_SCI_B3_BASE + R_SCI_B_FCR_BY_LH_OFFSET)
#define R_SCI_B3_FCR_HA_H                  (R_SCI_B3_BASE + R_SCI_B_FCR_HA_H_OFFSET)
#define R_SCI_B3_FCR_BY_HL                 (R_SCI_B3_BASE + R_SCI_B_FCR_BY_HL_OFFSET)
#define R_SCI_B3_FCR_BY_HH                 (R_SCI_B3_BASE + R_SCI_B_FCR_BY_HH_OFFSET)
#define R_SCI_B3_MCR                       (R_SCI_B3_BASE + R_SCI_B_MCR_OFFSET)
#define R_SCI_B3_MCR_HA_L                  (R_SCI_B3_BASE + R_SCI_B_MCR_HA_L_OFFSET)
#define R_SCI_B3_MCR_BY_LL                 (R_SCI_B3_BASE + R_SCI_B_MCR_BY_LL_OFFSET)
#define R_SCI_B3_MCR_BY_LH                 (R_SCI_B3_BASE + R_SCI_B_MCR_BY_LH_OFFSET)
#define R_SCI_B3_MCR_HA_H                  (R_SCI_B3_BASE + R_SCI_B_MCR_HA_H_OFFSET)
#define R_SCI_B3_MCR_BY_HL                 (R_SCI_B3_BASE + R_SCI_B_MCR_BY_HL_OFFSET)
#define R_SCI_B3_MCR_BY_HH                 (R_SCI_B3_BASE + R_SCI_B_MCR_BY_HH_OFFSET)
#define R_SCI_B3_DCR                       (R_SCI_B3_BASE + R_SCI_B_DCR_OFFSET)
#define R_SCI_B3_DCR_HA_L                  (R_SCI_B3_BASE + R_SCI_B_DCR_HA_L_OFFSET)
#define R_SCI_B3_DCR_BY_LL                 (R_SCI_B3_BASE + R_SCI_B_DCR_BY_LL_OFFSET)
#define R_SCI_B3_DCR_BY_LH                 (R_SCI_B3_BASE + R_SCI_B_DCR_BY_LH_OFFSET)
#define R_SCI_B3_DCR_HA_H                  (R_SCI_B3_BASE + R_SCI_B_DCR_HA_H_OFFSET)
#define R_SCI_B3_DCR_BY_HL                 (R_SCI_B3_BASE + R_SCI_B_DCR_BY_HL_OFFSET)
#define R_SCI_B3_XCR0                      (R_SCI_B3_BASE + R_SCI_B_XCR0_OFFSET)
#define R_SCI_B3_XCR0_HA_L                 (R_SCI_B3_BASE + R_SCI_B_XCR0_HA_L_OFFSET)
#define R_SCI_B3_XCR0_BY_LL                (R_SCI_B3_BASE + R_SCI_B_XCR0_BY_LL_OFFSET)
#define R_SCI_B3_XCR0_BY_LH                (R_SCI_B3_BASE + R_SCI_B_XCR0_BY_LH_OFFSET)
#define R_SCI_B3_XCR0_HA_H                 (R_SCI_B3_BASE + R_SCI_B_XCR0_HA_H_OFFSET)
#define R_SCI_B3_XCR0_BY_HL                (R_SCI_B3_BASE + R_SCI_B_XCR0_BY_HL_OFFSET)
#define R_SCI_B3_XCR0_BY_HH                (R_SCI_B3_BASE + R_SCI_B_XCR0_BY_HH_OFFSET)
#define R_SCI_B3_XCR1                      (R_SCI_B3_BASE + R_SCI_B_XCR1_OFFSET)
#define R_SCI_B3_XCR1_HA_L                 (R_SCI_B3_BASE + R_SCI_B_XCR1_HA_L_OFFSET)
#define R_SCI_B3_XCR1_BY_LL                (R_SCI_B3_BASE + R_SCI_B_XCR1_BY_LL_OFFSET)
#define R_SCI_B3_XCR1_BY_LH                (R_SCI_B3_BASE + R_SCI_B_XCR1_BY_LH_OFFSET)
#define R_SCI_B3_XCR1_HA_H                 (R_SCI_B3_BASE + R_SCI_B_XCR1_HA_H_OFFSET)
#define R_SCI_B3_XCR1_BY_HL                (R_SCI_B3_BASE + R_SCI_B_XCR1_BY_HL_OFFSET)
#define R_SCI_B3_XCR1_BY_HH                (R_SCI_B3_BASE + R_SCI_B_XCR1_BY_HH_OFFSET)
#define R_SCI_B3_XCR2                      (R_SCI_B3_BASE + R_SCI_B_XCR2_OFFSET)
#define R_SCI_B3_XCR2_HA_L                 (R_SCI_B3_BASE + R_SCI_B_XCR2_HA_L_OFFSET)
#define R_SCI_B3_XCR2_BY_LL                (R_SCI_B3_BASE + R_SCI_B_XCR2_BY_LL_OFFSET)
#define R_SCI_B3_XCR2_BY_LH                (R_SCI_B3_BASE + R_SCI_B_XCR2_BY_LH_OFFSET)
#define R_SCI_B3_XCR2_HA_H                 (R_SCI_B3_BASE + R_SCI_B_XCR2_HA_H_OFFSET)
#define R_SCI_B3_XCR2_BY_HL                (R_SCI_B3_BASE + R_SCI_B_XCR2_BY_HL_OFFSET)
#define R_SCI_B3_XCR2_BY_HH                (R_SCI_B3_BASE + R_SCI_B_XCR2_BY_HH_OFFSET)
#define R_SCI_B3_CSR                       (R_SCI_B3_BASE + R_SCI_B_CSR_OFFSET)
#define R_SCI_B3_ISR                       (R_SCI_B3_BASE + R_SCI_B_ISR_OFFSET)
#define R_SCI_B3_FRSR                      (R_SCI_B3_BASE + R_SCI_B_FRSR_OFFSET)
#define R_SCI_B3_FTSR                      (R_SCI_B3_BASE + R_SCI_B_FTSR_OFFSET)
#define R_SCI_B3_MSR                       (R_SCI_B3_BASE + R_SCI_B_MSR_OFFSET)
#define R_SCI_B3_XSR0                      (R_SCI_B3_BASE + R_SCI_B_XSR0_OFFSET)
#define R_SCI_B3_XSR1                      (R_SCI_B3_BASE + R_SCI_B_XSR1_OFFSET)
#define R_SCI_B3_CFCLR                     (R_SCI_B3_BASE + R_SCI_B_CFCLR_OFFSET)
#define R_SCI_B3_CFCLR_HA_L                (R_SCI_B3_BASE + R_SCI_B_CFCLR_HA_L_OFFSET)
#define R_SCI_B3_CFCLR_BY_LL               (R_SCI_B3_BASE + R_SCI_B_CFCLR_BY_LL_OFFSET)
#define R_SCI_B3_CFCLR_HA_H                (R_SCI_B3_BASE + R_SCI_B_CFCLR_HA_H_OFFSET)
#define R_SCI_B3_CFCLR_BY_HL               (R_SCI_B3_BASE + R_SCI_B_CFCLR_BY_HL_OFFSET)
#define R_SCI_B3_CFCLR_BY_HH               (R_SCI_B3_BASE + R_SCI_B_CFCLR_BY_HH_OFFSET)
#define R_SCI_B3_ICFCLR                    (R_SCI_B3_BASE + R_SCI_B_ICFCLR_OFFSET)
#define R_SCI_B3_ICFCLR_HA_L               (R_SCI_B3_BASE + R_SCI_B_ICFCLR_HA_L_OFFSET)
#define R_SCI_B3_ICFCLR_BY_LL              (R_SCI_B3_BASE + R_SCI_B_ICFCLR_BY_LL_OFFSET)
#define R_SCI_B3_FFCLR                     (R_SCI_B3_BASE + R_SCI_B_FFCLR_OFFSET)
#define R_SCI_B3_FFCLR_HA_L                (R_SCI_B3_BASE + R_SCI_B_FFCLR_HA_L_OFFSET)
#define R_SCI_B3_FFCLR_BY_LL               (R_SCI_B3_BASE + R_SCI_B_FFCLR_BY_LL_OFFSET)
#define R_SCI_B3_MFCLR                     (R_SCI_B3_BASE + R_SCI_B_MFCLR_OFFSET)
#define R_SCI_B3_MFCLR_HA_L                (R_SCI_B3_BASE + R_SCI_B_MFCLR_HA_L_OFFSET)
#define R_SCI_B3_MFCLR_BY_LL               (R_SCI_B3_BASE + R_SCI_B_MFCLR_BY_LL_OFFSET)
#define R_SCI_B3_XFCLR                     (R_SCI_B3_BASE + R_SCI_B_XFCLR_OFFSET)
#define R_SCI_B3_XFCLR_HA_L                (R_SCI_B3_BASE + R_SCI_B_XFCLR_HA_L_OFFSET)
#define R_SCI_B3_XFCLR_BY_LH               (R_SCI_B3_BASE + R_SCI_B_XFCLR_BY_LH_OFFSET)

/* SCI_B4 Registers */

#define R_SCI_B4_RDR                       (R_SCI_B4_BASE + R_SCI_B_RDR_OFFSET)
#define R_SCI_B4_TDR                       (R_SCI_B4_BASE + R_SCI_B_TDR_OFFSET)
#define R_SCI_B4_TDR_HA_L                  (R_SCI_B4_BASE + R_SCI_B_TDR_HA_L_OFFSET)
#define R_SCI_B4_TDR_BY_LL                 (R_SCI_B4_BASE + R_SCI_B_TDR_BY_LL_OFFSET)
#define R_SCI_B4_TDR_BY_LH                 (R_SCI_B4_BASE + R_SCI_B_TDR_BY_LH_OFFSET)
#define R_SCI_B4_CCR0                      (R_SCI_B4_BASE + R_SCI_B_CCR0_OFFSET)
#define R_SCI_B4_CCR0_HA_L                 (R_SCI_B4_BASE + R_SCI_B_CCR0_HA_L_OFFSET)
#define R_SCI_B4_CCR0_BY_LL                (R_SCI_B4_BASE + R_SCI_B_CCR0_BY_LL_OFFSET)
#define R_SCI_B4_CCR0_BY_LH                (R_SCI_B4_BASE + R_SCI_B_CCR0_BY_LH_OFFSET)
#define R_SCI_B4_CCR0_HA_H                 (R_SCI_B4_BASE + R_SCI_B_CCR0_HA_H_OFFSET)
#define R_SCI_B4_CCR0_BY_HL                (R_SCI_B4_BASE + R_SCI_B_CCR0_BY_HL_OFFSET)
#define R_SCI_B4_CCR0_BY_HH                (R_SCI_B4_BASE + R_SCI_B_CCR0_BY_HH_OFFSET)
#define R_SCI_B4_CCR1                      (R_SCI_B4_BASE + R_SCI_B_CCR1_OFFSET)
#define R_SCI_B4_CCR1_HA_L                 (R_SCI_B4_BASE + R_SCI_B_CCR1_HA_L_OFFSET)
#define R_SCI_B4_CCR1_BY_LL                (R_SCI_B4_BASE + R_SCI_B_CCR1_BY_LL_OFFSET)
#define R_SCI_B4_CCR1_BY_LH                (R_SCI_B4_BASE + R_SCI_B_CCR1_BY_LH_OFFSET)
#define R_SCI_B4_CCR1_HA_H                 (R_SCI_B4_BASE + R_SCI_B_CCR1_HA_H_OFFSET)
#define R_SCI_B4_CCR1_BY_HL                (R_SCI_B4_BASE + R_SCI_B_CCR1_BY_HL_OFFSET)
#define R_SCI_B4_CCR1_BY_HH                (R_SCI_B4_BASE + R_SCI_B_CCR1_BY_HH_OFFSET)
#define R_SCI_B4_CCR2                      (R_SCI_B4_BASE + R_SCI_B_CCR2_OFFSET)
#define R_SCI_B4_CCR2_HA_L                 (R_SCI_B4_BASE + R_SCI_B_CCR2_HA_L_OFFSET)
#define R_SCI_B4_CCR2_BY_LL                (R_SCI_B4_BASE + R_SCI_B_CCR2_BY_LL_OFFSET)
#define R_SCI_B4_CCR2_BY_LH                (R_SCI_B4_BASE + R_SCI_B_CCR2_BY_LH_OFFSET)
#define R_SCI_B4_CCR2_HA_H                 (R_SCI_B4_BASE + R_SCI_B_CCR2_HA_H_OFFSET)
#define R_SCI_B4_CCR2_BY_HL                (R_SCI_B4_BASE + R_SCI_B_CCR2_BY_HL_OFFSET)
#define R_SCI_B4_CCR2_BY_HH                (R_SCI_B4_BASE + R_SCI_B_CCR2_BY_HH_OFFSET)
#define R_SCI_B4_CCR3                      (R_SCI_B4_BASE + R_SCI_B_CCR3_OFFSET)
#define R_SCI_B4_CCR3_HA_L                 (R_SCI_B4_BASE + R_SCI_B_CCR3_HA_L_OFFSET)
#define R_SCI_B4_CCR3_BY_LL                (R_SCI_B4_BASE + R_SCI_B_CCR3_BY_LL_OFFSET)
#define R_SCI_B4_CCR3_BY_LH                (R_SCI_B4_BASE + R_SCI_B_CCR3_BY_LH_OFFSET)
#define R_SCI_B4_CCR3_HA_H                 (R_SCI_B4_BASE + R_SCI_B_CCR3_HA_H_OFFSET)
#define R_SCI_B4_CCR3_BY_HL                (R_SCI_B4_BASE + R_SCI_B_CCR3_BY_HL_OFFSET)
#define R_SCI_B4_CCR3_BY_HH                (R_SCI_B4_BASE + R_SCI_B_CCR3_BY_HH_OFFSET)
#define R_SCI_B4_CCR4                      (R_SCI_B4_BASE + R_SCI_B_CCR4_OFFSET)
#define R_SCI_B4_CCR4_HA_L                 (R_SCI_B4_BASE + R_SCI_B_CCR4_HA_L_OFFSET)
#define R_SCI_B4_CCR4_BY_LL                (R_SCI_B4_BASE + R_SCI_B_CCR4_BY_LL_OFFSET)
#define R_SCI_B4_CCR4_BY_LH                (R_SCI_B4_BASE + R_SCI_B_CCR4_BY_LH_OFFSET)
#define R_SCI_B4_CCR4_HA_H                 (R_SCI_B4_BASE + R_SCI_B_CCR4_HA_H_OFFSET)
#define R_SCI_B4_CCR4_BY_HL                (R_SCI_B4_BASE + R_SCI_B_CCR4_BY_HL_OFFSET)
#define R_SCI_B4_CCR4_BY_HH                (R_SCI_B4_BASE + R_SCI_B_CCR4_BY_HH_OFFSET)
#define R_SCI_B4_CESR                      (R_SCI_B4_BASE + R_SCI_B_CESR_OFFSET)
#define R_SCI_B4_HCR                       (R_SCI_B4_BASE + R_SCI_B_HCR_OFFSET)
#define R_SCI_B4_ICR                       (R_SCI_B4_BASE + R_SCI_B_ICR_OFFSET)
#define R_SCI_B4_ICR_HA_L                  (R_SCI_B4_BASE + R_SCI_B_ICR_HA_L_OFFSET)
#define R_SCI_B4_ICR_BY_LL                 (R_SCI_B4_BASE + R_SCI_B_ICR_BY_LL_OFFSET)
#define R_SCI_B4_ICR_BY_LH                 (R_SCI_B4_BASE + R_SCI_B_ICR_BY_LH_OFFSET)
#define R_SCI_B4_ICR_HA_H                  (R_SCI_B4_BASE + R_SCI_B_ICR_HA_H_OFFSET)
#define R_SCI_B4_ICR_BY_HL                 (R_SCI_B4_BASE + R_SCI_B_ICR_BY_HL_OFFSET)
#define R_SCI_B4_FCR                       (R_SCI_B4_BASE + R_SCI_B_FCR_OFFSET)
#define R_SCI_B4_FCR_HA_L                  (R_SCI_B4_BASE + R_SCI_B_FCR_HA_L_OFFSET)
#define R_SCI_B4_FCR_BY_LL                 (R_SCI_B4_BASE + R_SCI_B_FCR_BY_LL_OFFSET)
#define R_SCI_B4_FCR_BY_LH                 (R_SCI_B4_BASE + R_SCI_B_FCR_BY_LH_OFFSET)
#define R_SCI_B4_FCR_HA_H                  (R_SCI_B4_BASE + R_SCI_B_FCR_HA_H_OFFSET)
#define R_SCI_B4_FCR_BY_HL                 (R_SCI_B4_BASE + R_SCI_B_FCR_BY_HL_OFFSET)
#define R_SCI_B4_FCR_BY_HH                 (R_SCI_B4_BASE + R_SCI_B_FCR_BY_HH_OFFSET)
#define R_SCI_B4_MCR                       (R_SCI_B4_BASE + R_SCI_B_MCR_OFFSET)
#define R_SCI_B4_MCR_HA_L                  (R_SCI_B4_BASE + R_SCI_B_MCR_HA_L_OFFSET)
#define R_SCI_B4_MCR_BY_LL                 (R_SCI_B4_BASE + R_SCI_B_MCR_BY_LL_OFFSET)
#define R_SCI_B4_MCR_BY_LH                 (R_SCI_B4_BASE + R_SCI_B_MCR_BY_LH_OFFSET)
#define R_SCI_B4_MCR_HA_H                  (R_SCI_B4_BASE + R_SCI_B_MCR_HA_H_OFFSET)
#define R_SCI_B4_MCR_BY_HL                 (R_SCI_B4_BASE + R_SCI_B_MCR_BY_HL_OFFSET)
#define R_SCI_B4_MCR_BY_HH                 (R_SCI_B4_BASE + R_SCI_B_MCR_BY_HH_OFFSET)
#define R_SCI_B4_DCR                       (R_SCI_B4_BASE + R_SCI_B_DCR_OFFSET)
#define R_SCI_B4_DCR_HA_L                  (R_SCI_B4_BASE + R_SCI_B_DCR_HA_L_OFFSET)
#define R_SCI_B4_DCR_BY_LL                 (R_SCI_B4_BASE + R_SCI_B_DCR_BY_LL_OFFSET)
#define R_SCI_B4_DCR_BY_LH                 (R_SCI_B4_BASE + R_SCI_B_DCR_BY_LH_OFFSET)
#define R_SCI_B4_DCR_HA_H                  (R_SCI_B4_BASE + R_SCI_B_DCR_HA_H_OFFSET)
#define R_SCI_B4_DCR_BY_HL                 (R_SCI_B4_BASE + R_SCI_B_DCR_BY_HL_OFFSET)
#define R_SCI_B4_XCR0                      (R_SCI_B4_BASE + R_SCI_B_XCR0_OFFSET)
#define R_SCI_B4_XCR0_HA_L                 (R_SCI_B4_BASE + R_SCI_B_XCR0_HA_L_OFFSET)
#define R_SCI_B4_XCR0_BY_LL                (R_SCI_B4_BASE + R_SCI_B_XCR0_BY_LL_OFFSET)
#define R_SCI_B4_XCR0_BY_LH                (R_SCI_B4_BASE + R_SCI_B_XCR0_BY_LH_OFFSET)
#define R_SCI_B4_XCR0_HA_H                 (R_SCI_B4_BASE + R_SCI_B_XCR0_HA_H_OFFSET)
#define R_SCI_B4_XCR0_BY_HL                (R_SCI_B4_BASE + R_SCI_B_XCR0_BY_HL_OFFSET)
#define R_SCI_B4_XCR0_BY_HH                (R_SCI_B4_BASE + R_SCI_B_XCR0_BY_HH_OFFSET)
#define R_SCI_B4_XCR1                      (R_SCI_B4_BASE + R_SCI_B_XCR1_OFFSET)
#define R_SCI_B4_XCR1_HA_L                 (R_SCI_B4_BASE + R_SCI_B_XCR1_HA_L_OFFSET)
#define R_SCI_B4_XCR1_BY_LL                (R_SCI_B4_BASE + R_SCI_B_XCR1_BY_LL_OFFSET)
#define R_SCI_B4_XCR1_BY_LH                (R_SCI_B4_BASE + R_SCI_B_XCR1_BY_LH_OFFSET)
#define R_SCI_B4_XCR1_HA_H                 (R_SCI_B4_BASE + R_SCI_B_XCR1_HA_H_OFFSET)
#define R_SCI_B4_XCR1_BY_HL                (R_SCI_B4_BASE + R_SCI_B_XCR1_BY_HL_OFFSET)
#define R_SCI_B4_XCR1_BY_HH                (R_SCI_B4_BASE + R_SCI_B_XCR1_BY_HH_OFFSET)
#define R_SCI_B4_XCR2                      (R_SCI_B4_BASE + R_SCI_B_XCR2_OFFSET)
#define R_SCI_B4_XCR2_HA_L                 (R_SCI_B4_BASE + R_SCI_B_XCR2_HA_L_OFFSET)
#define R_SCI_B4_XCR2_BY_LL                (R_SCI_B4_BASE + R_SCI_B_XCR2_BY_LL_OFFSET)
#define R_SCI_B4_XCR2_BY_LH                (R_SCI_B4_BASE + R_SCI_B_XCR2_BY_LH_OFFSET)
#define R_SCI_B4_XCR2_HA_H                 (R_SCI_B4_BASE + R_SCI_B_XCR2_HA_H_OFFSET)
#define R_SCI_B4_XCR2_BY_HL                (R_SCI_B4_BASE + R_SCI_B_XCR2_BY_HL_OFFSET)
#define R_SCI_B4_XCR2_BY_HH                (R_SCI_B4_BASE + R_SCI_B_XCR2_BY_HH_OFFSET)
#define R_SCI_B4_CSR                       (R_SCI_B4_BASE + R_SCI_B_CSR_OFFSET)
#define R_SCI_B4_ISR                       (R_SCI_B4_BASE + R_SCI_B_ISR_OFFSET)
#define R_SCI_B4_FRSR                      (R_SCI_B4_BASE + R_SCI_B_FRSR_OFFSET)
#define R_SCI_B4_FTSR                      (R_SCI_B4_BASE + R_SCI_B_FTSR_OFFSET)
#define R_SCI_B4_MSR                       (R_SCI_B4_BASE + R_SCI_B_MSR_OFFSET)
#define R_SCI_B4_XSR0                      (R_SCI_B4_BASE + R_SCI_B_XSR0_OFFSET)
#define R_SCI_B4_XSR1                      (R_SCI_B4_BASE + R_SCI_B_XSR1_OFFSET)
#define R_SCI_B4_CFCLR                     (R_SCI_B4_BASE + R_SCI_B_CFCLR_OFFSET)
#define R_SCI_B4_CFCLR_HA_L                (R_SCI_B4_BASE + R_SCI_B_CFCLR_HA_L_OFFSET)
#define R_SCI_B4_CFCLR_BY_LL               (R_SCI_B4_BASE + R_SCI_B_CFCLR_BY_LL_OFFSET)
#define R_SCI_B4_CFCLR_HA_H                (R_SCI_B4_BASE + R_SCI_B_CFCLR_HA_H_OFFSET)
#define R_SCI_B4_CFCLR_BY_HL               (R_SCI_B4_BASE + R_SCI_B_CFCLR_BY_HL_OFFSET)
#define R_SCI_B4_CFCLR_BY_HH               (R_SCI_B4_BASE + R_SCI_B_CFCLR_BY_HH_OFFSET)
#define R_SCI_B4_ICFCLR                    (R_SCI_B4_BASE + R_SCI_B_ICFCLR_OFFSET)
#define R_SCI_B4_ICFCLR_HA_L               (R_SCI_B4_BASE + R_SCI_B_ICFCLR_HA_L_OFFSET)
#define R_SCI_B4_ICFCLR_BY_LL              (R_SCI_B4_BASE + R_SCI_B_ICFCLR_BY_LL_OFFSET)
#define R_SCI_B4_FFCLR                     (R_SCI_B4_BASE + R_SCI_B_FFCLR_OFFSET)
#define R_SCI_B4_FFCLR_HA_L                (R_SCI_B4_BASE + R_SCI_B_FFCLR_HA_L_OFFSET)
#define R_SCI_B4_FFCLR_BY_LL               (R_SCI_B4_BASE + R_SCI_B_FFCLR_BY_LL_OFFSET)
#define R_SCI_B4_MFCLR                     (R_SCI_B4_BASE + R_SCI_B_MFCLR_OFFSET)
#define R_SCI_B4_MFCLR_HA_L                (R_SCI_B4_BASE + R_SCI_B_MFCLR_HA_L_OFFSET)
#define R_SCI_B4_MFCLR_BY_LL               (R_SCI_B4_BASE + R_SCI_B_MFCLR_BY_LL_OFFSET)
#define R_SCI_B4_XFCLR                     (R_SCI_B4_BASE + R_SCI_B_XFCLR_OFFSET)
#define R_SCI_B4_XFCLR_HA_L                (R_SCI_B4_BASE + R_SCI_B_XFCLR_HA_L_OFFSET)
#define R_SCI_B4_XFCLR_BY_LH               (R_SCI_B4_BASE + R_SCI_B_XFCLR_BY_LH_OFFSET)

/* SCI_B9 Registers */

#define R_SCI_B9_RDR                       (R_SCI_B9_BASE + R_SCI_B_RDR_OFFSET)
#define R_SCI_B9_TDR                       (R_SCI_B9_BASE + R_SCI_B_TDR_OFFSET)
#define R_SCI_B9_TDR_HA_L                  (R_SCI_B9_BASE + R_SCI_B_TDR_HA_L_OFFSET)
#define R_SCI_B9_TDR_BY_LL                 (R_SCI_B9_BASE + R_SCI_B_TDR_BY_LL_OFFSET)
#define R_SCI_B9_TDR_BY_LH                 (R_SCI_B9_BASE + R_SCI_B_TDR_BY_LH_OFFSET)
#define R_SCI_B9_CCR0                      (R_SCI_B9_BASE + R_SCI_B_CCR0_OFFSET)
#define R_SCI_B9_CCR0_HA_L                 (R_SCI_B9_BASE + R_SCI_B_CCR0_HA_L_OFFSET)
#define R_SCI_B9_CCR0_BY_LL                (R_SCI_B9_BASE + R_SCI_B_CCR0_BY_LL_OFFSET)
#define R_SCI_B9_CCR0_BY_LH                (R_SCI_B9_BASE + R_SCI_B_CCR0_BY_LH_OFFSET)
#define R_SCI_B9_CCR0_HA_H                 (R_SCI_B9_BASE + R_SCI_B_CCR0_HA_H_OFFSET)
#define R_SCI_B9_CCR0_BY_HL                (R_SCI_B9_BASE + R_SCI_B_CCR0_BY_HL_OFFSET)
#define R_SCI_B9_CCR0_BY_HH                (R_SCI_B9_BASE + R_SCI_B_CCR0_BY_HH_OFFSET)
#define R_SCI_B9_CCR1                      (R_SCI_B9_BASE + R_SCI_B_CCR1_OFFSET)
#define R_SCI_B9_CCR1_HA_L                 (R_SCI_B9_BASE + R_SCI_B_CCR1_HA_L_OFFSET)
#define R_SCI_B9_CCR1_BY_LL                (R_SCI_B9_BASE + R_SCI_B_CCR1_BY_LL_OFFSET)
#define R_SCI_B9_CCR1_BY_LH                (R_SCI_B9_BASE + R_SCI_B_CCR1_BY_LH_OFFSET)
#define R_SCI_B9_CCR1_HA_H                 (R_SCI_B9_BASE + R_SCI_B_CCR1_HA_H_OFFSET)
#define R_SCI_B9_CCR1_BY_HL                (R_SCI_B9_BASE + R_SCI_B_CCR1_BY_HL_OFFSET)
#define R_SCI_B9_CCR1_BY_HH                (R_SCI_B9_BASE + R_SCI_B_CCR1_BY_HH_OFFSET)
#define R_SCI_B9_CCR2                      (R_SCI_B9_BASE + R_SCI_B_CCR2_OFFSET)
#define R_SCI_B9_CCR2_HA_L                 (R_SCI_B9_BASE + R_SCI_B_CCR2_HA_L_OFFSET)
#define R_SCI_B9_CCR2_BY_LL                (R_SCI_B9_BASE + R_SCI_B_CCR2_BY_LL_OFFSET)
#define R_SCI_B9_CCR2_BY_LH                (R_SCI_B9_BASE + R_SCI_B_CCR2_BY_LH_OFFSET)
#define R_SCI_B9_CCR2_HA_H                 (R_SCI_B9_BASE + R_SCI_B_CCR2_HA_H_OFFSET)
#define R_SCI_B9_CCR2_BY_HL                (R_SCI_B9_BASE + R_SCI_B_CCR2_BY_HL_OFFSET)
#define R_SCI_B9_CCR2_BY_HH                (R_SCI_B9_BASE + R_SCI_B_CCR2_BY_HH_OFFSET)
#define R_SCI_B9_CCR3                      (R_SCI_B9_BASE + R_SCI_B_CCR3_OFFSET)
#define R_SCI_B9_CCR3_HA_L                 (R_SCI_B9_BASE + R_SCI_B_CCR3_HA_L_OFFSET)
#define R_SCI_B9_CCR3_BY_LL                (R_SCI_B9_BASE + R_SCI_B_CCR3_BY_LL_OFFSET)
#define R_SCI_B9_CCR3_BY_LH                (R_SCI_B9_BASE + R_SCI_B_CCR3_BY_LH_OFFSET)
#define R_SCI_B9_CCR3_HA_H                 (R_SCI_B9_BASE + R_SCI_B_CCR3_HA_H_OFFSET)
#define R_SCI_B9_CCR3_BY_HL                (R_SCI_B9_BASE + R_SCI_B_CCR3_BY_HL_OFFSET)
#define R_SCI_B9_CCR3_BY_HH                (R_SCI_B9_BASE + R_SCI_B_CCR3_BY_HH_OFFSET)
#define R_SCI_B9_CCR4                      (R_SCI_B9_BASE + R_SCI_B_CCR4_OFFSET)
#define R_SCI_B9_CCR4_HA_L                 (R_SCI_B9_BASE + R_SCI_B_CCR4_HA_L_OFFSET)
#define R_SCI_B9_CCR4_BY_LL                (R_SCI_B9_BASE + R_SCI_B_CCR4_BY_LL_OFFSET)
#define R_SCI_B9_CCR4_BY_LH                (R_SCI_B9_BASE + R_SCI_B_CCR4_BY_LH_OFFSET)
#define R_SCI_B9_CCR4_HA_H                 (R_SCI_B9_BASE + R_SCI_B_CCR4_HA_H_OFFSET)
#define R_SCI_B9_CCR4_BY_HL                (R_SCI_B9_BASE + R_SCI_B_CCR4_BY_HL_OFFSET)
#define R_SCI_B9_CCR4_BY_HH                (R_SCI_B9_BASE + R_SCI_B_CCR4_BY_HH_OFFSET)
#define R_SCI_B9_CESR                      (R_SCI_B9_BASE + R_SCI_B_CESR_OFFSET)
#define R_SCI_B9_HCR                       (R_SCI_B9_BASE + R_SCI_B_HCR_OFFSET)
#define R_SCI_B9_ICR                       (R_SCI_B9_BASE + R_SCI_B_ICR_OFFSET)
#define R_SCI_B9_ICR_HA_L                  (R_SCI_B9_BASE + R_SCI_B_ICR_HA_L_OFFSET)
#define R_SCI_B9_ICR_BY_LL                 (R_SCI_B9_BASE + R_SCI_B_ICR_BY_LL_OFFSET)
#define R_SCI_B9_ICR_BY_LH                 (R_SCI_B9_BASE + R_SCI_B_ICR_BY_LH_OFFSET)
#define R_SCI_B9_ICR_HA_H                  (R_SCI_B9_BASE + R_SCI_B_ICR_HA_H_OFFSET)
#define R_SCI_B9_ICR_BY_HL                 (R_SCI_B9_BASE + R_SCI_B_ICR_BY_HL_OFFSET)
#define R_SCI_B9_FCR                       (R_SCI_B9_BASE + R_SCI_B_FCR_OFFSET)
#define R_SCI_B9_FCR_HA_L                  (R_SCI_B9_BASE + R_SCI_B_FCR_HA_L_OFFSET)
#define R_SCI_B9_FCR_BY_LL                 (R_SCI_B9_BASE + R_SCI_B_FCR_BY_LL_OFFSET)
#define R_SCI_B9_FCR_BY_LH                 (R_SCI_B9_BASE + R_SCI_B_FCR_BY_LH_OFFSET)
#define R_SCI_B9_FCR_HA_H                  (R_SCI_B9_BASE + R_SCI_B_FCR_HA_H_OFFSET)
#define R_SCI_B9_FCR_BY_HL                 (R_SCI_B9_BASE + R_SCI_B_FCR_BY_HL_OFFSET)
#define R_SCI_B9_FCR_BY_HH                 (R_SCI_B9_BASE + R_SCI_B_FCR_BY_HH_OFFSET)
#define R_SCI_B9_MCR                       (R_SCI_B9_BASE + R_SCI_B_MCR_OFFSET)
#define R_SCI_B9_MCR_HA_L                  (R_SCI_B9_BASE + R_SCI_B_MCR_HA_L_OFFSET)
#define R_SCI_B9_MCR_BY_LL                 (R_SCI_B9_BASE + R_SCI_B_MCR_BY_LL_OFFSET)
#define R_SCI_B9_MCR_BY_LH                 (R_SCI_B9_BASE + R_SCI_B_MCR_BY_LH_OFFSET)
#define R_SCI_B9_MCR_HA_H                  (R_SCI_B9_BASE + R_SCI_B_MCR_HA_H_OFFSET)
#define R_SCI_B9_MCR_BY_HL                 (R_SCI_B9_BASE + R_SCI_B_MCR_BY_HL_OFFSET)
#define R_SCI_B9_MCR_BY_HH                 (R_SCI_B9_BASE + R_SCI_B_MCR_BY_HH_OFFSET)
#define R_SCI_B9_DCR                       (R_SCI_B9_BASE + R_SCI_B_DCR_OFFSET)
#define R_SCI_B9_DCR_HA_L                  (R_SCI_B9_BASE + R_SCI_B_DCR_HA_L_OFFSET)
#define R_SCI_B9_DCR_BY_LL                 (R_SCI_B9_BASE + R_SCI_B_DCR_BY_LL_OFFSET)
#define R_SCI_B9_DCR_BY_LH                 (R_SCI_B9_BASE + R_SCI_B_DCR_BY_LH_OFFSET)
#define R_SCI_B9_DCR_HA_H                  (R_SCI_B9_BASE + R_SCI_B_DCR_HA_H_OFFSET)
#define R_SCI_B9_DCR_BY_HL                 (R_SCI_B9_BASE + R_SCI_B_DCR_BY_HL_OFFSET)
#define R_SCI_B9_XCR0                      (R_SCI_B9_BASE + R_SCI_B_XCR0_OFFSET)
#define R_SCI_B9_XCR0_HA_L                 (R_SCI_B9_BASE + R_SCI_B_XCR0_HA_L_OFFSET)
#define R_SCI_B9_XCR0_BY_LL                (R_SCI_B9_BASE + R_SCI_B_XCR0_BY_LL_OFFSET)
#define R_SCI_B9_XCR0_BY_LH                (R_SCI_B9_BASE + R_SCI_B_XCR0_BY_LH_OFFSET)
#define R_SCI_B9_XCR0_HA_H                 (R_SCI_B9_BASE + R_SCI_B_XCR0_HA_H_OFFSET)
#define R_SCI_B9_XCR0_BY_HL                (R_SCI_B9_BASE + R_SCI_B_XCR0_BY_HL_OFFSET)
#define R_SCI_B9_XCR0_BY_HH                (R_SCI_B9_BASE + R_SCI_B_XCR0_BY_HH_OFFSET)
#define R_SCI_B9_XCR1                      (R_SCI_B9_BASE + R_SCI_B_XCR1_OFFSET)
#define R_SCI_B9_XCR1_HA_L                 (R_SCI_B9_BASE + R_SCI_B_XCR1_HA_L_OFFSET)
#define R_SCI_B9_XCR1_BY_LL                (R_SCI_B9_BASE + R_SCI_B_XCR1_BY_LL_OFFSET)
#define R_SCI_B9_XCR1_BY_LH                (R_SCI_B9_BASE + R_SCI_B_XCR1_BY_LH_OFFSET)
#define R_SCI_B9_XCR1_HA_H                 (R_SCI_B9_BASE + R_SCI_B_XCR1_HA_H_OFFSET)
#define R_SCI_B9_XCR1_BY_HL                (R_SCI_B9_BASE + R_SCI_B_XCR1_BY_HL_OFFSET)
#define R_SCI_B9_XCR1_BY_HH                (R_SCI_B9_BASE + R_SCI_B_XCR1_BY_HH_OFFSET)
#define R_SCI_B9_XCR2                      (R_SCI_B9_BASE + R_SCI_B_XCR2_OFFSET)
#define R_SCI_B9_XCR2_HA_L                 (R_SCI_B9_BASE + R_SCI_B_XCR2_HA_L_OFFSET)
#define R_SCI_B9_XCR2_BY_LL                (R_SCI_B9_BASE + R_SCI_B_XCR2_BY_LL_OFFSET)
#define R_SCI_B9_XCR2_BY_LH                (R_SCI_B9_BASE + R_SCI_B_XCR2_BY_LH_OFFSET)
#define R_SCI_B9_XCR2_HA_H                 (R_SCI_B9_BASE + R_SCI_B_XCR2_HA_H_OFFSET)
#define R_SCI_B9_XCR2_BY_HL                (R_SCI_B9_BASE + R_SCI_B_XCR2_BY_HL_OFFSET)
#define R_SCI_B9_XCR2_BY_HH                (R_SCI_B9_BASE + R_SCI_B_XCR2_BY_HH_OFFSET)
#define R_SCI_B9_CSR                       (R_SCI_B9_BASE + R_SCI_B_CSR_OFFSET)
#define R_SCI_B9_ISR                       (R_SCI_B9_BASE + R_SCI_B_ISR_OFFSET)
#define R_SCI_B9_FRSR                      (R_SCI_B9_BASE + R_SCI_B_FRSR_OFFSET)
#define R_SCI_B9_FTSR                      (R_SCI_B9_BASE + R_SCI_B_FTSR_OFFSET)
#define R_SCI_B9_MSR                       (R_SCI_B9_BASE + R_SCI_B_MSR_OFFSET)
#define R_SCI_B9_XSR0                      (R_SCI_B9_BASE + R_SCI_B_XSR0_OFFSET)
#define R_SCI_B9_XSR1                      (R_SCI_B9_BASE + R_SCI_B_XSR1_OFFSET)
#define R_SCI_B9_CFCLR                     (R_SCI_B9_BASE + R_SCI_B_CFCLR_OFFSET)
#define R_SCI_B9_CFCLR_HA_L                (R_SCI_B9_BASE + R_SCI_B_CFCLR_HA_L_OFFSET)
#define R_SCI_B9_CFCLR_BY_LL               (R_SCI_B9_BASE + R_SCI_B_CFCLR_BY_LL_OFFSET)
#define R_SCI_B9_CFCLR_HA_H                (R_SCI_B9_BASE + R_SCI_B_CFCLR_HA_H_OFFSET)
#define R_SCI_B9_CFCLR_BY_HL               (R_SCI_B9_BASE + R_SCI_B_CFCLR_BY_HL_OFFSET)
#define R_SCI_B9_CFCLR_BY_HH               (R_SCI_B9_BASE + R_SCI_B_CFCLR_BY_HH_OFFSET)
#define R_SCI_B9_ICFCLR                    (R_SCI_B9_BASE + R_SCI_B_ICFCLR_OFFSET)
#define R_SCI_B9_ICFCLR_HA_L               (R_SCI_B9_BASE + R_SCI_B_ICFCLR_HA_L_OFFSET)
#define R_SCI_B9_ICFCLR_BY_LL              (R_SCI_B9_BASE + R_SCI_B_ICFCLR_BY_LL_OFFSET)
#define R_SCI_B9_FFCLR                     (R_SCI_B9_BASE + R_SCI_B_FFCLR_OFFSET)
#define R_SCI_B9_FFCLR_HA_L                (R_SCI_B9_BASE + R_SCI_B_FFCLR_HA_L_OFFSET)
#define R_SCI_B9_FFCLR_BY_LL               (R_SCI_B9_BASE + R_SCI_B_FFCLR_BY_LL_OFFSET)
#define R_SCI_B9_MFCLR                     (R_SCI_B9_BASE + R_SCI_B_MFCLR_OFFSET)
#define R_SCI_B9_MFCLR_HA_L                (R_SCI_B9_BASE + R_SCI_B_MFCLR_HA_L_OFFSET)
#define R_SCI_B9_MFCLR_BY_LL               (R_SCI_B9_BASE + R_SCI_B_MFCLR_BY_LL_OFFSET)
#define R_SCI_B9_XFCLR                     (R_SCI_B9_BASE + R_SCI_B_XFCLR_OFFSET)
#define R_SCI_B9_XFCLR_HA_L                (R_SCI_B9_BASE + R_SCI_B_XFCLR_HA_L_OFFSET)
#define R_SCI_B9_XFCLR_BY_LH               (R_SCI_B9_BASE + R_SCI_B_XFCLR_BY_LH_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Received Data Register (32-bits) *****************************************/

#define R_SCI_B_RDR_RDAT_SHIFT (0)
#define R_SCI_B_RDR_RDAT_MASK (0x1ff)
#define R_SCI_B_RDR_MPB (1 <<  9)   /* 200: Multiprocessor bit flag */
#define R_SCI_B_RDR_DR (1 << 10)    /* 400: Receive data ready flag */
#define R_SCI_B_RDR_FPER (1 << 11)  /* 800: Parity error flag */
#define R_SCI_B_RDR_FFER (1 << 12)  /* 1000: Framing error flag */
#define R_SCI_B_RDR_ORER (1 << 24)  /* 1000000: Overrun error flag */
#define R_SCI_B_RDR_PER (1 << 27)   /* 8000000: Parity Error Flag */
#define R_SCI_B_RDR_FER (1 << 28)   /* 10000000: Framing Error Flag */

/* Transmission Data Register (32-bits) *************************************/

#define R_SCI_B_TDR_TDAT_SHIFT (0)
#define R_SCI_B_TDR_TDAT_MASK (0x1ff)
#define R_SCI_B_TDR_MPBT (1 <<  9)   /* 200: Multi-processor transfer bit flag */
#define R_SCI_B_TDR_TSYNC (1 << 12)  /* 1000: Transmit sync data bit. */

/* Transmission Data Register (16-bits) *************************************/

#define R_SCI_B_TDR_HA_L_TDAT_SHIFT (0)
#define R_SCI_B_TDR_HA_L_TDAT_MASK (0x1ff)
#define R_SCI_B_TDR_HA_L_MPBT (1 <<  9)   /* 200: Multi-processor transfer bit flag */
#define R_SCI_B_TDR_HA_L_TSYNC (1 << 12)  /* 1000: Transmit sync data bit. */

/* Transmission Data Register (8-bits) **************************************/

#define R_SCI_B_TDR_BY_LL_TDAT_SHIFT (0)
#define R_SCI_B_TDR_BY_LL_TDAT_MASK (0xff)

/* Transmission Data Register (8-bits) **************************************/

#define R_SCI_B_TDR_BY_LH_TDAT (1 <<  0)   /* 01: Transmission data. */
#define R_SCI_B_TDR_BY_LH_MPBT (1 <<  1)   /* 02: Multi-processor transfer bit flag */
#define R_SCI_B_TDR_BY_LH_TSYNC (1 <<  4)  /* 10: Transmit sync data bit. */

/* Common Control Register 0 (32-bits) **************************************/

#define R_SCI_B_CCR0_RE (1 <<  0)     /* 01: Receive Enable */
#define R_SCI_B_CCR0_TE (1 <<  4)     /* 10: Transmit Enable */
#define R_SCI_B_CCR0_MPIE (1 <<  8)   /* 100: Multi-Processor Interrupt Enable */
#define R_SCI_B_CCR0_DCME (1 <<  9)   /* 200: Data Compare Match Enable */
#define R_SCI_B_CCR0_IDSEL (1 << 10)  /* 400: ID frame select Bit */
#define R_SCI_B_CCR0_RIE (1 << 16)    /* 10000: Receive Interrupt Enable */
#define R_SCI_B_CCR0_TIE (1 << 20)    /* 100000: Transmit Interrupt Enable */
#define R_SCI_B_CCR0_TEIE (1 << 21)   /* 200000: Transmit End Interrupt Enable */
#define R_SCI_B_CCR0_SSE (1 << 24)    /* 1000000: SSn# Pin Function Enable */

/* Common Control Register 0 (16-bits) **************************************/

#define R_SCI_B_CCR0_HA_L_RE (1 <<  0)     /* 01: Receive Enable */
#define R_SCI_B_CCR0_HA_L_TE (1 <<  4)     /* 10: Transmit Enable */
#define R_SCI_B_CCR0_HA_L_MPIE (1 <<  8)   /* 100: Multi-Processor Interrupt Enable */
#define R_SCI_B_CCR0_HA_L_DCME (1 <<  9)   /* 200: Data Compare Match Enable */
#define R_SCI_B_CCR0_HA_L_IDSEL (1 << 10)  /* 400: ID frame select Bit */

/* Common Control Register 0 (8-bits) ***************************************/

#define R_SCI_B_CCR0_BY_LL_RE (1 <<  0)  /* 01: Receive Enable */
#define R_SCI_B_CCR0_BY_LL_TE (1 <<  4)  /* 10: Transmit Enable */

/* Common Control Register 0 (8-bits) ***************************************/

#define R_SCI_B_CCR0_BY_LH_MPIE (1 <<  0)   /* 01: Multi-Processor Interrupt Enable */
#define R_SCI_B_CCR0_BY_LH_DCME (1 <<  1)   /* 02: Data Compare Match Enable */
#define R_SCI_B_CCR0_BY_LH_IDSEL (1 <<  2)  /* 04: ID frame select Bit */

/* Common Control Register 0 (16-bits) **************************************/

#define R_SCI_B_CCR0_HA_H_RIE (1 <<  0)   /* 01: Receive Interrupt Enable */
#define R_SCI_B_CCR0_HA_H_TIE (1 <<  4)   /* 10: Transmit Interrupt Enable */
#define R_SCI_B_CCR0_HA_H_TEIE (1 <<  5)  /* 20: Transmit End Interrupt Enable */
#define R_SCI_B_CCR0_HA_H_SSE (1 <<  8)   /* 100: SSn# Pin Function Enable */

/* Common Control Register 0 (8-bits) ***************************************/

#define R_SCI_B_CCR0_BY_HL_RIE (1 <<  0)   /* 01: Receive Interrupt Enable */
#define R_SCI_B_CCR0_BY_HL_TIE (1 <<  4)   /* 10: Transmit Interrupt Enable */
#define R_SCI_B_CCR0_BY_HL_TEIE (1 <<  5)  /* 20: Transmit End Interrupt Enable */

/* Common Control Register 0 (8-bits) ***************************************/

#define R_SCI_B_CCR0_BY_HH_SSE (1 <<  0)  /* 01: SSn# Pin Function Enable */

/* Common Control Register 1 (32-bits) **************************************/

#define R_SCI_B_CCR1_CTSE (1 <<  0)                              /* 01: CTS Enable */
#define R_SCI_B_CCR1_CTSPEN (1 <<  1)                            /* 02: CTS external terminal enable bit. */
#define R_SCI_B_CCR1_SPB2DT (1 <<  4)                            /* 10: Serial port break data select bit */
#define R_SCI_B_CCR1_SPB2IO (1 <<  5)                            /* 20: Serial port break I/O bit */
#define R_SCI_B_CCR1_PE (1 <<  8)                                /* 100: Parity Enable */
#define R_SCI_B_CCR1_PM (1 <<  9)                                /* 200: Parity Mode */
#define R_SCI_B_CCR1_TINV (1 << 12)                              /* 1000: TxD invert bit */
#define R_SCI_B_CCR1_RINV (1 << 13)                              /* 2000: RxD invert bit */
#define R_SCI_B_CCR1_SPLP (1 << 16)                              /* 10000: Serial communication Port LoopBack */
#define R_SCI_B_CCR1_SHARPS (1 << 20)                            /* 100000: TxD/RxD Pin Multiplexing Select */
#define R_SCI_B_CCR1_NFCS_SHIFT (24)
#define R_SCI_B_CCR1_NFCS_MASK (0x7)
#  define R_SCI_B_CCR1_NFCS_V000 (0 << R_SCI_B_CCR1_NFCS_SHIFT)  /* The base clock divided by 1 is used with the noise filter. */
#  define R_SCI_B_CCR1_NFCS_V001 (1 << R_SCI_B_CCR1_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 1 is used */
#  define R_SCI_B_CCR1_NFCS_V010 (2 << R_SCI_B_CCR1_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 2 is used */
#  define R_SCI_B_CCR1_NFCS_V011 (3 << R_SCI_B_CCR1_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 4 is used */
#  define R_SCI_B_CCR1_NFCS_V100 (4 << R_SCI_B_CCR1_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 8 is used */
#define R_SCI_B_CCR1_NFEN (1 << 28)                              /* 10000000: Digital Noise Filter Function Enable */

/* Common Control Register 1 (16-bits) **************************************/

#define R_SCI_B_CCR1_HA_L_CTSE (1 <<  0)    /* 01: CTS Enable */
#define R_SCI_B_CCR1_HA_L_CTSPEN (1 <<  1)  /* 02: CTS external terminal enable bit. */
#define R_SCI_B_CCR1_HA_L_SPB2DT (1 <<  4)  /* 10: Serial port break data select bit */
#define R_SCI_B_CCR1_HA_L_SPB2IO (1 <<  5)  /* 20: Serial port break I/O bit */
#define R_SCI_B_CCR1_HA_L_PE (1 <<  8)      /* 100: Parity Enable */
#define R_SCI_B_CCR1_HA_L_PM (1 <<  9)      /* 200: Parity Mode */
#define R_SCI_B_CCR1_HA_L_TINV (1 << 12)    /* 1000: TxD invert bit */
#define R_SCI_B_CCR1_HA_L_RINV (1 << 13)    /* 2000: RxD invert bit */

/* Common Control Register 1 (8-bits) ***************************************/

#define R_SCI_B_CCR1_BY_LL_CTSE (1 <<  0)    /* 01: CTS Enable */
#define R_SCI_B_CCR1_BY_LL_CTSPEN (1 <<  1)  /* 02: CTS external terminal enable bit. */
#define R_SCI_B_CCR1_BY_LL_SPB2DT (1 <<  4)  /* 10: Serial port break data select bit */
#define R_SCI_B_CCR1_BY_LL_SPB2IO (1 <<  5)  /* 20: Serial port break I/O bit */

/* Common Control Register 1 (8-bits) ***************************************/

#define R_SCI_B_CCR1_BY_LH_PE (1 <<  0)    /* 01: Parity Enable */
#define R_SCI_B_CCR1_BY_LH_PM (1 <<  1)    /* 02: Parity Mode */
#define R_SCI_B_CCR1_BY_LH_TINV (1 <<  4)  /* 10: TxD invert bit */
#define R_SCI_B_CCR1_BY_LH_RINV (1 <<  5)  /* 20: RxD invert bit */

/* Common Control Register 1 (16-bits) **************************************/

#define R_SCI_B_CCR1_HA_H_SPLP (1 <<  0)                                   /* 01: Serial communication Port LoopBack */
#define R_SCI_B_CCR1_HA_H_SHARPS (1 <<  4)                                 /* 10: TxD/RxD Pin Multiplexing Select */
#define R_SCI_B_CCR1_HA_H_NFCS_SHIFT (8)
#define R_SCI_B_CCR1_HA_H_NFCS_MASK (0x7)
#  define R_SCI_B_CCR1_HA_H_NFCS_V000 (0 << R_SCI_B_CCR1_HA_H_NFCS_SHIFT)  /* The base clock divided by 1 is used with the noise filter. */
#  define R_SCI_B_CCR1_HA_H_NFCS_V001 (1 << R_SCI_B_CCR1_HA_H_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 1 is used */
#  define R_SCI_B_CCR1_HA_H_NFCS_V010 (2 << R_SCI_B_CCR1_HA_H_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 2 is used */
#  define R_SCI_B_CCR1_HA_H_NFCS_V011 (3 << R_SCI_B_CCR1_HA_H_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 4 is used */
#  define R_SCI_B_CCR1_HA_H_NFCS_V100 (4 << R_SCI_B_CCR1_HA_H_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 8 is used */
#define R_SCI_B_CCR1_HA_H_NFEN (1 << 12)                                   /* 1000: Digital Noise Filter Function Enable */

/* Common Control Register 1 (8-bits) ***************************************/

#define R_SCI_B_CCR1_BY_HL_SPLP (1 <<  0)    /* 01: Serial communication Port LoopBack */
#define R_SCI_B_CCR1_BY_HL_SHARPS (1 <<  4)  /* 10: TxD/RxD Pin Multiplexing Select */

/* Common Control Register 1 (8-bits) ***************************************/

#define R_SCI_B_CCR1_BY_HH_NFCS_SHIFT (0)
#define R_SCI_B_CCR1_BY_HH_NFCS_MASK (0x7)
#  define R_SCI_B_CCR1_BY_HH_NFCS_V000 (0 << R_SCI_B_CCR1_BY_HH_NFCS_SHIFT)  /* The base clock divided by 1 is used with the noise filter. */
#  define R_SCI_B_CCR1_BY_HH_NFCS_V001 (1 << R_SCI_B_CCR1_BY_HH_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 1 is used */
#  define R_SCI_B_CCR1_BY_HH_NFCS_V010 (2 << R_SCI_B_CCR1_BY_HH_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 2 is used */
#  define R_SCI_B_CCR1_BY_HH_NFCS_V011 (3 << R_SCI_B_CCR1_BY_HH_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 4 is used */
#  define R_SCI_B_CCR1_BY_HH_NFCS_V100 (4 << R_SCI_B_CCR1_BY_HH_NFCS_SHIFT)  /* The on-chip baud rate generator clock divided by 8 is used */
#define R_SCI_B_CCR1_BY_HH_NFEN (1 <<  4)                                    /* 10: Digital Noise Filter Function Enable */

/* Common Control Register 2 (32-bits) **************************************/

#define R_SCI_B_CCR2_BCP_SHIFT (0)
#define R_SCI_B_CCR2_BCP_MASK (0x7)
#define R_SCI_B_CCR2_BGDM (1 <<  4)                                     /* 10: Baud Rate Generator Double-Speed Mode Select */
#define R_SCI_B_CCR2_ABCS (1 <<  5)                                     /* 20: Asynchronous Mode Base Clock Select */
#define R_SCI_B_CCR2_ABCSE (1 <<  6)                                    /* 40: Asynchronous Mode Base Clock Select1 */
#define R_SCI_B_CCR2_BRR_SHIFT (8)
#define R_SCI_B_CCR2_BRR_MASK (0xff)
#define R_SCI_B_CCR2_BRME (1 << 16)                                     /* 10000: Bit Modulation Enable */
#define R_SCI_B_CCR2_CKS_SHIFT (20)
#define R_SCI_B_CCR2_CKS_MASK (0x3)
#  define R_SCI_B_CCR2_CKS_TCLK_CLOCK (0 << R_SCI_B_CCR2_CKS_SHIFT)     /* TCLK clock */
#  define R_SCI_B_CCR2_CKS_TCLK_4_CLOCK (1 << R_SCI_B_CCR2_CKS_SHIFT)   /* TCLK/4 clock */
#  define R_SCI_B_CCR2_CKS_TCLK_16_CLOCK (2 << R_SCI_B_CCR2_CKS_SHIFT)  /* TCLK/16 clock */
#  define R_SCI_B_CCR2_CKS_TCLK_64_CLOCK (3 << R_SCI_B_CCR2_CKS_SHIFT)  /* TCLK/64 clock */
#define R_SCI_B_CCR2_MDDR_SHIFT (24)
#define R_SCI_B_CCR2_MDDR_MASK (0xff)

/* Common Control Register 2 (16-bits) **************************************/

#define R_SCI_B_CCR2_HA_L_BCP_SHIFT (0)
#define R_SCI_B_CCR2_HA_L_BCP_MASK (0x7)
#define R_SCI_B_CCR2_HA_L_BGDM (1 <<  4)   /* 10: Baud Rate Generator Double-Speed Mode Select */
#define R_SCI_B_CCR2_HA_L_ABCS (1 <<  5)   /* 20: Asynchronous Mode Base Clock Select */
#define R_SCI_B_CCR2_HA_L_ABCSE (1 <<  6)  /* 40: Asynchronous Mode Base Clock Select1 */
#define R_SCI_B_CCR2_HA_L_BRR_SHIFT (8)
#define R_SCI_B_CCR2_HA_L_BRR_MASK (0xff)

/* Common Control Register 2 (8-bits) ***************************************/

#define R_SCI_B_CCR2_BY_LL_BCP_SHIFT (0)
#define R_SCI_B_CCR2_BY_LL_BCP_MASK (0x7)
#define R_SCI_B_CCR2_BY_LL_BGDM (1 <<  4)   /* 10: Baud Rate Generator Double-Speed Mode Select */
#define R_SCI_B_CCR2_BY_LL_ABCS (1 <<  5)   /* 20: Asynchronous Mode Base Clock Select */
#define R_SCI_B_CCR2_BY_LL_ABCSE (1 <<  6)  /* 40: Asynchronous Mode Base Clock Select1 */

/* Common Control Register 2 (8-bits) ***************************************/

#define R_SCI_B_CCR2_BY_LH_BRR_SHIFT (0)
#define R_SCI_B_CCR2_BY_LH_BRR_MASK (0xff)

/* Common Control Register 2 (16-bits) **************************************/

#define R_SCI_B_CCR2_HA_H_BRME (1 <<  0)                                          /* 01: Bit Modulation Enable */
#define R_SCI_B_CCR2_HA_H_CKS_SHIFT (4)
#define R_SCI_B_CCR2_HA_H_CKS_MASK (0x3)
#  define R_SCI_B_CCR2_HA_H_CKS_TCLK_CLOCK (0 << R_SCI_B_CCR2_HA_H_CKS_SHIFT)     /* TCLK clock */
#  define R_SCI_B_CCR2_HA_H_CKS_TCLK_4_CLOCK (1 << R_SCI_B_CCR2_HA_H_CKS_SHIFT)   /* TCLK/4 clock */
#  define R_SCI_B_CCR2_HA_H_CKS_TCLK_16_CLOCK (2 << R_SCI_B_CCR2_HA_H_CKS_SHIFT)  /* TCLK/16 clock */
#  define R_SCI_B_CCR2_HA_H_CKS_TCLK_64_CLOCK (3 << R_SCI_B_CCR2_HA_H_CKS_SHIFT)  /* TCLK/64 clock */
#define R_SCI_B_CCR2_HA_H_MDDR_SHIFT (8)
#define R_SCI_B_CCR2_HA_H_MDDR_MASK (0xff)

/* Common Control Register 2 (8-bits) ***************************************/

#define R_SCI_B_CCR2_BY_HL_BRME (1 <<  0)                                           /* 01: Bit Modulation Enable */
#define R_SCI_B_CCR2_BY_HL_CKS_SHIFT (4)
#define R_SCI_B_CCR2_BY_HL_CKS_MASK (0x3)
#  define R_SCI_B_CCR2_BY_HL_CKS_TCLK_CLOCK (0 << R_SCI_B_CCR2_BY_HL_CKS_SHIFT)     /* TCLK clock */
#  define R_SCI_B_CCR2_BY_HL_CKS_TCLK_4_CLOCK (1 << R_SCI_B_CCR2_BY_HL_CKS_SHIFT)   /* TCLK/4 clock */
#  define R_SCI_B_CCR2_BY_HL_CKS_TCLK_16_CLOCK (2 << R_SCI_B_CCR2_BY_HL_CKS_SHIFT)  /* TCLK/16 clock */
#  define R_SCI_B_CCR2_BY_HL_CKS_TCLK_64_CLOCK (3 << R_SCI_B_CCR2_BY_HL_CKS_SHIFT)  /* TCLK/64 clock */

/* Common Control Register 2 (8-bits) ***************************************/

#define R_SCI_B_CCR2_BY_HH_MDDR_SHIFT (0)
#define R_SCI_B_CCR2_BY_HH_MDDR_MASK (0xff)

/* Common Control Register 3 (32-bits) **************************************/

#define R_SCI_B_CCR3_CPHA (1 <<  0)                                            /* 01: Clock Phase Setting */
#define R_SCI_B_CCR3_CPOL (1 <<  1)                                            /* 02: Clock Polarity Setting */
#define R_SCI_B_CCR3_BPEN (1 <<  7)                                            /* 80: Synchronizer ByPass Enable */
#define R_SCI_B_CCR3_CHR_SHIFT (8)
#define R_SCI_B_CCR3_CHR_MASK (0x3)
#  define R_SCI_B_CCR3_CHR_V00 (0 << R_SCI_B_CCR3_CHR_SHIFT)                   /* Transmit/receive in 9-bit length */
#  define R_SCI_B_CCR3_CHR_V01 (1 << R_SCI_B_CCR3_CHR_SHIFT)                   /* Transmit/receive in 9-bit length */
#  define R_SCI_B_CCR3_CHR_V10 (2 << R_SCI_B_CCR3_CHR_SHIFT)                   /* Transmit/receive in 8-bit length */
#  define R_SCI_B_CCR3_CHR_V11 (3 << R_SCI_B_CCR3_CHR_SHIFT)                   /* Transmit/receive in 7-bit length */
#define R_SCI_B_CCR3_LSBF (1 << 12)                                            /* 1000: LSB first select bit */
#define R_SCI_B_CCR3_SINV (1 << 13)                                            /* 2000: Transmitted/Received Data Invert */
#define R_SCI_B_CCR3_STP (1 << 14)                                             /* 4000: Stop Bit Length */
#define R_SCI_B_CCR3_RXDESEL (1 << 15)                                         /* 8000: Asynchronous Start Bit Edge Detection Select */
#define R_SCI_B_CCR3_MOD_SHIFT (16)
#define R_SCI_B_CCR3_MOD_MASK (0x7)
#  define R_SCI_B_CCR3_MOD_ASYNCHRONOUS (0 << R_SCI_B_CCR3_MOD_SHIFT)          /* Asynchronous */
#  define R_SCI_B_CCR3_MOD_SMART_CARD_INTERFACE (1 << R_SCI_B_CCR3_MOD_SHIFT)  /* Smart card interface */
#  define R_SCI_B_CCR3_MOD_CLOCK_SNCHRONOUS (2 << R_SCI_B_CCR3_MOD_SHIFT)      /* Clock snchronous */
#  define R_SCI_B_CCR3_MOD_SIMPLE_SPI (3 << R_SCI_B_CCR3_MOD_SHIFT)            /* Simple-SPI */
#  define R_SCI_B_CCR3_MOD_SIMPLE_I2C (4 << R_SCI_B_CCR3_MOD_SHIFT)            /* Simple-I2C */
#  define R_SCI_B_CCR3_MOD_MANCHESTER (5 << R_SCI_B_CCR3_MOD_SHIFT)            /* Manchester */
#  define R_SCI_B_CCR3_MOD_SIMPLE_LIN (6 << R_SCI_B_CCR3_MOD_SHIFT)            /* Simple-LIN */
#  define R_SCI_B_CCR3_MOD_PROHIBIT (7 << R_SCI_B_CCR3_MOD_SHIFT)              /* Prohibit */
#define R_SCI_B_CCR3_MP (1 << 19)                                              /* 80000: Multi-Processor mode */
#define R_SCI_B_CCR3_FM (1 << 20)                                              /* 100000: FIFO Mode Select */
#define R_SCI_B_CCR3_DEN (1 << 21)                                             /* 200000: Driver control Enable */
#define R_SCI_B_CCR3_CKE_SHIFT (24)
#define R_SCI_B_CCR3_CKE_MASK (0x3)
#  define R_SCI_B_CCR3_CKE_V00 (0 << R_SCI_B_CCR3_CKE_SHIFT)                   /* On-chip baud rate generator / On-chip baud rate generator The SCKn pin functions as I/O port. / Output disabled (The SCKn pin is available for use as an I/O port */
#  define R_SCI_B_CCR3_CKE_V01 (1 << R_SCI_B_CCR3_CKE_SHIFT)                   /* On-chip baud rate generator */
#  define R_SCI_B_CCR3_CKE_CLOCK_OUTPUT (1 << R_SCI_B_CCR3_CKE_SHIFT)          /* Clock output */
#  define R_SCI_B_CCR3_CKE_OUTPUT_FIXED_LOW (0 << R_SCI_B_CCR3_CKE_SHIFT)      /* Output fixed low */
#  define R_SCI_B_CCR3_CKE_OUTPUT_FIXED_HIGH (2 << R_SCI_B_CCR3_CKE_SHIFT)     /* Output fixed high */
#define R_SCI_B_CCR3_ACS0 (1 << 26)                                            /* 4000000: Asynchronous Mode Clock Source Select */
#define R_SCI_B_CCR3_GM (1 << 28)                                              /* 10000000: GSM Mode */
#define R_SCI_B_CCR3_BLK (1 << 29)                                             /* 20000000: Block transfer mode */

/* Common Control Register 3 (16-bits) **************************************/

#define R_SCI_B_CCR3_HA_L_CPHA (1 <<  0)                                /* 01: Clock Phase Setting */
#define R_SCI_B_CCR3_HA_L_CPOL (1 <<  1)                                /* 02: Clock Polarity Setting */
#define R_SCI_B_CCR3_HA_L_BPEN (1 <<  7)                                /* 80: Synchronizer ByPass Enable */
#define R_SCI_B_CCR3_HA_L_CHR_SHIFT (8)
#define R_SCI_B_CCR3_HA_L_CHR_MASK (0x3)
#  define R_SCI_B_CCR3_HA_L_CHR_V00 (0 << R_SCI_B_CCR3_HA_L_CHR_SHIFT)  /* Transmit/receive in 9-bit length */
#  define R_SCI_B_CCR3_HA_L_CHR_V01 (1 << R_SCI_B_CCR3_HA_L_CHR_SHIFT)  /* Transmit/receive in 9-bit length */
#  define R_SCI_B_CCR3_HA_L_CHR_V10 (2 << R_SCI_B_CCR3_HA_L_CHR_SHIFT)  /* Transmit/receive in 8-bit length */
#  define R_SCI_B_CCR3_HA_L_CHR_V11 (3 << R_SCI_B_CCR3_HA_L_CHR_SHIFT)  /* Transmit/receive in 7-bit length */
#define R_SCI_B_CCR3_HA_L_LSBF (1 << 12)                                /* 1000: LSB first select bit */
#define R_SCI_B_CCR3_HA_L_SINV (1 << 13)                                /* 2000: Transmitted/Received Data Invert */
#define R_SCI_B_CCR3_HA_L_STP (1 << 14)                                 /* 4000: Stop Bit Length */
#define R_SCI_B_CCR3_HA_L_RXDESEL (1 << 15)                             /* 8000: Asynchronous Start Bit Edge Detection Select */

/* Common Control Register 3 (8-bits) ***************************************/

#define R_SCI_B_CCR3_BY_LL_CPHA (1 <<  0)  /* 01: Clock Phase Setting */
#define R_SCI_B_CCR3_BY_LL_CPOL (1 <<  1)  /* 02: Clock Polarity Setting */
#define R_SCI_B_CCR3_BY_LL_BPEN (1 <<  7)  /* 80: Synchronizer ByPass Enable */

/* Common Control Register 3 (8-bits) ***************************************/

#define R_SCI_B_CCR3_BY_LH_CHR_SHIFT (0)
#define R_SCI_B_CCR3_BY_LH_CHR_MASK (0x3)
#  define R_SCI_B_CCR3_BY_LH_CHR_V00 (0 << R_SCI_B_CCR3_BY_LH_CHR_SHIFT)  /* Transmit/receive in 9-bit length */
#  define R_SCI_B_CCR3_BY_LH_CHR_V01 (1 << R_SCI_B_CCR3_BY_LH_CHR_SHIFT)  /* Transmit/receive in 9-bit length */
#  define R_SCI_B_CCR3_BY_LH_CHR_V10 (2 << R_SCI_B_CCR3_BY_LH_CHR_SHIFT)  /* Transmit/receive in 8-bit length */
#  define R_SCI_B_CCR3_BY_LH_CHR_V11 (3 << R_SCI_B_CCR3_BY_LH_CHR_SHIFT)  /* Transmit/receive in 7-bit length */
#define R_SCI_B_CCR3_BY_LH_LSBF (1 <<  4)                                 /* 10: LSB first select bit */
#define R_SCI_B_CCR3_BY_LH_SINV (1 <<  5)                                 /* 20: Transmitted/Received Data Invert */
#define R_SCI_B_CCR3_BY_LH_STP (1 <<  6)                                  /* 40: Stop Bit Length */
#define R_SCI_B_CCR3_BY_LH_RXDESEL (1 <<  7)                              /* 80: Asynchronous Start Bit Edge Detection Select */

/* Common Control Register 3 (16-bits) **************************************/

#define R_SCI_B_CCR3_HA_H_MOD_SHIFT (0)
#define R_SCI_B_CCR3_HA_H_MOD_MASK (0x7)
#  define R_SCI_B_CCR3_HA_H_MOD_ASYNCHRONOUS (0 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)          /* Asynchronous */
#  define R_SCI_B_CCR3_HA_H_MOD_SMART_CARD_INTERFACE (1 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)  /* Smart card interface */
#  define R_SCI_B_CCR3_HA_H_MOD_CLOCK_SNCHRONOUS (2 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)      /* Clock snchronous */
#  define R_SCI_B_CCR3_HA_H_MOD_SIMPLE_SPI (3 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)            /* Simple-SPI */
#  define R_SCI_B_CCR3_HA_H_MOD_SIMPLE_I2C (4 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)            /* Simple-I2C */
#  define R_SCI_B_CCR3_HA_H_MOD_MANCHESTER (5 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)            /* Manchester */
#  define R_SCI_B_CCR3_HA_H_MOD_SIMPLE_LIN (6 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)            /* Simple-LIN */
#  define R_SCI_B_CCR3_HA_H_MOD_PROHIBIT (7 << R_SCI_B_CCR3_HA_H_MOD_SHIFT)              /* Prohibit */
#define R_SCI_B_CCR3_HA_H_MP (1 <<  3)                                                   /* 08: Multi-Processor mode */
#define R_SCI_B_CCR3_HA_H_FM (1 <<  4)                                                   /* 10: FIFO Mode Select */
#define R_SCI_B_CCR3_HA_H_DEN (1 <<  5)                                                  /* 20: Driver control Enable */
#define R_SCI_B_CCR3_HA_H_CKE_SHIFT (8)
#define R_SCI_B_CCR3_HA_H_CKE_MASK (0x3)
#  define R_SCI_B_CCR3_HA_H_CKE_V00 (0 << R_SCI_B_CCR3_HA_H_CKE_SHIFT)                   /* On-chip baud rate generator / On-chip baud rate generator The SCKn pin functions as I/O port. / Output disabled (The SCKn pin is available for use as an I/O port */
#  define R_SCI_B_CCR3_HA_H_CKE_V01 (1 << R_SCI_B_CCR3_HA_H_CKE_SHIFT)                   /* On-chip baud rate generator */
#  define R_SCI_B_CCR3_HA_H_CKE_CLOCK_OUTPUT (1 << R_SCI_B_CCR3_HA_H_CKE_SHIFT)          /* Clock output */
#  define R_SCI_B_CCR3_HA_H_CKE_OUTPUT_FIXED_LOW (0 << R_SCI_B_CCR3_HA_H_CKE_SHIFT)      /* Output fixed low */
#  define R_SCI_B_CCR3_HA_H_CKE_OUTPUT_FIXED_HIGH (2 << R_SCI_B_CCR3_HA_H_CKE_SHIFT)     /* Output fixed high */
#define R_SCI_B_CCR3_HA_H_ACS0 (1 << 10)                                                 /* 400: Asynchronous Mode Clock Source Select */
#define R_SCI_B_CCR3_HA_H_GM (1 << 12)                                                   /* 1000: GSM Mode */
#define R_SCI_B_CCR3_HA_H_BLK (1 << 13)                                                  /* 2000: Block transfer mode */

/* Common Control Register 3 (8-bits) ***************************************/

#define R_SCI_B_CCR3_BY_HL_MOD_SHIFT (0)
#define R_SCI_B_CCR3_BY_HL_MOD_MASK (0x7)
#  define R_SCI_B_CCR3_BY_HL_MOD_ASYNCHRONOUS (0 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)          /* Asynchronous */
#  define R_SCI_B_CCR3_BY_HL_MOD_SMART_CARD_INTERFACE (1 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)  /* Smart card interface */
#  define R_SCI_B_CCR3_BY_HL_MOD_CLOCK_SNCHRONOUS (2 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)      /* Clock snchronous */
#  define R_SCI_B_CCR3_BY_HL_MOD_SIMPLE_SPI (3 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)            /* Simple-SPI */
#  define R_SCI_B_CCR3_BY_HL_MOD_SIMPLE_I2C (4 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)            /* Simple-I2C */
#  define R_SCI_B_CCR3_BY_HL_MOD_MANCHESTER (5 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)            /* Manchester */
#  define R_SCI_B_CCR3_BY_HL_MOD_SIMPLE_LIN (6 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)            /* Simple-LIN */
#  define R_SCI_B_CCR3_BY_HL_MOD_PROHIBIT (7 << R_SCI_B_CCR3_BY_HL_MOD_SHIFT)              /* Prohibit */
#define R_SCI_B_CCR3_BY_HL_MP (1 <<  3)                                                    /* 08: Multi-Processor mode */
#define R_SCI_B_CCR3_BY_HL_FM (1 <<  4)                                                    /* 10: FIFO Mode Select */
#define R_SCI_B_CCR3_BY_HL_DEN (1 <<  5)                                                   /* 20: Driver control Enable */

/* Common Control Register 3 (8-bits) ***************************************/

#define R_SCI_B_CCR3_BY_HH_CKE_SHIFT (0)
#define R_SCI_B_CCR3_BY_HH_CKE_MASK (0x3)
#  define R_SCI_B_CCR3_BY_HH_CKE_V00 (0 << R_SCI_B_CCR3_BY_HH_CKE_SHIFT)                /* On-chip baud rate generator / On-chip baud rate generator The SCKn pin functions as I/O port. / Output disabled (The SCKn pin is available for use as an I/O port */
#  define R_SCI_B_CCR3_BY_HH_CKE_V01 (1 << R_SCI_B_CCR3_BY_HH_CKE_SHIFT)                /* On-chip baud rate generator */
#  define R_SCI_B_CCR3_BY_HH_CKE_CLOCK_OUTPUT (1 << R_SCI_B_CCR3_BY_HH_CKE_SHIFT)       /* Clock output */
#  define R_SCI_B_CCR3_BY_HH_CKE_OUTPUT_FIXED_LOW (0 << R_SCI_B_CCR3_BY_HH_CKE_SHIFT)   /* Output fixed low */
#  define R_SCI_B_CCR3_BY_HH_CKE_OUTPUT_FIXED_HIGH (2 << R_SCI_B_CCR3_BY_HH_CKE_SHIFT)  /* Output fixed high */
#define R_SCI_B_CCR3_BY_HH_ACS0 (1 <<  2)                                               /* 04: Asynchronous Mode Clock Source Select */
#define R_SCI_B_CCR3_BY_HH_GM (1 <<  4)                                                 /* 10: GSM Mode */
#define R_SCI_B_CCR3_BY_HH_BLK (1 <<  5)                                                /* 20: Block transfer mode */

/* Common Control Register 4 (32-bits) **************************************/

#define R_SCI_B_CCR4_CMPD_SHIFT (0)
#define R_SCI_B_CCR4_CMPD_MASK (0x1ff)
#define R_SCI_B_CCR4_ASEN (1 << 16)                                     /* 10000: Adjustment enable bit for sampling timing */
#define R_SCI_B_CCR4_ATEN (1 << 17)                                     /* 20000: Adjustment enable bit for transmit waveform */
#define R_SCI_B_CCR4_AST_SHIFT (24)
#define R_SCI_B_CCR4_AST_MASK (0x7)
#  define R_SCI_B_CCR4_AST_V000 (0 << R_SCI_B_CCR4_AST_SHIFT)           /* No adjustment (The sampling at default timing) */
#define R_SCI_B_CCR4_AJD (1 << 27)                                      /* 8000000: Adjustment Direction for sampling timing */
#define R_SCI_B_CCR4_ATT_SHIFT (28)
#define R_SCI_B_CCR4_ATT_MASK (0x7)
#  define R_SCI_B_CCR4_ATT_NO_ADJUSTMENT (0 << R_SCI_B_CCR4_ATT_SHIFT)  /* No adjustment */
#define R_SCI_B_CCR4_AET (1 << 31)                                      /* 80000000: Adjustment Duty control level select */

/* Common Control Register 4 (16-bits) **************************************/

#define R_SCI_B_CCR4_HA_L_CMPD_SHIFT (0)
#define R_SCI_B_CCR4_HA_L_CMPD_MASK (0x1ff)

/* Common Control Register 4 (8-bits) ***************************************/

#define R_SCI_B_CCR4_BY_LL_CMPD_SHIFT (0)
#define R_SCI_B_CCR4_BY_LL_CMPD_MASK (0xff)

/* Common Control Register 4 (8-bits) ***************************************/

#define R_SCI_B_CCR4_BY_LH_CMPD (1 <<  0)  /* 01: Compare Match Data */

/* Common Control Register 4 (16-bits) **************************************/

#define R_SCI_B_CCR4_HA_H_ASEN (1 <<  0)                                          /* 01: Adjustment enable bit for sampling timing */
#define R_SCI_B_CCR4_HA_H_ATEN (1 <<  1)                                          /* 02: Adjustment enable bit for transmit waveform */
#define R_SCI_B_CCR4_HA_H_AST_SHIFT (8)
#define R_SCI_B_CCR4_HA_H_AST_MASK (0x7)
#  define R_SCI_B_CCR4_HA_H_AST_V000 (0 << R_SCI_B_CCR4_HA_H_AST_SHIFT)           /* No adjustment (The sampling at default timing) */
#define R_SCI_B_CCR4_HA_H_AJD (1 << 11)                                           /* 800: Adjustment Direction for sampling timing */
#define R_SCI_B_CCR4_HA_H_ATT_SHIFT (12)
#define R_SCI_B_CCR4_HA_H_ATT_MASK (0x7)
#  define R_SCI_B_CCR4_HA_H_ATT_NO_ADJUSTMENT (0 << R_SCI_B_CCR4_HA_H_ATT_SHIFT)  /* No adjustment */
#define R_SCI_B_CCR4_HA_H_AET (1 << 15)                                           /* 8000: Adjustment Duty control level select */

/* Common Control Register 4 (8-bits) ***************************************/

#define R_SCI_B_CCR4_BY_HL_ASEN (1 <<  0)  /* 01: Adjustment enable bit for sampling timing */
#define R_SCI_B_CCR4_BY_HL_ATEN (1 <<  1)  /* 02: Adjustment enable bit for transmit waveform */

/* Common Control Register 4 (8-bits) ***************************************/

#define R_SCI_B_CCR4_BY_HH_AST_SHIFT (0)
#define R_SCI_B_CCR4_BY_HH_AST_MASK (0x7)
#  define R_SCI_B_CCR4_BY_HH_AST_V000 (0 << R_SCI_B_CCR4_BY_HH_AST_SHIFT)           /* No adjustment (The sampling at default timing) */
#define R_SCI_B_CCR4_BY_HH_AJD (1 <<  3)                                            /* 08: Adjustment Direction for sampling timing */
#define R_SCI_B_CCR4_BY_HH_ATT_SHIFT (4)
#define R_SCI_B_CCR4_BY_HH_ATT_MASK (0x7)
#  define R_SCI_B_CCR4_BY_HH_ATT_NO_ADJUSTMENT (0 << R_SCI_B_CCR4_BY_HH_ATT_SHIFT)  /* No adjustment */
#define R_SCI_B_CCR4_BY_HH_AET (1 <<  7)                                            /* 80: Adjustment Duty control level select */

/* Communication Enable Status Register (8-bits) ****************************/

#define R_SCI_B_CESR_RIST (1 <<  0)  /* 01: Internal status of RE signal */
#define R_SCI_B_CESR_TIST (1 <<  4)  /* 10: Internal status of TE signal */

/* HBS valid mode Control Register (8-bits) *********************************/

#define R_SCI_B_HCR_HDEN (1 <<  0)  /* 01: HDC valid mode Enable */
#define R_SCI_B_HCR_HDOC (1 <<  2)  /* 04: HDC valid mode Output Control */
#define R_SCI_B_HCR_HDST (1 <<  3)  /* 08: HDC valid mode STart bit */
#define R_SCI_B_HCR_HDIC (1 <<  4)  /* 10: HDC valid mode start bit Initialize Control */

/* Simple-I2C Control Register (32-bits) ************************************/

#define R_SCI_B_ICR_IICDL_SHIFT (0)
#define R_SCI_B_ICR_IICDL_MASK (0x1f)
#  define R_SCI_B_ICR_IICDL_NO_OUTPUT_DELAY (0 << R_SCI_B_ICR_IICDL_SHIFT)          /* No output delay */
#define R_SCI_B_ICR_IICINTM (1 <<  8)                                               /* 100: I2C Interrupt Mode Select */
#define R_SCI_B_ICR_IICCSC (1 <<  9)                                                /* 200: Clock Synchronization */
#define R_SCI_B_ICR_IICACKT (1 << 13)                                               /* 2000: ACK Transmission Data */
#define R_SCI_B_ICR_IICSTAREQ (1 << 16)                                             /* 10000: Start Condition Generation */
#define R_SCI_B_ICR_IICRSTAREQ (1 << 17)                                            /* 20000: Restart Condition Generation */
#define R_SCI_B_ICR_IICSTPREQ (1 << 18)                                             /* 40000: Stop Condition Generation */
#define R_SCI_B_ICR_IICSDAS_SHIFT (20)
#define R_SCI_B_ICR_IICSDAS_MASK (0x3)
#  define R_SCI_B_ICR_IICSDAS_SERIAL_DATA_OUTPUT (0 << R_SCI_B_ICR_IICSDAS_SHIFT)   /* Serial data output */
#  define R_SCI_B_ICR_IICSDAS_V01 (1 << R_SCI_B_ICR_IICSDAS_SHIFT)                  /* Generate a start, restart, or stop condition. */
#  define R_SCI_B_ICR_IICSDAS_V10 (2 << R_SCI_B_ICR_IICSDAS_SHIFT)                  /* Output the low level on the SSDAn pin. */
#  define R_SCI_B_ICR_IICSDAS_V11 (3 << R_SCI_B_ICR_IICSDAS_SHIFT)                  /* Place the SSDAn pin in the high-impedance state. */
#define R_SCI_B_ICR_IICSCLS_SHIFT (22)
#define R_SCI_B_ICR_IICSCLS_MASK (0x3)
#  define R_SCI_B_ICR_IICSCLS_SERIAL_CLOCK_OUTPUT (0 << R_SCI_B_ICR_IICSCLS_SHIFT)  /* Serial clock output */
#  define R_SCI_B_ICR_IICSCLS_V01 (1 << R_SCI_B_ICR_IICSCLS_SHIFT)                  /* Generate a start, restart, or stop condition. */
#  define R_SCI_B_ICR_IICSCLS_V10 (2 << R_SCI_B_ICR_IICSCLS_SHIFT)                  /* Output the low level on the SSCLn pin. */
#  define R_SCI_B_ICR_IICSCLS_V11 (3 << R_SCI_B_ICR_IICSCLS_SHIFT)                  /* Place the SSCLn pin in the high-impedance state. */

/* Simple-I2C Control Register (16-bits) ************************************/

#define R_SCI_B_ICR_HA_L_IICDL_SHIFT (0)
#define R_SCI_B_ICR_HA_L_IICDL_MASK (0x1f)
#  define R_SCI_B_ICR_HA_L_IICDL_NO_OUTPUT_DELAY (0 << R_SCI_B_ICR_HA_L_IICDL_SHIFT)  /* No output delay */
#define R_SCI_B_ICR_HA_L_IICINTM (1 <<  8)                                            /* 100: I2C Interrupt Mode Select */
#define R_SCI_B_ICR_HA_L_IICCSC (1 <<  9)                                             /* 200: Clock Synchronization */
#define R_SCI_B_ICR_HA_L_IICACKT (1 << 13)                                            /* 2000: ACK Transmission Data */

/* Simple-I2C Control Register (8-bits) *************************************/

#define R_SCI_B_ICR_BY_LL_IICDL_SHIFT (0)
#define R_SCI_B_ICR_BY_LL_IICDL_MASK (0x1f)
#  define R_SCI_B_ICR_BY_LL_IICDL_NO_OUTPUT_DELAY (0 << R_SCI_B_ICR_BY_LL_IICDL_SHIFT)  /* No output delay */

/* Simple-I2C Control Register (8-bits) *************************************/

#define R_SCI_B_ICR_BY_LH_IICINTM (1 <<  0)  /* 01: I2C Interrupt Mode Select */
#define R_SCI_B_ICR_BY_LH_IICCSC (1 <<  1)   /* 02: Clock Synchronization */
#define R_SCI_B_ICR_BY_LH_IICACKT (1 <<  5)  /* 20: ACK Transmission Data */

/* Simple-I2C Control Register (16-bits) ************************************/

#define R_SCI_B_ICR_HA_H_IICSTAREQ (1 <<  0)                                                  /* 01: Start Condition Generation */
#define R_SCI_B_ICR_HA_H_IICRSTAREQ (1 <<  1)                                                 /* 02: Restart Condition Generation */
#define R_SCI_B_ICR_HA_H_IICSTPREQ (1 <<  2)                                                  /* 04: Stop Condition Generation */
#define R_SCI_B_ICR_HA_H_IICSDAS_SHIFT (4)
#define R_SCI_B_ICR_HA_H_IICSDAS_MASK (0x3)
#  define R_SCI_B_ICR_HA_H_IICSDAS_SERIAL_DATA_OUTPUT (0 << R_SCI_B_ICR_HA_H_IICSDAS_SHIFT)   /* Serial data output */
#  define R_SCI_B_ICR_HA_H_IICSDAS_V01 (1 << R_SCI_B_ICR_HA_H_IICSDAS_SHIFT)                  /* Generate a start, restart, or stop condition. */
#  define R_SCI_B_ICR_HA_H_IICSDAS_V10 (2 << R_SCI_B_ICR_HA_H_IICSDAS_SHIFT)                  /* Output the low level on the SSDAn pin. */
#  define R_SCI_B_ICR_HA_H_IICSDAS_V11 (3 << R_SCI_B_ICR_HA_H_IICSDAS_SHIFT)                  /* Place the SSDAn pin in the high-impedance state. */
#define R_SCI_B_ICR_HA_H_IICSCLS_SHIFT (6)
#define R_SCI_B_ICR_HA_H_IICSCLS_MASK (0x3)
#  define R_SCI_B_ICR_HA_H_IICSCLS_SERIAL_CLOCK_OUTPUT (0 << R_SCI_B_ICR_HA_H_IICSCLS_SHIFT)  /* Serial clock output */
#  define R_SCI_B_ICR_HA_H_IICSCLS_V01 (1 << R_SCI_B_ICR_HA_H_IICSCLS_SHIFT)                  /* Generate a start, restart, or stop condition. */
#  define R_SCI_B_ICR_HA_H_IICSCLS_V10 (2 << R_SCI_B_ICR_HA_H_IICSCLS_SHIFT)                  /* Output the low level on the SSCLn pin. */
#  define R_SCI_B_ICR_HA_H_IICSCLS_V11 (3 << R_SCI_B_ICR_HA_H_IICSCLS_SHIFT)                  /* Place the SSCLn pin in the high-impedance state. */

/* Simple-I2C Control Register (8-bits) *************************************/

#define R_SCI_B_ICR_BY_HL_IICSTAREQ (1 <<  0)                                                   /* 01: Start Condition Generation */
#define R_SCI_B_ICR_BY_HL_IICRSTAREQ (1 <<  1)                                                  /* 02: Restart Condition Generation */
#define R_SCI_B_ICR_BY_HL_IICSTPREQ (1 <<  2)                                                   /* 04: Stop Condition Generation */
#define R_SCI_B_ICR_BY_HL_IICSDAS_SHIFT (4)
#define R_SCI_B_ICR_BY_HL_IICSDAS_MASK (0x3)
#  define R_SCI_B_ICR_BY_HL_IICSDAS_SERIAL_DATA_OUTPUT (0 << R_SCI_B_ICR_BY_HL_IICSDAS_SHIFT)   /* Serial data output */
#  define R_SCI_B_ICR_BY_HL_IICSDAS_V01 (1 << R_SCI_B_ICR_BY_HL_IICSDAS_SHIFT)                  /* Generate a start, restart, or stop condition. */
#  define R_SCI_B_ICR_BY_HL_IICSDAS_V10 (2 << R_SCI_B_ICR_BY_HL_IICSDAS_SHIFT)                  /* Output the low level on the SSDAn pin. */
#  define R_SCI_B_ICR_BY_HL_IICSDAS_V11 (3 << R_SCI_B_ICR_BY_HL_IICSDAS_SHIFT)                  /* Place the SSDAn pin in the high-impedance state. */
#define R_SCI_B_ICR_BY_HL_IICSCLS_SHIFT (6)
#define R_SCI_B_ICR_BY_HL_IICSCLS_MASK (0x3)
#  define R_SCI_B_ICR_BY_HL_IICSCLS_SERIAL_CLOCK_OUTPUT (0 << R_SCI_B_ICR_BY_HL_IICSCLS_SHIFT)  /* Serial clock output */
#  define R_SCI_B_ICR_BY_HL_IICSCLS_V01 (1 << R_SCI_B_ICR_BY_HL_IICSCLS_SHIFT)                  /* Generate a start, restart, or stop condition. */
#  define R_SCI_B_ICR_BY_HL_IICSCLS_V10 (2 << R_SCI_B_ICR_BY_HL_IICSCLS_SHIFT)                  /* Output the low level on the SSCLn pin. */
#  define R_SCI_B_ICR_BY_HL_IICSCLS_V11 (3 << R_SCI_B_ICR_BY_HL_IICSCLS_SHIFT)                  /* Place the SSCLn pin in the high-impedance state. */

/* FIFO Control Register (32-bits) ******************************************/

#define R_SCI_B_FCR_DRES (1 <<  0)   /* 01: Receive data ready error select bit */
#define R_SCI_B_FCR_TTRG_SHIFT (8)
#define R_SCI_B_FCR_TTRG_MASK (0x1f)
#define R_SCI_B_FCR_TFRST (1 << 15)  /* 8000: Transmit FIFO Data Register Reset */
#define R_SCI_B_FCR_RTRG_SHIFT (16)
#define R_SCI_B_FCR_RTRG_MASK (0x1f)
#define R_SCI_B_FCR_RFRST (1 << 23)  /* 800000: Receive FIFO Data Register Reset */
#define R_SCI_B_FCR_RSTRG_SHIFT (24)
#define R_SCI_B_FCR_RSTRG_MASK (0x1f)

/* FIFO Control Register (16-bits) ******************************************/

#define R_SCI_B_FCR_HA_L_DRES (1 <<  0)   /* 01: Receive data ready error select bit */
#define R_SCI_B_FCR_HA_L_TTRG_SHIFT (8)
#define R_SCI_B_FCR_HA_L_TTRG_MASK (0x1f)
#define R_SCI_B_FCR_HA_L_TFRST (1 << 15)  /* 8000: Transmit FIFO Data Register Reset */

/* FIFO Control Register (8-bits) *******************************************/

#define R_SCI_B_FCR_BY_LL_DRES (1 <<  0)  /* 01: Receive data ready error select bit */

/* FIFO Control Register (8-bits) *******************************************/

#define R_SCI_B_FCR_BY_LH_TTRG_SHIFT (0)
#define R_SCI_B_FCR_BY_LH_TTRG_MASK (0x1f)
#define R_SCI_B_FCR_BY_LH_TFRST (1 <<  7)  /* 80: Transmit FIFO Data Register Reset */

/* FIFO Control Register (16-bits) ******************************************/

#define R_SCI_B_FCR_HA_H_RTRG_SHIFT (0)
#define R_SCI_B_FCR_HA_H_RTRG_MASK (0x1f)
#define R_SCI_B_FCR_HA_H_RFRST (1 <<  7)  /* 80: Receive FIFO Data Register Reset */
#define R_SCI_B_FCR_HA_H_RSTRG_SHIFT (8)
#define R_SCI_B_FCR_HA_H_RSTRG_MASK (0x1f)

/* FIFO Control Register (8-bits) *******************************************/

#define R_SCI_B_FCR_BY_HL_RTRG_SHIFT (0)
#define R_SCI_B_FCR_BY_HL_RTRG_MASK (0x1f)
#define R_SCI_B_FCR_BY_HL_RFRST (1 <<  7)  /* 80: Receive FIFO Data Register Reset */

/* FIFO Control Register (8-bits) *******************************************/

#define R_SCI_B_FCR_BY_HH_RSTRG_SHIFT (0)
#define R_SCI_B_FCR_BY_HH_RSTRG_MASK (0x1f)

/* Manchester Control Register (32-bits) ************************************/

#define R_SCI_B_MCR_RMPOL (1 <<  0)                                  /* 01: Receive Manchester polarity */
#define R_SCI_B_MCR_TMPOL (1 <<  1)                                  /* 02: Transmission Manchester polarity */
#define R_SCI_B_MCR_ERTEN (1 <<  2)                                  /* 04: Manchester edge retiming enable */
#define R_SCI_B_MCR_SYNVAL (1 <<  4)                                 /* 10: Sync type of Manchester code start bit is set. */
#define R_SCI_B_MCR_SYNSEL (1 <<  5)                                 /* 20: Sync select */
#define R_SCI_B_MCR_SBSEL (1 <<  6)                                  /* 40: Start bit select */
#define R_SCI_B_MCR_TPLEN_SHIFT (8)
#define R_SCI_B_MCR_TPLEN_MASK (0xf)
#  define R_SCI_B_MCR_TPLEN_V0 (0 << R_SCI_B_MCR_TPLEN_SHIFT)        /* Transmit preface generation disabled. */
#define R_SCI_B_MCR_TPPAT_SHIFT (12)
#define R_SCI_B_MCR_TPPAT_MASK (0x3)
#  define R_SCI_B_MCR_TPPAT_ALL_ZERO (0 << R_SCI_B_MCR_TPPAT_SHIFT)  /* ALL ZERO */
#  define R_SCI_B_MCR_TPPAT_ZERO_ONE (1 << R_SCI_B_MCR_TPPAT_SHIFT)  /* ZERO ONE */
#define R_SCI_B_MCR_RPLEN_SHIFT (16)
#define R_SCI_B_MCR_RPLEN_MASK (0xf)
#  define R_SCI_B_MCR_RPLEN_V0 (0 << R_SCI_B_MCR_RPLEN_SHIFT)        /* Receive preface generation disabled. */
#define R_SCI_B_MCR_RPPAT_SHIFT (20)
#define R_SCI_B_MCR_RPPAT_MASK (0x3)
#  define R_SCI_B_MCR_RPPAT_ALL_ZERO (0 << R_SCI_B_MCR_RPPAT_SHIFT)  /* ALL ZERO */
#  define R_SCI_B_MCR_RPPAT_ZERO_ONE (1 << R_SCI_B_MCR_RPPAT_SHIFT)  /* ZERO ONE */
#  define R_SCI_B_MCR_RPPAT_ONE_ZERO (2 << R_SCI_B_MCR_RPPAT_SHIFT)  /* ONE ZERO */
#  define R_SCI_B_MCR_RPPAT_ALL_ONE (3 << R_SCI_B_MCR_RPPAT_SHIFT)   /* ALL ONE */
#define R_SCI_B_MCR_PFEREN (1 << 24)                                 /* 1000000: Preface Error enable */
#define R_SCI_B_MCR_SYEREN (1 << 25)                                 /* 2000000: Sync Error enable */
#define R_SCI_B_MCR_SBEREN (1 << 26)                                 /* 4000000: Start bit error enable */

/* Manchester Control Register (16-bits) ************************************/

#define R_SCI_B_MCR_HA_L_RMPOL (1 <<  0)                                       /* 01: Receive Manchester polarity */
#define R_SCI_B_MCR_HA_L_TMPOL (1 <<  1)                                       /* 02: Transmission Manchester polarity */
#define R_SCI_B_MCR_HA_L_ERTEN (1 <<  2)                                       /* 04: Manchester edge retiming enable */
#define R_SCI_B_MCR_HA_L_SYNVAL (1 <<  4)                                      /* 10: Sync type of Manchester code start bit is set. */
#define R_SCI_B_MCR_HA_L_SYNSEL (1 <<  5)                                      /* 20: Sync select */
#define R_SCI_B_MCR_HA_L_SBSEL (1 <<  6)                                       /* 40: Start bit select */
#define R_SCI_B_MCR_HA_L_TPLEN_SHIFT (8)
#define R_SCI_B_MCR_HA_L_TPLEN_MASK (0xf)
#  define R_SCI_B_MCR_HA_L_TPLEN_V0 (0 << R_SCI_B_MCR_HA_L_TPLEN_SHIFT)        /* Transmit preface generation disabled. */
#define R_SCI_B_MCR_HA_L_TPPAT_SHIFT (12)
#define R_SCI_B_MCR_HA_L_TPPAT_MASK (0x3)
#  define R_SCI_B_MCR_HA_L_TPPAT_ALL_ZERO (0 << R_SCI_B_MCR_HA_L_TPPAT_SHIFT)  /* ALL ZERO */
#  define R_SCI_B_MCR_HA_L_TPPAT_ZERO_ONE (1 << R_SCI_B_MCR_HA_L_TPPAT_SHIFT)  /* ZERO ONE */

/* Manchester Control Register (8-bits) *************************************/

#define R_SCI_B_MCR_BY_LL_RMPOL (1 <<  0)   /* 01: Receive Manchester polarity */
#define R_SCI_B_MCR_BY_LL_TMPOL (1 <<  1)   /* 02: Transmission Manchester polarity */
#define R_SCI_B_MCR_BY_LL_ERTEN (1 <<  2)   /* 04: Manchester edge retiming enable */
#define R_SCI_B_MCR_BY_LL_SYNVAL (1 <<  4)  /* 10: Sync type of Manchester code start bit is set. */
#define R_SCI_B_MCR_BY_LL_SYNSEL (1 <<  5)  /* 20: Sync select */
#define R_SCI_B_MCR_BY_LL_SBSEL (1 <<  6)   /* 40: Start bit select */

/* Manchester Control Register (8-bits) *************************************/

#define R_SCI_B_MCR_BY_LH_TPLEN_SHIFT (0)
#define R_SCI_B_MCR_BY_LH_TPLEN_MASK (0xf)
#  define R_SCI_B_MCR_BY_LH_TPLEN_V0 (0 << R_SCI_B_MCR_BY_LH_TPLEN_SHIFT)        /* Transmit preface generation disabled. */
#define R_SCI_B_MCR_BY_LH_TPPAT_SHIFT (4)
#define R_SCI_B_MCR_BY_LH_TPPAT_MASK (0x3)
#  define R_SCI_B_MCR_BY_LH_TPPAT_ALL_ZERO (0 << R_SCI_B_MCR_BY_LH_TPPAT_SHIFT)  /* ALL ZERO */
#  define R_SCI_B_MCR_BY_LH_TPPAT_ZERO_ONE (1 << R_SCI_B_MCR_BY_LH_TPPAT_SHIFT)  /* ZERO ONE */

/* Manchester Control Register (16-bits) ************************************/

#define R_SCI_B_MCR_HA_H_RPLEN_SHIFT (0)
#define R_SCI_B_MCR_HA_H_RPLEN_MASK (0xf)
#  define R_SCI_B_MCR_HA_H_RPLEN_V0 (0 << R_SCI_B_MCR_HA_H_RPLEN_SHIFT)        /* Receive preface generation disabled. */
#define R_SCI_B_MCR_HA_H_RPPAT_SHIFT (4)
#define R_SCI_B_MCR_HA_H_RPPAT_MASK (0x3)
#  define R_SCI_B_MCR_HA_H_RPPAT_ALL_ZERO (0 << R_SCI_B_MCR_HA_H_RPPAT_SHIFT)  /* ALL ZERO */
#  define R_SCI_B_MCR_HA_H_RPPAT_ZERO_ONE (1 << R_SCI_B_MCR_HA_H_RPPAT_SHIFT)  /* ZERO ONE */
#  define R_SCI_B_MCR_HA_H_RPPAT_ONE_ZERO (2 << R_SCI_B_MCR_HA_H_RPPAT_SHIFT)  /* ONE ZERO */
#  define R_SCI_B_MCR_HA_H_RPPAT_ALL_ONE (3 << R_SCI_B_MCR_HA_H_RPPAT_SHIFT)   /* ALL ONE */
#define R_SCI_B_MCR_HA_H_PFEREN (1 <<  8)                                      /* 100: Preface Error enable */
#define R_SCI_B_MCR_HA_H_SYEREN (1 <<  9)                                      /* 200: Sync Error enable */
#define R_SCI_B_MCR_HA_H_SBEREN (1 << 10)                                      /* 400: Start bit error enable */

/* Manchester Control Register (8-bits) *************************************/

#define R_SCI_B_MCR_BY_HL_RPLEN_SHIFT (0)
#define R_SCI_B_MCR_BY_HL_RPLEN_MASK (0xf)
#  define R_SCI_B_MCR_BY_HL_RPLEN_V0 (0 << R_SCI_B_MCR_BY_HL_RPLEN_SHIFT)        /* Receive preface generation disabled. */
#define R_SCI_B_MCR_BY_HL_RPPAT_SHIFT (4)
#define R_SCI_B_MCR_BY_HL_RPPAT_MASK (0x3)
#  define R_SCI_B_MCR_BY_HL_RPPAT_ALL_ZERO (0 << R_SCI_B_MCR_BY_HL_RPPAT_SHIFT)  /* ALL ZERO */
#  define R_SCI_B_MCR_BY_HL_RPPAT_ZERO_ONE (1 << R_SCI_B_MCR_BY_HL_RPPAT_SHIFT)  /* ZERO ONE */
#  define R_SCI_B_MCR_BY_HL_RPPAT_ONE_ZERO (2 << R_SCI_B_MCR_BY_HL_RPPAT_SHIFT)  /* ONE ZERO */
#  define R_SCI_B_MCR_BY_HL_RPPAT_ALL_ONE (3 << R_SCI_B_MCR_BY_HL_RPPAT_SHIFT)   /* ALL ONE */

/* Manchester Control Register (8-bits) *************************************/

#define R_SCI_B_MCR_BY_HH_PFEREN (1 <<  0)  /* 01: Preface Error enable */
#define R_SCI_B_MCR_BY_HH_SYEREN (1 <<  1)  /* 02: Sync Error enable */
#define R_SCI_B_MCR_BY_HH_SBEREN (1 <<  2)  /* 04: Start bit error enable */

/* Driver Control Register (32-bits) ****************************************/

#define R_SCI_B_DCR_DEPOL (1 <<  0)  /* 01: Driver Enable POLarity select */
#define R_SCI_B_DCR_DEAST_SHIFT (8)
#define R_SCI_B_DCR_DEAST_MASK (0x1f)
#define R_SCI_B_DCR_DENGT_SHIFT (16)
#define R_SCI_B_DCR_DENGT_MASK (0x1f)

/* Driver Control Register (16-bits) ****************************************/

#define R_SCI_B_DCR_HA_L_DEPOL (1 <<  0)  /* 01: Driver Enable POLarity select */
#define R_SCI_B_DCR_HA_L_DEAST_SHIFT (8)
#define R_SCI_B_DCR_HA_L_DEAST_MASK (0x1f)

/* Driver Control Register (8-bits) *****************************************/

#define R_SCI_B_DCR_BY_LL_DEPOL (1 <<  0)  /* 01: Driver Enable POLarity select */

/* Driver Control Register (8-bits) *****************************************/

#define R_SCI_B_DCR_BY_LH_DEAST_SHIFT (0)
#define R_SCI_B_DCR_BY_LH_DEAST_MASK (0x1f)

/* Driver Control Register (16-bits) ****************************************/

#define R_SCI_B_DCR_HA_H_DENGT_SHIFT (0)
#define R_SCI_B_DCR_HA_H_DENGT_MASK (0x1f)

/* Driver Control Register (8-bits) *****************************************/

#define R_SCI_B_DCR_BY_HL_DENGT_SHIFT (0)
#define R_SCI_B_DCR_BY_HL_DENGT_MASK (0x1f)

/* Simple-LIN(SCIX) Control Register 0 (32-bits) ****************************/

#define R_SCI_B_XCR0_TCSS_SHIFT (0)
#define R_SCI_B_XCR0_TCSS_MASK (0x3)
#  define R_SCI_B_XCR0_TCSS_TCLK (0 << R_SCI_B_XCR0_TCSS_SHIFT)                   /* TCLK */
#  define R_SCI_B_XCR0_TCSS_TCLK_4 (1 << R_SCI_B_XCR0_TCSS_SHIFT)                 /* TCLK/4 */
#  define R_SCI_B_XCR0_TCSS_TCLK_16 (2 << R_SCI_B_XCR0_TCSS_SHIFT)                /* TCLK/16 */
#  define R_SCI_B_XCR0_TCSS_TCLK_64 (3 << R_SCI_B_XCR0_TCSS_SHIFT)                /* TCLK/64 */
#define R_SCI_B_XCR0_BFE (1 <<  8)                                                /* 100: Break field presence/absence select */
#define R_SCI_B_XCR0_CF0RE (1 <<  9)                                              /* 200: Control field 0 presence/absence select */
#define R_SCI_B_XCR0_CF1DS_SHIFT (10)
#define R_SCI_B_XCR0_CF1DS_MASK (0x3)
#  define R_SCI_B_XCR0_CF1DS_V00 (0 << R_SCI_B_XCR0_CF1DS_SHIFT)                  /* XCR1.PCF1D[7:0] is compare data */
#  define R_SCI_B_XCR0_CF1DS_V01 (1 << R_SCI_B_XCR0_CF1DS_SHIFT)                  /* XCR1.SCF1D[7:0] is compare data */
#  define R_SCI_B_XCR0_CF1DS_V10 (2 << R_SCI_B_XCR0_CF1DS_SHIFT)                  /* XCR1.PCF1D[7:0] and XCR1.SCF1D[7:0] are compare data */
#  define R_SCI_B_XCR0_CF1DS_PROHIBIT (3 << R_SCI_B_XCR0_CF1DS_SHIFT)             /* Prohibit */
#define R_SCI_B_XCR0_PIBE (1 << 12)                                               /* 1000: Priority Interrupt Bit Enable */
#define R_SCI_B_XCR0_PIBS_SHIFT (13)
#define R_SCI_B_XCR0_PIBS_MASK (0x7)
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_0 (0 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 0. */
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_1 (1 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 1. */
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_2 (2 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 2. */
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_3 (3 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 3. */
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_4 (4 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 4. */
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_5 (5 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 5. */
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_6 (6 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 6. */
#  define R_SCI_B_XCR0_PIBS_CONTROL_FIELD_1_BIT_7 (7 << R_SCI_B_XCR0_PIBS_SHIFT)  /* Control Field 1, bit 7. */
#define R_SCI_B_XCR0_BFOIE (1 << 16)                                              /* 10000: Break Filed Output end Interrupt Enable */
#define R_SCI_B_XCR0_BCDIE (1 << 17)                                              /* 20000: Bus Collision Detect Interrupt Enable */
#define R_SCI_B_XCR0_BFDIE (1 << 20)                                              /* 100000: Break Filed Detection Interrupt Enable */
#define R_SCI_B_XCR0_COFIE (1 << 21)                                              /* 200000: Counter Over Flow Interrupt Enable */
#define R_SCI_B_XCR0_AEDIE (1 << 22)                                              /* 400000: Active Edge Detection Interrupt Enable */
#define R_SCI_B_XCR0_BCCS_SHIFT (24)
#define R_SCI_B_XCR0_BCCS_MASK (0x3)
#  define R_SCI_B_XCR0_BCCS_RSCI_BASE_CLOCK (0 << R_SCI_B_XCR0_BCCS_SHIFT)        /* RSCI base clock */
#  define R_SCI_B_XCR0_BCCS_RSCI_BASE_CLOCK_2 (1 << R_SCI_B_XCR0_BCCS_SHIFT)      /* RSCI base clock/2 */
#  define R_SCI_B_XCR0_BCCS_RSCI_BASE_CLOCK_4 (2 << R_SCI_B_XCR0_BCCS_SHIFT)      /* RSCI base clock/4 */
#  define R_SCI_B_XCR0_BCCS_PROHIBIT (3 << R_SCI_B_XCR0_BCCS_SHIFT)               /* Prohibit */

/* Simple-LIN(SCIX) Control Register 0 (16-bits) ****************************/

#define R_SCI_B_XCR0_HA_L_TCSS_SHIFT (0)
#define R_SCI_B_XCR0_HA_L_TCSS_MASK (0x3)
#  define R_SCI_B_XCR0_HA_L_TCSS_TCLK (0 << R_SCI_B_XCR0_HA_L_TCSS_SHIFT)                   /* TCLK */
#  define R_SCI_B_XCR0_HA_L_TCSS_TCLK_4 (1 << R_SCI_B_XCR0_HA_L_TCSS_SHIFT)                 /* TCLK/4 */
#  define R_SCI_B_XCR0_HA_L_TCSS_TCLK_16 (2 << R_SCI_B_XCR0_HA_L_TCSS_SHIFT)                /* TCLK/16 */
#  define R_SCI_B_XCR0_HA_L_TCSS_TCLK_64 (3 << R_SCI_B_XCR0_HA_L_TCSS_SHIFT)                /* TCLK/64 */
#define R_SCI_B_XCR0_HA_L_BFE (1 <<  8)                                                     /* 100: Break field presence/absence select */
#define R_SCI_B_XCR0_HA_L_CF0RE (1 <<  9)                                                   /* 200: Control field 0 presence/absence select */
#define R_SCI_B_XCR0_HA_L_CF1DS_SHIFT (10)
#define R_SCI_B_XCR0_HA_L_CF1DS_MASK (0x3)
#  define R_SCI_B_XCR0_HA_L_CF1DS_V00 (0 << R_SCI_B_XCR0_HA_L_CF1DS_SHIFT)                  /* XCR1.PCF1D[7:0] is compare data */
#  define R_SCI_B_XCR0_HA_L_CF1DS_V01 (1 << R_SCI_B_XCR0_HA_L_CF1DS_SHIFT)                  /* XCR1.SCF1D[7:0] is compare data */
#  define R_SCI_B_XCR0_HA_L_CF1DS_V10 (2 << R_SCI_B_XCR0_HA_L_CF1DS_SHIFT)                  /* XCR1.PCF1D[7:0] and XCR1.SCF1D[7:0] are compare data */
#  define R_SCI_B_XCR0_HA_L_CF1DS_PROHIBIT (3 << R_SCI_B_XCR0_HA_L_CF1DS_SHIFT)             /* Prohibit */
#define R_SCI_B_XCR0_HA_L_PIBE (1 << 12)                                                    /* 1000: Priority Interrupt Bit Enable */
#define R_SCI_B_XCR0_HA_L_PIBS_SHIFT (13)
#define R_SCI_B_XCR0_HA_L_PIBS_MASK (0x7)
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_0 (0 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 0. */
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_1 (1 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 1. */
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_2 (2 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 2. */
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_3 (3 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 3. */
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_4 (4 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 4. */
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_5 (5 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 5. */
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_6 (6 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 6. */
#  define R_SCI_B_XCR0_HA_L_PIBS_CONTROL_FIELD_1_BIT_7 (7 << R_SCI_B_XCR0_HA_L_PIBS_SHIFT)  /* Control Field 1, bit 7. */

/* Simple-LIN(SCIX) Control Register 0 (8-bits) *****************************/

#define R_SCI_B_XCR0_BY_LL_TCSS_SHIFT (0)
#define R_SCI_B_XCR0_BY_LL_TCSS_MASK (0x3)
#  define R_SCI_B_XCR0_BY_LL_TCSS_TCLK (0 << R_SCI_B_XCR0_BY_LL_TCSS_SHIFT)     /* TCLK */
#  define R_SCI_B_XCR0_BY_LL_TCSS_TCLK_4 (1 << R_SCI_B_XCR0_BY_LL_TCSS_SHIFT)   /* TCLK/4 */
#  define R_SCI_B_XCR0_BY_LL_TCSS_TCLK_16 (2 << R_SCI_B_XCR0_BY_LL_TCSS_SHIFT)  /* TCLK/16 */
#  define R_SCI_B_XCR0_BY_LL_TCSS_TCLK_64 (3 << R_SCI_B_XCR0_BY_LL_TCSS_SHIFT)  /* TCLK/64 */

/* Simple-LIN(SCIX) Control Register 0 (8-bits) *****************************/

#define R_SCI_B_XCR0_BY_LH_BFE (1 <<  0)                                                      /* 01: Break field presence/absence select */
#define R_SCI_B_XCR0_BY_LH_CF0RE (1 <<  1)                                                    /* 02: Control field 0 presence/absence select */
#define R_SCI_B_XCR0_BY_LH_CF1DS_SHIFT (2)
#define R_SCI_B_XCR0_BY_LH_CF1DS_MASK (0x3)
#  define R_SCI_B_XCR0_BY_LH_CF1DS_V00 (0 << R_SCI_B_XCR0_BY_LH_CF1DS_SHIFT)                  /* XCR1.PCF1D[7:0] is compare data */
#  define R_SCI_B_XCR0_BY_LH_CF1DS_V01 (1 << R_SCI_B_XCR0_BY_LH_CF1DS_SHIFT)                  /* XCR1.SCF1D[7:0] is compare data */
#  define R_SCI_B_XCR0_BY_LH_CF1DS_V10 (2 << R_SCI_B_XCR0_BY_LH_CF1DS_SHIFT)                  /* XCR1.PCF1D[7:0] and XCR1.SCF1D[7:0] are compare data */
#  define R_SCI_B_XCR0_BY_LH_CF1DS_PROHIBIT (3 << R_SCI_B_XCR0_BY_LH_CF1DS_SHIFT)             /* Prohibit */
#define R_SCI_B_XCR0_BY_LH_PIBE (1 <<  4)                                                     /* 10: Priority Interrupt Bit Enable */
#define R_SCI_B_XCR0_BY_LH_PIBS_SHIFT (5)
#define R_SCI_B_XCR0_BY_LH_PIBS_MASK (0x7)
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_0 (0 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 0. */
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_1 (1 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 1. */
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_2 (2 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 2. */
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_3 (3 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 3. */
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_4 (4 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 4. */
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_5 (5 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 5. */
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_6 (6 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 6. */
#  define R_SCI_B_XCR0_BY_LH_PIBS_CONTROL_FIELD_1_BIT_7 (7 << R_SCI_B_XCR0_BY_LH_PIBS_SHIFT)  /* Control Field 1, bit 7. */

/* Simple-LIN(SCIX) Control Register 0 (16-bits) ****************************/

#define R_SCI_B_XCR0_HA_H_BFOIE (1 <<  0)                                               /* 01: Break Filed Output end Interrupt Enable */
#define R_SCI_B_XCR0_HA_H_BCDIE (1 <<  1)                                               /* 02: Bus Collision Detect Interrupt Enable */
#define R_SCI_B_XCR0_HA_H_BFDIE (1 <<  4)                                               /* 10: Break Filed Detection Interrupt Enable */
#define R_SCI_B_XCR0_HA_H_COFIE (1 <<  5)                                               /* 20: Counter Over Flow Interrupt Enable */
#define R_SCI_B_XCR0_HA_H_AEDIE (1 <<  6)                                               /* 40: Active Edge Detection Interrupt Enable */
#define R_SCI_B_XCR0_HA_H_BCCS_SHIFT (8)
#define R_SCI_B_XCR0_HA_H_BCCS_MASK (0x3)
#  define R_SCI_B_XCR0_HA_H_BCCS_RSCI_BASE_CLOCK (0 << R_SCI_B_XCR0_HA_H_BCCS_SHIFT)    /* RSCI base clock */
#  define R_SCI_B_XCR0_HA_H_BCCS_RSCI_BASE_CLOCK_2 (1 << R_SCI_B_XCR0_HA_H_BCCS_SHIFT)  /* RSCI base clock/2 */
#  define R_SCI_B_XCR0_HA_H_BCCS_RSCI_BASE_CLOCK_4 (2 << R_SCI_B_XCR0_HA_H_BCCS_SHIFT)  /* RSCI base clock/4 */
#  define R_SCI_B_XCR0_HA_H_BCCS_PROHIBIT (3 << R_SCI_B_XCR0_HA_H_BCCS_SHIFT)           /* Prohibit */

/* Simple-LIN(SCIX) Control Register 0 (8-bits) *****************************/

#define R_SCI_B_XCR0_BY_HL_BFOIE (1 <<  0)  /* 01: Break Filed Output end Interrupt Enable */
#define R_SCI_B_XCR0_BY_HL_BCDIE (1 <<  1)  /* 02: Bus Collision Detect Interrupt Enable */
#define R_SCI_B_XCR0_BY_HL_BFDIE (1 <<  4)  /* 10: Break Filed Detection Interrupt Enable */
#define R_SCI_B_XCR0_BY_HL_COFIE (1 <<  5)  /* 20: Counter Over Flow Interrupt Enable */
#define R_SCI_B_XCR0_BY_HL_AEDIE (1 <<  6)  /* 40: Active Edge Detection Interrupt Enable */

/* Simple-LIN(SCIX) Control Register 0 (8-bits) *****************************/

#define R_SCI_B_XCR0_BY_HH_BCCS_SHIFT (0)
#define R_SCI_B_XCR0_BY_HH_BCCS_MASK (0x3)
#  define R_SCI_B_XCR0_BY_HH_BCCS_RSCI_BASE_CLOCK (0 << R_SCI_B_XCR0_BY_HH_BCCS_SHIFT)    /* RSCI base clock */
#  define R_SCI_B_XCR0_BY_HH_BCCS_RSCI_BASE_CLOCK_2 (1 << R_SCI_B_XCR0_BY_HH_BCCS_SHIFT)  /* RSCI base clock/2 */
#  define R_SCI_B_XCR0_BY_HH_BCCS_RSCI_BASE_CLOCK_4 (2 << R_SCI_B_XCR0_BY_HH_BCCS_SHIFT)  /* RSCI base clock/4 */
#  define R_SCI_B_XCR0_BY_HH_BCCS_PROHIBIT (3 << R_SCI_B_XCR0_BY_HH_BCCS_SHIFT)           /* Prohibit */

/* Simple-LIN(SCIX) Control Register 1 (32-bits) ****************************/

#define R_SCI_B_XCR1_TCST (1 <<  0)                              /* 01: break field Timer Count Start trigger */
#define R_SCI_B_XCR1_SDST (1 <<  4)                              /* 10: Start frame Detection Start trigger */
#define R_SCI_B_XCR1_BMEN (1 <<  5)                              /* 20: Bit rate Measurement function Enable */
#define R_SCI_B_XCR1_PCF1D_SHIFT (8)
#define R_SCI_B_XCR1_PCF1D_MASK (0xff)
#define R_SCI_B_XCR1_SCF1D_SHIFT (16)
#define R_SCI_B_XCR1_SCF1D_MASK (0xff)
#define R_SCI_B_XCR1_CF1CE_SHIFT (24)
#define R_SCI_B_XCR1_CF1CE_MASK (0xff)
#  define R_SCI_B_XCR1_CF1CE_V0 (0 << R_SCI_B_XCR1_CF1CE_SHIFT)  /* Bit N of control Field 1 is not for comparison */
#  define R_SCI_B_XCR1_CF1CE_V1 (1 << R_SCI_B_XCR1_CF1CE_SHIFT)  /* Bit N of control Field 1 is for comparison */

/* Simple-LIN(SCIX) Control Register 1 (16-bits) ****************************/

#define R_SCI_B_XCR1_HA_L_TCST (1 <<  0)  /* 01: break field Timer Count Start trigger */
#define R_SCI_B_XCR1_HA_L_SDST (1 <<  4)  /* 10: Start frame Detection Start trigger */
#define R_SCI_B_XCR1_HA_L_BMEN (1 <<  5)  /* 20: Bit rate Measurement function Enable */
#define R_SCI_B_XCR1_HA_L_PCF1D_SHIFT (8)
#define R_SCI_B_XCR1_HA_L_PCF1D_MASK (0xff)

/* Simple-LIN(SCIX) Control Register 1 (8-bits) *****************************/

#define R_SCI_B_XCR1_BY_LL_TCST (1 <<  0)  /* 01: break field Timer Count Start trigger */
#define R_SCI_B_XCR1_BY_LL_SDST (1 <<  4)  /* 10: Start frame Detection Start trigger */
#define R_SCI_B_XCR1_BY_LL_BMEN (1 <<  5)  /* 20: Bit rate Measurement function Enable */

/* Simple-LIN(SCIX) Control Register 1 (8-bits) *****************************/

#define R_SCI_B_XCR1_BY_LH_PCF1D_SHIFT (0)
#define R_SCI_B_XCR1_BY_LH_PCF1D_MASK (0xff)

/* Simple-LIN(SCIX) Control Register 1 (16-bits) ****************************/

#define R_SCI_B_XCR1_HA_H_SCF1D_SHIFT (0)
#define R_SCI_B_XCR1_HA_H_SCF1D_MASK (0xff)
#define R_SCI_B_XCR1_HA_H_CF1CE_SHIFT (8)
#define R_SCI_B_XCR1_HA_H_CF1CE_MASK (0xff)
#  define R_SCI_B_XCR1_HA_H_CF1CE_V0 (0 << R_SCI_B_XCR1_HA_H_CF1CE_SHIFT)  /* Bit N of control Field 1 is not for comparison */
#  define R_SCI_B_XCR1_HA_H_CF1CE_V1 (1 << R_SCI_B_XCR1_HA_H_CF1CE_SHIFT)  /* Bit N of control Field 1 is for comparison */

/* Simple-LIN(SCIX) Control Register 1 (8-bits) *****************************/

#define R_SCI_B_XCR1_BY_HL_SCF1D_SHIFT (0)
#define R_SCI_B_XCR1_BY_HL_SCF1D_MASK (0xff)

/* Simple-LIN(SCIX) Control Register 1 (8-bits) *****************************/

#define R_SCI_B_XCR1_BY_HH_CF1CE_SHIFT (0)
#define R_SCI_B_XCR1_BY_HH_CF1CE_MASK (0xff)
#  define R_SCI_B_XCR1_BY_HH_CF1CE_V0 (0 << R_SCI_B_XCR1_BY_HH_CF1CE_SHIFT)  /* Bit N of control Field 1 is not for comparison */
#  define R_SCI_B_XCR1_BY_HH_CF1CE_V1 (1 << R_SCI_B_XCR1_BY_HH_CF1CE_SHIFT)  /* Bit N of control Field 1 is for comparison */

/* Simple-LIN(SCIX) Control Register 2 (32-bits) ****************************/

#define R_SCI_B_XCR2_CF0D_SHIFT (0)
#define R_SCI_B_XCR2_CF0D_MASK (0xff)
#define R_SCI_B_XCR2_CF0CE_SHIFT (8)
#define R_SCI_B_XCR2_CF0CE_MASK (0xff)
#  define R_SCI_B_XCR2_CF0CE_V0 (0 << R_SCI_B_XCR2_CF0CE_SHIFT)  /* Bit N of control Field 0 is not for comparison */
#  define R_SCI_B_XCR2_CF0CE_V1 (1 << R_SCI_B_XCR2_CF0CE_SHIFT)  /* Bit N of control Field 0 is for comparison */
#define R_SCI_B_XCR2_BFLW_SHIFT (16)
#define R_SCI_B_XCR2_BFLW_MASK (0xffff)

/* Simple-LIN(SCIX) Control Register 2 (16-bits) ****************************/

#define R_SCI_B_XCR2_HA_L_CF0D_SHIFT (0)
#define R_SCI_B_XCR2_HA_L_CF0D_MASK (0xff)
#define R_SCI_B_XCR2_HA_L_CF0CE_SHIFT (8)
#define R_SCI_B_XCR2_HA_L_CF0CE_MASK (0xff)
#  define R_SCI_B_XCR2_HA_L_CF0CE_V0 (0 << R_SCI_B_XCR2_HA_L_CF0CE_SHIFT)  /* Bit N of control Field 0 is not for comparison */
#  define R_SCI_B_XCR2_HA_L_CF0CE_V1 (1 << R_SCI_B_XCR2_HA_L_CF0CE_SHIFT)  /* Bit N of control Field 0 is for comparison */

/* Simple-LIN(SCIX) Control Register 2 (8-bits) *****************************/

#define R_SCI_B_XCR2_BY_LL_CF0D_SHIFT (0)
#define R_SCI_B_XCR2_BY_LL_CF0D_MASK (0xff)

/* Simple-LIN(SCIX) Control Register 2 (8-bits) *****************************/

#define R_SCI_B_XCR2_BY_LH_CF0CE_SHIFT (0)
#define R_SCI_B_XCR2_BY_LH_CF0CE_MASK (0xff)
#  define R_SCI_B_XCR2_BY_LH_CF0CE_V0 (0 << R_SCI_B_XCR2_BY_LH_CF0CE_SHIFT)  /* Bit N of control Field 0 is not for comparison */
#  define R_SCI_B_XCR2_BY_LH_CF0CE_V1 (1 << R_SCI_B_XCR2_BY_LH_CF0CE_SHIFT)  /* Bit N of control Field 0 is for comparison */

/* Simple-LIN(SCIX) Control Register 2 (16-bits) ****************************/

#define R_SCI_B_XCR2_HA_H_BFLW_SHIFT (0)
#define R_SCI_B_XCR2_HA_H_BFLW_MASK (0xffff)

/* Simple-LIN(SCIX) Control Register 2 (8-bits) *****************************/

#define R_SCI_B_XCR2_BY_HL_BFLW_SHIFT (0)
#define R_SCI_B_XCR2_BY_HL_BFLW_MASK (0xff)

/* Simple-LIN(SCIX) Control Register 2 (8-bits) *****************************/

#define R_SCI_B_XCR2_BY_HH_BFLW_SHIFT (0)
#define R_SCI_B_XCR2_BY_HH_BFLW_MASK (0xff)

/* Common Status Register (32-bits) *****************************************/

#define R_SCI_B_CSR_ERS (1 <<  4)     /* 10: Error Signal Status Flag */
#define R_SCI_B_CSR_RxDMON (1 << 15)  /* 8000: Serial input data monitor bit */
#define R_SCI_B_CSR_DCMF (1 << 16)    /* 10000: Data Compare Match Flag */
#define R_SCI_B_CSR_DPER (1 << 17)    /* 20000: Data Compare Match Parity Error Flag */
#define R_SCI_B_CSR_DFER (1 << 18)    /* 40000: Data Compare Match Framing Error Flag */
#define R_SCI_B_CSR_ORER (1 << 24)    /* 1000000: Overrun Error Flag */
#define R_SCI_B_CSR_MFF (1 << 26)     /* 4000000: Mode Fault Flag */
#define R_SCI_B_CSR_PER (1 << 27)     /* 8000000: Parity Error Flag */
#define R_SCI_B_CSR_FER (1 << 28)     /* 10000000: Framing Error Flag */
#define R_SCI_B_CSR_TDRE (1 << 29)    /* 20000000: Transmit Data Empty Flag */
#define R_SCI_B_CSR_TEND (1 << 30)    /* 40000000: Transmit End Flag */
#define R_SCI_B_CSR_RDRF (1 << 31)    /* 80000000: Receive Data Full Flag */

/* Simple-I2C Status Register (32-bits) *************************************/

#define R_SCI_B_ISR_IICACKR (1 <<  0)  /* 01: ACK Reception Data Flag */
#define R_SCI_B_ISR_IICBBS (1 <<  2)   /* 04: Bus busy flag. */
#define R_SCI_B_ISR_IICSTIF (1 <<  3)  /* 08: Issuing of Start, Restart, or Stop Condition Completed Flag */
#define R_SCI_B_ISR_IICSDAI (1 <<  4)  /* 10: SDA input monitor bit. */
#define R_SCI_B_ISR_IICSCLI (1 <<  5)  /* 20: SCL input monitor bit */

/* FIFO Receive Status Register (32-bits) ***********************************/

#define R_SCI_B_FRSR_DR (1 <<  0)   /* 01: Receive Data Ready flag */
#define R_SCI_B_FRSR_BRK (1 <<  1)  /* 02: Break detection signal flag */
#define R_SCI_B_FRSR_R_SHIFT (8)
#define R_SCI_B_FRSR_R_MASK (0x3f)
#define R_SCI_B_FRSR_PNUM_SHIFT (16)
#define R_SCI_B_FRSR_PNUM_MASK (0x3f)
#define R_SCI_B_FRSR_FNUM_SHIFT (24)
#define R_SCI_B_FRSR_FNUM_MASK (0x3f)

/* FIFO Transmit Status Register (32-bits) **********************************/

#define R_SCI_B_FTSR_T_SHIFT (0)
#define R_SCI_B_FTSR_T_MASK (0x3f)

/* Manchester Status Register (32-bits) *************************************/

#define R_SCI_B_MSR_PFER (1 <<  0)   /* 01: Preface Error Register */
#define R_SCI_B_MSR_SYER (1 <<  1)   /* 02: Sync Error Register */
#define R_SCI_B_MSR_SBER (1 <<  2)   /* 04: Start bit Error Register */
#define R_SCI_B_MSR_MER (1 <<  4)    /* 10: Manchester error flag */
#define R_SCI_B_MSR_RSYNC (1 <<  6)  /* 40: Receive sync data bit. */

/* Simple-LIN(SCIX) Status Register 0 (32-bits) *****************************/

#define R_SCI_B_XSR0_SFSF (1 <<  0)   /* 01: Start Frame Status Flag */
#define R_SCI_B_XSR0_RXDSF (1 <<  1)  /* 02: RXD input Status Flag */
#define R_SCI_B_XSR0_BFOF (1 <<  8)   /* 100: Break Field Output end Flag */
#define R_SCI_B_XSR0_BCDF (1 <<  9)   /* 200: Bus Collision Detection Flag */
#define R_SCI_B_XSR0_BFDF (1 << 10)   /* 400: Break Field Detection Flag */
#define R_SCI_B_XSR0_CF0MF (1 << 11)  /* 800: Control Field 0 Match Flag */
#define R_SCI_B_XSR0_CF1MF (1 << 12)  /* 1000: Control Field 1 Match Flag */
#define R_SCI_B_XSR0_PIBDF (1 << 13)  /* 2000: Priority Interrupt Bit Detection Flag */
#define R_SCI_B_XSR0_COF (1 << 14)    /* 4000: Counter Over flow Flag */
#define R_SCI_B_XSR0_AEDF (1 << 15)   /* 8000: Active Edge Detection Flag */
#define R_SCI_B_XSR0_CF0RD_SHIFT (16)
#define R_SCI_B_XSR0_CF0RD_MASK (0xff)
#define R_SCI_B_XSR0_CF1RD_SHIFT (24)
#define R_SCI_B_XSR0_CF1RD_MASK (0xff)

/* Simple-LIN(SCIX) Status Register 1 (32-bits) *****************************/

#define R_SCI_B_XSR1_TCNT_SHIFT (0)
#define R_SCI_B_XSR1_TCNT_MASK (0xffff)

/* Common Flag Clear Register (32-bits) *************************************/

#define R_SCI_B_CFCLR_ERSC (1 <<  4)   /* 10: ERS Clear bit */
#define R_SCI_B_CFCLR_DCMFC (1 << 16)  /* 10000: DCMF Clear bit */
#define R_SCI_B_CFCLR_DPERC (1 << 17)  /* 20000: DPER Clear bit */
#define R_SCI_B_CFCLR_DFERC (1 << 18)  /* 40000: DFER Clear bit */
#define R_SCI_B_CFCLR_ORERC (1 << 24)  /* 1000000: ORER Clear bit */
#define R_SCI_B_CFCLR_MFFC (1 << 26)   /* 4000000: MFF Clear bit */
#define R_SCI_B_CFCLR_PERC (1 << 27)   /* 8000000: PER Clear bit */
#define R_SCI_B_CFCLR_FERC (1 << 28)   /* 10000000: FER Clear bit */
#define R_SCI_B_CFCLR_TDREC (1 << 29)  /* 20000000: TDRE Clear bit */
#define R_SCI_B_CFCLR_RDRFC (1 << 31)  /* 80000000: RDRF Clear bit */

/* Common Flag Clear Register (16-bits) *************************************/

#define R_SCI_B_CFCLR_HA_L_ERSC (1 <<  4)  /* 10: ERS Clear bit */

/* Common Flag Clear Register (8-bits) **************************************/

#define R_SCI_B_CFCLR_BY_LL_ERSC (1 <<  4)  /* 10: ERS Clear bit */

/* Common Flag Clear Register (16-bits) *************************************/

#define R_SCI_B_CFCLR_HA_H_DCMFC (1 <<  0)  /* 01: DCMF Clear bit */
#define R_SCI_B_CFCLR_HA_H_DPERC (1 <<  1)  /* 02: DPER Clear bit */
#define R_SCI_B_CFCLR_HA_H_DFERC (1 <<  2)  /* 04: DFER Clear bit */
#define R_SCI_B_CFCLR_HA_H_ORERC (1 <<  8)  /* 100: ORER Clear bit */
#define R_SCI_B_CFCLR_HA_H_MFFC (1 << 10)   /* 400: MFF Clear bit */
#define R_SCI_B_CFCLR_HA_H_PERC (1 << 11)   /* 800: PER Clear bit */
#define R_SCI_B_CFCLR_HA_H_FERC (1 << 12)   /* 1000: FER Clear bit */
#define R_SCI_B_CFCLR_HA_H_TDREC (1 << 13)  /* 2000: TDRE Clear bit */
#define R_SCI_B_CFCLR_HA_H_RDRFC (1 << 15)  /* 8000: RDRF Clear bit */

/* Common Flag Clear Register (8-bits) **************************************/

#define R_SCI_B_CFCLR_BY_HL_DCMFC (1 <<  0)  /* 01: DCMF Clear bit */
#define R_SCI_B_CFCLR_BY_HL_DPERC (1 <<  1)  /* 02: DPER Clear bit */
#define R_SCI_B_CFCLR_BY_HL_DFERC (1 <<  2)  /* 04: DFER Clear bit */

/* Common Flag Clear Register (8-bits) **************************************/

#define R_SCI_B_CFCLR_BY_HH_ORERC (1 <<  0)  /* 01: ORER Clear bit */
#define R_SCI_B_CFCLR_BY_HH_MFFC (1 <<  2)   /* 04: MFF Clear bit */
#define R_SCI_B_CFCLR_BY_HH_PERC (1 <<  3)   /* 08: PER Clear bit */
#define R_SCI_B_CFCLR_BY_HH_FERC (1 <<  4)   /* 10: FER Clear bit */
#define R_SCI_B_CFCLR_BY_HH_TDREC (1 <<  5)  /* 20: TDRE Clear bit */
#define R_SCI_B_CFCLR_BY_HH_RDRFC (1 <<  7)  /* 80: RDRF Clear bit */

/* Simple-I2C Flag Clear Register (32-bits) *********************************/

#define R_SCI_B_ICFCLR_IICBBSC (1 <<  2)   /* 04: IICBBS Clear bit */
#define R_SCI_B_ICFCLR_IICSTIFC (1 <<  3)  /* 08: IICSTIF Clear bit */

/* Simple-I2C Flag Clear Register (16-bits) *********************************/

#define R_SCI_B_ICFCLR_HA_L_IICBBSC (1 <<  2)   /* 04: IICBBSC Clear bit */
#define R_SCI_B_ICFCLR_HA_L_IICSTIFC (1 <<  3)  /* 08: IICSTIF Clear bit */

/* Simple-I2C Flag Clear Register (8-bits) **********************************/

#define R_SCI_B_ICFCLR_BY_LL_IICBBSC (1 <<  2)   /* 04: IICBBSC Clear bit */
#define R_SCI_B_ICFCLR_BY_LL_IICSTIFC (1 <<  3)  /* 08: IICSTIF Clear bit */

/* FIFO Flag Clear Register (32-bits) ***************************************/

#define R_SCI_B_FFCLR_DRC (1 <<  0)   /* 01: DR Clear bit */
#define R_SCI_B_FFCLR_BRKC (1 <<  1)  /* 02: BRK Clear bit */

/* FIFO Flag Clear Register (16-bits) ***************************************/

#define R_SCI_B_FFCLR_HA_L_DRC (1 <<  0)   /* 01: DR Clear bit */
#define R_SCI_B_FFCLR_HA_L_BRKC (1 <<  1)  /* 02: BRK Clear bit */

/* FIFO Flag Clear Register (8-bits) ****************************************/

#define R_SCI_B_FFCLR_BY_LL_DRC (1 <<  0)   /* 01: DR Clear bit */
#define R_SCI_B_FFCLR_BY_LL_BRKC (1 <<  1)  /* 02: BRK Clear bit */

/* Manchester Flag Clear Register (32-bits) *********************************/

#define R_SCI_B_MFCLR_PFERC (1 <<  0)  /* 01: PFER Clear bit */
#define R_SCI_B_MFCLR_SYERC (1 <<  1)  /* 02: SYER Clear bit */
#define R_SCI_B_MFCLR_SBERC (1 <<  2)  /* 04: SBER Clear bit */
#define R_SCI_B_MFCLR_MERC (1 <<  4)   /* 10: MER Clear bit */

/* Manchester Flag Clear Register (16-bits) *********************************/

#define R_SCI_B_MFCLR_HA_L_PFERC (1 <<  0)  /* 01: PFER Clear bit */
#define R_SCI_B_MFCLR_HA_L_SYERC (1 <<  1)  /* 02: SYER Clear bit */
#define R_SCI_B_MFCLR_HA_L_SBERC (1 <<  2)  /* 04: SBER Clear bit */
#define R_SCI_B_MFCLR_HA_L_MERC (1 <<  4)   /* 10: MER Clear bit */

/* Manchester Flag Clear Register (8-bits) **********************************/

#define R_SCI_B_MFCLR_BY_LL_PFERC (1 <<  0)  /* 01: PFER Clear bit */
#define R_SCI_B_MFCLR_BY_LL_SYERC (1 <<  1)  /* 02: SYER Clear bit */
#define R_SCI_B_MFCLR_BY_LL_SBERC (1 <<  2)  /* 04: SBER Clear bit */
#define R_SCI_B_MFCLR_BY_LL_MERC (1 <<  4)   /* 10: MER Clear bit */

/* Simpe-LIN(SCIX) Flag Clear Register (32-bits) ****************************/

#define R_SCI_B_XFCLR_BFOC (1 <<  8)   /* 100: BFO Clear bit */
#define R_SCI_B_XFCLR_BCDC (1 <<  9)   /* 200: BCD Clear bit */
#define R_SCI_B_XFCLR_BFDC (1 << 10)   /* 400: BFD Clear bit */
#define R_SCI_B_XFCLR_CF0MC (1 << 11)  /* 800: CF0M Clear bit */
#define R_SCI_B_XFCLR_CF1MC (1 << 12)  /* 1000: CF1M Clear bit */
#define R_SCI_B_XFCLR_PIBDC (1 << 13)  /* 2000: PIBD Clear bit */
#define R_SCI_B_XFCLR_COFC (1 << 14)   /* 4000: COF Clear bit */
#define R_SCI_B_XFCLR_AEDC (1 << 15)   /* 8000: AED Clear bit */

/* Simpe-LIN(SCIX) Flag Clear Register (16-bits) ****************************/

#define R_SCI_B_XFCLR_HA_L_BFOC (1 <<  8)   /* 100: BFO Clear bit */
#define R_SCI_B_XFCLR_HA_L_BCDC (1 <<  9)   /* 200: BCD Clear bit */
#define R_SCI_B_XFCLR_HA_L_BFDC (1 << 10)   /* 400: BFD Clear bit */
#define R_SCI_B_XFCLR_HA_L_CF0MC (1 << 11)  /* 800: CF0M Clear bit */
#define R_SCI_B_XFCLR_HA_L_CF1MC (1 << 12)  /* 1000: CF1M Clear bit */
#define R_SCI_B_XFCLR_HA_L_PIBDC (1 << 13)  /* 2000: PIBD Clear bit */
#define R_SCI_B_XFCLR_HA_L_COFC (1 << 14)   /* 4000: COF Clear bit */
#define R_SCI_B_XFCLR_HA_L_AEDC (1 << 15)   /* 8000: AED Clear bit */

/* Simpe-LIN(SCIX) Flag Clear Register (8-bits) *****************************/

#define R_SCI_B_XFCLR_BY_LH_BFOC (1 <<  0)   /* 01: BFO Clear bit */
#define R_SCI_B_XFCLR_BY_LH_BCDC (1 <<  1)   /* 02: BCD Clear bit */
#define R_SCI_B_XFCLR_BY_LH_BFDC (1 <<  2)   /* 04: BFD Clear bit */
#define R_SCI_B_XFCLR_BY_LH_CF0MC (1 <<  3)  /* 08: CF0M Clear bit */
#define R_SCI_B_XFCLR_BY_LH_CF1MC (1 <<  4)  /* 10: CF1M Clear bit */
#define R_SCI_B_XFCLR_BY_LH_PIBDC (1 <<  5)  /* 20: PIBD Clear bit */
#define R_SCI_B_XFCLR_BY_LH_COFC (1 <<  6)   /* 40: COF Clear bit */
#define R_SCI_B_XFCLR_BY_LH_AEDC (1 <<  7)   /* 80: AED Clear bit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_SCI_H */
