/****************************************************************************
 * arch/arm/src/eoss3/hardware/eoss3_uart.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_UART_H
#define __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define EOSS3_UART_DR_OFFSET      0x0000  /* Data Register */
#define EOSS3_UART_RSR_ECR_OFFSET 0x0004  /* Status Reg / Error Clear Reg */
#define EOSS3_UART_TFR_OFFSET     0x0018  /* Flag Register */
#define EOSS3_UART_ILPRDIV_OFFSET 0x0020  /* Low Power Divisor */
#define EOSS3_UART_IBRD_OFFSET    0x0024  /* Integer Baud Rate Divisor */
#define EOSS3_UART_FBRD_OFFSET    0x0028  /* Fractional Baud Rate Divisor */
#define EOSS3_UART_LCR_H_OFFSET   0x002c  /* UART Line Control Register */
#define EOSS3_UART_CR_OFFSET      0x0030  /* UART Control Register */
#define EOSS3_UART_IFLS_OFFSET    0x0034  /* Interrupt FIFO Level Select */
#define EOSS3_UART_IMSC_OFFSET    0x0038  /* Interrupt Mask Set/Clear */
#define EOSS3_UART_RIS_OFFSET     0x003c  /* Raw Interrupt Status Register */
#define EOSS3_UART_MIS_OFFSET     0x0040  /* Masked Interrupt Status */
#define EOSS3_UART_ICR_OFFSET     0x0044  /* Interrupt Clear Register */

/* Register Addresses *******************************************************/

#define EOSS3_UART_DR             (EOSS3_UART_BASE + EOSS3_UART_DR_OFFSET)
#define EOSS3_UART_RSR_ECR        (EOSS3_UART_BASE + EOSS3_UART_RSR_ECR_OFFSET)
#define EOSS3_UART_TFR            (EOSS3_UART_BASE + EOSS3_UART_TFR_OFFSET)
#define EOSS3_UART_ILPRDIV        (EOSS3_UART_BASE + EOSS3_UART_ILPRDIV_OFFSET)
#define EOSS3_UART_IBRD           (EOSS3_UART_BASE + EOSS3_UART_IBRD_OFFSET)
#define EOSS3_UART_FBRD           (EOSS3_UART_BASE + EOSS3_UART_FBRD_OFFSET)
#define EOSS3_UART_LCR_H          (EOSS3_UART_BASE + EOSS3_UART_LCR_H_OFFSET)
#define EOSS3_UART_CR             (EOSS3_UART_BASE + EOSS3_UART_CR_OFFSET)
#define EOSS3_UART_IFLS           (EOSS3_UART_BASE + EOSS3_UART_IFLS_OFFSET)
#define EOSS3_UART_IMSC           (EOSS3_UART_BASE + EOSS3_UART_IMSC_OFFSET)
#define EOSS3_UART_RIS            (EOSS3_UART_BASE + EOSS3_UART_RIS_OFFSET)
#define EOSS3_UART_MIS            (EOSS3_UART_BASE + EOSS3_UART_MIS_OFFSET)
#define EOSS3_UART_ICR            (EOSS3_UART_BASE + EOSS3_UART_ICR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EOSS3_UART_DR Register */

#define UART_DR_DATA_SHIFT        (0)
#define UART_DR_DATA_MASK         (0xff << UART_DR_DATA_SHIFT)
#define UART_DR_FE                (1 << 8)
#define UART_DR_PE                (1 << 9)
#define UART_DR_BE                (1 << 10)
#define UART_DR_OE                (1 << 11)

/* EOSS3_UART_RSR_ECR Register */

#define UART_RSR_ECR_FE           (1 << 0)
#define UART_RST_ECR_PE           (1 << 1)
#define UART_RSR_ECR_BE           (1 << 2)
#define UART_RSR_ECR_OE           (1 << 3)

/* EOSS3_UART_TFR Register */

#define UART_TFR_CTS              (1 << 0)
#define UART_TFR_DSR              (1 << 1)
#define UART_TFR_DCD              (1 << 2)
#define UART_TFR_BUSY             (1 << 3)
#define UART_TFR_RXFE             (1 << 4)
#define UART_TFR_TXFF             (1 << 5)
#define UART_TFR_RXFF             (1 << 6)
#define UART_TFR_TXFE             (1 << 7)
#define UART_TFR_RI               (1 << 8)

/* EOSS3_UART_LCR_H Register */

#define UART_LCR_H_BRK            (1 << 0)
#define UART_LCR_H_PEN            (1 << 1)
#define UART_LCR_H_EPS            (1 << 2)
#define UART_LCR_H_STP2           (1 << 3)
#define UART_LCR_H_FEN            (1 << 4)
#define UART_LCR_H_WLEN_SHIFT     (5)
#define UART_LCR_H_WLEN_MASK      (0x3 << UART_LCR_H_WLEN_SHIFT)
#define UART_LCR_H_WLE_5          (0x0 << UART_LCR_H_WLEN_SHIFT)
#define UART_LCR_H_WLE_6          (0x1 << UART_LCR_H_WLEN_SHIFT)
#define UART_LCR_H_WLE_7          (0x2 << UART_LCR_H_WLEN_SHIFT)
#define UART_LCR_H_WLE_8          (0x3 << UART_LCR_H_WLEN_SHIFT)
#define UART_LCR_H_SPS            (1 << 7)

/* EOSS3_UART_CR Register */

#define UART_CR_UARTEN            (1 << 0)
#define UART_CR_SIREN             (1 << 1)
#define UART_CR_SIRLP             (1 << 2)
#define UART_CR_LBE               (1 << 7)
#define UART_CR_TXE               (1 << 8)
#define UART_CR_RXE               (1 << 9)
#define UART_CR_DTR               (1 << 10)
#define UART_CR_RTS               (1 << 11)
#define UART_CR_OUT1              (1 << 12)
#define UART_CR_OUT2              (1 << 13)
#define UART_CR_RTSEN             (1 << 14)
#define UART_CR_CTSEN             (1 << 15)

/* EOSS3_UART_IMSC Register */

#define UART_IMSC_RIMIM           (1 << 0)
#define UART_IMSC_CTSMIM          (1 << 1)
#define UART_IMSC_DCDMIM          (1 << 2)
#define UART_IMSC_DSRMIM          (1 << 3)
#define UART_IMSC_RXIM            (1 << 4)
#define UART_IMSC_TXIM            (1 << 5)
#define UART_IMSC_RTIM            (1 << 6)
#define UART_IMSC_FEIM            (1 << 7)
#define UART_IMSC_PEIM            (1 << 8)
#define UART_IMSC_BEIM            (1 << 9)
#define UART_IMSC_OEIM            (1 << 10)
#define UART_IMSC_ALLINTS         (0x7ff)

/* EOSS3_UART_IMSC Register */

#define UART_MIS_RIMMIS           (1 << 0)
#define UART_MIS_CTSMMIS          (1 << 1)
#define UART_MIS_DCDMMIS          (1 << 2)
#define UART_MIS_DSRMMIS          (1 << 3)
#define UART_MIS_RXMIS            (1 << 4)
#define UART_MIS_TXMIS            (1 << 5)
#define UART_MIS_RTMIS            (1 << 6)
#define UART_MIS_FEMIS            (1 << 7)
#define UART_MIS_PEMIS            (1 << 8)
#define UART_MIS_BEMIS            (1 << 9)
#define UART_MIS_OEMIS            (1 << 10)

/* EOSS3_UART_ICR Register */

#define UART_ICR_RIMIC           (1 << 0)
#define UART_ICR_CTSMIC          (1 << 1)
#define UART_ICR_DCDMIC          (1 << 2)
#define UART_ICR_DSRMIC          (1 << 3)
#define UART_ICR_RXIC            (1 << 4)
#define UART_ICR_TXIC            (1 << 5)
#define UART_ICR_RTIC            (1 << 6)
#define UART_ICR_FEIC            (1 << 7)
#define UART_ICR_PEIC            (1 << 8)
#define UART_ICR_BEIC            (1 << 9)
#define UART_ICR_OEIC            (1 << 10)

#endif /* __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_UART_H */
