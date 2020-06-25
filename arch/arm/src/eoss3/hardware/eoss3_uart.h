/****************************************************************************
 * arch/arm/src/eoss3/chip.h
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
#define EOSS3_IFLS_OFFSET         0x0034  /* Interrupt FIFO Level Select */
#define EOSS3_IMSC_OFFSET         0x0038  /* Interrupt Mask Set/Clear */
#define EOSS3_RIS_OFFSET          0x003c  /* Raw Interrupt Status Register */
#define EOSS3_MIS_OFFSET          0x0040  /* Masked Interrupt Status */
#define EOSS3_ICR_OFFSET          0x0044  /* Interrupt Clear Register */

/* Register Addresses *******************************************************/

#define EOSS3_UART_DR             (EOSS3_UART_BASE + EOSS3_UART_DR_OFFSET)
#define EOSS3_UART_RSR_ECR        (EOSS3_UART_BASE + EOSS3_UART_RSR_ECR_OFFSET)
#define EOSS3_UART_TFR            (EOSS3_UART_BASE + EOSS3_UART_TFR_OFFSET)
#define EOSS3_UART_ILPRDIV        (EOSS3_UART_BASE + EOSS3_UART_ILPRDIV_OFFSET)
#define EOSS3_UART_IBRD           (EOSS3_UART_BASE + EOSS3_UART_IBRD_OFFSET)
#define EOSS3_UART_FBRD           (EOSS3_UART_BASE + EOSS3_UART_FBRD_OFFSET)
#define EOSS3_UART_LCR_H          (EOSS3_UART_BASE + EOSS3_UART_LCR_H_OFFSET)
#define EOSS3_UART_CR             (EOSS3_UART_BASE + EOSS3_UART_CR_OFFSET)
#define EOSS3_IFLS                (EOSS3_UART_BASE + EOSS3_IFLS_OFFSET)
#define EOSS3_IMSC                (EOSS3_UART_BASE + EOSS3_IMSC_OFFSET)
#define EOSS3_RIS                 (EOSS3_UART_BASE + EOSS3_RIS_OFFSET)
#define EOSS3_MIS                 (EOSS3_UART_BASE + EOSS3_MIS_OFFSET)
#define EOSS3_ICR                 (EOSS3_UART_BASE + EOSS3_ICR_OFFSET)

/* Register Bitfield Definitions ********************************************/

#endif /* __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_UART_H */
