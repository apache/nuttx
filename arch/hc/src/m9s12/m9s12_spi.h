/****************************************************************************
 * arch/hc/src/m9s12/m9s12_spi.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_SPI_H
#define __ARCH_HC_SRC_M9S12_M9S12_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define HCS12_SPI_CR1_OFFSET      0x00 /* SPI Control Register 1 */
#define HCS12_SPI_CR2_OFFSET      0x01 /* SPI Control Register 2 */
#define HCS12_SPI_BR_OFFSET       0x02 /* SPI Baud Rate Register */
#define HCS12_SPI_SR_OFFSET       0x03 /* SPI Status Register */
#define HCS12_SPI_DR_OFFSET       0x05 /* SPI Data Register */

/* Register Addresses *******************************************************/

#define HCS12_SPI_CR1             (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_CR1_OFFSET)
#define HCS12_SPI_CR2             (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_CR2_OFFSET)
#define HCS12_SPI_BR              (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_BR_OFFSET)
#define HCS12_SPI_SR              (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_SR_OFFSET)
#define HCS12_SPI_DR              (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_DR_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* SPI Control Register 1 Bit-Field Definitions */

#define SPI_CR1_LSBFE             (1 << 0)  /* Bit 0:  LSB-First Enable */
#define SPI_CR1_SSOE              (1 << 1)  /* Bit 1:  Slave Select Output Enable */
#define SPI_CR1_CPHA              (1 << 2)  /* Bit 2:  SPI Clock Phase */
#define SPI_CR1_CPOL              (1 << 3)  /* Bit 3:  SPI Clock Polarity */
#define SPI_CR1_MSTR              (1 << 4)  /* Bit 4:  SPI Master/Slave Mode Select */
#define SPI_CR1_SPTIE             (1 << 5)  /* Bit 5:  SPI Transmit Interrupt Enable */
#define SPI_CR1_SPE               (1 << 6)  /* Bit 6:  SPI System Enable */
#define SPI_CR1_SPIE              (1 << 7)  /* Bit 7:  SPI Interrupt Enable */

/* SPI Control Register 2 Bit-Field Definitions */

#define SPI_CR2_SPC0              (1 << 0)  /* Bit 0:   Serial Pin Control Bit 0  */
#define SPI_CR2_SPISWAI           (1 << 1)  /* Bit 1:  SPI Stop in Wait Mode */
#define SPI_CR2_BIDIROE           (1 << 3)  /* Bit 3:  Output Enable in the Bidirectional */
#define SPI_CR2_MODFEN            (1 << 4)  /* Bit 4:  Mode Fault Enable */

/* SPI Baud Rate Register Bit-Field Definitions */

#define SPI_BR_SPR_SHIFT          (0)      /* Bits 0-2: SPI Baud Rate Selection */
#define SPI_BR_SPR_MASK           (7 << SPI_BR_SPR_SHIFT)
#define SPI_BR_SPPR_SHIFT         (4)      /* Bits 4-6: SPI Baud Rate Preselection */
#define SPI_BR_SPPR_MASK          (7 << SPI_BR_SPPR_SHIFT)

/* SPI Status Register Bit-Field Definitions */

#define SPI_SR_MODF               (1 << 4)  /* Bit 4:  Mode Fault */
#define SPI_SR_SPTEF              (1 << 5)  /* Bit 5:  SPI Transmit Empty Interrupt */
#define SPI_SR_SPIF               (1 << 7)  /* Bit 7:  SPIF Interrupt */

/* SPI Data Register Bit-Field Definitions */

/* 8-bit SPI data register */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_SPI_H */
