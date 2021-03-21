/*****************************************************************************
 * arch/arm/src/stm32l5/hardware/stm32l5_spi.h
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_SPI_H
#define __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_SPI_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Maximum allowed speed as per specifications for all SPIs */

#if defined(CONFIG_STM32L5_STM32L562XX)
#  define STM32L5_SPI_CLK_MAX     55000000UL
#else
#  error "Unsupported STM32 L5 chip"
#endif

/* Register Offsets **********************************************************/

#define STM32L5_SPI_CR1_OFFSET       0x0000  /* SPI Control Register 1 (16-bit) */
#define STM32L5_SPI_CR2_OFFSET       0x0004  /* SPI control register 2 (16-bit) */
#define STM32L5_SPI_SR_OFFSET        0x0008  /* SPI status register (16-bit) */
#define STM32L5_SPI_DR_OFFSET        0x000c  /* SPI data register (16-bit) */
#define STM32L5_SPI_CRCPR_OFFSET     0x0010  /* SPI CRC polynomial register (16-bit) */
#define STM32L5_SPI_RXCRCR_OFFSET    0x0014  /* SPI Rx CRC register (16-bit) */
#define STM32L5_SPI_TXCRCR_OFFSET    0x0018  /* SPI Tx CRC register (16-bit) */

/* Register Addresses ********************************************************/

#if STM32L5_NSPI > 0
#  define STM32L5_SPI1_CR1          (STM32L5_SPI1_BASE + STM32L5_SPI_CR1_OFFSET)
#  define STM32L5_SPI1_CR2          (STM32L5_SPI1_BASE + STM32L5_SPI_CR2_OFFSET)
#  define STM32L5_SPI1_SR           (STM32L5_SPI1_BASE + STM32L5_SPI_SR_OFFSET)
#  define STM32L5_SPI1_DR           (STM32L5_SPI1_BASE + STM32L5_SPI_DR_OFFSET)
#  define STM32L5_SPI1_CRCPR        (STM32L5_SPI1_BASE + STM32L5_SPI_CRCPR_OFFSET)
#  define STM32L5_SPI1_RXCRCR       (STM32L5_SPI1_BASE + STM32L5_SPI_RXCRCR_OFFSET)
#  define STM32L5_SPI1_TXCRCR       (STM32L5_SPI1_BASE + STM32L5_SPI_TXCRCR_OFFSET)
#endif

#if STM32L5_NSPI > 1
#  define STM32L5_SPI2_CR1          (STM32L5_SPI2_BASE + STM32L5_SPI_CR1_OFFSET)
#  define STM32L5_SPI2_CR2          (STM32L5_SPI2_BASE + STM32L5_SPI_CR2_OFFSET)
#  define STM32L5_SPI2_SR           (STM32L5_SPI2_BASE + STM32L5_SPI_SR_OFFSET)
#  define STM32L5_SPI2_DR           (STM32L5_SPI2_BASE + STM32L5_SPI_DR_OFFSET)
#  define STM32L5_SPI2_CRCPR        (STM32L5_SPI2_BASE + STM32L5_SPI_CRCPR_OFFSET)
#  define STM32L5_SPI2_RXCRCR       (STM32L5_SPI2_BASE + STM32L5_SPI_RXCRCR_OFFSET)
#  define STM32L5_SPI2_TXCRCR       (STM32L5_SPI2_BASE + STM32L5_SPI_TXCRCR_OFFSET)
#endif

#if STM32L5_NSPI > 2
#  define STM32L5_SPI3_CR1          (STM32L5_SPI3_BASE + STM32L5_SPI_CR1_OFFSET)
#  define STM32L5_SPI3_CR2          (STM32L5_SPI3_BASE + STM32L5_SPI_CR2_OFFSET)
#  define STM32L5_SPI3_SR           (STM32L5_SPI3_BASE + STM32L5_SPI_SR_OFFSET)
#  define STM32L5_SPI3_DR           (STM32L5_SPI3_BASE + STM32L5_SPI_DR_OFFSET)
#  define STM32L5_SPI3_CRCPR        (STM32L5_SPI3_BASE + STM32L5_SPI_CRCPR_OFFSET)
#  define STM32L5_SPI3_RXCRCR       (STM32L5_SPI3_BASE + STM32L5_SPI_RXCRCR_OFFSET)
#  define STM32L5_SPI3_TXCRCR       (STM32L5_SPI3_BASE + STM32L5_SPI_TXCRCR_OFFSET)
#endif

/* Register Bitfield Definitions *********************************************/

/* SPI Control Register 1 */

#define SPI_CR1_CPHA              (1 << 0)                /* Bit 0: Clock Phase */
#define SPI_CR1_CPOL              (1 << 1)                /* Bit 1: Clock Polarity */
#define SPI_CR1_MSTR              (1 << 2)                /* Bit 2: Master Selection */
#define SPI_CR1_BR_SHIFT          (3)                     /* Bits 5:3 Baud Rate Control */
#define SPI_CR1_BR_MASK           (7 << SPI_CR1_BR_SHIFT)
#  define SPI_CR1_FPCLCKd2        (0 << SPI_CR1_BR_SHIFT) /* 000: fPCLK/2 */
#  define SPI_CR1_FPCLCKd4        (1 << SPI_CR1_BR_SHIFT) /* 001: fPCLK/4 */
#  define SPI_CR1_FPCLCKd8        (2 << SPI_CR1_BR_SHIFT) /* 010: fPCLK/8 */
#  define SPI_CR1_FPCLCKd16       (3 << SPI_CR1_BR_SHIFT) /* 011: fPCLK/16 */
#  define SPI_CR1_FPCLCKd32       (4 << SPI_CR1_BR_SHIFT) /* 100: fPCLK/32 */
#  define SPI_CR1_FPCLCKd64       (5 << SPI_CR1_BR_SHIFT) /* 101: fPCLK/64 */
#  define SPI_CR1_FPCLCKd128      (6 << SPI_CR1_BR_SHIFT) /* 110: fPCLK/128 */
#  define SPI_CR1_FPCLCKd256      (7 << SPI_CR1_BR_SHIFT) /* 111: fPCLK/256 */
#define SPI_CR1_SPE               (1 << 6)                /* Bit 6: SPI Enable */
#define SPI_CR1_LSBFIRST          (1 << 7)                /* Bit 7: Frame Format */
#define SPI_CR1_SSI               (1 << 8)                /* Bit 8: Internal slave select */
#define SPI_CR1_SSM               (1 << 9)                /* Bit 9: Software slave management */
#define SPI_CR1_RXONLY            (1 << 10)               /* Bit 10: Receive only */
#define SPI_CR1_CRCL              (1 << 11)               /* Bit 11: CRC length */
#define SPI_CR1_CRCNEXT           (1 << 12)               /* Bit 12: Transmit CRC next */
#define SPI_CR1_CRCEN             (1 << 13)               /* Bit 13: Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE            (1 << 14)               /* Bit 14: Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE          (1 << 15)               /* Bit 15: Bidirectional data mode enable */

/* SPI Control Register 2 */

#define SPI_CR2_RXDMAEN           (1 << 0)  /* Bit 0: Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN           (1 << 1)  /* Bit 1: Tx Buffer DMA Enable */
#define SPI_CR2_SSOE              (1 << 2)  /* Bit 2: SS Output Enable */
#define SPI_CR2_NSSP              (1 << 3)  /* Bit 3: NSS pulse management */
#define SPI_CR2_FRF               (1 << 4)  /* Bit 4: Frame format */
#define SPI_CR2_ERRIE             (1 << 5)  /* Bit 5: Error interrupt enable */
#define SPI_CR2_RXNEIE            (1 << 6)  /* Bit 6: RX buffer not empty interrupt enable */
#define SPI_CR2_TXEIE             (1 << 7)  /* Bit 7: Tx buffer empty interrupt enable */
#define SPI_CR2_DS_SHIFT          (8)       /* Bits 8-11:  Data size */
#define SPI_CR2_DS_MASK           (15 << SPI_CR2_DS_SHIFT)
#  define SPI_CR2_DS_VAL(bits)    ( ((bits)-1) << SPI_CR2_DS_SHIFT)
#  define SPI_CR2_DS_4BIT         SPI_CR2_DS_VAL( 4)
#  define SPI_CR2_DS_5BIT         SPI_CR2_DS_VAL( 5)
#  define SPI_CR2_DS_6BIT         SPI_CR2_DS_VAL( 6)
#  define SPI_CR2_DS_7BIT         SPI_CR2_DS_VAL( 7)
#  define SPI_CR2_DS_8BIT         SPI_CR2_DS_VAL( 8)
#  define SPI_CR2_DS_9BIT         SPI_CR2_DS_VAL( 9)
#  define SPI_CR2_DS_10BIT        SPI_CR2_DS_VAL(10)
#  define SPI_CR2_DS_11BIT        SPI_CR2_DS_VAL(11)
#  define SPI_CR2_DS_12BIT        SPI_CR2_DS_VAL(12)
#  define SPI_CR2_DS_13BIT        SPI_CR2_DS_VAL(13)
#  define SPI_CR2_DS_14BIT        SPI_CR2_DS_VAL(14)
#  define SPI_CR2_DS_15BIT        SPI_CR2_DS_VAL(15)
#  define SPI_CR2_DS_16BIT        SPI_CR2_DS_VAL(16)
#define SPI_CR2_FRXTH             (1 << 12) /* Bit 12: FIFO reception threshold */
#define SPI_CR2_LDMARX            (1 << 13) /* Bit 13: Last DMA transfer for receptione */
#define SPI_CR2_LDMATX            (1 << 14) /* Bit 14: Last DMA transfer for transmission */

/* SPI status register */

#define SPI_SR_RXNE               (1 << 0)                 /* Bit 0: Receive buffer not empty */
#define SPI_SR_TXE                (1 << 1)                 /* Bit 1: Transmit buffer empty */
#define SPI_SR_CRCERR             (1 << 4)                 /* Bit 4: CRC error flag */
#define SPI_SR_MODF               (1 << 5)                 /* Bit 5: Mode fault */
#define SPI_SR_OVR                (1 << 6)                 /* Bit 6: Overrun flag */
#define SPI_SR_BSY                (1 << 7)                 /* Bit 7: Busy flag */
#define SPI_SR_FRE                (1 << 8)                 /* Bit 8: Frame format error */
#define SPI_SR_FRLVL_SHIFT       (9)                       /* Bits 9-10: FIFO reception level */
#define SPI_SR_FRLVL_MASK        (3 << SPI_SR_FRLVL_SHIFT)
#  define SPI_SR_FRLVL_EMPTY     (0 << SPI_SR_FRLVL_SHIFT) /* FIFO empty */
#  define SPI_SR_FRLVL_QUARTER   (1 << SPI_SR_FRLVL_SHIFT) /* 1/4 FIFO */
#  define SPI_SR_FRLVL_HALF      (2 << SPI_SR_FRLVL_SHIFT) /* 1/2 FIFO */
#  define SPI_SR_FRLVL_FULL      (3 << SPI_SR_FRLVL_SHIFT) /* FIFO full */
#define SPI_SR_FTLVL_SHIFT       (11)                      /* Bits 11-12: FIFO transmission level */
#define SPI_SR_FTLVL_MASK        (3 << SPI_SR_FTLVL_SHIFT)
#  define SPI_SR_FTLVL_EMPTY     (0 << SPI_SR_FTLVL_SHIFT) /* FIFO empty */
#  define SPI_SR_FTLVL_QUARTER   (1 << SPI_SR_FTLVL_SHIFT) /* 1/4 FIFO */
#  define SPI_SR_FTLVL_HALF      (2 << SPI_SR_FTLVL_SHIFT) /* 1/2 FIFO */
#  define SPI_SR_FTLVL_FULL      (3 << SPI_SR_FTLVL_SHIFT) /* FIFO full */

#endif /* __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_SPI_H */
