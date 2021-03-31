/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_spi.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_SPI_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI version **************************************************************/

/* SPI IP v1 is default unless v2 or greater is specified for this chip */

#undef HAVE_SPI_I2S               /* No I2S mode in the SPI peripheral */
#undef HAVE_SPI_I2S_ASTRT         /* No I2S asynchronous start capability */
#undef HAVE_SPI_TI_MODE           /* Motorola frame mode only; no TI mode */
#undef HAVE_SPI_ARB_DATA_SIZE     /* Data size 8 or 16 bit; not arbitrary 4-16 bit */
#undef HAVE_SPI_FIFOS             /* No Tx/Rx FIFOs */
#undef HAVE_SPI_NSSP              /* No NSS Pulse Management in master mode */

#if defined(STM32_HAVE_IP_SPI_V2)
#  define HAVE_SPI_I2S            /* Some SPI peripherals have I2S mode */
#  undef HAVE_SPI_I2S_ASTRT       /* No I2S asynchronous start capability */
#  define HAVE_SPI_TI_MODE        /* Have Motorola and TI frame modes */
#  undef HAVE_SPI_ARB_DATA_SIZE   /* Data size 8 or 16 bit; not arbitrary 4-16 bit */
#  undef HAVE_SPI_FIFOS           /* No Tx/Rx FIFOs */
#  undef HAVE_SPI_NSSP            /* No NSS Pulse Management in master mode */
#endif

#if defined(STM32_HAVE_IP_SPI_V3)
#  define HAVE_SPI_I2S            /* Some SPI peripherals have I2S mode */
#  undef HAVE_SPI_I2S_ASTRT       /* No I2S asynchronous start capability */
#  define HAVE_SPI_TI_MODE        /* Have Motorola and TI frame modes */
#  define HAVE_SPI_ARB_DATA_SIZE  /* Supports arbitrary data size from 4-16 bits */
#  define HAVE_SPI_FIFOS          /* Have Tx/Rx FIFOs */
#  undef HAVE_SPI_NSSP            /* No NSS Pulse Management in master mode */
#endif

#if defined(STM32_HAVE_IP_SPI_V4)
#  define HAVE_SPI_I2S            /* Some SPI peripherals have I2S mode */
#  define HAVE_SPI_I2S_ASTRT      /* Supports I2S asynchronous start capability */
#  define HAVE_SPI_TI_MODE        /* Have Motorola and TI frame modes */
#  define HAVE_SPI_ARB_DATA_SIZE  /* Supports arbitrary data size from 4-16 bits */
#  define HAVE_SPI_FIFOS          /* Have Tx/Rx FIFOs */
#  define HAVE_SPI_NSSP           /* Have NSS Pulse Management in master mode */
#endif

/* Maximum allowed speed as per specifications for all SPIs */

#if defined(CONFIG_STM32_STM32F4XXX)
#  define STM32_SPI_CLK_MAX     37500000UL
#else
#  define STM32_SPI_CLK_MAX     18000000UL
#endif

/* Register Offsets *********************************************************/

#define STM32_SPI_CR1_OFFSET       0x0000  /* SPI Control Register 1 (16-bit) */
#define STM32_SPI_CR2_OFFSET       0x0004  /* SPI control register 2 (16-bit) */
#define STM32_SPI_SR_OFFSET        0x0008  /* SPI status register (16-bit) */
#define STM32_SPI_DR_OFFSET        0x000c  /* SPI data register (16-bit) */
#define STM32_SPI_CRCPR_OFFSET     0x0010  /* SPI CRC polynomial register (16-bit) */
#define STM32_SPI_RXCRCR_OFFSET    0x0014  /* SPI Rx CRC register (16-bit) */
#define STM32_SPI_TXCRCR_OFFSET    0x0018  /* SPI Tx CRC register (16-bit) */

#if defined(HAVE_SPI_I2S)
#  define STM32_SPI_I2SCFGR_OFFSET 0x001c  /* I2S configuration register */
#  define STM32_SPI_I2SPR_OFFSET   0x0020  /* I2S prescaler register */
#endif

/* Register Addresses *******************************************************/

#if STM32_NSPI > 0
#  define STM32_SPI1_CR1          (STM32_SPI1_BASE + STM32_SPI_CR1_OFFSET)
#  define STM32_SPI1_CR2          (STM32_SPI1_BASE + STM32_SPI_CR2_OFFSET)
#  define STM32_SPI1_SR           (STM32_SPI1_BASE + STM32_SPI_SR_OFFSET)
#  define STM32_SPI1_DR           (STM32_SPI1_BASE + STM32_SPI_DR_OFFSET)
#  define STM32_SPI1_CRCPR        (STM32_SPI1_BASE + STM32_SPI_CRCPR_OFFSET)
#  define STM32_SPI1_RXCRCR       (STM32_SPI1_BASE + STM32_SPI_RXCRCR_OFFSET)
#  define STM32_SPI1_TXCRCR       (STM32_SPI1_BASE + STM32_SPI_TXCRCR_OFFSET)
#endif

#if STM32_NSPI > 1
#  define STM32_SPI2_CR1          (STM32_SPI2_BASE + STM32_SPI_CR1_OFFSET)
#  define STM32_SPI2_CR2          (STM32_SPI2_BASE + STM32_SPI_CR2_OFFSET)
#  define STM32_SPI2_SR           (STM32_SPI2_BASE + STM32_SPI_SR_OFFSET)
#  define STM32_SPI2_DR           (STM32_SPI2_BASE + STM32_SPI_DR_OFFSET)
#  define STM32_SPI2_CRCPR        (STM32_SPI2_BASE + STM32_SPI_CRCPR_OFFSET)
#  define STM32_SPI2_RXCRCR       (STM32_SPI2_BASE + STM32_SPI_RXCRCR_OFFSET)
#  define STM32_SPI2_TXCRCR       (STM32_SPI2_BASE + STM32_SPI_TXCRCR_OFFSET)
#  if defined(HAVE_SPI_I2S)
#    define STM32_SPI2_I2SCFGR    (STM32_SPI2_BASE + STM32_SPI_I2SCFGR_OFFSET)
#    define STM32_SPI2_I2SPR      (STM32_SPI2_BASE + STM32_SPI_I2SPR_OFFSET)
#  endif
#endif

#if STM32_NSPI > 2
#  define STM32_SPI3_CR1          (STM32_SPI3_BASE + STM32_SPI_CR1_OFFSET)
#  define STM32_SPI3_CR2          (STM32_SPI3_BASE + STM32_SPI_CR2_OFFSET)
#  define STM32_SPI3_SR           (STM32_SPI3_BASE + STM32_SPI_SR_OFFSET)
#  define STM32_SPI3_DR           (STM32_SPI3_BASE + STM32_SPI_DR_OFFSET)
#  define STM32_SPI3_CRCPR        (STM32_SPI3_BASE + STM32_SPI_CRCPR_OFFSET)
#  define STM32_SPI3_RXCRCR       (STM32_SPI3_BASE + STM32_SPI_RXCRCR_OFFSET)
#  define STM32_SPI3_TXCRCR       (STM32_SPI3_BASE + STM32_SPI_TXCRCR_OFFSET)
#  if defined(HAVE_SPI_I2S)
#    define STM32_SPI3_I2SCFGR    (STM32_SPI3_BASE + STM32_SPI_I2SCFGR_OFFSET)
#    define STM32_SPI3_I2SPR      (STM32_SPI3_BASE + STM32_SPI_I2SPR_OFFSET)
#  endif
#endif

/* Register Bitfield Definitions ********************************************/

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
#if defined(HAVE_SPI_ARB_DATA_SIZE)
#  define SPI_CR1_CRCL            (1 << 11)               /* Bit 11: CRC length */
#else
#  define SPI_CR1_DFF             (1 << 11)               /* Bit 11: Data Frame Format */
#endif
#define SPI_CR1_CRCNEXT           (1 << 12)               /* Bit 12: Transmit CRC next */
#define SPI_CR1_CRCEN             (1 << 13)               /* Bit 13: Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE            (1 << 14)               /* Bit 14: Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE          (1 << 15)               /* Bit 15: Bidirectional data mode enable */

/* SPI Control Register 2 */

#define SPI_CR2_RXDMAEN           (1 << 0)  /* Bit 0: Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN           (1 << 1)  /* Bit 1: Tx Buffer DMA Enable */
#define SPI_CR2_SSOE              (1 << 2)  /* Bit 2: SS Output Enable */

#if defined(HAVE_SPI_NSSP)
#  define SPI_CR2_NSSP            (1 << 3)  /* Bit 3: NSS Pulse Management (Master mode only) */
#endif

#if defined(HAVE_SPI_TI_MODE)
#  define SPI_CR2_FRF             (1 << 4)  /* Bit 4: Frame format: 0=Motorola, 1=TI */
#endif

#define SPI_CR2_ERRIE             (1 << 5)  /* Bit 5: Error interrupt enable */
#define SPI_CR2_RXNEIE            (1 << 6)  /* Bit 6: RX buffer not empty interrupt enable */
#define SPI_CR2_TXEIE             (1 << 7)  /* Bit 7: Tx buffer empty interrupt enable */

#if defined(HAVE_SPI_ARB_DATA_SIZE)
#  define SPI_CR2_DS_SHIFT        (8)       /* Bits 8-11:  Data size */
#  define SPI_CR2_DS_MASK         (15 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS(n)         ((uint32_t)((n) - 1) << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_4BIT       (3 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_5BIT       (4 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_6BIT       (5 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_7BIT       (6 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_8BIT       (7 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_9BIT       (8 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_10BIT      (9 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_11BIT      (10 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_12BIT      (11 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_13BIT      (12 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_14BIT      (13 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_15BIT      (14 << SPI_CR2_DS_SHIFT)
#    define SPI_CR2_DS_16BIT      (15 << SPI_CR2_DS_SHIFT)
#  define SPI_CR2_FRXTH           (1 << 12) /* Bit 12: FIFO reception threshold */
#  define SPI_CR2_LDMARX          (1 << 13) /* Bit 13: Last DMA transfer for reception */
#  define SPI_CR2_LDMATX          (1 << 14) /* Bit 14: Last DMA transfer for transmission */
#endif

/* SPI status register */

#define SPI_SR_RXNE               (1 << 0)  /* Bit 0: Receive buffer not empty */
#define SPI_SR_TXE                (1 << 1)  /* Bit 1: Transmit buffer empty */

#if defined(HAVE_SPI_I2S)
#  define SPI_SR_CHSIDE           (1 << 2)  /* Bit 2: Channel side */
#  define SPI_SR_UDR              (1 << 3)  /* Bit 3: Underrun flag */
#endif

#define SPI_SR_CRCERR             (1 << 4)  /* Bit 4: CRC error flag */
#define SPI_SR_MODF               (1 << 5)  /* Bit 5: Mode fault */
#define SPI_SR_OVR                (1 << 6)  /* Bit 6: Overrun flag */
#define SPI_SR_BSY                (1 << 7)  /* Bit 7: Busy flag */

#if defined(HAVE_SPI_I2S) || defined(HAVE_SPI_TI_MODE)
#  define SPI_SR_FRE              (1 << 8)  /* Bit 8: TI frame format error */
#endif

#if defined(HAVE_SPI_FIFOS)
#  define SPI_SR_FRLVL_SHIFT      (9)                       /* Bits 9-10: FIFO reception level */
#  define SPI_SR_FRLVL_MASK       (3 << SPI_SR_FRLVL_SHIFT)
#    define SPI_SR_FRLVL_EMPTY    (0 << SPI_SR_FRLVL_SHIFT) /* FIFO empty */
#    define SPI_SR_FRLVL_QUARTER  (1 << SPI_SR_FRLVL_SHIFT) /* 1/4 FIFO */
#    define SPI_SR_FRLVL_HALF     (2 << SPI_SR_FRLVL_SHIFT) /* 1/2 FIFO */
#    define SPI_SR_FRLVL_FULL     (3 << SPI_SR_FRLVL_SHIFT) /* FIFO full */
#  define SPI_SR_FTLVL_SHIFT      (11)                      /* Bits 11-12: FIFO transmission level */
#  define SPI_SR_FTLVL_MASK       (3 << SPI_SR_FTLVL_SHIFT)
#    define SPI_SR_FTLVL_EMPTY    (0 << SPI_SR_FTLVL_SHIFT) /* FIFO empty */
#    define SPI_SR_FTLVL_QUARTER  (1 << SPI_SR_FTLVL_SHIFT) /* 1/4 FIFO */
#    define SPI_SR_FTLVL_HALF     (2 << SPI_SR_FTLVL_SHIFT) /* 1/2 FIFO */
#    define SPI_SR_FTLVL_FULL     (3 << SPI_SR_FTLVL_SHIFT) /* FIFO full */
#endif

/* I2S configuration register */

#if defined(HAVE_SPI_I2S)
#  define SPI_I2SCFGR_CHLEN          (1 << 0)                               /* Bit 0: Channel length (number of bits per audio channel) */
#  define SPI_I2SCFGR_DATLEN_SHIFT   (1)                                    /* Bit 1-2: Data length to be transferred */
#  define SPI_I2SCFGR_DATLEN_MASK    (3 << SPI_I2SCFGR_DATLEN_SHIFT)
#    define SPI_I2SCFGR_DATLEN_16BIT (0 << SPI_I2SCFGR_DATLEN_SHIFT)        /* 00: 16-bit data length */
#    define SPI_I2SCFGR_DATLEN_8BIT  (1 << SPI_I2SCFGR_DATLEN_SHIFT)        /* 01: 24-bit data length */
#    define SPI_I2SCFGR_DATLEN_32BIT (2 << SPI_I2SCFGR_DATLEN_SHIFT)        /* 10: 32-bit data length */
#  define SPI_I2SCFGR_CKPOL          (1 << 3)                               /* Bit 3: Steady state clock polarity */
#  define SPI_I2SCFGR_I2SSTD_SHIFT   (4)                                    /* Bit 4-5: I2S standard selection */
#  define SPI_I2SCFGR_I2SSTD_MASK    (3 << SPI_I2SCFGR_I2SSTD_SHIFT)
#    define SPI_I2SCFGR_I2SSTD_PHILLIPS    (0 << SPI_I2SCFGR_I2SSTD_SHIFT)  /* 00: I2S Phillips standard. */
#    define SPI_I2SCFGR_I2SSTD_MSB   (1 << SPI_I2SCFGR_I2SSTD_SHIFT)        /* 01: MSB justified standard (left justified) */
#    define SPI_I2SCFGR_I2SSTD_LSB   (2 << SPI_I2SCFGR_I2SSTD_SHIFT)        /* 10: LSB justified standard (right justified) */
#    define SPI_I2SCFGR_I2SSTD_PCM   (3 << SPI_I2SCFGR_I2SSTD_SHIFT)        /* 11: PCM standard */
#  define SPI_I2SCFGR_PCMSYNC        (1 << 7)                               /* Bit 7: PCM frame synchronization */
#  define SPI_I2SCFGR_I2SCFG_SHIFT   (8)                                    /* Bit 8-9: I2S configuration mode */
#  define SPI_I2SCFGR_I2SCFG_MASK    (3 << SPI_I2SCFGR_I2SCFG_SHIFT)
#    define SPI_I2SCFGR_I2SCFG_STX   (0 << SPI_I2SCFGR_I2SCFG_SHIFT)        /* 00: Slave - transmit */
#    define SPI_I2SCFGR_I2SCFG_SRX   (1 << SPI_I2SCFGR_I2SCFG_SHIFT)        /* 01: Slave - receive */
#    define SPI_I2SCFGR_I2SCFG_MTX   (2 << SPI_I2SCFGR_I2SCFG_SHIFT)        /* 10: Master - transmit */
#    define SPI_I2SCFGR_I2SCFG_MRX   (3 << SPI_I2SCFGR_I2SCFG_SHIFT)        /* 11: Master - receive */
#  define SPI_I2SCFGR_I2SE           (1 << 10)                              /* Bit 10: I2S Enable */
#  define SPI_I2SCFGR_I2SMOD         (1 << 11)                              /* Bit 11: I2S mode selection */
#  if defined(HAVE_SPI_I2S_ASTRT)
#    define SPI_I2SCFGR_ASTRTEN      (1 << 12)                              /* Bit 12: Asynchronous start enable */
#  endif
#endif

/* I2S prescaler register */

#if defined(HAVE_SPI_I2S)
#  define SPI_I2SPR_I2SDIV_SHIFT     (0)       /* Bit 0-7: I2S Linear prescaler */
#  define SPI_I2SPR_I2SDIV_MASK      (0xff << SPI_I2SPR_I2SDIV_SHIFT)
#  define SPI_I2SPR_ODD              (1 << 8)  /* Bit 8: Odd factor for the prescaler */
#  define SPI_I2SPR_MCKOE            (1 << 9)  /* Bit 9: Master clock output enable */
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_SPI_H */
