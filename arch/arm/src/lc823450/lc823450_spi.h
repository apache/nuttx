/****************************************************************************
 * arch/arm/src/lc823450/lc823450_spi.h
 *
 *   Copyright 2014,2017 Sony Video & Sound Products Inc.
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SPI_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define LC823450_SPI_REGBASE   0x40088000

#define LC823450_SPI_STR    (LC823450_SPI_REGBASE + 0x00) /* Transfer */
#define LC823450_SPI_SRR    (LC823450_SPI_REGBASE + 0x04) /* Receive */
#define LC823450_SPI_SMD    (LC823450_SPI_REGBASE + 0x08) /* Mode */
#define LC823450_SPI_SSR    (LC823450_SPI_REGBASE + 0x0C) /* Status */
#define LC823450_SPI_BRG    (LC823450_SPI_REGBASE + 0x10) /* Baudrate Generator */
#define LC823450_SPI_ISR    (LC823450_SPI_REGBASE + 0x14) /* Interrupt Factor */
#define LC823450_SPI_DREQ   (LC823450_SPI_REGBASE + 0x18) /* DMA Request */
#define LC823450_SPI_TxFF   (LC823450_SPI_REGBASE + 0x1C) /* Transfer FIFO */
#define LC823450_SPI_RxFF   (LC823450_SPI_REGBASE + 0x20) /* Receive FIFO */
#define LC823450_SPI_FFCTL  (LC823450_SPI_REGBASE + 0x24) /* FIFO Control */
#define LC823450_SPI_MSK    (LC823450_SPI_REGBASE + 0x28) /* Interrupt Mask */
#define LC823450_SPI_INT    (LC823450_SPI_REGBASE + 0x2C) /* Interrupt Status */
#define LC823450_SPI_CSHT   (LC823450_SPI_REGBASE + 0x30) /* CS Setup/Hold time (not supported) */
#define LC823450_SPI_CSMD   (LC823450_SPI_REGBASE + 0x34) /* CS Mode (not supported) */

/* Register Bitfield Definitions ********************************************/

/* SPI Mode Register */

#define SPI_SMD_DMS_SHIFT   (8)
#define SPI_SMD_DMS_MASK    (3 << SPI_SMD_DMS_SHIFT)  /* Bits 9:8: Baud Rate Control */
#  define SPI_SMD_DMS_MANU  (0 << SPI_SMD_DMS_SHIFT)
#  define SPI_SMD_DMS_2TCYC (1 << SPI_SMD_DMS_SHIFT)
#  define SPI_SMD_DMS_4TCYC (2 << SPI_SMD_DMS_SHIFT)
#define SPI_SMD_REGCLR      (1 << 6)   /* Bit 6: Tx/Rx data register clear */
#define SPI_SMD_WTR         (1 << 5)   /* Bit 5; Burst transfer enable */
#define SPI_SMD_CHL         (1 << 4)   /* Bit 4: Transfer bit length */
#define SPI_SMD_PO          (1 << 3)   /* Bit 3: SCK Polarity */
#define SPI_SMD_LM          (1 << 2)   /* Bit 2: MSB first selection */
#define SPI_SMD_BGE         (1 << 1)   /* Bit 1: Baudrate generator enable */
#define SPI_SMD_SSTR        (1 << 0)   /* Bit 0: Frame transfer enable */

/* SPI Status Register */

#define SPI_SSR_TFF         (1 << 2)   /* Bit 2: STR register full */
#define SPI_SSR_SHRF        (1 << 1)   /* Bit 1: Shift register full */
#define SPI_SSR_RRF         (1 << 0)   /* Bit 0: SRR register full */

/* SPI Interrupt Factor Register */

#define SPI_ISR_CS_END      (1 << 14)  /* Bit 14: CS completion (not supported) */
#define SPI_ISR_BURST_END   (1 << 13)  /* Bit 13: Burst transfer completion */
#define SPI_ISR_RxORE       (1 << 12)  /* Bit 12: Rx FIFO overread */
#define SPI_ISR_TxORE       (1 << 11)  /* Bit 11: Tx FIFO overread */
#define SPI_ISR_RxOWE       (1 << 10)  /* Bit 10: Rx FIFO overwrite */
#define SPI_ISR_TxOWE       (1 << 9)   /* Bit  9: Tx FIFO overwrite */
#define SPI_ISR_RxFULL      (1 << 8)   /* Bit  8: Rx FIFO full */
#define SPI_ISR_TxFULL      (1 << 7)   /* Bit  7: Tx FIFO full */
#define SPI_ISR_RxEMP       (1 << 6)   /* Bit  6: Rx FIFO empty */
#define SPI_ISR_TxEMP       (1 << 5)   /* Bit  5: Tx FIFO empty */
#define SPI_ISR_RxWLM       (1 << 4)   /* Bit  4: Rx FIFO water level match */
#define SPI_ISR_TxWLM       (1 << 3)   /* Bit  3: Tx FIFO water level match */
#define SPI_ISR_ROWE        (1 << 2)   /* Bit  2: SRR register overwrite */
#define SPI_ISR_OVE         (1 << 1)   /* Bit  1: overrun */
#define SPI_ISR_SPIF        (1 << 0)   /* Bit  0: Frame transfer completion */

/* SPI FIFO contorl Register */

#define SPI_TxFF_EN         (1 << 0)
#define SPI_TxFF_WL2        (0 << 4)
#define SPI_TxFF_WL4        (1 << 4)
#define SPI_TxFF_WL8        (2 << 4)
#define SPI_TxFF_WL12       (3 << 4)
#define SPI_TxFF_WL14       (4 << 4)

/* SPI Interrupt Mask Register */

#define SPI_MSK_M_CS_END    (1 << 14)
#define SPI_MSK_M_BURST_END (1 << 13)
#define SPI_MSK_M_RxORE     (1 << 12)
#define SPI_MSK_M_TxORE     (1 << 11)
#define SPI_MSK_M_RxOWE     (1 << 10)
#define SPI_MSK_M_TxOWE     (1 << 9)
#define SPI_MSK_M_RxFULL    (1 << 8)
#define SPI_MSK_M_TxFULL    (1 << 7)
#define SPI_MSK_M_RxEMP     (1 << 6)
#define SPI_MSK_M_TxEMP     (1 << 5)
#define SPI_MSK_M_RxWLM     (1 << 4)
#define SPI_MSK_M_TxWLM     (1 << 3)
#define SPI_MSK_M_ROWE      (1 << 2)
#define SPI_MSK_M_OVE       (1 << 1)
#define SPI_MSK_M_SPIF      (1 << 0)

/* SPI DMA Request */

#define SPI_DREQ_DREQ_RX    (3 << 0)
#define SPI_DREQ_DREQ_TX    (2 << 0)

/* SPI Interrupt Status Register */

#define SPI_INT_I_CS_END    (1 << 14)
#define SPI_INT_I_BURST_END (1 << 13)
#define SPI_INT_I_RxORE     (1 << 12)
#define SPI_INT_I_TxORE     (1 << 11)
#define SPI_INT_I_RxOWE     (1 << 10)
#define SPI_INT_I_TxOWE     (1 << 9)
#define SPI_INT_I_RxFULL    (1 << 8)
#define SPI_INT_I_TxFULL    (1 << 7)
#define SPI_INT_I_RxEMP     (1 << 6)
#define SPI_INT_I_TxEMP     (1 << 5)
#define SPI_INT_I_RxWLM     (1 << 4)
#define SPI_INT_I_TxWLM     (1 << 3)
#define SPI_INT_I_ROWE      (1 << 2)
#define SPI_INT_I_OVE       (1 << 1)
#define SPI_INT_I_SPIF      (1 << 0)

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct spi_dev_s *lc823450_spibus_initialize(int bus);
void lc823450_spiinitialize(void);
void lc823450_spiselect(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t lc823450_spistatus(FAR struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lc823450_spicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SPI_H */
