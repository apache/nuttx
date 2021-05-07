/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_i2s.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_I2S_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC17_40_I2S_DAO_OFFSET         0x0000 /* Digital Audio Output Register */
#define LPC17_40_I2S_DAI_OFFSET         0x0004 /* Digital Audio Input Register */
#define LPC17_40_I2S_TXFIFO_OFFSET      0x0008 /* Transmit FIFO */
#define LPC17_40_I2S_RXFIFO_OFFSET      0x000c /* Receive FIFO */
#define LPC17_40_I2S_STATE_OFFSET       0x0010 /* Status Feedback Register */
#define LPC17_40_I2S_DMA1_OFFSET        0x0014 /* DMA Configuration Register 1 */
#define LPC17_40_I2S_DMA2_OFFSET        0x0018 /* DMA Configuration Register 2 */
#define LPC17_40_I2S_IRQ_OFFSET         0x001c /* Interrupt Request Control Register */
#define LPC17_40_I2S_TXRATE_OFFSET      0x0020 /* Transmit MCLK divider */
#define LPC17_40_I2S_RXRATE_OFFSET      0x0024 /* Receive MCLK divider */
#define LPC17_40_I2S_TXBITRATE_OFFSET   0x0028 /* Transmit bit rate divider */
#define LPC17_40_I2S_RXBITRATE_OFFSET   0x002c /* Receive bit rate divider */
#define LPC17_40_I2S_TXMODE_OFFSET      0x0030 /* Transmit mode control */
#define LPC17_40_I2S_RXMODE_OFFSET      0x0034 /* Receive mode control */

/* Register addresses *******************************************************/

#define LPC17_40_I2S_DAO                (LPC17_40_I2S_BASE+LPC17_40_I2S_DAO_OFFSET)
#define LPC17_40_I2S_DAI                (LPC17_40_I2S_BASE+LPC17_40_I2S_DAI_OFFSET)
#define LPC17_40_I2S_TXFIFO             (LPC17_40_I2S_BASE+LPC17_40_I2S_TXFIFO_OFFSET)
#define LPC17_40_I2S_RXFIFO             (LPC17_40_I2S_BASE+LPC17_40_I2S_RXFIFO_OFFSET)
#define LPC17_40_I2S_STATE              (LPC17_40_I2S_BASE+LPC17_40_I2S_STATE_OFFSET)
#define LPC17_40_I2S_DMA1               (LPC17_40_I2S_BASE+LPC17_40_I2S_DMA1_OFFSET)
#define LPC17_40_I2S_DMA2               (LPC17_40_I2S_BASE+LPC17_40_I2S_DMA2_OFFSET)
#define LPC17_40_I2S_IRQ                (LPC17_40_I2S_BASE+LPC17_40_I2S_IRQ_OFFSET)
#define LPC17_40_I2S_TXRATE             (LPC17_40_I2S_BASE+LPC17_40_I2S_TXRATE_OFFSET)
#define LPC17_40_I2S_RXRATE             (LPC17_40_I2S_BASE+LPC17_40_I2S_RXRATE_OFFSET)
#define LPC17_40_I2S_TXBITRATE          (LPC17_40_I2S_BASE+LPC17_40_I2S_TXBITRATE_OFFSET)
#define LPC17_40_I2S_RXBITRATE          (LPC17_40_I2S_BASE+LPC17_40_I2S_RXBITRATE_OFFSET)
#define LPC17_40_I2S_TXMODE             (LPC17_40_I2S_BASE+LPC17_40_I2S_TXMODE_OFFSET)
#define LPC17_40_I2S_RXMODE             (LPC17_40_I2S_BASE+LPC17_40_I2S_RXMODE_OFFSET)

/* Register bit definitions *************************************************/

/* Digital Audio Output Register */

#define I2S_DAO_WDWID_SHIFT             (0)       /* Bits 0-1: Selects the number of bytes in data */
#define I2S_DAO_WDWID_MASK              (3 << I2S_DAO_WDWID_SHIFT)
#  define I2S_DAO_WDWID_8BITS           (0 << I2S_DAO_WDWID_SHIFT)
#  define I2S_DAO_WDWID_16BITS          (1 << I2S_DAO_WDWID_SHIFT)
#  define I2S_DAO_WDWID_32BITS          (3 << I2S_DAO_WDWID_SHIFT)
#define I2S_DAO_MONO                    (1 << 2)  /* Bit 2:  Mono format */
#define I2S_DAO_STOP                    (1 << 3)  /* Bit 3:  Disable FIFOs / mute mode */
#define I2S_DAO_RESET                   (1 << 4)  /* Bit 4:  Reset TX channel and FIFO */
#define I2S_DAO_WSSEL                   (1 << 5)  /* Bit 5:  Slave mode select */
#define I2S_DAO_WSHALFPER_SHIFT         (6)       /* Bits 6-14: Word select half period minus 1 */
#define I2S_DAO_WSHALFPER_MASK          (0x01ff << I2S_DAO_WSHALFPER_SHIFT)
#define I2S_DAO_MUTE                    (1 << 15) /* Bit 15: Send only zeros on channel */
                                                  /* Bits 16-31: Reserved */

/* Digital Audio Input Register */

#define I2S_DAI_WDWID_SHIFT             (0)       /* Bits 0-1: Selects the number of bytes in data */
#define I2S_DAI_WDWID_MASK              (3 << I2S_DAI_WDWID_SHIFT)
#  define I2S_DAI_WDWID_8BITS           (0 << I2S_DAI_WDWID_SHIFT)
#  define I2S_DAI_WDWID_16BITS          (1 << I2S_DAI_WDWID_SHIFT)
#  define I2S_DAI_WDWID_32BITS          (3 << I2S_DAI_WDWID_SHIFT)
#define I2S_DAI_MONO                    (1 << 2)  /* Bit 2:  Mono format */
#define I2S_DAI_STOP                    (1 << 3)  /* Bit 3:  Disable FIFOs / mute mode */
#define I2S_DAI_RESET                   (1 << 4)  /* Bit 4:  Reset TX channel and FIFO */
#define I2S_DAI_WSSEL                   (1 << 5)  /* Bit 5:  Slave mode select */
#define I2S_DAI_WSHALFPER_SHIFT         (6)       /* Bits 6-14: Word select half period minus 1 */
#define I2S_DAI_WSHALFPER_MASK          (0x01ff << I2S_DAI_WSHALFPER_SHIFT)
                                                  /* Bits 15-31: Reserved */

/* Transmit FIFO: 8 - 32-bit transmit FIFO
 * Receive FIFO: 8 - 32-bit receive FIFO
 */

/* Status Feedback Register */

#define I2S_STATE_IRQ                   (1 << 0)  /* Bit 0:  Receive Transmit Interrupt */
#define I2S_STATE_DMAREQ1               (1 << 1)  /* Bit 1:  Receive or Transmit DMA Request 1 */
#define I2S_STATE_DMAREQ2               (1 << 2)  /* Bit 2:  Receive or Transmit DMA Request 2 */
                                                  /* Bits 3-7: Reserved */
#define I2S_STATE_RXLEVEL_SHIFT         (8)       /* Bits 8-11: Current level of the Receive FIFO */
#define I2S_STATE_RXLEVEL_MASK          (15 << I2S_STATE_RXLEVEL_SHIFT)
                                                  /* Bits 12-15: Reserved */
#define I2S_STATE_TXLEVEL_SHIFT         (16)      /* Bits 16-19: Current level of the Transmit FIFO */
#define I2S_STATE_TXLEVEL_MASK          (15 << I2S_STATE_TXLEVEL_SHIFT)
                                                  /* Bits 20-31: Reserved */

/* DMA Configuration Register 1 and 2 */

#define I2S_DMA_RXDMAEN                 (1 << 0)  /* Bit 0:  Enable DMA1 for I2S receive */
#define I2S_DMA_TXDMAEN                 (1 << 1)  /* Bit 1:  Enable DMA1 for I2S transmit */
                                                  /* Bits 2-7: Reserved */
#define I2S_DMA_RXDEPTH_SHIFT           (8)       /* Bits 8-11: FIFO level that triggers RX request on DMA1 */
#define I2S_DMA_RXDEPTH_MASK            (15 << I2S_DMA_RXDEPTH_SHIFT)
                                                  /* Bits 12-15: Reserved */
#define I2S_DMA_TXDEPTH_SHIFT           (16)      /* Bits 16-19: FIFO level that triggers a TX request on DMA1 */
#define I2S_DMA_TXDEPTH_MASK            (15 << I2S_DMA_TXDEPTH_SHIFT)
                                                  /* Bits 20-31: Reserved */

/* Interrupt Request Control Register */

#define I2S_IRQ_RXEN                    (1 << 0)  /* Bit 0:  Enable I2S receive interrupt */
#define I2S_IRQ_TXEN                    (1 << 1)  /* Bit 1:  Enable I2S transmit interrupt */
                                                  /* Bits 2-7: Reserved */
#define I2S_IRQ_RXDEPTH_SHIFT           (8)       /* Bits 8-11: Set FIFO level for irq request */
#define I2S_IRQ_RXDEPTH_MASK            (15 << I2S_IRQ_RXDEPTH_SHIFT)
                                                  /* Bits 12-15: Reserved */
#define I2S_IRQ_TXDEPTH_SHIFT           (16)      /* Bits 16-19: Set FIFO level for irq request */
#define I2S_IRQ_TXDEPTH_MASK            (15 << I2S_IRQ_TXDEPTH_SHIFT)
                                                  /* Bits 20-31: Reserved */

/* Transmit and Receive MCLK divider */

#define I2S_RATE_YDIV_SHIFT             (0)       /* Bits 0-7: I2S transmit MCLK rate denominator */
#define I2S_RATE_YDIV_MASK              (0xff << I2S_RATE_YDIV_SHIFT)
#define I2S_RATE_XDIV_SHIFT             (8)       /* Bits 8-15: I2S transmit MCLK rate numerator */
#define I2S_RATE_XDIV_MASK              (0xff << I2S_RATE_XDIV_SHIFT)
                                                  /* Bits 16-31: Reserved */

/* Transmit and received bit rate divider */

#define I2S_BITRATE_SHIFT               (0)       /* Bits 0-5: I2S transmit bit rate */
#define I2S_BITRATE_MASK                (0x3f << I2S_BITRATE_SHIFT)
                                                  /* Bits 6-31: Reserved */

/* Transmit and Receive mode control */

#define I2S_MODE_CLKSEL_SHIFT           (0)       /* Bits 0-1: Clock source for bit clock divider */
#define I2S_MODE_CLKSEL_MASK            (3 << I2S_MODE_CLKSEL_SHIFT)
#  define I2S_MODE_CLKSEL_FRACDIV       (0 << I2S_MODE_CLKSEL_SHIFT) /* TX/RX fractional rate divider */
#  define I2S_MODE_CLKSEL_RXMCLK        (2 << I2S_MODE_CLKSEL_SHIFT) /* RX_CLCK for TX_MCLK source */
#  define I2S_MODE_CLKSEL_TXMCLK        (2 << I2S_MODE_CLKSEL_SHIFT) /* TX_CLCK for RX_MCLK source */

#define I2S_MODE_4PIN                   (1 << 2)  /* Bit 2:  Transmit/Receive 4-pin mode selection */
#define I2S_MODE_MCENA                  (1 << 3)  /* Bit 3:  Enable for the TX/RX_MCLK output */
                                                  /* Bits 4-31: Reserved */

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_I2S_H */
