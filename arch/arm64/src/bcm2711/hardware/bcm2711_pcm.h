/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_pcm.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_PCM_H
#define __ARCH_ARM64_SRC_BCM2711_PCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCM register offsets */

#define BCM_PCM_CS_A_OFFSET 0x00
#define BCM_PCM_FIFO_A_OFFSET 0x04
#define BCM_PCM_MODE_A_OFFSET 0x08
#define BCM_PCM_RXC_A_OFFSET 0x0c
#define BCM_PCM_TXC_A_OFFSET 0x10
#define BCM_PCM_DREQ_A_OFFSET 0x14
#define BCM_PCM_INTEN_A_OFFSET 0x18
#define BCM_PCM_INTSTC_A_OFFSET 0x1c
#define BCM_PCM_GRAY_OFFSET 0x20

/* PCM register addresses */

#define _BCM_PCM(offset) (BCM_PCM_BASEADDR + offset)

#define BCM_PCM_CS_A _BCM_PCM(BCM_PCM_CS_A_OFFSET)     /* Control & status */
#define BCM_PCM_FIFO_A _BCM_PCM(BCM_PCM_FIFO_A_OFFSET) /* FIFO data */
#define BCM_PCM_MODE_A _BCM_PCM(BCM_PCM_MODE_A_OFFSET) /* Mode */
#define BCM_PCM_RXC_A _BCM_PCM(BCM_PCM_RXC_A_OFFSET)   /* Receive config */
#define BCM_PCM_TXC_A _BCM_PCM(BCM_PCM_TXC_A_OFFSET)   /* Transmit config */
#define BCM_PCM_DREQ_A _BCM_PCM(BCM_PCM_DREQ_A_OFFSET) /* DMA rqst lvl */
#define BCM_PCM_INTEN_A _BCM_PCM(BCM_PCM_INTEN_A_OFFSET)
#define BCM_PCM_INTSTC_A _BCM_PCM(BCM_PCM_INTSTC_A_OFFSET)
#define BCM_PCM_GRAY _BCM_PCM(BCM_PCM_GRAY_OFFSET) /* Gray mode control */

/* PCM register bit definitions */

#define BCM_PCM_CS_A_SYNC (1 << 24)
#define BCM_PCM_CS_A_RXSEX (1 << 23)
#define BCM_PCM_CS_A_RXF (1 << 22)
#define BCM_PCM_CS_A_TXE (1 << 21)
#define BCM_PCM_CS_A_RXD (1 << 20)
#define BCM_PCM_CS_A_TXD (1 << 19)
#define BCM_PCM_CS_A_RXR (1 << 18)
#define BCM_PCM_CS_A_TXW (1 << 17)
#define BCM_PCM_CS_A_RXERR (1 << 16)
#define BCM_PCM_CS_A_TXERR (1 << 15)
#define BCM_PCM_CS_A_RXSYNC (1 << 14)
#define BCM_PCM_CS_A_TXSYNC (1 << 13)
#define BCM_PCM_CS_A_DMAEN (1 << 9)
#define BCM_PCM_CS_A_RXTHR (0x3 << 7)
#define BCM_PCM_CS_A_RXTHR_SINGLE (0 << 7)
#define BCM_PCM_CS_A_RXTHR_QUARTER (1 << 7)
#define BCM_PCM_CS_A_RXTHR_3QUARTER (2 << 7)
#define BCM_PCM_CS_A_RXTHR_FULL (3 << 7)
#define BCM_PCM_CS_A_TXTHR (0x3 << 5)
#define BCM_PCM_CS_A_TXTHR_EMPTY (0 << 5)
#define BCM_PCM_CS_A_TXTHR_QUARTER (1 << 5)
#define BCM_PCM_CS_A_TXTHR_3QUARTER (2 << 5)
#define BCM_PCM_CS_A_TXTHR_FULL (3 << 5)
#define BCM_PCM_CS_A_RXCLR (1 << 4)
#define BCM_PCM_CS_A_TXCLR (1 << 3)
#define BCM_PCM_CS_A_TXON (1 << 2)
#define BCM_PCM_CS_A_RXON (1 << 1)
#define BCM_PCM_CS_A_EN (1 << 0)

#define BCM_PCM_MODE_A_CLK_DIS (1 << 28)
#define BCM_PCM_MODE_A_PDMN (1 << 27)
#define BCM_PCM_MODE_A_PDME (1 << 26)
#define BCM_PCM_MODE_A_FRXP (1 << 25)
#define BCM_PCM_MODE_A_FTXP (1 << 24)
#define BCM_PCM_MODE_A_CLKM (1 << 23)
#define BCM_PCM_MODE_A_CLKI (1 << 22)
#define BCM_PCM_MODE_A_FSM (1 << 21)
#define BCM_PCM_MODE_A_FSI (1 << 20)
#define BCM_PCM_MODE_A_FLEN (0x3ff << 10)
#define BCM_PCM_MODE_A_FSLEN (0x3ff)

#define BCM_PCM_RXC_A_CH1WEX (1 << 31)
#define BCM_PCM_RXC_A_CH1EN (1 << 30)
#define BCM_PCM_RXC_A_CH1POS (0x3ff << 20)
#define BCM_PCM_RXC_A_CH1WID (0xf << 16)
#define BCM_PCM_RXC_A_CH2WEX (1 << 15)
#define BCM_PCM_RXC_A_CH2EN (1 << 14)
#define BCM_PCM_RXC_A_CH2POS (0x3ff << 4)
#define BCM_PCM_RXC_A_CH2WID (0xf)

#define BCM_PCM_TXC_A_CH1WEX (1 << 31)
#define BCM_PCM_TXC_A_CH1EN (1 << 30)
#define BCM_PCM_TXC_A_CH1POS (0x3ff << 20)
#define BCM_PCM_TXC_A_CH1WID (0xf << 16)
#define BCM_PCM_TXC_A_CH2WEX (1 << 15)
#define BCM_PCM_TXC_A_CH2EN (1 << 14)
#define BCM_PCM_TXC_A_CH2POS (0x3ff << 4)
#define BCM_PCM_TXC_A_CH2WID (0xf)

#define BCM_PCM_DREQ_A_TX_PANIC (0x7f << 24)
#define BCM_PCM_DREQ_A_RX_PANIC (0x7f << 16)
#define BCM_PCM_DREQ_A_RX_REQ (0x7f)

#define BCM_PCM_INTEN_A_RXERR (1 << 3)
#define BCM_PCM_INTEN_A_TXERR (1 << 2)
#define BCM_PCM_INTEN_A_RXR (1 << 1)
#define BCM_PCM_INTEN_A_TXW (1 << 0)

#define BCM_PCM_INTSTC_A_RXERR (1 << 3)
#define BCM_PCM_INTSTC_A_TXERR (1 << 2)
#define BCM_PCM_INTSTC_A_RXR (1 << 1)
#define BCM_PCM_INTSTC_A_TXW (1 << 0)

#define BCM_PCM_GRAY_RXFIFOLEVEL (0x3f << 16)
#define BCM_PCM_GRAY_FLUSHED (0x3f << 10)
#define BCM_PCM_GRAY_RXLEVEL (0x3f << 4)
#define BCM_PCM_GRAY_FLUSH (1 << 2)
#define BCM_PCM_GRAY_CLR (1 << 1)
#define BCM_PCM_GRAY_EN (1 << 0)

#endif /* __ARCH_ARM64_SRC_BCM2711_PCM_H */
