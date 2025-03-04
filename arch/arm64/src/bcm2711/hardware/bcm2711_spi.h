/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_spi.h
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

#ifndef __ARCH_ARM64_SRC_BCM2711_SPI_H
#define __ARCH_ARM64_SRC_BCM2711_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI register offsets */

#define BCM_SPI_CS_OFFSET 0x00
#define BCM_SPI_FIFO_OFFSET 0x04
#define BCM_SPI_CLK_OFFSET 0x08
#define BCM_SPI_DLEN_OFFSET 0x0c
#define BCM_SPI_LTOH_OFFSET 0x10
#define BCM_SPI_DC_OFFSET 0x14

/* SPI register addresses */

#define BCM_SPI0_CS (BCM_SPI0_BASEADDR + BCM_SPI_CS_OFFSET)
#define BCM_SPI0_FIFO (BCM_SPI0_BASEADDR + BCM_SPI_FIFO_OFFSET)
#define BCM_SPI0_CLK (BCM_SPI0_BASEADDR + BCM_SPI_CLK_OFFSET)
#define BCM_SPI0_DLEN (BCM_SPI0_BASEADDR + BCM_SPI_DLEN_OFFSET)
#define BCM_SPI0_LTOH (BCM_SPI0_BASEADDR + BCM_SPI_LTOH_OFFSET)
#define BCM_SPI0_DC (BCM_SPI0_BASEADDR + BCM_SPI_DC_OFFSET)

#define BCM_SPI3_CS (BCM_SPI3_BASEADDR + BCM_SPI_CS_OFFSET)
#define BCM_SPI3_FIFO (BCM_SPI3_BASEADDR + BCM_SPI_FIFO_OFFSET)
#define BCM_SPI3_CLK (BCM_SPI3_BASEADDR + BCM_SPI_CLK_OFFSET)
#define BCM_SPI3_DLEN (BCM_SPI3_BASEADDR + BCM_SPI_DLEN_OFFSET)
#define BCM_SPI3_LTOH (BCM_SPI3_BASEADDR + BCM_SPI_LTOH_OFFSET)
#define BCM_SPI3_DC (BCM_SPI3_BASEADDR + BCM_SPI_DC_OFFSET)

#define BCM_SPI4_CS (BCM_SPI4_BASEADDR + BCM_SPI_CS_OFFSET)
#define BCM_SPI4_FIFO (BCM_SPI4_BASEADDR + BCM_SPI_FIFO_OFFSET)
#define BCM_SPI4_CLK (BCM_SPI4_BASEADDR + BCM_SPI_CLK_OFFSET)
#define BCM_SPI4_DLEN (BCM_SPI4_BASEADDR + BCM_SPI_DLEN_OFFSET)
#define BCM_SPI4_LTOH (BCM_SPI4_BASEADDR + BCM_SPI_LTOH_OFFSET)
#define BCM_SPI4_DC (BCM_SPI4_BASEADDR + BCM_SPI_DC_OFFSET)

#define BCM_SPI5_CS (BCM_SPI5_BASEADDR + BCM_SPI_CS_OFFSET)
#define BCM_SPI5_FIFO (BCM_SPI5_BASEADDR + BCM_SPI_FIFO_OFFSET)
#define BCM_SPI5_CLK (BCM_SPI5_BASEADDR + BCM_SPI_CLK_OFFSET)
#define BCM_SPI5_DLEN (BCM_SPI5_BASEADDR + BCM_SPI_DLEN_OFFSET)
#define BCM_SPI5_LTOH (BCM_SPI5_BASEADDR + BCM_SPI_LTOH_OFFSET)
#define BCM_SPI5_DC (BCM_SPI5_BASEADDR + BCM_SPI_DC_OFFSET)

#define BCM_SPI6_CS (BCM_SPI6_BASEADDR + BCM_SPI_CS_OFFSET)
#define BCM_SPI6_FIFO (BCM_SPI6_BASEADDR + BCM_SPI_FIFO_OFFSET)
#define BCM_SPI6_CLK (BCM_SPI6_BASEADDR + BCM_SPI_CLK_OFFSET)
#define BCM_SPI6_DLEN (BCM_SPI6_BASEADDR + BCM_SPI_DLEN_OFFSET)
#define BCM_SPI6_LTOH (BCM_SPI6_BASEADDR + BCM_SPI_LTOH_OFFSET)
#define BCM_SPI6_DC (BCM_SPI6_BASEADDR + BCM_SPI_DC_OFFSET)

/* SPI register bit definitions */

#define BCM_SPI_CS_LEN_LONG (1 << 25)
#define BCM_SPI_CS_DMA_LEN (1 << 24)
#define BCM_SPI_CS_CSPOL2 (1 << 23)
#define BCM_SPI_CS_CSPOL1 (1 << 22)
#define BCM_SPI_CS_CSPOL0 (1 << 21)
#define BCM_SPI_CS_RXF (1 << 20)
#define BCM_SPI_CS_RXR (1 << 19)
#define BCM_SPI_CS_TXD (1 << 18)
#define BCM_SPI_CS_RXD (1 << 17)
#define BCM_SPI_CS_DONE (1 << 16)
#define BCM_SPI_CS_TE_EN (1 << 15)
#define BCM_SPI_CS_LMONO (1 << 14)
#define BCM_SPI_CS_LEN (1 << 13)
#define BCM_SPI_CS_REN (1 << 12)
#define BCM_SPI_CS_ADCS (1 << 11)
#define BCM_SPI_CS_INTR (1 << 10)
#define BCM_SPI_CS_INTD (1 << 9)
#define BCM_SPI_CS_DMAEN (1 << 8)
#define BCM_SPI_CS_TA (1 << 7)
#define BCM_SPI_CS_CSPOL (1 << 6)
#define BCM_SPI_CS_CLEAR (0x3 << 4)
#define BCM_SPI_CS_NOCLEAR (0 << 4)
#define BCM_SPI_CS_CLEARTX (1 << 4)
#define BCM_SPI_CS_CLEARRX (2 << 4)
#define BCM_SPI_CS_CPOL (1 << 3)
#define BCM_SPI_CS_CPHA (1 << 2)
#define BCM_SPI_CS_CS (0x3 << 0)
#define BCM_SPI_CS_CS0 (0 << 0)
#define BCM_SPI_CS_CS1 (1 << 0)
#define BCM_SPI_CS_CS2 (2 << 0)

#define BCM_SPI_CLK_CDIV (0xffff << 0)

#define BCM_SPI_DLEN_LEN (0xffff << 0)

#define BCM_SPI_LTOH_TOH (0xf << 0)

#define BCM_SPI_DC_RPANIC (0xff << 24)
#define BCM_SPI_DC_RDREQ (0xff << 16)
#define BCM_SPI_DC_TPANIC (0xff << 8)
#define BCM_SPI_DC_TDREQ (0xff << 0)

#endif /* __ARCH_ARM64_SRC_BCM2711_SPI_H */
