/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_dma.h
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

#ifndef __ARCH_ARM64_SRC_BCM2711_DMA_H
#define __ARCH_ARM64_SRC_BCM2711_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/types.h>

#include <nuttx/compiler.h>

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCM_DMA_CHANNUM (16) /* Number of DMA channels */

/* DMA channel addresses
 *
 * NOTE:
 *
 * DMA 7-10 are DMA Lite channels.
 * DMA 11-14 are DMA4 channels.
 *
 * DMA15 has its own unique base address and cannot be looked up with the
 * BCM_DMA(n) macro. It is exclusively used by the VPU.
 *
 * DMA11 can access the PCIe interface.
 *
 * DMA0 and DMA15 have an external 128-bit 8-word read FIFO.
 */

#define BCM_DMA(n) (BCM_DMA0_BASE + 0x100 * (n))
#define BCM_DMA15 (BCM_DMA15_BASE)

/* DMA control block data structures */

/* DMA control block definition.
 *
 * Must start at a 256-bit aligned address.
 * Fields marked reserved must be zeroed.
 */

begin_packed_struct struct bcm2711_dma_cb_s
{
  volatile _uint32_t ti;        /* Transfer information */
  volatile _uint32_t source_ad; /* Source address */
  volatile _uint32_t dest_ad;   /* Destination address */
  volatile _uint32_t txfr_len;  /* Transfer length */
  volatile _uint32_t stride;    /* 2D mode stride */
  volatile _uint32_t nextconbk; /* Next control block address */
  volatile _uint32_t _reserved1;
  volatile _uint32_t _reserved2;
} aligned_data(32) end_packed_struct;

/* DMA Lite control block definition
 *
 * DMA Lite has a 128 bit internal data structure, so a 128-bit wide read
 * burst will stall the bus if more than 1 beat.
 *
 * 2D transfers are not supported with stride registers.
 *
 * Length register is 16 bits, so max transfer is 65536 bytes.
 */

begin_packed_struct struct bcm2711_dmalite_cb_s
{
  volatile _uint32_t ti;        /* Transfer information */
  volatile _uint32_t source_ad; /* Source address */
  volatile _uint32_t dest_ad;   /* Destination address */
  volatile _uint32_t txfr_len;  /* Transfer length */
  volatile _uint32_t _reserved1;
  volatile _uint32_t nextconbk; /* Next control block address */
  volatile _uint32_t _reserved2;
  volatile _uint32_t _reserved3;
} aligned_data(32) end_packed_struct;

/* DMA 4 control block definition
 *
 * Higher performance; decoupled read/write. Can access 40 address bits. Can
 * perform write bursts.
 */

begin_packed_struct struct bcm2711_dma4_cb_s
{
  volatile _uint32_t ti;        /* Transfer information */
  volatile _uint32_t src;       /* Source address */
  volatile _uint32_t srci;      /* Source information */
  volatile _uint32_t dest;      /* Destination address */
  volatile _uint32_t desti;     /* Destination information */
  volatile _uint32_t len;       /* Transfer length */
  volatile _uint32_t nextconbk; /* Next control block address */
  volatile _uint32_t _reserved;
} aligned_data(32) end_packed_struct;

/* Regular & DMA Lite register offsets */

#define BCM_DMA_CS_OFFSET 0x00        /* Control and status */
#define BCM_DMA_CONBLK_AD_OFFSET 0x04 /* Cntrl blk address */
#define BCM_DMA_TI_OFFSET 0x08        /* Cntrl blk transfer info */
#define BCM_DMA_SOURCE_AD_OFFSET 0x0c /* Cntrl blk source address */
#define BCM_DMA_DEST_AD_OFFSET 0x10   /* Cntrl blk destination address */
#define BCM_DMA_TXFR_LEN_OFFSET 0x14  /* Cntrl blk txfr length remaining */
#define BCM_DMA_STRIDE_OFFSET 0x18    /* Cntrl blk 2D stride */
#define BCM_DMA_NEXTCONBK_OFFSET 0x1c /* Next cntrl blk address */
#define BCM_DMA_DEBUG_OFFSET 0x20     /* Debug */

/* DMA4 register offsets */

#define BCM_DMA4_CS_OFFSET BCM_DMA_CS_OFFSET
#define BCM_DMA4_CB_OFFSET BCM_DMA_CONBLK_AD_OFFSET /* Cntrl blk address */
#define BCM_DMA4_DEBUG_OFFSET 0x0c                  /* Debug */
#define BCM_DMA4_TI_OFFSET 0x10                     /* Cntrl blk txfr info */
#define BCM_DMA4_SRC_OFFSET 0x14     /* Cntrl blk source addr */
#define BCM_DMA4_SRCI_OFFSET 0x18    /* Cntrl blk source addr & info */
#define BCM_DMA4_DEST_OFFSET 0x1c    /* Cntrl blk dest addr */
#define BCM_DMA4_DESTI_OFFSET 0x20   /* Cntrl blk dest addr & info */
#define BCM_DMA4_LEN_OFFSET 0x24     /* Cntrl blk txfr length remaining */
#define BCM_DMA4_NEXT_CB_OFFSET 0x28 /* Next cntrl blk addr */

/* Regular & DMA Lite registers */

#define BCM_DMA_CS(base) ((base) + BCM_DMA_CS_OFFSET)
#define BCM_DMA_CONBLK_AD(base) ((base) + BCM_DMA_CONBLK_AD_OFFSET)
#define BCM_DMA_TI(base) ((base) + BCM_DMA_TI_OFFSET)
#define BCM_DMA_SOURCE_AD(base) ((base) + BCM_DMA_SOURCE_AD_OFFSET)
#define BCM_DMA_DEST_AD(base) ((base) + BCM_DMA_DEST_AD_OFFSET)
#define BCM_DMA_TXFR_LEN(base) ((base) + BCM_DMA_TXFR_LEN_OFFSET)
#define BCM_DMA_STRIDE(base) ((base) + BCM_DMA_STRIDE_OFFSET)
#define BCM_DMA_NEXTCONBK(base) ((base) + BCM_DMA_NEXTCONBK_OFFSET)
#define BCM_DMA_DEBUG(base) ((base) + BCM_DMA_DEBUG_OFFSET)

/* DMA4 registers */

#define BCM_DMA4_CS(base) ((base) + BCM_DMA4_CS_OFFSET)
#define BCM_DMA4_CB(base) ((base) + BCM_DMA4_CB_OFFSET)
#define BCM_DMA4_DEBUG(base) ((base) + BCM_DMA4_DEBUG_OFFSET)
#define BCM_DMA4_TI(base) ((base) + BCM_DMA4_TI_OFFSET)
#define BCM_DMA4_SRC(base) ((base) + BCM_DMA4_SRC_OFFSET)
#define BCM_DMA4_SRCI(base) ((base) + BCM_DMA4_SRCI_OFFSET)
#define BCM_DMA4_DEST(base) ((base) + BCM_DMA4_DEST_OFFSET)
#define BCM_DMA4_DESTI(base) ((base) + BCM_DMA4_DESTI_OFFSET)
#define BCM_DMA4_LEN(base) ((base) + BCM_DMA4_LEN_OFFSET)
#define BCM_DMA4_NEXT_CB(base) ((base) + BCM_DMA4_NEXT_CB_OFFSET)

/* Global DMA registers used for interrupt status and enabling */

#define BCM_DMA_INT_STATUS (BCM_DMA0_BASE + 0xfe0) /* Interrupt status */
#define BCM_DMA_ENABLE (BCM_DMA0_BASE + 0xff0)     /* Enable bits */

/* DMA register bit definitions */

#define BCM_DMA_CS_RESET (1 << 31)       /* Channel reset */
#define BCM_DMA_CS_ABORT (1 << 30)       /* Abort DMA */
#define BCM_DMA_CS_DISDEBUG (1 << 29)    /* Disable debug pause */
#define BCM_DMA_CS_WAIT (1 << 28)        /* Wait for outstanding writes */
#define BCM_DMA_CS_PANICPRIO (0xf << 20) /* AXI panic priority level */
#define BCM_DMA_CS_PRIO (0xf << 16)      /* AXI priority level */
#define BCM_DMA_CS_ERROR (1 << 8)        /* DMA error */
#define BCM_DMA_CS_WAITING (1 << 6)      /* Waiting for outstanding write */
#define BCM_DMA_CS_DREQ_STOPS (1 << 5)   /* DMA paused by DREQ */
#define BCM_DMA_CS_PAUSED (1 << 4)       /* DMA paused */
#define BCM_DMA_CS_DREQ (1 << 3)         /* Requesting = 1, no request = 0*/
#define BCM_DMA_CS_INT (1 << 2)          /* Interrupt status */
#define BCM_DMA_CS_END (1 << 1)          /* DMA end flag */
#define BCM_DMA_CS_ACTIVE (1 << 0)       /* Activate DMA (CB_ADDR non-zero) */

#define BCM_DMA_TI_NOWIBURST (1 << 26)  /* Wide writes not 2 beat burst */
#define BCM_DMA_TI_WAITS (0x1f << 21)   /* Add wait cycles mask */
#define BCM_DMA_TI_PERMAP (0x1f << 16)  /* Peripheral mapping mask */
#define BCM_DMA_TI_BURSTLEN (0xf << 12) /* Burst transfer length */
#define BCM_DMA_TI_SRC_IGNORE (1 << 11) /* Ignore reads */
#define BCM_DMA_TI_SRC_DREQ (1 << 10)   /* Control source reads with DREQ */
#define BCM_DMA_TI_SRC_WIDTH (1 << 9)   /* 1 = 128 bit, 0 = 32 bit */
#define BCM_DMA_TI_SRC_INC (1 << 8)     /* Increment src addr after read */
#define BCM_DMA_TI_DEST_IGNORE (1 << 7) /* Ignore writes */
#define BCM_DMA_TI_DEST_DREQ (1 << 6)   /* Control dest writes with DREQ */
#define BCM_DMA_TI_DEST_WIDTH (1 << 5)  /* 1 = 128 bit, 0 = 32 bit */
#define BCM_DMA_TI_DEST_INC (1 << 4)    /* Increment destination address */
#define BCM_DMA_TI_WAIT_RESP (1 << 3)   /* Wait for write response */
#define BCM_DMA_TI_TDMODE (1 << 1)      /* 2D mode */
#define BCM_DMA_TI_INTEN (1 << 0)       /* Interrupt enable */

#define BCM_DMA_TXFR_LEN_YLENGTH (0x3fff << 16) /* Y txfr len in 2D mode */
#define BCM_DMA_TXFR_LEN_XLENGTH (0xffff)       /* Transfer len in bytes */

#define BCM_DMA_STRIDE_D_STRIDE (0xffff << 16) /* Dest stride in 2D mode */
#define BCM_DMA_STRIDE_S_STRIDE (0xffff)       /* Source stride in 2D mode */

#define BCM_DMA_DEBUG_LITE (1 << 28)          /* DMA lite */
#define BCM_DMA_DEBUG_VERSION (0x7 << 25)     /* DMA version number */
#define BCM_DMA_DEBUG_DMA_STATE (0x1ff << 16) /* DMA state machine state */
#define BCM_DMA_DEBUG_DMA_ID (0xff << 8)      /* DMA AXI ID */
#define BCM_DMA_DEBUG_OUTSWRITES (0xf << 4)   /* Outstanding writes count */
#define BCM_DMA_DEBUG_READ_ERROR (1 << 2)     /* Slave read response error */
#define BCM_DMA_DEBUG_FIFO_ERROR (1 << 1)     /* FIFO error */
#define BCM_DMA_DEBUG_RDLASTNSET_ERR (1 << 0) /* Read last not set error */

/* DMA4 registers; TODO: how to differentiate from other types? */

#define BCM_DMA_CS_HALT (1 << 31)       /* Halt current DMA transfer */
#define BCM_DMA_CS_OUTSTRANS (1 << 25)  /* Outstanding transactions */
#define BCM_DMA_CS_DMABUSY (1 << 24)    /* DMA4 is busy */
#define BCM_DMA_CS_PANICQOS (0xf << 20) /* AXI panic QOS level */
#define BCM_DMA_CS_QOS (0xf << 16)      /* AXI QOS level */
#define BCM_DMA4_CS_ERROR (1 << 10)     /* AXI QOS level */
#define BCM_DMA4_CS_WAITING (1 << 7)    /* Waiting for outstanding writes */
#define BCM_DMA4_CS_DREQ_STOPS (1 << 6) /* Paused by DREQ */
#define BCM_DMA_CS_WR_PAUSED (1 << 5)   /* Write paused */
#define BCM_DMA_CS_RD_PAUSED (1 << 4)   /* Read paused */
#define BCM_DMA4_CS_DREQ (1 << 3)       /* DREQ state */

#define BCM_DMA4_DEBUG_VERSION (0xf << 28)   /* DMA version number */
#define BCM_DMA4_DEBUG_ID (0xf << 24)        /* DMA ID */
#define BCM_DMA4_DEBUG_RESET (1 << 23)       /* DMA reset */
#define BCM_DMA4_DEBUG_W_STATE (0xf << 18)   /* Write state machine state */
#define BCM_DMA4_DEBUG_R_STATE (0xf << 14)   /* Read state machine state */
#define BCM_DMA4_DEBUG_DIS_CLKGATE (1 << 11) /* Disable clk gating logic */
#define BCM_DMA4_DEBUG_ABORT_ERR (1 << 10)   /* Abort on error */
#define BCM_DMA4_DEBUG_HALT_ERR (1 << 9)     /* Halt on error */
#define BCM_DMA4_DEBUG_INT_ERR (1 << 8)      /* Interrupt on error */
#define BCM_DMA4_DEBUG_READ_CB_ERR (1 << 3)  /* Slave error on CB read */
#define BCM_DMA4_DEBUG_READ_ERROR (1 << 2)   /* Slave read response error */
#define BCM_DMA4_DEBUG_FIFO_ERROR (1 << 1)   /* FIFO error */
#define BCM_DMA4_DEBUG_WRITE_ERROR (1 << 0)  /* Slave write response error */

#define BCM_DMA4_TI_D_WAITS (0xff << 24)  /* Write wait cycles */
#define BCM_DMA4_TI_S_WAITS (0xff << 16)  /* Read wait cycles */
#define BCM_DMA4_TI_D_DREQ (1 << 15)      /* Control dest writes with DREQ */
#define BCM_DMA4_TI_S_DREQ (1 << 14)      /* Control src reads with DREQ */
#define BCM_DMA4_TI_S_PERMAP (0x1f << 9)  /* Peripheral mapping */
#define BCM_DMA4_TI_WAIT_RD_RESP (1 << 3) /* Wait for read response */
#define BCM_DMA4_TI_WAIT_RESP (1 << 2)    /* Wait for rite response */
#define BCM_DMA4_TI_TDMODE (1 << 1)       /* 2D mode transfer */
#define BCM_DMA4_TI_INTEN (1 << 0)        /* Interrupt enable */

#define BCM_DMA4_SRCI_STRIDE (0xffff << 16) /* Source stride */
#define BCM_DMA4_SRCI_IGNORE (1 << 15)      /* Ignore reads */
#define BCM_DMA4_SRCI_SIZE (0x3 << 13)      /* Source transfer width */
#define BCM_DMA4_SRCI_INC (1 << 12)         /* Increment source address */
#define BCM_DMA4_SRCI_BURSTLEN (0xf << 8)   /* Burst transfer length */
#define BCM_DMA4_SRCI_ADDR (0xff)           /* High bits of source address */
#define BCM_DMA4_SRCI_ADDR_SHIFT (32)       /* Bit shift high bits down */

#define BCM_DMA4_DESTI_STRIDE (0xffff << 16) /* Destination stride */
#define BCM_DMA4_DESTI_IGNORE (1 << 15)      /* Ignore writes */
#define BCM_DMA4_DESTI_SIZE (0x3 << 13)      /* Destination transfer width */
#define BCM_DMA4_DESTI_INC (1 << 12)         /* Increment dest address */
#define BCM_DMA4_DESTI_BURSTLEN (0xf << 8)   /* Burst transfer length */
#define BCM_DMA4_DESTI_ADDR (0xff)           /* High bits of dest address */
#define BCM_DMA4_DESTI_ADDR_SHIFT (32)       /* Bit shift high bits down */

#define BCM_DMA4_LEN_YLENGTH (0x3fff << 16) /* Y transfer len in 2D mode */
#define BCM_DMA4_LEN_XLENGTH (0xffff)       /* X transfer len in bytes */

#define BCM_DMA4_DEBUG2_OUTSREADS (0x1ff << 16) /* Outstanding read count */
#define BCM_DMA4_DEBUG2_OUTSWRITE (0xff)        /* Outstanding write count */

/* Interrupt status register bit definitions */

#define BCM_DMA_INT_STATUS_INT(n)                                            \
  (1 << (n)) /* Interrupt status of DMA channel 'n' */

/* Enable register bit definitions */

#define BCM_DMA_ENABLE_PAGELITE (0xf << 28) /* Set 1G SDRAM page */
#define BCM_DMA_ENABLE_PAGE (0xf << 24)     /* Set 1G SDRAM page */
#define BCM_DMA_ENABLE_EN(n) (1 << (n))     /* Enable DMA channel 'n' */

/* TODO: Section 4.2.1.3 Peripheral DREQ Signals of Datasheet */

#endif /* __ARCH_ARM64_SRC_BCM2711_DMA_H */
