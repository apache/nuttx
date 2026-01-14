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

#include "bcm2711_memmap.h"
#include <arch/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA channel offsets */

#define BCM_DMA_CH0_OFFSET 0x000
#define BCM_DMA_CH1_OFFSET 0x100
#define BCM_DMA_CH2_OFFSET 0x200
#define BCM_DMA_CH3_OFFSET 0x300
#define BCM_DMA_CH4_OFFSET 0x400
#define BCM_DMA_CH5_OFFSET 0x500
#define BCM_DMA_CH6_OFFSET 0x600
#define BCM_DMA_CH7_OFFSET 0x700
#define BCM_DMA_CH8_OFFSET 0x800
#define BCM_DMA_CH9_OFFSET 0x900
#define BCM_DMA_CH10_OFFSET 0xa00
#define BCM_DMA_CH11_OFFSET 0xb00
#define BCM_DMA_CH12_OFFSET 0xc00
#define BCM_DMA_CH13_OFFSET 0xd00
#define BCM_DMA_CH14_OFFSET 0xe00
#define BCM_DMA_CH15_OFFSET 0x000

/* DMA channel addresses */

#define BCM_DMA0 (BCM_DMA0_BASE + BCM_DMA_CH0_OFFSET)
#define BCM_DMA1 (BCM_DMA0_BASE + BCM_DMA_CH1_OFFSET)
#define BCM_DMA2 (BCM_DMA0_BASE + BCM_DMA_CH2_OFFSET)
#define BCM_DMA3 (BCM_DMA0_BASE + BCM_DMA_CH3_OFFSET)
#define BCM_DMA4 (BCM_DMA0_BASE + BCM_DMA_CH4_OFFSET)
#define BCM_DMA5 (BCM_DMA0_BASE + BCM_DMA_CH5_OFFSET)
#define BCM_DMA6 (BCM_DMA0_BASE + BCM_DMA_CH6_OFFSET)
#define BCM_DMA7 (BCM_DMA0_BASE + BCM_DMA_CH7_OFFSET)   /* Lite */
#define BCM_DMA8 (BCM_DMA0_BASE + BCM_DMA_CH8_OFFSET)   /* Lite */
#define BCM_DMA9 (BCM_DMA0_BASE + BCM_DMA_CH9_OFFSET)   /* Lite */
#define BCM_DMA10 (BCM_DMA0_BASE + BCM_DMA_CH10_OFFSET) /* Lite */
#define BCM_DMA11 (BCM_DMA0_BASE + BCM_DMA_CH11_OFFSET) /* DMA4 */
#define BCM_DMA12 (BCM_DMA0_BASE + BCM_DMA_CH12_OFFSET) /* DMA4 */
#define BCM_DMA13 (BCM_DMA0_BASE + BCM_DMA_CH13_OFFSET) /* DMA4 */
#define BCM_DMA14 (BCM_DMA0_BASE + BCM_DMA_CH14_OFFSET) /* DMA4 */
#define BCM_DMA15 (BCM_DMA15_BASE + BCM_DMA_CH15_OFFSET)

/* DMA control block data structures */

/* DMA control block definition */

struct bcm2711_dma_cb_s
{
  _uint32_t ti;        /* Transfer information */
  _uint32_t source_ad; /* Source address */
  _uint32_t dest_ad;   /* Destination address */
  _uint32_t txfr_len;  /* Transfer length */
  _uint32_t stride;    /* 2D mode stride */
  _uint32_t nextconbk; /* Next control block address */
  _uint32_t _reserved1;
  _uint32_t _reserved2;
};

/* DMA Lite control block definition */

struct bcm2711_dmalite_cb_s
{
  _uint32_t ti;        /* Transfer information */
  _uint32_t source_ad; /* Source address */
  _uint32_t dest_ad;   /* Destination address */
  _uint32_t txfr_len;  /* Transfer length */
  _uint32_t _reserved1;
  _uint32_t nextconbk; /* Next control block address */
  _uint32_t _reserved2;
  _uint32_t _reserved3;
};

/* DMA 4 control block definition */

struct bcm2711_dma4_cb_s
{
  _uint32_t ti;        /* Transfer information */
  _uint32_t src;       /* Source address */
  _uint32_t srci;      /* Source information */
  _uint32_t dest;      /* Destination address */
  _uint32_t desti;     /* Destination information */
  _uint32_t len;       /* Transfer length */
  _uint32_t nextconbk; /* Next control block address */
  _uint32_t _reserved;
};

/* DMA registers offsets */

#define BCM_DMA_CS_OFFSET 0x00        /* Control and status */
#define BCM_DMA_CONBLK_AD_OFFSET 0x04 /* Control block address */
#define BCM_DMA_DEBUG 0x020           /* Debug */

/* TODO: Do I need to do base + offset for the above three for all 14
 * channels?
 */

/* DMA registers */

#define BCM_DMA_INT_STATUS (BCM_DMA0_BASE + 0xfe0) /* Interrupt status */
#define BCM_DMA_INT_ENABLE (BCM_DMA0_BASE + 0xff0) /* Enable bits */

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

#define BCM_DMA4_DESTI_STRIDE (0xffff << 16) /* Destination stride */
#define BCM_DMA4_DESTI_IGNORE (1 << 15)      /* Ignore writes */
#define BCM_DMA4_DESTI_SIZE (0x3 << 13)      /* Destination transfer width */
#define BCM_DMA4_DESTI_INC (1 << 12)         /* Increment dest address */
#define BCM_DMA4_DESTI_BURSTLEN (0xf << 8)   /* Burst transfer length */
#define BCM_DMA4_DESTI_ADDR (0xff)           /* High bits of dest address */

#define BCM_DMA4_LEN_YLENGTH (0x3fff << 16) /* Y transfer len in 2D mode */
#define BCM_DMA4_LEN_XLENGTH (0xffff)       /* X transfer len in bytes */

#define BCM_DMA4_DEBUG2_OUTSREADS (0x1ff << 16) /* Outstanding read count */
#define BCM_DMA4_DEBUG2_OUTSWRITE (0xff)        /* Outstanding write count */

/* Interrupt status register bit definitions */

#define BCM_DMA_INT15 (1 << 15) /* Interrupt status of DMA15 */
#define BCM_DMA_INT14 (1 << 14) /* Interrupt status of DMA14 */
#define BCM_DMA_INT13 (1 << 13) /* Interrupt status of DMA13 */
#define BCM_DMA_INT12 (1 << 12) /* Interrupt status of DMA12 */
#define BCM_DMA_INT11 (1 << 11) /* Interrupt status of DMA11 */
#define BCM_DMA_INT10 (1 << 10) /* Interrupt status of DMA10 */
#define BCM_DMA_INT9 (1 << 9)   /* Interrupt status of DMA9 */
#define BCM_DMA_INT8 (1 << 8)   /* Interrupt status of DMA8 */
#define BCM_DMA_INT7 (1 << 7)   /* Interrupt status of DMA7 */
#define BCM_DMA_INT6 (1 << 6)   /* Interrupt status of DMA6 */
#define BCM_DMA_INT5 (1 << 5)   /* Interrupt status of DMA5 */
#define BCM_DMA_INT4 (1 << 4)   /* Interrupt status of DMA4 */
#define BCM_DMA_INT2 (1 << 2)   /* Interrupt status of DMA2 */
#define BCM_DMA_INT1 (1 << 1)   /* Interrupt status of DMA1 */
#define BCM_DMA_INT0 (1 << 0)   /* Interrupt status of DMA0 */

/* Enable register bit definitions */

#define BCM_DMA_ENABLE_PAGELITE (0xf << 28) /* Set 1G SDRAM page */
#define BCM_DMA_ENABLE_PAGE (0xf << 24)     /* Set 1G SDRAM page */
#define BCM_DMA_ENABLE_EN14 (1 << 14)       /* Enable DMA14 */
#define BCM_DMA_ENABLE_EN13 (1 << 13)       /* Enable DMA13 */
#define BCM_DMA_ENABLE_EN12 (1 << 12)       /* Enable DMA12 */
#define BCM_DMA_ENABLE_EN11 (1 << 11)       /* Enable DMA11 */
#define BCM_DMA_ENABLE_EN10 (1 << 10)       /* Enable DMA10 */
#define BCM_DMA_ENABLE_EN9 (1 << 9)         /* Enable DMA9 */
#define BCM_DMA_ENABLE_EN8 (1 << 8)         /* Enable DMA8 */
#define BCM_DMA_ENABLE_EN7 (1 << 7)         /* Enable DMA7 */
#define BCM_DMA_ENABLE_EN6 (1 << 6)         /* Enable DMA6 */
#define BCM_DMA_ENABLE_EN5 (1 << 5)         /* Enable DMA5 */
#define BCM_DMA_ENABLE_EN4 (1 << 4)         /* Enable DMA4 */
#define BCM_DMA_ENABLE_EN3 (1 << 3)         /* Enable DMA3 */
#define BCM_DMA_ENABLE_EN2 (1 << 2)         /* Enable DMA2 */
#define BCM_DMA_ENABLE_EN1 (1 << 1)         /* Enable DMA1 */
#define BCM_DMA_ENABLE_EN0 (1 << 0)         /* Enable DMA0 */

/* TODO: Section 4.2.1.3 Peripheral DREQ Signals of Datasheet */

#endif /* __ARCH_ARM64_SRC_BCM2711_DMA_H */
