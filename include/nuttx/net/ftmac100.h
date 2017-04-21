/****************************************************************************
 * include/nuttx/net/ftmac100.h
 * Faraday FTMAC100 Ethernet MAC Definitions
 *
 *   Copyright (C) 2015 Anton D. Kachalov. All rights reserved.
 *   Author: Anton D. Kachalov <mouse@yandex.ru>
 *
 * Based on definitions provided by Po-Yu Chuang <ratbert@faraday-tech.com>
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

#ifndef __INCLUDE_NUTTX_NET_FTMAC100_H
#define __INCLUDE_NUTTX_NET_FTMAC100_H

/****************************************************************************'
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_FTMAC100

/****************************************************************************'
 * Pre-processor definitions
 ****************************************************************************/

/* Interrupt status register & interrupt mask register definitions*/

#define FTMAC100_INT_RPKT_FINISH      (1 << 0)
#define FTMAC100_INT_NORXBUF          (1 << 1)
#define FTMAC100_INT_XPKT_FINISH      (1 << 2)
#define FTMAC100_INT_NOTXBUF          (1 << 3)
#define FTMAC100_INT_XPKT_OK          (1 << 4)
#define FTMAC100_INT_XPKT_LOST        (1 << 5)
#define FTMAC100_INT_RPKT_SAV         (1 << 6)
#define FTMAC100_INT_RPKT_LOST        (1 << 7)
#define FTMAC100_INT_AHB_ERR          (1 << 8)
#define FTMAC100_INT_PHYSTS_CHG       (1 << 9)

/* Interrupt timer control register */

#define FTMAC100_ITC_RXINT_CNT(x)     (((x) & 0xf) << 0)
#define FTMAC100_ITC_RXINT_THR(x)     (((x) & 0x7) << 4)
#define FTMAC100_ITC_RXINT_TIME_SEL   (1 << 7)
#define FTMAC100_ITC_TXINT_CNT(x)     (((x) & 0xf) << 8)
#define FTMAC100_ITC_TXINT_THR(x)     (((x) & 0x7) << 12)
#define FTMAC100_ITC_TXINT_TIME_SEL   (1 << 15)

/* Automatic polling timer control register definitions */

#define FTMAC100_APTC_RXPOLL_CNT(x)   (((x) & 0xf) << 0)
#define FTMAC100_APTC_RXPOLL_TIME_SEL (1 << 4)
#define FTMAC100_APTC_TXPOLL_CNT(x)   (((x) & 0xf) << 8)
#define FTMAC100_APTC_TXPOLL_TIME_SEL (1 << 12)

/* DMA burst length and arbitration control register definitions */

#define FTMAC100_DBLAC_INCR4_EN       (1 << 0)
#define FTMAC100_DBLAC_INCR8_EN       (1 << 1)
#define FTMAC100_DBLAC_INCR16_EN      (1 << 2)
#define FTMAC100_DBLAC_RXFIFO_LTHR(x) (((x) & 0x7) << 3)
#define FTMAC100_DBLAC_RXFIFO_HTHR(x) (((x) & 0x7) << 6)
#define FTMAC100_DBLAC_RX_THR_EN      (1 << 9)

/* MAC control register definitions */

#define FTMAC100_MACCR_XDMA_EN        (1 << 0)
#define FTMAC100_MACCR_RDMA_EN        (1 << 1)
#define FTMAC100_MACCR_SW_RST         (1 << 2)
#define FTMAC100_MACCR_LOOP_EN        (1 << 3)
#define FTMAC100_MACCR_CRC_DIS        (1 << 4)
#define FTMAC100_MACCR_XMT_EN         (1 << 5)
#define FTMAC100_MACCR_ENRX_IN_HALFTX (1 << 6)
#define FTMAC100_MACCR_RCV_EN         (1 << 8)
#define FTMAC100_MACCR_HT_MULTI_EN    (1 << 9)
#define FTMAC100_MACCR_RX_RUNT        (1 << 10)
#define FTMAC100_MACCR_RX_FTL         (1 << 11)
#define FTMAC100_MACCR_RCV_ALL        (1 << 12)
#define FTMAC100_MACCR_CRC_APD        (1 << 14)
#define FTMAC100_MACCR_FULLDUP        (1 << 15)
#define FTMAC100_MACCR_RX_MULTIPKT    (1 << 16)
#define FTMAC100_MACCR_RX_BROADPKT    (1 << 17)

/* PHY control register definitions */

#define FTMAC100_PHYCR_MIIRDATA       0xffff
#define FTMAC100_PHYCR_PHYAD(x)       (((x) & 0x1f) << 16)
#define FTMAC100_PHYCR_REGAD(x)       (((x) & 0x1f) << 21)
#define FTMAC100_PHYCR_MIIRD          (1 << 26)
#define FTMAC100_PHYCR_MIIWR          (1 << 27)

/* Transmit descriptor definitions */

#define FTMAC100_TXDES0_TXPKT_LATECOL (1 << 0)
#define FTMAC100_TXDES0_TXPKT_EXSCOL  (1 << 1)
#define FTMAC100_TXDES0_TXDMA_OWN     (1 << 31)

#define FTMAC100_TXDES1_TXBUF_SIZE(x) ((x) & 0x7ff)
#define FTMAC100_TXDES1_LTS           (1 << 27)
#define FTMAC100_TXDES1_FTS           (1 << 28)
#define FTMAC100_TXDES1_TX2FIC        (1 << 29)
#define FTMAC100_TXDES1_TXIC          (1 << 30)
#define FTMAC100_TXDES1_EDOTR         (1 << 31)

/* Receive descriptor definitions */

#define FTMAC100_RXDES0_RFL(des)      ((des) & 0x7ff)
#define FTMAC100_RXDES0_MULTICAST     (1 << 16)
#define FTMAC100_RXDES0_BROADCAST     (1 << 17)
#define FTMAC100_RXDES0_RX_ERR        (1 << 18)
#define FTMAC100_RXDES0_CRC_ERR       (1 << 19)
#define FTMAC100_RXDES0_FTL           (1 << 20)
#define FTMAC100_RXDES0_RUNT          (1 << 21)
#define FTMAC100_RXDES0_RX_ODD_NB     (1 << 22)
#define FTMAC100_RXDES0_LRS           (1 << 28)
#define FTMAC100_RXDES0_FRS           (1 << 29)
#define FTMAC100_RXDES0_RXDMA_OWN     (1 << 31)

#define FTMAC100_RXDES1_RXBUF_SIZE(x) ((x) & 0x7ff)
#define FTMAC100_RXDES1_EDORR         (1 << 31)

/****************************************************************************'
 * Public Types
 ****************************************************************************/

struct ftmac100_register_s
{
  uint32_t isr;                   /* 0x00 */
  uint32_t imr;                   /* 0x04 */
  uint32_t mac_madr;              /* 0x08 */
  uint32_t mac_ladr;              /* 0x0c */
  uint32_t maht0;                 /* 0x10 */
  uint32_t maht1;                 /* 0x14 */
  uint32_t txpd;                  /* 0x18 */
  uint32_t rxpd;                  /* 0x1c */
  uint32_t txr_badr;              /* 0x20 */
  uint32_t rxr_badr;              /* 0x24 */
  uint32_t itc;                   /* 0x28 */
  uint32_t aptc;                  /* 0x2c */
  uint32_t dblac;                 /* 0x30 */
  uint32_t pad1[3];               /* 0x34 - 0x3c */
  uint32_t pad2[16];              /* 0x40 - 0x7c */
  uint32_t pad3[2];               /* 0x80 - 0x84 */
  uint32_t maccr;                 /* 0x88 */
  uint32_t macsr;                 /* 0x8c */
  uint32_t phycr;                 /* 0x90 */
  uint32_t phywdata;              /* 0x94 */
  uint32_t fcr;                   /* 0x98 */
  uint32_t bpr;                   /* 0x9c */
  uint32_t pad4[8];               /* 0xa0 - 0xbc */
  uint32_t pad5;                  /* 0xc0 */
  uint32_t ts;                    /* 0xc4 */
  uint32_t dmafifos;              /* 0xc8 */
  uint32_t tm;                    /* 0xcc */
  uint32_t pad6;                  /* 0xd0 */
  uint32_t tx_mcol_scol;          /* 0xd4 */
  uint32_t rpf_aep;               /* 0xd8 */
  uint32_t xm_pg;                 /* 0xdc */
  uint32_t runt_tlcc;             /* 0xe0 */
  uint32_t crcer_ftl;             /* 0xe4 */
  uint32_t rlc_rcc;               /* 0xe8 */
  uint32_t broc;                  /* 0xec */
  uint32_t mulca;                 /* 0xf0 */
  uint32_t rp;                    /* 0xf4 */
  uint32_t xp;                    /* 0xf8 */
};

/* Transmit descriptor, aligned to 16 bytes */

struct ftmac100_txdes_s
{
  uint32_t txdes0;
  uint32_t txdes1;
  uint32_t txdes2;  /* TXBUF_BADR */
  uint32_t txdes3;  /* not used by HW */
} __attribute__ ((aligned(16)));

/* Receive descriptor, aligned to 16 bytes */

struct ftmac100_rxdes_s
{
  uint32_t rxdes0;
  uint32_t rxdes1;
  uint32_t rxdes2;  /* RXBUF_BADR */
  uint32_t rxdes3;  /* not used by HW */
} __attribute__ ((aligned(16)));

/****************************************************************************'
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Name: ftmac100_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ftmac100_initialize(int intf);

#endif /* CONFIG_NET_FTMAC100 */
#endif /* __INCLUDE_NUTTX_NET_FTMAC100_H */
