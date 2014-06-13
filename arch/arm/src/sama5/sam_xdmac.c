/****************************************************************************
 * arch/arm/src/sama5/sam_xdmac.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "cache.h"
#include "up_internal.h"
#include "os_internal.h"

#include "chip.h"
#include "sam_dmac.h"
#include "sam_periphclks.h"
#include "sam_memories.h"
#include "chip/sam_pmc.h"
#include "chip/sam_xdmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* All of the currently supported SAMA5 chips support two DMA controllers
 * of 8 DMA Channels each.
 */

#if SAM_NDMAC < 1
#  undef CONFIG_SAMA5_XDMAC1
#  undef CONFIG_SAMA5_XDMAC0
#elif SAM_NDMAC < 2
#  undef CONFIG_SAMA5_XDMAC1
#endif

/* Condition out the whole file unless DMA is selected in the configuration */

#if defined(CONFIG_SAMA5_XDMAC0) || defined(CONFIG_SAMA5_XDMAC1)

/* If SAMA5 DMA support is enabled, then OS DMA support should also be
 * enabled
 */

#ifndef CONFIG_ARCH_DMA
#  warning "SAMA5 DMA enabled but CONFIG_ARCH_DMA disabled"
#endif

/* Check the number of link list descriptors to allocate */

#ifndef CONFIG_SAMA5_NLLDESC
#  define CONFIG_SAMA5_NLLDESC SAM_NDMACHAN
#endif

#if CONFIG_SAMA5_NLLDESC < SAM_NDMACHAN
#  warning "At least SAM_NDMACHAN descriptors must be allocated"

#  undef CONFIG_SAMA5_NLLDESC
#  define CONFIG_SAMA5_NLLDESC SAM_NDMACHAN
#endif

/* Register values **********************************************************/

#define XDMAC_CH_CTRLB_BOTHDSCR \
  (XDMAC_CH_CTRLB_SRCDSCR | XDMAC_CH_CTRLB_DSTDSCR)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure maps a peripheral ID to an DMA channel */

struct sam_pidmap_s
{
  uint8_t pid;                    /* Peripheral identifier */
  uint8_t pchan;                  /* DMA channel */
};

/* This structure descibes one DMA channel */

struct sam_xdmach_s
{
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
  uint8_t xdmac;                  /* DMA controller number (0-1) */
#endif
  uint8_t chan;                   /* DMA channel number (0-15) */
  bool inuse;                     /* TRUE: The DMA channel is in use */
  bool rx;                        /* TRUE: Peripheral to memory transfer */
  uint32_t flags;                 /* DMA channel flags */
  uint32_t base;                  /* DMA register channel base address */
  uint32_t cfg;                   /* Pre-calculated CFG register for transfer */
  dma_callback_t callback;        /* Callback invoked when the DMA completes */
  void *arg;                      /* Argument passed to callback function */
  uint32_t rxaddr;                /* RX memory address */
  size_t rxsize;                  /* Size of RX memory region */
  struct dma_linklist_s *llhead;  /* DMA link list head */
  struct dma_linklist_s *lltail;  /* DMA link list head */
};

/* This structure describes the stae of one DMA controller */

struct sam_xdmac_s
{
  /* These semaphores protect the DMA channel and descriptor tables */

  sem_t chsem;                       /* Protects channel table */
  sem_t dsem;                        /* Protects descriptor table */
  uint32_t base;                     /* DMA register channel base address */

  /* This array describes the available link list descriptors */

  struct dma_linklist_s *desc;

  /* This array describes each DMA channel */

  struct sam_xdmach_s *xdmach;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* CTRLA field lookups */

static const uint32_t g_chanwidth[4] =
{
  XDMACH_CC_DWIDTH_BYTE,
  XDMACH_CC_DWIDTH_HWORD,
  XDMACH_CC_DWIDTH_WORD,
  XDMACH_CC_DWIDTH_DWORD
};

static const uint32_t g_fifocfg[3] =
{
  XDMAC_CH_CFG_FIFOCFG_ALAP,
  XDMAC_CH_CFG_FIFOCFG_HALF,
  XDMAC_CH_CFG_FIFOCFG_ASAP
};

/* These tables map peripheral IDs to channels.  A lookup is performed
 * before each DMA transfer in order to map the peripheral IDs to the
 * correct channel.  This must be done because the channel can change with
 * the direction of the transfer.
 */

#ifdef CONFIG_SAMA5_XDMAC0
/* DMA controller 0, RX DMA: */

static const struct sam_pidmap_s g_xdmac0_rxchan[] =
{
  { SAM_PID_HSMCI0, XDMAC0_CH_HSMCI0    }, /* HSMCI0 Receive/Transmit */
  { SAM_PID_HSMCI1, XDMAC0_CH_HSMCI1    }, /* HSMCI1 Receive/Transmit */
  { SAM_PID_TWI0,   XDMAC0_CH_TWI0_RX   }, /* TWI0 Receive */
  { SAM_PID_TWI1,   XDMAC0_CH_TWI1_RX   }, /* TWI1 Receive */
  { SAM_PID_TWI2,   XDMAC0_CH_TWI2_RX   }, /* TWI2 Receive */
  { SAM_PID_TWI3,   XDMAC0_CH_TWI3_RX   }, /* TWI3 Receive */
  { SAM_PID_SPI0,   XDMAC0_CH_SPI0_RX   }, /* SPI0 Receive */
  { SAM_PID_SPI1,   XDMAC0_CH_SPI1_RX   }, /* SPI1 Receive */
  { SAM_PID_SPI2,   XDMAC0_CH_SPI2_RX   }, /* SPI2 Receive */
  { SAM_PID_USART2, XDMAC0_CH_USART2_RX }, /* USART2 Receive */
  { SAM_PID_USART3, XDMAC0_CH_USART3_RX }, /* USART3 Receive */
  { SAM_PID_USART4, XDMAC0_CH_USART4_RX }, /* USART4 Receive */
  { SAM_PID_UART0,  XDMAC0_CH_UART0_RX  }, /* UART0 Receive */
  { SAM_PID_UART1,  XDMAC0_CH_UART1_RX  }, /* UART1 Receive */
  { SAM_PID_SSC0,   XDMAC0_CH_SSC0_RX   }, /* SSC0 Receive */
  { SAM_PID_SSC1,   XDMAC0_CH_SSC1_RX   }, /* SSC1 Receive */
  { SAM_PID_DBGU,   XDMAC0_CH_DBGU_RX   }, /* DBGU Receive */
  { SAM_PID_ADC,    XDMAC0_CH_ADC_RX    }, /* ADC Receive */
  { SAM_PID_SMD,    XDMAC0_CH_SMD_RX    }, /* SMD Receive */
  { SAM_PID_USART0, XDMAC0_CH_USART0_RX }, /* USART0 Receive */
  { SAM_PID_USART1, XDMAC0_CH_USART1_RX }, /* USART1 Receive */
  { SAM_PID_AES,    XDMAC0_CH_AES_RX    }, /* AES Receive */
  { SAM_PID_TDES,   XDMAC0_CH_TDES_RX   }, /* TDES Receive */
  { SAM_PID_CATB,   XDMAC0_CH_CATB_RX   }, /* CATB Receive */
};
#define NXDMAC0_RXCHANNELS (sizeof(g_xdmac0_rxchan) / sizeof(struct sam_pidmap_s))

/* DMA controller 0, TX DMA: */

static const struct sam_pidmap_s g_xdmac0_txchan[] =
{
  { SAM_PID_HSMCI0, XDMAC0_CH_HSMCI0    }, /* HSMCI0 Receive/Transmit */
  { SAM_PID_HSMCI1, XDMAC0_CH_HSMCI1    }, /* HSMCI1 Receive/Transmit */
  { SAM_PID_TWI0,   XDMAC0_CH_TWI0_TX   }, /* TWI0 Transmit */
  { SAM_PID_TWI1,   XDMAC0_CH_TWI1_TX   }, /* TWI1 Transmit */
  { SAM_PID_TWI2,   XDMAC0_CH_TWI2_TX   }, /* TWI2 Transmit */
  { SAM_PID_TWI3,   XDMAC0_CH_TWI3_TX   }, /* TWI3 Transmit */
  { SAM_PID_SPI0,   XDMAC0_CH_SPI0_TX   }, /* SPI0 Transmit */
  { SAM_PID_SPI1,   XDMAC0_CH_SPI1_TX   }, /* SPI1 Transmit */
  { SAM_PID_SPI2,   XDMAC0_CH_SPI2_TX   }, /* SPI2 Transmit */
  { SAM_PID_USART2, XDMAC0_CH_USART2_TX }, /* USART2 Transmit */
  { SAM_PID_USART3, XDMAC0_CH_USART3_TX }, /* USART3 Transmit */
  { SAM_PID_USART4, XDMAC0_CH_USART4_TX }, /* USART4 Transmit */
  { SAM_PID_UART0,  XDMAC0_CH_UART0_TX  }, /* UART0 Transmit */
  { SAM_PID_UART1,  XDMAC0_CH_UART1_TX  }, /* UART1 Transmit */
  { SAM_PID_SSC0,   XDMAC0_CH_SSC0_TX   }, /* SSC0 Transmit */
  { SAM_PID_SSC1,   XDMAC0_CH_SSC1_TX   }, /* SSC1 Transmit */
  { SAM_PID_DBGU,   XDMAC0_CH_DBGU_TX   }, /* DBGU Transmit */
  { SAM_PID_SMD,    XDMAC0_CH_SMD_TX    }, /* SMD Transmit */
  { SAM_PID_USART0, XDMAC0_CH_USART0_TX }, /* USART0 Transmit */
  { SAM_PID_USART1, XDMAC0_CH_USART1_TX }, /* USART1 Transmit */
  { SAM_PID_AES,    XDMAC0_CH_AES_TX    }, /* AES Transmit */
  { SAM_PID_TDES,   XDMAC0_CH_TDES_TX   }, /* TDES Transmit */
  { SAM_PID_SHA,    XDMAC0_CH_SHA_TX    }, /* SHA Transmit */
  { SAM_PID_CATB,   XDMAC0_CH_CATB_TX   }, /* CATB Transmit */
};
#define NXDMAC0_TXCHANNELS (sizeof(g_xdmac0_txchan) / sizeof(struct sam_pidmap_s))
#endif

#ifdef CONFIG_SAMA5_XDMAC1
/* DMA controller 1, RX DMA: */

static const struct sam_pidmap_s g_xdmac1_rxchan[] =
{
  { SAM_PID_HSMCI0, XDMAC1_CH_HSMCI0    }, /* HSMCI0 Receive/Transmit */
  { SAM_PID_HSMCI1, XDMAC1_CH_HSMCI1    }, /* HSMCI1 Receive/Transmit */
  { SAM_PID_TWI0,   XDMAC1_CH_TWI0_RX   }, /* TWI0 Receive */
  { SAM_PID_TWI1,   XDMAC1_CH_TWI1_RX   }, /* TWI1 Receive */
  { SAM_PID_TWI2,   XDMAC1_CH_TWI2_RX   }, /* TWI2 Receive */
  { SAM_PID_TWI3,   XDMAC1_CH_TWI3_RX   }, /* TWI3 Receive */
  { SAM_PID_SPI0,   XDMAC1_CH_SPI0_RX   }, /* SPI0 Receive */
  { SAM_PID_SPI1,   XDMAC1_CH_SPI1_RX   }, /* SPI1 Receive */
  { SAM_PID_SPI2,   XDMAC1_CH_SPI2_RX   }, /* SPI2 Receive */
  { SAM_PID_USART2, XDMAC1_CH_USART2_RX }, /* USART2 Receive */
  { SAM_PID_USART3, XDMAC1_CH_USART3_RX }, /* USART3 Receive */
  { SAM_PID_USART4, XDMAC1_CH_USART4_RX }, /* USART4 Receive */
  { SAM_PID_UART0,  XDMAC1_CH_UART0_RX  }, /* UART0 Receive */
  { SAM_PID_UART1,  XDMAC1_CH_UART1_RX  }, /* UART1 Receive */
  { SAM_PID_SSC0,   XDMAC1_CH_SSC0_RX   }, /* SSC0 Receive */
  { SAM_PID_SSC1,   XDMAC1_CH_SSC1_RX   }, /* SSC1 Receive */
  { SAM_PID_DBGU,   XDMAC1_CH_DBGU_RX   }, /* DBGU Receive */
  { SAM_PID_ADC,    XDMAC1_CH_ADC_RX    }, /* ADC Receive */
  { SAM_PID_SMD,    XDMAC1_CH_SMD_RX    }, /* SMD Receive */

};
#define NXDMAC1_RXCHANNELS (sizeof(g_xdmac1_rxchan) / sizeof(struct sam_pidmap_s))

/* DMA controller 1, TX DMA: */

static const struct sam_pidmap_s g_xdmac1_txchan[] =
{
  { SAM_PID_HSMCI0, XDMAC1_CH_HSMCI0    }, /* HSMCI0 Receive/Transmit */
  { SAM_PID_HSMCI1, XDMAC1_CH_HSMCI1    }, /* HSMCI1 Receive/Transmit */
  { SAM_PID_TWI0,   XDMAC1_CH_TWI0_TX   }, /* TWI0 Transmit */
  { SAM_PID_TWI1,   XDMAC1_CH_TWI1_TX   }, /* TWI1 Transmit */
  { SAM_PID_TWI2,   XDMAC1_CH_TWI2_TX   }, /* TWI2 Transmit */
  { SAM_PID_TWI3,   XDMAC1_CH_TWI3_TX   }, /* TWI3 Transmit */
  { SAM_PID_SPI0,   XDMAC1_CH_SPI0_TX   }, /* SPI0 Transmit */
  { SAM_PID_SPI1,   XDMAC1_CH_SPI1_TX   }, /* SPI1 Transmit */
  { SAM_PID_SPI2,   XDMAC1_CH_SPI2_TX   }, /* SPI2 Transmit */
  { SAM_PID_USART2, XDMAC1_CH_USART2_TX }, /* USART2 Transmit */
  { SAM_PID_USART3, XDMAC1_CH_USART3_TX }, /* USART3 Transmit */
  { SAM_PID_USART4, XDMAC1_CH_USART4_TX }, /* USART4 Transmit */
  { SAM_PID_UART0,  XDMAC1_CH_UART0_TX  }, /* UART0 Transmit */
  { SAM_PID_UART1,  XDMAC1_CH_UART1_TX  }, /* UART1 Transmit */
  { SAM_PID_SSC0,   XDMAC1_CH_SSC0_TX   }, /* SSC0 Transmit */
  { SAM_PID_SSC1,   XDMAC1_CH_SSC1_TX   }, /* SSC1 Transmit */
  { SAM_PID_DBGU,   XDMAC1_CH_DBGU_TX   }, /* DBGU Transmit */
  { SAM_PID_SMD,    XDMAC1_CH_SMD_TX    }, /* SMD Transmit */
};
#define NXDMAC1_TXCHANNELS (sizeof(g_xdmac1_txchan) / sizeof(struct sam_pidmap_s))
#endif

#ifdef CONFIG_SAMA5_XDMAC0

/* This array describes the available link list descriptors */

struct dma_linklist_s g_desc0[CONFIG_SAMA5_NLLDESC];

/* This array describes the state of each XDMAC0 channel 0 */

static struct sam_xdmach_s g_xdmach0[SAM_NDMACHAN] =
{
#if SAM_NDMACHAN > 0
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 0,
    .base     = SAM_XDMAC0_CH0_BASE,
  },
#endif
#if SAM_NDMACHAN > 1
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 1,
    .base     = SAM_XDMAC0_CH1_BASE,
  },
#endif
#if SAM_NDMACHAN > 2
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 2,
    .base     = SAM_XDMAC0_CH2_BASE,
  },
#endif
#if SAM_NDMACHAN > 3
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 3,
    .base     = SAM_XDMAC0_CH3_BASE,
  },
#endif
#if SAM_NDMACHAN > 4
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 4,
    .base     = SAM_XDMAC0_CH4_BASE,
  },
#endif
#if SAM_NDMACHAN > 5
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 5,
    .base     = SAM_XDMAC0_CH5_BASE,
  },
#endif
#if SAM_NDMACHAN > 6
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 6,
    .base     = SAM_XDMAC0_CH6_BASE,
  },
#endif
#if SAM_NDMACHAN > 7
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 7,
    .base     = SAM_XDMAC0_CH7_BASE,
  },
#endif
#if SAM_NDMACHAN > 8
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 8,
    .base     = SAM_XDMAC0_CH8_BASE,
  },
#endif
#if SAM_NDMACHAN > 9
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 9,
    .base     = SAM_XDMAC0_CH9_BASE,
  },
#endif
#if SAM_NDMACHAN > 10
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 10,
    .base     = SAM_XDMAC0_CH10_BASE,
  },
#endif
#if SAM_NDMACHAN > 11
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 11,
    .base     = SAM_XDMAC0_CH11_BASE,
  },
#endif
#if SAM_NDMACHAN > 12
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 12,
    .base     = SAM_XDMAC0_CH12_BASE,
  },
#endif
#if SAM_NDMACHAN > 13
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 13,
    .base     = SAM_XDMAC0_CH13_BASE,
  },
#endif
#if SAM_NDMACHAN > 14
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 14,
    .base     = SAM_XDMAC0_CH14_BASE,
  },
#endif
#if SAM_NDMACHAN > 15
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 0,
#endif
    .chan     = 15,
    .base     = SAM_XDMAC0_CH15_BASE,
  }
#endif
};

/* This describes the overall state of DMA controller 0 */

static struct sam_xdmac_s g_xdmac0 =
{
  /* XDMAC 0 base address */

  .base       = SAM_XDMAC0_VBASE,

  /* This array describes the available link list descriptors */

  .desc       = g_desc0,

  /* This array describes each DMA channel */

  .xdmach     = g_xdmach0,
};

#endif /* CONFIG_SAMA5_XDMAC0 */

/* This array describes the state of DMA controller 1 */

#ifdef CONFIG_SAMA5_XDMAC1
/* This array describes the available link list descriptors */

struct dma_linklist_s g_desc1[CONFIG_SAMA5_NLLDESC];

static struct sam_xdmach_s g_xdmach1[SAM_NDMACHAN] =
{
#if SAM_NDMACHAN > 0
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 0,
    .base     = SAM_XDMAC1_CH0_BASE,
  },
#endif
#if SAM_NDMACHAN > 1
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 1,
    .base     = SAM_XDMAC1_CH1_BASE,
  },
#endif
#if SAM_NDMACHAN > 2
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 2,
    .base     = SAM_XDMAC1_CH2_BASE,
  },
#endif
#if SAM_NDMACHAN > 3
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 3,
    .base     = SAM_XDMAC1_CH3_BASE,
  },
#endif
#if SAM_NDMACHAN > 4
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 4,
    .base     = SAM_XDMAC1_CH4_BASE,
  },
#endif
#if SAM_NDMACHAN > 5
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 5,
    .base     = SAM_XDMAC1_CH5_BASE,
  },
#endif
#if SAM_NDMACHAN > 6
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 6,
    .base     = SAM_XDMAC1_CH6_BASE,
  },
#endif
#if SAM_NDMACHAN > 7
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 7,
    .base     = SAM_XDMAC1_CH7_BASE,
  },
#endif
#if SAM_NDMACHAN > 8
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 8,
    .base     = SAM_XDMAC1_CH8_BASE,
  },
#endif
#if SAM_NDMACHAN > 9
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 9,
    .base     = SAM_XDMAC1_CH9_BASE,
  },
#endif
#if SAM_NDMACHAN > 10
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 10,
    .base     = SAM_XDMAC1_CH10_BASE,
  },
#endif
#if SAM_NDMACHAN > 11
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 11,
    .base     = SAM_XDMAC1_CH11_BASE,
  },
#endif
#if SAM_NDMACHAN > 12
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 12,
    .base     = SAM_XDMAC1_CH12_BASE,
  },
#endif
#if SAM_NDMACHAN > 13
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 13,
    .base     = SAM_XDMAC1_CH13_BASE,
  },
#endif
#if SAM_NDMACHAN > 14
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 14,
    .base     = SAM_XDMAC1_CH14_BASE,
  },
#endif
#if SAM_NDMACHAN > 15
  {
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
    .xdmac    = 1,
#endif
    .chan     = 15,
    .base     = SAM_XDMAC1_CH15_BASE,
  }
#endif
};

/* This describes the overall state of DMA controller 1 */

static struct sam_xdmac_s g_xdmac1 =
{
  /* XDMAC 0 base address */

  .base       = SAM_XDMAC1_VBASE,

  /* This array describes the available link list descriptors */

  .desc       = g_desc1,

  /* This array describes each DMA channel */

  .xdmach     = g_xdmach1,
};

#endif /* CONFIG_SAMA5_XDMAC1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_takechsem() and sam_givechsem()
 *
 * Description:
 *   Used to get exclusive access to the DMA channel table
 *
 ****************************************************************************/

static void sam_takechsem(struct sam_xdmac_s *xdmac)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&xdmac->chsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam_givechsem(struct sam_xdmac_s *xdmac)
{
  (void)sem_post(&xdmac->chsem);
}

/****************************************************************************
 * Name: sam_takedsem() and sam_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

static void sam_takedsem(struct sam_xdmac_s *xdmac)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&xdmac->dsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam_givedsem(struct sam_xdmac_s *xdmac)
{
  (void)sem_post(&xdmac->dsem);
}

/****************************************************************************
 * Name: sam_getdmac
 *
 * Description:
 *  Read a global XDMAC register
 *
 ****************************************************************************/

static inline uint32_t sam_getdmac(struct sam_xdmac_s *xdmac,
                                   unsigned int offset)
{
  return getreg32(xdmac->base + offset);
}

/****************************************************************************
 * Name: sam_putdmac
 *
 * Description:
 *  Write a value to a global XDMAC register
 *
 ****************************************************************************/

static inline void sam_putdmac(struct sam_xdmac_s *xdmac, uint32_t value,
                               unsigned int offset)
{
  putreg32(value, xdmac->base + offset);
}

/****************************************************************************
 * Name: sam_getdmach
 *
 * Description:
 *  Read a XDMAC channel register
 *
 ****************************************************************************/

static inline uint32_t sam_getdmach(struct sam_xdmach_s *xdmach,
                                    unsigned int offset)
{
  return getreg32(xdmach->base + offset);
}

/****************************************************************************
 * Name: sam_putdmach
 *
 * Description:
 *  Write a value to a XDMAC channel register
 *
 ****************************************************************************/

static inline void sam_putdmach(struct sam_xdmach_s *xdmach, uint32_t value,
                                unsigned int offset)
{
  putreg32(value, xdmach->base + offset);
}

/****************************************************************************
 * Name: sam_controller
 *
 * Description:
 *    Given a DMA channel instance, return a pointer to the parent DMA
 *    controller instance.
 *
 ****************************************************************************/

static inline struct sam_xdmac_s *sam_controller(struct sam_xdmach_s *xdmach)
{
#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
  return xdmach->xdmac ? &g_xdmac1 : &g_xdmac0;
#elif defined(CONFIG_SAMA5_XDMAC0)
  return &g_xdmac0;
#else
  return &g_xdmac1;
#endif
}

/****************************************************************************
 * Name: sam_channel, sam_source_channel, and sam_sink_channel
 *
 * Description:
 *   Return the RX or TX channel associated with a PID.  As a clarification:
 *
 *   The source channel refers to the source of data for the DMA.  This is,
 *   either (1) memory for a TX DMA or (2) a peripheral register for an RX
 *   DMA.
 *
 *   The sink channel is the recipient of the DMA data.  This is either
 *   (1) memory for an RX DMA, or (2) a peripheral register for a TX DMA.
 *
 ****************************************************************************/

static uint8_t sam_channel(uint8_t pid, const struct sam_pidmap_s *table,
                           unsigned int nentries)
{
  int i;

  /* Search until either the entry with the matching PID is found or until
   * all of the table entries have been examined without finding the PID.
   */

  for (i = 0; i < nentries; i++, table++)
    {
      if (table->pid == pid)
        {
          return table->pchan;
        }
    }

  dmadbg("No channel found for pid %d\n", pid);
  DEBUGPANIC();
  return 0x3f;
}

static uint32_t sam_source_channel(struct sam_xdmach_s *xdmach, uint8_t pid,
                                   bool isperiph)
{
  const struct sam_pidmap_s *table;
  unsigned int nentries;

  if (!isperiph)
    {
      /* The source is memory, not a peripheral. */

      return 0x3f;
    }
  else

#ifdef CONFIG_SAMA5_XDMAC0
#ifdef CONFIG_SAMA5_XDMAC1
  if (xdmach->xdmac == 0)
#endif
    {
      /* Use the XDMAC0 lookup table */

      table = g_xdmac0_rxchan;
      nentries = NXDMAC0_RXCHANNELS;
    }
#endif

#ifdef CONFIG_SAMA5_XDMAC1
#ifdef CONFIG_SAMA5_XDMAC0
  else
#endif
    {
      /* Use the XDMAC1 lookup table */

      table = g_xdmac1_rxchan;
      nentries = NXDMAC1_RXCHANNELS;
    }
#endif

  return (uint32_t)sam_channel(pid, table, nentries);
}

static uint32_t sam_sink_channel(struct sam_xdmach_s *xdmach, uint8_t pid,
                                 bool isperiph)
{
  const struct sam_pidmap_s *table;
  unsigned int nentries;

  if (!isperiph)
    {
      /* The source is memory, not a peripheral. */

      return 0x3f;
    }
  else

#ifdef CONFIG_SAMA5_XDMAC0
#ifdef CONFIG_SAMA5_XDMAC1
  if (xdmach->xdmac == 0)
#endif
    {
      /* Use the XDMAC0 lookup table */

      table = g_xdmac0_txchan;
      nentries = NXDMAC0_TXCHANNELS;
    }
#endif

#ifdef CONFIG_SAMA5_XDMAC1
#ifdef CONFIG_SAMA5_XDMAC0
  else
#endif
    {
      /* Use the XDMAC1 lookup table */

      table = g_xdmac1_txchan;
      nentries = NXDMAC1_TXCHANNELS;
    }
#endif

  return (uint32_t)sam_channel(pid, table, nentries);
}

/****************************************************************************
 * Name: sam_txcfg
 *
 * Description:
 *  Decode the flags to get the correct CFG register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_txcfg(struct sam_xdmach_s *xdmach)
{
  uint32_t regval;
  unsigned int pid;
  unsigned int pchan;
  bool isperiph;

  /* Set transfer (memory to peripheral) DMA channel configuration register */

  regval   = XDMAC_CH_CFG_SOD;

  pid      = (xdmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT;
  isperiph = ((xdmach->flags & DMACH_FLAG_MEMISPERIPH) != 0);
  pchan    = sam_source_channel(xdmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << XDMAC_CH_CFG_SRCPER_SHIFT);
  regval  |= ((pchan & 0x30) << (XDMAC_CH_CFG_SRCPERMSB_SHIFT-4));
  regval  |= (xdmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? XDMAC_CH_CFG_SRCH2SEL : 0;

  pid      = (xdmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT;
  isperiph = ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0);
  pchan    = sam_sink_channel(xdmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << XDMAC_CH_CFG_DSTPER_SHIFT);
  regval  |= ((pchan & 0x30) << (XDMAC_CH_CFG_DSTPERMSB_SHIFT-4));
  regval  |= (xdmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? XDMAC_CH_CFG_DSTH2SEL : 0;

  return regval;
}

/****************************************************************************
 * Name: sam_rxcfg
 *
 * Description:
 *  Decode the flags to get the correct CFG register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_rxcfg(struct sam_xdmach_s *xdmach)
{
  uint32_t regval;
  unsigned int pid;
  unsigned int pchan;
  bool isperiph;

  /* Set received (peripheral to memory) DMA channel config */

  regval   = XDMAC_CH_CFG_SOD;

  pid      = (xdmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT;
  isperiph = ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0);
  pchan    = sam_source_channel(xdmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << XDMAC_CH_CFG_SRCPER_SHIFT);
  regval  |= ((pchan & 0x30) << (XDMAC_CH_CFG_SRCPERMSB_SHIFT-4));
  regval  |= (xdmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? XDMAC_CH_CFG_SRCH2SEL : 0;

  pid      = (xdmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT;
  isperiph = ((xdmach->flags & DMACH_FLAG_MEMISPERIPH) != 0);
  pchan    = sam_sink_channel(xdmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << XDMAC_CH_CFG_DSTPER_SHIFT);
  regval  |= ((pchan & 0x30) << (XDMAC_CH_CFG_DSTPERMSB_SHIFT-4));
  regval  |= (xdmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? XDMAC_CH_CFG_DSTH2SEL : 0;

  return regval;
}

/****************************************************************************
 * Name: sam_txctrlabits
 *
 * Description:
 *  Decode the flags to get the correct CTRLA register bit settings for
 *  a transmit (memory to peripheral) transfer.  These are only the "fixed"
 *  CTRLA values and  need to be updated with the actual transfer size before
 *  being written to CTRLA sam_txctrla).
 *
 ****************************************************************************/

static inline uint32_t sam_txctrlabits(struct sam_xdmach_s *xdmach)
{
  uint32_t regval;
  unsigned int ndx;
  unsigned int chunksize;

  DEBUGASSERT(xdmach);

  /* Since this is a transmit, the source is described by the memory selections.
   * Set the source width (memory width).
   */

  ndx = (xdmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 4);
  regval = g_chanwidth[ndx];

  /* Set the source chunk size (memory chunk size) */

  chunksize = (xdmach->flags & DMACH_FLAG_MEMCHUNKSIZE_MASK)
    >> DMACH_FLAG_MEMCHUNKSIZE_SHIFT;
  regval |= chunksize << XDMAC_CH_CTRLA_SCSIZE_SHIFT;

  /* Since this is a transmit, the destination is described by the peripheral selections.
   * Set the destination width (peripheral width).
   */

  ndx = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 4);
  regval |= g_chanwidth[ndx];

  /* Set the destination chunk size (peripheral chunk size) */

  chunksize = (xdmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE_MASK)
    >> DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT;
  regval |= chunksize << XDMAC_CH_CTRLA_DCSIZE_SHIFT;

  return regval;
}

/****************************************************************************
 * Name: sam_maxtxtransfer
 *
 * Description:
 *  Maximum number of bytes that can be sent in on transfer
 *
 ****************************************************************************/

static size_t sam_maxtxtransfer(struct sam_xdmach_s *xdmach)
{
  unsigned int srcwidth;
  size_t maxtransfer;

  /* Get the maximum transfer size in bytes.  BTSIZE is "the number of
   * transfers to be performed, that is, for writes it refers to the number
   * of source width transfers to perform when XDMAC is flow controller. For
   * Reads, BTSIZE refers to the number of transfers completed on the Source
   * Interface. ..."
   */

  srcwidth = (xdmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        maxtransfer = XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 1: /* 16 bits, 2 bytes */
        maxtransfer = 2 * XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 2: /* 32 bits 4 bytes */
        maxtransfer = 4 * XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 3: /* 64 bits, 8 bytes */
        maxtransfer = 8 * XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;
    }

  return maxtransfer;
}

/****************************************************************************
 * Name: sam_ntxtransfers
 *
 * Description:
 *   Number of TX transfers via DMA
 *
 ****************************************************************************/

static uint32_t sam_ntxtransfers(struct sam_xdmach_s *xdmach, uint32_t dmasize)
{
  unsigned int srcwidth;

  /* Adjust the source transfer size for the source chunk size (memory
   * chunk size).  BTSIZE is "the number of transfers to be performed, that
   * is, for writes it refers to the number of source width transfers
   * to perform when XDMAC is flow controller. For Reads, BTSIZE refers to
   * the number of transfers completed on the Source Interface. ..."
   */

  srcwidth = (xdmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        break;

      case 1: /* 16 bits, 2 bytes */
        dmasize = (dmasize + 1) >> 1;
        break;

      case 2: /* 32 bits, 4 bytes */
        dmasize = (dmasize + 3) >> 2;
        break;

      case 3: /* 64 bits, 8 bytes */
        dmasize = (dmasize + 7) >> 3;
        break;
    }

  return dmasize;
}

/****************************************************************************
 * Name: sam_txctrla
 *
 * Description:
 *  'OR' in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam_txctrla(struct sam_xdmach_s *xdmach,
                                   uint32_t ctrla, uint32_t dmasize)
{
  uint32_t ntransfers;

  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  ntransfers = sam_ntxtransfers(xdmach, dmasize);

  DEBUGASSERT(ntransfers <= XDMAC_CH_CTRLA_BTSIZE_MAX);
  return (ctrla & ~XDMAC_CH_CTRLA_BTSIZE_MASK) |
         (ntransfers << XDMAC_CH_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam_rxctrlabits
 *
 * Description:
 *  Decode the flags to get the correct CTRLA register bit settings for
 *  a read (peripheral to memory) transfer. These are only the "fixed" CTRLA
 *  values and need to be updated with the actual transfer size before being
 *  written to CTRLA sam_rxctrla).
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrlabits(struct sam_xdmach_s *xdmach)
{
  uint32_t regval;
  unsigned int ndx;
  unsigned int chunksize;

  DEBUGASSERT(xdmach);

  /* Since this is a receive, the source is described by the peripheral
   * selections. Set the source width (peripheral width).
   */

  ndx = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  DEBUGASSERT(ndx < 4);
  regval = g_chanwidth[ndx];

  /* Set the source chunk size (peripheral chunk size) */

  chunksize = (xdmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE_MASK)
    >> DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT;
  regval |= chunksize << XDMAC_CH_CTRLA_SCSIZE_SHIFT;

  /* Since this is a receive, the destination is described by the memory
   * selections. Set the destination width (memory width).
   */

  ndx = (xdmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  DEBUGASSERT(ndx < 4);
  regval |= g_chanwidth[ndx];

  /* Set the destination chunk size (memory chunk size) */

  chunksize = (xdmach->flags & DMACH_FLAG_MEMCHUNKSIZE_MASK)
    >> DMACH_FLAG_MEMCHUNKSIZE_SHIFT;
  regval |= chunksize << XDMAC_CH_CTRLA_DCSIZE_SHIFT;

  return regval;
}

/****************************************************************************
 * Name: sam_maxrxtransfer
 *
 * Description:
 *  Maximum number of bytes that can be sent in on transfer
 *
 ****************************************************************************/

static size_t sam_maxrxtransfer(struct sam_xdmach_s *xdmach)
{
  unsigned int srcwidth;
  size_t maxtransfer;

  /* Get the maximum transfer size in bytes.  BTSIZE is "the number of
   * transfers to be performed, that is, for writes it refers to the number
   * of source width transfers to perform when XDMAC is flow controller. For
   * Reads, BTSIZE refers to the number of transfers completed on the Source
   * Interface. ..."
   */

  srcwidth = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        maxtransfer = XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 1: /* 16 bits, 2 bytes */
        maxtransfer = 2 * XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 2: /* 32 bits, 4 bytes */
        maxtransfer = 4 * XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 3: /* 64 bits, 8 bytes */
        maxtransfer = 8 * XDMAC_CH_CTRLA_BTSIZE_MAX;
        break;
    }

  return maxtransfer;
}

/****************************************************************************
 * Name: sam_nrxtransfers
 *
 * Description:
 *  Number of RX transfers via DMA
 *
 ****************************************************************************/

static uint32_t sam_nrxtransfers(struct sam_xdmach_s *xdmach, uint32_t dmasize)
{
  unsigned int srcwidth;

  /* Adjust the source transfer size for the source chunk size (peripheral
   * chunk size).  BTSIZE is "the number of transfers to be performed, that
   * is, for writes it refers to the number of source width transfers
   * to perform when XDMAC is flow controller. For Reads, BTSIZE refers to
   * the number of transfers completed on the Source Interface. ..."
   */

  srcwidth = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        break;

      case 1: /* 16 bits, 2 bytes */
        dmasize = (dmasize + 1) >> 1;
        break;

      case 2: /* 32 bits, 4 bytes */
        dmasize = (dmasize + 3) >> 2;
        break;

      case 3: /* 64 bits, 8 bytes */
        dmasize = (dmasize + 7) >> 3;
        break;
    }

  return dmasize;
}

/****************************************************************************
 * Name: sam_rxctrla
 *
 * Description:
 *  'OR' in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrla(struct sam_xdmach_s *xdmach,
                                   uint32_t ctrla, uint32_t dmasize)
{
  uint32_t ntransfers;

  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  ntransfers = sam_nrxtransfers(xdmach, dmasize);

  DEBUGASSERT(ntransfers <= XDMAC_CH_CTRLA_BTSIZE_MAX);
  return (ctrla & ~XDMAC_CH_CTRLA_BTSIZE_MASK) |
         (ntransfers << XDMAC_CH_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam_txctrlb
 *
 * Description:
 *  Decode the flags to get the correct CTRLB register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_txctrlb(struct sam_xdmach_s *xdmach)
{
  uint32_t regval;
  unsigned int ahbif;

  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = XDMAC_CH_CTRLB_BOTHDSCR | XDMAC_CH_CTRLB_IEN;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from TX is memory to peripheral, but that is really
   * be determined by bits in the DMA flags.
   */

  /* Is the memory source really a peripheral? */

  if ((xdmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
    {
      /* Yes.. is the peripheral destination also a peripheral? */

      if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= XDMAC_CH_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= XDMAC_CH_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the source is memory.  Is the peripheral destination a
       * peripheral
       */

      if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= XDMAC_CH_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= XDMAC_CH_CTRLB_FC_M2M;
        }
    }

  /* Source ABH layer */

  ahbif = (xdmach->flags & DMACH_FLAG_MEMAHB_MASK) >> DMACH_FLAG_MEMAHB_SHIFT;
  regval |= (ahbif << XDMAC_CH_CTRLB_SIF_SHIFT);

  /* Select source address incrementing */

  if ((xdmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= XDMAC_CH_CTRLB_SRCINCR_FIXED;
    }

  /* Destination ABH layer */

  ahbif = (xdmach->flags & DMACH_FLAG_PERIPHAHB_MASK) >> DMACH_FLAG_PERIPHAHB_SHIFT;
  regval |= (ahbif << XDMAC_CH_CTRLB_DIF_SHIFT);

  /* Select destination address incrementing */

  if ((xdmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= XDMAC_CH_CTRLB_DSTINCR_FIXED;
    }

  return regval;
}

/****************************************************************************
 * Name: sam_rxctrlb
 *
 * Description:
 *  Decode the flags to get the correct CTRLB register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrlb(struct sam_xdmach_s *xdmach)
{
  uint32_t regval;
  unsigned int ahbif;

  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = XDMAC_CH_CTRLB_BOTHDSCR | XDMAC_CH_CTRLB_IEN;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from RX is peripheral to memory, but that is really
   * be determined by bits in the DMA flags.
   */

  /* Is the peripheral source really a peripheral? */

  if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Yes.. is the memory destination also a peripheral? */

      if ((xdmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= XDMAC_CH_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= XDMAC_CH_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the peripheral source is memory.  Is the memory destination
       * a peripheral
       */

      if ((xdmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= XDMAC_CH_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= XDMAC_CH_CTRLB_FC_M2M;
        }
    }

  /* Source ABH layer */

  ahbif = (xdmach->flags & DMACH_FLAG_PERIPHAHB_MASK) >> DMACH_FLAG_PERIPHAHB_SHIFT;
  regval |= (ahbif << XDMAC_CH_CTRLB_SIF_SHIFT);

  /* Select source address incrementing */

  if ((xdmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= XDMAC_CH_CTRLB_SRCINCR_FIXED;
    }

  /* Destination ABH layer */

  ahbif = (xdmach->flags & DMACH_FLAG_MEMAHB_MASK) >> DMACH_FLAG_MEMAHB_SHIFT;
  regval |= (ahbif << XDMAC_CH_CTRLB_DIF_SHIFT);

  /* Select address incrementing */

  if ((xdmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= XDMAC_CH_CTRLB_DSTINCR_FIXED;
    }

  return regval;
}

/****************************************************************************
 * Name: sam_allocdesc
 *
 * Description:
 *  Allocate and add one descriptor to the DMA channel's link list.
 *
 *  NOTE: link list entries are freed by the DMA interrupt handler.
 *  However, since the setting/clearing of the 'in use' indication is
 *  atomic, no special actions need be performed.  It would be a good thing
 *  to add logic to handle the case where all of the entries are exhausted
 *  and we could wait for some to be freed by the interrupt handler.
 *
 ****************************************************************************/

static struct dma_linklist_s *
sam_allocdesc(struct sam_xdmach_s *xdmach, struct dma_linklist_s *prev,
              uint32_t csa, uint32_t cda, uint32_t ctrla, uint32_t ctrlb)
{
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);
  struct dma_linklist_s *desc = NULL;
  int i;

  /* Sanity check -- csa == 0 is the indication that the link is unused.
   * Obviously setting it to zero would break that usage.
   */

#ifdef CONFIG_DEBUG
  if (csa != 0)
#endif
    {
      /* Table a descriptor table semaphore count.  When we get one, then there
       * is at least one free descriptor in the table and it is ours.
       */

      sam_takedsem(xdmac);

      /* Examine each link list entry to find an available one -- i.e., one
       * with csa == 0.  That csa field is set to zero by the DMA transfer
       * complete interrupt handler.  The following should be safe because
       * that is an atomic operation.
       */

      sam_takechsem(xdmac);
      for (i = 0; i < CONFIG_SAMA5_NLLDESC; i++)
        {
          if (xdmac->desc[i].csa == 0)
            {
              /* We have it.  Initialize the new link list entry */

              desc        = &xdmac->desc[i];
              desc->csa   = csa;  /* Source address */
              desc->cda   = cda;  /* Destination address */
              desc->ctrla = ctrla;  /* Control A value */
              desc->ctrlb = ctrlb;  /* Control B value */
              desc->dscr  = 0;      /* Next descriptor address */

              /* And then hook it at the tail of the link list */

              if (!prev)
                {
                  /* There is no previous link.  This is the new head of
                   * the list
                   */

                  DEBUGASSERT(xdmach->llhead == NULL && xdmach->lltail == NULL);
                  xdmach->llhead = desc;
                }
              else
                {
                  DEBUGASSERT(xdmach->llhead != NULL && xdmach->lltail == prev);

                  /* When the second link is added to the list, that is the
                   * cue that we are going to do the link list transfer.
                   *
                   * Enable the source and destination descriptor in the link
                   * list entry just before this one.  We assume that both
                   * source and destination buffers are non-continuous, but
                   * this should work even if that is not the case.
                   */

                  prev->ctrlb &= ~XDMAC_CH_CTRLB_BOTHDSCR;

                  /* Link the previous tail to the new tail.
                   * REVISIT:  This assumes that the next description is fetched
                   * via AHB IF0.
                   */

                  prev->dscr = (uint32_t)desc;
                }

              /* In any event, this is the new tail of the list.  The source
               * and destination descriptors must be disabled for the last entry
               * in the link list. */

              desc->ctrlb  |= XDMAC_CH_CTRLB_BOTHDSCR;
              xdmach->lltail = desc;

              /* Assume that we will be doing multple buffer transfers and that
               * that hardware will be accessing the descriptor via DMA.
               */

              cp15_clean_dcache((uintptr_t)desc,
                                (uintptr_t)desc + sizeof(struct dma_linklist_s));
              break;
            }
        }

      /* Because we hold a count from the counting semaphore, the above
       * search loop should always be successful.
       */

      sam_givechsem(xdmac);
      DEBUGASSERT(desc != NULL);
    }

  return desc;
}

/****************************************************************************
 * Name: sam_freelinklist
 *
 * Description:
 *  Free all descriptors in the DMA channel's link list.
 *
 *  NOTE: Called from the DMA interrupt handler.
 *
 ****************************************************************************/

static void sam_freelinklist(struct sam_xdmach_s *xdmach)
{
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);
  struct dma_linklist_s *desc;
  uintptr_t paddr;

  /* Get the head of the link list and detach the link list from the DMA
   * channel
   */

  desc           = xdmach->llhead;
  xdmach->llhead = NULL;
  xdmach->lltail = NULL;

  while (desc != NULL)
    {
      /* Valid, in-use descriptors never have csa == 0 */

      DEBUGASSERT(desc->csa != 0);

      /* Get the physical address of the next descriptor in the list */

      paddr = desc->dscr;

      /* Free the descriptor by simply nullifying it and bumping up the
       * semaphore count.
       */

      memset(desc, 0, sizeof(struct dma_linklist_s));
      sam_givedsem(xdmac);

      /* Get the virtual address of the next descriptor in the list */

      desc = (struct dma_linklist_s *)sam_virtramaddr(paddr);
    }
}

/****************************************************************************
 * Name: sam_txbuffer
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam_txbuffer(struct sam_xdmach_s *xdmach, uint32_t paddr,
                        uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (xdmach->llhead)
    {
      regval = xdmach->llhead->ctrla;
      ctrlb  = xdmach->llhead->ctrlb;
    }
  else
    {
      regval = sam_txctrlabits(xdmach);
      ctrlb  = sam_txctrlb(xdmach);
    }

  ctrla = sam_txctrla(xdmach, regval, nbytes);

  /* Add the new link list entry */

  if (!sam_allocdesc(xdmach, xdmach->lltail, maddr, paddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the transmit CFG register setting (it won't be used until
   * the DMA is started).
   */

  xdmach->cfg = sam_txcfg(xdmach);
  return OK;
}

/****************************************************************************
 * Name: sam_rxbuffer
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam_rxbuffer(struct sam_xdmach_s *xdmach, uint32_t paddr,
                        uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (xdmach->llhead)
    {
      regval = xdmach->llhead->ctrla;
      ctrlb  = xdmach->llhead->ctrlb;
    }
  else
    {
      regval = sam_rxctrlabits(xdmach);
      ctrlb  = sam_rxctrlb(xdmach);
    }

   ctrla = sam_rxctrla(xdmach, regval, nbytes);

  /* Add the new link list entry */

  if (!sam_allocdesc(xdmach, xdmach->lltail, paddr, maddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the receive CFG register setting (it won't be used until
   * the DMA is started).
   */

  xdmach->cfg = sam_rxcfg(xdmach);
  return OK;
}

/****************************************************************************
 * Name: sam_single
 *
 * Description:
 *   Start a single buffer DMA.
 *
 ****************************************************************************/

static inline int sam_single(struct sam_xdmach_s *xdmach)
{
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);
  struct dma_linklist_s *llhead = xdmach->llhead;

  /* Clear any pending interrupts from any previous XDMAC transfer by
   * reading the XDMAC Channel Interrupt Status Register (CIS).
   */

  (void)sam_getdmach(xdmach, SAM_XDMACH_CIS_OFFSET);

  /* Write the starting source address in the Channel Source Address (CSA)
   * Register.
   */

  DEBUGASSERT(llhead != NULL && llhead->csa != 0);
  sam_putdmach(xdmach, llhead->csa, SAM_XDMACH_CSA_OFFSET);

  /* Write the starting destination address in the  Channel Destination
   * Address Register (CDA).
   */

  sam_putdmach(xdmach, llhead->cda, SAM_XDMACH_CDA_OFFSET);

  /* Clear the next descriptor address */

  sam_putdmach(xdmach, 0, SAM_XDMAC_CH_DSCR_OFFSET);

  /* Set up the CTRLA register */

  sam_putdmach(xdmach, llhead->ctrla, SAM_XDMAC_CH_CTRLA_OFFSET);

  /* Set up the CTRLB register */

  sam_putdmach(xdmach, llhead->ctrlb, SAM_XDMAC_CH_CTRLB_OFFSET);

  /* Both the DST and SRC DSCR bits should be '1' in CTRLB */

  DEBUGASSERT((llhead->ctrlb & XDMAC_CH_CTRLB_BOTHDSCR) ==
              XDMAC_CH_CTRLB_BOTHDSCR);

  /* Set up the CFG register */

  sam_putdmach(xdmach, xdmach->cfg, SAM_XDMAC_CH_CFG_OFFSET);

  /* Enable the channel by writing a ‘1’ to the CHER enable bit */

  sam_putdmac(xdmac, XDMAC_CHER_ENA(xdmach->chan), SAM_XDMAC_CHER_OFFSET);

  /* The DMA has been started. Once the transfer completes, hardware sets
   * the interrupts and disables the channel.  We will receive buffer
   * complete and transfer complete interrupts.
   *
   * Enable error, buffer complete and transfer complete interrupts.
   * (Since there is only a single buffer, we don't need the buffer
   * complete interrupt).
   */

  sam_putdmac(xdmac, XDMAC_EBC_CBTCINTS(xdmach->chan), SAM_XDMAC_EBCIER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_multiple
 *
 * Description:
 *   Start a multiple buffer DMA.
 *
 ****************************************************************************/

static inline int sam_multiple(struct sam_xdmach_s *xdmach)
{
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);
  struct dma_linklist_s *llhead = xdmach->llhead;

  DEBUGASSERT(llhead != NULL && llhead->csa != 0);

  /* Check the first and last CTRLB values */

  DEBUGASSERT((llhead->ctrlb & XDMAC_CH_CTRLB_BOTHDSCR) == 0);
  DEBUGASSERT((xdmach->lltail->ctrlb & XDMAC_CH_CTRLB_BOTHDSCR) ==
              XDMAC_CH_CTRLB_BOTHDSCR);

  /* Clear any pending interrupts from any previous XDMAC transfer by
   * reading the XDMAC Channel Interrupt Status Register (CIS).
   */

  (void)sam_getdmach(xdmach, SAM_XDMACH_CIS_OFFSET);

  /* Set up the initial CTRLA register */

  sam_putdmach(xdmach, llhead->ctrla, SAM_XDMAC_CH_CTRLA_OFFSET);

  /* Set up the CTRLB register (will enable descriptors) */

  sam_putdmach(xdmach, llhead->ctrlb, SAM_XDMAC_CH_CTRLB_OFFSET);

  /* Write the channel configuration information into the CFG register */

  sam_putdmach(xdmach, xdmach->cfg, SAM_XDMAC_CH_CFG_OFFSET);

  /* Program the DSCR register with the pointer to the firstlink list entry. */

  sam_putdmach(xdmach, (uint32_t)llhead, SAM_XDMAC_CH_DSCR_OFFSET);

  /* Finally, enable the channel by writing a ‘1’ to the CHER enable */

  sam_putdmac(xdmac, XDMAC_CHER_ENA(xdmach->chan), SAM_XDMAC_CHER_OFFSET);

  /* As each buffer of data is transferred, the CTRLA register is written
   * back into the link list entry.  The CTRLA contains updated BTSIZE and
   * DONE bits.  Additionally, the CTRLA DONE bit is asserted when the
   * buffer transfer has completed.
   *
   * The XDMAC transfer continues until the CTRLB register disables the
   * descriptor (DSCR bits) registers at the final buffer transfer.
   *
   * Enable error, buffer complete and transfer complete interrupts.  We
   * don't really need the buffer complete interrupts, but we will take them
   * just to handle stall conditions.
   */

  sam_putdmac(xdmac, XDMAC_EBC_CHANINTS(xdmach->chan), SAM_XDMAC_EBCIER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void sam_dmaterminate(struct sam_xdmach_s *xdmach, int result)
{
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);

  /* Disable all channel interrupts */

  sam_putdmac(xdmac, XDMAC_EBC_CHANINTS(xdmach->chan), SAM_XDMAC_EBCIDR_OFFSET);

  /* Disable the channel by writing one to the write-only channel disable
   * register.
   */

  sam_putdmac(xdmac, XDMAC_CHDR_DIS(xdmach->chan), SAM_XDMAC_CHDR_OFFSET);

  /* Free the linklist */

  sam_freelinklist(xdmach);

  /* If this was an RX DMA (peripheral-to-memory), then invalidate the cache
   * to force reloads from memory.
   */

  if (xdmach->rx)
    {
      cp15_invalidate_dcache(xdmach->rxaddr, xdmach->rxaddr + xdmach->rxsize);
    }

  /* Perform the DMA complete callback */

  if (xdmach->callback)
    {
      xdmach->callback((DMA_HANDLE)xdmach, xdmach->arg, result);
    }

  xdmach->callback = NULL;
  xdmach->arg      = NULL;
}

/****************************************************************************
 * Name: sam_xdmac_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int sam_xdmac_interrupt(struct sam_xdmac_s *xdmac)
{
  struct sam_xdmach_s *xdmach;
  unsigned int chndx;
  uint32_t regval;

  /* Get the XDMAC status register value.  Ignore all masked interrupt
   * status bits.
   */

  regval = sam_getdmac(xdmac, SAM_XDMAC_EBCISR_OFFSET) &
           sam_getdmac(xdmac, SAM_XDMAC_EBCIMR_OFFSET);

  /* Check if the any transfer has completed or any errors have ocurred. */

  if (regval & XDMAC_EBC_ALLCHANINTS)
    {
      /* Yes.. Check each bit  to see which channel has interrupted */

      for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
        {
          /* Are any interrupts pending for this channel? */

          if ((regval & XDMAC_EBC_CHANINTS(chndx)) != 0)
            {
              xdmach = &xdmac->xdmach[chndx];

              /* Yes.. Did an error occur? */

              if ((regval & XDMAC_EBC_ERR(chndx)) != 0)
                {
                   /* Yes... Terminate the transfer with an error? */

                   dmalldbg("ERROR: DMA failed: %08x\n", regval);
                   sam_dmaterminate(xdmach, -EIO);
                }

              /* Is the transfer complete? */

              else if ((regval & XDMAC_EBC_CBTC(chndx)) != 0)
               {
                  /* Yes.. Terminate the transfer with success */

                  sam_dmaterminate(xdmach, OK);
                }

              /* Otherwise, this must be a Bufffer Transfer Complete (BTC)
               * interrupt as part of a multiple buffer transfer.
               */

              else /* if ((regval & XDMAC_EBC_BTC(chndx)) != 0) */
                {
                  /* Write the KEEPON field to clear the STALL states */

                  sam_putdmac(xdmac, XDMAC_CHER_KEEP(xdmach->chan),
                              SAM_XDMAC_CHER_OFFSET);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_xdmac0_interrupt and sam_xdmac1_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_XDMAC0
static int sam_xdmac0_interrupt(int irq, void *context)
{
  return sam_xdmac_interrupt(&g_xdmac0);
}
#endif

#ifdef CONFIG_SAMA5_XDMAC1
static int sam_xdmac1_interrupt(int irq, void *context)
{
  return sam_xdmac_interrupt(&g_xdmac1);
}
#endif

/****************************************************************************
 * Name: sam_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmainitialize(struct sam_xdmac_s *xdmac)
{
  /* Disable all DMA interrupts */

  sam_putdmac(xdmac, XDMAC_EBC_ALLINTS, SAM_XDMAC_EBCIDR_OFFSET);

  /* Disable all DMA channels */

  sam_putdmac(xdmac, XDMAC_CHDR_DIS_ALL, SAM_XDMAC_CHDR_OFFSET);

  /* Enable the DMA controller */

  sam_putdmac(xdmac,XDMAC_EN_ENABLE, SAM_XDMAC_EN_OFFSET);

  /* Initialize semaphores */

  sem_init(&xdmac->chsem, 0, 1);
  sem_init(&xdmac->dsem, 0, SAM_NDMACHAN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
#ifdef CONFIG_SAMA5_XDMAC0
  dmallvdbg("Initialize XDMAC0\n");

  /* Enable peripheral clock */

  sam_xdmac0_enableclk();

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_XDMAC0, sam_xdmac0_interrupt);

  /* Initialize the controller */

  sam_dmainitialize(&g_xdmac0);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_XDMAC0);
#endif

#ifdef CONFIG_SAMA5_XDMAC1
  dmallvdbg("Initialize XDMAC1\n");

  /* Enable peripheral clock */

  sam_xdmac1_enableclk();

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_XDMAC1, sam_xdmac1_interrupt);

  /* Initialize the controller */

  sam_dmainitialize(&g_xdmac1);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_XDMAC1);
#endif
}

/****************************************************************************
 * Name: sam_xdmachannel
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel then
 *   gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  Howerver, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMA_HANDLE sam_xdmachannel(uint8_t dmacno, uint32_t chflags)
{
  struct sam_xdmac_s *xdmac;
  struct sam_xdmach_s *xdmach;
  unsigned int chndx;

  /* Pick the DMA controller */

#ifdef CONFIG_SAMA5_XDMAC0
  if (dmacno == 0)
    {
      xdmac = &g_xdmac0;
    }
  else
#endif

#ifdef CONFIG_SAMA5_XDMAC1
  if (dmacno == 1)
    {
      xdmac = &g_xdmac1;
    }
  else
#endif

    {
      dmadbg("ERROR: Bad XDMAC number: %d\n", dmacno);
      DEBUGPANIC();
      return (DMA_HANDLE)NULL;
    }

  /* Search for an available DMA channel with at least the requested FIFO
   * size.
   */

  xdmach = NULL;
  sam_takechsem(xdmac);
  for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
    {
      struct sam_xdmach_s *candidate = &xdmac->xdmach[chndx];
      if (!candidate->inuse)
        {
          xdmach        = candidate;
          xdmach->inuse = true;

          /* Clear the pending Interrupt Status bits by reading the XDMAC
           * Channel Interrupt Status (CIS) Register
           */

          (void)sam_getdmach(xdmach, SAM_XDMACH_CIS_OFFSET);

          /* Disable the channel by writing one to the write-only Global
           * Channel Disable (GD) Register
           */

          sam_putdmac(xdmac, XDMAC_CHAN(chndx), SAM_XDMAC_GD_OFFSET);

          /* Set the DMA channel flags. */

          xdmach->flags = chflags;
          break;
        }
    }

  sam_givechsem(xdmac);

  /* Show the result of the allocation */

  if (xdmach)
    {
      dmavdbg("XDMAC%d CH%d: chflags: %08x returning xdmach: %p\n",
              (int)dmacno, xdmach->chan, (int)chflags, xdmach);
    }
  else
    {
      dmadbg("ERROR: Failed allocate XDMAC%d channel\n", (int)dmacno);
    }

  return (DMA_HANDLE)xdmach;
}

/************************************************************************************
 * Name: sam_xdmaconfig
 *
 * Description:
 *   There are two channel usage models:  (1) The channel is allocated and configured
 *   in one step.  This is the typical case where a DMA channel performs a constant
 *   role.  The alternative is (2) where the DMA channel is reconfigured on the fly.
 *   In this case, the chflags provided to sam_xdmachannel are not used and
 *   sam_xdmaconfig() is called before each DMA to configure the DMA channel
 *   appropriately.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void sam_xdmaconfig(DMA_HANDLE handle, uint32_t chflags)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;

  /* Set the new DMA channel flags. */

  xdmach->flags = chflags;

#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
  dmavdbg("XDMAC%d CH%d: chflags: %08x\n",
          xdmach->xdmac, xdmach->chan, (int)chflags);
#elif defined(CONFIG_SAMA5_XDMAC0)
  dmavdbg("XDMAC0 CH%d: chflags: %08x\n",
          xdmach->chan, (int)chflags);
#else
  dmavdbg("XDMAC1 CH%d: chflags: %08x\n",
          xdmach->chan, (int)chflags);
#endif
}

/****************************************************************************
 * Name: sam_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until sam_xdmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmafree(DMA_HANDLE handle)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;

  dmavdbg("xdmach: %p\n", xdmach);
  DEBUGASSERT((xdmach != NULL) && (xdmach->inuse));

  /* Make sure that the channel is disabled by writing one to the write-only
   * Global Channel Disable (GD) Register
   */

  sam_putdmac(xdmac, XDMAC_CHAN(xdmach->chan), SAM_XDMAC_GD_OFFSET);

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  xdmach->flags = 0;
  xdmach->inuse = false;                   /* No longer in use */
}

/****************************************************************************
 * Name: sam_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  size_t maxtransfer;
  size_t remaining;
  int ret = OK;

  dmavdbg("xdmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          xdmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(xdmach);
  dmavdbg("llhead: %p lltail: %p\n", xdmach->llhead, xdmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtxtransfer(xdmach);
  remaining   = nbytes;

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_txbuffer(xdmach, paddr, maddr, maxtransfer);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do so).
           */

          if ((xdmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((xdmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_txbuffer(xdmach, paddr, maddr, remaining);
    }

  /* Save an indication so that the DMA interrupt completion logic will know
   * that this was not an RX transfer.
   */

  xdmach->rx = false;

  /* Clean caches associated with the DMA memory */

  cp15_clean_dcache(maddr, maddr + nbytes);
  return ret;
}

/****************************************************************************
 * Name: sam_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  size_t maxtransfer;
  size_t remaining;
  int ret = OK;

  dmavdbg("xdmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          xdmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(xdmach);
  dmavdbg("llhead: %p lltail: %p\n", xdmach->llhead, xdmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxrxtransfer(xdmach);
  remaining   = nbytes;

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_rxbuffer(xdmach, paddr, maddr, maxtransfer);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do so).
           */

          if ((xdmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((xdmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_rxbuffer(xdmach, paddr, maddr, remaining);
    }

  /* Save an indication so that the DMA interrupt completion logic will know
   * that this was an RX transfer and will invalidate the cache.
   */

  xdmach->rx     = true;
  xdmach->rxaddr = maddr;
  xdmach->rxsize = (xdmach->flags & DMACH_FLAG_MEMINCREMENT) != 0 ? nbytes : sizeof(uint32_t);

  /* Clean caches associated with the DMA memory */

  cp15_clean_dcache(maddr, maddr + nbytes);
  return ret;
}

/****************************************************************************
 * Name: sam_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int sam_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  int ret = -EINVAL;

  dmavdbg("xdmach: %p callback: %p arg: %p\n", xdmach, callback, arg);
  DEBUGASSERT(xdmach != NULL);

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * link list).
   */

  if (xdmach->llhead)
    {
      /* Save the callback info.  This will be invoked whent the DMA commpletes */

      xdmach->callback = callback;
      xdmach->arg      = arg;

      /* Is this a single block transfer?  Or a multiple block tranfer? */

      if (xdmach->llhead == xdmach->lltail)
        {
          ret = sam_single(xdmach);
        }
      else
        {
          ret = sam_multiple(xdmach);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam_dmastop() is called, the DMA channel is
 *   reset and sam_dmarx/txsetup() must be called before sam_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void sam_dmastop(DMA_HANDLE handle)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  irqstate_t flags;

  dmavdbg("xdmach: %p\n", xdmach);
  DEBUGASSERT(xdmach != NULL);

  flags = irqsave();
  sam_dmaterminate(xdmach, -EINTR);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_xdmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam_dmasample(DMA_HANDLE handle, struct sam_dmaregs_s *regs)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);
  irqstate_t flags;

  /* Sample global registers.  NOTE: reading GIS clears interrupts, but
   * that should be okay IF interrupts are enabled when this function is
   * called.  But there is a race condition where this instrumentation could
   * cause lost interrupts.
   */

  flags        = irqsave();

  regs->gtype  = sam_getdmac(xdmac, SAM_XDMAC_GTYPE_OFFSET);
  regs->gcfg   = sam_getdmac(xdmac, SAM_XDMAC_GCFG_OFFSET);
  regs->gwac   = sam_getdmac(xdmac, SAM_XDMAC_GWAC_OFFSET);
  regs->gim    = sam_getdmac(xdmac, SAM_XDMAC_GIM_OFFSET);
  regs->gis    = sam_getdmac(xdmac, SAM_XDMAC_GIS_OFFSET);
  regs->gs     = sam_getdmac(xdmac, SAM_XDMAC_GS_OFFSET);
  regs->grs    = sam_getdmac(xdmac, SAM_XDMAC_GRS_OFFSET);
  regs->gws    = sam_getdmac(xdmac, SAM_XDMAC_GWS_OFFSET);
  regs->gsws   = sam_getdmac(xdmac, SAM_XDMAC_GSWS_OFFSET);

  /* Sample channel registers */

  regs->cis    = sam_getdmach(xdmach, SAM_XDMACH_CIS_OFFSET);
  regs->csa    = sam_getdmach(xdmach, SAM_XDMACH_CSA_OFFSET);
  regs->cda    = sam_getdmach(xdmach, SAM_XDMACH_CDA_OFFSET);
  regs->cnda   = sam_getdmach(xdmach, SAM_XDMACH_CNDA_OFFSET);
  regs->cndc   = sam_getdmach(xdmach, SAM_XDMACH_CNDC_OFFSET);
  regs->cubc   = sam_getdmach(xdmach, SAM_XDMACH_CUBC_OFFSET);
  regs->cbc    = sam_getdmach(xdmach, SAM_XDMACH_CBC_OFFSET);
  regs->cc     = sam_getdmach(xdmach, SAM_XDMACH_CC_OFFSET);
  regs->cdsmsp = sam_getdmach(xdmach, SAM_XDMACH_CDSMSP_OFFSET);
  regs->csus   = sam_getdmach(xdmach, SAM_XDMACH_CSUS_OFFSET);
  regs->cdus   = sam_getdmach(xdmach, SAM_XDMACH_CDUS_OFFSET);

  irqrestore(flags);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: sam_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_xdmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam_dmadump(DMA_HANDLE handle, const struct sam_dmaregs_s *regs,
                 const char *msg)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);

  dmadbg("%s\n", msg);
  dmadbg("  DMA Global Registers:\n");
  dmadbg("     GTYPE[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GTYPE_OFFSET, regs->gtype);
  dmadbg("      GCFG[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GCFG_OFFSET, regs->gcfg);
  dmadbg("      GWAC[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GWAC_OFFSET, regs->gwac);
  dmadbg("       GIM[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GIM_OFFSET, regs->gim);
  dmadbg("       GIS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GIS_OFFSET, regs->gis);
  dmadbg("        GS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GS_OFFSET, regs->gs);
  dmadbg("       GRS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GRS_OFFSET, regs->grs);
  dmadbg("       GWS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GWS_OFFSET, regs->gws);
  dmadbg("      GSWS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GSWS_OFFSET, regs->gsws);
  dmadbg("  DMA Channel Registers:\n");
  dmadbg("       CIS[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CIS_OFFSET, regs->cis);
  dmadbg("       CSA[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CSA_OFFSET, regs->csa);
  dmadbg("       CDA[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CDA_OFFSET, regs->cda);
  dmadbg("      CNDA[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CNDA_OFFSET, regs->cnda);
  dmadbg("      CNDC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CNDC_OFFSET, regs->cndc);
  dmadbg("      CUBC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CUBC_OFFSET, regs->cubc);
  dmadbg("       CBC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CBC_OFFSET, regs->cbc);
  dmadbg("        CC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CC_OFFSET, regs->cc);
  dmadbg("    CDSMSP[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CDSMSP_OFFSET, regs->cdsmsp);
  dmadbg("      CSUS[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CSUS_OFFSET, regs->csus);
  dmadbg("      CDUS[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CDUS_OFFSET, regs->cdus);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_SAMA5_XDMAC0 || CONFIG_SAMA5_XDMAC1 */
