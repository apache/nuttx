/****************************************************************************
 * arch/arm/src/sama5/sam_xdmac.c
 *
 *   Copyright (C) 2014, 2016-2017 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"
#include "cache.h"
#include "up_internal.h"
#include "sched/sched.h"

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

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure maps a peripheral ID to an DMA channel */

struct sam_pidmap_s
{
  uint8_t pid;                    /* Peripheral identifier */
  uint8_t pchan;                  /* DMA channel */
};

/* This structure describes one DMA channel */

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
  uint32_t cc;                    /* Pre-calculated CC register for transfer */
  dma_callback_t callback;        /* Callback invoked when the DMA completes */
  void *arg;                      /* Argument passed to callback function */
  uint32_t rxaddr;                /* RX memory address to be invalidated */
  size_t rxsize;                  /* Size of RX memory region */
  struct chnext_view1_s *llhead;  /* DMA link list head */
  struct chnext_view1_s *lltail;  /* DMA link list head */
};

/* This structure describes the state of one DMA controller */

struct sam_xdmac_s
{
  /* These semaphores protect the DMA channel and descriptor tables */

  sem_t chsem;                       /* Protects channel table */
  sem_t dsem;                        /* Protects descriptor table */
  uint32_t base;                     /* DMA register channel base address */

  /* This array describes the available link list descriptors */

  struct chnext_view1_s *descr;

  /* This array describes each DMA channel */

  struct sam_xdmach_s *xdmach;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

struct chnext_view1_s g_desc0[CONFIG_SAMA5_NLLDESC];

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

  .descr       = g_desc0,

  /* This array describes each DMA channel */

  .xdmach     = g_xdmach0,
};

#endif /* CONFIG_SAMA5_XDMAC0 */

/* This array describes the state of DMA controller 1 */

#ifdef CONFIG_SAMA5_XDMAC1
/* This array describes the available link list descriptors */

struct chnext_view1_s g_desc1[CONFIG_SAMA5_NLLDESC];

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

  .descr       = g_desc1,

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
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&xdmac->chsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

static inline void sam_givechsem(struct sam_xdmac_s *xdmac)
{
  (void)nxsem_post(&xdmac->chsem);
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
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&xdmac->dsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

static inline void sam_givedsem(struct sam_xdmac_s *xdmac)
{
  (void)nxsem_post(&xdmac->dsem);
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

  dmaerr("ERROR: No channel found for pid %d\n", pid);
  DEBUGPANIC();
  return 0x3f;
}

static uint32_t sam_source_channel(struct sam_xdmach_s *xdmach, uint8_t pid)
{
  const struct sam_pidmap_s *table;
  unsigned int nentries;

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

static uint32_t sam_sink_channel(struct sam_xdmach_s *xdmach, uint8_t pid)
{
  const struct sam_pidmap_s *table;
  unsigned int nentries;

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
 * Name: sam_maxtransfer
 *
 * Description:
 *  Maximum number of bytes that can be sent in one transfer
 *
 ****************************************************************************/

static size_t sam_maxtransfer(struct sam_xdmach_s *xdmach)
{
  unsigned int xfrwidth;
  size_t maxtransfer;

  /* Get the maximum transfer size in bytes */

  xfrwidth = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >>
              DMACH_FLAG_PERIPHWIDTH_SHIFT;

  switch (xfrwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        maxtransfer = XDMACH_CUBC_UBLEN_MAX;
        break;

      case 1: /* 16 bits, 2 bytes */
        maxtransfer = 2 * XDMACH_CUBC_UBLEN_MAX;
        break;

      case 2: /* 32 bits 4 bytes */
        maxtransfer = 4 * XDMACH_CUBC_UBLEN_MAX;
        break;

      case 3: /* 64 bits, 8 bytes */
        maxtransfer = 8 * XDMACH_CUBC_UBLEN_MAX;
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

static uint32_t sam_ntxtransfers(struct sam_xdmach_s *xdmach,
                                 uint32_t dmasize)
{
  unsigned int srcwidth;

  /* Adjust the source transfer size for the source chunk size (memory
   * chunk size).  BTSIZE is "the number of transfers to be performed, that
   * is, for writes it refers to the number of source width transfers
   * to perform when XDMAC is flow controller. For Reads, BTSIZE refers to
   * the number of transfers completed on the Source Interface. ..."
   */

  srcwidth = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >>
              DMACH_FLAG_PERIPHWIDTH_SHIFT;

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
 * Name: sam_cubc
 *
 * Description:
 *  Calculate the CUBC transfer size
 *
 ****************************************************************************/

static inline uint32_t sam_cubc(struct sam_xdmach_s *xdmach,
                                uint32_t dmasize)
{
  uint32_t ntransfers;

  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  ntransfers = sam_ntxtransfers(xdmach, dmasize);

  DEBUGASSERT(ntransfers <= XDMACH_CUBC_UBLEN_MAX);
  return (ntransfers << XDMACH_CUBC_UBLEN_SHIFT);
}

/****************************************************************************
 * Name: sam_txcc
 *
 * Description:
 *  Decode the flags to get the correct Channel Control (CC) Register bit
 *  settings for a transmit (memory to peripheral) transfer.
 *
 * Single Block:
 *  1. Clear TYPE for a memory to memory transfer, otherwise set
 *     this bit for memory to/from peripheral transfer.
 *  2. Program MBSIZE to the memory burst size used.
 *  3. Program SAM/DAM to the memory addressing scheme.
 *  4. Program SYNC to select the peripheral transfer direction.
 *  5. Set PROT to activate a secure channel.
 *  6. Program CSIZE to configure the channel chunk size (only
 *     relevant for peripheral synchronized transfer).
 *  7. Program DWIDTH to configure the transfer data width.
 *  8. Program SIF, DIF to configure the master interface
 *     used to read data and write data respectively.
 *  9. Program PERID to select the active hardware request line
 *     (only relevant for a peripheral synchronized transfer).
 *  10. Set SWREQ to use software request (only relevant for a
 *      peripheral synchronized transfer).
 *
 ****************************************************************************/

static inline uint32_t sam_txcc(struct sam_xdmach_s *xdmach)
{
  uint32_t regval = 0;
  uint32_t field;

  /* 1. Clear TYPE for a memory to memory transfer, otherwise set
   *    this bit for memory to/from peripheral transfer.
   *
   * By convention, TX refers to a memory to peripheral transfer.  So the
   * source is memory.  Is the "peripheral" destination a peripheral? or
   * it is really memory?
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Yes.. Use peripheral synchronized mode */

      regval |= XDMACH_CC_TYPE;
    }

  /* 2. Program MBSIZE to the memory burst size used.
   *
   * NOTE: This assumes the same encoding in the DMACH flags as in the CC
   * register MBSIZE field.
   */

  field   = (xdmach->flags & DMACH_FLAG_MEMBURST_MASK) >>
             DMACH_FLAG_MEMBURST_SHIFT;
  regval |= (field << XDMACH_CC_MBSIZE_SHIFT);

  /* 3. Program SAM/DAM to the memory addressing scheme.
   *
   * NOTE: This assumes that 0 means non-incrementing.
   * TX -> Source is memory.
   */

  if ((xdmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
    {
      regval |= XDMACH_CC_SAM_INCR;
    }

  /* TX -> Destination is peripheral. */

  if ((xdmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
    {
      regval |= XDMACH_CC_DAM_INCR;
    }

  /* 4. Program DSYNC to select the peripheral transfer direction.
   *
   * TX -> Memory to peripheral
   */

  regval |= XDMACH_CC_DSYNC;

#if 0 /* REVISIT */
  /* 5. Set PROT to activate a secure channel (REVISIT). */

  regval |= XDMACH_CC_PROT; /* Channel is unsecured */
#endif

  /* 6. Program CSIZE to configure the channel chunk size (only
   *    relevant for peripheral synchronized transfer).
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Peripheral synchronized mode -- set chunk size.
       * NOTE that we assume that encoding in the XDMACH flags is the same
       * as in the CC register CSIZE field.
       */

      field   = (xdmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE_MASK) >>
                 DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT;
      regval |= (field << XDMACH_CC_CSIZE_SHIFT);
    }

  /* 7. Program DWIDTH to configure the transfer data width.
   *
   * NOTE that we assume that encoding in the XDMACH flags is the same as in
   * the CC register CSIZE field.
   */

  field   = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >>
             DMACH_FLAG_PERIPHWIDTH_SHIFT;
  regval |= (field << XDMACH_CC_DWIDTH_SHIFT);

  /* 8. Program SIF, DIF to configure the master interface
   *    used to read data and write data respectively.
   *
   * TX -> Source is memory
   */

  if ((xdmach->flags & DMACH_FLAG_MEMAHB_MASK) == DMACH_FLAG_MEMAHB_AHB_IF1)
    {
      regval |= XDMACH_CC_SIF;
    }

  /* TX -> Destination is peripheral */

  if ((xdmach->flags & DMACH_FLAG_PERIPHAHB_MASK) == DMACH_FLAG_PERIPHAHB_AHB_IF1)
    {
      regval |= XDMACH_CC_DIF;
    }

  /* 9. Program PERID to select the active hardware request line
   *    (only relevant for a peripheral synchronized transfer).
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      int pid;

      /* Get the PID from the DMACH flags */

      pid = (xdmach->flags & DMACH_FLAG_PERIPHPID_MASK) >>
             DMACH_FLAG_PERIPHPID_SHIFT;

      /* Look up the DMA channel code for TX:  Peripheral is the sink. */

      field   = sam_sink_channel(xdmach, pid);
      regval |= (field << XDMACH_CC_CSIZE_SHIFT);

#if 0 /* Not supported */
      /* 10. Set SWREQ to use software request (only relevant for a
       *     peripheral synchronized transfer).
       */

      regval |= XDMACH_CC_SWREQ;
#endif
    }

  return regval;
}

/****************************************************************************
 * Name: sam_rxcc
 *
 * Description:
 *  Decode the flags to get the correct Channel Control (CC) Register bit
 *  settings for a receive (peripheral to memory) transfer.
 *
 *  1. Clear TYPE for a memory to memory transfer, otherwise set
 *     this bit for memory to/from peripheral transfer.
 *  2. Program MBSIZE to the memory burst size used.
 *  3. Program SAM/DAM to the memory addressing scheme.
 *  4. Program SYNC to select the peripheral transfer direction.
 *  5. Set PROT to activate a secure channel.
 *  6. Program CSIZE to configure the channel chunk size (only
 *     relevant for peripheral synchronized transfer).
 *  7. Program DWIDTH to configure the transfer data width.
 *  8. Program SIF, DIF to configure the master interface
 *     used to read data and write data respectively.
 *  9. Program PERID to select the active hardware request line
 *     (only relevant for a peripheral synchronized transfer).
 *  10. Set SWREQ to use software request (only relevant for a
 *      peripheral synchronized transfer).
 *
 ****************************************************************************/

static inline uint32_t sam_rxcc(struct sam_xdmach_s *xdmach)
{
  uint32_t regval = 0;
  uint32_t field;

  /* 1. Clear TYPE for a memory to memory transfer, otherwise set
   *    this bit for memory to/from peripheral transfer.
   *
   * By convention, RX refers to a peripheral to memory transfer.  So the
   * source is peripheral.  Is the "peripheral" source a peripheral? or
   * is it also memory?
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Yes.. Use peripheral synchronized mode */

      regval |= XDMACH_CC_TYPE;
    }

  /* 2. Program MBSIZE to the memory burst size used.
   *
   * NOTE: This assumes the same encoding in the DMACH flags as in the CC
   * register MBSIZE field.
   */

  field   = (xdmach->flags & DMACH_FLAG_MEMBURST_MASK) >>
             DMACH_FLAG_MEMBURST_SHIFT;
  regval |= (field << XDMACH_CC_MBSIZE_SHIFT);

  /* 3. Program SAM/DAM to the memory addressing scheme.
   *
   * NOTE: This assumes that 0 means non-incrementing.
   * RX -> Source is peripheral.
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
    {
      regval |= XDMACH_CC_SAM_INCR;
    }

  /* RX -> Destination is memory. */

  if ((xdmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
    {
      regval |= XDMACH_CC_DAM_INCR;
    }

  /* 4. Program DSYNC to select the peripheral transfer direction.
   *
   * RX -> Peripheral to memory (DSYNC == 0)
   */

#if 0 /* REVISIT */
  /* 5. Set PROT to activate a secure channel (REVISIT) */

  regval |= XDMACH_CC_PROT; /* Channel is unsecured */
#endif

  /* 6. Program CSIZE to configure the channel chunk size (only
   *    relevant for peripheral synchronized transfer).
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Peripheral synchronized mode -- set chunk size.
       * NOTE that we assume that encoding in the XDMACH flags is the same
       * as in the CC register CSIZE field.
       */

      field   = (xdmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE_MASK) >>
                 DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT;
      regval |= (field << XDMACH_CC_CSIZE_SHIFT);
    }

  /* 7. Program DWIDTH to configure the transfer data width.
   *
   * NOTE that we assume that encoding in the XDMACH flags is the same as in
   * the CC register CSIZE field.
   */

  field   = (xdmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >>
             DMACH_FLAG_PERIPHWIDTH_SHIFT;
  regval |= (field << XDMACH_CC_DWIDTH_SHIFT);

  /* 8. Program SIF, DIF to configure the master interface
   *    used to read data and write data respectively.
   *
   * RX -> Source is peripheral
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHAHB_MASK) == DMACH_FLAG_PERIPHAHB_AHB_IF1)
    {
      regval |= XDMACH_CC_SIF;
    }

  /* RX -> Destination is memory */

  if ((xdmach->flags & DMACH_FLAG_MEMAHB_MASK) == DMACH_FLAG_MEMAHB_AHB_IF1)
    {
      regval |= XDMACH_CC_DIF;
    }

  /* 9. Program PERID to select the active hardware request line
   *    (only relevant for a peripheral synchronized transfer).
   */

  if ((xdmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      int pid;

      /* Get the PID from the DMACH flags */

      pid = (xdmach->flags & DMACH_FLAG_PERIPHPID_MASK) >>
             DMACH_FLAG_PERIPHPID_SHIFT;

      /* Look up the DMA channel code for RX:  Peripheral is the source. */

      field   = sam_source_channel(xdmach, pid);
      regval |= (field << XDMACH_CC_CSIZE_SHIFT);

#if 0 /* Not supported */
      /* 10. Set SWREQ to use software request (only relevant for a
       *     peripheral synchronized transfer).
       */

      regval |= XDMACH_CC_SWREQ;
#endif
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

static struct chnext_view1_s *
sam_allocdesc(struct sam_xdmach_s *xdmach, struct chnext_view1_s *prev,
              uint32_t csa, uint32_t cda, uint32_t cubc)
{
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);
  struct chnext_view1_s *descr = NULL;
  int i;

  /* Sanity check -- csa == 0 is the indication that the link is unused.
   * Obviously setting it to zero would break that usage.
   */

#ifdef CONFIG_DEBUG_FEATURES
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
          if (xdmac->descr[i].csa == 0)
            {
              /* We have it.  Initialize the new link list entry */

              descr        = &xdmac->descr[i];
              descr->cnda  = 0;    /* Next Descriptor Address */
              descr->cubc  = cubc; /* Channel Microblock Control Register */
              descr->csa   = csa;  /* Source address */
              descr->cda   = cda;  /* Destination address */

              /* And then hook it at the tail of the link list */

              if (!prev)
                {
                  /* There is no previous link.  This is the new head of
                   * the list
                   */

                  DEBUGASSERT(xdmach->llhead == NULL && xdmach->lltail == NULL);
                  xdmach->llhead = descr;
                }
              else
                {
                  DEBUGASSERT(xdmach->llhead != NULL && xdmach->lltail == prev);

                  /* When the second link is added to the list, that is the
                   * cue that we are going to do the link list transfer.
                   *
                   * Set the NDE field in the previous descriptor; Clear the
                   * NDE field in the final descriptor.
                   */

                  prev->cubc |= CHNEXT_UBC_NDE;

                  /* Link the previous tail to the new tail.
                   * REVISIT:  This assumes that the next description is fetched
                   * via AHB IF0.
                   */

                  prev->cnda = (uint32_t)sam_physramaddr((uintptr_t)descr);
                }

              /* In any event, this is the new tail of the list */

              xdmach->lltail = descr;

              /* Assume that we will be doing multiple buffer transfers and that
               * that hardware will be accessing the descriptor via DMA.
               */

              arch_clean_dcache((uintptr_t)descr,
                                (uintptr_t)descr + sizeof(struct chnext_view1_s));
              break;
            }
        }

      /* Because we hold a count from the counting semaphore, the above
       * search loop should always be successful.
       */

      sam_givechsem(xdmac);
      DEBUGASSERT(descr != NULL);
    }

  return descr;
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
  struct chnext_view1_s *descr;
  uintptr_t paddr;

  /* Get the head of the link list and detach the link list from the DMA
   * channel
   */

  descr           = xdmach->llhead;
  xdmach->llhead = NULL;
  xdmach->lltail = NULL;

  while (descr != NULL)
    {
      /* Valid, in-use descriptors never have csa == 0 */

      DEBUGASSERT(descr->csa != 0);

      /* Get the physical address of the next descriptor in the list */

      paddr = descr->cnda;

      /* Free the descriptor by simply nullifying it and bumping up the
       * semaphore count.
       */

      memset(descr, 0, sizeof(struct chnext_view1_s));
      sam_givedsem(xdmac);

      /* Get the virtual address of the next descriptor in the list */

      descr = (struct chnext_view1_s *)sam_virtramaddr(paddr);
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
  uint32_t cubc;

  /* If we are appending a buffer to a linklist, then re-use the previously
   * calculated CC register value.  Otherwise, create the CC register value
   * from the properties of the transfer.
   */

  if (!xdmach->llhead)
    {
      xdmach->cc = sam_txcc(xdmach);
    }

  /* Calculate the number of transfers for CUBC */

  cubc  = sam_cubc(xdmach, nbytes);
  cubc |= (CHNEXT_UBC_NVIEW_1 | CHNEXT_UBC_NSEN);

  /* Add the new link list entry */

  if (!sam_allocdesc(xdmach, xdmach->lltail, maddr, paddr, cubc))
    {
      return -ENOMEM;
    }

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
  uint32_t cubc;

  /* If we are appending a buffer to a linklist, then re-use the previously
   * calculated CC register value.  Otherwise, create the CC register value
   * from the properties of the transfer.
   */

  if (!xdmach->llhead)
    {
      xdmach->cc = sam_rxcc(xdmach);
    }

  /* Calculate the number of transfers for CUBC */

  cubc  = sam_cubc(xdmach, nbytes);
  cubc |= (CHNEXT_UBC_NVIEW_1 | CHNEXT_UBC_NSEN);

  /* Add the new link list entry */

  if (!sam_allocdesc(xdmach, xdmach->lltail, paddr, maddr, cubc))
    {
      return -ENOMEM;
    }

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
  struct chnext_view1_s *llhead = xdmach->llhead;

  /* 1. Read the XDMAC Global Channel Status Register (XDMAC_GS) to choose a
   *    free channel.
   *
   * In this implementation, the free channel is assigned in a different
   * manner.
   */

  /* 2. Clear any pending interrupts from any previous XDMAC transfer by
   *    reading the XDMAC Channel Interrupt Status Register (CIS).
   */

  (void)sam_getdmach(xdmach, SAM_XDMACH_CIS_OFFSET);

  /* 3. Write the starting source address in the Channel Source Address (CSA)
   *    Register.
   */

  DEBUGASSERT(llhead != NULL && llhead->csa != 0);
  sam_putdmach(xdmach, llhead->csa, SAM_XDMACH_CSA_OFFSET);

  /* 4. Write the starting destination address in the Channel Destination
   *    Address Register (CDA).
   */

  sam_putdmach(xdmach, llhead->cda, SAM_XDMACH_CDA_OFFSET);

  /* 5. Program field UBLEN in the XDMAC Channel Microblock Control (CUBC)
   *    Register with the number of data.
   */

  sam_putdmach(xdmach, llhead->cubc, SAM_XDMACH_CUBC_OFFSET);

  /* 6. Program the Channel Control (CC) Register */

  sam_putdmach(xdmach, xdmach->cc, SAM_XDMACH_CC_OFFSET);

  /* 7. Clear the following five registers:
   *
   *    XDMAC Channel Next Descriptor Control (CNDC) Register
   *    XDMAC Channel Block Control (CBC) Register
   *    XDMAC Channel Data Stride Memory Set Pattern (CDSMSP) Register
   *    XDMAC Channel Source Microblock Stride (CSUS) Register
   *    XDMAC Channel Destination Microblock Stride (CDUS)Register
   *
   * This respectively indicates that the linked list is disabled, there is
   * only one block and striding is disabled
   */

  sam_putdmach(xdmach, 0, SAM_XDMACH_CNDC_OFFSET);
  sam_putdmach(xdmach, 0, SAM_XDMACH_CBC_OFFSET);
  sam_putdmach(xdmach, 0, SAM_XDMACH_CDSMSP_OFFSET);
  sam_putdmach(xdmach, 0, SAM_XDMACH_CSUS_OFFSET);
  sam_putdmach(xdmach, 0, SAM_XDMACH_CDUS_OFFSET);

  /* 8. Enable the Microblock interrupt by setting the "End of Block"
   *    interrupt bit in the XDMAC Channel Interrupt Enable (CIE) Register.
   */

  sam_putdmach(xdmach, XDMAC_CHINT_BI | XDMAC_CHINT_ERRORS,
               SAM_XDMACH_CIE_OFFSET);

  /*    Enable the Channel Interrupt Enable bit by setting the corresponding
   *    bit in the XDMAC Global Interrupt Enable (GIE) Register.
   */

  sam_putdmac(xdmac, XDMAC_CHAN(xdmach->chan), SAM_XDMAC_GIE_OFFSET);

  /* 9. Enable the channel by setting the corresponding bit in the
   *    XDMAC Global Channel Enable (GE) Register.  The channel bit will
   *    be set in the GS register by hardware.
   */

  sam_putdmac(xdmac, XDMAC_CHAN(xdmach->chan), SAM_XDMAC_GE_OFFSET);

  /* 10. The DMA has been started. Once completed, the DMA channel sets the
   *     corresponding "End of Block Interrupt" Status bit in the channel
   *     CIS register and generates an global interrupt for the channel.
   *     The channel bit will be cleared in the GS register by hardware.
   */

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
#ifdef CONFIG_DEBUG_ASSERTIONS
  struct chnext_view1_s *llhead = xdmach->llhead;
#endif
  uintptr_t paddr;
  uint32_t regval;

  DEBUGASSERT(llhead != NULL && llhead->csa != 0);

  /* 1. Read the XDMAC Global Channel Status Register (XDMAC_GS) to choose a
   *    free channel.
   *
   * In this implementation, the free channel is assigned in a different
   * manner.
   */

  /* 2. Clear any pending interrupts from any previous XDMAC transfer by
   *    reading the XDMAC Channel Interrupt Status Register (CIS).
   */

  (void)sam_getdmach(xdmach, SAM_XDMACH_CIS_OFFSET);

  /* 3. Build a linked list of transfer descriptors in memory. The
   *    descriptor view is programmable on a per descriptor basis. The
   *    linked list items structure must be word aligned.  CUBC NDE
   *    must be configured to 0 in the last descriptor to terminate the
   *    list.
   *
   * This was done during the RX/TX setup phases of the DMA transfer.
   */

  /* Since we are using view1, I imagine that we need to set the Channel
   * Control (CC) Register.
   */

  sam_putdmach(xdmach, xdmach->cc, SAM_XDMACH_CC_OFFSET);

  /* 4. Program field NDA in the XDMAC Channel Next Descriptor Address
   *    (CNDA) Register with the first descriptor address and bit NDAIF
   *    with the master interface identifier.
   *
   * REVIST:  Using NDAIF=0.  Is that correct?
   */

  paddr = sam_physramaddr((uintptr_t)xdmach->llhead);
  sam_putdmach(xdmach, (uint32_t)paddr, SAM_XDMACH_CNDA_OFFSET);

  /* 5. Program the CNDC register:
   *
   *    a. Set NDE to enable the descriptor fetch.
   *    b. Set NDSUP to update the source address at the descriptor fetch
   *       time, otherwise clear this bit.
   *    c. Set NDDUP to update the destination address at the descriptor
   *       fetch time, otherwise clear this bit.
   *    d. Program the NDVIEW field to define the length of the first
   *       descriptor.
   */

  regval = (XDMACH_CNDC_NDE | XDMACH_CNDC_NDVIEW_NDV2);

  /* Update the source address if this is a memory-to-* transfer.
   *
   *   TYPE = 0 -> memory-to-memory
   *   TYPE = 1 && DSYNC = 1 -> memory-to-peripheral
   */

  if ((xdmach->cc & XDMACH_CC_TYPE) == 0 ||
      (xdmach->cc & XDMACH_CC_DSYNC) != 0)
    {
      regval |= XDMACH_CNDC_NDSUP;
    }

  /* Update the destination address if this is a *-to-memory transfer.
   *
   *   TYPE = 0 -> memory-to-memory
   *   TYPE = 1 && DSYNC = 0 -> peripheral-to-memory
   */

  if ((xdmach->cc & XDMACH_CC_TYPE) == 0 ||
      (xdmach->cc & XDMACH_CC_TYPE) == 0)
    {
      regval |= XDMACH_CNDC_NDDUP;
    }

  sam_putdmach(xdmach, regval, SAM_XDMACH_CNDC_OFFSET);

  /* 6. Enable the End of Linked List interrupt by setting the LI bit in the
   *    CIE register.
   */

  sam_putdmach(xdmach, XDMAC_CHINT_LI | XDMAC_CHINT_ERRORS,
               SAM_XDMACH_CIE_OFFSET);

  /*    Enable the Channel Interrupt Enable bit by setting the corresponding
   *    bit in the XDMAC Global Interrupt Enable (GIE) Register.
   */

  sam_putdmac(xdmac, XDMAC_CHAN(xdmach->chan), SAM_XDMAC_GIE_OFFSET);

  /* 7. Enable the channel by setting the corresponding bit in the
   *    XDMAC Global Channel Enable (GE) Register.  The channel bit will
   *    be set in the GS register by hardware.
   */

  sam_putdmac(xdmac, XDMAC_CHAN(xdmach->chan), SAM_XDMAC_GE_OFFSET);

  /* 8. The DMA has been started. Once completed, the DMA channel sets the
   *    corresponding "End of Block Interrupt" Status bit in the channel
   *    CIS register and generates an global interrupt for the channel.
   *    The channel bit will be cleared in the GS register by hardware.
   */

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
  uint32_t chanbit = XDMAC_CHAN(xdmach->chan);

  /* Disable all channel interrupts */

  sam_putdmac(xdmac, chanbit, SAM_XDMAC_GID_OFFSET);
  sam_putdmach(xdmach, XDMAC_CHINT_ALL, SAM_XDMACH_CID_OFFSET);

  /* Under normal operation, the software enables a channel by setting the
   * associated bit in the Global Channel Enable (GE) Register.  The hardware
   * then disables a channel on transfer completion by clearing channel bit
   * in the GS register. To disable a channel, setting the channel bit in the
   * GD register and poll the channel bit in GS register to determine when
   * the channel has been disabled.
   */

  sam_putdmac(xdmac, chanbit, SAM_XDMAC_GD_OFFSET);
  while ((sam_getdmac(xdmac, SAM_XDMAC_GS_OFFSET) & chanbit) != 0);

  /* Free the linklist */

  sam_freelinklist(xdmach);

  /* If this was an RX DMA (peripheral-to-memory), then invalidate the cache
   * to force reloads from memory.
   */

  if (xdmach->rx)
    {
      arch_invalidate_dcache(xdmach->rxaddr, xdmach->rxaddr + xdmach->rxsize);
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

static int sam_xdmac_interrupt(int irq, void *context, FAR void *arg)
{
  struct sam_xdmac_s *xdmac = (struct sam_xdmac_s *)arg;
  struct sam_xdmach_s *xdmach;
  unsigned int chndx;
  uint32_t gpending;
  uint32_t chpending;
  uint32_t bit;

  DEBUGASSERT(xdmac != NULL);

  /* Get the set of pending, unmasked global XDMAC interrupts */

  gpending = sam_getdmac(xdmac, SAM_XDMAC_GIS_OFFSET) &
             sam_getdmac(xdmac, SAM_XDMAC_GIM_OFFSET);

  /* Yes.. Check each bit to see which channel(s) have interrupted */

  for (chndx = 0; chndx < SAM_NDMACHAN && gpending != 0; chndx++)
    {
      /* Are any interrupts pending for this channel? */

      bit = XDMAC_CHAN(chndx);
      if ((gpending & bit) != 0)
        {
          xdmach = &xdmac->xdmach[chndx];

          /* Get the set of interrupts pending for this channel */

          chpending = sam_getdmach(xdmach, SAM_XDMACH_CIS_OFFSET) &
                      sam_getdmach(xdmach, SAM_XDMACH_CIM_OFFSET);

          /* Yes.. Did an error occur? */

          if ((chpending & XDMAC_CHINT_ERRORS) != 0)
            {
              /* Yes... Terminate the transfer with an error? */

              dmaerr("ERROR: DMA failed: %08x\n", chpending);
              sam_dmaterminate(xdmach, -EIO);
            }

          /* Is the transfer complete? */

          else if ((chpending & (XDMAC_CHINT_BI | XDMAC_CHINT_LI)) != 0)
            {
              /* Yes.. Terminate the transfer with success */

              sam_dmaterminate(xdmach, OK);
            }

          /* Else what? */

          else
            {
              dmaerr("ERROR: Unexpected interrupt: %08x\n", chpending);
              DEBUGPANIC();
            }

          /* Clear the bit in the sampled set of pending global interrupts */

          gpending &= !bit;
        }
    }

  return OK;
}

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

  sam_putdmac(xdmac, XDMAC_CHAN_ALL, SAM_XDMAC_GID_OFFSET);

  /* Disable all DMA channels */

  sam_putdmac(xdmac, XDMAC_CHAN_ALL, SAM_XDMAC_GD_OFFSET);

  /* Initialize semaphores */

  nxsem_init(&xdmac->chsem, 0, 1);
  nxsem_init(&xdmac->dsem, 0, SAM_NDMACHAN);
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
  dmainfo("Initialize XDMAC0\n");

  /* Enable peripheral clock */

  sam_xdmac0_enableclk();

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_XDMAC0, sam_xdmac_interrupt, &g_xdmac0);

  /* Initialize the controller */

  sam_dmainitialize(&g_xdmac0);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_XDMAC0);
#endif

#ifdef CONFIG_SAMA5_XDMAC1
  dmainfo("Initialize XDMAC1\n");

  /* Enable peripheral clock */

  sam_xdmac1_enableclk();

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_XDMAC1, sam_xdmac_interrupt, &g_xdmac1);

  /* Initialize the controller */

  sam_dmainitialize(&g_xdmac1);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_XDMAC1);
#endif
}

/****************************************************************************
 * Name: sam_dmachannel
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

DMA_HANDLE sam_dmachannel(uint8_t dmacno, uint32_t chflags)
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
      dmaerr("ERROR: Bad XDMAC number: %d\n", dmacno);
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
      dmainfo("XDMAC%d CH%d: chflags: %08x returning xdmach: %p\n",
              (int)dmacno, xdmach->chan, (int)chflags, xdmach);
    }
  else
    {
      dmaerr("ERROR: Failed allocate XDMAC%d channel\n", (int)dmacno);
    }

  return (DMA_HANDLE)xdmach;
}

/************************************************************************************
 * Name: sam_dmaconfig
 *
 * Description:
 *   There are two channel usage models:  (1) The channel is allocated and configured
 *   in one step.  This is the typical case where a DMA channel performs a constant
 *   role.  The alternative is (2) where the DMA channel is reconfigured on the fly.
 *   In this case, the chflags provided to sam_dmachannel are not used and
 *   sam_dmaconfig() is called before each DMA to configure the DMA channel
 *   appropriately.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t chflags)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;

  /* Set the new DMA channel flags. */

  xdmach->flags = chflags;

#if defined(CONFIG_SAMA5_XDMAC0) && defined(CONFIG_SAMA5_XDMAC1)
  dmainfo("XDMAC%d CH%d: chflags: %08x\n",
          xdmach->xdmac, xdmach->chan, (int)chflags);
#elif defined(CONFIG_SAMA5_XDMAC0)
  dmainfo("XDMAC0 CH%d: chflags: %08x\n",
          xdmach->chan, (int)chflags);
#else
  dmainfo("XDMAC1 CH%d: chflags: %08x\n",
          xdmach->chan, (int)chflags);
#endif
}

/****************************************************************************
 * Name: sam_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until sam_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmafree(DMA_HANDLE handle)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  struct sam_xdmac_s *xdmac;

  dmainfo("xdmach: %p\n", xdmach);
  DEBUGASSERT((xdmach != NULL) && (xdmach->inuse));

  xdmac = sam_controller(xdmach);

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

  dmainfo("xdmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          xdmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(xdmach);
  dmainfo("llhead: %p lltail: %p\n", xdmach->llhead, xdmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtransfer(xdmach);
  remaining   = nbytes;

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_txbuffer(xdmach, paddr, maddr, maxtransfer);
      if (ret == OK)
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

  arch_clean_dcache(maddr, maddr + nbytes);
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

  dmainfo("xdmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          xdmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(xdmach);
  dmainfo("llhead: %p lltail: %p\n", xdmach->llhead, xdmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtransfer(xdmach);
  remaining   = nbytes;

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_rxbuffer(xdmach, paddr, maddr, maxtransfer);
      if (ret == OK)
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

  arch_clean_dcache(maddr, maddr + nbytes);
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

  dmainfo("xdmach: %p callback: %p arg: %p\n", xdmach, callback, arg);
  DEBUGASSERT(xdmach != NULL);

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * link list).
   */

  if (xdmach->llhead)
    {
      /* Save the callback info.  This will be invoked whent the DMA commpletes */

      xdmach->callback = callback;
      xdmach->arg      = arg;

      /* Is this a single block transfer?  Or a multiple block transfer? */

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

  dmainfo("xdmach: %p\n", xdmach);
  DEBUGASSERT(xdmach != NULL);

  flags = enter_critical_section();
  sam_dmaterminate(xdmach, -EINTR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
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

  flags        = enter_critical_section();

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

  regs->cim    = sam_getdmach(xdmach, SAM_XDMACH_CIM_OFFSET);
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

  leave_critical_section(flags);
}
#endif /* CONFIG_DEBUG_DMA_INFO */

/****************************************************************************
 * Name: sam_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void sam_dmadump(DMA_HANDLE handle, const struct sam_dmaregs_s *regs,
                 const char *msg)
{
  struct sam_xdmach_s *xdmach = (struct sam_xdmach_s *)handle;
  struct sam_xdmac_s *xdmac = sam_controller(xdmach);

  dmainfo("%s\n", msg);
  dmainfo("  DMA Global Registers:\n");
  dmainfo("     GTYPE[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GTYPE_OFFSET, regs->gtype);
  dmainfo("      GCFG[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GCFG_OFFSET, regs->gcfg);
  dmainfo("      GWAC[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GWAC_OFFSET, regs->gwac);
  dmainfo("       GIM[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GIM_OFFSET, regs->gim);
  dmainfo("       GIS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GIS_OFFSET, regs->gis);
  dmainfo("        GS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GS_OFFSET, regs->gs);
  dmainfo("       GRS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GRS_OFFSET, regs->grs);
  dmainfo("       GWS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GWS_OFFSET, regs->gws);
  dmainfo("      GSWS[%08x]: %08x\n", xdmac->base + SAM_XDMAC_GSWS_OFFSET, regs->gsws);
  dmainfo("  DMA Channel Registers:\n");
  dmainfo("       CIM[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CIM_OFFSET, regs->cim);
  dmainfo("       CIS[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CIS_OFFSET, regs->cis);
  dmainfo("       CSA[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CSA_OFFSET, regs->csa);
  dmainfo("       CDA[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CDA_OFFSET, regs->cda);
  dmainfo("      CNDA[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CNDA_OFFSET, regs->cnda);
  dmainfo("      CNDC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CNDC_OFFSET, regs->cndc);
  dmainfo("      CUBC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CUBC_OFFSET, regs->cubc);
  dmainfo("       CBC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CBC_OFFSET, regs->cbc);
  dmainfo("        CC[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CC_OFFSET, regs->cc);
  dmainfo("    CDSMSP[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CDSMSP_OFFSET, regs->cdsmsp);
  dmainfo("      CSUS[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CSUS_OFFSET, regs->csus);
  dmainfo("      CDUS[%08x]: %08x\n", xdmach->base + SAM_XDMACH_CDUS_OFFSET, regs->cdus);
}
#endif /* CONFIG_DEBUG_DMA_INFO */
#endif /* CONFIG_SAMA5_XDMAC0 || CONFIG_SAMA5_XDMAC1 */
