/****************************************************************************
 * arch/arm/src/sama5/sam_dmac.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "sched/sched.h"

#include "chip.h"
#include "sam_dmac.h"
#include "sam_periphclks.h"
#include "sam_memories.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* All of the currently supported SAMA5 chips support two DMA controllers
 * of 8 DMA Channels each.
 */

#if SAM_NDMAC < 1
#  undef CONFIG_SAMA5_DMAC1
#  undef CONFIG_SAMA5_DMAC0
#elif SAM_NDMAC < 2
#  undef CONFIG_SAMA5_DMAC1
#endif

/* Condition out the whole file unless DMA is selected in the configuration */

#if defined(CONFIG_SAMA5_DMAC0) || defined(CONFIG_SAMA5_DMAC1)

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

#define DMAC_CH_CTRLB_BOTHDSCR \
  (DMAC_CH_CTRLB_SRCDSCR | DMAC_CH_CTRLB_DSTDSCR)

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

struct sam_dmach_s
{
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
  uint8_t dmac;                   /* DMA controller number (0-1) */
#endif
  uint8_t chan;                   /* DMA channel number (0-6) */
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

/* This structure describes the state of one DMA controller */

struct sam_dmac_s
{
  /* These semaphores protect the DMA channel and descriptor tables */

  sem_t chsem;                       /* Protects channel table */
  sem_t dsem;                        /* Protects descriptor table */
  uint32_t base;                     /* DMA register channel base address */

  /* This array describes the available link list descriptors */

  struct dma_linklist_s *desc;

  /* This array describes each DMA channel */

  struct sam_dmach_s *dmach;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* CTRLA field lookups */

static const uint32_t g_srcwidth[4] =
{
  DMAC_CH_CTRLA_SRCWIDTH_BYTE,
  DMAC_CH_CTRLA_SRCWIDTH_HWORD,
  DMAC_CH_CTRLA_SRCWIDTH_WORD,
  DMAC_CH_CTRLA_SRCWIDTH_DWORD
};

static const uint32_t g_destwidth[4] =
{
  DMAC_CH_CTRLA_DSTWIDTH_BYTE,
  DMAC_CH_CTRLA_DSTWIDTH_HWORD,
  DMAC_CH_CTRLA_DSTWIDTH_WORD,
  DMAC_CH_CTRLA_DSTWIDTH_DWORD
};

static const uint32_t g_fifocfg[3] =
{
  DMAC_CH_CFG_FIFOCFG_ALAP,
  DMAC_CH_CFG_FIFOCFG_HALF,
  DMAC_CH_CFG_FIFOCFG_ASAP
};

/* These tables map peripheral IDs to channels.  A lookup is performed
 * before each DMA transfer in order to map the peripheral IDs to the
 * correct channel.  This must be done because the channel can change with
 * the direction of the transfer.
 */

#ifdef CONFIG_SAMA5_DMAC0
/* DMA controller 0, RX DMA: */

static const struct sam_pidmap_s g_dmac0_rxchan[] =
{
  { SAM_PID_HSMCI0, DMAC0_CH_HSMCI0    },    /* HSMCI0 Receive/transmit */
  { SAM_PID_SPI0,   DMAC0_CH_SPI0_RX   },    /* SPI0 Receive */
  { SAM_PID_USART0, DMAC0_CH_USART0_RX },    /* USART0 Receive */
  { SAM_PID_USART1, DMAC0_CH_USART1_RX },    /* USART1 Receive */
  { SAM_PID_TWI0,   DMAC0_CH_TWI0_RX   },    /* TWI0 Receive */
  { SAM_PID_TWI1,   DMAC0_CH_TWI1_RX   },    /* TWI1 Receive */
  { SAM_PID_UART0,  DMAC0_CH_UART0_RX  },    /* UART0 Receive */
  { SAM_PID_SSC0,   DMAC0_CH_SSC0_RX   },    /* SSC0 Receive */
  { SAM_PID_SMD,    DMAC0_CH_SMD_RX    },    /* SMD Receive */
};
#define NDMAC0_RXCHANNELS (sizeof(g_dmac0_rxchan) / sizeof(struct sam_pidmap_s))

/* DMA controller 0, TX DMA: */

static const struct sam_pidmap_s g_dmac0_txchan[] =
{
  { SAM_PID_HSMCI0, DMAC0_CH_HSMCI0    },    /* HSMCI0 Receive/transmit */
  { SAM_PID_SPI0,   DMAC0_CH_SPI0_TX   },    /* SPI0 Transmit */
  { SAM_PID_USART0, DMAC0_CH_USART0_TX },    /* USART0 Transmit */
  { SAM_PID_USART1, DMAC0_CH_USART1_TX },    /* USART1 Transmit */
  { SAM_PID_TWI0,   DMAC0_CH_TWI0_TX   },    /* TWI0 Transmit */
  { SAM_PID_TWI1,   DMAC0_CH_TWI1_TX   },    /* TWI1 Transmit */
  { SAM_PID_UART0,  DMAC0_CH_UART0_TX  },    /* UART0 Transmit */
  { SAM_PID_SSC0,   DMAC0_CH_SSC0_TX   },    /* SSC0 Transmit */
  { SAM_PID_SMD,    DMAC0_CH_SMD_TX    },    /* SMD Transmit */
};
#define NDMAC0_TXCHANNELS (sizeof(g_dmac0_txchan) / sizeof(struct sam_pidmap_s))
#endif

#ifdef CONFIG_SAMA5_DMAC1
/* DMA controller 1, RX DMA: */

static const struct sam_pidmap_s g_dmac1_rxchan[] =
{
  { SAM_PID_HSMCI1, DMAC1_CH_HSMCI1    },    /* HSMCI1 Receive/transmit */
  { SAM_PID_HSMCI2, DMAC1_CH_HSMCI2    },    /* HSMCI2 Receive/transmit */
  { SAM_PID_ADC,    DMAC1_CH_ADC_RX    },    /* ADC Receive */
  { SAM_PID_SSC1,   DMAC1_CH_SSC1_RX   },    /* SSC1 Receive */
  { SAM_PID_UART1,  DMAC1_CH_UART1_RX  },    /* UART1 Receive */
  { SAM_PID_USART2, DMAC1_CH_USART2_RX },    /* USART2 Receive */
  { SAM_PID_USART3, DMAC1_CH_USART3_RX },    /* USART3 Receive */
  { SAM_PID_TWI2,   DMAC1_CH_TWI2_RX   },    /* TWI2 Receive */
  { SAM_PID_DBGU,   DMAC1_CH_DBGU_RX   },    /* DBGU Receive */
  { SAM_PID_SPI1,   DMAC1_CH_SPI1_RX   },    /* SPI1 Receive */
  { SAM_PID_AES,    DMAC1_CH_AES_RX    },    /* AES Receive */
  { SAM_PID_TDES,   DMAC1_CH_TDES_RX   },    /* TDES Receive */
};
#define NDMAC1_RXCHANNELS (sizeof(g_dmac1_rxchan) / sizeof(struct sam_pidmap_s))

/* DMA controller 1, TX DMA: */

static const struct sam_pidmap_s g_dmac1_txchan[] =
{
  { SAM_PID_HSMCI1, DMAC1_CH_HSMCI1    },    /* HSMCI1 Receive/transmit */
  { SAM_PID_HSMCI2, DMAC1_CH_HSMCI2    },    /* HSMCI2 Receive/transmit */
  { SAM_PID_SSC1,   DMAC1_CH_SSC1_TX   },    /* SSC1 Transmit */
  { SAM_PID_UART1,  DMAC1_CH_UART1_TX  },    /* UART1 Transmit */
  { SAM_PID_USART2, DMAC1_CH_USART2_TX },    /* USART2 Transmit */
  { SAM_PID_USART3, DMAC1_CH_USART3_TX },    /* USART3 Transmit */
  { SAM_PID_TWI2,   DMAC1_CH_TWI2_TX   },    /* TWI2 Transmit */
  { SAM_PID_DBGU,   DMAC1_CH_DBGU_TX   },    /* DBGU Transmit */
  { SAM_PID_SPI1,   DMAC1_CH_SPI1_TX   },    /* SPI1 Transmit */
  { SAM_PID_SHA,    DMAC1_CH_SHA_TX    },    /* SHA Transmit */
  { SAM_PID_AES,    DMAC1_CH_AES_TX    },    /* AES Transmit */
  { SAM_PID_TDES,   DMAC1_CH_TDES_TX   },    /* TDES Transmit */
};
#define NDMAC1_TXCHANNELS (sizeof(g_dmac1_txchan) / sizeof(struct sam_pidmap_s))
#endif

#ifdef CONFIG_SAMA5_DMAC0

/* This array describes the available link list descriptors */

struct dma_linklist_s g_desc0[CONFIG_SAMA5_NLLDESC];

/* This array describes the state of each DMAC0 channel 0 */

static struct sam_dmach_s g_dmach0[SAM_NDMACHAN] =
{
#if SAM_NDMACHAN > 0
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 0,
    .base     = SAM_DMAC0_CH0_BASE,
  },
#endif
#if SAM_NDMACHAN > 1
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 1,
    .base     = SAM_DMAC0_CH1_BASE,
  },
#endif
#if SAM_NDMACHAN > 2
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 2,
    .base     = SAM_DMAC0_CH2_BASE,
  },
#endif
#if SAM_NDMACHAN > 3
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 3,
    .base     = SAM_DMAC0_CH3_BASE,
  },
#endif
#if SAM_NDMACHAN > 4
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 4,
    .base     = SAM_DMAC0_CH4_BASE,
  },
#endif
#if SAM_NDMACHAN > 5
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 5,
    .base     = SAM_DMAC0_CH5_BASE,
  },
#endif
#if SAM_NDMACHAN > 6
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 6,
    .base     = SAM_DMAC0_CH6_BASE,
  },
#endif
#if SAM_NDMACHAN > 7
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 7,
    .base     = SAM_DMAC0_CH7_BASE,
  }
#endif
};

/* This describes the overall state of DMA controller 0 */

static struct sam_dmac_s g_dmac0 =
{
  /* DMAC 0 base address */

  .base       = SAM_DMAC0_VBASE,

  /* This array describes the available link list descriptors */

  .desc       = g_desc0,

  /* This array describes each DMA channel */

  .dmach      = g_dmach0,
};

#endif /* CONFIG_SAMA5_DMAC0 */

/* This array describes the state of DMA controller 1 */

#ifdef CONFIG_SAMA5_DMAC1
/* This array describes the available link list descriptors */

struct dma_linklist_s g_desc1[CONFIG_SAMA5_NLLDESC];

static struct sam_dmach_s g_dmach1[SAM_NDMACHAN] =
{
#if SAM_NDMACHAN > 0
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 0,
    .base     = SAM_DMAC1_CH0_BASE,
  },
#endif
#if SAM_NDMACHAN > 1
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 1,
    .base     = SAM_DMAC1_CH1_BASE,
  },
#endif
#if SAM_NDMACHAN > 2
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 2,
    .base     = SAM_DMAC1_CH2_BASE,
  },
#endif
#if SAM_NDMACHAN > 3
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 3,
    .base     = SAM_DMAC1_CH3_BASE,
  },
#endif
#if SAM_NDMACHAN > 4
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 4,
    .base     = SAM_DMAC1_CH4_BASE,
  },
#endif
#if SAM_NDMACHAN > 5
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 5,
    .base     = SAM_DMAC1_CH5_BASE,
  },
#endif
#if SAM_NDMACHAN > 6
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 6,
    .base     = SAM_DMAC1_CH6_BASE,
  },
#endif
#if SAM_NDMACHAN > 7
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 7,
    .base     = SAM_DMAC1_CH7_BASE,
  }
#endif
};

/* This describes the overall state of DMA controller 1 */

static struct sam_dmac_s g_dmac1 =
{
  /* DMAC 0 base address */

  .base       = SAM_DMAC1_VBASE,

  /* This array describes the available link list descriptors */

  .desc       = g_desc1,

  /* This array describes each DMA channel */

  .dmach      = g_dmach1,
};

#endif /* CONFIG_SAMA5_DMAC1 */

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

static int sam_takechsem(struct sam_dmac_s *dmac)
{
  return nxsem_wait_uninterruptible(&dmac->chsem);
}

static inline void sam_givechsem(struct sam_dmac_s *dmac)
{
  nxsem_post(&dmac->chsem);
}

/****************************************************************************
 * Name: sam_takedsem() and sam_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

static int sam_takedsem(struct sam_dmac_s *dmac)
{
  return nxsem_wait_uninterruptible(&dmac->dsem);
}

static inline void sam_givedsem(struct sam_dmac_s *dmac)
{
  nxsem_post(&dmac->dsem);
}

/****************************************************************************
 * Name: sam_getdmac
 *
 * Description:
 *  Read a global DMAC register
 *
 ****************************************************************************/

static inline uint32_t sam_getdmac(struct sam_dmac_s *dmac,
                                   unsigned int offset)
{
  return getreg32(dmac->base + offset);
}

/****************************************************************************
 * Name: sam_putdmac
 *
 * Description:
 *  Write a value to a global DMAC register
 *
 ****************************************************************************/

static inline void sam_putdmac(struct sam_dmac_s *dmac, uint32_t value,
                               unsigned int offset)
{
  putreg32(value, dmac->base + offset);
}

/****************************************************************************
 * Name: sam_getdmach
 *
 * Description:
 *  Read a DMAC channel register
 *
 ****************************************************************************/

static inline uint32_t sam_getdmach(struct sam_dmach_s *dmach,
                                    unsigned int offset)
{
  return getreg32(dmach->base + offset);
}

/****************************************************************************
 * Name: sam_putdmach
 *
 * Description:
 *  Write a value to a DMAC channel register
 *
 ****************************************************************************/

static inline void sam_putdmach(struct sam_dmach_s *dmach, uint32_t value,
                                unsigned int offset)
{
  putreg32(value, dmach->base + offset);
}

/****************************************************************************
 * Name: sam_controller
 *
 * Description:
 *    Given a DMA channel instance, return a pointer to the parent DMA
 *    controller instance.
 *
 ****************************************************************************/

static inline struct sam_dmac_s *sam_controller(struct sam_dmach_s *dmach)
{
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
  return dmach->dmac ? &g_dmac1 : &g_dmac0;
#elif defined(CONFIG_SAMA5_DMAC0)
  return &g_dmac0;
#else
  return &g_dmac1;
#endif
}

/****************************************************************************
 * Name: sam_fifocfg
 *
 * Description:
 *  Decode the FIFO config from the flags
 *
 ****************************************************************************/

static inline uint32_t sam_fifocfg(struct sam_dmach_s *dmach)
{
  unsigned int ndx;

  ndx = (dmach->flags & DMACH_FLAG_FIFOCFG_MASK) >> DMACH_FLAG_FIFOCFG_SHIFT;
  DEBUGASSERT(ndx < 3);
  return g_fifocfg[ndx];
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

static uint32_t sam_source_channel(struct sam_dmach_s *dmach, uint8_t pid,
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

#ifdef CONFIG_SAMA5_DMAC0
#ifdef CONFIG_SAMA5_DMAC1
  if (dmach->dmac == 0)
#endif
    {
      /* Use the DMAC0 lookup table */

      table = g_dmac0_rxchan;
      nentries = NDMAC0_RXCHANNELS;
    }
#endif

#ifdef CONFIG_SAMA5_DMAC1
#ifdef CONFIG_SAMA5_DMAC0
  else
#endif
    {
      /* Use the DMAC1 lookup table */

      table = g_dmac1_rxchan;
      nentries = NDMAC1_RXCHANNELS;
    }
#endif

  return (uint32_t)sam_channel(pid, table, nentries);
}

static uint32_t sam_sink_channel(struct sam_dmach_s *dmach, uint8_t pid,
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

#ifdef CONFIG_SAMA5_DMAC0
#ifdef CONFIG_SAMA5_DMAC1
  if (dmach->dmac == 0)
#endif
    {
      /* Use the DMAC0 lookup table */

      table = g_dmac0_txchan;
      nentries = NDMAC0_TXCHANNELS;
    }
#endif

#ifdef CONFIG_SAMA5_DMAC1
#ifdef CONFIG_SAMA5_DMAC0
  else
#endif
    {
      /* Use the DMAC1 lookup table */

      table = g_dmac1_txchan;
      nentries = NDMAC1_TXCHANNELS;
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

static inline uint32_t sam_txcfg(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int pid;
  unsigned int pchan;
  bool isperiph;

  /* Set transfer (memory to peripheral) DMA channel configuration register */

  regval   = DMAC_CH_CFG_SOD;

  pid      = (dmach->flags & DMACH_FLAG_MEMPID_MASK) >>
              DMACH_FLAG_MEMPID_SHIFT;
  isperiph = ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0);
  pchan    = sam_source_channel(dmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << DMAC_CH_CFG_SRCPER_SHIFT);
  regval  |= ((pchan & 0x30) << (DMAC_CH_CFG_SRCPERMSB_SHIFT - 4));
  regval  |= (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ?
              DMAC_CH_CFG_SRCH2SEL : 0;

  pid      = (dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >>
              DMACH_FLAG_PERIPHPID_SHIFT;
  isperiph = ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0);
  pchan    = sam_sink_channel(dmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << DMAC_CH_CFG_DSTPER_SHIFT);
  regval  |= ((pchan & 0x30) << (DMAC_CH_CFG_DSTPERMSB_SHIFT - 4));
  regval  |= (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ?
              DMAC_CH_CFG_DSTH2SEL : 0;

  regval  |= sam_fifocfg(dmach);
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

static inline uint32_t sam_rxcfg(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int pid;
  unsigned int pchan;
  bool isperiph;

  /* Set received (peripheral to memory) DMA channel config */

  regval   = DMAC_CH_CFG_SOD;

  pid      = (dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >>
              DMACH_FLAG_PERIPHPID_SHIFT;
  isperiph = ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0);
  pchan    = sam_source_channel(dmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << DMAC_CH_CFG_SRCPER_SHIFT);
  regval  |= ((pchan & 0x30) << (DMAC_CH_CFG_SRCPERMSB_SHIFT - 4));
  regval  |= (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ?
              DMAC_CH_CFG_SRCH2SEL : 0;

  pid      = (dmach->flags & DMACH_FLAG_MEMPID_MASK) >>
              DMACH_FLAG_MEMPID_SHIFT;
  isperiph = ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0);
  pchan    = sam_sink_channel(dmach, pid, isperiph);

  regval  |= ((pchan & 0x0f) << DMAC_CH_CFG_DSTPER_SHIFT);
  regval  |= ((pchan & 0x30) << (DMAC_CH_CFG_DSTPERMSB_SHIFT - 4));
  regval  |= (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ?
              DMAC_CH_CFG_DSTH2SEL : 0;

  regval  |= sam_fifocfg(dmach);
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

static inline uint32_t sam_txctrlabits(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int ndx;
  unsigned int chunksize;

  DEBUGASSERT(dmach);

  /* Since this is a transmit, the source is described by the memory
   * selections. Set the source width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >>
         DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 4);
  regval = g_srcwidth[ndx];

  /* Set the source chunk size (memory chunk size) */

  chunksize = (dmach->flags & DMACH_FLAG_MEMCHUNKSIZE_MASK)
    >> DMACH_FLAG_MEMCHUNKSIZE_SHIFT;
  regval |= chunksize << DMAC_CH_CTRLA_SCSIZE_SHIFT;

  /* Since this is a transmit, the destination is described by the peripheral
   * selections. Set the destination width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >>
         DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 4);
  regval |= g_destwidth[ndx];

  /* Set the destination chunk size (peripheral chunk size) */

  chunksize = (dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE_MASK)
    >> DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT;
  regval |= chunksize << DMAC_CH_CTRLA_DCSIZE_SHIFT;

  return regval;
}

/****************************************************************************
 * Name: sam_maxtxtransfer
 *
 * Description:
 *  Maximum number of bytes that can be sent in on transfer
 *
 ****************************************************************************/

static size_t sam_maxtxtransfer(struct sam_dmach_s *dmach)
{
  unsigned int srcwidth;
  size_t maxtransfer;

  /* Get the maximum transfer size in bytes.  BTSIZE is "the number of
   * transfers to be performed, that is, for writes it refers to the number
   * of source width transfers to perform when DMAC is flow controller. For
   * Reads, BTSIZE refers to the number of transfers completed on the Source
   * Interface. ..."
   */

  srcwidth = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        maxtransfer = DMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 1: /* 16 bits, 2 bytes */
        maxtransfer = 2 * DMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 2: /* 32 bits 4 bytes */
        maxtransfer = 4 * DMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 3: /* 64 bits, 8 bytes */
        maxtransfer = 8 * DMAC_CH_CTRLA_BTSIZE_MAX;
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

static uint32_t sam_ntxtransfers(struct sam_dmach_s *dmach, uint32_t dmasize)
{
  unsigned int srcwidth;

  /* Adjust the source transfer size for the source chunk size (memory
   * chunk size).  BTSIZE is "the number of transfers to be performed, that
   * is, for writes it refers to the number of source width transfers
   * to perform when DMAC is flow controller. For Reads, BTSIZE refers to
   * the number of transfers completed on the Source Interface. ..."
   */

  srcwidth = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
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

static inline uint32_t sam_txctrla(struct sam_dmach_s *dmach,
                                   uint32_t ctrla, uint32_t dmasize)
{
  uint32_t ntransfers;

  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  ntransfers = sam_ntxtransfers(dmach, dmasize);

  DEBUGASSERT(ntransfers <= DMAC_CH_CTRLA_BTSIZE_MAX);
  return (ctrla & ~DMAC_CH_CTRLA_BTSIZE_MASK) |
         (ntransfers << DMAC_CH_CTRLA_BTSIZE_SHIFT);
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

static inline uint32_t sam_rxctrlabits(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int ndx;
  unsigned int chunksize;

  DEBUGASSERT(dmach);

  /* Since this is a receive, the source is described by the peripheral
   * selections. Set the source width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  DEBUGASSERT(ndx < 4);
  regval = g_srcwidth[ndx];

  /* Set the source chunk size (peripheral chunk size) */

  chunksize = (dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE_MASK)
    >> DMACH_FLAG_PERIPHCHUNKSIZE_SHIFT;
  regval |= chunksize << DMAC_CH_CTRLA_SCSIZE_SHIFT;

  /* Since this is a receive, the destination is described by the memory
   * selections. Set the destination width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  DEBUGASSERT(ndx < 4);
  regval |= g_destwidth[ndx];

  /* Set the destination chunk size (memory chunk size) */

  chunksize = (dmach->flags & DMACH_FLAG_MEMCHUNKSIZE_MASK)
    >> DMACH_FLAG_MEMCHUNKSIZE_SHIFT;
  regval |= chunksize << DMAC_CH_CTRLA_DCSIZE_SHIFT;

  return regval;
}

/****************************************************************************
 * Name: sam_maxrxtransfer
 *
 * Description:
 *  Maximum number of bytes that can be sent in on transfer
 *
 ****************************************************************************/

static size_t sam_maxrxtransfer(struct sam_dmach_s *dmach)
{
  unsigned int srcwidth;
  size_t maxtransfer;

  /* Get the maximum transfer size in bytes.  BTSIZE is "the number of
   * transfers to be performed, that is, for writes it refers to the number
   * of source width transfers to perform when DMAC is flow controller. For
   * Reads, BTSIZE refers to the number of transfers completed on the Source
   * Interface. ..."
   */

  srcwidth = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        maxtransfer = DMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 1: /* 16 bits, 2 bytes */
        maxtransfer = 2 * DMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 2: /* 32 bits, 4 bytes */
        maxtransfer = 4 * DMAC_CH_CTRLA_BTSIZE_MAX;
        break;

      case 3: /* 64 bits, 8 bytes */
        maxtransfer = 8 * DMAC_CH_CTRLA_BTSIZE_MAX;
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

static uint32_t sam_nrxtransfers(struct sam_dmach_s *dmach, uint32_t dmasize)
{
  unsigned int srcwidth;

  /* Adjust the source transfer size for the source chunk size (peripheral
   * chunk size).  BTSIZE is "the number of transfers to be performed, that
   * is, for writes it refers to the number of source width transfers
   * to perform when DMAC is flow controller. For Reads, BTSIZE refers to
   * the number of transfers completed on the Source Interface. ..."
   */

  srcwidth = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
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

static inline uint32_t sam_rxctrla(struct sam_dmach_s *dmach,
                                   uint32_t ctrla, uint32_t dmasize)
{
  uint32_t ntransfers;

  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  ntransfers = sam_nrxtransfers(dmach, dmasize);

  DEBUGASSERT(ntransfers <= DMAC_CH_CTRLA_BTSIZE_MAX);
  return (ctrla & ~DMAC_CH_CTRLA_BTSIZE_MASK) |
         (ntransfers << DMAC_CH_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam_txctrlb
 *
 * Description:
 *  Decode the flags to get the correct CTRLB register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_txctrlb(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int ahbif;

  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = DMAC_CH_CTRLB_BOTHDSCR | DMAC_CH_CTRLB_IEN;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from TX is memory to peripheral, but that is really
   * be determined by bits in the DMA flags.
   */

  /* Is the memory source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
    {
      /* Yes.. is the peripheral destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the source is memory.  Is the peripheral destination a
       * peripheral
       */

      if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_M2M;
        }
    }

  /* Source ABH layer */

  ahbif = (dmach->flags & DMACH_FLAG_MEMAHB_MASK) >> DMACH_FLAG_MEMAHB_SHIFT;
  regval |= (ahbif << DMAC_CH_CTRLB_SIF_SHIFT);

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_SRCINCR_FIXED;
    }

  /* Destination ABH layer */

  ahbif = (dmach->flags & DMACH_FLAG_PERIPHAHB_MASK) >>
           DMACH_FLAG_PERIPHAHB_SHIFT;
  regval |= (ahbif << DMAC_CH_CTRLB_DIF_SHIFT);

  /* Select destination address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_DSTINCR_FIXED;
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

static inline uint32_t sam_rxctrlb(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int ahbif;

  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = DMAC_CH_CTRLB_BOTHDSCR | DMAC_CH_CTRLB_IEN;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from RX is peripheral to memory, but that is really
   * be determined by bits in the DMA flags.
   */

  /* Is the peripheral source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Yes.. is the memory destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the peripheral source is memory.  Is the memory destination
       * a peripheral
       */

      if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_M2M;
        }
    }

  /* Source ABH layer */

  ahbif = (dmach->flags & DMACH_FLAG_PERIPHAHB_MASK) >>
           DMACH_FLAG_PERIPHAHB_SHIFT;
  regval |= (ahbif << DMAC_CH_CTRLB_SIF_SHIFT);

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_SRCINCR_FIXED;
    }

  /* Destination ABH layer */

  ahbif = (dmach->flags & DMACH_FLAG_MEMAHB_MASK) >> DMACH_FLAG_MEMAHB_SHIFT;
  regval |= (ahbif << DMAC_CH_CTRLB_DIF_SHIFT);

  /* Select address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_DSTINCR_FIXED;
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
sam_allocdesc(struct sam_dmach_s *dmach, struct dma_linklist_s *prev,
              uint32_t saddr, uint32_t daddr, uint32_t ctrla, uint32_t ctrlb)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *desc = NULL;
  int i;
  int ret;

  /* Sanity check -- saddr == 0 is the indication that the link is unused.
   * Obviously setting it to zero would break that usage.
   */

#ifdef CONFIG_DEBUG_FEATURES
  if (saddr != 0)
#endif
    {
      /* Table a descriptor table semaphore count.  When we get one, then
       * there is at least one free descriptor in the table and it is ours.
       */

      ret = sam_takedsem(dmac);
      if (ret < 0)
        {
          return NULL;
        }

      /* Examine each link list entry to find an available one -- i.e., one
       * with saddr == 0.  That saddr field is set to zero by the DMA
       * transfer complete interrupt handler.  The following should be safe
       * because that is an atomic operation.
       */

      ret = sam_takechsem(dmac);
      if (ret < 0)
        {
          sam_givedsem(dmac);
          return NULL;
        }

      for (i = 0; i < CONFIG_SAMA5_NLLDESC; i++)
        {
          if (dmac->desc[i].saddr == 0)
            {
              /* We have it.  Initialize the new link list entry */

              desc        = &dmac->desc[i];
              desc->saddr = saddr;  /* Source address */
              desc->daddr = daddr;  /* Destination address */
              desc->ctrla = ctrla;  /* Control A value */
              desc->ctrlb = ctrlb;  /* Control B value */
              desc->dscr  = 0;      /* Next descriptor address */

              /* And then hook it at the tail of the link list */

              if (!prev)
                {
                  /* There is no previous link.  This is the new head of
                   * the list
                   */

                  DEBUGASSERT(dmach->llhead == NULL &&
                              dmach->lltail == NULL);
                  dmach->llhead = desc;
                }
              else
                {
                  DEBUGASSERT(dmach->llhead != NULL &&
                              dmach->lltail == prev);

                  /* When the second link is added to the list, that is the
                   * cue that we are going to do the link list transfer.
                   *
                   * Enable the source and destination descriptor in the link
                   * list entry just before this one.  We assume that both
                   * source and destination buffers are non-continuous, but
                   * this should work even if that is not the case.
                   */

                  prev->ctrlb &= ~DMAC_CH_CTRLB_BOTHDSCR;

                  /* Link the previous tail to the new tail.
                   * REVISIT:  This assumes that the next description is
                   * fetched via AHB IF0.
                   */

                  prev->dscr = (uint32_t)sam_physramaddr((uintptr_t)desc);
                }

              /* In any event, this is the new tail of the list.  The source
               * and destination descriptors must be disabled for the last
               * entry in the link list.
               */

              desc->ctrlb  |= DMAC_CH_CTRLB_BOTHDSCR;
              dmach->lltail = desc;

              /* Assume that we will be doing multiple buffer transfers and
               * that hardware will be accessing the descriptor via DMA.
               */

              up_clean_dcache((uintptr_t)desc,
                              (uintptr_t)desc +
                              sizeof(struct dma_linklist_s));
              break;
            }
        }

      /* Because we hold a count from the counting semaphore, the above
       * search loop should always be successful.
       */

      sam_givechsem(dmac);
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

static void sam_freelinklist(struct sam_dmach_s *dmach)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *desc;
  uintptr_t paddr;

  /* Get the head of the link list and detach the link list from the DMA
   * channel
   */

  desc             = dmach->llhead;
  dmach->llhead    = NULL;
  dmach->lltail    = NULL;

  while (desc != NULL)
    {
      /* Valid, in-use descriptors never have saddr == 0 */

      DEBUGASSERT(desc->saddr != 0);

      /* Get the physical address of the next descriptor in the list */

      paddr = desc->dscr;

      /* Free the descriptor by simply nullifying it and bumping up the
       * semaphore count.
       */

      memset(desc, 0, sizeof(struct dma_linklist_s));
      sam_givedsem(dmac);

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

static int sam_txbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                        uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (dmach->llhead)
    {
      regval = dmach->llhead->ctrla;
      ctrlb  = dmach->llhead->ctrlb;
    }
  else
    {
      regval = sam_txctrlabits(dmach);
      ctrlb  = sam_txctrlb(dmach);
    }

  ctrla = sam_txctrla(dmach, regval, nbytes);

  /* Add the new link list entry */

  if (!sam_allocdesc(dmach, dmach->lltail, maddr, paddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the transmit CFG register setting (it won't be used until
   * the DMA is started).
   */

  dmach->cfg = sam_txcfg(dmach);
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

static int sam_rxbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                        uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (dmach->llhead)
    {
      regval = dmach->llhead->ctrla;
      ctrlb  = dmach->llhead->ctrlb;
    }
  else
    {
      regval = sam_rxctrlabits(dmach);
      ctrlb  = sam_rxctrlb(dmach);
    }

  ctrla = sam_rxctrla(dmach, regval, nbytes);

  /* Add the new link list entry */

  if (!sam_allocdesc(dmach, dmach->lltail, paddr, maddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the receive CFG register setting (it won't be used until
   * the DMA is started).
   */

  dmach->cfg = sam_rxcfg(dmach);
  return OK;
}

/****************************************************************************
 * Name: sam_single
 *
 * Description:
 *   Start a single buffer DMA.
 *
 ****************************************************************************/

static inline int sam_single(struct sam_dmach_s *dmach)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *llhead = dmach->llhead;

  /* Clear any pending interrupts from any previous DMAC transfer by reading
   * the interrupt status register.
   */

  sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);

  /* Write the starting source address in the SADDR register */

  DEBUGASSERT(llhead != NULL && llhead->saddr != 0);
  sam_putdmach(dmach, llhead->saddr, SAM_DMAC_CH_SADDR_OFFSET);

  /* Write the starting destination address in the DADDR register */

  sam_putdmach(dmach, llhead->daddr, SAM_DMAC_CH_DADDR_OFFSET);

  /* Clear the next descriptor address */

  sam_putdmach(dmach, 0, SAM_DMAC_CH_DSCR_OFFSET);

  /* Set up the CTRLA register */

  sam_putdmach(dmach, llhead->ctrla, SAM_DMAC_CH_CTRLA_OFFSET);

  /* Set up the CTRLB register */

  sam_putdmach(dmach, llhead->ctrlb, SAM_DMAC_CH_CTRLB_OFFSET);

  /* Both the DST and SRC DSCR bits should be '1' in CTRLB */

  DEBUGASSERT((llhead->ctrlb & DMAC_CH_CTRLB_BOTHDSCR) ==
              DMAC_CH_CTRLB_BOTHDSCR);

  /* Set up the CFG register */

  sam_putdmach(dmach, dmach->cfg, SAM_DMAC_CH_CFG_OFFSET);

  /* Enable the channel by writing a 1 to the CHER enable bit */

  sam_putdmac(dmac, DMAC_CHER_ENA(dmach->chan), SAM_DMAC_CHER_OFFSET);

  /* The DMA has been started. Once the transfer completes, hardware sets
   * the interrupts and disables the channel.  We will receive buffer
   * complete and transfer complete interrupts.
   *
   * Enable error, buffer complete and transfer complete interrupts.
   * (Since there is only a single buffer, we don't need the buffer
   * complete interrupt).
   */

  sam_putdmac(dmac, DMAC_EBC_CBTCINTS(dmach->chan), SAM_DMAC_EBCIER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_multiple
 *
 * Description:
 *   Start a multiple buffer DMA.
 *
 ****************************************************************************/

static inline int sam_multiple(struct sam_dmach_s *dmach)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *llhead = dmach->llhead;

  DEBUGASSERT(llhead != NULL && llhead->saddr != 0);

  /* Check the first and last CTRLB values */

  DEBUGASSERT((llhead->ctrlb & DMAC_CH_CTRLB_BOTHDSCR) == 0);
  DEBUGASSERT((dmach->lltail->ctrlb & DMAC_CH_CTRLB_BOTHDSCR) ==
              DMAC_CH_CTRLB_BOTHDSCR);

  /* Clear any pending interrupts from any previous DMAC transfer by reading
   * the status register
   */

  sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);

  /* Set up the initial CTRLA register */

  sam_putdmach(dmach, llhead->ctrla, SAM_DMAC_CH_CTRLA_OFFSET);

  /* Set up the CTRLB register (will enable descriptors) */

  sam_putdmach(dmach, llhead->ctrlb, SAM_DMAC_CH_CTRLB_OFFSET);

  /* Write the channel configuration information into the CFG register */

  sam_putdmach(dmach, dmach->cfg, SAM_DMAC_CH_CFG_OFFSET);

  /* Program the DSCR register with the pointer to the firstlink list
   * entry.
   */

  sam_putdmach(dmach, (uint32_t)llhead, SAM_DMAC_CH_DSCR_OFFSET);

  /* Finally, enable the channel by writing a 1 to the CHER enable */

  sam_putdmac(dmac, DMAC_CHER_ENA(dmach->chan), SAM_DMAC_CHER_OFFSET);

  /* As each buffer of data is transferred, the CTRLA register is written
   * back into the link list entry.  The CTRLA contains updated BTSIZE and
   * DONE bits.  Additionally, the CTRLA DONE bit is asserted when the
   * buffer transfer has completed.
   *
   * The DMAC transfer continues until the CTRLB register disables the
   * descriptor (DSCR bits) registers at the final buffer transfer.
   *
   * Enable error, buffer complete and transfer complete interrupts.  We
   * don't really need the buffer complete interrupts, but we will take them
   * just to handle stall conditions.
   */

  sam_putdmac(dmac, DMAC_EBC_CHANINTS(dmach->chan), SAM_DMAC_EBCIER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void sam_dmaterminate(struct sam_dmach_s *dmach, int result)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);

  /* Disable all channel interrupts */

  sam_putdmac(dmac, DMAC_EBC_CHANINTS(dmach->chan), SAM_DMAC_EBCIDR_OFFSET);

  /* Disable the channel by writing one to the write-only channel disable
   * register.
   */

  sam_putdmac(dmac, DMAC_CHDR_DIS(dmach->chan), SAM_DMAC_CHDR_OFFSET);

  /* Free the linklist */

  sam_freelinklist(dmach);

  /* If this was an RX DMA (peripheral-to-memory), then invalidate the cache
   * to force reloads from memory.
   */

  if (dmach->rx)
    {
      up_invalidate_dcache(dmach->rxaddr, dmach->rxaddr + dmach->rxsize);
    }

  /* Perform the DMA complete callback */

  if (dmach->callback)
    {
      dmach->callback((DMA_HANDLE)dmach, dmach->arg, result);
    }

  dmach->callback = NULL;
  dmach->arg      = NULL;
}

/****************************************************************************
 * Name: sam_dmac_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int sam_dmac_interrupt(int irq, void *context, void *arg)
{
  struct sam_dmac_s *dmac = (struct sam_dmac_s *)arg;
  struct sam_dmach_s *dmach;
  unsigned int chndx;
  uint32_t regval;

  DEBUGASSERT(dmac != NULL);

  /* Get the DMAC status register value.  Ignore all masked interrupt
   * status bits.
   */

  regval = sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET) &
           sam_getdmac(dmac, SAM_DMAC_EBCIMR_OFFSET);

  /* Check if the any transfer has completed or any errors have occurred. */

  if (regval & DMAC_EBC_ALLCHANINTS)
    {
      /* Yes.. Check each bit  to see which channel has interrupted */

      for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
        {
          /* Are any interrupts pending for this channel? */

          if ((regval & DMAC_EBC_CHANINTS(chndx)) != 0)
            {
              dmach = &dmac->dmach[chndx];

              /* Yes.. Did an error occur? */

              if ((regval & DMAC_EBC_ERR(chndx)) != 0)
                {
                  /* Yes... Terminate the transfer with an error? */

                  dmaerr("ERROR: DMA failed: %08" PRIx32 "\n", regval);
                  sam_dmaterminate(dmach, -EIO);
                }

              /* Is the transfer complete? */

              else if ((regval & DMAC_EBC_CBTC(chndx)) != 0)
                {
                  /* Yes.. Terminate the transfer with success */

                  sam_dmaterminate(dmach, OK);
                }

              /* Otherwise, this must be a Buffer Transfer Complete (BTC)
               * interrupt as part of a multiple buffer transfer.
               */

              else /* if ((regval & DMAC_EBC_BTC(chndx)) != 0) */
                {
                  /* Write the KEEPON field to clear the STALL states */

                  sam_putdmac(dmac, DMAC_CHER_KEEP(dmach->chan),
                              SAM_DMAC_CHER_OFFSET);
                }
            }
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

void sam_dmainitialize(struct sam_dmac_s *dmac)
{
  /* Disable all DMA interrupts */

  sam_putdmac(dmac, DMAC_EBC_ALLINTS, SAM_DMAC_EBCIDR_OFFSET);

  /* Disable all DMA channels */

  sam_putdmac(dmac, DMAC_CHDR_DIS_ALL, SAM_DMAC_CHDR_OFFSET);

  /* Enable the DMA controller */

  sam_putdmac(dmac, DMAC_EN_ENABLE, SAM_DMAC_EN_OFFSET);

  /* Initialize semaphores */

  nxsem_init(&dmac->chsem, 0, 1);
  nxsem_init(&dmac->dsem, 0, SAM_NDMACHAN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
#ifdef CONFIG_SAMA5_DMAC0
  dmainfo("Initialize DMAC0\n");

  /* Enable peripheral clock */

  sam_dmac0_enableclk();

  /* Attach DMA interrupt vector */

  irq_attach(SAM_IRQ_DMAC0, sam_dmac_interrupt, &g_dmac0);

  /* Initialize the controller */

  sam_dmainitialize(&g_dmac0);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMAC0);
#endif

#ifdef CONFIG_SAMA5_DMAC1
  dmainfo("Initialize DMAC1\n");

  /* Enable peripheral clock */

  sam_dmac1_enableclk();

  /* Attach DMA interrupt vector */

  irq_attach(SAM_IRQ_DMAC1, sam_dmac_interrupt, &g_dmac1);

  /* Initialize the controller */

  sam_dmainitialize(&g_dmac1);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMAC1);
#endif
}

/****************************************************************************
 * Name: sam_dmachannel
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel then
 *   gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  However, the interface
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
  struct sam_dmac_s *dmac;
  struct sam_dmach_s *dmach;
  unsigned int chndx;
  int ret;

  /* Pick the DMA controller */

#ifdef CONFIG_SAMA5_DMAC0
  if (dmacno == 0)
    {
      dmac = &g_dmac0;
    }
  else
#endif

#ifdef CONFIG_SAMA5_DMAC1
  if (dmacno == 1)
    {
      dmac = &g_dmac1;
    }
  else
#endif

    {
      dmaerr("ERROR: Bad DMAC number: %d\n", dmacno);
      DEBUGPANIC();
      return (DMA_HANDLE)NULL;
    }

  /* Search for an available DMA channel with at least the requested FIFO
   * size.
   */

  dmach = NULL;
  ret = sam_takechsem(dmac);
  if (ret < 0)
    {
      return NULL;
    }

  for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
    {
      struct sam_dmach_s *candidate = &dmac->dmach[chndx];
      if (!candidate->inuse)
        {
          dmach         = candidate;
          dmach->inuse  = true;

          /* Read the status register to clear any pending interrupts on the
           * channel
           */

          sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);

          /* Disable the channel by writing one to the write-only channel
           * disable register
           */

          sam_putdmac(dmac, DMAC_CHDR_DIS(chndx), SAM_DMAC_CHDR_OFFSET);

          /* Set the DMA channel flags. */

          dmach->flags = chflags;
          break;
        }
    }

  sam_givechsem(dmac);

  /* Show the result of the allocation */

  if (dmach)
    {
      dmainfo("DMAC%d CH%d: chflags: %08x returning dmach: %p\n",
              (int)dmacno, dmach->chan, (int)chflags, dmach);
    }
  else
    {
      dmaerr("ERROR: Failed allocate DMAC%d channel\n", (int)dmacno);
    }

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: sam_dmaconfig
 *
 * Description:
 *   There are two channel usage models:  (1) The channel is allocated and
 *   configured in one step.  This is the typical case where a DMA channel
 *   performs a constant role.  The alternative is (2) where the DMA channel
 *   is reconfigured on the fly. In this case, the chflags provided to
 *   sam_dmachannel are not used and sam_dmaconfig() is called before each
 *   DMA to configure the DMA channel appropriately.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t chflags)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;

  /* Set the new DMA channel flags. */

  dmach->flags = chflags;

#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
  dmainfo("DMAC%d CH%d: chflags: %08x\n",
          dmach->dmac, dmach->chan, (int)chflags);
#elif defined(CONFIG_SAMA5_DMAC0)
  dmainfo("DMAC0 CH%d: chflags: %08x\n",
          dmach->chan, (int)chflags);
#else
  dmainfo("DMAC1 CH%d: chflags: %08x\n",
          dmach->chan, (int)chflags);
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
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT((dmach != NULL) && (dmach->inuse));

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->flags = 0;
  dmach->inuse = false;                   /* No longer in use */
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
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  size_t maxtransfer;
  size_t remaining;
  int ret = OK;

  dmainfo("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);
  dmainfo("llhead: %p lltail: %p\n", dmach->llhead, dmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtxtransfer(dmach);
  remaining   = nbytes;

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_txbuffer(dmach, paddr, maddr, maxtransfer);
      if (ret == OK)
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate
           * to do so).
           */

          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_txbuffer(dmach, paddr, maddr, remaining);
    }

  /* Save an indication so that the DMA interrupt completion logic will know
   * that this was not an RX transfer.
   */

  dmach->rx = false;

  /* Clean caches associated with the DMA memory */

  up_clean_dcache(maddr, maddr + nbytes);
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
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  size_t maxtransfer;
  size_t remaining;
  int ret = OK;

  dmainfo("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);
  dmainfo("llhead: %p lltail: %p\n", dmach->llhead, dmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxrxtransfer(dmach);
  remaining   = nbytes;

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_rxbuffer(dmach, paddr, maddr, maxtransfer);
      if (ret == OK)
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate
           * to do so).
           */

          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_rxbuffer(dmach, paddr, maddr, remaining);
    }

  /* Save an indication so that the DMA interrupt completion logic will know
   * that this was an RX transfer and will invalidate the cache.
   */

  dmach->rx     = true;
  dmach->rxaddr = maddr;
  dmach->rxsize = (dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0 ?
                   nbytes : sizeof(uint32_t);

  /* Clean caches associated with the DMA memory */

  up_clean_dcache(maddr, maddr + nbytes);
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
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  int ret = -EINVAL;

  dmainfo("dmach: %p callback: %p arg: %p\n", dmach, callback, arg);
  DEBUGASSERT(dmach != NULL);

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * link list).
   */

  if (dmach->llhead)
    {
      /* Save the callback info.  This will be invoked whent the DMA
       * completes
       */

      dmach->callback = callback;
      dmach->arg      = arg;

      /* Is this a single block transfer?  Or a multiple block transfer? */

      if (dmach->llhead == dmach->lltail)
        {
          ret = sam_single(dmach);
        }
      else
        {
          ret = sam_multiple(dmach);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam_dmastop() is called, the DMA channel is
 *   reset and sam_dmarx/txsetup() must be called before sam_dmastart() can
 *   be called again
 *
 ****************************************************************************/

void sam_dmastop(DMA_HANDLE handle)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  irqstate_t flags;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL);

  flags = enter_critical_section();
  sam_dmaterminate(dmach, -EINTR);
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

#ifdef CONFIG_DEBUG_DMA
void sam_dmasample(DMA_HANDLE handle, struct sam_dmaregs_s *regs)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  struct sam_dmac_s *dmac = sam_controller(dmach);
  irqstate_t flags;

  /* Sample global registers.  NOTE: reading EBCISR clears interrupts, but
   * that should be okay IF interrupts are enabled when this function is
   * called.  But there is a race condition where this instrumentation could
   * cause lost interrupts.
   */

  flags        = enter_critical_section();
  regs->gcfg   = sam_getdmac(dmac, SAM_DMAC_GCFG_OFFSET);
  regs->en     = sam_getdmac(dmac, SAM_DMAC_EN_OFFSET);
  regs->sreq   = sam_getdmac(dmac, SAM_DMAC_SREQ_OFFSET);
  regs->creq   = sam_getdmac(dmac, SAM_DMAC_CREQ_OFFSET);
  regs->last   = sam_getdmac(dmac, SAM_DMAC_LAST_OFFSET);
  regs->ebcimr = sam_getdmac(dmac, SAM_DMAC_EBCIMR_OFFSET);
  regs->ebcisr = sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);
  regs->chsr   = sam_getdmac(dmac, SAM_DMAC_CHSR_OFFSET);
  regs->wpmr   = sam_getdmac(dmac, SAM_DMAC_WPMR_OFFSET);
  regs->wpsr   = sam_getdmac(dmac, SAM_DMAC_WPSR_OFFSET);

  /* Sample channel registers */

  regs->saddr  = sam_getdmach(dmach, SAM_DMAC_CH_SADDR_OFFSET);
  regs->daddr  = sam_getdmach(dmach, SAM_DMAC_CH_DADDR_OFFSET);
  regs->dscr   = sam_getdmach(dmach, SAM_DMAC_CH_DSCR_OFFSET);
  regs->ctrla  = sam_getdmach(dmach, SAM_DMAC_CH_CTRLA_OFFSET);
  regs->ctrlb  = sam_getdmach(dmach, SAM_DMAC_CH_CTRLB_OFFSET);
  regs->cfg    = sam_getdmach(dmach, SAM_DMAC_CH_CFG_OFFSET);
  regs->spip   = sam_getdmach(dmach, SAM_DMAC_CH_SPIP_OFFSET);
  regs->dpip   = sam_getdmach(dmach, SAM_DMAC_CH_DPIP_OFFSET);
  leave_critical_section(flags);
}
#endif /* CONFIG_DEBUG_DMA */

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

#ifdef CONFIG_DEBUG_DMA
void sam_dmadump(DMA_HANDLE handle, const struct sam_dmaregs_s *regs,
                 const char *msg)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  struct sam_dmac_s *dmac = sam_controller(dmach);

  dmainfo("%s\n", msg);
  dmainfo("  DMA Global Registers:\n");
  dmainfo("      GCFG[%08x]: %08x\n",
          dmac->base + SAM_DMAC_GCFG_OFFSET, regs->gcfg);
  dmainfo("        EN[%08x]: %08x\n",
          dmac->base + SAM_DMAC_EN_OFFSET, regs->en);
  dmainfo("      SREQ[%08x]: %08x\n",
          dmac->base + SAM_DMAC_SREQ_OFFSET, regs->sreq);
  dmainfo("      CREQ[%08x]: %08x\n",
          dmac->base + SAM_DMAC_CREQ_OFFSET, regs->creq);
  dmainfo("      LAST[%08x]: %08x\n",
          dmac->base + SAM_DMAC_LAST_OFFSET, regs->last);
  dmainfo("    EBCIMR[%08x]: %08x\n",
          dmac->base + SAM_DMAC_EBCIMR_OFFSET, regs->ebcimr);
  dmainfo("    EBCISR[%08x]: %08x\n",
          dmac->base + SAM_DMAC_EBCISR_OFFSET, regs->ebcisr);
  dmainfo("      CHSR[%08x]: %08x\n",
          dmac->base + SAM_DMAC_CHSR_OFFSET, regs->chsr);
  dmainfo("      WPMR[%08x]: %08x\n",
          dmac->base + SAM_DMAC_WPMR_OFFSET, regs->wpmr);
  dmainfo("      WPSR[%08x]: %08x\n",
          dmac->base + SAM_DMAC_WPSR_OFFSET, regs->wpsr);
  dmainfo("  DMA Channel Registers:\n");
  dmainfo("     SADDR[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_SADDR_OFFSET, regs->saddr);
  dmainfo("     DADDR[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_DADDR_OFFSET, regs->daddr);
  dmainfo("      DSCR[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_DSCR_OFFSET, regs->dscr);
  dmainfo("     CTRLA[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_CTRLA_OFFSET, regs->ctrla);
  dmainfo("     CTRLB[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_CTRLB_OFFSET, regs->ctrlb);
  dmainfo("       CFG[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_CFG_OFFSET, regs->cfg);
  dmainfo("      SPIP[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_SPIP_OFFSET, regs->spip);
  dmainfo("      DPIP[%08x]: %08x\n",
          dmach->base + SAM_DMAC_CH_DPIP_OFFSET, regs->dpip);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_SAMA5_DMAC0 || CONFIG_SAMA5_DMAC1 */
