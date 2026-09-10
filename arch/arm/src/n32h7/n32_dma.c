/****************************************************************************
 * arch/arm/src/n32h7/n32_dma.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/n32h7/chip.h>

#include "arm_internal.h"
#include "sched/sched.h"

#include "n32_dma.h"
#include "hardware/n32h7_dma.h"
#include "hardware/n32h7_dmamux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMAMUX_NUM      2
#define DMA_CONTROLLERS 4

#ifdef CONFIG_N32H7_MDMA
#  define MDMA_NCHAN     16
#else
#  define MDMA_NCHAN     0
#endif
#ifdef CONFIG_N32H7_DMA1
#  define DMA1_NCHAN  8
#else
#  define DMA1_NCHAN  0
#endif
#ifdef CONFIG_N32H7_DMA2
#  define DMA2_NCHAN  8
#else
#  define DMA2_NCHAN  0
#endif
#ifdef CONFIG_N32H7_DMA3
#  define DMA3_NCHAN  8
#else
#  define DMA3_NCHAN  0
#endif

#define MDMA_FIRST       (0)
#define MDMA_LAST        (MDMA_FIRST+MDMA_NCHAN)
#define DMA1_FIRST       (MDMA_LAST)
#define DMA1_LAST        (DMA1_FIRST+DMA1_NCHAN)
#define DMA2_FIRST       (DMA1_LAST)
#define DMA2_LAST        (DMA2_FIRST+DMA2_NCHAN)
#define DMA3_FIRST       (DMA2_LAST)
#define DMA3_LAST        (DMA3_FIRST+DMA3_NCHAN)

/* All available DMA channels (chanels from standard DMA and
 * channels from DMA3 and MDMA)
 */

#define DMA_NCHANNELS    (DMA1_NCHAN+DMA2_NCHAN+MDMA_NCHAN+DMA3_NCHAN)

/* Default DMA, MDMA and DMA3 priorities */

#ifndef CONFIG_DMA_PRI
#  define CONFIG_DMA_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_MDMA_PRI
#  define CONFIG_MDMA_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure described one DMAMUX device */

struct n32_dmamux_s
{
  uint8_t  id;                  /* DMAMUX id */
  uint8_t  nchan;               /* DMAMUX channels */
  uint32_t base;                /* DMAMUX base address */
};

typedef struct n32_dmamux_s *DMA_MUX;

/* This structure describes one DMA controller */

struct n32_dma_s
{
  uint8_t  first;                /* Offset in n32_dmach_s array */
  uint8_t  nchan;                /* Number of channels */
  uint8_t  dmamux_offset;        /* DMAMUX channel offset */
  uint32_t base;                 /* Base address */
  DMA_MUX  dmamux;               /* DMAMUX associated with controller */
};

/* This structure describes one DMA channel (DMA123 or MDMA) */

struct n32_dmach_s
{
  bool           used;      /* Channel in use */
  uint8_t        ctrl:3;    /* DMA controller */
  uint8_t        chan:5;    /* DMA chanel/channel channel id */
  uint8_t        irq;       /* DMA chanel IRQ number */
  uint32_t       base;      /* DMA register channel base address */
  dma_callback_t callback;  /* Callback invoked when the DMA completes */
  void          *arg;       /* Argument passed to callback function */
};

typedef struct n32_dmach_s *DMA_CHANNEL;

/* DMA operations */

struct n32_dma_ops_s
{
  /* Start the DMA transfer */

  void (*dma_disable)(DMA_CHANNEL dmachan);

  /* DMA interrupt */

  int (*dma_interrupt)(int irq, void *context, void *arg);

  /* Setup the DMA */

  void (*dma_setup)(DMA_HANDLE handle, n32_dmacfg_t *cfg);

  /* Start the DMA */

  void (*dma_start)(DMA_HANDLE handle, dma_callback_t callback,
                    void *arg);

  /* Read remaining DMA bytes */

  size_t (*dma_residual)(DMA_HANDLE handle);

  /* Check the DMA configuration  */

  bool (*dma_capable)(n32_dmacfg_t *cfg);

  /* Dump the DMA registers */

  void (*dma_dump)(DMA_HANDLE handle, const char *msg);
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_N32H7_MDMA
static void n32_mdma_disable(DMA_CHANNEL dmachan);
static int n32_mdma_interrupt(int irq, void *context, void *arg);
static void n32_mdma_setup(DMA_HANDLE handle, n32_dmacfg_t *cfg);
static void n32_mdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg);
static size_t n32_mdma_residual(DMA_HANDLE handle);
#ifdef CONFIG_N32H7_DMACAPABLE
static bool n32_mdma_capable(n32_dmacfg_t *cfg);
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
static void n32_mdma_dump(DMA_HANDLE handle, const char *msg);
#endif
#endif

#if defined(CONFIG_N32H7_DMA1) || defined(CONFIG_N32H7_DMA2) || defined(CONFIG_N32H7_DMA3)
static void n32_sdma_disable(DMA_CHANNEL dmachan);
static int n32_sdma_interrupt(int irq, void *context, void *arg);
static void n32_sdma_setup(DMA_HANDLE handle, n32_dmacfg_t *cfg);
static void n32_sdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg);
static size_t n32_sdma_residual(DMA_HANDLE handle);
#ifdef CONFIG_N32H7_DMACAPABLE
static bool n32_sdma_capable(n32_dmacfg_t *cfg);
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
static void n32_sdma_dump(DMA_HANDLE handle, const char *msg);
#endif
#endif

static uint32_t dmachan_getbase(DMA_CHANNEL dmachan);
static uint32_t dmabase_getreg(DMA_CHANNEL dmachan, uint32_t offset);
static void dmabase_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value);
static uint32_t dmachan_getreg(DMA_CHANNEL dmachan, uint32_t offset);
static void dmachan_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value);
static void dmamux_putreg(DMA_MUX dmamux, uint32_t offset, uint32_t value);
#ifdef CONFIG_DEBUG_DMA_INFO
static uint32_t dmamux_getreg(DMA_MUX dmamux, uint32_t offset);
static void n32_dmamux_dump(DMA_MUX dmamux, uint8_t chan);
#endif
static DMA_CHANNEL n32_dma_channel_get(uint8_t channel,
                                         uint8_t controller);
static void n32_gdma_limits_get(uint8_t controller, uint8_t *first,
                                  uint8_t *last);

static inline void dmachan_modifyreg32(DMA_CHANNEL dmachan,
                                       uint32_t offset, uint32_t clrbits,
                                       uint32_t setbits)
{
  modifyreg32(dmachan->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Operations specific to DMA controller */

struct n32_dma_ops_s g_dma_ops[DMA_CONTROLLERS] =
{
#ifdef CONFIG_N32H7_MDMA
  /* 0 - MDMA */

    {
      .dma_disable   = n32_mdma_disable,
      .dma_interrupt = n32_mdma_interrupt,
      .dma_setup     = n32_mdma_setup,
      .dma_start     = n32_mdma_start,
      .dma_residual  = n32_mdma_residual,
#ifdef CONFIG_N32H7_DMACAPABLE
      .dma_capable   = n32_mdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = n32_mdma_dump,
#endif
    },
#else
    {
      NULL
    },
#endif

#ifdef CONFIG_N32H7_DMA1
  /* 1 - DMA1 */

    {
      .dma_disable   = n32_sdma_disable,
      .dma_interrupt = n32_sdma_interrupt,
      .dma_setup     = n32_sdma_setup,
      .dma_start     = n32_sdma_start,
      .dma_residual  = n32_sdma_residual,
#ifdef CONFIG_N32H7_DMACAPABLE
      .dma_capable   = n32_sdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = n32_sdma_dump,
#endif
    },
#else
    {
      NULL
    },
#endif

#ifdef CONFIG_N32H7_DMA2
  /* 2 - DMA2 */

    {
      .dma_disable   = n32_sdma_disable,
      .dma_interrupt = n32_sdma_interrupt,
      .dma_setup     = n32_sdma_setup,
      .dma_start     = n32_sdma_start,
      .dma_residual  = n32_sdma_residual,
#ifdef CONFIG_N32H7_DMACAPABLE
      .dma_capable   = n32_sdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = n32_sdma_dump,
#endif
    },
#else
    {
      NULL
    },
#endif

#ifdef CONFIG_N32H7_DMA3
  /* 3 - DMA3 */

    {
      .dma_disable   = n32_sdma_disable,
      .dma_interrupt = n32_sdma_interrupt,
      .dma_setup     = n32_sdma_setup,
      .dma_start     = n32_sdma_start,
      .dma_residual  = n32_sdma_residual,
#ifdef CONFIG_N32H7_DMACAPABLE
      .dma_capable   = n32_sdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = n32_sdma_dump,
#endif
    },
#else
    {
      NULL
    },
#endif
};

/* This array describes the state of DMAMUX controller */

struct n32_dmamux_s g_dmamux[DMAMUX_NUM] =
{
    {
      .id      = 1,
      .nchan   = 24,              /* 0-7 - DMA1, 8-15 - DMA2, 16-23 - DMA3 */
      .base    = N32_DMAMUX1_BASE
    },

    {
      .id      = 2,
      .nchan   = 16,              /* 0-15 - MDMA */
      .base    = N32_DMAMUX2_BASE
    }
};

/* This array describes the state of each controller */

struct n32_dma_s g_dma[DMA_NCHANNELS] =
{
  /* 0 - MDMA */

    {
      .base   = N32_MDMA_BASE,
      .first  = MDMA_FIRST,
      .nchan  = MDMA_NCHAN,
      .dmamux = &g_dmamux[DMAMUX2], /* DMAMUX2 channels 0-15 */
      .dmamux_offset = 0
    },

  /* 1 - DMA1 */

    {
      .base   = N32_DMA1_BASE,
      .first  = DMA1_FIRST,
      .nchan  = DMA1_NCHAN,
      .dmamux = &g_dmamux[DMAMUX1], /* DMAMUX1 channels 0-7 */
      .dmamux_offset = 0
    },

  /* 2 - DMA2 */

    {
      .base   = N32_DMA2_BASE,
      .first  = DMA2_FIRST,
      .nchan  = DMA2_NCHAN,
      .dmamux = &g_dmamux[DMAMUX1], /* DMAMUX1 channels 8-15 */
      .dmamux_offset = 8
    },

  /* 3 - DMA3 */

    {
      .base   = N32_DMA3_BASE,
      .first  = DMA3_FIRST,
      .nchan  = DMA3_NCHAN,
      .dmamux = &g_dmamux[DMAMUX1], /* DMAMUX1 channels 16-23 */
      .dmamux_offset = 16
    }
};

/* This array describes the state of each DMA channel.
 * Note that we keep here standard DMA chanels, BDMA channels and MDMA
 * channels.
 */

static struct n32_dmach_s g_dmach[DMA_NCHANNELS] =
{
#ifdef CONFIG_N32H7_MDMA
  /* MDMA */

    {
      .ctrl     = MDMA,
      .chan     = 0,
      .irq      = N32_IRQ_MDMA_CHANNEL0,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(0),
    },

    {
      .ctrl     = MDMA,
      .chan     = 1,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(1),
    },

    {
      .ctrl     = MDMA,
      .chan     = 2,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(2),
    },

    {
      .ctrl     = MDMA,
      .chan     = 3,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(3),
    },

    {
      .ctrl     = MDMA,
      .chan     = 4,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(4),
    },

    {
      .ctrl     = MDMA,
      .chan     = 5,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(5),
    },

    {
      .ctrl     = MDMA,
      .chan     = 6,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(6),
    },

    {
      .ctrl     = MDMA,
      .chan     = 7,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(7),
    },

    {
      .ctrl     = MDMA,
      .chan     = 8,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(8),
    },

    {
      .ctrl     = MDMA,
      .chan     = 9,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(9),
    },

    {
      .ctrl     = MDMA,
      .chan     = 10,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(10),
    },

    {
      .ctrl     = MDMA,
      .chan     = 11,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(11),
    },

    {
      .ctrl     = MDMA,
      .chan     = 12,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(12),
    },

    {
      .ctrl     = MDMA,
      .chan     = 13,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(13),
    },

    {
      .ctrl     = MDMA,
      .chan     = 14,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(15),
    },

    {
      .ctrl     = MDMA,
      .chan     = 15,
      .irq      = N32_IRQ_MDMA,
      .base     = N32_MDMA_BASE + N32_MDMA_OFFSET(15),
    },
#endif

#ifdef CONFIG_N32H7_DMA1
  /* DMA1 */

    {
      .ctrl     = DMA1,
      .chan     = 0,
      .irq      = N32_IRQ_DMA1_CHANNEL0,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(0),
    },

    {
      .ctrl     = DMA1,
      .chan     = 1,
      .irq      = N32_IRQ_DMA1_CHANNEL1,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(1),
    },

    {
      .ctrl     = DMA1,
      .chan     = 2,
      .irq      = N32_IRQ_DMA1_CHANNEL2,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(2),
    },

    {
      .ctrl     = DMA1,
      .chan     = 3,
      .irq      = N32_IRQ_DMA1_CHANNEL3,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(3),
    },

    {
      .ctrl     = DMA1,
      .chan     = 4,
      .irq      = N32_IRQ_DMA1_CHANNEL4,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(4),
    },

    {
      .ctrl     = DMA1,
      .chan     = 5,
      .irq      = N32_IRQ_DMA1_CHANNEL5,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(5),
    },

    {
      .ctrl     = DMA1,
      .chan     = 6,
      .irq      = N32_IRQ_DMA1_CHANNEL6,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(6),
    },

    {
      .ctrl     = DMA1,
      .chan     = 7,
      .irq      = N32_IRQ_DMA1_CHANNEL7,
      .base     = N32_DMA1_BASE + N32_DMA_CH_OFFSET(7),
    },
#endif

#ifdef CONFIG_N32H7_DMA2
  /* DMA2 */

    {
      .ctrl     = DMA2,
      .chan     = 0,
      .irq      = N32_IRQ_DMA2_CHANNEL0,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(0),
    },

    {
      .ctrl     = DMA2,
      .chan     = 1,
      .irq      = N32_IRQ_DMA2_CHANNEL1,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(1),
    },

    {
      .ctrl     = DMA2,
      .chan     = 2,
      .irq      = N32_IRQ_DMA2_CHANNEL2,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(2),
    },

    {
      .ctrl     = DMA2,
      .chan     = 3,
      .irq      = N32_IRQ_DMA2_CHANNEL3,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(3),
    },

    {
      .ctrl     = DMA2,
      .chan     = 4,
      .irq      = N32_IRQ_DMA2_CHANNEL4,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(4),
    },

    {
      .ctrl     = DMA2,
      .chan     = 5,
      .irq      = N32_IRQ_DMA2_CHANNEL5,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(5),
    },

    {
      .ctrl     = DMA2,
      .chan     = 6,
      .irq      = N32_IRQ_DMA2_CHANNEL6,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(6),
    },

    {
      .ctrl     = DMA2,
      .chan     = 7,
      .irq      = N32_IRQ_DMA2_CHANNEL7,
      .base     = N32_DMA2_BASE + N32_DMA_CH_OFFSET(7),
    },
#endif

#ifdef CONFIG_N32H7_DMA3
  /* DMA3 */

    {
      .ctrl     = DMA3,
      .chan     = 0,
      .irq      = N32_IRQ_DMA3_CHANNEL0,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(0),
    },

    {
      .ctrl     = DMA3,
      .chan     = 1,
      .irq      = N32_IRQ_DMA3_CHANNEL1,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(1),
    },

    {
      .ctrl     = DMA3,
      .chan     = 2,
      .irq      = N32_IRQ_DMA3_CHANNEL2,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(2),
    },

    {
      .ctrl     = DMA3,
      .chan     = 3,
      .irq      = N32_IRQ_DMA3_CHANNEL3,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(3),
    },

    {
      .ctrl     = DMA3,
      .chan     = 4,
      .irq      = N32_IRQ_DMA3_CHANNEL4,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(4),
    },

    {
      .ctrl     = DMA3,
      .chan     = 5,
      .irq      = N32_IRQ_DMA3_CHANNEL5,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(5),
    },

    {
      .ctrl     = DMA3,
      .chan     = 6,
      .irq      = N32_IRQ_DMA3_CHANNEL6,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(6),
    },

    {
      .ctrl     = DMA3,
      .chan     = 7,
      .irq      = N32_IRQ_DMA3_CHANNEL7,
      .base     = N32_DMA3_BASE + N32_DMA_CH_OFFSET(7),
    },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * DMA register access functions
 ****************************************************************************/

/****************************************************************************
 * Name: dmachan_getbase
 *
 * Description:
 *  Get base DMA address for dmachan
 *
 ****************************************************************************/

static uint32_t dmachan_getbase(DMA_CHANNEL dmachan)
{
  uint8_t controller = dmachan->ctrl;

  return g_dma[controller].base;
}

/****************************************************************************
 * Name: dmabase_getreg
 *
 * Description:
 *  Get non-channel register from DMA controller
 *
 ****************************************************************************/

static uint32_t dmabase_getreg(DMA_CHANNEL dmachan, uint32_t offset)
{
  uint32_t dmabase = dmachan_getbase(dmachan);

  return getreg32(dmabase + offset);
}

/****************************************************************************
 * Name: dmabase_putreg
 *
 * Description:
 *  Write to non-channel register in DMA controller
 *
 ****************************************************************************/

static void dmabase_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value)
{
  uint32_t dmabase = dmachan_getbase(dmachan);

  putreg32(value, dmabase + offset);
}

/****************************************************************************
 * Name: dmachan_getreg
 *
 * Description:
 *  Get channel register.
 *
 ****************************************************************************/

static uint32_t dmachan_getreg(DMA_CHANNEL dmachan, uint32_t offset)
{
  return getreg32(dmachan->base + offset);
}

/****************************************************************************
 * Name: dmachan_putreg
 *
 * Description:
 *  Write to channel register.
 *
 ****************************************************************************/

static void dmachan_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value)
{
  putreg32(value, dmachan->base + offset);
}

/****************************************************************************
 * Name: dmamux_getreg
 *
 * Description:
 *  Write to DMAMUX
 *
 ****************************************************************************/

static void dmamux_putreg(DMA_MUX dmamux, uint32_t offset, uint32_t value)
{
  putreg32(value, dmamux->base + offset);
}

/****************************************************************************
 * Name: dmamux_getreg
 *
 * Description:
 *  Get DMAMUX register.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static uint32_t dmamux_getreg(DMA_MUX dmamux, uint32_t offset)
{
  return getreg32(dmamux->base + offset);
}
#endif

/****************************************************************************
 * Name: n32_dma_channel_get
 *
 * Description:
 *  Get the g_dmach table entry associated with a given DMA controller
 *  and channel number.
 *
 ****************************************************************************/

static DMA_CHANNEL n32_dma_channel_get(uint8_t channel, uint8_t controller)
{
  uint8_t first = 0;
  uint8_t nchan = 0;

  /* Get limits for g_dma array */

  n32_gdma_limits_get(controller, &first, &nchan);

  DEBUGASSERT(channel <= nchan);

  return &g_dmach[first + channel];
}

/****************************************************************************
 * Name: n32_gdma_limits_get
 *
 * Description:
 *  Get g_dma array limits for a given DMA controller.
 *
 ****************************************************************************/

static void n32_gdma_limits_get(uint8_t controller, uint8_t *first,
                                  uint8_t *nchan)
{
  DEBUGASSERT(first != NULL);
  DEBUGASSERT(nchan != NULL);

  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  *first = g_dma[controller].first;
  *nchan  = g_dma[controller].nchan;
}

/****************************************************************************
 * Master DMA functions
 ****************************************************************************/

#ifdef CONFIG_N32H7_MDMA

/****************************************************************************
 * Name: n32_mdma_disable
 *
 * Description:
 *  Disable the master DMA
 *
 ****************************************************************************/

static void n32_mdma_disable(DMA_CHANNEL dmachan)
{
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

#warning n32_mdma_disable
}

/****************************************************************************
 * Name: n32_mdma_interrupt
 *
 * Description:
 *  Master DMA interrupt handler
 *
 ****************************************************************************/

static int n32_mdma_interrupt(int irq, void *context, void *arg)
{
#warning n32_mdma_interrupt
}

/****************************************************************************
 * Name: n32_mdma_setup
 *
 * Description:
 *   Configure master DMA before using
 *
 ****************************************************************************/

static void n32_mdma_setup(DMA_HANDLE handle, n32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

#warning n32_mdma_setup not implemented
}

/****************************************************************************
 * Name: n32_mdma_start
 *
 * Description:
 *   Start the master DMA transfer
 ****************************************************************************/

static void n32_mdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

#warning n32_mdma_start not implemented
}

/****************************************************************************
 * Name: n32_mdma_residual
 ****************************************************************************/

static size_t n32_mdma_residual(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;
  uint32_t    residual   = 0;

  DEBUGASSERT(controller == MDMA);

  /* REVISIT */

  /* Fetch the count of blocks remaining to be transferred */

  residual = dmachan_getreg(dmachan, N32_MDMACH_CBNDTR_OFFSET);

  return (size_t)(residual & MDMA_CBNDTR_BNDT_MASK);
}

/****************************************************************************
 * Name: n32_mdma_capable
 ****************************************************************************/

#ifdef CONFIG_N32H7_DMACAPABLE
static bool n32_mdma_capable(n32_dmacfg_t *cfg)
{
  uint32_t transfer_size;
  uint32_t mend;
  uint32_t ccr  = cfg->cfg1;
  uint32_t ctcr = cfg->cfg2;

  dmainfo("0x%08" PRIx32 "/%" PRIu32 " 0x%08" PRIx32 " 0x%08" PRIx32 "\n",
          cfg->maddr, cfg->ndata, ccr, ctcr);

#warning n32_mdma_capable not implemented

  return true;
}
#endif

/****************************************************************************
 * Name: n32_mdma_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void n32_mdma_dump(DMA_HANDLE handle, const char *msg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

  dmainfo("   CISR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CISR_OFFSET));
  dmainfo("   CESR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CESR_OFFSET));
  dmainfo("   CCR:    %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CCR_OFFSET));
  dmainfo("   CTCR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CTCR_OFFSET));
  dmainfo("   CBNDTR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CBNDTR_OFFSET));
  dmainfo("   CSAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CSAR_OFFSET));
  dmainfo("   CDAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CDAR_OFFSET));
  dmainfo("   CBRUR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CBRUR_OFFSET));
  dmainfo("   CLAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CLAR_OFFSET));
  dmainfo("   CTBR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CTBR_OFFSET));
  dmainfo("   CMAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CMAR_OFFSET));
  dmainfo("   CMDR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_MDMACH_CMDR_OFFSET));
}
#endif

#endif /* CONFIG_N32H7_MDMA */

/****************************************************************************
 * Standard DMA functions
 ****************************************************************************/

#if defined(CONFIG_N32H7_DMA1) || defined(CONFIG_N32H7_DMA2) || defined(CONFIG_N32H7_DMA3)

/****************************************************************************
 * Name: n32_sdma_disable
 *
 * Description:
 *  Disable standard DMA chanel (DMA1/DMA2)
 *
 ****************************************************************************/

static void n32_sdma_disable(DMA_CHANNEL dmachan)
{
  uint32_t regval     = 0;
  uint8_t  chanel     = 0;

  DEBUGASSERT(dmachan->ctrl == DMA1 ||
              dmachan->ctrl == DMA2 ||
              dmachan->ctrl == DMA3);

  /* Get DMA chanel */

  chanel = dmachan->chan;

  DEBUGASSERT(chanel < 8);

  /* Disable the DMA chanel */

  /* Set chanel enable bit writable */

  regval  = dmabase_getreg(dmachan, N32_DMA_CHEN_OFFSET);
  regval |= DMA_CHEN_CHWEN(chanel);

  /* Clear chanel enable bit */

  regval &= ~DMA_CHEN_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_CHEN_OFFSET, regval);

  while (dmachan_getreg(dmachan, N32_DMA_CHEN_OFFSET) &
                       DMA_CHEN_CH(chanel));

  /* Clear pending chanel interrupts */

  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_TCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_BTCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_STCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_DTCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_ERRINTCLR_OFFSET, regval);

  /* Disable all interrupts at the DMA controller */

  regval = dmachan_getreg(dmachan, N32_DMA_CH_CTRL_OFFSET);
  regval &= ~DMA_CHCTRL_INTEN;
  dmachan_putreg(dmachan, N32_DMA_CH_CTRL_OFFSET, regval);
}

/****************************************************************************
 * Name: n32_sdma_interrupt
 *
 * Description:
 *  Standard DMA interrupt handler
 *
 ****************************************************************************/

static int n32_sdma_interrupt(int irq, void *context, void *arg)
{
  DMA_CHANNEL dmachan     = NULL;
  uint32_t    status      = 0;
  uint32_t    regval      = 0;
  uint8_t     chanel      = 0;
  uint8_t     controller  = 0;

  /* Get the chanel and the controller that generated the interrupt */

#ifdef CONFIG_N32H7_DMA1
  if (irq >= N32_IRQ_DMA1_CHANNEL0 && irq <= N32_IRQ_DMA1_CHANNEL7)
    {
      chanel     = irq - N32_IRQ_DMA1_CHANNEL0;
      controller = DMA1;
    }
  else
#endif
#ifdef CONFIG_N32H7_DMA2
  if (irq >= N32_IRQ_DMA2_CHANNEL0 && irq <= N32_IRQ_DMA2_CHANNEL7)
    {
      chanel     = irq - N32_IRQ_DMA2_CHANNEL0;
      controller = DMA2;
    }
  else
#endif
#ifdef CONFIG_N32H7_DMA3
  if (irq >= N32_IRQ_DMA3_CHANNEL0 && irq <= N32_IRQ_DMA3_CHANNEL7)
    {
      chanel     = irq - N32_IRQ_DMA3_CHANNEL0;
      controller = DMA3;
    }
  else
#endif
    {
      DEBUGPANIC();
    }

  /* Get the channel structure from the chanel and controller numbers */

  dmachan = n32_dma_channel_get(chanel, controller);

  /* Get the interrupt status for this chanel */

  status = (((dmabase_getreg(dmachan, N32_DMA_TCINTSTS_OFFSET)  &
            DMA_INT_CH(chanel)) >> chanel) << 0) |
           (((dmabase_getreg(dmachan, N32_DMA_BTCINTSTS_OFFSET) &
            DMA_INT_CH(chanel)) >> chanel) << 1) |
           (((dmabase_getreg(dmachan, N32_DMA_STCINTSTS_OFFSET) &
            DMA_INT_CH(chanel)) >> chanel) << 2) |
           (((dmabase_getreg(dmachan, N32_DMA_DTCINTSTS_OFFSET) &
            DMA_INT_CH(chanel)) >> chanel) << 3) |
           (((dmabase_getreg(dmachan, N32_DMA_ERRINTSTS_OFFSET) &
            DMA_INT_CH(chanel)) >> chanel) << 4);

  /* Clear fetched chanel interrupts */

  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_TCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_BTCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_STCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_DTCINTCLR_OFFSET, regval);
  regval  = DMA_INT_CH(chanel);
  dmabase_putreg(dmachan, N32_DMA_ERRINTCLR_OFFSET, regval);

  /* Invoke the callback */

  if (dmachan->callback)
    {
      dmachan->callback(dmachan, status, dmachan->arg);
    }

  return OK;
}

/****************************************************************************
 * Name: n32_sdma_setup
 *
 * Description:
 *   Configure standard DMA before using
 *
 ****************************************************************************/

static void n32_sdma_setup(DMA_HANDLE handle, n32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint8_t ch = dmachan->chan;
  uint32_t reg;

  /* Disable channel if active, wait until stopped */

  if (dmabase_getreg(dmachan, N32_DMA_CHEN_OFFSET) & DMA_CHEN_CH(ch))
    {
      reg = dmabase_getreg(dmachan, N32_DMA_CHEN_OFFSET);
      reg |= DMA_CHEN_CHWEN(ch);
      reg &= ~DMA_CHEN_CH(ch);
      dmabase_putreg(dmachan, N32_DMA_CHEN_OFFSET, reg);

      int timeout = 100;

      while (timeout-- && (dmabase_getreg(dmachan, N32_DMA_CHEN_OFFSET) &
              DMA_CHEN_CH(ch)))
        {
          up_udelay(1);
        }

      DEBUGASSERT((dmabase_getreg(dmachan, N32_DMA_CHEN_OFFSET) &
                                  DMA_CHEN_CH(ch)) == 0);
    }

  /* Clear all pending interrupts */

  uint32_t clr = DMA_INT_CH(ch);

  dmabase_putreg(dmachan, N32_DMA_TCINTCLR_OFFSET,  clr);
  dmabase_putreg(dmachan, N32_DMA_BTCINTCLR_OFFSET, clr);
  dmabase_putreg(dmachan, N32_DMA_STCINTCLR_OFFSET, clr);
  dmabase_putreg(dmachan, N32_DMA_DTCINTCLR_OFFSET, clr);
  dmabase_putreg(dmachan, N32_DMA_ERRINTCLR_OFFSET, clr);

  /* Set address and linked-list registers */

  dmachan_putreg(dmachan, N32_DMA_CH_SA_OFFSET,  cfg->src_addr);
  dmachan_putreg(dmachan, N32_DMA_CH_DA_OFFSET,  cfg->dst_addr);
  dmachan_putreg(dmachan, N32_DMA_CH_LLP_OFFSET, (uint32_t)
                  (uintptr_t)cfg->link_list);

  /* Write control register (merge block_size into BTS field) */

  uint64_t full_ctrl = cfg->ctrl;

  full_ctrl &= ~DMA_CHCTRL_BTS_MASK;
  full_ctrl |= DMA_CHCTRL_BTS((uint64_t)cfg->block_size & 0xfffull);
  dmachan_putreg(dmachan, N32_DMA_CH_CTRL_OFFSET,
                 (uint32_t)full_ctrl);
  dmachan_putreg(dmachan, N32_DMA_CH_CTRL_OFFSET + 4,
                 (uint32_t)(full_ctrl >> 32));

  /* Optional SG/DS registers */

  if (cfg->sg_cfg)
    {
      dmachan_putreg(dmachan, N32_DMA_CH_SG_OFFSET,
                     cfg->sg_cfg);
    }

  if (cfg->ds_cfg)
    {
      dmachan_putreg(dmachan, N32_DMA_CH_DS_OFFSET,
                     cfg->ds_cfg);
    }

  /* Configure CH_CFG: handshake interfaces, priority, and HSSEL bits */

  uint32_t cfg_low  = dmachan_getreg(dmachan, N32_DMA_CH_CFG_OFFSET);
  uint32_t cfg_high = dmachan_getreg(dmachan, N32_DMA_CH_CFG_OFFSET + 4);

  cfg_high &= ~(DMA_CHCFG_SRCPER_MASK >> 32);
  cfg_high &= ~(DMA_CHCFG_DSTPER_MASK >> 32);
  cfg_high |= ((uint64_t)(cfg->src_hs_if & 0x7) <<
              (DMA_CHCFG_SRCPER_SHIFT - 32));
  cfg_high |= ((uint64_t)(cfg->dst_hs_if & 0x7) <<
              (DMA_CHCFG_DSTPER_SHIFT - 32));

  cfg_low &= ~(DMA_CHCFG_HSSELSRC | DMA_CHCFG_HSSELDST |
               DMA_CHCFG_CHPRIOR_MASK);

  if (cfg->src_hs_mode == 1)
    {
      cfg_low |= DMA_CHCFG_HSSELSRC;
    }

  if (cfg->dst_hs_mode == 1)
    {
      cfg_low |= DMA_CHCFG_HSSELDST;
    }

  cfg_low |= ((cfg->priority & 0x7) << DMA_CHCFG_CHPRIOR_SHIFT);

  dmachan_putreg(dmachan, N32_DMA_CH_CFG_OFFSET,     cfg_low);
  dmachan_putreg(dmachan, N32_DMA_CH_CFG_OFFSET + 4, cfg_high);
}

/****************************************************************************
 * Name: n32_sdma_start
 *
 * Description:
 *   Start the standard DMA transfer
 ****************************************************************************/

static void n32_sdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint32_t regval = 0;

  DEBUGASSERT(handle != NULL);
  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  /* Save the callback info.  This will be invoked when the DMA completes */

  dmachan->callback = callback;
  dmachan->arg      = arg;

  /* UnMask TFC and ERR interrupt */

  regval  = DMA_CHEN_CHWEN(dmachan->chan) | DMA_CHEN_CH(dmachan->chan);
  dmabase_putreg(dmachan, N32_DMA_TCINTMSK_OFFSET, regval);
  dmabase_putreg(dmachan, N32_DMA_ERRINTMSK_OFFSET, regval);
  dmabase_putreg(dmachan, N32_DMA_BTCINTMSK_OFFSET, regval);
  dmabase_putreg(dmachan, N32_DMA_STCINTMSK_OFFSET, regval);
  dmabase_putreg(dmachan, N32_DMA_DTCINTMSK_OFFSET, regval);

  /* Enable all interrupts at the DMA controller */

  regval  = dmachan_getreg(dmachan, N32_DMA_CH_CTRL_OFFSET);
  regval |= DMA_CHCTRL_INTEN;
  dmachan_putreg(dmachan, N32_DMA_CH_CTRL_OFFSET, regval);

  /* Set chanel enable bit writable */

  regval  = dmabase_getreg(dmachan, N32_DMA_CHEN_OFFSET);
  regval |= DMA_CHEN_CHWEN(dmachan->chan);

  /* Set chanel enable bit */

  regval |= DMA_CHEN_CH(dmachan->chan);
  dmabase_putreg(dmachan, N32_DMA_CHEN_OFFSET, regval);

  n32_dmadump(handle, "DMA after start");
}

/****************************************************************************
 * Name: n32_sdma_residual
 ****************************************************************************/

static size_t n32_sdma_residual(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan  = (DMA_CHANNEL)handle;
  uint32_t    residual = 0;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  /* Fetch the count of bytes remaining to be transferred.
   *
   * If the FIFO is enabled, this count may be inaccurate.  ST don't
   * appear to document whether this counts the peripheral or the memory
   * side of the channel, and they don't make the memory pointer
   * available either.
   *
   * For reception in circular mode the FIFO is disabled in order that
   * this value can be useful.
   */

  residual = dmachan_getreg(dmachan, N32_DMA_CH_SG_OFFSET);

  return (size_t)((residual & DMA_SG_SGC_MASK) >> DMA_SG_SGC_SHIFT);
}

/****************************************************************************
 * Name: n32_sdma_capable
 ****************************************************************************/

#ifdef CONFIG_N32H7_DMACAPABLE
static bool n32_sdma_capable(n32_dmacfg_t *cfg)
{
  uint32_t transfer_size;
  uint32_t burst_length;
  uint32_t mend;
  uint32_t ccr = cfg->cfg1;

  dmainfo("0x%08" PRIx32 "/%" PRIu32 " 0x%08" PRIx32 "\n",
          cfg->maddr, cfg->ndata, cfg->cfg1);

  /* Verify that the address conforms to the memory transfer size.
   * Transfers to/from memory performed by the DMA controller are
   * required to be aligned to their size.
   *
   * See ST RM0410 DocID028270 Rev 2, section 8.3.11 Single and burst
   * transfers
   *
   * Compute mend inline to avoid a possible non-constant integer
   * multiply.
   */

  switch (ccr & DMA_SCR_MSIZE_MASK)
    {
      case DMA_SCR_MSIZE_8BITS:
        {
          transfer_size = 1;
          mend          = cfg->maddr + cfg->ndata - 1;
          break;
        }

      case DMA_SCR_MSIZE_16BITS:
        {
          transfer_size = 2;
          mend          = cfg->maddr + (cfg->ndata << 1) - 1;
          break;
        }

      case DMA_SCR_MSIZE_32BITS:
        {
          transfer_size = 4;
          mend          = cfg->maddr + (cfg->ndata << 2) - 1;
          break;
        }

      default:
        {
          dmainfo("n32_dmacapable: bad transfer size in CCR\n");
          return false;
        }
    }

  if ((cfg->maddr & (transfer_size - 1)) != 0)
    {
      dmainfo("n32_dmacapable: transfer unaligned\n");
      return false;
    }

#  if defined(CONFIG_ARMV7M_DCACHE) && \
     !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
  /* buffer alignment is required for RX DMA transfers with dcache in
   * buffered mode (not write-through) because arch_invalidate_dcache could
   * lose buffered writes
   */

  if ((ccr & DMA_SCR_DIR_MASK) == DMA_SCR_DIR_P2M ||
      (ccr & DMA_SCR_DIR_MASK) == DMA_SCR_DIR_M2M)
    {
      if ((cfg->maddr & (ARMV7M_DCACHE_LINESIZE - 1)) != 0 ||
          ((mend + 1) & (ARMV7M_DCACHE_LINESIZE - 1)) != 0)
        {
          dmainfo("n32_dmacapable: dcache unaligned "
                  "maddr:0x%08" PRIx32 " mend:0x%08" PRIx32 "\n",
                  cfg->maddr, mend);
#if !defined(CONFIG_N32H7_DMACAPABLE_ASSUME_CACHE_ALIGNED)
          return false;
#endif
        }
    }
#  endif

  /* Verify that burst transfers do not cross a 1KiB boundary. */

  if ((cfg->maddr / 1024) != (mend / 1024))
    {
      /* The transfer as a whole crosses a 1KiB boundary.
       * Verify that no burst does by asserting that the address
       * is aligned to the burst length.
       */

      switch (ccr & DMA_SCR_MBURST_MASK)
        {
          case DMA_SCR_MBURST_SINGLE:
            {
              burst_length = transfer_size;
              break;
            }

          case DMA_SCR_MBURST_INCR4:
            {
              burst_length = transfer_size << 2;
              break;
            }

          case DMA_SCR_MBURST_INCR8:
            {
              burst_length = transfer_size << 3;
              break;
            }

          case DMA_SCR_MBURST_INCR16:
            {
              burst_length = transfer_size << 4;
              break;
            }

          default:
            {
              dmainfo("n32_dmacapable: bad burst size in CCR\n");
              return false;
            }
        }

      if ((cfg->maddr & (burst_length - 1)) != 0)
        {
          dmainfo("n32_dmacapable: burst crosses 1KiB\n");
          return false;
        }
    }

  /* Verify that transfer is froma a supported memory region */

  if ((cfg->paddr & N32_PREGION_MASK) != N32_D2_BASE)
    {
      /* DMA1/DMA2 support only D2 domain */

      dmainfo("transfer from unknown/unsupported region\n");
      return false;
    }

  /* Verify that the transfer is to a memory region that supports DMA. */

  if ((cfg->maddr & N32_REGION_MASK) != (mend & N32_REGION_MASK))
    {
      dmainfo("n32_dmacapable: transfer crosses memory region\n");
      return false;
    }

  switch (cfg->maddr & N32_REGION_MASK)
    {
      case N32_AXISRAM_BASE:
      case N32_FMC_BANK1:
      case N32_FMC_BANK2:
      case N32_FMC_BANK3:
      case N32_FMC_BANK4:
      case N32_FMC_BANK5:
      case N32_FMC_BANK6:
        {
          /* All RAM and FMC is supported */

          break;
        }

      case N32_SRAM_BASE:
        {
          /* DTCM not supported for standard DMA (DMA1/DMA2) */

          if (cfg->maddr >= N32_DTCRAM_BASE
              && (cfg->maddr - N32_DTCRAM_BASE) < 0x1ffff)
            {
              dmainfo("transfer targets DTCRAM\n");
              return false;
            }
          break;
        }

      case N32_CODE_BASE:
        {
          /* ITCM not supported for standard DMA (DMA1/DMA2) */

          if (cfg->maddr >= N32_ITCM_BASE
              && (cfg->maddr - N32_ITCM_BASE) < 0xffff)
            {
              dmainfo("transfer targets ITCM RAM\n");
              return false;
            }
          break;
        }

      default:
        {
          /* Everything else is unsupported by DMA */

          dmainfo("transfer targets unknown/unsupported region\n");
          return false;
        }
    }

  dmainfo("transfer OK\n");
  return true;
}
#endif

/****************************************************************************
 * Name: n32_sdma_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void n32_sdma_dump(DMA_HANDLE handle, const char *msg)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  dmainfo("   LISR: %08" PRIx32 "\n",
          dmabase_getreg(dmachan, N32_DMA_LISR_OFFSET));
  dmainfo("   HISR: %08" PRIx32 "\n",
          dmabase_getreg(dmachan, N32_DMA_HISR_OFFSET));
  dmainfo("   SCR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_DMA_SCR_OFFSET));
  dmainfo("   SNDTR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_DMA_SNDTR_OFFSET));
  dmainfo("   SPAR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_DMA_SPAR_OFFSET));
  dmainfo("   SM0AR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_DMA_SM0AR_OFFSET));
  dmainfo("   SM1AR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_DMA_SM1AR_OFFSET));
  dmainfo("   SFCR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, N32_DMA_SFCR_OFFSET));

  n32_dmamux_dump(g_dma[dmachan->ctrl].dmamux,
                    dmachan->chan + g_dma[dmachan->ctrl].dmamux_offset);
}
#endif

#endif /* CONFIG_N32H7_DMA1 || CONFIG_N32H7_DMA2 || CONFIG_N32H7_DMA3 */

/****************************************************************************
 * Name: n32_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem (DMA1, DMA2, MDMA and DMA3)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_dma_initialize(void)
{
  DMA_CHANNEL dmachan    = NULL;
  uint8_t     controller = 0;
  int         channel    = 0;

  dmainfo("Initialize DMA\n");

  /* Initialize DMA channels */

  for (channel = 0; channel < DMA_NCHANNELS; channel++)
    {
      dmachan = &g_dmach[channel];

      /* Initialize flag */

      dmachan->used = false;

      /* Get DMA controller associated with channel */

      controller = dmachan->ctrl;

      DEBUGASSERT(controller >= MDMA && controller <= DMA3);

      /* Attach standard DMA interrupt vectors */

      irq_attach(dmachan->irq, g_dma_ops[controller].dma_interrupt,
                dmachan);

      /* Disable the DMA stream */

      g_dma_ops[controller].dma_disable(dmachan);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(dmachan->irq);

      /* Set the interrupt priority */

#ifdef CONFIG_ARCH_IRQPRIO
      switch (controller)
        {
#if defined(CONFIG_N32H7_DMA1) || defined(CONFIG_N32H7_DMA2) || defined(CONFIG_N32H7_DMA3)
          case DMA1:
          case DMA2:
          case DMA3:
            {
              up_prioritize_irq(dmachan->irq, CONFIG_DMA_PRI);
              break;
            }
#endif /* CONFIG_N32H7_DMA1 && CONFIG_N32H7_DMA2 */

#ifdef CONFIG_N32H7_MDMA
          case MDMA:
            {
              up_prioritize_irq(dmachan->irq, CONFIG_MDMA_PRI);
              break;
            }
#endif /* CONFIG_N32H7_MDMA */

          default:
            {
              PANIC();
              break;
            }
        }
#endif
    }
}

/****************************************************************************
 * Name: n32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   It is common for standard DMA (DMA1, DMA2), master DMA (MDMA) and
 *   basic DMA (BDMA) controllers.
 *
 * Input Parameters:
 *   dmamap - Identifies the stream/channel resource. For the N32 H7, this
 *     is a bit-encoded  value as provided by the DMAMAP_* definitions
 *     in chip/n32h7xxxxxxx_dmamux.h
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE n32_dmachannel(unsigned int dmamap)
{
  DMA_CHANNEL dmachan    = NULL;
  DMA_MUX     dmamux     = NULL;
  irqstate_t  flags;
  uint8_t     controller = 0;
  uint8_t     dmamux_req = 0;
  uint32_t    regval     = 0;
  uint8_t     first      = 0;
  uint8_t     nchan      = 0;
  int         item       = -1;
  int         i          = 0;

  /* Get DMA controller from encoded DMAMAP value */

  controller = DMAMAP_CONTROLLER(dmamap);
  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  /* Get DMAMUX channel from encoded DMAMAP value */

  dmamux_req = DMAMAP_REQUEST(dmamap);

  /* Get g_dma array limits for given controller */

  n32_gdma_limits_get(controller, &first, &nchan);

  /* Find available channel for given controller */

  flags = enter_critical_section();
  for (i = first; i < first + nchan; i += 1)
    {
      if (g_dmach[i].used == false)
        {
          item = i;
          g_dmach[i].used = true;
          break;
        }
    }

  leave_critical_section(flags);

  dmainfo("ctrl=%" PRIu8 " item=%d\n", controller, item);

  if (item == -1)
    {
      dmainfo("No available DMA chan for CTRL=%" PRIu8 "\n",
              controller);

      /* No available channel */

      goto errout;
    }

  /* Assign DMA item */

  dmachan = &g_dmach[item];

  dmainfo("Get g_dmach[%d] CTRL=%" PRIu8 " CH=%" PRIu8 "\n",
          i, controller, dmachan->chan);

  /* Be sure that we have proper DMA controller */

  DEBUGASSERT(dmachan->ctrl == controller);

  /* Get DMAMUX associated with DMA controller */

  dmamux = g_dma[controller].dmamux;

  /* No DMAMUX for Master DMA */

  if (dmamux != NULL)
    {
      uint8_t dmamux_chan = dmachan->chan + g_dma[controller].dmamux_offset;

      dmainfo("Get DMAMUX%" PRIu8 " CH %" PRIu8 "\n",
              dmamux->id, dmamux_chan);

      /* DMAMUX Set DMA channel source */

      regval = dmamux_req << DMAMUX_CXCR_REQID_SHIFT;
      dmamux_putreg(dmamux, N32_DMAMUX_CXCR_OFFSET(dmamux_chan), regval);

      /* DMAMUX Set RGCR register */

      regval = 0;
      dmamux_putreg(dmamux, N32_DMAMUX_RGXCR_OFFSET(dmamux_chan), regval);
    }

errout:
  return (DMA_HANDLE)dmachan;
}

/****************************************************************************
 * Name: n32_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until n32_dmachannel() is called again to re-gain access to the
 *   channel
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void n32_dmafree(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;
  DMA_MUX     dmamux     = NULL;
  irqstate_t  flags;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  /* Get DMAMUX associated with DMA controller */

  dmamux = g_dma[controller].dmamux;

  /* No DMAMUX for Master DMA */

  if (dmamux != NULL)
    {
      uint8_t dmamux_chan = dmachan->chan + g_dma[controller].dmamux_offset;

      dmainfo("Free DMAMUX%" PRIu8 " CH %" PRIu8 "\n",
              dmamux->id, dmamux_chan);

      /* Clear DMAMUX CCR register associated with channel */

      dmamux_putreg(dmamux, N32_DMAMUX_CXCR_OFFSET(dmamux_chan), 0);

      /* Clear DMAMUX RGCR register associated with channel */

      dmamux_putreg(dmamux, N32_DMAMUX_RGXCR_OFFSET(dmamux_chan), 0);
    }

  /* Release the channel */

  flags = enter_critical_section();
  dmachan->used = false;
  leave_critical_section(flags);

  dmainfo("Unmapping DMAMUX(%" PRIu8 ")\n", dmachan->chan);
}

/****************************************************************************
 * Name: n32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void n32_dmasetup(DMA_HANDLE handle, n32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  /* Get DMA controller */

  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  g_dma_ops[controller].dma_setup(handle, cfg);
}

/****************************************************************************
 * Name: n32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by n32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void n32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  g_dma_ops[controller].dma_start(handle, callback, arg);
}

/****************************************************************************
 * Name: n32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After n32_dmastop() is called, the DMA channel is
 *   reset and n32_dmasetup() must be called before n32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by n32_dmachannel()
 *
 ****************************************************************************/

void n32_dmastop(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  g_dma_ops[controller].dma_disable(dmachan);
}

/****************************************************************************
 * Name: n32_getchanel
 *
 * Description:
 *   Read the DMA bytes-remaining register.
 *
 * Assumptions:
 *   - DMA handle allocated by n32_dmachannel()
 *
 ****************************************************************************/

size_t n32_getchanel(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;

  return dmachan->chan;
}

uint32_t n32_dmaptr_src(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;

  return dmachan_getreg(dmachan, N32_DMA_CH_SA_OFFSET);
}

uint32_t n32_dmaptr_dst(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;

  return dmachan_getreg(dmachan, N32_DMA_CH_DA_OFFSET);
}

uint32_t n32_getcount(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;

  return dmachan_getreg(dmachan, N32_DMA_CH_CTRL_OFFSET + 4) & 0x0fff;
}

/****************************************************************************
 * Name: n32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address. This depends on the internal connections in the ARM bus matrix
 *   of the processor. Note that this only applies to memory addresses, it
 *   will return false for any peripheral address.
 *
 * Input Parameters:
 *   cfg - DMA transfer configuration
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_DMACAPABLE
bool n32_dmacapable(DMA_HANDLE handle, n32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  return g_dma_ops[controller].dma_capable(cfg);
}
#endif

/****************************************************************************
 * Name: n32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by n32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void n32_dmadump(DMA_HANDLE handle, const char *msg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= DMA3);

  dmainfo("DMA %" PRIu8 " CH%" PRIu8 " Registers: %s\n",
          dmachan->ctrl, dmachan->ctrl, msg);

  g_dma_ops[controller].dma_dump(handle, msg);
}
#endif
