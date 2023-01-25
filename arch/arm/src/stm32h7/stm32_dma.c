/****************************************************************************
 * arch/arm/src/stm32h7/stm32_dma.c
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
#include <arch/stm32h7/chip.h>

#include "arm_internal.h"
#include "sched/sched.h"
#include "stm32_dma.h"
#include "hardware/stm32_bdma.h"
#include "hardware/stm32_mdma.h"
#include "hardware/stm32_dmamux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMAMUX_NUM      2
#define DMA_CONTROLLERS 4

#ifdef CONFIG_STM32H7_MDMA
#  define MDMA_NCHAN     16
#else
#  define MDMA_NCHAN     0
#endif
#ifdef CONFIG_STM32H7_DMA1
#  define DMA1_NSTREAMS  8
#else
#  define DMA1_NSTREAMS  0
#endif
#ifdef CONFIG_STM32H7_DMA2
#  define DMA2_NSTREAMS  8
#else
#  define DMA2_NSTREAMS  0
#endif
#ifdef CONFIG_STM32H7_BDMA
#  define BDMA_NCHAN     8
#else
#  define BDMA_NCHAN     0
#endif

#define MDMA_FIRST       (0)
#define MDMA_LAST        (MDMA_FIRST+MDMA_NCHAN)
#define DMA1_FIRST       (MDMA_LAST)
#define DMA1_LAST        (DMA1_FIRST+DMA1_NSTREAMS)
#define DMA2_FIRST       (DMA1_LAST)
#define DMA2_LAST        (DMA2_FIRST+DMA2_NSTREAMS)
#define BDMA_FIRST       (DMA2_LAST)
#define BDMA_LAST        (BDMA_FIRST+BDMA_NCHAN)

/* All available DMA channels (streams from standard DMA and
 * channels from BDMA and MDMA)
 */

#define DMA_NCHANNELS    (DMA1_NSTREAMS+DMA2_NSTREAMS+MDMA_NCHAN+BDMA_NCHAN)

/* Default DMA, MDMA and BDMA priorities */

#ifndef CONFIG_DMA_PRI
#  define CONFIG_DMA_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_MDMA_PRI
#  define CONFIG_MDMA_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_BDMA_PRI
#  define CONFIG_BDMA_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure described one DMAMUX device */

struct stm32_dmamux_s
{
  uint8_t  id;                  /* DMAMUX id */
  uint8_t  nchan;               /* DMAMUX channels */
  uint32_t base;                /* DMAMUX base address */
};

typedef struct stm32_dmamux_s *DMA_MUX;

/* This structure describes one DMA controller */

struct stm32_dma_s
{
  uint8_t  first;                /* Offset in stm32_dmach_s array */
  uint8_t  nchan;                /* Number of channels */
  uint8_t  dmamux_offset;        /* DMAMUX channel offset */
  uint32_t base;                 /* Base address */
  DMA_MUX  dmamux;               /* DMAMUX associated with controller */
};

/* This structure describes one DMA channel (DMA12, MDMA or BDMA) */

struct stm32_dmach_s
{
  bool           used;      /* Channel in use */
  uint8_t        ctrl:3;    /* DMA controller */
  uint8_t        chan:5;    /* DMA stream/channel channel id */
  uint8_t        irq;       /* DMA stream IRQ number */
  uint8_t        shift;     /* ISR/IFCR bit shift value */
  uint32_t       base;      /* DMA register channel base address */
  dma_callback_t callback;  /* Callback invoked when the DMA completes */
  void          *arg;       /* Argument passed to callback function */
};

typedef struct stm32_dmach_s *DMA_CHANNEL;

/* DMA operations */

struct stm32_dma_ops_s
{
  /* Start the DMA transfer */

  void (*dma_disable)(DMA_CHANNEL dmachan);

  /* DMA interrupt */

  int (*dma_interrupt)(int irq, void *context, void *arg);

  /* Setup the DMA */

  void (*dma_setup)(DMA_HANDLE handle, stm32_dmacfg_t *cfg);

  /* Start the DMA */

  void (*dma_start)(DMA_HANDLE handle, dma_callback_t callback,
                    void *arg, bool half);

  /* Read remaining DMA bytes */

  size_t (*dma_residual)(DMA_HANDLE handle);

  /* Check the DMA configuration  */

  bool (*dma_capable)(stm32_dmacfg_t *cfg);

  /* Dump the DMA registers */

  void (*dma_dump)(DMA_HANDLE handle, const char *msg);
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_STM32H7_MDMA
static void stm32_mdma_disable(DMA_CHANNEL dmachan);
static int stm32_mdma_interrupt(int irq, void *context, void *arg);
static void stm32_mdma_setup(DMA_HANDLE handle, stm32_dmacfg_t *cfg);
static void stm32_mdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg, bool half);
static size_t stm32_mdma_residual(DMA_HANDLE handle);
#ifdef CONFIG_STM32H7_DMACAPABLE
static bool stm32_mdma_capable(stm32_dmacfg_t *cfg);
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32_mdma_dump(DMA_HANDLE handle, const char *msg);
#endif
#endif

#if defined(CONFIG_STM32H7_DMA1) || defined(CONFIG_STM32H7_DMA2)
static void stm32_sdma_disable(DMA_CHANNEL dmachan);
static int stm32_sdma_interrupt(int irq, void *context, void *arg);
static void stm32_sdma_setup(DMA_HANDLE handle, stm32_dmacfg_t *cfg);
static void stm32_sdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg, bool half);
static size_t stm32_sdma_residual(DMA_HANDLE handle);
#ifdef CONFIG_STM32H7_DMACAPABLE
static bool stm32_sdma_capable(stm32_dmacfg_t *cfg);
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32_sdma_dump(DMA_HANDLE handle, const char *msg);
#endif
#endif

#ifdef CONFIG_STM32H7_BDMA
static void stm32_bdma_disable(DMA_CHANNEL dmachan);
static int stm32_bdma_interrupt(int irq, void *context, void *arg);
static void stm32_bdma_setup(DMA_HANDLE handle, stm32_dmacfg_t *cfg);
static void stm32_bdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg, bool half);
static size_t stm32_bdma_residual(DMA_HANDLE handle);
#ifdef CONFIG_STM32H7_DMACAPABLE
static bool stm32_bdma_capable(stm32_dmacfg_t *cfg);
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32_bdma_dump(DMA_HANDLE handle, const char *msg);
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
static void stm32_dmamux_dump(DMA_MUX dmamux, uint8_t chan);
#endif
static DMA_CHANNEL stm32_dma_channel_get(uint8_t channel,
                                         uint8_t controller);
static void stm32_gdma_limits_get(uint8_t controller, uint8_t *first,
                                  uint8_t *last);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Operations specific to DMA controller */

struct stm32_dma_ops_s g_dma_ops[DMA_CONTROLLERS] =
{
#ifdef CONFIG_STM32H7_MDMA
  /* 0 - MDMA */

    {
      .dma_disable   = stm32_mdma_disable,
      .dma_interrupt = stm32_mdma_interrupt,
      .dma_setup     = stm32_mdma_setup,
      .dma_start     = stm32_mdma_start,
      .dma_residual  = stm32_mdma_residual,
#ifdef CONFIG_STM32H7_DMACAPABLE
      .dma_capable   = stm32_mdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = stm32_mdma_dump,
#endif
    },
#else
    {
      NULL
    },
#endif

#ifdef CONFIG_STM32H7_DMA1
  /* 1 - DMA1 */

    {
      .dma_disable   = stm32_sdma_disable,
      .dma_interrupt = stm32_sdma_interrupt,
      .dma_setup     = stm32_sdma_setup,
      .dma_start     = stm32_sdma_start,
      .dma_residual  = stm32_sdma_residual,
#ifdef CONFIG_STM32H7_DMACAPABLE
      .dma_capable   = stm32_sdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = stm32_sdma_dump,
#endif
    },
#else
    {
      NULL
    },
#endif

#ifdef CONFIG_STM32H7_DMA2
  /* 2 - DMA2 */

    {
      .dma_disable   = stm32_sdma_disable,
      .dma_interrupt = stm32_sdma_interrupt,
      .dma_setup     = stm32_sdma_setup,
      .dma_start     = stm32_sdma_start,
      .dma_residual  = stm32_sdma_residual,
#ifdef CONFIG_STM32H7_DMACAPABLE
      .dma_capable   = stm32_sdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = stm32_sdma_dump,
#endif
    },
#else
    {
      NULL
    },
#endif

#ifdef CONFIG_STM32H7_BDMA
  /* 3 - BDMA */

    {
      .dma_disable   = stm32_bdma_disable,
      .dma_interrupt = stm32_bdma_interrupt,
      .dma_setup     = stm32_bdma_setup,
      .dma_start     = stm32_bdma_start,
      .dma_residual  = stm32_bdma_residual,
#ifdef CONFIG_STM32H7_DMACAPABLE
      .dma_capable   = stm32_bdma_capable,
#endif
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_dump      = stm32_bdma_dump,
#endif
    }
#else
    {
      NULL
    }
#endif
};

/* This array describes the state of DMAMUX controller */

struct stm32_dmamux_s g_dmamux[DMAMUX_NUM] =
{
    {
      .id      = 1,
      .nchan   = 16,              /* 0-7 - DMA1, 8-15 - DMA2 */
      .base    = STM32_DMAMUX1_BASE
    },

    {
      .id      = 2,
      .nchan   = 8,               /* 0-7 - BDMA */
      .base    = STM32_DMAMUX2_BASE
    }
};

/* This array describes the state of each controller */

struct stm32_dma_s g_dma[DMA_NCHANNELS] =
{
  /* 0 - MDMA */

    {
      .base   = STM32_MDMA_BASE,
      .first  = MDMA_FIRST,
      .nchan  = MDMA_NCHAN,
      .dmamux = NULL,              /* No DMAMUX */
      .dmamux_offset = 0
    },

  /* 1 - DMA1 */

    {
      .base   = STM32_DMA1_BASE,
      .first  = DMA1_FIRST,
      .nchan  = DMA1_NSTREAMS,
      .dmamux = &g_dmamux[DMAMUX1], /* DMAMUX1 channels 0-7 */
      .dmamux_offset = 0
    },

  /* 2 - DMA2 */

    {
      .base   = STM32_DMA2_BASE,
      .first  = DMA2_FIRST,
      .nchan  = DMA2_NSTREAMS,
      .dmamux = &g_dmamux[DMAMUX1], /* DMAMUX1 channels 8-15 */
      .dmamux_offset = 8
    },

  /* 3 - BDMA */

    {
      .base   = STM32_BDMA_BASE,
      .first  = BDMA_FIRST,
      .nchan  = BDMA_NCHAN,
      .dmamux = &g_dmamux[DMAMUX2], /* DMAMUX2 channels 0-7 */
      .dmamux_offset = 0
    }
};

/* This array describes the state of each DMA channel.
 * Note that we keep here standard DMA streams, BDMA channels and MDMA
 * channels.
 */

static struct stm32_dmach_s g_dmach[DMA_NCHANNELS] =
{
#ifdef CONFIG_STM32H7_MDMA
  /* MDMA */

    {
      .ctrl     = MDMA,
      .chan     = 0,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(0),
    },

    {
      .ctrl     = MDMA,
      .chan     = 1,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(1),
    },

    {
      .ctrl     = MDMA,
      .chan     = 2,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(2),
    },

    {
      .ctrl     = MDMA,
      .chan     = 3,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(3),
    },

    {
      .ctrl     = MDMA,
      .chan     = 4,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(4),
    },

    {
      .ctrl     = MDMA,
      .chan     = 5,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(5),
    },

    {
      .ctrl     = MDMA,
      .chan     = 6,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(6),
    },

    {
      .ctrl     = MDMA,
      .chan     = 7,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(7),
    },

    {
      .ctrl     = MDMA,
      .chan     = 8,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(8),
    },

    {
      .ctrl     = MDMA,
      .chan     = 9,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(9),
    },

    {
      .ctrl     = MDMA,
      .chan     = 10,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(10),
    },

    {
      .ctrl     = MDMA,
      .chan     = 11,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(11),
    },

    {
      .ctrl     = MDMA,
      .chan     = 12,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(12),
    },

    {
      .ctrl     = MDMA,
      .chan     = 13,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(13),
    },

    {
      .ctrl     = MDMA,
      .chan     = 14,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(15),
    },

    {
      .ctrl     = MDMA,
      .chan     = 15,
      .irq      = STM32_IRQ_MDMA,
      .shift    = 0,
      .base     = STM32_MDMA_BASE + STM32_MDMA_OFFSET(15),
    },
#endif

#ifdef CONFIG_STM32H7_DMA1
  /* DMA1 */

    {
      .ctrl     = DMA1,
      .chan     = 0,
      .irq      = STM32_IRQ_DMA1S0,
      .shift    = DMA_INT_STREAM0_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(0),
    },

    {
      .ctrl     = DMA1,
      .chan     = 1,
      .irq      = STM32_IRQ_DMA1S1,
      .shift    = DMA_INT_STREAM1_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(1),
    },

    {
      .ctrl     = DMA1,
      .chan     = 2,
      .irq      = STM32_IRQ_DMA1S2,
      .shift    = DMA_INT_STREAM2_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(2),
    },

    {
      .ctrl     = DMA1,
      .chan     = 3,
      .irq      = STM32_IRQ_DMA1S3,
      .shift    = DMA_INT_STREAM3_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(3),
    },

    {
      .ctrl     = DMA1,
      .chan     = 4,
      .irq      = STM32_IRQ_DMA1S4,
      .shift    = DMA_INT_STREAM4_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(4),
    },

    {
      .ctrl     = DMA1,
      .chan     = 5,
      .irq      = STM32_IRQ_DMA1S5,
      .shift    = DMA_INT_STREAM5_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(5),
    },

    {
      .ctrl     = DMA1,
      .chan     = 6,
      .irq      = STM32_IRQ_DMA1S6,
      .shift    = DMA_INT_STREAM6_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(6),
    },

    {
      .ctrl     = DMA1,
      .chan     = 7,
      .irq      = STM32_IRQ_DMA1S7,
      .shift    = DMA_INT_STREAM7_SHIFT,
      .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(7),
    },
#endif

#ifdef CONFIG_STM32H7_DMA2
  /* DMA2 */

    {
      .ctrl     = DMA2,
      .chan     = 0,
      .irq      = STM32_IRQ_DMA2S0,
      .shift    = DMA_INT_STREAM0_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(0),
    },

    {
      .ctrl     = DMA2,
      .chan     = 1,
      .irq      = STM32_IRQ_DMA2S1,
      .shift    = DMA_INT_STREAM1_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(1),
    },

    {
      .ctrl     = DMA2,
      .chan     = 2,
      .irq      = STM32_IRQ_DMA2S2,
      .shift    = DMA_INT_STREAM2_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(2),
    },

    {
      .ctrl     = DMA2,
      .chan     = 3,
      .irq      = STM32_IRQ_DMA2S3,
      .shift    = DMA_INT_STREAM3_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(3),
    },

    {
      .ctrl     = DMA2,
      .chan     = 4,
      .irq      = STM32_IRQ_DMA2S4,
      .shift    = DMA_INT_STREAM4_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(4),
    },

    {
      .ctrl     = DMA2,
      .chan     = 5,
      .irq      = STM32_IRQ_DMA2S5,
      .shift    = DMA_INT_STREAM5_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(5),
    },

    {
      .ctrl     = DMA2,
      .chan     = 6,
      .irq      = STM32_IRQ_DMA2S6,
      .shift    = DMA_INT_STREAM6_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(6),
    },

    {
      .ctrl     = DMA2,
      .chan     = 7,
      .irq      = STM32_IRQ_DMA2S7,
      .shift    = DMA_INT_STREAM7_SHIFT,
      .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(7),
    },
#endif

#ifdef CONFIG_STM32H7_BDMA
  /* BDMA */

    {
      .ctrl     = BDMA,
      .chan     = 0,
      .irq      = STM32_IRQ_BDMACH1,
      .shift    = BDMA_CHAN_SHIFT(0),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(0),
    },

    {
      .ctrl     = BDMA,
      .chan     = 1,
      .irq      = STM32_IRQ_BDMACH2,
      .shift    = BDMA_CHAN_SHIFT(1),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(1),
    },

    {
      .ctrl     = BDMA,
      .chan     = 2,
      .irq      = STM32_IRQ_BDMACH3,
      .shift    = BDMA_CHAN_SHIFT(2),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(2),
    },

    {
      .ctrl     = BDMA,
      .chan     = 3,
      .irq      = STM32_IRQ_BDMACH4,
      .shift    = BDMA_CHAN_SHIFT(3),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(3),
    },

    {
      .ctrl     = BDMA,
      .chan     = 4,
      .irq      = STM32_IRQ_BDMACH5,
      .shift    = BDMA_CHAN_SHIFT(4),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(4),
    },

    {
      .ctrl     = BDMA,
      .chan     = 5,
      .irq      = STM32_IRQ_BDMACH6,
      .shift    = BDMA_CHAN_SHIFT(5),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(5),
    },

    {
      .ctrl     = BDMA,
      .chan     = 6,
      .irq      = STM32_IRQ_BDMACH7,
      .shift    = BDMA_CHAN_SHIFT(6),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(6),
    },

    {
      .ctrl     = BDMA,
      .chan     = 7,
      .irq      = STM32_IRQ_BDMACH8,
      .shift    = BDMA_CHAN_SHIFT(7),
      .base     = STM32_BDMA_BASE + STM32_BDMA_OFFSET(7),
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
 * Name: stm32_dma_channel_get
 *
 * Description:
 *  Get the g_dmach table entry associated with a given DMA controller
 *  and channel number.
 *
 ****************************************************************************/

static DMA_CHANNEL stm32_dma_channel_get(uint8_t channel, uint8_t controller)
{
  uint8_t first = 0;
  uint8_t nchan = 0;

  /* Get limits for g_dma array */

  stm32_gdma_limits_get(controller, &first, &nchan);

  DEBUGASSERT(channel <= nchan);

  return &g_dmach[first + channel];
}

/****************************************************************************
 * Name: stm32_gdma_limits_get
 *
 * Description:
 *  Get g_dma array limits for a given DMA controller.
 *
 ****************************************************************************/

static void stm32_gdma_limits_get(uint8_t controller, uint8_t *first,
                                  uint8_t *nchan)
{
  DEBUGASSERT(first != NULL);
  DEBUGASSERT(nchan != NULL);

  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  *first = g_dma[controller].first;
  *nchan  = g_dma[controller].nchan;
}

/****************************************************************************
 * Master DMA functions
 ****************************************************************************/

#ifdef CONFIG_STM32H7_MDMA

/****************************************************************************
 * Name: stm32_mdma_disable
 *
 * Description:
 *  Disable the master DMA
 *
 ****************************************************************************/

static void stm32_mdma_disable(DMA_CHANNEL dmachan)
{
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

#warning stm32_mdma_disable
}

/****************************************************************************
 * Name: stm32_mdma_interrupt
 *
 * Description:
 *  Master DMA interrupt handler
 *
 ****************************************************************************/

static int stm32_mdma_interrupt(int irq, void *context, void *arg)
{
#warning stm32_mdma_interrupt
}

/****************************************************************************
 * Name: stm32_mdma_setup
 *
 * Description:
 *   Configure master DMA before using
 *
 ****************************************************************************/

static void stm32_mdma_setup(DMA_HANDLE handle, stm32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

#warning stm32_mdma_setup not implemented
}

/****************************************************************************
 * Name: stm32_mdma_start
 *
 * Description:
 *   Start the master DMA transfer
 ****************************************************************************/

static void stm32_mdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg, bool half)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

#warning stm32_mdma_start not implemented
}

/****************************************************************************
 * Name: stm32_mdma_residual
 ****************************************************************************/

static size_t stm32_mdma_residual(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;
  uint32_t    residual   = 0;

  DEBUGASSERT(controller == MDMA);

  /* REVISIT */

  /* Fetch the count of blocks remaining to be transferred */

  residual = dmachan_getreg(dmachan, STM32_MDMACH_CBNDTR_OFFSET);

  return (size_t)(residual & MDMA_CBNDTR_BNDT_MASK);
}

/****************************************************************************
 * Name: stm32_mdma_capable
 ****************************************************************************/

#ifdef CONFIG_STM32H7_DMACAPABLE
static bool stm32_mdma_capable(stm32_dmacfg_t *cfg)
{
  uint32_t transfer_size;
  uint32_t mend;
  uint32_t ccr  = cfg->cfg1;
  uint32_t ctcr = cfg->cfg2;

  dmainfo("0x%08" PRIx32 "/%" PRIu32 " 0x%08" PRIx32 " 0x%08" PRIx32 "\n",
          cfg->maddr, cfg->ndata, ccr, ctcr);

#warning stm32_mdma_capable not implemented

  return true;
}
#endif

/****************************************************************************
 * Name: stm32_mdma_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32_mdma_dump(DMA_HANDLE handle, const char *msg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == MDMA);

  dmainfo("   CISR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CISR_OFFSET));
  dmainfo("   CESR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CESR_OFFSET));
  dmainfo("   CCR:    %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CCR_OFFSET));
  dmainfo("   CTCR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CTCR_OFFSET));
  dmainfo("   CBNDTR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CBNDTR_OFFSET));
  dmainfo("   CSAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CSAR_OFFSET));
  dmainfo("   CDAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CDAR_OFFSET));
  dmainfo("   CBRUR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CBRUR_OFFSET));
  dmainfo("   CLAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CLAR_OFFSET));
  dmainfo("   CTBR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CTBR_OFFSET));
  dmainfo("   CMAR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CMAR_OFFSET));
  dmainfo("   CMDR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_MDMACH_CMDR_OFFSET));
}
#endif

#endif /* CONFIG_STM32H7_MDMA */

/****************************************************************************
 * Standard DMA functions
 ****************************************************************************/

#if defined(CONFIG_STM32H7_DMA1) || defined(CONFIG_STM32H7_DMA2)

/****************************************************************************
 * Name: stm32_sdma_disable
 *
 * Description:
 *  Disable standard DMA stream (DMA1/DMA2)
 *
 ****************************************************************************/

static void stm32_sdma_disable(DMA_CHANNEL dmachan)
{
  uint32_t regoffset  = 0;
  uint32_t regval     = 0;
  uint8_t  stream     = 0;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  /* Get DMA stream */

  stream = dmachan->chan;

  DEBUGASSERT(stream < 8);

  /* Disable all interrupts at the DMA controller */

  regval = dmachan_getreg(dmachan, STM32_DMA_SCR_OFFSET);
  regval &= ~DMA_SCR_ALLINTS;

  /* Disable the DMA stream */

  regval &= ~DMA_SCR_EN;
  dmachan_putreg(dmachan, STM32_DMA_SCR_OFFSET, regval);

  /* Clear pending stream interrupts by setting bits in the upper or lower
   * IFCR register
   */

  if (stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }

  dmabase_putreg(dmachan, regoffset, (DMA_STREAM_MASK << dmachan->shift));
}

/****************************************************************************
 * Name: stm32_sdma_interrupt
 *
 * Description:
 *  Standard DMA interrupt handler
 *
 ****************************************************************************/

static int stm32_sdma_interrupt(int irq, void *context, void *arg)
{
  DMA_CHANNEL dmachan     = NULL;
  uint32_t    status      = 0;
  uint32_t    regoffset   = 0;
  uint8_t     stream      = 0;
  uint8_t     controller  = 0;

  /* Get the stream and the controller that generated the interrupt */

#ifdef CONFIG_STM32H7_DMA1
  if (irq >= STM32_IRQ_DMA1S0 && irq <= STM32_IRQ_DMA1S6)
    {
      stream     = irq - STM32_IRQ_DMA1S0;
      controller = DMA1;
    }
  else if (irq == STM32_IRQ_DMA1S7)
    {
      stream     = 7;
      controller = DMA1;
    }
  else
#endif
#ifdef CONFIG_STM32H7_DMA2
  if (irq >= STM32_IRQ_DMA2S0 && irq <= STM32_IRQ_DMA2S4)
    {
      stream     = irq - STM32_IRQ_DMA2S0;
      controller = DMA2;
    }
  else if (irq >= STM32_IRQ_DMA2S5 && irq <= STM32_IRQ_DMA2S7)
    {
      stream     = irq - STM32_IRQ_DMA2S5 + 5;
      controller = DMA2;
    }
  else
#endif
    {
      DEBUGPANIC();
    }

  /* Get the channel structure from the stream and controller numbers */

  dmachan = stm32_dma_channel_get(stream, controller);

  /* Select the interrupt status register (either the LISR or HISR)
   * based on the stream number that caused the interrupt.
   */

  if (stream < 4)
    {
      regoffset = STM32_DMA_LISR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HISR_OFFSET;
    }

  /* Get the interrupt status for this stream */

  status = (dmabase_getreg(dmachan, regoffset) >> dmachan->shift)
            & DMA_STREAM_MASK;

  /* Clear fetched stream interrupts by setting bits in the upper or lower
   * IFCR register
   */

  if (stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }

  dmabase_putreg(dmachan, regoffset, (status << dmachan->shift));

  /* Invoke the callback */

  if (dmachan->callback)
    {
      dmachan->callback(dmachan, status, dmachan->arg);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_sdma_setup
 *
 * Description:
 *   Configure standard DMA before using
 *
 ****************************************************************************/

static void stm32_sdma_setup(DMA_HANDLE handle, stm32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint32_t regoffset  = 0;
  uint32_t regval     = 0;
  uint8_t  stream     = 0;
  uint32_t scr        = cfg->cfg1;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  dmainfo("paddr: %08" PRIx32 " maddr: %08" PRIx32 " ndata: %" PRIu32 " "
          "scr: %08" PRIx32 "\n",
          cfg->paddr, cfg->maddr, cfg->ndata, cfg->cfg1);

#ifdef CONFIG_STM32H7_DMACAPABLE
  DEBUGASSERT(stm32_sdma_capable(cfg));
#endif

  /* "If the stream is enabled, disable it by resetting the EN bit in the
   * DMA_SxCR register, then read this bit in order to confirm that there is
   * no ongoing stream operation. Writing this bit to 0 is not immediately
   * effective since it is actually written to 0 once all the current
   * transfers have finished. When the EN bit is read as 0, this means that
   * the stream is ready to be configured. It is therefore necessary to wait
   * for the EN bit to be cleared before starting any stream configuration."
   */

  while ((dmachan_getreg(dmachan, STM32_DMA_SCR_OFFSET) & DMA_SCR_EN) != 0);

  /* "... All the stream dedicated bits set in the status register (DMA_LISR
   * and DMA_HISR) from the previous data block DMA transfer should be
   * cleared before the stream can be re-enabled."
   *
   * Clear pending stream interrupts by setting bits in the upper or lower
   * IFCR register
   */

  stream = dmachan->chan;

  if (stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }

  dmabase_putreg(dmachan, regoffset, (DMA_STREAM_MASK << dmachan->shift));

  /* "Set the peripheral register address in the DMA_SPARx register. The data
   *  will be moved from/to this address to/from the memory after the
   *  peripheral event.
   */

  dmachan_putreg(dmachan, STM32_DMA_SPAR_OFFSET, cfg->paddr);

  /* "Set the memory address in the DMA_SM0ARx ... register. The data will be
   *  written to or read from this memory after the peripheral event."
   *
   * Note that in double-buffered mode it is explicitly assumed that the
   * second buffer immediately follows the first.
   */

  dmachan_putreg(dmachan, STM32_DMA_SM0AR_OFFSET, cfg->maddr);
  if (scr & DMA_SCR_DBM)
    {
      dmachan_putreg(dmachan, STM32_DMA_SM1AR_OFFSET,
                     cfg->maddr + cfg->ndata);
    }

  /* "Configure the total number of data items to be transferred in the
   *  DMA_SNDTRx register.  After each peripheral event, this value will be
   *  decremented."
   *
   * "When the peripheral flow controller is used for a given stream, the
   *  value written into the DMA_SxNDTR has no effect on the DMA transfer.
   *  Actually, whatever the value written, it will be forced by hardware to
   *  0xFFFF as soon as the stream is enabled..."
   */

  dmachan_putreg(dmachan, STM32_DMA_SNDTR_OFFSET, cfg->ndata);

  /* "Configure the stream priority using the PL[1:0] bits in the DMA_SCRx"
   *  register."
   */

  regval  = dmachan_getreg(dmachan, STM32_DMA_SCR_OFFSET);
  regval &= ~(DMA_SCR_PL_MASK);
  regval |= scr & DMA_SCR_PL_MASK;
  dmachan_putreg(dmachan, STM32_DMA_SCR_OFFSET, regval);

  /* "Configure the FIFO usage (enable or disable, threshold in transmission
   * and reception)"
   *
   * "Caution is required when choosing the FIFO threshold (bits FTH[1:0] of
   * the DMA_SxFCR register) and the size of the memory burst (MBURST[1:0] of
   * the DMA_SxCR register): The content pointed by the FIFO threshold must
   * exactly
   *  match to an integer number of memory burst transfers. If this is not in
   *  the case, a FIFO error (flag FEIFx of the DMA_HISR or DMA_LISR
   *  register) will be generated when the stream is enabled, then the stream
   *  will be automatically disabled."
   *
   * The FIFO is disabled in circular mode when transferring data from a
   * peripheral to memory, as in this case it is usually desirable to know
   * that every byte from the peripheral is transferred immediately to
   * memory. It is not practical to flush the DMA FIFO, as this requires
   * disabling the channel which triggers the transfer-complete interrupt.
   *
   * NOTE: The FEIFx error interrupt is not enabled because the FEIFx seems
   * to be reported spuriously causing good transfers to be marked as
   * failures.
   */

  regval  = dmachan_getreg(dmachan, STM32_DMA_SFCR_OFFSET);
  regval &= ~(DMA_SFCR_FTH_MASK | DMA_SFCR_FS_MASK | DMA_SFCR_FEIE);
  if (!((scr & (DMA_SCR_CIRC | DMA_SCR_DIR_MASK)) ==
      (DMA_SCR_CIRC | DMA_SCR_DIR_P2M)))
    {
      regval |= (DMA_SFCR_FTH_FULL | DMA_SFCR_DMDIS);
    }

  dmachan_putreg(dmachan, STM32_DMA_SFCR_OFFSET, regval);

  /* "Configure data transfer direction, circular mode, peripheral & memory
   *  incremented mode, peripheral & memory data size, and interrupt after
   *  half and/or full transfer in the DMA_CCRx register."
   *
   * Note: The CT bit is always reset.
   */

  regval  = dmachan_getreg(dmachan, STM32_DMA_SCR_OFFSET);
  regval &= ~(DMA_SCR_PFCTRL | DMA_SCR_DIR_MASK | DMA_SCR_PINC |
              DMA_SCR_MINC | DMA_SCR_PSIZE_MASK | DMA_SCR_MSIZE_MASK |
              DMA_SCR_PINCOS | DMA_SCR_CIRC | DMA_SCR_DBM | DMA_SCR_CT |
              DMA_SCR_PBURST_MASK | DMA_SCR_MBURST_MASK);
  scr    &=  (DMA_SCR_PFCTRL | DMA_SCR_DIR_MASK | DMA_SCR_PINC |
              DMA_SCR_MINC | DMA_SCR_PSIZE_MASK | DMA_SCR_MSIZE_MASK |
              DMA_SCR_PINCOS | DMA_SCR_DBM | DMA_SCR_CIRC |
              DMA_SCR_PBURST_MASK | DMA_SCR_MBURST_MASK | DMA_SCR_TRBUFF);
  regval |= scr;
  dmachan_putreg(dmachan, STM32_DMA_SCR_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32_sdma_start
 *
 * Description:
 *   Start the standard DMA transfer
 ****************************************************************************/

static void stm32_sdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg, bool half)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint32_t scr = 0;

  DEBUGASSERT(handle != NULL);
  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  /* Save the callback info.  This will be invoked when the DMA completes */

  dmachan->callback = callback;
  dmachan->arg      = arg;

  /* Activate the stream by setting the ENABLE bit in the DMA_SCRx register.
   * As soon as the stream is enabled, it can serve any DMA request from the
   * peripheral connected on the stream.
   */

  scr  = dmachan_getreg(dmachan, STM32_DMA_SCR_OFFSET);
  scr |= DMA_SCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * and double-buffered modes, always interrupt on buffer wrap, and
   * optionally interrupt at the halfway point.
   */

  if ((scr & (DMA_SCR_DBM | DMA_SCR_CIRC)) == 0)
    {
      /* Once half of the bytes are transferred, the half-transfer flag
       * (HTIF) is set and an interrupt is generated if the
       * Half-Transfer Interrupt Enable bit (HTIE) is set. At the end of the
       * transfer, the Transfer Complete Flag (TCIF) is set and an interrupt
       * is generated if the Transfer Complete Interrupt Enable bit (TCIE) is
       * set.
       */

      scr |= (half ? (DMA_SCR_HTIE | DMA_SCR_TEIE) :
                      (DMA_SCR_TCIE | DMA_SCR_TEIE));
    }
  else
    {
      /* In non-stop modes, when the transfer completes it immediately resets
       * and starts again.  The transfer-complete interrupt is thus always
       * enabled, and the half-complete interrupt can be used in circular
       * mode to determine when the buffer is half-full, or in
       * double-buffered mode to determine when one of the two buffers is
       * full
       */

      scr |= (half ? DMA_SCR_HTIE : 0) | DMA_SCR_TCIE | DMA_SCR_TEIE;
    }

  dmachan_putreg(dmachan, STM32_DMA_SCR_OFFSET, scr);

  stm32_dmadump(handle, "DMA after start");
}

/****************************************************************************
 * Name: stm32_sdma_residual
 ****************************************************************************/

static size_t stm32_sdma_residual(DMA_HANDLE handle)
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

  residual = dmachan_getreg(dmachan, STM32_DMA_SNDTR_OFFSET);

  return (size_t)(residual & DMA_SNDTR_NDT_MASK);
}

/****************************************************************************
 * Name: stm32_sdma_capable
 ****************************************************************************/

#ifdef CONFIG_STM32H7_DMACAPABLE
static bool stm32_sdma_capable(stm32_dmacfg_t *cfg)
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
          dmainfo("stm32_dmacapable: bad transfer size in CCR\n");
          return false;
        }
    }

  if ((cfg->maddr & (transfer_size - 1)) != 0)
    {
      dmainfo("stm32_dmacapable: transfer unaligned\n");
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
          dmainfo("stm32_dmacapable: dcache unaligned "
                  "maddr:0x%08" PRIx32 " mend:0x%08" PRIx32 "\n",
                  cfg->maddr, mend);
#if !defined(CONFIG_STM32H7_DMACAPABLE_ASSUME_CACHE_ALIGNED)
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
              dmainfo("stm32_dmacapable: bad burst size in CCR\n");
              return false;
            }
        }

      if ((cfg->maddr & (burst_length - 1)) != 0)
        {
          dmainfo("stm32_dmacapable: burst crosses 1KiB\n");
          return false;
        }
    }

  /* Verify that transfer is froma a supported memory region */

  if ((cfg->paddr & STM32_PREGION_MASK) != STM32_D2_BASE)
    {
      /* DMA1/DMA2 support only D2 domain */

      dmainfo("transfer from unknown/unsupported region\n");
      return false;
    }

  /* Verify that the transfer is to a memory region that supports DMA. */

  if ((cfg->maddr & STM32_REGION_MASK) != (mend & STM32_REGION_MASK))
    {
      dmainfo("stm32_dmacapable: transfer crosses memory region\n");
      return false;
    }

  switch (cfg->maddr & STM32_REGION_MASK)
    {
      case STM32_AXISRAM_BASE:
      case STM32_FMC_BANK1:
      case STM32_FMC_BANK2:
      case STM32_FMC_BANK3:
      case STM32_FMC_BANK4:
      case STM32_FMC_BANK5:
      case STM32_FMC_BANK6:
        {
          /* All RAM and FMC is supported */

          break;
        }

      case STM32_SRAM_BASE:
        {
          /* DTCM not supported for standard DMA (DMA1/DMA2) */

          if (cfg->maddr >= STM32_DTCRAM_BASE
              && (cfg->maddr - STM32_DTCRAM_BASE) < 0x1ffff)
            {
              dmainfo("transfer targets DTCRAM\n");
              return false;
            }
          break;
        }

      case STM32_CODE_BASE:
        {
          /* ITCM not supported for standard DMA (DMA1/DMA2) */

          if (cfg->maddr >= STM32_ITCM_BASE
              && (cfg->maddr - STM32_ITCM_BASE) < 0xffff)
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
 * Name: stm32_sdma_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32_sdma_dump(DMA_HANDLE handle, const char *msg)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  dmainfo("   LISR: %08" PRIx32 "\n",
          dmabase_getreg(dmachan, STM32_DMA_LISR_OFFSET));
  dmainfo("   HISR: %08" PRIx32 "\n",
          dmabase_getreg(dmachan, STM32_DMA_HISR_OFFSET));
  dmainfo("   SCR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_DMA_SCR_OFFSET));
  dmainfo("   SNDTR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_DMA_SNDTR_OFFSET));
  dmainfo("   SPAR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_DMA_SPAR_OFFSET));
  dmainfo("   SM0AR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_DMA_SM0AR_OFFSET));
  dmainfo("   SM1AR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_DMA_SM1AR_OFFSET));
  dmainfo("   SFCR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_DMA_SFCR_OFFSET));

  stm32_dmamux_dump(g_dma[dmachan->ctrl].dmamux,
                    dmachan->chan + g_dma[dmachan->ctrl].dmamux_offset);
}
#endif

#endif /* CONFIG_STM32H7_DMA1 || CONFIG_STM32H7_DMA2 */

/****************************************************************************
 * Basic DMA functions
 ****************************************************************************/

#ifdef CONFIG_STM32H7_BDMA

/****************************************************************************
 * Name: stm32_bdma_channel_disable
 *
 * Description:
 *  Disable the Basic DMA
 *
 ****************************************************************************/

static void stm32_bdma_disable(DMA_CHANNEL dmachan)
{
  uint32_t regval = 0;

  DEBUGASSERT(dmachan->ctrl == BDMA);
  DEBUGASSERT(dmachan->chan < BDMA_NCHAN);

  /* Disable all interrupts at the DMA controller */

  regval = dmachan_getreg(dmachan, STM32_BDMACH_CCR_OFFSET);
  regval &= ~BDMA_CCR_ALLINTS;

  /* Disable the DMA stream */

  regval &= ~BDMA_CCR_EN;
  dmachan_putreg(dmachan, STM32_BDMACH_CCR_OFFSET, regval);

  dmabase_putreg(dmachan, STM32_BDMA_IFCR_OFFSET,
                   (BDMA_CHAN_MASK << dmachan->shift));
}

/****************************************************************************
 * Name: stm32_bdma_interrupt
 *
 * Description:
 *  Basic DMA interrupt handler
 *
 ****************************************************************************/

static int stm32_bdma_interrupt(int irq, void *context, void *arg)
{
  DMA_CHANNEL dmachan     = NULL;
  uint32_t    status      = 0;
  uint32_t    scrstatus   = 0;
  uint8_t     stream      = irq - STM32_IRQ_BDMACH1;
  uint8_t     controller  = BDMA;

  /* Get the channel structure from the stream and controller numbers */

  dmachan = stm32_dma_channel_get(stream, controller);

  /* Get the interrupt status for this stream */

  status = (dmabase_getreg(dmachan, STM32_BDMA_ISR_OFFSET) >> dmachan->shift)
            & BDMA_CHAN_MASK;

  dmabase_putreg(dmachan, STM32_BDMA_IFCR_OFFSET,
                 (status << dmachan->shift));

  /* Invoke the callback */

  if (dmachan->callback)
    {
      /* Map to the SDMA status */

      scrstatus  = (status & BDMA_CHAN_TEIF) ? DMA_STREAM_FEIF_BIT : 0;
      scrstatus |= (status & BDMA_CHAN_TCIF) ? DMA_STREAM_TCIF_BIT : 0;
      scrstatus |= (status & BDMA_CHAN_HTIF) ? DMA_STREAM_HTIF_BIT : 0;
      dmachan->callback(dmachan, scrstatus, dmachan->arg);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_sdma_scr_2_bdma_ccr
 *
 * Description:
 *   Maps the DMA SCR bits to the BDMA CCR bits
 *
 ****************************************************************************/

static inline int32_t stm32_sdma_scr_2_bdma_ccr(int32_t scr)
{
  uint32_t ccr = 0;
  ccr |= (scr & DMA_SCR_CT) ? BDMA_CCR_CT : 0;
  ccr |= (scr & DMA_SCR_DBM) ? BDMA_CCR_DBM : 0;
  ccr |= (scr & DMA_SCR_PL_MASK) >> (DMA_SCR_PL_SHIFT - BDMA_CCR_PRILO);
  ccr |= (scr & DMA_SCR_MSIZE_MASK) >>
               (DMA_SCR_MSIZE_SHIFT - BDMA_CCR_MSIZE_SHIFT);
  ccr |= (scr & DMA_SCR_PSIZE_MASK) >>
               (DMA_SCR_PSIZE_SHIFT - BDMA_CCR_PSIZE_SHIFT);
  ccr |= (scr & DMA_SCR_MINC) ? BDMA_CCR_MINC : 0;
  ccr |= (scr & DMA_SCR_PINC) ? BDMA_CCR_PINC : 0;
  ccr |= (scr & DMA_SCR_CIRC) ? BDMA_CCR_CIRC : 0;
  ccr |= (scr & DMA_SCR_DIR_M2P) ? BDMA_CCR_DIR : 0;
  ccr |= (scr & DMA_SCR_DIR_M2M) ? BDMA_CCR_M2M : 0;
  ccr |= (scr & DMA_SCR_TCIE) ? BDMA_CCR_TCIE : 0;
  ccr |= (scr & DMA_SCR_HTIE) ? BDMA_CCR_HTIE : 0;
  ccr |= (scr & DMA_SCR_TEIE) ? BDMA_CCR_TEIE : 0;
  return ccr;
}

/****************************************************************************
 * Name: stm32_bdma_setup
 *
 * Description:
 *   Configure basic DMA before using
 *
 ****************************************************************************/

static void stm32_bdma_setup(DMA_HANDLE handle, stm32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint32_t    regval     = 0;
  uint32_t    scr        = cfg->cfg1;
  uint32_t    ccr        = 0;

  DEBUGASSERT(handle != NULL);
  DEBUGASSERT(dmachan->ctrl == BDMA);

  dmainfo("paddr: %08" PRIx32 " maddr: %08" PRIx32 " ndata: %" PRIu32 " "
          "scr: %08" PRIx32 "\n",
          cfg->paddr, cfg->maddr, cfg->ndata, cfg->cfg1);

#ifdef CONFIG_STM32H7_DMACAPABLE
  DEBUGASSERT(stm32_bdma_capable(cfg));
#endif

  /* "If the stream is enabled, disable it by resetting the EN bit in the
   * DMA_SxCR register, then read this bit in order to confirm that there is
   * no ongoing stream operation. Writing this bit to 0 is not immediately
   * effective since it is actually written to 0 once all the current
   * transfers have finished. When the EN bit is read as 0, this means that
   * the stream is ready to be configured. It is therefore necessary to wait
   * for the EN bit to be cleared before starting any stream configuration."
   */

  regval =  dmachan_getreg(dmachan, STM32_BDMACH_CCR_OFFSET);
  regval &= ~BDMA_CCR_EN;
  dmachan_putreg(dmachan, STM32_BDMACH_CCR_OFFSET, regval);

  while ((dmachan_getreg(dmachan, STM32_BDMACH_CCR_OFFSET) &
          BDMA_CCR_EN) != 0);

  /* "... All the stream dedicated bits set in the status register BDMA_ISR
   * from the previous data block DMA transfer should be cleared before the
   * stream can be re-enabled."
   *
   * Clear pending stream interrupts by setting bits in the BDMA_IFCR
   * register.
   */

  dmabase_putreg(dmachan, STM32_BDMA_IFCR_OFFSET,
                 (BDMA_CHAN_MASK << dmachan->shift));

  /* "Set the peripheral register address in the BDMA_SPARx register. The
   * data will be moved from/to this address to/from the memory after the
   *  peripheral event.
   */

  dmachan_putreg(dmachan, STM32_BDMACH_CPAR_OFFSET, cfg->paddr);

  /* "Set the memory address in the BDMA_CM1ARx register. The data will be
   *  written to or read from this memory after the peripheral event."
   *
   * Note that in double-buffered mode it is explicitly assumed that the
   * second buffer immediately follows the first.
   */

  dmachan_putreg(dmachan, STM32_BDMACH_CM0AR_OFFSET, cfg->maddr);
  if (scr & DMA_SCR_DBM)
    {
      dmachan_putreg(dmachan, STM32_BDMACH_CM1AR_OFFSET,
                     cfg->maddr + cfg->ndata);
    }

  /* "Configure the total number of data items to be transferred in the
   *  BDMA_CNDTRx register.  After each peripheral event, this value will be
   *  decremented."
   */

  dmachan_putreg(dmachan, STM32_BDMACH_CNDTR_OFFSET, cfg->ndata);

  /* "Configure the stream priority using the PL[1:0] bits, data transfer
   *  direction, circular mode, peripheral & memory incremented mode,
   *  peripheral & memory data size, and interrupt after
   *  half and/or full transfer in the BDMACH_CCRx register."
   *
   * Note: The CT bit is always reset.
   */

  regval  = dmachan_getreg(dmachan, STM32_BDMACH_CCR_OFFSET);
  regval &= ~(BDMA_CCR_DIR | BDMA_CCR_CIRC | BDMA_CCR_PINC |
              BDMA_CCR_MINC | BDMA_CCR_PSIZE_MASK | BDMA_CCR_MSIZE_MASK |
              BDMA_CCR_PL_MASK | BDMA_CCR_M2M | BDMA_CCR_DBM | BDMA_CCR_CT);
  ccr = stm32_sdma_scr_2_bdma_ccr(scr);
  ccr    &=  (BDMA_CCR_DIR | BDMA_CCR_CIRC | BDMA_CCR_PINC |
              BDMA_CCR_MINC | BDMA_CCR_PSIZE_MASK | BDMA_CCR_MSIZE_MASK |
              BDMA_CCR_PL_MASK | BDMA_CCR_M2M | BDMA_CCR_DBM);
  regval |= ccr;
  dmachan_putreg(dmachan, STM32_BDMACH_CCR_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32_bdma_start
 *
 * Description:
 *   Start the basic DMA transfer
 *
 ****************************************************************************/

static void stm32_bdma_start(DMA_HANDLE handle, dma_callback_t callback,
                             void *arg, bool half)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint32_t ccr = 0;

  DEBUGASSERT(handle != NULL);
  DEBUGASSERT(dmachan->ctrl == BDMA);

  /* Save the callback info.  This will be invoked when the DMA completes */

  dmachan->callback = callback;
  dmachan->arg      = arg;

  /* Activate the stream by setting the ENABLE bit in the DMA_SCRx register.
   * As soon as the stream is enabled, it can serve any DMA request from the
   * peripheral connected on the stream.
   */

  ccr  = dmachan_getreg(dmachan, STM32_BDMACH_CCR_OFFSET);
  ccr |= BDMA_CCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * and double-buffered modes, always interrupt on buffer wrap, and
   * optionally interrupt at the halfway point.
   */

  if ((ccr & (BDMA_CCR_DBM | BDMA_CCR_CIRC)) == 0)
    {
      /* Once half of the bytes are transferred, the half-transfer flag
       * (HTIF) is set and an interrupt is generated if the
       * Half-Transfer Interrupt Enable bit (HTIE) is set. At the end of the
       * transfer, the Transfer Complete Flag (TCIF) is set and an interrupt
       * is generated if the Transfer Complete Interrupt Enable bit (TCIE) is
       * set.
       */

      ccr |= (half ? (BDMA_CCR_HTIE | BDMA_CCR_TEIE) :
                     (BDMA_CCR_TCIE | BDMA_CCR_TEIE));
    }
  else
    {
      /* In non-stop modes, when the transfer completes it immediately resets
       * and starts again.  The transfer-complete interrupt is thus always
       * enabled, and the half-complete interrupt can be used in circular
       * mode to determine when the buffer is half-full, or in
       * double-buffered mode to determine when one of the two buffers is
       * full
       */

      ccr |= (half ? BDMA_CCR_HTIE : 0) | BDMA_CCR_TCIE | BDMA_CCR_TEIE;
    }

  dmachan_putreg(dmachan, STM32_BDMACH_CCR_OFFSET, ccr);

  stm32_dmadump(handle, "DMA after start");
}

/****************************************************************************
 * Name: stm32_bdma_residual
 ****************************************************************************/

static size_t stm32_bdma_residual(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint32_t    residual   = 0;

  DEBUGASSERT(handle != NULL);
  DEBUGASSERT(dmachan->ctrl == BDMA);

  /* Fetch the count of bytes remaining to be transferred */

  residual = dmachan_getreg(dmachan, STM32_BDMACH_CNDTR_OFFSET);

  return (size_t)(residual & BDMA_CNDTR_NDT_MASK);
}

/****************************************************************************
 * Name: stm32_bdma_capable
 ****************************************************************************/

#ifdef CONFIG_STM32H7_DMACAPABLE
static bool stm32_bdma_capable(stm32_dmacfg_t *cfg)
{
  uint32_t transfer_size;
  uint32_t mend;
  uint32_t count = cfg->ndata;
  uint32_t maddr = cfg->maddr;
  uint32_t ccr   = cfg->cfg1;

  dmainfo("0x%08" PRIx32 "/%" PRIu32 " 0x%08" PRIx32 "\n",
          cfg->maddr, cfg->ndata, cfg->cfg1);

  /* Verify that the address conforms to the memory transfer size.
   * Transfers to/from memory performed by the BDMA controller are
   * required to be aligned to their size.
   *
   * See ST RM0090 rev4, section 9.3.11
   *
   * Compute mend inline to avoid a possible non-constant integer
   * multiply.
   */

  switch (ccr & BDMA_CCR_MSIZE_MASK)
    {
      case BDMA_CCR_MSIZE_8BITS:
        {
          transfer_size = 1;
          mend          = maddr + count - 1;
          break;
        }

      case BDMA_CCR_MSIZE_16BITS:
        {
          transfer_size = 2;
          mend          = maddr + (count << 1) - 1;
          break;
        }

      case BDMA_CCR_MSIZE_32BITS:
        {
          transfer_size = 4;
          mend          = maddr + (count << 2) - 1;
          break;
        }

      default:
        {
          return false;
        }
    }

  if ((maddr & (transfer_size - 1)) != 0)
    {
      return false;
    }

#  if defined(CONFIG_ARMV7M_DCACHE) &&                  \
      !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
  /* buffer alignment is required for DMA transfers with dcache in buffered
   * mode (not write-through) because a) arch_invalidate_dcache could lose
   * buffered writes and b) arch_flush_dcache could corrupt adjacent memory
   * if the maddr and the mend+1, the next next address are not on
   * ARMV7M_DCACHE_LINESIZE boundaries.
   */

  if ((cfg->maddr & (ARMV7M_DCACHE_LINESIZE - 1)) != 0 ||
      ((mend + 1) & (ARMV7M_DCACHE_LINESIZE - 1)) != 0)
    {
      dmainfo("stm32_dmacapable: dcache unaligned "
              "maddr:0x%08" PRIx32 " mend:0x%08" PRIx32 "\n",
              cfg->maddr, mend);
#if !defined(CONFIG_STM32H7_DMACAPABLE_ASSUME_CACHE_ALIGNED)
      return false;
#endif
    }
#  endif

  /* Verify that transfer is froma a supported memory region */

  if ((cfg->paddr & STM32_PREGION_MASK) != STM32_D3_BASE)
    {
      /* BDMA support only D3 domain */

      dmainfo("transfer from unknown/unsupported region\n");
      return false;
    }

  /* Verify that the transfer is to a memory region that supports DMA. */

  if ((maddr & STM32_REGION_MASK) != (mend & STM32_REGION_MASK))
    {
      return false;
    }

  switch (maddr & STM32_REGION_MASK)
    {
      case STM32_SRAM4_BASE:
        {
          /* BDMA transfers are limited to D3 domain resources.
           * NOTE: STM32_SRAM4_BASE region mask cover STM32_BBRAM_BASE mask
           */

          return true;
        }

      default:
        {
          /* Everything else is unsupported by DMA */

          return false;
        }
    }

  dmainfo("transfer OK\n");
  return true;
}
#endif

/****************************************************************************
 * Name: stm32_bdma_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32_bdma_dump(DMA_HANDLE handle, const char *msg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller == BDMA);

  dmainfo("   ISR:   %08" PRIx32 "\n",
          dmabase_getreg(dmachan, STM32_BDMA_ISR_OFFSET));
  dmainfo("   CCR:   %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_BDMACH_CCR_OFFSET));
  dmainfo("   CNDTR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_BDMACH_CNDTR_OFFSET));
  dmainfo("   CPAR:  %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_BDMACH_CPAR_OFFSET));
  dmainfo("   CM0AR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_BDMACH_CM0AR_OFFSET));
  dmainfo("   CM1AR: %08" PRIx32 "\n",
          dmachan_getreg(dmachan, STM32_BDMACH_CM1AR_OFFSET));

  stm32_dmamux_dump(g_dma[dmachan->ctrl].dmamux, controller);
}
#endif

#endif /* CONFIG_STM32H7_BDMA */

/****************************************************************************
 * Name: stm32_dmamux_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32_dmamux_dump(DMA_MUX dmamux, uint8_t chan)
{
  dmainfo("DMAMUX%" PRIu8 " CH=%" PRIu8 "\n", dmamux->id, chan);
  dmainfo("   CCR:   %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_CXCR_OFFSET(chan)));
  dmainfo("   CSR:   %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_CSR_OFFSET));
  dmainfo("   RG0CR: %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_RG0CR_OFFSET));
  dmainfo("   RG1CR: %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_RG1CR_OFFSET));
  dmainfo("   RG2CR: %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_RG2CR_OFFSET));
  dmainfo("   RG3CR: %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_RG3CR_OFFSET));
  dmainfo("   RGSR:  %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_RGSR_OFFSET));
  dmainfo("   RGCFR: %08" PRIx32 "\n",
          dmamux_getreg(dmamux, STM32_DMAMUX_RGCFR_OFFSET));
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem (DMA1, DMA2, MDMA and BDMA)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
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

      DEBUGASSERT(controller >= MDMA && controller <= BDMA);

      /* Attach standard DMA interrupt vectors */

      irq_attach(dmachan->irq, g_dma_ops[controller].dma_interrupt,
                 dmachan);

      /* Disable the DMA stream */

      g_dma_ops[controller].dma_disable(dmachan);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(dmachan->irq);

      /* Set the interrupt priority
       * TODO: refactor
       */

#ifdef CONFIG_ARCH_IRQPRIO
      switch (controller)
        {
#if defined(CONFIG_STM32H7_DMA1) || defined(CONFIG_STM32H7_DMA2)
          case DMA1:
          case DMA2:
            {
              up_prioritize_irq(dmachan->irq, CONFIG_DMA_PRI);
              break;
            }
#endif /* CONFIG_STM32H7_DMA1 && CONFIG_STM32H7_DMA2 */

#ifdef CONFIG_STM32H7_MDMA
          case MDMA:
            {
              up_prioritize_irq(dmachan->irq, CONFIG_MDMA_PRI);
              break;
            }
#endif /* CONFIG_STM32H7_MDMA */

#ifdef CONFIG_STM32H7_BDMA
          case BDMA:
            {
              up_prioritize_irq(dmachan->irq, CONFIG_BDMA_PRI);
              break;
            }
#endif /* CONFIG_STM32H7_BDMA */

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
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   It is common for standard DMA (DMA1, DMA2), master DMA (MDMA) and
 *   basic DMA (BDMA) controllers.
 *
 * Input Parameters:
 *   dmamap - Identifies the stream/channel resource. For the STM32 H7, this
 *     is a bit-encoded  value as provided by the DMAMAP_* definitions
 *     in chip/stm32h7xxxxxxx_dmamux.h
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

DMA_HANDLE stm32_dmachannel(unsigned int dmamap)
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
  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  /* Get DMAMUX channel from encoded DMAMAP value */

  dmamux_req = DMAMAP_REQUEST(dmamap);

  /* Get g_dma array limits for given controller */

  stm32_gdma_limits_get(controller, &first, &nchan);

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

      regval = dmamux_req << DMAMUX_CCR_DMAREQID_SHIFT;
      dmamux_putreg(dmamux, STM32_DMAMUX_CXCR_OFFSET(dmamux_chan), regval);

      /* DMAMUX Set RGCR register */

      regval = 0;
      dmamux_putreg(dmamux, STM32_DMAMUX_RGXCR_OFFSET(dmamux_chan), regval);
    }

errout:
  return (DMA_HANDLE)dmachan;
}

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until stm32_dmachannel() is called again to re-gain access to the
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

void stm32_dmafree(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;
  DMA_MUX     dmamux     = NULL;
  irqstate_t  flags;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  /* Get DMAMUX associated with DMA controller */

  dmamux = g_dma[controller].dmamux;

  /* No DMAMUX for Master DMA */

  if (dmamux != NULL)
    {
      uint8_t dmamux_chan = dmachan->chan + g_dma[controller].dmamux_offset;

      dmainfo("Free DMAMUX%" PRIu8 " CH %" PRIu8 "\n",
              dmamux->id, dmamux_chan);

      /* Clear DMAMUX CCR register associated with channel */

      dmamux_putreg(dmamux, STM32_DMAMUX_CXCR_OFFSET(dmamux_chan), 0);

      /* Clear DMAMUX RGCR register associated with channel */

      dmamux_putreg(dmamux, STM32_DMAMUX_RGXCR_OFFSET(dmamux_chan), 0);
    }

  /* Release the channel */

  flags = enter_critical_section();
  dmachan->used = false;
  leave_critical_section(flags);

  dmainfo("Unmapping DMAMUX(%" PRIu8 ")\n", dmachan->chan);
}

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, stm32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  /* Get DMA controller */

  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  g_dma_ops[controller].dma_setup(handle, cfg);
}

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  g_dma_ops[controller].dma_start(handle, callback, arg, half);
}

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  g_dma_ops[controller].dma_disable(dmachan);
}

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Read the DMA bytes-remaining register.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  return g_dma_ops[controller].dma_residual(handle);
}

/****************************************************************************
 * Name: stm32_dmacapable
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

#ifdef CONFIG_STM32H7_DMACAPABLE
bool stm32_dmacapable(DMA_HANDLE handle, stm32_dmacfg_t *cfg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  return g_dma_ops[controller].dma_capable(cfg);
}
#endif

/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmadump(DMA_HANDLE handle, const char *msg)
{
  DMA_CHANNEL dmachan    = (DMA_CHANNEL)handle;
  uint8_t     controller = dmachan->ctrl;

  DEBUGASSERT(controller >= MDMA && controller <= BDMA);

  dmainfo("DMA %" PRIu8 " CH%" PRIu8 " Registers: %s\n",
          dmachan->ctrl, dmachan->ctrl, msg);

  g_dma_ops[controller].dma_dump(handle, msg);
}
#endif
