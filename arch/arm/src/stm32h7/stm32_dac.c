/****************************************************************************
 * arch/arm/src/stm32h7/stm32_dac.c
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/analog/dac.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_dac.h"
#include "stm32_rcc.h"
#include "stm32_dma.h"
#include "stm32_tim.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Only DAC1 is supported, but it has two channels */

#if !defined(CONFIG_STM32_DAC1)
#  error "DAC1 must be enabled"
#endif

#if !defined(CONFIG_STM32_DAC1CH1) && !defined(CONFIG_STM32_DAC1CH2)
#  error "At least one DAC1 channel must be enabled"
#endif

/* DMA configuration per channel */

#ifdef CONFIG_STM32_DAC1CH1_DMA
#  if !defined(CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE) || CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE < 1
#    define CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE 1
#  endif
#  if !defined(CONFIG_STM32_DAC1CH1_TIMER)
#    warning "A timer number must be specified in CONFIG_STM32_DAC1CH1_TIMER"
#    undef CONFIG_STM32_DAC1CH1_DMA
#    undef CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY) || \
        (CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY < 1)
#    warning "A timer frequency (>0) must be specified in CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY"
#    undef CONFIG_STM32_DAC1CH1_DMA
#    undef CONFIG_STM32_DAC1CH1_TIMER
#  endif
#endif

#ifdef CONFIG_STM32_DAC1CH2_DMA
#  if !defined(CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE) || CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE < 1
#    define CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE 1
#  endif
#  if !defined(CONFIG_STM32_DAC1CH2_TIMER)
#    warning "A timer number must be specified in CONFIG_STM32_DAC1CH2_TIMER"
#    undef CONFIG_STM32_DAC1CH2_DMA
#    undef CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY) || \
        (CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY < 1)
#    warning "A timer frequency (>0) must be specified in CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY"
#    undef CONFIG_STM32_DAC1CH2_DMA
#    undef CONFIG_STM32_DAC1CH2_TIMER
#  endif
#endif

/* Select DMA channels for each DAC channel */

#ifdef CONFIG_STM32_DAC1CH1_DMA
#  if defined(CONFIG_STM32_DMA1)
#    define DAC1_CH1_DMA_CHAN   DMAMAP_DMA12_DAC1CH1_0
#  elif defined(CONFIG_STM32_DMA2)
#    define DAC1_CH1_DMA_CHAN   DMAMAP_DMA12_DAC1CH1_1
#  else
#    error "No DMA channel for DAC1 CH1"
#  endif
#endif

#ifdef CONFIG_STM32_DAC1CH2_DMA
#  if defined(CONFIG_STM32_DMA1)
#    define DAC1_CH2_DMA_CHAN   DMAMAP_DMA12_DAC1CH2_0
#  elif defined(CONFIG_STM32_DMA2)
#    define DAC1_CH2_DMA_CHAN   DMAMAP_DMA12_DAC1CH2_1
#  else
#    error "No DMA channel for DAC1 CH2"
#  endif
#endif

/* Determine if any DMA is enabled (for common code) */
#if defined(CONFIG_STM32_DAC1CH1_DMA) || defined(CONFIG_STM32_DAC1CH2_DMA)
#  define HAVE_DMA 1
#endif

/* DMA priority macros from per-channel Kconfig choices */

#ifdef CONFIG_STM32_DAC1CH1_DMA
#  if defined(CONFIG_STM32_DAC1CH1_DMA_PRIORITY_LOW)
#    define DAC1CH1_DMA_PRI  0
#  elif defined(CONFIG_STM32_DAC1CH1_DMA_PRIORITY_MEDIUM)
#    define DAC1CH1_DMA_PRI  1
#  elif defined(CONFIG_STM32_DAC1CH1_DMA_PRIORITY_HIGH)
#    define DAC1CH1_DMA_PRI  2
#  else
#    define DAC1CH1_DMA_PRI  3
#  endif
#endif

#ifdef CONFIG_STM32_DAC1CH2_DMA
#  if defined(CONFIG_STM32_DAC1CH2_DMA_PRIORITY_LOW)
#    define DAC1CH2_DMA_PRI  0
#  elif defined(CONFIG_STM32_DAC1CH2_DMA_PRIORITY_MEDIUM)
#    define DAC1CH2_DMA_PRI  1
#  elif defined(CONFIG_STM32_DAC1CH2_DMA_PRIORITY_HIGH)
#    define DAC1CH2_DMA_PRI  2
#  else
#    define DAC1CH2_DMA_PRI  3
#  endif
#endif

/* Timer trigger selection for DAC1 channels.
 * On STM32H7, TSEL[3:0] encoding (from stm32h7xx_ll_dac.h):
 *   0001: TIM1_TRGO       (DAC_CR_TSEL1_0)
 *   0010: TIM2_TRGO       (DAC_CR_TSEL1_1)
 *   0011: TIM4_TRGO       (DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0)
 *   0100: TIM5_TRGO       (DAC_CR_TSEL1_2)
 *   0101: TIM6_TRGO       (DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0)
 *   0110: TIM7_TRGO       (DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1)
 *   0111: TIM8_TRGO       (DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0)
 *   1000: TIM15_TRGO      (DAC_CR_TSEL1_3)
 *   1011: LPTIM1_OUT
 *   1100: LPTIM2_OUT
 *   1110: EXTI_LINE9
 *   1111: SWTRIG
 *   NOTE: TSEL=0000 is RESERVED on STM32H7 (unlike F4/F7 families).
 * For channel 2, the same encoding is used but shifted by 16 bits.
 */

#define DAC_TSEL_TIM1  (1 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_TIM2  (2 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_TIM4  (3 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_TIM5  (4 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_TIM6  (5 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_TIM7  (6 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_TIM8  (7 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_TIM15 (8 << DAC_CR_TSEL1_SHIFT)
#define DAC_TSEL_SW    (15 << DAC_CR_TSEL1_SHIFT)

#undef NEED_TIM1
#undef NEED_TIM2
#undef NEED_TIM3
#undef NEED_TIM4
#undef NEED_TIM5
#undef NEED_TIM6
#undef NEED_TIM7
#undef NEED_TIM8
#undef NEED_TIM15

/* Helper to define timer needs for a given channel */
#define DAC_TIMER_NEED(ch) \
  defined(CONFIG_STM32_DAC1CH##ch##_DMA) && CONFIG_STM32_DAC1CH##ch##_TIMER

#if DAC_TIMER_NEED(1)
#  if CONFIG_STM32_DAC1CH1_TIMER == 1
#    ifndef CONFIG_STM32_TIM1
#      error "CONFIG_STM32_TIM1 required for DAC1 CH1"
#    endif
#    define NEED_TIM1
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM1
#    define DAC1_CH1_TIMER_BASE STM32_TIM1_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 2
#    ifndef CONFIG_STM32_TIM2
#      error "CONFIG_STM32_TIM2 required for DAC1 CH1"
#    endif
#    define NEED_TIM2
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM2
#    define DAC1_CH1_TIMER_BASE STM32_TIM2_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 3
#    error "TIM3_TRGO not available on STM32H7. Use TIM2/4/5/6/7/15."
#  elif CONFIG_STM32_DAC1CH1_TIMER == 4
#    ifndef CONFIG_STM32_TIM4
#      error "CONFIG_STM32_TIM4 required for DAC1 CH1"
#    endif
#    define NEED_TIM4
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM4
#    define DAC1_CH1_TIMER_BASE STM32_TIM4_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 5
#    ifndef CONFIG_STM32_TIM5
#      error "CONFIG_STM32_TIM5 required for DAC1 CH1"
#    endif
#    define NEED_TIM5
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM5
#    define DAC1_CH1_TIMER_BASE STM32_TIM5_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 6
#    ifndef CONFIG_STM32_TIM6
#      error "CONFIG_STM32_TIM6 required for DAC1 CH1"
#    endif
#    define NEED_TIM6
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM6
#    define DAC1_CH1_TIMER_BASE STM32_TIM6_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 7
#    ifndef CONFIG_STM32_TIM7
#      error "CONFIG_STM32_TIM7 required for DAC1 CH1"
#    endif
#    define NEED_TIM7
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM7
#    define DAC1_CH1_TIMER_BASE STM32_TIM7_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 8
#    ifndef CONFIG_STM32_TIM8
#      error "CONFIG_STM32_TIM8 required for DAC1 CH1"
#    endif
#    define NEED_TIM8
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM8
#    define DAC1_CH1_TIMER_BASE STM32_TIM8_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 15
#    ifndef CONFIG_STM32_TIM15
#      error "CONFIG_STM32_TIM15 required for DAC1 CH1"
#    endif
#    define NEED_TIM15
#    define DAC1_CH1_TSEL_VALUE DAC_TSEL_TIM15
#    define DAC1_CH1_TIMER_BASE STM32_TIM15_BASE
#  else
#    error "Unsupported CONFIG_STM32_DAC1CH1_TIMER"
#  endif
#else
#  define DAC1_CH1_TSEL_VALUE DAC_TSEL_SW
#endif

#if DAC_TIMER_NEED(2)
#  if CONFIG_STM32_DAC1CH2_TIMER == 1
#    ifndef CONFIG_STM32_TIM1
#      error "CONFIG_STM32_TIM1 required for DAC1 CH2"
#    endif
#    define NEED_TIM1
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM1
#    define DAC1_CH2_TIMER_BASE STM32_TIM1_BASE
#  elif CONFIG_STM32_DAC1CH2_TIMER == 2
#    ifndef CONFIG_STM32_TIM2
#      error "CONFIG_STM32_TIM2 required for DAC1 CH2"
#    endif
#    define NEED_TIM2
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM2
#    define DAC1_CH2_TIMER_BASE STM32_TIM2_BASE
#  elif CONFIG_STM32_DAC1CH2_TIMER == 3
#    error "TIM3_TRGO not available on STM32H7. Use TIM2/4/5/6/7/15."
#  elif CONFIG_STM32_DAC1CH2_TIMER == 4
#    ifndef CONFIG_STM32_TIM4
#      error "CONFIG_STM32_TIM4 required for DAC1 CH2"
#    endif
#    define NEED_TIM4
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM4
#    define DAC1_CH2_TIMER_BASE STM32_TIM4_BASE
#  elif CONFIG_STM32_DAC1CH2_TIMER == 5
#    ifndef CONFIG_STM32_TIM5
#      error "CONFIG_STM32_TIM5 required for DAC1 CH2"
#    endif
#    define NEED_TIM5
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM5
#    define DAC1_CH2_TIMER_BASE STM32_TIM5_BASE
#  elif CONFIG_STM32_DAC1CH2_TIMER == 6
#    ifndef CONFIG_STM32_TIM6
#      error "CONFIG_STM32_TIM6 required for DAC1 CH2"
#    endif
#    define NEED_TIM6
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM6
#    define DAC1_CH2_TIMER_BASE STM32_TIM6_BASE
#  elif CONFIG_STM32_DAC1CH2_TIMER == 7
#    ifndef CONFIG_STM32_TIM7
#      error "CONFIG_STM32_TIM7 required for DAC1 CH2"
#    endif
#    define NEED_TIM7
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM7
#    define DAC1_CH2_TIMER_BASE STM32_TIM7_BASE
#  elif CONFIG_STM32_DAC1CH2_TIMER == 8
#    ifndef CONFIG_STM32_TIM8
#      error "CONFIG_STM32_TIM8 required for DAC1 CH2"
#    endif
#    define NEED_TIM8
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM8
#    define DAC1_CH2_TIMER_BASE STM32_TIM8_BASE
#  elif CONFIG_STM32_DAC1CH2_TIMER == 15
#    ifndef CONFIG_STM32_TIM15
#      error "CONFIG_STM32_TIM15 required for DAC1 CH2"
#    endif
#    define NEED_TIM15
#    define DAC1_CH2_TSEL_VALUE DAC_TSEL_TIM15
#    define DAC1_CH2_TIMER_BASE STM32_TIM15_BASE
#  else
#    error "Unsupported CONFIG_STM32_DAC1CH2_TIMER"
#  endif
#else
#  define DAC1_CH2_TSEL_VALUE DAC_TSEL_SW
#endif

/* DMA control word */

#define DAC_DMA_CONTROL_WORD (DMA_SCR_MSIZE_16BITS | \
                              DMA_SCR_PSIZE_16BITS | \
                              DMA_SCR_MINC | \
                              DMA_SCR_CIRC | \
                              DMA_SCR_DIR_M2P)

/* Helper macros for register bits (already channel-specific) */

#define DAC_CR_EN(ch)         ((ch) == 1 ? DAC_CR_EN1 : DAC_CR_EN2)
#define DAC_CR_TEN(ch)        ((ch) == 1 ? DAC_CR_TEN1 : DAC_CR_TEN2)
#define DAC_CR_DMAEN(ch)      ((ch) == 1 ? DAC_CR_DMAEN1 : DAC_CR_DMAEN2)
#define DAC_CR_TSEL_MASK(ch)  ((ch) == 1 ? DAC_CR_TSEL1_MASK : DAC_CR_TSEL2_MASK)
#define DAC_CR_WAVE_MASK(ch)  ((ch) == 1 ? DAC_CR_WAVE1_MASK : DAC_CR_WAVE2_MASK)
#define DAC_CR_MAMP_MASK(ch)  ((ch) == 1 ? DAC_CR_MAMP1_MASK : DAC_CR_MAMP2_MASK)
#define DAC_MCR_MODE_MASK(ch) ((ch) == 1 ? DAC_MCR_MODE1_MASK : DAC_MCR_MODE2_MASK)
#define DAC_MCR_MODE_EXT(ch)  ((ch) == 1 ? DAC_MCR_MODE1_EXT : DAC_MCR_MODE2_EXT)
#define DAC_MCR_MODE_ONCHIP(ch) ((ch) == 1 ? DAC_MCR_MODE1_ON_CHIP : DAC_MCR_MODE2_ON_CHIP)

/* DRO register for each channel */
#define DAC_DHR12R(ch)        ((ch) == 1 ? STM32_DAC1_DHR12R1 : STM32_DAC1_DHR12R2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the DAC block (DAC1) */

struct stm32_dac_s
{
  uint8_t init : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one DAC channel */

struct stm32_chan_s
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  const struct stm32_dac_ops_s *llops; /* Low-level DAC ops */
#endif
  uint8_t inuse  : 1;   /* True, the driver is in use */
  uint8_t ch;           /* Channel number (1 or 2) */
  uint32_t pin;         /* Pin configuration (or 0xffffffff for internal) */
  uint32_t dro;         /* Data output register address */
  uint32_t tsel;        /* CR trigger select value */
#ifdef HAVE_DMA
  uint8_t hasdma        : 1;   /* True, this channel supports DMA */
  uint8_t dma_active    : 1;   /* True, DMA transfer is running */
  uint8_t halfint        : 1;  /* True, HT interrupt enabled */
  uint8_t dma_priority   : 2;  /* DMA PL field (DMA_SCR_PRI*) */
  uint8_t timer;               /* Timer number for DMA trigger */
  uint16_t dmachan;            /* DMA channel */
  uint16_t buffer_len;         /* DMA buffer length */
  DMA_HANDLE dma;              /* Allocated DMA channel */
  uint32_t   tbase;            /* Timer base address */
  uint32_t tfrequency;         /* Desired timer frequency */
  int result;                  /* DMA result */
  uint16_t buffer_pos;         /* Position in dmabuffer */
  uint16_t *dmabuffer;         /* DMA transfer buffer */
  sem_t dma_halfsem;           /* Stream half-completion sem */
  uint8_t dma_half_q[16];      /* Ring of completed half indices */
  uint8_t dma_half_q_head;
  uint8_t dma_half_q_tail;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t stm32_dac_getreg(struct stm32_chan_s *priv, int offset);
static void     stm32_dac_dumpregs(struct stm32_chan_s *priv);

#ifdef HAVE_DMA
static inline void stm32_tim_putreg(struct stm32_chan_s *chan, int offset,
                                    uint32_t value);
static inline void stm32_tim_modifyreg(struct stm32_chan_s *chan, int offset,
                                       uint32_t clearbits, uint32_t setbits);
#endif

static void stm32_dac_reset(struct dac_dev_s *dev);
static int  stm32_dac_setup(struct dac_dev_s *dev);
static void stm32_dac_shutdown(struct dac_dev_s *dev);
static void stm32_dac_txint(struct dac_dev_s *dev, bool enable);
static int  stm32_dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int  stm32_dac_ioctl(struct dac_dev_s *dev, int cmd,
                             unsigned long arg);

#ifdef HAVE_DMA
static int  stm32_dac_timinit(struct stm32_chan_s *chan);
static void stm32_dac_timstart(struct stm32_chan_s *chan);
static void stm32_dac_dma_start(struct stm32_chan_s *chan, bool halfint);
static void stm32_dac_dma_stop(struct stm32_chan_s *chan);
static int stm32_dac_dmainit(struct stm32_chan_s *chan);
#endif
static int  stm32_dac_chaninit(struct stm32_chan_s *chan);
static void stm32_dac_blockinit(void);

#ifdef CONFIG_STM32_DAC_LL_OPS
static void stm32_dac_llops_enable(struct stm32_dac_dev_s *dev,
                                   bool enabled);
static void stm32_dac_llops_writedro(struct stm32_dac_dev_s *dev,
                                     uint16_t data);
#ifdef HAVE_DMA
static void stm32_dac_llops_startdma(struct stm32_dac_dev_s *dev);
static void stm32_dac_llops_stopdma(struct stm32_dac_dev_s *dev);
#endif
static void stm32_dac_llops_dumpregs(struct stm32_dac_dev_s *dev);
#endif /* CONFIG_STM32_DAC_LL_OPS */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = stm32_dac_reset,
  .ao_setup    = stm32_dac_setup,
  .ao_shutdown = stm32_dac_shutdown,
  .ao_txint    = stm32_dac_txint,
  .ao_send     = stm32_dac_send,
  .ao_ioctl    = stm32_dac_ioctl,
};

#ifdef CONFIG_STM32_DAC_LL_OPS
static const struct stm32_dac_ops_s g_dac_llops =
{
  .enable        = stm32_dac_llops_enable,
  .write_dro     = stm32_dac_llops_writedro,
#ifdef HAVE_DMA
  .start_dma     = stm32_dac_llops_startdma,
  .stop_dma      = stm32_dac_llops_stopdma,
#endif
  .dump_regs     = stm32_dac_llops_dumpregs
};
#endif /* CONFIG_STM32_DAC_LL_OPS */

#ifdef CONFIG_STM32_DAC1CH1
/* DAC1 channel 1 */

#ifdef CONFIG_STM32_DAC1CH1_DMA
uint16_t stm32_dac1_ch1_dmabuffer[CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE];
#endif

static struct stm32_chan_s g_dac1ch1priv =
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  .llops      = &g_dac_llops,
#endif
  .ch         = 1,
#ifdef CONFIG_STM32_DAC1CH1_OUTPUT_ADC
  .pin        = 0xffffffffu,
#else
  .pin        = GPIO_DAC1_OUT1_0,
#endif
  .dro        = DAC_DHR12R(1),
  .tsel       = DAC1_CH1_TSEL_VALUE,
#ifdef CONFIG_STM32_DAC1CH1_DMA
  .hasdma     = 1,
  .dmachan    = DAC1_CH1_DMA_CHAN,
  .buffer_len = CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE,
  .dmabuffer  = stm32_dac1_ch1_dmabuffer,
  .timer      = CONFIG_STM32_DAC1CH1_TIMER,
  .tbase      = DAC1_CH1_TIMER_BASE,
  .tfrequency = CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY,
  .dma_priority = DAC1CH1_DMA_PRI,
#endif
};

static struct dac_dev_s g_dac1ch1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch1priv,
};
#endif /* CONFIG_STM32_DAC1CH1 */

#ifdef CONFIG_STM32_DAC1CH2
/* DAC1 channel 2 */

#ifdef CONFIG_STM32_DAC1CH2_DMA
uint16_t stm32_dac1_ch2_dmabuffer[CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE];
#endif

static struct stm32_chan_s g_dac1ch2priv =
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  .llops      = &g_dac_llops,
#endif
  .ch         = 2,
#ifdef CONFIG_STM32_DAC1CH2_OUTPUT_ADC
  .pin        = 0xffffffffu,
#else
  .pin        = GPIO_DAC1_OUT2_0,
#endif
  .dro        = DAC_DHR12R(2),
  .tsel       = DAC1_CH2_TSEL_VALUE,
#ifdef CONFIG_STM32_DAC1CH2_DMA
  .hasdma     = 1,
  .dmachan    = DAC1_CH2_DMA_CHAN,
  .buffer_len = CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE,
  .dmabuffer  = stm32_dac1_ch2_dmabuffer,
  .timer      = CONFIG_STM32_DAC1CH2_TIMER,
  .tbase      = DAC1_CH2_TIMER_BASE,
  .tfrequency = CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY,
  .dma_priority = DAC1CH2_DMA_PRI,
#endif
};

static struct dac_dev_s g_dac1ch2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch2priv,
};
#endif /* CONFIG_STM32_DAC1CH2 */

static struct stm32_dac_s g_dacblock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_getreg
 ****************************************************************************/

static uint32_t stm32_dac_getreg(struct stm32_chan_s *priv, int offset)
{
  return getreg32(STM32_DAC1_BASE + offset);
}

/****************************************************************************
 * Name: stm32_dac_dumpregs
 ****************************************************************************/

static void stm32_dac_dumpregs(struct stm32_chan_s *priv)
{
  UNUSED(priv);
  syslog(50, "CR:  0x%08" PRIx32 " SWTRGR: 0x%08" PRIx32 "\n"
        "SR:  0x%08" PRIx32 "    MCR: 0x%08" PRIx32 "\n",
        stm32_dac_getreg(priv, STM32_DAC1_CR_OFFSET),
        stm32_dac_getreg(priv, STM32_DAC1_SWTRGR_OFFSET),
        stm32_dac_getreg(priv, STM32_DAC1_SR_OFFSET),
        stm32_dac_getreg(priv, STM32_DAC1_MCR_OFFSET));
  syslog(50, "DHR12R1: 0x%08x\n",
         (unsigned int)getreg32(STM32_DAC1_DHR12R1));
  syslog(50, "DHR12R2: 0x%08x\n",
         (unsigned int)getreg32(STM32_DAC1_DHR12R2));
  syslog(50, "DOR1:    0x%08x\n", (unsigned int)getreg32(STM32_DAC1_DOR1));
  syslog(50, "DOR2:    0x%08x\n", (unsigned int)getreg32(STM32_DAC1_DOR2));
}

/****************************************************************************
 * Name: stm32_tim_putreg / stm32_tim_modifyreg
 ****************************************************************************/

#ifdef HAVE_DMA
static inline void stm32_tim_putreg(struct stm32_chan_s *chan, int offset,
                                    uint32_t value)
{
  putreg32(value, chan->tbase + offset);
}

static inline void stm32_tim_modifyreg(struct stm32_chan_s *chan, int offset,
                                       uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(chan->tbase + offset, clearbits, setbits);
}
#endif

/****************************************************************************
 * Name: stm32_dac_modify_cr
 ****************************************************************************/

static inline void stm32_dac_modify_cr(struct stm32_chan_s *chan,
                                       uint32_t clearbits, uint32_t setbits)
{
  /* Note: clearbits and setbits are already shifted for the specific
   * channel. Do NOT add extra shift here.
   */

  modifyreg32(STM32_DAC1_CR, clearbits, setbits);
}

/****************************************************************************
 * Name: stm32_dac_modify_mcr
 ****************************************************************************/

static inline void stm32_dac_modify_mcr(struct stm32_chan_s *chan,
                                        uint32_t clearbits, uint32_t setbits)
{
  /* Note: clearbits and setbits are already shifted for the specific
   * channel. Do NOT add extra shift here.
   */

  modifyreg32(STM32_DAC1_MCR, clearbits, setbits);
}

/****************************************************************************
 * Name: stm32_dac_reset
 ****************************************************************************/

static void stm32_dac_reset(struct dac_dev_s *dev)
{
  struct stm32_chan_s *chan = dev->ad_priv;

  stm32_dac_modify_cr(chan, DAC_CR_EN(chan->ch), 0);

  uint32_t clearbits = DAC_CR_TSEL_MASK(chan->ch) |
                       DAC_CR_WAVE_MASK(chan->ch) |
                       DAC_CR_MAMP_MASK(chan->ch);
  stm32_dac_modify_cr(chan, clearbits, 0);

  stm32_dac_modify_mcr(chan, DAC_MCR_MODE_MASK(chan->ch), 0);
}

/****************************************************************************
 * Name: stm32_dac_setup
 ****************************************************************************/

static int stm32_dac_setup(struct dac_dev_s *dev)
{
  struct stm32_chan_s *chan = dev->ad_priv;

  /* Configure the channel if not already configured */

  if (!chan->inuse)
    {
      int ret = stm32_dac_chaninit(chan);
      if (ret < 0)
        {
          aerr("ERROR: DAC channel init failed: %d\n", ret);
          return ret;
        }
    }

#ifdef HAVE_DMA
  chan->buffer_pos = 0;
  chan->halfint = 0;
  nxsem_init(&chan->dma_halfsem, 0, 0);
  chan->dma_half_q_head = 0;
  chan->dma_half_q_tail = 0;
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32_dac_shutdown
 ****************************************************************************/

static void stm32_dac_shutdown(struct dac_dev_s *dev)
{
  struct stm32_chan_s *chan = dev->ad_priv;

#ifdef HAVE_DMA
  stm32_dac_dma_stop(chan);
  nxsem_destroy(&chan->dma_halfsem);
#endif

  stm32_dac_modify_cr(chan, DAC_CR_EN(chan->ch), 0);
}

/****************************************************************************
 * Name: stm32_dac_txint
 ****************************************************************************/

static void stm32_dac_txint(struct dac_dev_s *dev, bool enable)
{
  /* No interrupt support for DAC */
}

/****************************************************************************
 * Name: stm32_dac_dmatxcallback
 ****************************************************************************/

#ifdef HAVE_DMA
static void stm32_dac_dmatxcallback(DMA_HANDLE handle, uint8_t isr,
                                    void *arg)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)arg;

  DEBUGASSERT(chan);

  if (chan->result == -EBUSY)
    {
      if (isr & DMA_STREAM_TEIF_BIT)
        {
          chan->result = -EIO;
        }
      else if (chan->halfint)
        {
          uint8_t h;

          if (isr & DMA_STREAM_HTIF_BIT)
            {
              h = 0;
            }
          else
            {
              h = 1;
            }

          chan->dma_half_q[chan->dma_half_q_head] = h;
          chan->dma_half_q_head =
            (chan->dma_half_q_head + 1) & 15;
          nxsem_post(&chan->dma_halfsem);
        }
      else
        {
          chan->result = OK;
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_dac_send
 ****************************************************************************/

static int stm32_dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
  struct stm32_chan_s *chan = dev->ad_priv;

  stm32_dac_modify_cr(chan, 0, DAC_CR_EN(chan->ch));
  putreg16(msg->am_data, chan->dro);
  dac_txdone(dev);

  return OK;
}

/****************************************************************************
 * Name: stm32_dac_ioctl
 ****************************************************************************/

static int stm32_dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  int ret = -ENOTTY;
  struct stm32_chan_s *chan = dev->ad_priv;

  switch (cmd)
    {
#ifdef HAVE_DMA
    case ANIOC_DAC_DMABUFF_INIT:
      {
        uint16_t *buffer = (uint16_t *)arg;

        /* The caller is responsible for providing buffer with
         * suitable length equal to CONFIG_STM32_DACxCHy_DMA_BUFFER_SIZE
         */

        uint32_t len = chan->buffer_len * sizeof(uint16_t);
        memcpy(chan->dmabuffer, buffer, len);
#ifdef CONFIG_ARMV7M_DCACHE
        up_clean_dcache((uintptr_t)chan->dmabuffer,
                        (uintptr_t)chan->dmabuffer + len);
#endif
        ret = OK;
      }
      break;

    case ANIOC_DAC_DMA_START:
      {
        struct dac_dma_start_s *req =
          (struct dac_dma_start_s *)arg;

        chan->halfint = req->halfint;
        stm32_dac_dma_start(chan, req->halfint);
        ret = OK;
      }
      break;

    case ANIOC_DAC_DMA_STOP:
      stm32_dac_dma_stop(chan);
      chan->halfint = 0;
      ret = OK;
      break;

    case ANIOC_DAC_DMA_GET_EVENT:
      {
        struct dac_dma_event_s *req =
          (struct dac_dma_event_s *)arg;

        ret = nxsem_wait(&chan->dma_halfsem);
        if (ret == OK)
          {
            req->half =
              chan->dma_half_q[chan->dma_half_q_tail];
            chan->dma_half_q_tail =
              (chan->dma_half_q_tail + 1) & 15;
          }
      }
      break;

    case ANIOC_DAC_DMAHBUF_WRITE:
      {
        struct dac_dma_event_s *req =
          (struct dac_dma_event_s *)arg;
        uint32_t half_len = chan->buffer_len / 2;
        uint16_t *dst = chan->dmabuffer + req->half * half_len;

        memcpy(dst, req->buffer, half_len * sizeof(uint16_t));
#ifdef CONFIG_ARMV7M_DCACHE
        up_clean_dcache((uintptr_t)dst,
                        (uintptr_t)dst + half_len * sizeof(uint16_t));
#endif
        ret = OK;
      }
      break;
#endif

    case ANIOC_DAC_INFO:
      {
        struct dac_info_s *info = (struct dac_info_s *)arg;

        info->sample_bits  = 12;
#ifdef HAVE_DMA
        info->dma_enabled  = chan->dma_active;
        info->halfint_enabled = chan->halfint;
        info->dma_buffer_size  = chan->buffer_len;
        info->dma_timer_frequency = chan->tfrequency;
#else
        info->dma_enabled  = false;
        info->halfint_enabled = 0;
        info->dma_buffer_size  = 0;
        info->dma_timer_frequency = 0;
#endif
        ret = OK;
      }
      break;

    default:
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_dac_timinit
 ****************************************************************************/

#ifdef HAVE_DMA
static int stm32_dac_timinit(struct stm32_chan_s *chan)
{
  uint32_t pclk;
  uint32_t prescaler;
  uint32_t timclk;
  uint32_t reload;
  uint32_t regaddr;
  uint32_t setbits;

  switch (chan->timer)
    {
#ifdef NEED_TIM1
      case 1:
        regaddr = STM32_RCC_APB2ENR;
        setbits = RCC_APB2ENR_TIM1EN;
        pclk    = STM32_TIM1_CLKIN;
        break;
#endif
#ifdef NEED_TIM2
      case 2:
        regaddr = STM32_RCC_APB1LENR;
        setbits = RCC_APB1LENR_TIM2EN;
        pclk    = STM32_TIM2_CLKIN;
        break;
#endif
#ifdef NEED_TIM4
      case 4:
        regaddr = STM32_RCC_APB1LENR;
        setbits = RCC_APB1LENR_TIM4EN;
        pclk    = STM32_TIM4_CLKIN;
        break;
#endif
#ifdef NEED_TIM5
      case 5:
        regaddr = STM32_RCC_APB1LENR;
        setbits = RCC_APB1LENR_TIM5EN;
        pclk    = STM32_TIM5_CLKIN;
        break;
#endif
#ifdef NEED_TIM6
      case 6:
        regaddr = STM32_RCC_APB1LENR;
        setbits = RCC_APB1LENR_TIM6EN;
        pclk    = STM32_TIM6_CLKIN;
        break;
#endif
#ifdef NEED_TIM7
      case 7:
        regaddr = STM32_RCC_APB1LENR;
        setbits = RCC_APB1LENR_TIM7EN;
        pclk    = STM32_TIM7_CLKIN;
        break;
#endif
#ifdef NEED_TIM8
      case 8:
        regaddr = STM32_RCC_APB2ENR;
        setbits = RCC_APB2ENR_TIM8EN;
        pclk    = STM32_TIM8_CLKIN;
        break;
#endif
#ifdef NEED_TIM15
      case 15:
        regaddr = STM32_RCC_APB2ENR;
        setbits = RCC_APB2ENR_TIM15EN;
        pclk    = STM32_TIM15_CLKIN;
        break;
#endif
      default:
        aerr("ERROR: Unsupported timer %d\n", chan->timer);
        return -EINVAL;
    }

  modifyreg32(regaddr, 0, setbits);

  /* Calculate prescaler and reload */

  prescaler = (pclk / chan->tfrequency + 65534) / 65535;
  if (prescaler < 1) prescaler = 1;
  if (prescaler > 65536) prescaler = 65536;

  timclk = pclk / prescaler;
  reload = (timclk + chan->tfrequency / 2) / chan->tfrequency - 1;
  if (reload > 65535) reload = 65535;

  stm32_tim_putreg(chan, STM32_GTIM_ARR_OFFSET, (uint16_t)reload);
  stm32_tim_putreg(chan, STM32_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  stm32_tim_modifyreg(chan, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_ARPE);
  stm32_tim_modifyreg(chan, STM32_GTIM_CR2_OFFSET, GTIM_CR2_MMS_MASK,
                GTIM_CR2_MMS_UPDATE);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_dac_timstart
 ****************************************************************************/

#ifdef HAVE_DMA
static void stm32_dac_timstart(struct stm32_chan_s *chan)
{
  stm32_tim_modifyreg(chan, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
  stm32_tim_modifyreg(chan, STM32_GTIM_EGR_OFFSET, 0, GTIM_EGR_UG);
}
#endif

/****************************************************************************
 * Name: stm32_dac_dma_start
 ****************************************************************************/

#ifdef HAVE_DMA
static void stm32_dac_dma_start(struct stm32_chan_s *chan, bool halfint)
{
  if (chan->dma_active)
    return;

  chan->result = -EBUSY;

#ifdef CONFIG_ARMV7M_DCACHE
  up_clean_dcache((uintptr_t)chan->dmabuffer,
                  (uintptr_t)chan->dmabuffer +
                  chan->buffer_len * sizeof(uint16_t));
#endif

  stm32_dmastart(chan->dma, stm32_dac_dmatxcallback, chan, halfint);
  stm32_dac_modify_cr(chan, 0, DAC_CR_EN(chan->ch) |
                                DAC_CR_TEN(chan->ch) |
                                DAC_CR_DMAEN(chan->ch));
  stm32_dac_timstart(chan);
  chan->dma_active = 1;
  chan->buffer_pos = 0;
}
#endif

/****************************************************************************
 * Name: stm32_dac_dma_stop
 ****************************************************************************/

#ifdef HAVE_DMA
static void stm32_dac_dma_stop(struct stm32_chan_s *chan)
{
  if (!chan->dma_active)
    return;

  stm32_tim_modifyreg(chan, STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
  stm32_dac_modify_cr(chan, DAC_CR_TEN(chan->ch) |
                            DAC_CR_DMAEN(chan->ch), 0);
  stm32_dmastop(chan->dma);
  chan->dma_active = 0;
}
#endif

#ifdef HAVE_DMA
static int stm32_dac_dmainit(struct stm32_chan_s *chan)
{
  if (chan->hasdma)
    {
      chan->dma = stm32_dmachannel(chan->dmachan);
      if (!chan->dma)
        {
          return -EINVAL;
        }

      /* Configure for circular DMA */

      struct stm32_dma_config_s dmacfg;

      dmacfg.paddr  = chan->dro;
      dmacfg.maddr  = (uint32_t)chan->dmabuffer;
      dmacfg.ndata  = chan->buffer_len;
      dmacfg.cfg1   = DAC_DMA_CONTROL_WORD |
                       (chan->dma_priority << DMA_SCR_PL_SHIFT);
      dmacfg.cfg2   = 0;
      stm32_dmasetup(chan->dma, &dmacfg);

      chan->result     = -EBUSY;
      chan->dma_active = 0;
      return OK;
    }

  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: stm32_dac_chaninit
 ****************************************************************************/

static int stm32_dac_chaninit(struct stm32_chan_s *chan)
{
  uint32_t clearbits;
  uint32_t setbits;

  if (chan->inuse)
    {
      return -EBUSY;
    }

  /* Configure GPIO pin if needed */

  if (chan->pin != 0xffffffffu)
    stm32_configgpio(chan->pin);

  /* Disable channel before configuration */

  stm32_dac_modify_cr(chan, DAC_CR_EN(chan->ch), 0);

  /* Configure trigger, waveform (none), amplitude (0) */

  clearbits = DAC_CR_TSEL_MASK(chan->ch) | DAC_CR_WAVE_MASK(chan->ch) |
              DAC_CR_MAMP_MASK(chan->ch);
  setbits   = chan->tsel;  /* WAVE=0, MAMP=0 (no noise/triangle) */
  stm32_dac_modify_cr(chan, clearbits, setbits);

  /* Configure MCR (mode).
   * MODE=000: external pin with buffer, supports DMA requests.
   * MODE=001 (MODE1_EXT): external pin with buffer, NO DMA requests.
   * MODE=101 (MODE1_ON_CHIP): internal (to ADC), supports DMA requests.
   */

  clearbits = DAC_MCR_MODE_MASK(chan->ch);
  if (chan->pin != 0xffffffffu)
    {
#ifdef HAVE_DMA
      if (chan->hasdma)
        setbits = 0;  /* MODE=000: DMA capable */
      else
#endif
        setbits = DAC_MCR_MODE_EXT(chan->ch);  /* MODE=001 */
    }
  else
    {
      setbits = DAC_MCR_MODE_ONCHIP(chan->ch); /* MODE=101 */
    }

  stm32_dac_modify_mcr(chan, clearbits, setbits);

#ifdef HAVE_DMA
  if (chan->hasdma)
    {
      /* MODE=000 is also sample-and-hold if DAC_CCR CxMODE=1.
       * Force CxMODE=0 for normal DMA mode.
       * DAC_CCR bit 0 = C1MODE, bit 1 = C2MODE.
       */

      modifyreg32(STM32_DAC1_CCR, (1 << (chan->ch - 1)), 0);

      /* Enable trigger */

      stm32_dac_modify_cr(chan, 0, DAC_CR_TEN(chan->ch));

      int ret = stm32_dac_dmainit(chan);
      if (ret < 0)
        {
          aerr("ERROR: Failed to allocate DMA channel\n");
          return ret;
        }

      ret = stm32_dac_timinit(chan);
      if (ret < 0)
        {
          aerr("ERROR: Timer init failed: %d\n", ret);
          stm32_dmafree(chan->dma);
          return ret;
        }
    }
#endif

  chan->inuse = 1;
  return OK;
}

/****************************************************************************
 * Name: stm32_dac_blockinit
 ****************************************************************************/

static void stm32_dac_blockinit(void)
{
  if (g_dacblock.init)
    return;

  irqstate_t flags = enter_critical_section();

  /* Enable clock for DAC1 (APB1L) and de-assert reset */

  modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_DAC1EN);
  modifyreg32(STM32_RCC_APB1LRSTR, 0, RCC_APB1LRSTR_DAC1RST);
  modifyreg32(STM32_RCC_APB1LRSTR, RCC_APB1LRSTR_DAC1RST, 0);

  g_dacblock.init = 1;
  leave_critical_section(flags);
}

#ifdef CONFIG_STM32_DAC_LL_OPS

/****************************************************************************
 * Name: stm32_dac_llops_enable
 ****************************************************************************/

static void stm32_dac_llops_enable(struct stm32_dac_dev_s *dev, bool enabled)
{
  struct stm32_chan_s *priv = (struct stm32_chan_s *)dev;
  if (enabled)
    stm32_dac_modify_cr(priv, 0, DAC_CR_EN(priv->ch));
  else
    stm32_dac_modify_cr(priv, DAC_CR_EN(priv->ch), 0);
}

/****************************************************************************
 * Name: stm32_dac_llops_writedro
 ****************************************************************************/

static void stm32_dac_llops_writedro(struct stm32_dac_dev_s *dev,
                                     uint16_t data)
{
  struct stm32_chan_s *priv = (struct stm32_chan_s *)dev;
  putreg16(data, priv->dro);
}

#ifdef HAVE_DMA
/****************************************************************************
 * Name: stm32_dac_llops_startdma
 ****************************************************************************/

static void stm32_dac_llops_startdma(struct stm32_dac_dev_s *dev)
{
  struct stm32_chan_s *priv = (struct stm32_chan_s *)dev;
  stm32_dac_dma_start(priv, false);
}

/****************************************************************************
 * Name: stm32_dac_llops_stopdma
 ****************************************************************************/

static void stm32_dac_llops_stopdma(struct stm32_dac_dev_s *dev)
{
  struct stm32_chan_s *priv = (struct stm32_chan_s *)dev;
  stm32_dac_dma_stop(priv);
}
#endif

/****************************************************************************
 * Name: stm32_dac_llops_dumpregs
 ****************************************************************************/

static void stm32_dac_llops_dumpregs(struct stm32_dac_dev_s *dev)
{
  struct stm32_chan_s *priv = (struct stm32_chan_s *)dev;
  stm32_dac_dumpregs(priv);
}

#endif /* CONFIG_STM32_DAC_LL_OPS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC. The interface number corresponds to the channel:
 *   0 = DAC1 channel 1, 1 = DAC1 channel 2.
 *
 ****************************************************************************/

struct dac_dev_s *stm32_dacinitialize(int intf)
{
  struct dac_dev_s *dev;

  switch (intf)
    {
#ifdef CONFIG_STM32_DAC1CH1
      case 0:
        dev = &g_dac1ch1dev;
        break;
#endif
#ifdef CONFIG_STM32_DAC1CH2
      case 1:
        dev = &g_dac1ch2dev;
        break;
#endif
      default:
        aerr("ERROR: Invalid DAC interface: %d\n", intf);
        return NULL;
    }

  stm32_dac_blockinit();

  return dev;
}

#endif /* CONFIG_DAC */