/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_dac.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Juha Niskanen <juha.niskanen@haltian.com>
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/analog/dac.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32l4.h"
#include "stm32l4_dac.h"
#include "stm32l4_rcc.h"
#include "stm32l4_dma.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Up to 1 DAC interface for up to 2 channels are supported */

#if STM32L4_NDAC > 2
#  warning "Extra DAC channels. Only DAC1 and DAC2 are supported"
#endif

#if STM32L4_NDAC < 2
#  undef CONFIG_STM32L4_DAC2
#  undef CONFIG_STM32L4_DAC2_DMA
#  undef CONFIG_STM32L4_DAC2_TIMER
#  undef CONFIG_STM32L4_DAC2_TIMER_FREQUENCY
#endif

#if STM32L4_NDAC < 1
#  undef CONFIG_STM32L4_DAC1
#  undef CONFIG_STM32L4_DAC1_DMA
#  undef CONFIG_STM32L4_DAC1_TIMER
#  undef CONFIG_STM32L4_DAC1_TIMER_FREQUENCY
#endif

#if defined(CONFIG_STM32L4_DAC1) || defined(CONFIG_STM32L4_DAC2)

/* DMA configuration. */

/* If DMA is selected, then a buffer, a timer and output frequency must
 * also be provided to support the DMA transfer.  The DMA transfer could be
 * supported by an EXTI trigger, but this feature is not currently
 * supported by the driver.
 */

#ifdef CONFIG_STM32L4_DAC1_DMA
#  if !defined(CONFIG_STM32L4_DAC1_DMA_BUFFER_SIZE) || CONFIG_STM32L4_DAC1_DMA_BUFFER_SIZE < 1
#    define CONFIG_STM32L4_DAC1_DMA_BUFFER_SIZE 1
#  endif
#  if !defined(CONFIG_STM32L4_DAC1_TIMER)
#    warning "A timer number must be specified in CONFIG_STM32L4_DAC1_TIMER"
#    undef CONFIG_STM32L4_DAC1_DMA
#    undef CONFIG_STM32L4_DAC1_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32L4_DAC1_TIMER_FREQUENCY) || \
        (CONFIG_STM32L4_DAC1_TIMER_FREQUENCY < 1)
#    warning "A timer frequency (>0) must be specified in CONFIG_STM32L4_DAC1_TIMER_FREQUENCY"
#    undef CONFIG_STM32L4_DAC1_DMA
#    undef CONFIG_STM32L4_DAC1_TIMER
#  endif
#endif

#ifdef CONFIG_STM32L4_DAC2_DMA
#  if !defined(CONFIG_STM32L4_DAC2_DMA_BUFFER_SIZE) || CONFIG_STM32L4_DAC2_DMA_BUFFER_SIZE < 1
#    define CONFIG_STM32L4_DAC2_DMA_BUFFER_SIZE 1
#  endif
#  if !defined(CONFIG_STM32L4_DAC2_TIMER)
#    warning "A timer number must be specified in CONFIG_STM32L4_DAC2_TIMER"
#    undef CONFIG_STM32L4_DAC2_DMA
#    undef CONFIG_STM32L4_DAC2_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32L4_DAC2_TIMER_FREQUENCY) || \
        (CONFIG_STM32L4_DAC2_TIMER_FREQUENCY < 1)
#    warning "A timer frequency (>0) must be specified in CONFIG_STM32L4_DAC2_TIMER_FREQUENCY"
#    undef CONFIG_STM32L4_DAC2_DMA
#    undef CONFIG_STM32L4_DAC2_TIMER
#  endif
#endif

/* Select DMA channels, favor DMA1 if configured. */

#undef HAVE_DMA
#ifdef CONFIG_STM32L4_DAC1_DMA
#  if defined(CONFIG_STM32L4_DMAMUX1) && defined(CONFIG_STM32L4_DMA1)
#    define DAC1_DMA_CHAN   DMAMAP_DAC1_0
#  elif defined(CONFIG_STM32L4_DMAMUX1) && defined(CONFIG_STM32L4_DMA2)
#    define DAC1_DMA_CHAN   DMAMAP_DAC1_1
#  elif defined(CONFIG_STM32L4_DMA1)
#    define DAC1_DMA_CHAN   DMACHAN_DAC1_1
#  elif defined(CONFIG_STM32L4_DMA2)
#    define DAC1_DMA_CHAN   DMACHAN_DAC1_2
#  else
#    error "No DMA channel for DAC1"
#  endif
#  define HAVE_DMA
#endif
#ifdef CONFIG_STM32L4_DAC2_DMA
#  if defined(CONFIG_STM32L4_DMAMUX1) && defined(CONFIG_STM32L4_DMA1)
#    define DAC2_DMA_CHAN   DMAMAP_DAC2_0
#  elif defined(CONFIG_STM32L4_DMAMUX1) && defined(CONFIG_STM32L4_DMA2)
#    define DAC2_DMA_CHAN   DMAMAP_DAC2_1
#  elif defined(CONFIG_STM32L4_DMA1)
#    define DAC2_DMA_CHAN   DMACHAN_DAC2_1
#  elif defined(CONFIG_STM32L4_DMA2)
#    define DAC2_DMA_CHAN   DMACHAN_DAC2_2
#  else
#    error "No DMA channel for DAC2"
#  endif
#  define HAVE_DMA
#endif

/* Timer configuration.  The STM32L4 supports 8 different trigger for DAC
 * output:
 *
 * TSEL SOURCE                  DEVICES
 * ---- ----------------------- -------------------------------------
 * 000  Timer 6 TRGO event      ALL
 * 001  Timer 8 TRGO event      STM32L4x5 and STM32L4x6
 * 010  Timer 7 TRGO event      Not on STM32L45xxx and STM32L46xxx
 * 011  Timer 5 TRGO event      STM32L4x5 and STM32L4x6
 * 100  Timer 2 TRGO event      ALL
 * 101  Timer 4 TRGO event      STM32L4x5 and STM32L4x6
 * 110  EXTI line9              ALL
 * 111  SWTRIG Software control ALL
 *
 * This driver does not support the EXTI trigger.
 */

#undef NEED_TIM6
#undef NEED_TIM8
#undef NEED_TIM7
#undef NEED_TIM5
#undef NEED_TIM2
#undef NEED_TIM4

#ifdef CONFIG_STM32L4_DAC1_DMA
#  if CONFIG_STM32L4_DAC1_TIMER == 6
#    ifndef CONFIG_STM32L4_TIM6_DAC
#      error "CONFIG_STM32L4_TIM6_DAC required for DAC1"
#    endif
#    define NEED_TIM6
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC1_TIMER_BASE           STM32L4_TIM6_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  elif CONFIG_STM32L4_DAC1_TIMER == 8
#    ifndef CONFIG_STM32L4_TIM8_DAC
#      error "CONFIG_STM32L4_TIM8_DAC required for DAC1"
#    endif
#    define NEED_TIM8
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM8
#    define DAC1_TIMER_BASE           STM32L4_TIM8_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32L4_PCLK2_FREQUENCY
#  elif CONFIG_STM32L4_DAC1_TIMER == 7
#    ifndef CONFIG_STM32L4_TIM7_DAC
#      error "CONFIG_STM32L4_TIM7_DAC required for DAC1"
#    endif
#    define NEED_TIM7
#    define DAC1_TSEL_VALUE DAC_CR_TSEL_TIM7
#    define DAC1_TIMER_BASE STM32L4_TIM7_BASE
#  elif CONFIG_STM32L4_DAC1_TIMER == 5
#    ifndef CONFIG_STM32L4_TIM5_DAC
#      error "CONFIG_STM32L4_TIM5_DAC required for DAC1"
#    endif
#    define NEED_TIM5
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC1_TIMER_BASE           STM32L4_TIM5_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  elif CONFIG_STM32L4_DAC1_TIMER == 2
#    ifndef CONFIG_STM32L4_TIM2_DAC
#      error "CONFIG_STM32L4_TIM2_DAC required for DAC1"
#    endif
#    define NEED_TIM2
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC1_TIMER_BASE           STM32L4_TIM2_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  elif CONFIG_STM32L4_DAC1_TIMER == 4
#    ifndef CONFIG_STM32L4_TIM4_DAC
#      error "CONFIG_STM32L4_TIM4_DAC required for DAC1"
#    endif
#    define NEED_TIM4
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC1_TIMER_BASE           STM32L4_TIM4_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  else
#    error "Unsupported CONFIG_STM32L4_DAC1_TIMER"
#  endif
#else
#  define DAC1_TSEL_VALUE DAC_CR_TSEL_SW
#endif

#ifdef CONFIG_STM32L4_DAC2_DMA
#  if CONFIG_STM32L4_DAC2_TIMER == 6
#    ifndef CONFIG_STM32L4_TIM6_DAC
#      error "CONFIG_STM32L4_TIM6_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC2_TIMER_BASE           STM32L4_TIM6_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  elif CONFIG_STM32L4_DAC2_TIMER == 8
#    ifndef CONFIG_STM32L4_TIM8_DAC
#      error "CONFIG_STM32L4_TIM8_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM8
#    define DAC2_TIMER_BASE           STM32L4_TIM8_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32L4_PCLK2_FREQUENCY
#  elif CONFIG_STM32L4_DAC2_TIMER == 7
#    ifndef CONFIG_STM32L4_TIM7_DAC
#      error "CONFIG_STM32L4_TIM7_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM7
#    define DAC2_TIMER_BASE           STM32L4_TIM7_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  elif CONFIG_STM32L4_DAC2_TIMER == 5
#    ifndef CONFIG_STM32L4_TIM5_DAC
#      error "CONFIG_STM32L4_TIM5_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC2_TIMER_BASE           STM32L4_TIM5_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  elif CONFIG_STM32L4_DAC2_TIMER == 2
#    ifndef CONFIG_STM32L4_TIM2_DAC
#      error "CONFIG_STM32L4_TIM2_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC2_TIMER_BASE           STM32L4_TIM2_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  elif CONFIG_STM32L4_DAC2_TIMER == 4
#    ifndef CONFIG_STM32L4_TIM4_DAC
#      error "CONFIG_STM32L4_TIM4_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC2_TIMER_BASE           STM32L4_TIM4_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32L4_PCLK1_FREQUENCY
#  else
#    error "Unsupported CONFIG_STM32L4_DAC2_TIMER"
#  endif
#else
#  define DAC2_TSEL_VALUE DAC_CR_TSEL_SW
#endif

/* Calculate timer divider values based upon DACn_TIMER_PCLK_FREQUENCY and
 * CONFIG_STM32L4_DACn_TIMER_FREQUENCY.
 */

#warning "Missing Logic"

/* DMA stream/channel configuration */

#define DAC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                              DMA_CCR_PSIZE_16BITS | \
                              DMA_CCR_MINC | \
                              DMA_CCR_CIRC | \
                              DMA_CCR_DIR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the single STM32 DAC block */

struct stm32_dac_s
{
  uint8_t    init   : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one STM32 DAC channel */

struct stm32_chan_s
{
  uint8_t    inuse  : 1; /* True, the driver is in use and not available */
#ifdef HAVE_DMA
  uint8_t    hasdma : 1; /* True, this channel supports DMA */
  uint8_t    timer;      /* Timer number 2-8 */
#endif
  uint8_t    intf;       /* DAC zero-based interface number (0 or 1) */
  uint32_t   pin;        /* Pin configuration */
  uint32_t   dro;        /* Data output register */
  uint32_t   cr;         /* Control register */
  uint32_t   tsel;       /* CR trigger select value */
#ifdef HAVE_DMA
  uint16_t   dmachan;    /* DMA channel needed by this DAC */
  uint16_t   buffer_len; /* DMA buffer length */
  DMA_HANDLE dma;        /* Allocated DMA channel */
  uint32_t   tbase;      /* Timer base address */
  uint32_t   tfrequency; /* Timer frequency */
  int        result;     /* DMA result */
  uint16_t   buffer_pos; /* Position in dmabuffer where to write new value */
  uint16_t   *dmabuffer; /* DMA transfer buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* DAC Register access */

#ifdef HAVE_DMA
static inline void tim_putreg(FAR struct stm32_chan_s *chan, int offset,
                              uint32_t value);
static inline void tim_modifyreg(FAR struct stm32_chan_s *chan, int offset,
                                 uint32_t clearbits, uint32_t setbits);
#endif

/* DAC methods */

static void dac_reset(FAR struct dac_dev_s *dev);
static int  dac_setup(FAR struct dac_dev_s *dev);
static void dac_shutdown(FAR struct dac_dev_s *dev);
static void dac_txint(FAR struct dac_dev_s *dev, bool enable);
static int  dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);
static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg);

/* Initialization */

#ifdef HAVE_DMA
static int  dac_timinit(FAR struct stm32_chan_s *chan);
#endif
static int  dac_chaninit(FAR struct stm32_chan_s *chan);
static void dac_blockinit(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = dac_reset,
  .ao_setup    = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint    = dac_txint,
  .ao_send     = dac_send,
  .ao_ioctl    = dac_ioctl,
};

#ifdef CONFIG_STM32L4_DAC1
/* Channel 1 */

#ifdef CONFIG_STM32L4_DAC1_DMA
uint16_t stm32l4_dac1_dmabuffer[CONFIG_STM32L4_DAC1_DMA_BUFFER_SIZE];
#endif

static struct stm32_chan_s g_dac1priv =
{
  .intf       = 0,
#ifdef CONFIG_STM32L4_DAC1_OUTPUT_ADC
  .pin        = 0xffffffffU,
#else
  .pin        = GPIO_DAC1_OUT,
#endif
  .dro        = STM32L4_DAC_DHR12R1,
  .cr         = STM32L4_DAC_CR,
#ifdef CONFIG_STM32L4_DAC1_DMA
  .hasdma     = 1,
  .dmachan    = DAC1_DMA_CHAN,
  .buffer_len = CONFIG_STM32L4_DAC1_DMA_BUFFER_SIZE,
  .dmabuffer  = stm32l4_dac1_dmabuffer,
  .timer      = CONFIG_STM32L4_DAC1_TIMER,
  .tsel       = DAC1_TSEL_VALUE,
  .tbase      = DAC1_TIMER_BASE,
  .tfrequency = CONFIG_STM32L4_DAC1_TIMER_FREQUENCY,
#endif
};

static struct dac_dev_s g_dac1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1priv,
};

#endif /* CONFIG_STM32L4_DAC1 */

#ifdef CONFIG_STM32L4_DAC2
/* Channel 2 */

#ifdef CONFIG_STM32L4_DAC2_DMA
uint16_t stm32l4_dac2_dmabuffer[CONFIG_STM32L4_DAC2_DMA_BUFFER_SIZE];
#endif

static struct stm32_chan_s g_dac2priv =
{
  .intf       = 1,
#ifdef CONFIG_STM32L4_DAC2_OUTPUT_ADC
  .pin        = 0xffffffffU,
#else
  .pin        = GPIO_DAC2_OUT,
#endif
  .dro        = STM32L4_DAC_DHR12R2,
  .cr         = STM32L4_DAC_CR,
#ifdef CONFIG_STM32L4_DAC2_DMA
  .hasdma     = 1,
  .dmachan    = DAC2_DMA_CHAN,
  .buffer_len = CONFIG_STM32L4_DAC2_DMA_BUFFER_SIZE,
  .dmabuffer  = stm32l4_dac2_dmabuffer,
  .timer      = CONFIG_STM32L4_DAC2_TIMER,
  .tsel       = DAC2_TSEL_VALUE,
  .tbase      = DAC2_TIMER_BASE,
  .tfrequency = CONFIG_STM32L4_DAC2_TIMER_FREQUENCY,
#endif
};

static struct dac_dev_s g_dac2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac2priv,
};

#endif /* CONFIG_STM32L4_DAC2 */

static struct stm32_dac_s g_dacblock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_dac_modify_cr
 *
 * Description:
 *   Modify the contents of the DAC control register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *   clearbits - Bits in the control register to be cleared
 *   setbits - Bits in the control register to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32l4_dac_modify_cr(FAR struct stm32_chan_s *chan,
                                         uint32_t clearbits, uint32_t setbits)
{
  unsigned int shift;

  /* DAC channels 1 and 2 share the STM32L4_DAC[1]_CR control register. If
   * future chips have DAC channel 3 (and perhaps channel 4) they likely have
   * their own register like in STM32. In either case, bit 0 of the interface
   * number provides the correct shift.
   *
   *   Bit 0 = 0: Shift = 0
   *   Bit 0 = 1: Shift = 16
   */

  shift = (chan->intf & 1) << 4;
  modifyreg32(chan->cr, clearbits << shift, setbits << shift);
}

/****************************************************************************
 * Name: stm32l4_dac_modify_mcr
 *
 * Description:
 *   Modify the contents of the DAC mode register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *   clearbits - Bits in the control register to be cleared
 *   setbits - Bits in the control register to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32l4_dac_modify_mcr(FAR struct stm32_chan_s *chan,
                                          uint32_t clearbits, uint32_t setbits)
{
  unsigned int shift;

  /* DAC channels 1 and 2 share the STM32L4_DAC_MCR control register.
   * Bit 0 of the interface number provides the correct shift.
   *
   *   Bit 0 = 0: Shift = 0
   *   Bit 0 = 1: Shift = 16
   */

  shift = (chan->intf & 1) << 4;
  modifyreg32(STM32L4_DAC_MCR, clearbits << shift, setbits << shift);
}

/****************************************************************************
 * Name: tim_putreg
 *
 * Description:
 *   Read the value of an DMA timer register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_DMA
static inline void tim_putreg(FAR struct stm32_chan_s *chan, int offset,
                              uint32_t value)
{
  putreg32(value, chan->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_modifyreg
 *
 * Description:
 *   Modify the value of an DMA timer register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *   offset - The timer register offset
 *   clearbits - Bits in the control register to be cleared
 *   setbits - Bits in the control register to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_DMA
static inline void tim_modifyreg(FAR struct stm32_chan_s *chan, int offset,
                                 uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(chan->tbase + offset, clearbits, setbits);
}
#endif

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC channel.  Called early to initialize the hardware. This
 *   is called, before dac_setup() and on error conditions.
 *
 *   NOTE:  DAC reset will reset both DAC channels!
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_reset(FAR struct dac_dev_s *dev)
{
  irqstate_t flags;

  /* Reset only the selected DAC channel; the other DAC channel must remain
   * functional.
   */

  flags   = enter_critical_section();

#warning "Missing logic"

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts.  Interrupts
 *   are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_setup(FAR struct dac_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: dac_shutdown
 *
 * Description:
 *   Disable the DAC.  This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_shutdown(FAR struct dac_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: dac_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_txint(FAR struct dac_dev_s *dev, bool enable)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: dac_dmatxcallback
 *
 * Description:
 *   DMA callback function.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_DMA
static void dac_dmatxcallback(DMA_HANDLE handle, uint8_t isr, FAR void *arg)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)arg;
  struct dac_dev_s    *dev;

  DEBUGASSERT(chan);

  switch (chan->intf)
    {
#ifdef CONFIG_STM32L4_DAC1
      case 0:
        dev = &g_dac1dev;
        break;
#endif
#ifdef CONFIG_STM32L4_DAC2
      case 1:
        dev = &g_dac2dev;
        break;
#endif
      default:
        DEBUGPANIC();
        break;
    }

  DEBUGASSERT(dev->ad_priv == chan);

  /* Report the result of the transfer only if the TX callback has not already
   * reported an error.
   */

  if (chan->result == -EBUSY)
    {
      /* Save the result of the transfer if no error was previously reported. */

      chan->result = (isr & DMA_CHAN_TEIF_BIT) ? -EIO : OK;
      dac_txdone(dev);
    }
}
#endif

/****************************************************************************
 * Name: dac_send
 *
 * Description:
 *   Set the DAC output.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
  FAR struct stm32_chan_s *chan = dev->ad_priv;

  /* Enable DAC Channel */

  stm32l4_dac_modify_cr(chan, 0, DAC_CR_EN);

#ifdef HAVE_DMA
  if (chan->hasdma)
    {
      /* Copy the value to circular buffer. Since dmabuffer is initialized to zero,
       * writing e.g. monotonously increasing values creates a continuously repeating
       * ramp-effect, alternating with periods of zero output.
       *
       * In real use it the dmabuffer should be initialized with a desired pattern
       * beforehand, followed by a single dummy write to initiate circular DMA. If want
       * to write just one value at a time with DMA, set the buffer size to 1 (mostly
       * useful for just testing the functionality).
       */

      chan->dmabuffer[chan->buffer_pos] = (uint16_t)msg->am_data;
      chan->buffer_pos = (chan->buffer_pos + 1) % chan->buffer_len;

      /* Configure the DMA stream/channel.
       *
       * - Channel number
       * - Peripheral address
       * - Direction: Memory to peripheral
       * - Disable peripheral address increment
       * - Enable memory address increment
       * - Peripheral data size: half word
       * - Mode: circular???
       * - Priority: ?
       * - FIFO mode: disable
       * - FIFO threshold: half full
       * - Memory Burst: single
       * - Peripheral Burst: single
       */

      stm32l4_dmasetup(chan->dma, chan->dro, (uint32_t)chan->dmabuffer,
                       chan->buffer_len, DAC_DMA_CONTROL_WORD);

      /* Start the DMA */

      chan->result = -EBUSY;
      stm32l4_dmastart(chan->dma, dac_dmatxcallback, chan, false);

      /* Enable DMA for DAC Channel */

      stm32l4_dac_modify_cr(chan, 0, DAC_CR_DMAEN);
    }
  else
#endif
    {
      /* Non-DMA transfer */

      putreg16(msg->am_data, chan->dro);
      dac_txdone(dev);
    }

  /* Reset counters (generate an update) */

#ifdef HAVE_DMA
  tim_modifyreg(chan, STM32L4_BTIM_EGR_OFFSET, 0, ATIM_EGR_UG);
#endif
  return OK;
}

/****************************************************************************
 * Name: dac_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: dac_timinit
 *
 * Description:
 *   Initialize the timer that drivers the DAC DMA for this channel using
 *   the pre-calculated timer divider definitions.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef HAVE_DMA
static int dac_timinit(FAR struct stm32_chan_s *chan)
{
  uint32_t pclk;
  uint32_t prescaler;
  uint32_t timclk;
  uint32_t reload;
  uint32_t regaddr;
  uint32_t setbits;

  /* Configure the time base: Timer period, prescaler, clock division,
   * counter mode (up).
   */

  /* Enable the timer.  At most, two of the following cases (plus the
   * default) will be enabled
   */

  regaddr = STM32L4_RCC_APB1ENR1;

  switch (chan->timer)
    {
#ifdef NEED_TIM2
      case 2:
        setbits = RCC_APB1ENR1_TIM2EN;
        pclk    = BOARD_TIM2_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM3
      case 3:
        setbits = RCC_APB1ENR1_TIM3EN;
        pclk    = BOARD_TIM3_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM4
      case 4:
        setbits = RCC_APB1ENR1_TIM4EN;
        pclk    = BOARD_TIM4_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM5
      case 5:
        setbits = RCC_APB1ENR1_TIM5EN;
        pclk    = BOARD_TIM5_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM6
      case 6:
        setbits = RCC_APB1ENR1_TIM6EN;
        pclk    = BOARD_TIM6_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM7
      case 7:
        setbits = RCC_APB1ENR1_TIM7EN;
        pclk    = BOARD_TIM7_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM8
      case 8:
        regaddr = STM32L4_RCC_APB2ENR;
        setbits = RCC_APB2ENR_TIM8EN;
        pclk    = BOARD_TIM8_FREQUENCY;
        break;
#endif
      default:
        aerr("ERROR: Could not enable timer\n");
        return -EINVAL;
    }

  /* Enable the timer. */

  modifyreg32(regaddr, 0, setbits);

  /* Calculate optimal values for the timer prescaler and for the timer reload
   * register.  If 'frequency' is the desired frequency, then
   *
   *   reload = timclk / frequency
   *   timclk = pclk / presc
   *
   * Or,
   *
   *   reload = pclk / presc / frequency
   *
   * There are many solutions to this this, but the best solution will be the
   * one that has the largest reload value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   0 <= presc  <= 65536
   *   1 <= reload <= 65535
   *
   * So presc = pclk / 65535 / frequency would be optimal.
   *
   * Example:
   *
   *  pclk      = 42 MHz
   *  frequency = 100 Hz
   *
   *  prescaler = 42,000,000 / 65,535 / 100
   *            = 6.4 (or 7 -- taking the ceiling always)
   *  timclk    = 42,000,000 / 7
   *            = 6,000,000
   *  reload    = 6,000,000 / 100
   *            = 60,000
   */

  prescaler = (pclk / chan->tfrequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timclk = pclk / prescaler;

  reload = timclk / chan->tfrequency;
  if (reload < 1)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }

  /* Set the reload and prescaler values */

  tim_putreg(chan, STM32L4_BTIM_ARR_OFFSET, (uint16_t)reload);
  tim_putreg(chan, STM32L4_BTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  /* Count mode up, auto reload */

  tim_modifyreg(chan, STM32L4_BTIM_CR1_OFFSET, 0, ATIM_CR1_ARPE);

  /* Selection TRGO selection: update */

  tim_modifyreg(chan, STM32L4_BTIM_CR2_OFFSET, ATIM_CR2_MMS_MASK,
                ATIM_CR2_MMS_UPDATE);

  /* Update DMA request enable ???? */
#if 0
  tim_modifyreg(chan, STM32L4_BTIM_DIER_OFFSET, 0, ATIM_DIER_UDE);
#endif

  /* Enable the counter */

  tim_modifyreg(chan, STM32L4_BTIM_CR1_OFFSET, 0, ATIM_CR1_CEN);
  return OK;
}
#endif

/****************************************************************************
 * Name: dac_chaninit
 *
 * Description:
 *   Initialize the DAC channel.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_chaninit(FAR struct stm32_chan_s *chan)
{
  uint16_t clearbits;
  uint16_t setbits;

  /* Is the selected channel already in-use? */

  if (chan->inuse)
    {
      /* Yes.. then return EBUSY */

      return -EBUSY;
    }

  /* Configure the DAC output pin:
   *
   * DAC -" Once the DAC channelx is enabled, the corresponding GPIO pin
   * (PA4 or PA5) is automatically connected to the analog converter output
   * (DAC_OUTx). In order to avoid parasitic consumption, the PA4 or PA5 pin
   * should first be configured to analog (AIN)".
   */

  if (chan->pin != 0xffffffffU)
    {
      stm32l4_configgpio(chan->pin);
    }

  /* DAC channel configuration:
   *
   * - Set the trigger selection based upon the configuration.
   * - Set wave generation == None.
   * - Enable the output buffer.
   */

  /* Disable before change */

  stm32l4_dac_modify_cr(chan, DAC_CR_EN, 0);

  clearbits = DAC_CR_TSEL_MASK |
              DAC_CR_MAMP_MASK |
              DAC_CR_WAVE_MASK;
  setbits =
      chan->tsel |           /* Set trigger source (SW or timer TRGO event) */
      DAC_CR_MAMP_AMP1 |     /* Set waveform characteristics */
      DAC_CR_WAVE_DISABLED;  /* Set no noise */
  stm32l4_dac_modify_cr(chan, clearbits, setbits);

  /* Enable output buffer or route DAC output to on-chip peripherals (ADC) */

  clearbits = DAC_MCR_MODE1_MASK;
  if (chan->pin != 0xffffffffU)
    {
      setbits = DAC_MCR_MODE_EXTBUF;
    }
  else
    {
      setbits = DAC_MCR_MODE_IN;
    }
  stm32l4_dac_modify_mcr(chan, clearbits, setbits);

#ifdef HAVE_DMA
  /* Determine if DMA is supported by this channel */

  if (chan->hasdma)
    {
      int ret;

      /* Yes.. DAC trigger enable */

      stm32l4_dac_modify_cr(chan, 0, DAC_CR_TEN);

      /* Allocate a DMA channel */

      chan->dma = stm32l4_dmachannel(chan->dmachan);
      if (!chan->dma)
        {
          aerr("ERROR: Failed to allocate a DMA channel\n");
          return -EBUSY;
        }

      /* Configure the timer that supports the DMA operation */

      ret = dac_timinit(chan);
      if (ret < 0)
        {
          aerr("ERROR: Failed to initialize the DMA timer: %d\n", ret);
          stm32l4_dmafree(chan->dma);
          return ret;
        }
    }
#endif

  /* Mark the DAC channel "in-use" */

  chan->inuse = 1;
  return OK;
}

/****************************************************************************
 * Name: dac_blockinit
 *
 * Description:
 *   Initialize the DAC block.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void dac_blockinit(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Has the DAC block already been initialized? */

  if (g_dacblock.init)
    {
      return;
    }

  /* Put the entire DAC block in reset state */

  flags   = enter_critical_section();
  regval  = getreg32(STM32L4_RCC_APB1RSTR1);
  regval |= RCC_APB1RSTR1_DAC1RST;
  putreg32(regval, STM32L4_RCC_APB1RSTR1);

  /* Take the DAC out of reset state */

  regval &= ~RCC_APB1RSTR1_DAC1RST;
  putreg32(regval, STM32L4_RCC_APB1RSTR1);
  leave_critical_section(flags);

  /* Mark the DAC block as initialized */

  g_dacblock.init = 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_dacinitialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid dac device structure reference on success; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the DAC block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct dac_dev_s *stm32l4_dacinitialize(int intf)
{
  FAR struct dac_dev_s    *dev;
  FAR struct stm32_chan_s *chan;
  int ret;

  switch (intf)
    {
#ifdef CONFIG_STM32L4_DAC1
      case 0:
        ainfo("DAC1-1 Selected\n");
        dev = &g_dac1dev;
        break;
#endif
#ifdef CONFIG_STM32L4_DAC2
      case 1:
        ainfo("DAC1-2 Selected\n");
        dev = &g_dac2dev;
        break;
#endif
      default:
        aerr("ERROR: No such DAC interface: %d\n", intf);
        return NULL;
    }

  /* Make sure that the DAC block has been initialized */

  dac_blockinit();

  /* Configure the selected DAC channel */

  chan = dev->ad_priv;
  ret  = dac_chaninit(chan);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize DAC channel %d: %d\n", intf, ret);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_STM32L4_DAC1 || CONFIG_STM32L4_DAC2 */
#endif /* CONFIG_DAC */
