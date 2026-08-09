/****************************************************************************
 * arch/arm/src/nrf52/nrf52_adc.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "nrf52_gpio.h"
#include "nrf52_adc.h"
#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
#  include "nrf52_ppi.h"
#endif

#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
#  include "nrf52_ppi.h"
#  include "nrf52_tim.h"
#endif

#include "hardware/nrf52_saadc.h"
#include "hardware/nrf52_utils.h"
#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
#  include "hardware/nrf52_ppi.h"
#  include "hardware/nrf52_tim.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_NRF52_SAADC_TASK) && \
    !defined(CONFIG_NRF52_SAADC_TIMER) && \
    !defined(CONFIG_NRF52_SAADC_TIMER_PPI)
#  error SAADC trigger not selected
#endif

#if defined(CONFIG_NRF52_SAADC_TIMER_PPI) && \
    defined(CONFIG_NRF52_SAADC_CONTINUOUS)
#  if CONFIG_NRF52_SAADC_PPI_CHANNEL == CONFIG_NRF52_SAADC_CONTINUOUS_PPI_CH
#    error SAADC sample and restart PPI channels must differ
#  endif
#endif

#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
/* Samples in each of the two EasyDMA buffers.  A scan is one conversion of
 * every enabled channel, so a buffer holds CONTINUOUS_BUFLEN whole scans.
 */

#  define NRF52_SAADC_DMALEN (CONFIG_NRF52_SAADC_CHANNELS * \
                              CONFIG_NRF52_SAADC_CONTINUOUS_BUFLEN)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_adc_s
{
  /* Upper-half callback */

  const struct adc_callback_s *cb;

  /* Channels configuration */

  struct nrf52_adc_channel_s channels[CONFIG_NRF52_SAADC_CHANNELS];

  /* Samples buffer */

  int16_t                    buffer[CONFIG_NRF52_SAADC_CHANNELS];

#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
  /* Double-buffered EasyDMA for gapless continuous sampling. The SAADC
   * fills one dmabuf while the driver processes the other; the PPI channel
   * auto-restarts the SAADC on END and the STARTED interrupt reloads the
   * DMA pointer to the free buffer. 'next' is the buffer being filled.
   *
   * With more than one channel enabled a buffer holds whole interleaved
   * scans and 'chanmap' carries the channel index of every sample.
   */

  int16_t                    dmabuf[2][NRF52_SAADC_DMALEN];
  uint32_t                   batch[NRF52_SAADC_DMALEN];
  uint8_t                    chanmap[NRF52_SAADC_DMALEN];
  uint16_t                   dmalen;  /* Samples per buffer: chan_len*BUFLEN */
  uint8_t                    next;
#endif

  uint8_t                    chan_len;   /* Configured channels */
  uint32_t                   base;       /* Base address of ADC register */
  uint32_t                   irq;        /* ADC interrupt */
  uint8_t                    resolution; /* ADC resolution */
#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
  struct nrf52_tim_dev_s    *tim;        /* Timer used for PPI sampling */
  bool                       ppi_en;     /* PPI channel enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

static inline void nrf52_adc_putreg(struct nrf52_adc_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf52_adc_getreg(struct nrf52_adc_s *priv,
                                        uint32_t offset);

/* ADC helpers */

static int nrf52_adc_configure(struct nrf52_adc_s *priv);
static int nrf52_adc_calibrate(struct nrf52_adc_s *priv);
static uint32_t nrf52_adc_ch_config(struct nrf52_adc_channel_s *cfg);
static uint32_t nrf52_adc_chanpsel(int psel);
static int nrf52_adc_chancfg(struct nrf52_adc_s *priv, uint8_t chan,
                             struct nrf52_adc_channel_s *cfg);
static int nrf52_adc_isr(int irq, void *context, void *arg);
#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
static int nrf52_adc_ppi_setup(struct nrf52_adc_s *priv);
static void nrf52_adc_ppi_shutdown(struct nrf52_adc_s *priv);
static int nrf52_adc_ppi_start(struct nrf52_adc_s *priv);
#endif

/* ADC Driver Methods */

static int  nrf52_adc_bind(struct adc_dev_s *dev,
                           const struct adc_callback_s *callback);
static void nrf52_adc_reset(struct adc_dev_s *dev);
static int  nrf52_adc_setup(struct adc_dev_s *dev);
static void nrf52_adc_shutdown(struct adc_dev_s *dev);
static void nrf52_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  nrf52_adc_ioctl(struct adc_dev_s *dev, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_nrf52_adcops =
{
  .ao_bind        = nrf52_adc_bind,
  .ao_reset       = nrf52_adc_reset,
  .ao_setup       = nrf52_adc_setup,
  .ao_shutdown    = nrf52_adc_shutdown,
  .ao_rxint       = nrf52_adc_rxint,
  .ao_ioctl       = nrf52_adc_ioctl,
};

/* SAADC device */

struct nrf52_adc_s g_nrf52_adcpriv =
{
  .cb         = NULL,
  .base       = NRF52_SAADC_BASE,
  .irq        = NRF52_IRQ_SAADC,
  .resolution = CONFIG_NRF52_SAADC_RESOLUTION,
#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
  .tim        = NULL,
  .ppi_en     = false
#endif
};

/* Upper-half ADC device */

static struct adc_dev_s g_nrf52_adc =
{
  .ad_ops      = &g_nrf52_adcops,
  .ad_priv     = &g_nrf52_adcpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_adc_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf52_adc_putreg(struct nrf52_adc_s *priv,
                                    uint32_t offset,
                                    uint32_t value)
{
  DEBUGASSERT(priv);

  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_adc_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf52_adc_getreg(struct nrf52_adc_s *priv,
                                        uint32_t offset)
{
  DEBUGASSERT(priv);

  return getreg32(priv->base + offset);
}

#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
/****************************************************************************
 * Name: nrf52_adc_tim_base
 *
 * Description:
 *   Get TIMER base address for configured SAADC PPI trigger timer.
 *
 ****************************************************************************/

static inline uint32_t nrf52_adc_tim_base(void)
{
#if CONFIG_NRF52_SAADC_PPI_TIMER == 0
  return NRF52_TIMER0_BASE;
#elif CONFIG_NRF52_SAADC_PPI_TIMER == 1
  return NRF52_TIMER1_BASE;
#elif CONFIG_NRF52_SAADC_PPI_TIMER == 2
  return NRF52_TIMER2_BASE;
#elif CONFIG_NRF52_SAADC_PPI_TIMER == 3
  return NRF52_TIMER3_BASE;
#elif CONFIG_NRF52_SAADC_PPI_TIMER == 4
  return NRF52_TIMER4_BASE;
#else
#  error Unsupported CONFIG_NRF52_SAADC_PPI_TIMER value
#endif
}

/****************************************************************************
 * Name: nrf52_adc_ppi_setup
 *
 * Description:
 *   Configure a general purpose TIMER and route its compare event to SAADC
 *   SAMPLE task over PPI.
 *
 ****************************************************************************/

static int nrf52_adc_ppi_setup(struct nrf52_adc_s *priv)
{
  uint32_t tim_base = 0;
  uint32_t ppi_ch   = CONFIG_NRF52_SAADC_PPI_CHANNEL;
  int      ret      = OK;

  DEBUGASSERT(priv);

  priv->tim = nrf52_tim_init(CONFIG_NRF52_SAADC_PPI_TIMER);
  if (priv->tim == NULL)
    {
      aerr("ERROR: failed to get TIMER%d for SAADC PPI\n",
           CONFIG_NRF52_SAADC_PPI_TIMER);
      ret = -EBUSY;
      goto errout;
    }

  tim_base = nrf52_adc_tim_base();

  /* Configure TIMER for periodic compare events */

  NRF52_TIM_STOP(priv->tim);
  NRF52_TIM_CLEAR(priv->tim);
  NRF52_TIM_CONFIGURE(priv->tim, NRF52_TIM_MODE_TIMER, NRF52_TIM_WIDTH_32B);
  NRF52_TIM_SETPRE(priv->tim, CONFIG_NRF52_SAADC_PPI_PRE);
  NRF52_TIM_SETCC(priv->tim, NRF52_TIM_CC0, CONFIG_NRF52_SAADC_PPI_CC);
  NRF52_TIM_SHORTS(priv->tim, NRF52_TIM_SHORT_COMPARE_CLEAR,
                   NRF52_TIM_CC0, true);

  /* Ensure compare event is clear before enabling PPI channel */

  putreg32(0, tim_base + NRF52_TIM_EVENTS_COMPARE_OFFSET(0));

  if ((getreg32(NRF52_PPI_CHEN) & PPI_CHEN_CH(ppi_ch)) != 0)
    {
      aerr("ERROR: PPI channel %ld already in use\n", (long)ppi_ch);
      ret = -EBUSY;
      goto errout;
    }

  nrf52_ppi_set_event_ep(ppi_ch,
                         tim_base + NRF52_TIM_EVENTS_COMPARE_OFFSET(0));
  nrf52_ppi_set_task_ep(ppi_ch,
                        priv->base + NRF52_SAADC_TASKS_SAMPLE_OFFSET);
  nrf52_ppi_channel_enable(ppi_ch, true);
  priv->ppi_en = true;

errout:
  if (ret < 0)
    {
      nrf52_adc_ppi_shutdown(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_adc_ppi_start
 *
 * Description:
 *   Start TIMER used for PPI-driven sampling.
 *
 ****************************************************************************/

static int nrf52_adc_ppi_start(struct nrf52_adc_s *priv)
{
  int ret = OK;

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->tim != NULL);

  /* Clear event/counter and then start periodic triggering */

  NRF52_TIM_STOP(priv->tim);
  NRF52_TIM_CLEAR(priv->tim);
  ret = NRF52_TIM_START(priv->tim);

  return ret;
}

/****************************************************************************
 * Name: nrf52_adc_ppi_shutdown
 *
 * Description:
 *   Release TIMER/PPI resources used for PPI-driven sampling.
 *
 ****************************************************************************/

static void nrf52_adc_ppi_shutdown(struct nrf52_adc_s *priv)
{
  uint32_t ppi_ch = CONFIG_NRF52_SAADC_PPI_CHANNEL;

  DEBUGASSERT(priv);

  if (priv->ppi_en)
    {
      nrf52_ppi_channel_enable(ppi_ch, false);
      priv->ppi_en = false;
    }

  if (priv->tim != NULL)
    {
      NRF52_TIM_SHORTS(priv->tim, NRF52_TIM_SHORT_COMPARE_CLEAR,
                       NRF52_TIM_CC0, false);
      NRF52_TIM_STOP(priv->tim);
      nrf52_tim_deinit(priv->tim);
      priv->tim = NULL;
    }
}
#endif

/****************************************************************************
 * Name: nrf52_adc_isr
 *
 * Description:
 *   Common ADC interrupt service routine
 *
 ****************************************************************************/

static int nrf52_adc_isr(int irq, void *context, void *arg)
{
  struct adc_dev_s   *dev  = (struct adc_dev_s *) arg;
  struct nrf52_adc_s *priv = NULL;
  int                 ret  = OK;
  int                 i    = 0;

  DEBUGASSERT(dev);

  priv = (struct nrf52_adc_s *) dev->ad_priv;
  DEBUGASSERT(priv);

  ainfo("nrf52_adc_isr\n");

#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
  /* END event: the current DMA buffer is full. Hand it to the upper half
   * as a batch. The PPI channel has already retriggered START (into the
   * buffer the STARTED handler preloaded). END must be handled before
   * STARTED - both can be pending at once: END consumes priv->next and
   * advances it, then STARTED uses the advanced value to reload the free
   * buffer's DMA pointer.
   */

  if (nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_END_OFFSET) == 1)
    {
      DEBUGASSERT(priv->cb != NULL);
      DEBUGASSERT(priv->cb->au_receive_batch != NULL);

      /* Give the completed buffer to the ADC driver */

      for (i = 0; i < priv->dmalen; i += 1)
        {
          priv->batch[i] = (uint32_t)priv->dmabuf[priv->next][i];
        }

      ret = priv->cb->au_receive_batch(dev, priv->chanmap, priv->batch,
                                       priv->dmalen);
      if (ret == -ENOMEM)
        {
          /* Receive FIFO overrun */

          DEBUGASSERT(priv->cb->au_reset != NULL);
          priv->cb->au_reset(dev);
        }

      /* The next END completes the other buffer */

      priv->next ^= 1;

      /* Clear event (read back to flush the write so the IRQ does not
       * spuriously re-fire before the clear takes effect).
       */

      nrf52_adc_putreg(priv, NRF52_SAADC_EVENTS_END_OFFSET, 0);
      (void)nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_END_OFFSET);
    }

  /* STARTED event: the SAADC latched the preloaded pointer and began
   * sampling. Point the DMA pointer at the other (now free) buffer so the
   * PPI END->START restart fills it next.
   */

  if (nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_STARTED_OFFSET) == 1)
    {
      nrf52_adc_putreg(priv, NRF52_SAADC_PTR_OFFSET,
                       (uint32_t)&priv->dmabuf[priv->next ^ 1]);

      /* Clear event (read back to flush the write) */

      nrf52_adc_putreg(priv, NRF52_SAADC_EVENTS_STARTED_OFFSET, 0);
      (void)nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_STARTED_OFFSET);
    }
#else
  /* END event */

  if (nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_END_OFFSET) == 1)
    {
      DEBUGASSERT(priv->cb != NULL);
      DEBUGASSERT(priv->cb->au_receive != NULL);

      /* Give the ADC data to the ADC driver */

      for (i = 0; i < priv->chan_len; i += 1)
        {
          priv->cb->au_receive(dev, i, priv->buffer[i]);
        }

      /* Clear event */

      nrf52_adc_putreg(priv, NRF52_SAADC_EVENTS_END_OFFSET, 0);

#if defined(CONFIG_NRF52_SAADC_TIMER) || defined(CONFIG_NRF52_SAADC_TIMER_PPI)
      /* In timer mode, END means the DMA buffer is full.  Re-start the
       * SAADC so that the trigger source keeps producing conversions
       * after a single ANIOC_TRIGGER.  This applies to both the internal
       * sample timer and an external TIMER feeding SAMPLE over PPI.
       */

      nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_START_OFFSET, 1);
#endif
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nrf52_adc_configure
 *
 * Description:
 *   Configure ADC
 *
 ****************************************************************************/

static int nrf52_adc_configure(struct nrf52_adc_s *priv)
{
  int regval = 0;
#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
  int i      = 0;
#endif

  DEBUGASSERT(priv);

  /* Configure ADC resolution */

  regval = CONFIG_NRF52_SAADC_RESOLUTION;
  nrf52_adc_putreg(priv, NRF52_SAADC_RESOLUTION_OFFSET, regval);

  /* Configure oversampling */

  regval = CONFIG_NRF52_SAADC_OVERSAMPLE;
  nrf52_adc_putreg(priv, NRF52_SAADC_OVERSAMPLE_OFFSET, regval);

  /* Configure sample rate */

#ifdef CONFIG_NRF52_SAADC_TIMER
  /* Sampling driven by the SAADC internal timer */

  regval = SAADC_SAMPLERATE_MODE_TIMERS;
  regval |= ((CONFIG_NRF52_SAADC_TIMER_CC & SAADC_SAMPLERATE_CC_MASK)
             << SAADC_SAMPLERATE_CC_SHIFT);
#else
  /* Sampling driven by the SAMPLE task, written either by the CPU
   * (NRF52_SAADC_TASK) or by a TIMER compare event over PPI
   * (NRF52_SAADC_TIMER_PPI).
   */

  regval = SAADC_SAMPLERATE_MODE_TASK;
#endif

  nrf52_adc_putreg(priv, NRF52_SAADC_SAMPLERATE_OFFSET, regval);

  /* Configure ADC buffer */

#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
  /* Continuous mode: gapless double-buffered DMA auto-restarted by PPI.
   * Each buffer holds CONTINUOUS_BUFLEN scans of all enabled channels.
   */

  priv->next   = 0;
  priv->dmalen = priv->chan_len * CONFIG_NRF52_SAADC_CONTINUOUS_BUFLEN;

  DEBUGASSERT(priv->dmalen <= SAADC_MAXCNT_MASK);

  /* Samples are interleaved scan by scan, so the channel index simply
   * cycles.  Build the map once instead of per interrupt.
   */

  for (i = 0; i < priv->dmalen; i += 1)
    {
      priv->chanmap[i] = i % priv->chan_len;
    }

  regval = (uint32_t)&priv->dmabuf[0];
  DEBUGASSERT(nrf52_easydma_valid(regval));
  nrf52_adc_putreg(priv, NRF52_SAADC_PTR_OFFSET, regval);

  nrf52_adc_putreg(priv, NRF52_SAADC_MAXCNT_OFFSET, priv->dmalen);

  /* Auto-restart the SAADC: END event -> START task via PPI */

  nrf52_ppi_set_event_ep(CONFIG_NRF52_SAADC_CONTINUOUS_PPI_CH,
                         priv->base + NRF52_SAADC_EVENTS_END_OFFSET);
  nrf52_ppi_set_task_ep(CONFIG_NRF52_SAADC_CONTINUOUS_PPI_CH,
                        priv->base + NRF52_SAADC_TASKS_START_OFFSET);
#else
  regval = (uint32_t)&priv->buffer;
  DEBUGASSERT(nrf52_easydma_valid(regval));
  nrf52_adc_putreg(priv, NRF52_SAADC_PTR_OFFSET, regval);

  regval = priv->chan_len;
  DEBUGASSERT(regval <= SAADC_MAXCNT_MASK);
  nrf52_adc_putreg(priv, NRF52_SAADC_MAXCNT_OFFSET, regval);
#endif

  return OK;
}

/****************************************************************************
 * Name: nrf52_adc_calibrate
 *
 * Description:
 *   Calibrate ADC
 *
 ****************************************************************************/

static int nrf52_adc_calibrate(struct nrf52_adc_s *priv)
{
  /* Clear Event */

  nrf52_adc_putreg(priv, NRF52_SAADC_EVENTS_CALDONE_OFFSET, 0);

  /* Start calibration */

  nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_CALOFFSET_OFFSET, 1);

  /* Wait for calibration done */

  while (nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_CALDONE_OFFSET) != 1);

  return OK;
}

/****************************************************************************
 * Name: nrf52_adc_ch_config
 ****************************************************************************/

static uint32_t nrf52_adc_ch_config(struct nrf52_adc_channel_s *cfg)
{
  uint32_t regval = 0;

  /* Positive channel resistor control */

  switch (cfg->resp)
    {
      case NRF52_ADC_RES_BYPASS:
        {
          regval |= SAADC_CONFIG_RESP_NONE;
          break;
        }

      case NRF52_ADC_RES_PULLDOWN:
        {
          regval |= SAADC_CONFIG_RESP_PD;
          break;
        }

      case NRF52_ADC_RES_PULLUP:
        {
          regval |= SAADC_CONFIG_RESP_PU;
          break;
        }

      case NRF52_ADC_RES_VDD_2:
        {
          regval |= SAADC_CONFIG_RESP_VDD1P2;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->resp: %d\n", cfg->resp);
        }
    }

  /* Negative channel resistor control */

  switch (cfg->resn)
    {
      case NRF52_ADC_RES_BYPASS:
        {
          regval |= SAADC_CONFIG_RESN_NONE;
          break;
        }

      case NRF52_ADC_RES_PULLDOWN:
        {
          regval |= SAADC_CONFIG_RESN_PD;
          break;
        }

      case NRF52_ADC_RES_PULLUP:
        {
          regval |= SAADC_CONFIG_RESN_PU;
          break;
        }

      case NRF52_ADC_RES_VDD_2:
        {
          regval |= SAADC_CONFIG_RESN_VDD1P2;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->resn: %d\n", cfg->resn);
        }
    }

  /* Gain control */

  switch (cfg->gain)
    {
      case NRF52_ADC_GAIN_1_6:
        {
          regval |= SAADC_CONFIG_GAIN_1P6;
          break;
        }

      case NRF52_ADC_GAIN_1_5:
        {
          regval |= SAADC_CONFIG_GAIN_1P5;
          break;
        }

      case NRF52_ADC_GAIN_1_4:
        {
          regval |= SAADC_CONFIG_GAIN_1P4;
          break;
        }

      case NRF52_ADC_GAIN_1_3:
        {
          regval |= SAADC_CONFIG_GAIN_1P3;
          break;
        }

      case NRF52_ADC_GAIN_1_2:
        {
          regval |= SAADC_CONFIG_GAIN_1P2;
          break;
        }

      case NRF52_ADC_GAIN_1:
        {
          regval |= SAADC_CONFIG_GAIN_1;
          break;
        }

      case NRF52_ADC_GAIN_2:
        {
          regval |= SAADC_CONFIG_GAIN_2;
          break;
        }

      case NRF52_ADC_GAIN_4:
        {
          regval |= SAADC_CONFIG_GAIN_4;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->gain: %d\n", cfg->gain);
        }
    }

  /* Reference control */

  switch (cfg->refsel)
    {
      case NRF52_ADC_REFSEL_INTERNAL:
        {
          regval |= SAADC_CONFIG_REFSEL_INTERNAL;
          break;
        }

      case NRF52_ADC_REFSEL_VDD_4:
        {
          regval |= SAADC_CONFIG_REFSEL_VDD1P4;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->refsel: %d\n", cfg->refsel);
        }
    }

  /* Acquisition time */

  switch (cfg->tacq)
    {
      case NRF52_ADC_TACQ_3US:
        {
          regval |= SAADC_CONFIG_TACQ_3US;
          break;
        }

      case NRF52_ADC_TACQ_5US:
        {
          regval |= SAADC_CONFIG_TACQ_5US;
          break;
        }

      case NRF52_ADC_TACQ_10US:
        {
          regval |= SAADC_CONFIG_TACQ_10US;
          break;
        }

      case NRF52_ADC_TACQ_15US:
        {
          regval |= SAADC_CONFIG_TACQ_15US;
          break;
        }

      case NRF52_ADC_TACQ_20US:
        {
          regval |= SAADC_CONFIG_TACQ_20US;
          break;
        }

      case NRF52_ADC_TACQ_40US:
        {
          regval |= SAADC_CONFIG_TACQ_40US;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->tacq: %d\n", cfg->tacq);
        }
    }

  /* Singe-ended or differential mode */

  switch (cfg->mode)
    {
      case NRF52_ADC_MODE_SE:
        {
          regval |= SAADC_CONFIG_MODE_SE;
          break;
        }

      case NRF52_ADC_MODE_DIFF:
        {
          regval |= SAADC_CONFIG_MODE_DIFF;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->mode: %d\n", cfg->mode);
        }
    }

  /* Burst mode */

  switch (cfg->burst)
    {
      case NRF52_ADC_BURST_DISABLE:
        {
          regval |= SAADC_CONFIG_BURS_DIS;
          break;
        }

      case NRF52_ADC_BURST_ENABLE:
        {
          regval |= SAADC_CONFIG_BURS_EN;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->burst: %d\n", cfg->burst);
        }
    }

  return regval;
}

/****************************************************************************
 * Name: nrf52_adc_chanpsel
 ****************************************************************************/

static uint32_t nrf52_adc_chanpsel(int psel)
{
  uint32_t regval = 0;

  switch (psel)
    {
      case NRF52_ADC_IN_NC:
        {
          regval = SAADC_CHPSEL_NC;
          break;
        }

      case NRF52_ADC_IN_IN0:
        {
          regval = SAADC_CHPSEL_IN0;
          break;
        }

      case NRF52_ADC_IN_IN1:
        {
          regval = SAADC_CHPSEL_IN1;
          break;
        }

      case NRF52_ADC_IN_IN2:
        {
          regval = SAADC_CHPSEL_IN2;
          break;
        }

      case NRF52_ADC_IN_IN3:
        {
          regval = SAADC_CHPSEL_IN3;
          break;
        }

      case NRF52_ADC_IN_IN4:
        {
          regval = SAADC_CHPSEL_IN4;
          break;
        }

      case NRF52_ADC_IN_IN5:
        {
          regval = SAADC_CHPSEL_IN5;
          break;
        }

      case NRF52_ADC_IN_IN6:
        {
          regval = SAADC_CHPSEL_IN6;
          break;
        }

      case NRF52_ADC_IN_IN7:
        {
          regval = SAADC_CHPSEL_IN7;
          break;
        }

      case NRF52_ADC_IN_VDD:
        {
          regval = SAADC_CHPSEL_VDD;
          break;
        }

      case NRF52_ADC_IN_VDDHDIV5:
        {
          regval = SAADC_CHPSEL_VDDHDIV5;
          break;
        }

      default:
        {
          aerr("ERROR: invalid psel: %d\n", psel);
        }
    }

  return regval;
}

/****************************************************************************
 * Name: nrf52_adc_chancfg
 *
 * Description:
 *   Configure ADC channel
 *
 ****************************************************************************/

static int nrf52_adc_chancfg(struct nrf52_adc_s *priv, uint8_t chan,
                             struct nrf52_adc_channel_s *cfg)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(priv);

  /* Configure positive input */

  regval = nrf52_adc_chanpsel(cfg->p_psel);
  nrf52_adc_putreg(priv, NRF52_SAADC_CHPSELP_OFFSET(chan), regval);

  /* Configure negative input */

  regval = nrf52_adc_chanpsel(cfg->n_psel);
  nrf52_adc_putreg(priv, NRF52_SAADC_CHPSELN_OFFSET(chan), regval);

  /* Get channel configuration */

  regval = nrf52_adc_ch_config(cfg);

  /* Write channel configuration */

  nrf52_adc_putreg(priv, NRF52_SAADC_CHCONFIG_OFFSET(chan), regval);

#ifdef CONFIG_NRF52_SAADC_LIMITS
  /* Configure limits */

  regval = (cfg->limith < 16) | (cfg->limith << 0);
  nrf52_adc_putreg(priv, NRF52_SAADC_CHLIMIT_OFFSET(chan), regval);
#endif

  return ret;
}

/****************************************************************************
 * Name: nrf52_adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int nrf52_adc_bind(struct adc_dev_s *dev,
                          const struct adc_callback_s *callback)
{
  struct nrf52_adc_s *priv = (struct nrf52_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: nrf52_adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware.
 *   This is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void nrf52_adc_reset(struct adc_dev_s *dev)
{
  struct nrf52_adc_s *priv = (struct nrf52_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  /* TODO */

  UNUSED(priv);
}

/****************************************************************************
 * Name: nrf52_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int nrf52_adc_setup(struct adc_dev_s *dev)
{
  struct nrf52_adc_s *priv = (struct nrf52_adc_s *) dev->ad_priv;
  int                 i    = 0;
  int                 ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  /* Disable ADC */

  nrf52_adc_putreg(priv, NRF52_SAADC_ENABLE_OFFSET, 0);

  /* Configure ADC */

  ret = nrf52_adc_configure(priv);
  if (ret < 0)
    {
      aerr("ERROR: nrf52_adc_configure failed: %d\n", ret);
      goto errout;
    }

  /* Configure ADC channels */

  for (i = 0; i < priv->chan_len; i += 1)
    {
      ret = nrf52_adc_chancfg(priv, i, &priv->channels[i]);
      if (ret < 0)
        {
          aerr("ERROR: chancfg failed: %d %d\n", i, ret);
          goto errout;
        }
    }

#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
  /* Acquire the TIMER and PPI channel that drive TASKS_SAMPLE */

  ret = nrf52_adc_ppi_setup(priv);
  if (ret < 0)
    {
      aerr("ERROR: nrf52_adc_ppi_setup failed: %d\n", ret);
      goto errout;
    }
#endif

  /* Enable ADC */

  nrf52_adc_putreg(priv, NRF52_SAADC_ENABLE_OFFSET, 1);

  /* Calibrate ADC */

  ret = nrf52_adc_calibrate(priv);
  if (ret < 0)
    {
      aerr("ERROR: adc calibration failed: %d\n", ret);
      goto errout;
    }

  /* Attach the ADC interrupt */

  ret = irq_attach(priv->irq, nrf52_adc_isr, dev);
  if (ret < 0)
    {
      aerr("ERROR: irq_attach failed: %d\n", ret);
      goto errout;
    }

  /* Enable the ADC interrupt */

  up_enable_irq(priv->irq);

errout:
#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
  if (ret < 0)
    {
      nrf52_adc_ppi_shutdown(priv);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nrf52_adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void nrf52_adc_shutdown(struct adc_dev_s *dev)
{
  struct nrf52_adc_s *priv = (struct nrf52_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
  /* Stop the trigger TIMER before halting the SAADC */

  if (priv->tim != NULL)
    {
      NRF52_TIM_STOP(priv->tim);
    }
#endif

#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
  /* Stop the auto-restart PPI channel so STOP actually halts the SAADC */

  nrf52_ppi_channel_enable(CONFIG_NRF52_SAADC_CONTINUOUS_PPI_CH, false);
#endif

  /* Stop SAADC */

  nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_STOP_OFFSET, 1);

  /* Wait for SAADC stopped */

  while (nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_STOPPED_OFFSET) != 1);

  /* Disable SAADC */

  nrf52_adc_putreg(priv, NRF52_SAADC_ENABLE_OFFSET, 0);

#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
  /* Release the trigger TIMER and its PPI channel */

  nrf52_adc_ppi_shutdown(priv);
#endif
}

/****************************************************************************
 * Name: nrf52_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 ****************************************************************************/

static void nrf52_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct nrf52_adc_s *priv   = (struct nrf52_adc_s *) dev->ad_priv;
  uint32_t            regval = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  ainfo("RXINT enable: %d\n", enable ? 1 : 0);

#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
  /* STARTED reloads the double-buffer DMA pointer; END delivers batches */

  regval = SAADC_INT_END | SAADC_INT_STARTED;
#else
  regval = SAADC_INT_END;
#endif

  if (enable)
    {
      nrf52_adc_putreg(priv, NRF52_SAADC_INTENSET_OFFSET, regval);
    }
  else
    {
      nrf52_adc_putreg(priv, NRF52_SAADC_INTENCLR_OFFSET, regval);
    }
}

/****************************************************************************
 * Name: nrf52_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int nrf52_adc_ioctl(struct adc_dev_s *dev, int cmd,
                           unsigned long arg)
{
  struct nrf52_adc_s *priv = (struct nrf52_adc_s *) dev->ad_priv;
  int ret                  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
#ifdef CONFIG_NRF52_SAADC_CONTINUOUS
          /* Enable the auto-restart PPI channel so the SAADC is restarted
           * from its own END event without the CPU in the loop.
           */

          nrf52_ppi_channel_enable(CONFIG_NRF52_SAADC_CONTINUOUS_PPI_CH,
                                   true);
#endif

          /* Start ADC */

          nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_START_OFFSET, 1);

          /* Trigger sampling.  With NRF52_SAADC_TIMER_PPI the TIMER
           * compare event drives TASKS_SAMPLE over PPI; otherwise the
           * first conversion is kicked here.
           */

#ifdef CONFIG_NRF52_SAADC_TIMER_PPI
          ret = nrf52_adc_ppi_start(priv);
          if (ret < 0)
            {
              aerr("ERROR: failed to start SAADC PPI timer: %d\n", ret);
            }
#else
          nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_SAMPLE_OFFSET, 1);
#endif
        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->chan_len;
        }
        break;

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_adcinitialize
 *
 * Description:
 *   Initialize the ADC. See nrf52_adc.c for more details.
 *
 * Input Parameters:
 *   chanlist  - channels configuration
 *   nchannels - number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *nrf52_adcinitialize(
    const struct nrf52_adc_channel_s *chan, int channels)
{
  struct adc_dev_s   *dev  = NULL;
  struct nrf52_adc_s *priv = NULL;
  int                 i    = 0;

  DEBUGASSERT(chan != NULL);
  DEBUGASSERT(channels <= CONFIG_NRF52_SAADC_CHANNELS);

#ifdef CONFIG_NRF52_SAADC_TIMER
  if (channels > 1)
    {
      aerr("ERROR: timer trigger works only for 1 channel!\n");
      goto errout;
    }
#endif

  /* Get device */

  dev = &g_nrf52_adc;

  /* Get private data */

  priv = (struct nrf52_adc_s *) dev->ad_priv;

  /* Copy channels configuration */

  ainfo("channels: %d\n", channels);

  for (i = 0; i < channels; i += 1)
    {
      memcpy(&priv->channels[i], &chan[i],
             sizeof(struct nrf52_adc_channel_s));
    }

  priv->chan_len = channels;

#ifdef CONFIG_NRF52_SAADC_TIMER
errout:
#endif
  return dev;
}
