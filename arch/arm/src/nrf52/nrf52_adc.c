/****************************************************************************
 * arch/arm/src/nrf52/nrf52_adc.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_arch.h"

#include "nrf52_gpio.h"
#include "nrf52_adc.h"

#include "hardware/nrf52_saadc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_adc_s
{
  /* Upper-half callback */

  FAR const struct adc_callback_s *cb;

  /* Channels configuration */

  struct nrf52_adc_channel_s channels[CONFIG_NRF52_SAADC_CHANNELS];

  /* Samples buffer */

  int16_t                    buffer[CONFIG_NRF52_SAADC_CHANNELS];

  uint8_t                    chan_len;   /* Configured channels */
  uint32_t                   base;       /* Base address of ADC register */
  uint32_t                   irq;        /* ADC interrupt */
  uint8_t                    resolution; /* ADC resolution */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

static inline void nrf52_adc_putreg(FAR struct nrf52_adc_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf52_adc_getreg(FAR struct nrf52_adc_s *priv,
                                        uint32_t offset);

/* ADC helpers */

static int nrf52_adc_configure(FAR struct nrf52_adc_s *priv);
static int nrf52_adc_calibrate(FAR struct nrf52_adc_s *priv);
static uint32_t nrf52_adc_ch_config(FAR struct nrf52_adc_channel_s *cfg);
static uint32_t nrf52_adc_chanpsel(int psel);
static int nrf52_adc_chancfg(FAR struct nrf52_adc_s *priv, uint8_t chan,
                             FAR struct nrf52_adc_channel_s *cfg);
static int nrf52_adc_isr(int irq, void *context, FAR void *arg);

/* ADC Driver Methods */

static int  nrf52_adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void nrf52_adc_reset(FAR struct adc_dev_s *dev);
static int  nrf52_adc_setup(FAR struct adc_dev_s *dev);
static void nrf52_adc_shutdown(FAR struct adc_dev_s *dev);
static void nrf52_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  nrf52_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
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
  .resolution = CONFIG_NRF52_SAADC_RESOLUTION
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

static inline void nrf52_adc_putreg(FAR struct nrf52_adc_s *priv,
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

static inline uint32_t nrf52_adc_getreg(FAR struct nrf52_adc_s *priv,
                                        uint32_t offset)
{
  DEBUGASSERT(priv);

  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_adc_isr
 *
 * Description:
 *   Common ADC interrupt service routine
 *
 ****************************************************************************/

static int nrf52_adc_isr(int irq, void *context, FAR void *arg)
{
  FAR struct adc_dev_s   *dev  = (FAR struct adc_dev_s *) arg;
  FAR struct nrf52_adc_s *priv = NULL;
  int                     ret  = OK;
  int                     i    = 0;

  DEBUGASSERT(dev);

  priv = (FAR struct nrf52_adc_s *) dev->ad_priv;
  DEBUGASSERT(priv);

  ainfo("nrf52_adc_isr\n");

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
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_adc_configure
 *
 * Description:
 *   Configure ADC
 *
 ****************************************************************************/

static int nrf52_adc_configure(FAR struct nrf52_adc_s *priv)
{
  int regval = 0;

  DEBUGASSERT(priv);

  /* Configure ADC resolution */

  regval = CONFIG_NRF52_SAADC_RESOLUTION;
  nrf52_adc_putreg(priv, NRF52_SAADC_RESOLUTION_OFFSET, regval);

  /* Configure oversampling */

  regval = CONFIG_NRF52_SAADC_OVERSAMPLE;
  nrf52_adc_putreg(priv, NRF52_SAADC_OVERSAMPLE_OFFSET, regval);

  /* Configure sample rate */

#if defined(CONFIG_NRF52_SAADC_TIMER)
  /* Trigger from local timer */

  regval = SAADC_SAMPLERATE_MODE_TIMERS;
  regval |= ((CONFIG_NRF52_SAADC_TIMER_CC & SAADC_SAMPLERATE_CC_MASK)
             << SAADC_SAMPLERATE_CC_SHIFT);
#elif defined(CONFIG_NRF52_SAADC_TASK)
  /* Trigger on SAMPLE tas */

  regval = SAADC_SAMPLERATE_MODE_TASK;
#else
#  error SAADC trigger not selected
#endif

  nrf52_adc_putreg(priv, NRF52_SAADC_SAMPLERATE_OFFSET, regval);

  /* Configure ADC buffer */

  regval = (uint32_t)&priv->buffer;
  nrf52_adc_putreg(priv, NRF52_SAADC_PTR_OFFSET, regval);

  regval = priv->chan_len;
  nrf52_adc_putreg(priv, NRF52_SAADC_MAXCNT_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: nrf52_adc_calibrate
 *
 * Description:
 *   Calibrate ADC
 *
 ****************************************************************************/

static int nrf52_adc_calibrate(FAR struct nrf52_adc_s *priv)
{
  /* Start calibration */

  nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_CALOFFSET_OFFSET, 1);

  /* Wait for calibration done */

  while (nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_CALDONE_OFFSET) != 1);

  return OK;
}

/****************************************************************************
 * Name: nrf52_adc_ch_config
 ****************************************************************************/

static uint32_t nrf52_adc_ch_config(FAR struct nrf52_adc_channel_s *cfg)
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

static int nrf52_adc_chancfg(FAR struct nrf52_adc_s *priv, uint8_t chan,
                             FAR struct nrf52_adc_channel_s *cfg)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(priv);

  /* Configure positive input */

  regval = nrf52_adc_chanpsel(cfg->p_psel);
  nrf52_adc_putreg(priv, NRF52_SAADC_CHPSELP_OFFSET(chan), regval);

  /* Configure negative input */

  regval = nrf52_adc_chanpsel(cfg->p_psel);
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

static int nrf52_adc_bind(FAR struct adc_dev_s *dev,
                          FAR const struct adc_callback_s *callback)
{
  FAR struct nrf52_adc_s *priv = (FAR struct nrf52_adc_s *) dev->ad_priv;

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

static void nrf52_adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct nrf52_adc_s *priv = (FAR struct nrf52_adc_s *) dev->ad_priv;

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

static int nrf52_adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct nrf52_adc_s *priv = (FAR struct nrf52_adc_s *) dev->ad_priv;
  int                     i    = 0;
  int                     ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  /* Enable ADC */

  nrf52_adc_putreg(priv, NRF52_SAADC_ENABLE_OFFSET, 1);

  /* Calibrate ADC */

  ret = nrf52_adc_calibrate(priv);
  if (ret < 0)
    {
      aerr("ERROR: adc calibration failed: %d\n", ret);
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

  /* Confgiure ADC */

  ret = nrf52_adc_configure(priv);
  if (ret < 0)
    {
      aerr("ERROR: nrf52_adc_configure failed: %d\n", ret);
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

static void nrf52_adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct nrf52_adc_s *priv = (FAR struct nrf52_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  /* Stop SAADC */

  nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_STOP_OFFSET, 0);

  /* Wait for SAADC stopped */

  while (nrf52_adc_getreg(priv, NRF52_SAADC_EVENTS_STOPPED_OFFSET) != 1);

  /* Disable SAADC */

  nrf52_adc_putreg(priv, NRF52_SAADC_ENABLE_OFFSET, 0);
}

/****************************************************************************
 * Name: nrf52_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 ****************************************************************************/

static void nrf52_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct nrf52_adc_s *priv   = (FAR struct nrf52_adc_s *) dev->ad_priv;
  uint32_t                regval = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  ainfo("RXINT enable: %d\n", enable ? 1 : 0);

  regval = SAADC_INT_END;

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

static int nrf52_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                           unsigned long arg)
{
  FAR struct nrf52_adc_s *priv = (FAR struct nrf52_adc_s *) dev->ad_priv;
  int ret                      = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start ADC */

          nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_START_OFFSET, 1);

          /* Trigger first sample */

          nrf52_adc_putreg(priv, NRF52_SAADC_TASKS_SAMPLE_OFFSET, 1);

          break;
        }

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
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

struct adc_dev_s *nrf52_adcinitialize(FAR struct nrf52_adc_channel_s *chan,
                                      int channels)
{
  FAR struct adc_dev_s   *dev  = NULL;
  FAR struct nrf52_adc_s *priv = NULL;
  int                     i    = 0;

  DEBUGASSERT(chan != NULL);
  DEBUGASSERT(channels <= CONFIG_NRF52_SAADC_CHANNELS);

#ifdef CONFIG_NRF52_SAADC_TIMER
  if (channels > 1)
    {
      aerr("ERORR: timer trigger works only for 1 channel!\n");
      set_errno(-EINVAL);
      goto errout;
    }
#endif

  /* Get device */

  dev = &g_nrf52_adc;

  /* Get private data */

  priv = (FAR struct nrf52_adc_s *) dev->ad_priv;

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
