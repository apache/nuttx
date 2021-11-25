/****************************************************************************
 * arch/arm/src/samv7/sam_afec.c
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
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "hardware/sam_matrix.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_pio.h"
#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "sam_afec.h"

#ifdef CONFIG_ADC

#if defined(CONFIG_SAMV7_AFEC0) || defined(CONFIG_SAMV7_AFEC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_MAX_CHANNELS 11

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samv7_dev_s
{
  FAR const struct adc_callback_s *cb;  /* Upper driver callback */
  uint8_t  intf;                        /* ADC number (i.e. ADC1, ADC2) */
  uint32_t base;                        /* ADC register base */
  uint8_t  initialized;                 /* ADC initialization counter */
  uint8_t  resolution;                  /* ADC resolution (SAMV7_AFECn_RES) */
  int      irq;                         /* ADC IRQ number */
  int      nchannels;                   /* Number of configured channels */
  uint8_t  chanlist[ADC_MAX_CHANNELS];  /* ADC channel list */
  uint8_t  current;                     /* Current channel being converted */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void afec_putreg(FAR struct samv7_dev_s *priv, uint32_t offset,
                       uint32_t value);
static uint32_t afec_getreg(FAR struct samv7_dev_s *priv, uint32_t offset);

/* ADC methods */

static int  afec_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void afec_reset(FAR struct adc_dev_s *dev);
static int  afec_setup(FAR struct adc_dev_s *dev);
static void afec_shutdown(FAR struct adc_dev_s *dev);
static void afec_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  afec_ioctl(FAR struct adc_dev_s *dev,
                       int cmd, unsigned long arg);
static int  afec_interrupt(int irq, void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = afec_bind,
  .ao_reset    = afec_reset,
  .ao_setup    = afec_setup,
  .ao_shutdown = afec_shutdown,
  .ao_rxint    = afec_rxint,
  .ao_ioctl    = afec_ioctl,
};

#ifdef CONFIG_SAMV7_AFEC0
static struct samv7_dev_s g_adcpriv0 =
{
  .irq         = SAM_IRQ_AFEC0,
  .intf        = 0,
  .initialized = 0,
  .resolution  = CONFIG_SAMV7_AFEC0_RES,
  .base        = SAM_AFEC0_BASE,
};

static struct adc_dev_s g_adcdev0 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv0,
};

gpio_pinset_t g_adcpinlist0[ADC_MAX_CHANNELS] =
{
    GPIO_AFE0_AD0,
    GPIO_AFE0_AD1,
    GPIO_AFE0_AD2,
    GPIO_AFE0_AD3,
    GPIO_AFE0_AD4,
    GPIO_AFE0_AD5,
    GPIO_AFE0_AD6,
    GPIO_AFE0_AD7,
    GPIO_AFE0_AD8,
    GPIO_AFE0_AD0,
    GPIO_AFE0_AD10,
};
#endif

#ifdef CONFIG_SAMV7_AFEC1
static struct samv7_dev_s g_adcpriv1 =
{
  .irq         = SAM_IRQ_AFEC1,
  .intf        = 1,
  .initialized = 0,
  .resolution  = CONFIG_SAMV7_AFEC1_RES,
  .base        = SAM_AFEC1_BASE,
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv0,
};

gpio_pinset_t g_adcpinlist1[ADC_MAX_CHANNELS] =
{
    GPIO_AFE1_AD0,
    GPIO_AFE1_AD1,
    GPIO_AFE1_AD2,
    GPIO_AFE1_AD3,
    GPIO_AFE1_AD4,
    GPIO_AFE1_AD5,
    GPIO_AFE1_AD6,
    GPIO_AFE1_AD7,
    GPIO_AFE1_AD8,
    GPIO_AFE1_AD0,
    GPIO_AFE1_AD10,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void afec_putreg(FAR struct samv7_dev_s *priv, uint32_t offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static uint32_t afec_getreg(FAR struct samv7_dev_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: afec_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int afec_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: afec_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before afec_setup() and on error conditions.
 *
 ****************************************************************************/

static void afec_reset(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Do nothing if ADC instance is currently in use */

  if (priv->initialized > 0)
    {
      goto exit_leave_critical;
    }

  /* Configure clock gating */

  switch (priv->intf)
    {
#ifdef CONFIG_SAMV7_AFEC0
      case 0:
        sam_afec0_enableclk();
        break;
#endif
#ifdef CONFIG_SAMV7_AFEC1
      case 1:
        sam_afec1_enableclk();
        break;
#endif
      default:
        aerr("ERROR: Tried to reset non-existing ADC: %d\n", priv->intf);
        goto exit_leave_critical;
    }

  leave_critical_section(flags);

  /* Software reset */

  afec_putreg(priv, SAM_AFEC_CR_OFFSET, AFEC_CR_SWRST);

  /* Configure Mode Register */

  uint32_t afec_mr = AFEC_MR_STARTUP_64 | AFEC_MR_PRESCAL(2) | AFEC_MR_ONE;
  afec_putreg(priv, SAM_AFEC_MR_OFFSET, afec_mr);

  /* Configure Extended Mode register */

  uint32_t afec_emr = AFEC_EMR_TAG | AFEC_EMR_STM | \
                      AFEC_EMR_RES(priv->resolution);
  afec_putreg(priv, SAM_AFEC_EMR_OFFSET, afec_emr);

  /* Configure Analog Control Register */

  uint32_t afec_acr = AFEC_ACR_PGA0EN | AFEC_ACR_PGA1EN | AFEC_ACR_IBCTL(2);
  afec_putreg(priv, SAM_AFEC_ACR_OFFSET, afec_acr);

  /* Pad configuration */

  gpio_pinset_t *pinlist = NULL;
  switch (priv->intf)
    {
#ifdef CONFIG_SAMV7_AFEC0
      case 0:
        pinlist = g_adcpinlist0;
        break;
#endif
#ifdef CONFIG_SAMV7_AFEC1
      case 1:
        pinlist = g_adcpinlist1;
        break;
#endif
      default:
        /* We have already checked the intf number earlier in this function,
         * so we should never get here.
         */

        return;
    }

  /* Desible write protection (should already be disabled by default) */

  afec_putreg(priv, SAM_AFEC_WPMR_OFFSET, AFEC_WPMR_WPKEY);

  /* Disable all channels */

  afec_putreg(priv, SAM_AFEC_CHDR_OFFSET, AFEC_CHALL);

  gpio_pinset_t pinset = 0;
  uint32_t afec_cher = 0;
  for (int i = 0; i < priv->nchannels; i++)
    {
      DEBUGASSERT(priv->chanlist[i] < ADC_MAX_CHANNELS);
      pinset = pinlist[priv->chanlist[i]];
      sam_configgpio(pinset);

      afec_putreg(priv, SAM_AFEC_CSELR_OFFSET,
                  AFEC_CSELR_CSEL(priv->chanlist[i]));
      afec_putreg(priv, SAM_AFEC_COCR_OFFSET, 0x200);

      afec_cher |= AFEC_CH(priv->chanlist[i]);
    }

  /* Enable channels */

  afec_putreg(priv, SAM_AFEC_CHER_OFFSET, afec_cher);

  return;

exit_leave_critical:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: afec_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int afec_setup(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  /* Do nothing when the ADC device is already set up */

  if (priv->initialized > 0)
    {
      return OK;
    }

  priv->initialized++;

  int ret = irq_attach(priv->irq, afec_interrupt, dev);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  up_enable_irq(priv->irq);

  /* Start the first conversion */

  priv->current = 0;

  uint32_t afec_cselr = AFEC_CSELR_CSEL(priv->chanlist[priv->current]);
  afec_putreg(priv, SAM_AFEC_CSELR_OFFSET, afec_cselr);

  return ret;
}

/****************************************************************************
 * Name: afec_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void afec_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  uint32_t afec_ixr = 0;

  for (int i = 0; i < priv->nchannels; i++)
    {
      afec_ixr |= AFEC_INT_EOC(priv->chanlist[i]);
    }

  /* Enable interrupts */

  if (enable)
    {
      afec_putreg(priv, SAM_AFEC_IER_OFFSET, afec_ixr);
    }
  else
    {
      afec_putreg(priv, SAM_AFEC_IDR_OFFSET, afec_ixr);
    }
}

/****************************************************************************
 * Name: afec_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void afec_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  /* Shutdown the ADC device only when not in use */

  priv->initialized--;

  if (priv->initialized > 0)
    {
      return;
    }

  /* Disable ADC interrupts */

  up_disable_irq(priv->irq);

  /* Then detach the ADC interrupt handler. */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: afec_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *   OK in success or error value
 *
 ****************************************************************************/

static int afec_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          afec_putreg(priv, SAM_AFEC_CR_OFFSET, AFEC_CR_START);
        }
        break;
      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->nchannels;
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
 * Name: afec_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int afec_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct samv7_dev_s *priv = (FAR struct samv7_dev_s *)dev->ad_priv;
  int32_t data;

  if ((afec_getreg(priv, SAM_AFEC_ISR_OFFSET) & \
                   AFEC_CH(priv->chanlist[priv->current])) != 0)
    {
      /* Read data */

      data = (int32_t)afec_getreg(priv, SAM_AFEC_CDR_OFFSET);

      if (priv->cb != NULL)
        {
          DEBUGASSERT(priv->cb->au_receive != NULL);
          priv->cb->au_receive(dev, priv->chanlist[priv->current],  data);
        }

      /* Set the channel number of the next channel that will complete
       * conversion.
       */

      priv->current++;

      if (priv->current >= priv->nchannels)
        {
          /* Restart the conversion sequence from the beginning */

          priv->current = 0;
        }

      /* Start the next conversion */

      uint32_t afec_cselr = AFEC_CSELR_CSEL(priv->chanlist[priv->current]);
      afec_putreg(priv, SAM_AFEC_CSELR_OFFSET, afec_cselr);
    }

  /* There are no interrupt flags left to clear */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_afec_initialize
 *
 * Description:
 *   Initialize the adc
 *
 * Input Parameters:
 *   intf      - ADC number (0 or 1)
 *   chanlist  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *sam_afec_initialize(int intf,
                                          FAR const uint8_t *chanlist,
                                          int nchannels)
{
  FAR struct adc_dev_s *dev;
  FAR struct samv7_dev_s *priv;

  DEBUGASSERT(nchannels > 0);

  switch (intf)
    {
#ifdef CONFIG_SAMV7_AFEC0
      case 0:
        {
          dev = &g_adcdev0;
          break;
        }
#endif /* CONFIG_SAMV7_AFEC0 */

#ifdef CONFIG_SAMV7_AFEC1
      case 1:
        {
          dev = &g_adcdev1;
          break;
        }
#endif /* CONFIG_SAMV7_AFEC1 */

      default:
        {
          aerr("ERROR: Tried to initialize invalid ADC: %d\n", intf);
          return NULL;
        }
    }

  priv = (FAR struct samv7_dev_s *)dev->ad_priv;

  priv->nchannels = nchannels;
  memcpy(priv->chanlist, chanlist, nchannels);

  ainfo("intf: %d nchannels: %d\n", priv->intf, priv->nchannels);

  return dev;
}

#endif /* CONFIG_SAMV7_AFEC0 || CONFIG_SAMV7_AFEC1 */

#endif /* CONFIG_ADC */
