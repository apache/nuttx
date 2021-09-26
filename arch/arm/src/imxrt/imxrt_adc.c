/****************************************************************************
 * arch/arm/src/imxrt/imxrt_adc.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "hardware/imxrt_adc.h"
#include "hardware/imxrt_pinmux.h"
#include "imxrt_gpio.h"
#include "imxrt_periphclks.h"

#ifdef CONFIG_IMXRT_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_IMXRT_ADC1) || defined(CONFIG_IMXRT_ADC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_MAX_CHANNELS 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_dev_s
{
  FAR const struct adc_callback_s *cb;  /* Upper driver callback */
  uint8_t  intf;                        /* ADC number (i.e. ADC1, ADC2) */
  uint32_t base;                        /* ADC register base */
  uint8_t  initialized;                 /* ADC initialization counter */
  int      irq;                         /* ADC IRQ number */
  int      nchannels;                   /* Number of configured ADC channels */
  uint8_t  chanlist[ADC_MAX_CHANNELS];  /* ADC channel list */
  uint8_t  current;                     /* Current channel being converted */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void adc_putreg(FAR struct imxrt_dev_s *priv, uint32_t offset,
                       uint32_t value);
static uint32_t adc_getreg(FAR struct imxrt_dev_s *priv, uint32_t offset);
static void adc_modifyreg(FAR struct imxrt_dev_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits);

/* ADC methods */

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);
static int  adc_interrupt(int irq, void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = adc_bind,
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

#ifdef CONFIG_IMXRT_ADC1
static struct imxrt_dev_s g_adcpriv1 =
{
  .irq         = IMXRT_IRQ_ADC1,
  .intf        = 1,
  .initialized = 0,
  .base        = IMXRT_ADC1_BASE,
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv1,
};

gpio_pinset_t g_adcpinlist1[ADC_MAX_CHANNELS] =
{
    GPIO_ADC1_CH0,
    GPIO_ADC1_CH1,
    GPIO_ADC1_CH2,
    GPIO_ADC1_CH3,
    GPIO_ADC1_CH4,
    GPIO_ADC1_CH5,
    GPIO_ADC1_CH6,
    GPIO_ADC1_CH7,
    GPIO_ADC1_CH8,
    GPIO_ADC1_CH9,
    GPIO_ADC1_CH10,
    GPIO_ADC1_CH11,
    GPIO_ADC1_CH12,
    GPIO_ADC1_CH13,
    GPIO_ADC1_CH14,
    GPIO_ADC1_CH15,
};
#endif

#ifdef CONFIG_IMXRT_ADC2
static struct imxrt_dev_s g_adcpriv2 =
{
  .irq         = IMXRT_IRQ_ADC2,
  .intf        = 2,
  .initialized = 0,
  .base        = IMXRT_ADC2_BASE,
};

static struct adc_dev_s g_adcdev2 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv2,
};

gpio_pinset_t g_adcpinlist2[ADC_MAX_CHANNELS] =
{
    GPIO_ADC2_CH0,
    GPIO_ADC2_CH1,
    GPIO_ADC2_CH2,
    GPIO_ADC2_CH3,
    GPIO_ADC2_CH4,
    GPIO_ADC2_CH5,
    GPIO_ADC2_CH6,
    GPIO_ADC2_CH7,
    GPIO_ADC2_CH8,
    GPIO_ADC2_CH9,
    GPIO_ADC2_CH10,
    GPIO_ADC2_CH11,
    GPIO_ADC2_CH12,
    GPIO_ADC2_CH13,
    GPIO_ADC2_CH14,
    GPIO_ADC2_CH15,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void adc_putreg(FAR struct imxrt_dev_s *priv, uint32_t offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static uint32_t adc_getreg(FAR struct imxrt_dev_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void adc_modifyreg(FAR struct imxrt_dev_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct imxrt_dev_s *priv = (FAR struct imxrt_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct imxrt_dev_s *priv = (FAR struct imxrt_dev_s *)dev->ad_priv;
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
#ifdef CONFIG_IMXRT_ADC1
      case 1:
        imxrt_clockall_adc1();
        break;
#endif
#ifdef CONFIG_IMXRT_ADC2
      case 2:
        imxrt_clockall_adc2();
        break;
#endif
      default:
        aerr("ERROR: Tried to reset non-existing ADC: %d\n", priv->intf);
        goto exit_leave_critical;
    }

  leave_critical_section(flags);

  /* Configure ADC */

  uint32_t adc_cfg = ADC_CFG_AVGS_4SMPL | ADC_CFG_ADTRG_SW |
      ADC_CFG_REFSEL_VREF | ADC_CFG_ADSTS_7_21 | ADC_CFG_ADIV_DIV8 | \
      ADC_CFG_ADLSMP | ADC_CFG_MODE_10BIT | ADC_CFG_ADICLK_IPGDIV2;
  adc_putreg(priv, IMXRT_ADC_CFG_OFFSET, adc_cfg);

  uint32_t adc_gc = 0;
  adc_putreg(priv, IMXRT_ADC_GC_OFFSET, adc_gc);

  /* Calibration - After ADC has been configured as desired.
   * ADTRG in ADC_CFG must be SW (0) during calibration
   */

  /* Clear calibration error */

  adc_modifyreg(priv, IMXRT_ADC_GS_OFFSET, 0, ADC_GS_CALF);

  /* Start calibration */

  adc_modifyreg(priv, IMXRT_ADC_GC_OFFSET, 0, ADC_GC_CAL);

  while ((adc_getreg(priv, IMXRT_ADC_GC_OFFSET) & ADC_GC_CAL) != 0 &&
      (adc_getreg(priv, IMXRT_ADC_GS_OFFSET) & ADC_GS_CALF) == 0);

  if ((adc_getreg(priv, IMXRT_ADC_GS_OFFSET) & ADC_GS_CALF) != 0 ||
      (adc_getreg(priv, IMXRT_ADC_HS_OFFSET) & ADC_HS_COCO0) == 0)
    {
      aerr("ERROR: ADC%d calibration failed\n", priv->intf);
      return;
    }

  /* Clear "conversion complete" */

  uint32_t adc_r0 = adc_getreg(priv, IMXRT_ADC_R0_OFFSET);
  UNUSED(adc_r0);

  /* Pad configuration */

  gpio_pinset_t *pinlist = NULL;
  switch (priv->intf)
    {
#ifdef CONFIG_IMXRT_ADC1
      case 1:
        pinlist = g_adcpinlist1;
        break;
#endif
#ifdef CONFIG_IMXRT_ADC2
      case 2:
        pinlist = g_adcpinlist2;
        break;
#endif
      default:
        /* We have already checked the intf number earlier in this function,
         * so we should never get here.
         */

        return;
    }

  gpio_pinset_t pinset = 0;
  for (int i = 0; i < priv->nchannels; i++)
    {
      DEBUGASSERT(priv->chanlist[i] < ADC_MAX_CHANNELS);
      pinset = pinlist[priv->chanlist[i]] | IOMUX_ADC_DEFAULT;
      imxrt_config_gpio(pinset);
    }

  return;

exit_leave_critical:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct imxrt_dev_s *priv = (FAR struct imxrt_dev_s *)dev->ad_priv;

  /* Do nothing when the ADC device is already set up */

  if (priv->initialized > 0)
    {
      return OK;
    }

  priv->initialized++;

  int ret = irq_attach(priv->irq, adc_interrupt, dev);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  up_enable_irq(priv->irq);

  /* Start the first conversion */

  priv->current = 0;
  adc_putreg(priv, IMXRT_ADC_HC0_OFFSET,
             ADC_HC_ADCH(priv->chanlist[priv->current]));

  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct imxrt_dev_s *priv = (FAR struct imxrt_dev_s *)dev->ad_priv;

  /* Shutdown the ADC device only when not in use */

  priv->initialized--;

  if (priv->initialized > 0)
    {
      return;
    }

  /* Disable ADC interrupts, both at the level of the ADC device and at the
   * level of the NVIC.
   */

  /* Disable interrupt and stop any on-going conversion */

  adc_putreg(priv, IMXRT_ADC_HC0_OFFSET, ~ADC_HC_AIEN | ADC_HC_ADCH_DIS);

  up_disable_irq(priv->irq);

  /* Then detach the ADC interrupt handler. */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct imxrt_dev_s *priv = (FAR struct imxrt_dev_s *)dev->ad_priv;

  if (enable)
    {
      adc_modifyreg(priv, IMXRT_ADC_HC0_OFFSET, 0, ADC_HC_AIEN);
    }
  else
    {
      adc_modifyreg(priv, IMXRT_ADC_HC0_OFFSET, ADC_HC_AIEN, 0);
    }
}

/****************************************************************************
 * Name: adc_ioctl
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
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  /* TODO: ANIOC_TRIGGER, for SW triggered conversion */

  FAR struct imxrt_dev_s *priv = (FAR struct imxrt_dev_s *)dev->ad_priv;
  int ret = -ENOTTY;

  switch (cmd)
    {
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
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct imxrt_dev_s *priv = (FAR struct imxrt_dev_s *)dev->ad_priv;
  int32_t data;

  if ((adc_getreg(priv, IMXRT_ADC_HS_OFFSET) & ADC_HS_COCO0) != 0)
    {
      /* Read data. This also clears the COCO bit. */

      data = (int32_t)adc_getreg(priv, IMXRT_ADC_R0_OFFSET);

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

      adc_modifyreg(priv, IMXRT_ADC_HC0_OFFSET, ADC_HC_ADCH_MASK,
                    ADC_HC_ADCH(priv->chanlist[priv->current]));
    }

  /* There are no interrupt flags left to clear */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Input Parameters:
 *   intf      - ADC number (1 or 2)
 *   chanlist  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *imxrt_adcinitialize(int intf,
                                          FAR const uint8_t *chanlist,
                                          int nchannels)
{
  FAR struct adc_dev_s *dev;
  FAR struct imxrt_dev_s *priv;

  DEBUGASSERT(nchannels > 0);

  switch (intf)
    {
#ifdef CONFIG_IMXRT_ADC1
      case 1:
        {
          dev = &g_adcdev1;
          break;
        }
#endif /* CONFIG_IMXRT_ADC1 */

#ifdef CONFIG_IMXRT_ADC2
      case 2:
        {
          dev = &g_adcdev2;
          break;
        }
#endif /* CONFIG_IMXRT_ADC2 */

      default:
        {
          aerr("ERROR: Tried to initialize invalid ADC: %d\n", intf);
          return NULL;
        }
    }

  priv = (FAR struct imxrt_dev_s *)dev->ad_priv;

  priv->nchannels = nchannels;
  memcpy(priv->chanlist, chanlist, nchannels);

  ainfo("intf: %d nchannels: %d\n", priv->intf, priv->nchannels);

  return dev;
}

#endif /* CONFIG_IMXRT_ADC1 || CONFIG_IMXRT_ADC2 */

#endif /* CONFIG_IMXRT_ADC */
