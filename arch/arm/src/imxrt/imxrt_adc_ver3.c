/****************************************************************************
 * arch/arm/src/imxrt/imxrt_adc_ver3.c
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

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/imxrt_adc.h"
#include "imxrt_periphclks.h"

#ifdef CONFIG_IMXRT_ADC
#if defined(CONFIG_IMXRT_ADC1) || defined(CONFIG_IMXRT_ADC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ADC_MAX_CHANNELS 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_adc_ver3_s
{
  const struct adc_callback_s *cb;
  uint8_t intf;
  uint32_t base;
  int irq;
  uint8_t initialized;
  uint8_t nchannels;
  uint8_t chanlist[ADC_MAX_CHANNELS];
  uint8_t current;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);
static int adc_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind = adc_bind,
  .ao_reset = adc_reset,
  .ao_setup = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint = adc_rxint,
  .ao_ioctl = adc_ioctl,
};

#ifdef CONFIG_IMXRT_ADC1
static struct imxrt_adc_ver3_s g_adcpriv1 =
{
  .intf = 1,
  .base = IMXRT_ADC1_BASE,
  .irq  = IMXRT_IRQ_ADC1
};
static struct adc_dev_s g_adcdev1 =
{
  &g_adcops,
  &g_adcpriv1
};
#endif

#ifdef CONFIG_IMXRT_ADC2
static struct imxrt_adc_ver3_s g_adcpriv2 =
{
  .intf = 2,
  .base = IMXRT_ADC2_BASE,
  .irq  = IMXRT_IRQ_ADC2
};
static struct adc_dev_s g_adcdev2 =
{
  &g_adcops,
  &g_adcpriv2
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t adc_getreg(struct imxrt_adc_ver3_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void adc_putreg(struct imxrt_adc_ver3_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
  ((struct imxrt_adc_ver3_s *)dev->ad_priv)->cb = callback;
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

static void adc_reset(struct adc_dev_s *dev)
{
  struct imxrt_adc_ver3_s *priv = dev->ad_priv;

  if (priv->initialized != 0)
    {
      return;
    }

  if (priv->intf == 1)
    {
      imxrt_clockall_adc1();
    }
  else
    {
      imxrt_clockall_adc2();
    }

  adc_putreg(priv, IMXRT_ADC_CTRL_OFFSET, IMXRT_ADC_CTRL_RST);
  adc_putreg(priv, IMXRT_ADC_CTRL_OFFSET, 0);
  adc_putreg(priv, IMXRT_ADC_CFG_OFFSET, IMXRT_ADC_CFG_PWREN |
             (0x10u << IMXRT_ADC_CFG_PUDLY_SHIFT));
  adc_putreg(priv, IMXRT_ADC_FCTRL0_OFFSET, IMXRT_ADC_FCTRL_FWMARK(0));
  adc_putreg(priv, IMXRT_ADC_IE_OFFSET, 0);

  /* Keep calibration out of the reset path until the gain-result sequence
   * (GCC/GCR) is implemented.  The SDK examples initialize this way too.
   */

  adc_putreg(priv, IMXRT_ADC_CTRL_OFFSET, IMXRT_ADC_CTRL_ADCEN);
}

static void adc_program(struct imxrt_adc_ver3_s *priv, uint8_t channel)
{
  uint8_t hardware_channel = channel;
  uint32_t channel_type = IMXRT_ADC_CMDL_CTYPE_A;

  /* IDs 0-7 are A0-A7; IDs 8-15 are B0-B7. */

  if (channel >= 8)
    {
      hardware_channel = channel - 8;
      channel_type = IMXRT_ADC_CMDL_CTYPE_B;
    }

  uint32_t cmdl = IMXRT_ADC_CMDL_ADCH(hardware_channel) |
                  IMXRT_ADC_CMDL_CTYPE(channel_type) |
                  IMXRT_ADC_CMDL_CSCALE(1);

  adc_putreg(priv, IMXRT_ADC_CMDL1_OFFSET, cmdl);
  adc_putreg(priv, IMXRT_ADC_CMDH1_OFFSET, 0);
  adc_putreg(priv, IMXRT_ADC_TCTRL0_OFFSET, IMXRT_ADC_TCTRL_TCMD(1));
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

static int adc_setup(struct adc_dev_s *dev)
{
  struct imxrt_adc_ver3_s *priv = dev->ad_priv;
  int ret;

  if (priv->initialized++ != 0)
    {
      return OK;
    }

  priv->current = 0;
  adc_program(priv, priv->chanlist[0]);
  ret = irq_attach(priv->irq, adc_interrupt, dev);
  if (ret < 0)
    {
      priv->initialized--;
      return ret;
    }

  up_enable_irq(priv->irq);
  adc_putreg(priv, IMXRT_ADC_SWTRIG_OFFSET, 1);
  return OK;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct imxrt_adc_ver3_s *priv = dev->ad_priv;

  if (priv->initialized == 0 || --priv->initialized != 0)
    {
      return;
    }

  adc_putreg(priv, IMXRT_ADC_IE_OFFSET, 0);
  adc_putreg(priv, IMXRT_ADC_SWTRIG_OFFSET, 0);
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct imxrt_adc_ver3_s *priv = dev->ad_priv;
  adc_putreg(priv, IMXRT_ADC_IE_OFFSET, enable ? IMXRT_ADC_IE_FWMIE0 : 0);
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

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct imxrt_adc_ver3_s *priv = dev->ad_priv;
  return cmd == ANIOC_GET_NCHANNELS ? priv->nchannels : -ENOTTY;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev = arg;
  struct imxrt_adc_ver3_s *priv = dev->ad_priv;

  while ((adc_getreg(priv, IMXRT_ADC_FCTRL0_OFFSET) &
          IMXRT_ADC_FCTRL_FCOUNT_MASK) != 0)
    {
      uint32_t result = adc_getreg(priv, IMXRT_ADC_RESFIFO0_OFFSET);
      if ((result & IMXRT_ADC_RESFIFO_VALID) == 0)
        {
          break;
        }

      if (priv->cb != NULL && priv->cb->au_receive != NULL)
        {
          priv->cb->au_receive(dev, priv->chanlist[priv->current],
                               result & IMXRT_ADC_RESFIFO_DATA_MASK);
        }

      if (++priv->current >= priv->nchannels)
        {
          priv->current = 0;
        }

      adc_program(priv, priv->chanlist[priv->current]);
      adc_putreg(priv, IMXRT_ADC_SWTRIG_OFFSET, 1);
    }

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

struct adc_dev_s *imxrt_adcinitialize(int intf, const uint8_t *chanlist,
                                      int nchannels)
{
  struct adc_dev_s *dev;
  struct imxrt_adc_ver3_s *priv;

  if (nchannels <= 0 || nchannels > ADC_MAX_CHANNELS || chanlist == NULL)
    {
      return NULL;
    }

  for (int i = 0; i < nchannels; i++)
    {
      if (chanlist[i] >= ADC_MAX_CHANNELS)
        {
          return NULL;
        }
    }

  switch (intf)
    {
#ifdef CONFIG_IMXRT_ADC1
      case 1:
        dev = &g_adcdev1;
        break;
#endif
#ifdef CONFIG_IMXRT_ADC2
      case 2:
        dev = &g_adcdev2;
        break;
#endif
      default:
        return NULL;
    }

  priv = dev->ad_priv;
  memcpy(priv->chanlist, chanlist, nchannels);
  priv->nchannels = nchannels;
  return dev;
}

#endif
#endif
