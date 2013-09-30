/************************************************************************************
 * arch/arm/src/sama5/sam_adc.c
 *
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "chip/sam_adc.h"
#include "sam_adc.h"

#if defined(CONFIG_SAMA5_ADC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the overall state of the ADC */

struct sam_adc_s
{
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC interrupt handling */

static int  adc_interrupt(int irq, void *context);

/* ADC methods */

static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC lower half device operations */

static const struct adc_ops_s g_adcops =
{
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

/* ADC internal state */

static struct sam_adc_s g_adcpriv =
{
};

/* ADC device instance */

static struct adc_dev_s g_adcdev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  FAR struct sam_adc_s *priv = (FAR struct sam_adc_s *)dev->ad_priv;
  irqstate_t flags;
  uint32_t regval;

  flags = irqsave();
#warning Missing logic

  irqrestore(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.  Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct sam_adc_s *priv = (FAR struct sam_adc_s *)dev->ad_priv;
  int ret;

  /* Attach the ADC interrupt */

  ret = irq_attach(SAM_IRQ_ADC, adc_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", SAM_IRQ_ADC, ret);
      return ret;
    }

  /* Enable the ADC interrupt */

  up_enable_irq(SAM_IRQ_ADC);
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

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct sam_adc_s *priv = (FAR struct sam_adc_s *)dev->ad_priv;

  /* Disable ADC interrupts, both at the level of the ADC device and at the
   * level of the NVIC.
   */
#warning Missing logic

  up_disable_irq(SAM_IRQ_ADC);

  /* Then detach the ADC interrupt handler. */

  irq_detach(SAM_IRQ_ADC);
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
  FAR struct sam_adc_s *priv = (FAR struct sam_adc_s *)dev->ad_priv;

  /* Are we enabling or disabling? */
  if (enable)
    {
#warning Missing logic
    }
  else
    {
#warning Missing logic
    }
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *  All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  /* No ioctl commands supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context)
{
  FAR struct sam_adc_s *priv = (FAR struct sam_adc_s *)g_adcdev.ad_priv;
  uint32_t regval;

#warning Missing logic

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *sam_adcinitialize(void)
{
  return &g_adcdev;
}

#endif /* CONFIG_SAMA5_ADC */
