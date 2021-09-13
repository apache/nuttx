/****************************************************************************
 * arch/arm/src/samv7/sam_adc.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/kmalloc.h>

#include "../../include/samv7/sam_adc.h"
//#include "sam_gpio.h"


#include "arm_arch.h"
#include "sam_afec.h"
#include "sam_periphclks.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC methods */

static int  sam_adc_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback);
static void sam_adc_reset(FAR struct adc_dev_s *dev);
static int  sam_adc_setup(FAR struct adc_dev_s *dev);
static void sam_adc_shutdown(FAR struct adc_dev_s *dev);
static void sam_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  sam_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_adc_priv
{
  int                    genclk;                /* clock generator */
  const struct adc_callback_s *adc_callback;    /* callback for upper driver */
  uint8_t                cur_channel;           /* current channel in progress */
  uint8_t                logic2physicalCHmap[BOARD_ADC_NUM_CHANNELS];   /* mapping of logical to physical channels */
  uint8_t                num_channels;       /* number of channels */
  uint8_t                ref;                   /* reference selection */
  uint32_t               neg;                   /* negative input selection */
  uint8_t                samplen;               /* sampling time length */
  uint32_t               prescaler;             /* prescaler configuration */
  uint8_t                averaging;             /* number of samples to be collected */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s sam_adc_ops =
{
  sam_adc_bind,
  sam_adc_reset,
  sam_adc_setup,
  sam_adc_shutdown,
  sam_adc_rxint,
  sam_adc_ioctl
};

static struct adc_dev_s g_sam_adc_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/


static int sam_adc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t result;
  uint32_t regval;
  struct adc_dev_s    *dev = (struct adc_dev_s *)arg;
  struct sam_adc_priv *priv = (struct sam_adc_priv *)dev->ad_priv;

  regval = 0;
  for (priv->cur_channel=0;priv->cur_channel<priv->num_channels; priv->cur_channel++)
    {
      regval = AFEC_CSELR_CSEL(priv->logic2physicalCHmap[priv->cur_channel]);
      putreg32(regval,SAM_AFEC1_CSELR);

      result = getreg32(SAM_AFEC1_CDR) & AFEC_CDR_MASK;

      ainfo("ADC CH %d Result = %d:\n", priv->cur_channel, result);

      priv->adc_callback->au_receive(dev, priv->cur_channel,
                                     result);
    }
  result = getreg32(SAM_AFEC1_LCDR); /* read this register to clear drdy interrupt bit */

  return 0;
}

static int sam_adc_init_clock(struct adc_dev_s *dev)
{
  uint16_t regval;
  struct sam_adc_priv *priv = (struct sam_adc_priv *)dev->ad_priv;

  sam_afec0_enableclk();
  sam_afec1_enableclk();

  return 0;
}

static int sam_adc_calibrate(struct adc_dev_s *dev)
{
  uint8_t linearity;
  uint8_t bias;
  uint16_t regval;


  return 0;
}

/* Bind the upper-half driver callbacks to the lower-half implementation.
 * This must be called early in order to receive ADC event notifications.
 */

static int sam_adc_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  struct sam_adc_priv *priv = (struct sam_adc_priv *)dev->ad_priv;
  priv->adc_callback = callback;
  return 0;
}

/****************************************************************************
 * Name: sam_adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before sam_adc_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev   - A reference to the ADC device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_adc_reset(FAR struct adc_dev_s *dev)
{
  /* Reset ADC simulating a hardware reset*/
  //putreg32(AFEC_CR_SWRST, SAM_AFEC0_CR);
  //putreg32(AFEC_CR_SWRST, SAM_AFEC1_CR);
}

/****************************************************************************
 * Name: sam_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts. Interrupts
 *   are all disabled upon return.
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before sam_adc_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev   - A reference to the ADC device structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_adc_setup(FAR struct adc_dev_s *dev)
{
  uint8_t regval;
  struct sam_adc_priv *priv = (struct sam_adc_priv *)dev->ad_priv;

  priv->cur_channel = 0;

  /* Enable ADC */
  putreg32(AFEC_CR_START, SAM_AFEC0_CR);
  putreg32(AFEC_CR_START, SAM_AFEC1_CR);

  return 0;
}

/****************************************************************************
 * Name: sam_adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev   - A reference to the ADC device structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sam_adc_shutdown(FAR struct adc_dev_s *dev)
{
  /* Disable ADC */
  putreg32(0, SAM_AFEC0_CR);
  putreg32(0, SAM_AFEC1_CR);
}

/****************************************************************************
 * Name: sam_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 * Input Parameters:
 *   dev   - A reference to the ADC device structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sam_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  struct sam_adc_priv *priv = (struct sam_adc_priv *)dev->ad_priv;

  if (enable)
    {
      priv->cur_channel = 0;
      putreg32(AFEC_INT_DRDY, SAM_AFEC0_IER);
      putreg32(AFEC_INT_DRDY, SAM_AFEC1_IER);
    }
  else
    {
	  putreg32(0, SAM_AFEC0_IER);
	  putreg32(0, SAM_AFEC1_IER);
    }
}

/****************************************************************************
 * Name: sam_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev   - A reference to the ADC device structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_adc_ioctl(FAR struct adc_dev_s *dev,
                         int cmd, unsigned long arg)
{
  int ret = 0;
  struct sam_adc_priv    *priv = (struct sam_adc_priv *)dev->ad_priv;
  struct sam_adc_param_s *params = (struct sam_adc_param_s *)arg;

  switch (cmd)
    {
      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->num_channels;
        }
        break;
      case ANIOC_TRIGGER:
		  {
			  /* to be implemented */
			  break;
		  }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adcinitialize
 *
 * Description:
 *   Register SAMD2x/L2x ADC driver
 *
 * Input Parameters:
 *   genclk - The Clock Generator to be used on ADC block.
 *
 * Returned Value:
 *   A reference to the ADC device structure if succeed or NULL otherwise.
 *
 ****************************************************************************/

struct adc_dev_s *sam_adcinitialize(int genclk)
{
  irqstate_t flags;
  uint32_t regval;
  uint8_t iii = 0;
  int ret;

  struct sam_adc_priv *priv = (struct sam_adc_priv *)
                              kmm_malloc(sizeof(struct sam_adc_priv));
  priv->genclk = genclk;

  g_sam_adc_dev.ad_priv = priv;
  g_sam_adc_dev.ad_ops = &sam_adc_ops;

  priv->num_channels = BOARD_ADC_NUM_CHANNELS;
  priv->logic2physicalCHmap[0] = 7;
  priv->logic2physicalCHmap[1] = 8;
  priv->logic2physicalCHmap[2] = 4;
  priv->logic2physicalCHmap[3] = 5;
  priv->logic2physicalCHmap[4] = 6;

  sam_adc_init_clock(&g_sam_adc_dev);

  /* disable writeproteciton of afec registers */
  putreg32(AFEC_WPMR_WPKEY, SAM_AFEC1_WPMR);

  regval = AFEC_MR_TRANSFER(2)|  \
           AFEC_MR_TRACKTIM(15)| \
           AFEC_MR_ONE|          \
           AFEC_MR_FREERUN|      \
           AFEC_MR_STARTUP_0|    \
           AFEC_MR_PRESCAL(0xFF);
  putreg32(regval,SAM_AFEC1_MR);

  putreg32(AFEC_EMR_TAG|AFEC_EMR_RES_NOAVG,SAM_AFEC1_EMR);

  regval = 0;
  for (iii=0;iii<priv->num_channels; iii++)
    {
      regval |= AFEC_CH(priv->logic2physicalCHmap[iii]);
    }

  putreg32(regval,SAM_AFEC1_CHER);


  /* Set all channels CM voltage to Vrefp/2 (512). */
  for (iii=0;iii<priv->num_channels; iii++)
    {
      regval = AFEC_CSELR_CSEL(priv->logic2physicalCHmap[iii]);
      putreg32(regval, SAM_AFEC1_CSELR);

      putreg32(512, SAM_AFEC1_COCR);
    }

  putreg32(AFEC_ACR_PGA0EN|AFEC_ACR_PGA1EN|AFEC_ACR_IBCTL(1), SAM_AFEC1_ACR);

  flags = enter_critical_section();

  /* Attach Interrupt Handler */

  ret = irq_attach(SAM_IRQ_AFEC1, sam_adc_interrupt, &g_sam_adc_dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach irq %d\n", SAM_IRQ_AFEC1);
      leave_critical_section(flags);
      return NULL;
    }

  up_enable_irq(SAM_IRQ_AFEC1);

  leave_critical_section(flags);

  /* enable writeproteciton of afec registers */
  putreg32(AFEC_WPMR_WPKEY | AFEC_WPMR_WPEN, SAM_AFEC1_WPMR);


  return (struct adc_dev_s *)&g_sam_adc_dev;
}
