/*****************************************************************************
 * arch/arm/include/samd2l2/sam_adc.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Alexander Vasiliev<alexvasiljev@gmail.com>
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
#include <nuttx/kmalloc.h>

#include <arch/chip/sam_adc.h>

#include "arm_arch.h"
#include "sam_adc.h"
#include "sam_pinmap.h"
#include "sam_gclk.h"
#include "sam_port.h"
#include "sam_fuses.h"
#include "samd_periphclks.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#define ADC_INSERT_INPUT(n) \
  if (ch_pos < priv->num_channels) \
    { \
      sam_configport(PORT_AIN##n); \
      priv->channels[ch_pos++] = n; \
    }

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
  int                    genclk;       /* clock generator */
  const struct adc_callback_s *adc_callback; /* callback for upper driver */
  int                    cur_channel;  /* current channel in progress */
  int                    num_channels; /* number of channels */
  int                   *channels;     /* channels to process */
  uint8_t                ref;          /* reference selection */
  uint32_t               neg;          /* negative input selection */
  uint8_t                samplen;      /* sampling time length */
  uint32_t               prescaler;    /* prescaler configuration */
  uint8_t                averaging;    /* number of samples to be collected */
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

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

static void sam_adc_synchronization(void)
{
  while ((getreg8(SAM_ADC_STATUS) & ADC_STATUS_SYNCBUSY) != 0);
}

static int sam_adc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t result;
  struct adc_dev_s    *dev = (struct adc_dev_s *)arg;
  struct sam_adc_priv *priv = (struct sam_adc_priv *)dev->ad_priv;

  sam_adc_synchronization();
  result = getreg16(SAM_ADC_RESULT);

  ainfo("ADC Result = %d:\n", result);

  priv->adc_callback->au_receive(dev, priv->channels[priv->cur_channel], result);

  putreg8(ADC_INT_RESRDY, SAM_ADC_INTFLAG);

  /* If all channels were read, restart to the first channel */

  if (++priv->cur_channel == priv->num_channels)
    {
      priv->cur_channel = 0;
    }

  putreg32(priv->channels[priv->cur_channel] | priv->neg, SAM_ADC_INPUTCTRL);

  sam_adc_synchronization();

  putreg8(ADC_SWTRIG_START, SAM_ADC_SWTRIG);

  return 0;
}

static int sam_adc_init_clock(struct adc_dev_s *dev)
{
  uint16_t regval;
  struct sam_adc_priv *priv = (struct sam_adc_priv *)dev->ad_priv;

  regval = GCLK_CLKCTRL_ID_ADC;

  /* Select and disable the ADC_GCLK_ID_CORE generic clock */

  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Wait for clock to become disabled */

  while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) != 0);

  /* Select the ADC_GCLK_ID_CORE source clock generator */

  regval |= (uint16_t)priv->genclk << GCLK_CLKCTRL_GEN_SHIFT;

  /* Write the new configuration */

  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Enable the ADC_GCLK_ID_CORE generic clock */

  regval |= GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  while ((getreg16(SAM_GCLK_STATUS) & GCLK_STATUS_SYNCBUSY) != 0);

  return 0;
}

static int sam_adc_calibrate(struct adc_dev_s *dev)
{
  uint8_t linearity;
  uint8_t bias;
  uint16_t regval;

  bias = (getreg8(ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_MASK) >>
         ADC_FUSES_BIASCAL_SHIFT;
  linearity = getreg16(SAM_AUX1_AREA4 + 3) >> 3;

  regval = linearity | (bias << ADC_CALIB_BIAS_OFFSET);
  putreg16(regval, SAM_ADC_CALIB);

  return 0;
}

/* Bind the upper-half driver callbacks to the lower-half implementation.  This
 * must be called early in order to receive ADC event notifications.
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
  /* Disable ADC */

  putreg8(0, SAM_ADC_CTRLA);

  sam_adc_synchronization();

  /* Reset ADC */

  putreg8(ADC_CTRLA_SWRST, SAM_ADC_CTRLA);

  while ((getreg8(SAM_ADC_STATUS) & ADC_STATUS_SYNCBUSY) != 0 ||
         (getreg8(SAM_ADC_CTRLA) & ADC_CTRLA_SWRST) != 0);
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

  sam_adc_calibrate(dev);

  /* ADC continues normal operation during debug mode */

  regval = ADC_DBGCTRL_DBGRUN;
  putreg8(regval, SAM_ADC_ADC_DBGCTRL);

  regval = priv->ref | ADC_REFCTRL_REFCOMP;
  putreg8(regval, SAM_ADC_REFCTL);

  regval = priv->channels[0] | priv->neg;
  putreg32(regval, SAM_ADC_INPUTCTRL);
  priv->cur_channel = 0;

  sam_adc_synchronization();

  regval = priv->samplen;
  putreg8(regval, SAM_ADC_SAMPCTRL);

  putreg16(ADC_CTRLB_RESSEL_16BIT |
           (priv->prescaler << ADC_CTRLB_PRESCALER_OFFSET), SAM_ADC_CTRLB);

  sam_adc_synchronization();

  regval = priv->averaging;
  putreg8(regval, SAM_ADC_AVGCTRL);

  /* Enable ADC */

  regval = ADC_CTRLA_ENABLE;
  putreg8(regval, SAM_ADC_CTRLA);

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

  putreg8(0, SAM_ADC_CTRLA);

  sam_adc_synchronization();
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

      putreg8(ADC_INT_RESRDY, SAM_ADC_INTENSET);

      sam_adc_synchronization();

      putreg8(ADC_SWTRIG_START, SAM_ADC_SWTRIG);

      sam_adc_synchronization();
    }
  else
    {
      putreg8(0, SAM_ADC_INTENCLR);

      sam_adc_synchronization();
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

static int sam_adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  int ret = 0;
  struct sam_adc_priv    *priv = (struct sam_adc_priv *)dev->ad_priv;
  struct sam_adc_param_s *params = (struct sam_adc_param_s *)arg;

  switch (cmd)
  {
    case SAMD_ADC_IOCTL_START:
      sam_adc_setup(dev);
      sam_adc_rxint(dev, true);
      break;

    case SAMD_ADC_IOCTL_STOP:
      sam_adc_rxint(dev, false);
      sam_adc_shutdown(dev);
      break;

    case SAMD_ADC_IOCTL_SET_PARAMS:
      if ((getreg8(SAM_ADC_CTRLA) & ADC_CTRLA_ENABLE) != 0)
        {
          ret = -EBUSY;
          break;
        }

      priv->averaging = params->averaging;
      priv->prescaler = params->prescaler;
      priv->samplen = params->samplen;
      break;

    case SAMD_ADC_IOCTL_GET_PARAMS:
      params->averaging = priv->averaging;
      params->prescaler = priv->prescaler;
      params->samplen = priv->samplen;
      break;
  }

  return ret;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

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
  int ret;
  int ch_pos = 0;

  struct sam_adc_priv *priv = (struct sam_adc_priv *)
                              kmm_malloc(sizeof(struct sam_adc_priv));
  priv->genclk = genclk;

  g_sam_adc_dev.ad_priv = priv;
  g_sam_adc_dev.ad_ops = &sam_adc_ops;

  priv->num_channels = BOARD_ADC_NUM_CHANNELS;
  priv->channels = (int *)kmm_malloc(priv->num_channels * sizeof(int));

#if BOARD_ADC_REF == ADC_REFCTRL_REFSEL_VREFA
  sam_configport(PORT_ADC_VREFA);
#elif BOARD_ADC_REF == ADC_REFCTRL_REFSEL_VREFB
  sam_configport(PORT_ADC_VREFB);
#endif

#if BOARD_ADC_NEG == 0
  sam_configport(PORT_AIN0);
#elif BOARD_ADC_NEG == 1
  sam_configport(PORT_AIN1);
#elif BOARD_ADC_NEG == 2
  sam_configport(PORT_AIN2);
#elif BOARD_ADC_NEG == 3
  sam_configport(PORT_AIN3);
#elif BOARD_ADC_NEG == 4
  sam_configport(PORT_AIN4);
#elif BOARD_ADC_NEG == 5
  sam_configport(PORT_AIN5);
#elif BOARD_ADC_NEG == 6
  sam_configport(PORT_AIN6);
#elif BOARD_ADC_NEG == 7
  sam_configport(PORT_AIN7);
#endif
  priv->ref = BOARD_ADC_REF;
  priv->neg = BOARD_ADC_NEG;

#ifdef BOARD_ADC_SAMPLEN
  priv->samplen = BOARD_ADC_SAMPLEN;
#else
  priv->samplen = ADC_AVGCTRL_SAMPLENUM_32;
#endif
#ifdef BOARD_ADC_AVERAGING
  priv->averaging = BOARD_ADC_AVERAGING;
#else
  priv->averaging = ADC_AVGCTRL_SAMPLENUM_1024;
#endif
#ifdef BOARD_ADC_PRESCALER
  priv->prescaler = BOARD_ADC_PRESCALER;
#else
  priv->prescaler = ADC_CTRLB_PRESCALER_DIV256;
#endif

#ifdef BOARD_ADC_INPUT0
  ADC_INSERT_INPUT(0);
#endif
#ifdef BOARD_ADC_INPUT1
  ADC_INSERT_INPUT(1);
#endif
#ifdef BOARD_ADC_INPUT2
  ADC_INSERT_INPUT(2);
#endif
#ifdef BOARD_ADC_INPUT3
  ADC_INSERT_INPUT(3);
#endif
#ifdef BOARD_ADC_INPUT4
  ADC_INSERT_INPUT(4);
#endif
#ifdef BOARD_ADC_INPUT5
  ADC_INSERT_INPUT(5);
#endif
#ifdef BOARD_ADC_INPUT6
  ADC_INSERT_INPUT(6);
#endif
#ifdef BOARD_ADC_INPUT7
  ADC_INSERT_INPUT(7);
#endif
#ifdef BOARD_ADC_INPUT8
  ADC_INSERT_INPUT(8);
#endif
#ifdef BOARD_ADC_INPUT9
  ADC_INSERT_INPUT(9);
#endif
#ifdef BOARD_ADC_INPUT10
  ADC_INSERT_INPUT(10);
#endif
#ifdef BOARD_ADC_INPUT11
  ADC_INSERT_INPUT(11);
#endif
#ifdef BOARD_ADC_INPUT12
  ADC_INSERT_INPUT(12);
#endif
#ifdef BOARD_ADC_INPUT13
  ADC_INSERT_INPUT(13);
#endif
#ifdef BOARD_ADC_INPUT14
  ADC_INSERT_INPUT(14);
#endif
#ifdef BOARD_ADC_INPUT15
  ADC_INSERT_INPUT(15);
#endif
#ifdef BOARD_ADC_INPUT16
  ADC_INSERT_INPUT(16);
#endif
#ifdef BOARD_ADC_INPUT17
  ADC_INSERT_INPUT(17);
#endif
#ifdef BOARD_ADC_INPUT18
  ADC_INSERT_INPUT(18);
#endif
#ifdef BOARD_ADC_INPUT19
  ADC_INSERT_INPUT(19);
#endif
#ifdef BOARD_ADC_INPUT_BANDGAP
  if (ch_pos < priv->num_channels)
    {
      priv->channels[ch_pos++] = 0x19;
    }
#endif

#ifdef BOARD_ADC_INPUT_SCALEDCOREVCC
  if (ch_pos < priv->num_channels)
    {
      priv->channels[ch_pos++] = 0x1a;
    }
#endif

#ifdef BOARD_ADC_INPUT_SCALEDIOVCC
  if (ch_pos < priv->num_channels)
    {
      priv->channels[ch_pos++] = 0x1b;
    }
#endif

#ifdef BOARD_ADC_INPUT_DAC
  if (ch_pos < priv->num_channels)
    {
      priv->channels[ch_pos++] = 0x1c;
    }
#endif

  sam_adc_enableperiph();
  sam_adc_init_clock(&g_sam_adc_dev);

  flags = enter_critical_section();

  /* Attach Interrupt Handler */

  ret = irq_attach(SAM_IRQ_ADC, sam_adc_interrupt, &g_sam_adc_dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach irq %d\n", SAM_IRQ_ADC);
      leave_critical_section(flags);
      return NULL;
    }

  up_enable_irq(SAM_IRQ_ADC);
  leave_critical_section(flags);

  return (struct adc_dev_s *)&g_sam_adc_dev;
}
