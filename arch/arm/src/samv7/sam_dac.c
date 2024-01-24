/****************************************************************************
 * arch/arm/src/samv7/sam_dac.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>

#include "arm_internal.h"
#include "hardware/sam_dacc.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_pinmap.h"

#include "sam_gpio.h"
#include "sam_xdmac.h"
#include "sam_periphclks.h"
#include "sam_tc.h"
#include "sam_dac.h"

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get TC channel number from Trigger Selection value */

#define SAMV7_DAC_TC_CHANNEL (CONFIG_SAMV7_DAC_TRIGGER_SELECT - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of a single SAMV7 DAC
 * module
 */

struct sam_dac_s
{
  uint8_t   users;                 /* Number of channels using peripheral */
#ifdef CONFIG_SAMV7_DAC_TRIGGER
  TC_HANDLE tc;                    /* Timer handle */
#endif
};

/* This structure represents the internal state of one SAMV7 DAC channel */

struct sam_chan_s
{
  bool      inuse;                 /* True, the driver is in use and not available */
  uint8_t   intf;                  /* DAC zero-based interface number (0 or 1) */
  uint32_t  dro;                   /* Conversion Data Register */
#ifdef CONFIG_SAMV7_DAC_TRIGGER
  uint32_t  reg_dacc_trigr_clear;  /* channel DACC_TRIGR register clear bits */
  uint32_t  reg_dacc_trigr_set;    /* channel DACC_TRIGR register set bits */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handler */

static int  dac_interrupt(int irq, void *context, void *arg);

/* DAC methods */

static void dac_reset(struct dac_dev_s *dev);
static int  dac_setup(struct dac_dev_s *dev);
static void dac_shutdown(struct dac_dev_s *dev);
static void dac_txint(struct dac_dev_s *dev, bool enable);
static int  dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int  dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg);

/* Initialization */

#ifdef CONFIG_SAMV7_DAC_TRIGGER
static int  dac_timer_init(struct sam_dac_s *priv, uint32_t freq_required,
                           int channel);
static void dac_timer_free(struct sam_dac_s *priv);
#endif
static int  dac_channel_init(struct sam_chan_s *chan);
static int  dac_module_init(void);

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

#ifdef CONFIG_SAMV7_DAC0
static struct sam_chan_s g_dac1priv =
{
  .intf       = 0,
  .dro        = SAM_DACC_CDR0,
#ifdef CONFIG_SAMV7_DAC_TRIGGER
  .reg_dacc_trigr_clear = DACC_TRIGR_TRGSEL0_MASK | DACC_TRIGR_TRGEN0_EN,
  .reg_dacc_trigr_set   = DACC_TRIGR_TRGSEL0(
                            CONFIG_SAMV7_DAC_TRIGGER_SELECT) |
                            DACC_TRIGR_TRGEN0,
#endif
};

static struct dac_dev_s g_dac1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1priv,
};
#endif

#ifdef CONFIG_SAMV7_DAC1
static struct sam_chan_s g_dac2priv =
{
  .intf       = 1,
  .dro        = SAM_DACC_CDR1,
#ifdef CONFIG_SAMV7_DAC_TRIGGER
  .reg_dacc_trigr_clear = DACC_TRIGR_TRGSEL1_MASK | DACC_TRIGR_TRGEN1_EN,
  .reg_dacc_trigr_set   = DACC_TRIGR_TRGSEL1(
                            CONFIG_SAMV7_DAC_TRIGGER_SELECT) |
                            DACC_TRIGR_TRGEN1,
#endif
};

static struct dac_dev_s g_dac2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac2priv,
};
#endif

static struct sam_dac_s g_dacmodule;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_interrupt
 *
 * Description:
 *   DAC interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int dac_interrupt(int irq, void *context, void *arg)
{
  uint32_t status;

  status = getreg32(SAM_DACC_ISR) & getreg32(SAM_DACC_IMR);

#ifdef CONFIG_SAMV7_DAC0
  if (status & DACC_INT_TXRDY0)
    {
      dac_txdone(&g_dac1dev);
    }
#endif

#ifdef CONFIG_SAMV7_DAC1
  if (status & DACC_INT_TXRDY1)
    {
      dac_txdone(&g_dac2dev);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC channel.  Called early to initialize the hardware. This
 *   is called, before dac_setup() and on error conditions.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_reset(struct dac_dev_s *dev)
{
  struct sam_chan_s *chan = dev->ad_priv;
#ifdef CONFIG_SAMV7_DAC_TRIGGER
  uint32_t regval;
#endif
  irqstate_t flags;

  /* Reset only the selected DAC channel; the other DAC channel must remain
   * functional. The controller however does not have an option to reset
   * single channel, therefore we have to do this manually by writing zeroes
   * to all important registers.
   */

  /* This should be called only before dac_setup(), therefore the channel
   * should not be in use. Skip reset if it is.
   */

  if (chan->inuse)
    {
      /* Yes.. then return EBUSY */

      return;
    }

  flags = enter_critical_section();

  /* Disable channel */

  putreg32(1u << chan->intf, SAM_DACC_CHDR);

  /* Disable interrupts */

  dac_txint(dev, false);

#ifdef CONFIG_SAMV7_DAC_TRIGGER
  /* Reset trigger mode */

  regval = getreg32(SAM_DACC_TRIGR);
  regval &= ~chan->reg_dacc_trigr_clear;
  putreg32(regval, SAM_DACC_TRIGR);
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_setup(struct dac_dev_s *dev)
{
  struct sam_chan_s *chan = dev->ad_priv;
  int ret;

  /* Initialize the DAC peripheral module */

  ret = dac_module_init();
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize the DAC peripheral module: %d\n",
           ret);
      return ret;
    }

  /* Add channel user. We can do this because the upper layer checks
   * whether the device is opened for the first time and calls dac_setup
   * only if this is true. Therefore there can not be a situation where
   * the application would open DAC1 two times and dac_setup would be called
   * two times. We however have to check number of users (DAC0 and DAC1)
   * to know whether we want to disable the entire peripheral during
   * shutdown.
   */

  g_dacmodule.users++;

  /* Configure the selected DAC channel */

  ret = dac_channel_init(chan);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize DAC channel %d: %d\n",
           chan->intf, ret);
      return ret;
    }

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

static void dac_shutdown(struct dac_dev_s *dev)
{
  struct sam_chan_s *chan = dev->ad_priv;

  /* We can use dac_reset() to disable the channel. */

  chan->inuse = false;
  dac_reset(dev);

  /* Decrement number of peripheral users. */

  g_dacmodule.users--;

  if (g_dacmodule.users == 0)
    {
      /* This means there are no more channels using the peripheral. We
       * can disable entire peripheral and free timer.
       */

      /* Reset peripheral. This will simulate HW reset and clean all
       * registers.
       */

      putreg32(DACC_CR_SWRST, SAM_DACC_CR);

      /* Disable interrupt */

      up_disable_irq(SAM_IRQ_DACC);

      /* Detach interrupt */

      irq_detach(SAM_IRQ_DACC);

#ifdef CONFIG_SAMV7_DAC_TRIGGER
      /* Free timer */

      dac_timer_free(&g_dacmodule);
#endif
    }
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

static void dac_txint(struct dac_dev_s *dev, bool enable)
{
  struct sam_chan_s *chan;

  chan = dev->ad_priv;
  if (enable)
    {
      putreg32(DACC_INT_TXRDY0 << chan->intf, SAM_DACC_IER);
    }
  else
    {
      putreg32(DACC_INT_TXRDY0 << chan->intf, SAM_DACC_IDR);
    }
}

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

static int dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
  struct sam_chan_s *chan = dev->ad_priv;

  /* Interrupt based transfer */

  putreg32(msg->am_data & DACC_CDR_DATA0_MASK, chan->dro);

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

static int dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: dac_timer_init
 *
 * Description:
 *   Configure a timer to periodically trigger conversion. Only channels TC0,
 *   TC1, TC2 can be used with DAC.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_DAC_TRIGGER
static int dac_timer_init(struct sam_dac_s *priv, uint32_t freq_required,
                          int channel)
{
  uint32_t mode;
  uint32_t regval;
  uint32_t freq_actual;

  ainfo("required frequency=%ld [Hz], channel=%d\n",
        (long)freq_required, channel);

  DEBUGASSERT(priv && (freq_required > 0) && (channel >= 0 && channel <= 2));

  /* Set the timer/counter waveform mode the clock input. Use smallest
   * MCK divisor of 8 to have highest clock resolution thus smallest
   * frequency error. With 32 bit counter the lowest possible frequency of
   * 1 Hz is easily supported.
   */

  /* TODO Add support for TC_CMR_TCCLKS_PCK6 to reduce frequency error */

  mode = (TC_CMR_TCCLKS_MCK8 |      /* Use MCK/8 clock signal */
          TC_CMR_WAVSEL_UPRC |      /* UP mode w/ trigger on RC Compare */
          TC_CMR_WAVE |             /* Wave mode */
          TC_CMR_ACPA_CLEAR |       /* RA Compare Effect on TIOA: Clear */
          TC_CMR_ACPC_SET);         /* RC Compare Effect on TIOA: Set */

  /* Now allocate and configure the channel */

  priv->tc = sam_tc_allocate(channel, mode);
  if (!priv->tc)
    {
      aerr("ERROR: Failed to allocate channel %d mode %08" PRIx32 "\n",
            channel, mode);
      return -EINVAL;
    }

  /* Calculate the actual counter value from this divider and the tc input
   * frequency.
   */

  regval = BOARD_MCK_FREQUENCY / 8 / freq_required;
  DEBUGASSERT(regval > 0); /* Will check for integer underflow */

  /* Set up TC_RA and TC_RC.  The frequency is determined by RA and RC:
   * TIOA is cleared on RA match; TIOA is set on RC match.
   */

  sam_tc_setregister(priv->tc, TC_REGA, regval >> 1);
  sam_tc_setregister(priv->tc, TC_REGC, regval);

  freq_actual = BOARD_MCK_FREQUENCY / 8 / regval;
  ainfo("configured frequency=%ld [Hz]\n", (long)freq_actual);

  /* And start the timer */

  sam_tc_start(priv->tc);
  return OK;
}
#endif

/****************************************************************************
 * Name: dac_timer_free
 *
 * Description:
 *   Free the timer resource
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_DAC_TRIGGER
static void dac_timer_free(struct sam_dac_s *priv)
{
  /* Is a timer allocated? */

  ainfo("tc=%p\n", priv->tc);

  if (priv->tc)
    {
      /* Yes.. stop it and free it */

      sam_tc_stop(priv->tc);
      sam_tc_free(priv->tc);
      priv->tc = NULL;
    }
}
#endif

/****************************************************************************
 * Name: dac_channel_init
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

static int dac_channel_init(struct sam_chan_s *chan)
{
  /* Is the selected channel already in-use? */

  if (chan->inuse)
    {
      /* Yes.. then return EBUSY */

      return -EBUSY;
    }

#ifdef CONFIG_SAMV7_DAC_TRIGGER
  /* Configure trigger mode operation */

  ainfo("Enabled trigger mode for DAC%d\n", chan->intf);

  modifyreg32(SAM_DACC_TRIGR,
              chan->reg_dacc_trigr_clear,
              chan->reg_dacc_trigr_set);
#endif

  /* Enable DAC Channel */

  putreg32(1u << chan->intf, SAM_DACC_CHER);

  /* Mark the DAC channel "in-use" */

  chan->inuse = true;
  return OK;
}

/****************************************************************************
 * Name: dac_module_init
 *
 * Description:
 *   Initialize the DAC. All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_module_init(void)
{
  uint32_t regval;
  int ret;

  /* Has the DAC block already been users? */

  if (g_dacmodule.users > 0)
    {
      /* Yes.. then return success  We only have to do this once */

      return OK;
    }

  ainfo("Initializing...\n");

  /* Disable DAC peripheral clock */

  sam_dacc_disableclk();

  /* Configure DAC pins */

#ifdef CONFIG_SAMV7_DAC0
  sam_configgpio(GPIO_DAC0);
#endif
#ifdef CONFIG_SAMV7_DAC1
  sam_configgpio(GPIO_DAC1);
#endif

  /* Enable the DAC peripheral clock */

  sam_dacc_enableclk();

  /* Reset the DAC controller */

  putreg32(DACC_CR_SWRST, SAM_DACC_CR);

  /* Set the MCK clock prescaler: PRESCAL = (MCK / DACClock) - 2 */

  regval =  DACC_MR_PRESCALER(CONFIG_SAMV7_DAC_PRESCAL);
  putreg32(regval, SAM_DACC_MR);

  /* Configure trigger mode operation */

#ifdef CONFIG_SAMV7_DAC_TRIGGER
  ret = dac_timer_init(&g_dacmodule,
                       CONFIG_SAMV7_DAC_TRIGGER_FREQUENCY,
                       SAMV7_DAC_TC_CHANNEL);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize the timer: %d\n", ret);
      return ret;
    }
#endif

  /* Configure interrupts */

  ret = irq_attach(SAM_IRQ_DACC, dac_interrupt, NULL);
  if (ret < 0)
    {
      aerr("irq_attach failed: %d\n", ret);
      return ret;
    }

  ainfo("Enable the DAC interrupt: irq=%d\n", SAM_IRQ_DACC);
  up_enable_irq(SAM_IRQ_DACC);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dac_initialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid DAC device structure reference on success, NULL on failure.
 *
 ****************************************************************************/

struct dac_dev_s *sam_dac_initialize(int intf)
{
  struct dac_dev_s *dev;

#ifdef CONFIG_SAMV7_DAC0
  if (intf == 0)
    {
      ainfo("DAC1 Selected\n");
      dev = &g_dac1dev;
    }
  else
#endif
#ifdef CONFIG_SAMV7_DAC1
  if (intf == 1)
    {
      ainfo("DAC2 Selected\n");
      dev = &g_dac2dev;
    }
  else
#endif
    {
      aerr("ERROR: No such DAC interface: %d\n", intf);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_SAMV7_DAC0 || CONFIG_SAMV7_DAC1 */
