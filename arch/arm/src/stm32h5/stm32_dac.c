/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dac.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/analog/dac.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_dac.h"
#include "stm32_rcc.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only DAC1 is supported, but it has two channels */

#if !defined(CONFIG_STM32_DAC1)
#  error "DAC1 must be enabled"
#endif

#if !defined(CONFIG_STM32_DAC1CH1) && !defined(CONFIG_STM32_DAC1CH2)
#  error "At least one DAC1 channel must be enabled"
#endif

/* Helper macros for register bits */

#define DAC_CR_EN(ch)         ((ch) == 1 ? DAC_CR_EN1 : DAC_CR_EN2)
#define DAC_CR_TSEL_MASK(ch)  ((ch) == 1 ? DAC_CR_TSEL1_MASK : DAC_CR_TSEL2_MASK)
#define DAC_CR_WAVE_MASK(ch)  ((ch) == 1 ? DAC_CR_WAVE1_MASK : DAC_CR_WAVE2_MASK)
#define DAC_CR_MAMP_MASK(ch)  ((ch) == 1 ? DAC_CR_MAMP1_MASK : DAC_CR_MAMP2_MASK)
#define DAC_MCR_MODE_MASK(ch) ((ch) == 1 ? DAC_MCR_MODE1_MASK : DAC_MCR_MODE2_MASK)
#define DAC_MCR_MODE_EXT(ch)  ((ch) == 1 ? DAC_MCR_MODE1_NORM_EXT_BUF : \
                               DAC_MCR_MODE2_NORM_EXT_BUF)

/* The high-frequency interface mode must match the AHB clock frequency */

#if STM32_HCLK_FREQUENCY > 160000000
#  define DAC_MCR_HFSEL       DAC_MCR_HFSEL_AHB_160MHZ
#elif STM32_HCLK_FREQUENCY > 80000000
#  define DAC_MCR_HFSEL       DAC_MCR_HFSEL_AHB_80MHZ
#else
#  define DAC_MCR_HFSEL       DAC_MCR_HFSEL_DISABLED
#endif

/* Time to wait after enabling a channel before it accepts data (tWAKEUP is
 * at most a few microseconds; leave some margin).
 */

#define DAC_WAKEUP_USEC       15

/* Data holding register (12-bit, right aligned) for each channel */

#define DAC_DHR12R(ch)        ((ch) == 1 ? STM32_DAC1_DHR12R1 : STM32_DAC1_DHR12R2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the DAC block (DAC1) */

struct stm32_dac_s
{
  uint8_t init : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one DAC channel */

struct stm32_chan_s
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  const struct stm32_dac_ops_s *llops; /* Low-level DAC ops */
#endif
  uint8_t inuse  : 1;   /* True, the driver is in use */
  uint8_t ch;           /* Channel number (1 or 2) */
  uint32_t pin;         /* Pin configuration */
  uint32_t dro;         /* Data holding register address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stm32_dac_dumpregs(struct stm32_chan_s *chan);
static void stm32_dac_enable(struct stm32_chan_s *chan);

static void stm32_dac_reset(struct dac_dev_s *dev);
static int  stm32_dac_setup(struct dac_dev_s *dev);
static void stm32_dac_shutdown(struct dac_dev_s *dev);
static void stm32_dac_txint(struct dac_dev_s *dev, bool enable);
static int  stm32_dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int  stm32_dac_ioctl(struct dac_dev_s *dev, int cmd,
                            unsigned long arg);

static int  stm32_dac_chaninit(struct stm32_chan_s *chan);
static void stm32_dac_blockinit(void);

#ifdef CONFIG_STM32_DAC_LL_OPS
static void stm32_dac_llops_enable(struct stm32_dac_dev_s *dev,
                                   bool enabled);
static void stm32_dac_llops_writedro(struct stm32_dac_dev_s *dev,
                                     uint16_t data);
static void stm32_dac_llops_dumpregs(struct stm32_dac_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = stm32_dac_reset,
  .ao_setup    = stm32_dac_setup,
  .ao_shutdown = stm32_dac_shutdown,
  .ao_txint    = stm32_dac_txint,
  .ao_send     = stm32_dac_send,
  .ao_ioctl    = stm32_dac_ioctl,
};

#ifdef CONFIG_STM32_DAC_LL_OPS
static const struct stm32_dac_ops_s g_dac_llops =
{
  .enable        = stm32_dac_llops_enable,
  .write_dro     = stm32_dac_llops_writedro,
  .dump_regs     = stm32_dac_llops_dumpregs
};
#endif

#ifdef CONFIG_STM32_DAC1CH1
/* DAC1 channel 1 */

static struct stm32_chan_s g_dac1ch1priv =
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  .llops      = &g_dac_llops,
#endif
  .ch         = 1,
  .pin        = GPIO_DAC1_OUT1_0,
  .dro        = DAC_DHR12R(1),
};

static struct dac_dev_s g_dac1ch1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch1priv,
};
#endif /* CONFIG_STM32_DAC1CH1 */

#ifdef CONFIG_STM32_DAC1CH2
/* DAC1 channel 2 */

static struct stm32_chan_s g_dac1ch2priv =
{
#ifdef CONFIG_STM32_DAC_LL_OPS
  .llops      = &g_dac_llops,
#endif
  .ch         = 2,
  .pin        = GPIO_DAC1_OUT2_0,
  .dro        = DAC_DHR12R(2),
};

static struct dac_dev_s g_dac1ch2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch2priv,
};
#endif /* CONFIG_STM32_DAC1CH2 */

static struct stm32_dac_s g_dacblock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_dumpregs
 ****************************************************************************/

static void stm32_dac_dumpregs(struct stm32_chan_s *chan)
{
  UNUSED(chan);

  ainfo("CR:   0x%08" PRIx32 "  SR:  0x%08" PRIx32 "  MCR: 0x%08" PRIx32
        "\n",
        getreg32(STM32_DAC1_CR), getreg32(STM32_DAC1_SR),
        getreg32(STM32_DAC1_MCR));
  ainfo("DOR1: 0x%08" PRIx32 "  DOR2: 0x%08" PRIx32 "\n",
        getreg32(STM32_DAC1_DOR1), getreg32(STM32_DAC1_DOR2));
}

/****************************************************************************
 * Name: stm32_dac_enable
 *
 * Description:
 *   Enable a channel and wait for it to wake up.  Data written to the
 *   holding register while the channel is still waking up can be lost, so
 *   this must complete before any data is written.
 *
 ****************************************************************************/

static void stm32_dac_enable(struct stm32_chan_s *chan)
{
  if ((getreg32(STM32_DAC1_CR) & DAC_CR_EN(chan->ch)) == 0)
    {
      modifyreg32(STM32_DAC1_CR, 0, DAC_CR_EN(chan->ch));
      up_udelay(DAC_WAKEUP_USEC);
    }
}

/****************************************************************************
 * Name: stm32_dac_reset
 ****************************************************************************/

static void stm32_dac_reset(struct dac_dev_s *dev)
{
  struct stm32_chan_s *chan = dev->ad_priv;

  /* Disable the channel and clear its trigger, waveform and mode setup */

  modifyreg32(STM32_DAC1_CR, DAC_CR_EN(chan->ch) |
              DAC_CR_TSEL_MASK(chan->ch) | DAC_CR_WAVE_MASK(chan->ch) |
              DAC_CR_MAMP_MASK(chan->ch), 0);
  modifyreg32(STM32_DAC1_MCR, DAC_MCR_MODE_MASK(chan->ch), 0);
}

/****************************************************************************
 * Name: stm32_dac_setup
 ****************************************************************************/

static int stm32_dac_setup(struct dac_dev_s *dev)
{
  struct stm32_chan_s *chan = dev->ad_priv;
  int ret;

  /* Configure the channel if not already configured */

  if (!chan->inuse)
    {
      ret = stm32_dac_chaninit(chan);
      if (ret < 0)
        {
          aerr("ERROR: DAC channel init failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_dac_shutdown
 ****************************************************************************/

static void stm32_dac_shutdown(struct dac_dev_s *dev)
{
  /* Leave the channel enabled so that the last value written keeps being
   * driven after the device is closed.  Applications such as the dac
   * example open, write and close the device for every update, and
   * disabling the channel here would put the output back into high
   * impedance.  stm32_dac_reset() is what turns the channel off.
   */
}

/****************************************************************************
 * Name: stm32_dac_txint
 ****************************************************************************/

static void stm32_dac_txint(struct dac_dev_s *dev, bool enable)
{
  /* No interrupt support for DAC */
}

/****************************************************************************
 * Name: stm32_dac_send
 ****************************************************************************/

static int stm32_dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
  struct stm32_chan_s *chan = dev->ad_priv;

  /* The trigger is disabled, so the value is transferred to the output
   * as soon as it is written to the holding register.  The register must
   * be written with a 32-bit access; a halfword access faults on the H5.
   */

  stm32_dac_enable(chan);
  putreg32(msg->am_data & 0xfff, chan->dro);
  dac_txdone(dev);

  return OK;
}

/****************************************************************************
 * Name: stm32_dac_ioctl
 ****************************************************************************/

static int stm32_dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: stm32_dac_chaninit
 ****************************************************************************/

static int stm32_dac_chaninit(struct stm32_chan_s *chan)
{
  if (chan->inuse)
    {
      return -EBUSY;
    }

  stm32_configgpio(chan->pin);

  /* Disable the channel before configuration.  Leave the trigger disabled
   * (TEN=0) with no wave generation so that writes to the holding register
   * update the output directly.
   */

  modifyreg32(STM32_DAC1_CR, DAC_CR_EN(chan->ch) |
              DAC_CR_TSEL_MASK(chan->ch) | DAC_CR_WAVE_MASK(chan->ch) |
              DAC_CR_MAMP_MASK(chan->ch), 0);

  /* Normal mode, output to the external pin with the buffer enabled.  The
   * high-frequency interface mode is shared by both channels.
   */

  modifyreg32(STM32_DAC1_MCR, DAC_MCR_MODE_MASK(chan->ch) |
              DAC_MCR_HFSEL_MASK, DAC_MCR_MODE_EXT(chan->ch) |
              DAC_MCR_HFSEL);

  /* Enable the channel now so that it has woken up by the first write */

  stm32_dac_enable(chan);

  chan->inuse = 1;
  return OK;
}

/****************************************************************************
 * Name: stm32_dac_blockinit
 ****************************************************************************/

static void stm32_dac_blockinit(void)
{
  irqstate_t flags;

  if (g_dacblock.init)
    {
      return;
    }

  flags = enter_critical_section();

  /* Enable the DAC1 clock (AHB2) and pulse its reset */

  modifyreg32(STM32_RCC_AHB2ENR, 0, RCC_AHB2ENR_DAC1EN);
  modifyreg32(STM32_RCC_AHB2RSTR, 0, RCC_AHB2RSTR_DACRST);
  modifyreg32(STM32_RCC_AHB2RSTR, RCC_AHB2RSTR_DACRST, 0);

  g_dacblock.init = 1;
  leave_critical_section(flags);
}

#ifdef CONFIG_STM32_DAC_LL_OPS

/****************************************************************************
 * Name: stm32_dac_llops_enable
 ****************************************************************************/

static void stm32_dac_llops_enable(struct stm32_dac_dev_s *dev, bool enabled)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)dev;

  if (enabled)
    {
      stm32_dac_enable(chan);
    }
  else
    {
      modifyreg32(STM32_DAC1_CR, DAC_CR_EN(chan->ch), 0);
    }
}

/****************************************************************************
 * Name: stm32_dac_llops_writedro
 ****************************************************************************/

static void stm32_dac_llops_writedro(struct stm32_dac_dev_s *dev,
                                     uint16_t data)
{
  struct stm32_chan_s *chan = (struct stm32_chan_s *)dev;

  putreg32(data & 0xfff, chan->dro);
}

/****************************************************************************
 * Name: stm32_dac_llops_dumpregs
 ****************************************************************************/

static void stm32_dac_llops_dumpregs(struct stm32_dac_dev_s *dev)
{
  stm32_dac_dumpregs((struct stm32_chan_s *)dev);
}

#endif /* CONFIG_STM32_DAC_LL_OPS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.  The interface number corresponds to the channel:
 *   0 = DAC1 channel 1, 1 = DAC1 channel 2.
 *
 ****************************************************************************/

struct dac_dev_s *stm32_dacinitialize(int intf)
{
  struct dac_dev_s *dev;

  switch (intf)
    {
#ifdef CONFIG_STM32_DAC1CH1
      case 0:
        dev = &g_dac1ch1dev;
        break;
#endif
#ifdef CONFIG_STM32_DAC1CH2
      case 1:
        dev = &g_dac1ch2dev;
        break;
#endif
      default:
        aerr("ERROR: Invalid DAC interface: %d\n", intf);
        return NULL;
    }

  stm32_dac_blockinit();

  return dev;
}

#endif /* CONFIG_DAC */
