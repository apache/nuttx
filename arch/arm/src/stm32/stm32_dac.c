/************************************************************************************
 * arch/arm/src/stm32/stm32_dac.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32_internal.h"
#include "stm32_dac.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Up to 2 DAC interfaces are supported */

#if STM32_NDAC < 2
#  undef CONFIG_STM32_DAC2
#endif

#if STM32_NDAC < 1
#  undef CONFIG_STM32_DAC1
#endif

#if defined(CONFIG_STM32_DAC1) || defined(CONFIG_STM32_DAC2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the single STM32 DAC block */

struct stm32_dac_s
{
  uint8_t  init   : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one STM32 DAC channel */

struct stm32_chan_s
{
  uint8_t  inuse  : 1; /* True, the driver is in use and not available */
  uint8_t  intf;       /* DAC zero-based interface number (0 or 1) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* DAC Register access */

static uint32_t dac_getreg(struct stm32_chan_s *chan, int offset);
static void dac_putreg(struct stm32_chan_s *chan, int offset, uint32_t value);

/* Interrupt handler */

#ifdef CONFIG_STM32_STM32F40XX
static int  dac_interrupt(int irq, void *context);
#endif

/* DAC methods */

static void dac_reset(FAR struct dac_dev_s *dev);
static int  dac_setup(FAR struct dac_dev_s *dev);
static void dac_shutdown(FAR struct dac_dev_s *dev);
static void dac_txint(FAR struct dac_dev_s *dev, bool enable);
static int  dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);
static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg);
static int  dac_interrupt(int irq, void *context);

/* Initialization */

static int  dac_chaninit(struct stm32_chan_s *chan);
static int  dac_blockinit(void);

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

#ifdef CONFIG_STM32_DAC1
static struct stm32_chan_s g_dac1priv =
{
  .intf       = 0;
}

static struct dac_dev_s g_dac1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1priv,
};
#endif

#ifdef CONFIG_STM32_DAC2
static struct stm32_chan_s g_dac2priv =
{
  .intf       = 1;
}

static struct dac_dev_s g_dac2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac2priv,
};
#endif

static struct stm32_dac_s g_dacblock; 

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_getreg
 *
 * Description:
 *   Read the value of an DAC register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t dac_getreg(struct stm32_chan_s *chan, int offset)
{
  return getreg32(chan->base + offset);
}

/****************************************************************************
 * Name: dac_getreg
 *
 * Description:
 *   Read the value of an DAC register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_putreg(struct stm32_chan_s *chan, int offset, uint32_t value)
{
  putreg32(value, chan->base + offset);
}

/****************************************************************************
 * Name: dac_interrupt
 *
 * Description:
 *   DAC interrupt handler.  The STM32 F4 family supports a only a DAC
 *   underrun interrupt.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32F40XX
static int dac_interrupt(int irq, void *context)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC channel.  Called early to initialize the hardware. This
 *   is called, before dac_setup() and on error conditions.
 *
 *   NOTE:  DAC reset will reset both DAC channels!
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_reset(FAR struct dac_dev_s *dev)
{
  irqstate_t flags;
  uint32_t regval;

  flags   = irqsave();

#warning "Missing logic"

  irqrestore(flags);
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts.  Interrupts
 *   are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_setup(FAR struct dac_dev_s *dev)
{
# warning "Missing logic"
  return -ENOSYS;
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

static void dac_shutdown(FAR struct dac_dev_s *dev)
{
# warning "Missing logic"
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

static void dac_txint(FAR struct dac_dev_s *dev, bool enable)
{
# warning "Missing logic"
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

static int dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
# warning "Missing logic"
  return -ENOSYS;
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

static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
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

static int dac_chaninit(struct stm32_chan_s *chan)
{
  /* Is the selected channel already in-use? */

  if (chan->inuse)
    {
      /* Yes.. then return EBUSY */

      return -EBUSY;
    }

  /* Mark the DAC channel "in-use" */

  chan->inuse = 1;
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

static int dac_blockinit(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Has the DMA block already been initialized? */

  if (g_dacblock.init)
    {
      /* Yes.. then return success  We only have to do this once */

      return OK;
    }

  /* Put the entire DAC block in reset state */

  flags   = irqsave();
  regval  = getreg32(STM32_RCC_APB1RSTR);
  regval |= RCC_APB1RSTR_DACRST
  putreg32(regval, STM32_RCC_APB1RSTR);

  /* Take the DAC out of reset state */

  regval &= ~RCC_APB1RSTR_DACRST
  putreg32(regval, STM32_RCC_APB1RSTR);
  irqrestore(flags);

  /* Mark the DAC block as initialized */

  g_dacblock.init = 1;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid dac device structure reference on succcess; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the DAC block has enabled,
 *   2. Board-specific logic has already configured 
 *
 ****************************************************************************/

FAR struct dac_dev_s *stm32_dacinitialize(int intf)
{
  FAR struct dac_dev_s    *dev;
  FAR struct stm32_chan_s *chan;
  int ret;
  
#ifdef CONFIG_STM32_DAC1
  if (intf == 1)
    {
      avdbg("DAC1 Selected\n");
      dev = &g_dacdev1;
    }
  else
#endif
#ifdef CONFIG_STM32_DAC2
  if (intf == 2)
    {
      avdbg("DAC2 Selected\n");
      dev = &g_dac2dev;
    }
  else
#endif
    {
      adbg("No such DAC interface: %d\n", intf);
      return NULL;
    }

  /* Make sure that the DAC block has been initialized */

  ret = dac_blockinit();
  if (ret < 0)
    {
      adbg("Failed to initialize the DAC block: %d\n", ret);
      return ret;
    }

  /* Configure the selected DAC channel */

  chan = dev->ad_priv;
  ret  = dac_chaninit(chan);
  if (ret < 0)
    {
      adbg("Failed to initialize DAC channel %d: %d\n", intf, ret);
      return ret;
    }

  return dev;
}

#endif /* CONFIG_STM32_DAC1 || CONFIG_STM32_DAC2 */
#endif /* CONFIG_DAC */
