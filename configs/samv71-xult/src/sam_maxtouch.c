/************************************************************************************
 * configs/samv7-xult/src/sam_maxtouch.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/mxt.h>

#include "up_arch.h"
#include "sam_gpio.h"
#include "sam_twihs.h"

#include "samv71-xult.h"
#include "atmxt-xpro.h"

#ifdef HAVE_MAXTOUCH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SAMV71XULT_MXT_I2CFREQUENCY
#  define CONFIG_SAMV71XULT_MXT_I2CFREQUENCY 500000
#endif

#ifndef CONFIG_SAMV71XULT_MXT_DEVMINOR
#  define CONFIG_SAMV71XULT_MXT_DEVMINOR 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sama5d4ek_tscinfo_s
{
  /* Standard maXTouch interface */

  struct mxt_lower_s lower;

  /* Extensions for the sama5d4ek board */

  mxt_handler_t handler;
  FAR void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the maXTouch driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the maXTouch interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int  mxt_attach(FAR const struct mxt_lower_s *lower, mxt_handler_t isr,
                       FAR void *arg);
static void mxt_enable(FAR const struct mxt_lower_s *lower, bool enable);
static void mxt_clear(FAR const struct mxt_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the maXTouch
 * driver.  This structure provides information about the configuration
 * of the maXTouch and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

static struct sama5d4ek_tscinfo_s g_mxtinfo =
{
  .lower =
  {
    .address   = MXT_I2C_ADDRESS,
    .frequency = CONFIG_SAMV71XULT_MXT_I2CFREQUENCY,
#ifdef CONFIG_SAMV71XULT_MXT_SWAPXY
    .swapxy    = true;
#endif

    .attach    = mxt_attach,
    .enable    = mxt_enable,
    .clear     = mxt_clear,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the maXTouch driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the maXTouch interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 *
 ****************************************************************************/

static int mxt_attach(FAR const struct mxt_lower_s *lower, mxt_handler_t isr,
                      FAR void *arg)
{
  if (isr)
    {
      /* Just save the address of the handler and its argument for now.  The
       * new handler will called via mxt_interrupt() when the interrupt occurs.
       */

      iinfo("Attaching %p\n", isr);
      g_mxtinfo.handler = isr;
      g_mxtinfo.arg = arg;
    }
  else
    {
      iinfo("Detaching %p\n", g_mxtinfo.handler);
      mxt_enable(lower, false);
      g_mxtinfo.handler = NULL;
      g_mxtinfo.arg = NULL;
    }

  return OK;
}

static void mxt_enable(FAR const struct mxt_lower_s *lower, bool enable)
{
  /* Enable or disable interrupts */

  if (enable && g_mxtinfo.handler)
    {
      sam_gpioirqenable(IRQ_MXT_CHG);
    }
  else
    {
      sam_gpioirqdisable(IRQ_MXT_CHG);
    }
}

static void mxt_clear(FAR const struct mxt_lower_s *lower)
{
  /* Does nothing */
}

static int mxt_interrupt(int irq, FAR void *context, FAR void *arg)
{
  /* Just forward the interrupt to the maXTouch driver */

  if (g_mxtinfo.handler)
    {
      return g_mxtinfo.handler(&g_mxtinfo.lower, g_mxtinfo.arg);
    }

  /* We got an interrupt with no handler.  This should not
   * happen.
   */

  sam_gpioirqdisable(IRQ_MXT_CHG);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_tsc_setup(int minor)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure the maXTouch CHG interrupt pin */

  (void)sam_configgpio(GPIO_MXT_CHG);

  /* Get an instance of the I2C interface for the touchscreen chip select */

  i2c = sam_i2cbus_initialize(MXT_TWI_BUS);
  if (!i2c)
    {
      ierr("ERROR: Failed to initialize I2C%d\n", MXT_TWI_BUS);
      return -ENODEV;
    }

  /* Configure maXTouch CHG interrupts */

  sam_gpioirq(GPIO_MXT_CHG);
  (void)irq_attach(IRQ_MXT_CHG, mxt_interrupt, NULL);

  /* Initialize and register the I2C touchscreen device */

  ret = mxt_register(i2c, &g_mxtinfo.lower, CONFIG_SAMV71XULT_MXT_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register touchscreen device\n");
      irq_detach(IRQ_MXT_CHG);
      /* sam_i2cbus_uninitialize(i2c); */
      return -ENODEV;
    }

  return OK;
}

#endif /* HAVE_MAXTOUCH */
