/************************************************************************************
 * configs/samv71-xult/src/sam_wm8904.c
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

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/wm8904.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_gpio.h"
#include "sam_twihs.h"
#include "sam_ssc.h"
#include "sam_pck.h"

#include "samv71-xult.h"

#ifdef HAVE_WM8904

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samv71xult_mwinfo_s
{
  /* Standard MW8904 interface */

  struct wm8904_lower_s lower;

  /* Extensions for the samv71xult board */

  wm8904_handler_t handler;
  FAR void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the WM8904 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the WM8904 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 */

static int  wm8904_attach(FAR const struct wm8904_lower_s *lower,
                          wm8904_handler_t isr, FAR void *arg);
static bool wm8904_enable(FAR const struct wm8904_lower_s *lower,
                          bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the WM8904
 * driver.  This structure provides information about the configuration
 * of the WM8904 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

static struct samv71xult_mwinfo_s g_wm8904info =
{
  .lower =
  {
    .address   = WM8904_I2C_ADDRESS,
    .frequency = CONFIG_SAMV7D4EK_WM8904_I2CFREQUENCY,
#ifdef CONFIG_SAMV7D4EK_WM8904_SRCSCK
    .mclk      = BOARD_SLOWCLK_FREQUENCY,
#else
    .mclk      = BOARD_MAINCK_FREQUENCY,
#endif

    .attach    = wm8904_attach,
    .enable    = wm8904_enable,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the WM8904 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the WM8904 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 *
 ****************************************************************************/

static int wm8904_attach(FAR const struct wm8904_lower_s *lower,
                         wm8904_handler_t isr,  FAR void *arg)
{
  if (isr)
    {
      /* Just save the address of the handler and its argument for now.  The
       * new handler will called via wm8904_interrupt() when the interrupt occurs.
       */

      audinfo("Attaching %p\n", isr);
      g_wm8904info.handler = isr;
      g_wm8904info.arg = arg;
    }
  else
    {
      audinfo("Detaching %p\n", g_wm8904info.handler);
      (void)wm8904_enable(lower, false);
      g_wm8904info.handler = NULL;
      g_wm8904info.arg = NULL;
    }

  return OK;
}

static bool wm8904_enable(FAR const struct wm8904_lower_s *lower, bool enable)
{
  static bool enabled;
  irqstate_t flags;
  bool ret;

  /* Has the interrupt state changed */

  flags = enter_critical_section();
  if (enable != enabled)
    {
      /* Enable or disable interrupts */

      if (enable && g_wm8904info.handler)
        {
          audinfo("Enabling\n");
          sam_gpioirqenable(IRQ_INT_WM8904);
          enabled = true;
        }
      else
        {
          audinfo("Disabling\n");
          sam_gpioirqdisable(IRQ_INT_WM8904);
          enabled = false;
        }
    }

  ret = enabled;
  leave_critical_section(flags);
  return ret;
}

static int wm8904_interrupt(int irq, FAR void *context, FAR void *arg)
{
  /* Just forward the interrupt to the WM8904 driver */

  audinfo("handler %p\n", g_wm8904info.handler);
  if (g_wm8904info.handler)
    {
      return g_wm8904info.handler(&g_wm8904info.lower, g_wm8904info.arg);
    }

  /* We got an interrupt with no handler.  This should not
   * happen.
   */

  sam_gpioirqdisable(IRQ_INT_WM8904);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wm8904_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the WM8904 device.  This function will register the driver
 *   as /dev/wm8904[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_wm8904_initialize(int minor)
{
  FAR struct audio_lowerhalf_s *wm8904;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2c_master_s *i2c;
  FAR struct i2s_dev_s *i2s;
  static bool initialized = false;
  char devname[12];
  int ret;

  audinfo("minor %d\n", minor);
  DEBUGASSERT(minor >= 0 && minor <= 25);

  /* Have we already initialized?  Since we never uninitialize we must prevent
   * multiple initializations.  This is necessary, for example, when the
   * touchscreen example is used as a built-in application in NSH and can be
   * called numerous time.  It will attempt to initialize each time.
   */

  if (!initialized)
    {
      /* Configure the WM8904 interrupt pin */

      (void)sam_configgpio(GPIO_INT_WM8904);

      /* Get an instance of the I2C interface for the WM8904 chip select */

      i2c = sam_i2cbus_initialize(WM8904_TWI_BUS);
      if (!i2c)
        {
          auderr("ERROR: Failed to initialize TWI%d\n", WM8904_TWI_BUS);
          ret = -ENODEV;
          goto errout;
        }

      /* Get an instance of the I2S interface for the WM8904 data channel */

      i2s = sam_ssc_initialize(WM8904_SSC_BUS);
      if (!i2s)
        {
          auderr("ERROR: Failed to initialize SSC%d\n", WM8904_SSC_BUS);
          ret = -ENODEV;
          goto errout_with_i2c;
        }

      /* Configure the DAC master clock.  This clock is provided by PCK2 (PB10)
       * that is connected to the WM8904 MCLK.
       */

#ifdef CONFIG_SAMV7D4EK_WM8904_SRCSCK
      /* Drive the DAC with the slow clock (32.768 KHz).  The slow clock was
       * enabled in sam_boot.c if needed.
       */

      (void)sam_pck_configure(PCK2, PCKSRC_SCK, BOARD_SLOWCLK_FREQUENCY);
#else
      /* Drive the DAC with the main clock (12 MHz) */

      (void)sam_pck_configure(PCK2, PCKSRC_MAINCK, BOARD_MAINCK_FREQUENCY);
#endif

      /* Enable the DAC master clock */

      sam_pck_enable(PCK2, true);

      /* Configure WM8904 interrupts */

      sam_gpioirq(GPIO_INT_WM8904);
      ret = irq_attach(IRQ_INT_WM8904, wm8904_interrupt, NULL);
      if (ret < 0)
        {
          auderr("ERROR: Failed to attach WM8904 interrupt: %d\n", ret);
          goto errout_with_i2s;
        }

      /* Now we can use these I2C and I2S interfaces to initialize the
       * MW8904 which will return an audio interface.
       */

      wm8904 = wm8904_initialize(i2c, i2s, &g_wm8904info.lower);
      if (!wm8904)
        {
          auderr("ERROR: Failed to initialize the WM8904\n");
          ret = -ENODEV;
          goto errout_with_irq;
        }

      /* No we can embed the WM8904/I2C/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the WM8904
       * driver.
       */

      pcm = pcm_decode_initialize(wm8904);
      if (!pcm)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto errout_with_wm8904;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm%d",  minor);

      /* Finally, we can register the PCM/WM8904/I2C/I2S audio device.
       *
       * Is anyone young enough to remember Rube Goldberg?
       */

      ret = audio_register(devname, pcm);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/%s device: %d\n", devname, ret);
          goto errout_with_pcm;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

  /* Error exits.  Unfortunately there is no mechanism in place now to
   * recover resources from most errors on initialization failures.
   */

errout_with_pcm:
errout_with_wm8904:
errout_with_irq:
  irq_detach(IRQ_INT_WM8904);
errout_with_i2s:
errout_with_i2c:
errout:
  return ret;
}

#endif /* HAVE_WM8904 */
