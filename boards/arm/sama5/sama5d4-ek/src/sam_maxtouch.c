/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_maxtouch.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/mxt.h>

#include "arm_internal.h"
#include "sam_pio.h"
#include "sam_twi.h"

#include "sama5d4-ek.h"

#ifdef HAVE_MAXTOUCH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAMA5D4EK_MXT_I2CFREQUENCY
#  define CONFIG_SAMA5D4EK_MXT_I2CFREQUENCY 500000
#endif

#ifndef CONFIG_SAMA5D4EK_MXT_DEVMINOR
#  define CONFIG_SAMA5D4EK_MXT_DEVMINOR 0
#endif

/* The touchscreen communicates on TWI0, I2C address 0x4c */

#define MXT_TWI_BUS      0
#define MXT_I2C_ADDRESS  0x4c

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sama5d4ek_tscinfo_s
{
  /* Standard maXTouch interface */

  struct mxt_lower_s lower;

  /* Extensions for the sama5d4ek board */

  mxt_handler_t handler;
  void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/PIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the maXTouch driver from differences in PIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the maXTouch interrupt handler to the PIO interrupt
 *   enable  - Enable or disable the PIO interrupt
 *   clear   - Acknowledge/clear any pending PIO interrupt
 */

static int  mxt_attach(const struct mxt_lower_s *lower,
                       mxt_handler_t isr,
                       void *arg);
static void mxt_enable(const struct mxt_lower_s *lower, bool enable);
static void mxt_clear(const struct mxt_lower_s *lower);

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
    .frequency = CONFIG_SAMA5D4EK_MXT_I2CFREQUENCY,

    .attach    = mxt_attach,
    .enable    = mxt_enable,
    .clear     = mxt_clear,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * IRQ/PIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the maXTouch driver from differences in PIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the maXTouch interrupt handler to the PIO interrupt
 *   enable  - Enable or disable the PIO interrupt
 *   clear   - Acknowledge/clear any pending PIO interrupt
 *
 ****************************************************************************/

static int mxt_attach(const struct mxt_lower_s *lower, mxt_handler_t isr,
                      void *arg)
{
  if (isr)
    {
      /* Just save the address of the handler and its argument for now.
       * The new handler will called via mxt_interrupt() when the interrupt
       * occurs.
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

static void mxt_enable(const struct mxt_lower_s *lower, bool enable)
{
  /* Enable or disable interrupts */

  if (enable && g_mxtinfo.handler)
    {
      sam_pioirqenable(IRQ_CHG_MXT);
    }
  else
    {
      sam_pioirqdisable(IRQ_CHG_MXT);
    }
}

static void mxt_clear(const struct mxt_lower_s *lower)
{
  /* Does nothing */
}

static int mxt_interrupt(int irq, void *context, void *arg)
{
  /* Just forward the interrupt to the maXTouch driver */

  if (g_mxtinfo.handler)
    {
      return g_mxtinfo.handler(&g_mxtinfo.lower, g_mxtinfo.arg);
    }

  /* We got an interrupt with no handler.  This should not
   * happen.
   */

  sam_pioirqdisable(IRQ_CHG_MXT);
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
  struct i2c_master_s *i2c;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure the maXTouch CHG interrupt pin */

  sam_configpio(PIO_CHG_MXT);

  /* Get an instance of the I2C interface for the touchscreen chip select */

  i2c = sam_i2cbus_initialize(MXT_TWI_BUS);
  if (!i2c)
    {
      ierr("ERROR: Failed to initialize I2C%d\n", MXT_TWI_BUS);
      return -ENODEV;
    }

  /* Configure maXTouch CHG interrupts */

  sam_pioirq(PIO_CHG_MXT);
  irq_attach(IRQ_CHG_MXT, mxt_interrupt, NULL);

  /* Initialize and register the I2C touchscreen device */

  ret = mxt_register(i2c, &g_mxtinfo.lower, CONFIG_SAMA5D4EK_MXT_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register touchscreen device\n");
      irq_detach(IRQ_CHG_MXT);

      /* sam_i2cbus_uninitialize(i2c); */

      return -ENODEV;
    }

  return OK;
}

#endif /* HAVE_MAXTOUCH */
