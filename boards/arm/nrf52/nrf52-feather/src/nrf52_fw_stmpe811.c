/****************************************************************************
 * boards/arm/nrf52/nrf52-feather/src/nrf52_fw_stmpe811.c
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
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/stmpe811.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>

#include <nuttx/irq.h>

#include "nrf52_spi.h"

#include "nrf52-feather.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stmpe811_pollwork_s
{
    xcpt_t isr;
    FAR void *arg;
    struct work_s work;
    clock_t delay;
};

struct stmpe811_int_s
{
    FAR void (*handler)(FAR void *arg);
    FAR void *arg;
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the STMPE811 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.* so that contact and loss-
 * of-contact events can be detected.
 *
 * attach  - Attach the STMPE811 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int  stmpe811_attach(FAR struct stmpe811_config_s *state, xcpt_t isr,
                            FAR void *arg);
static void stmpe811_enable(FAR struct stmpe811_config_s *state,
                            bool enable);
static void stmpe811_clear(FAR struct stmpe811_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the STMPE811
 * driver.  This structure provides information about the configuration
 * of the STMPE811 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct stmpe811_config_s stmpe811_config =
{
#ifdef CONFIG_STMPE811_I2C
  .address   = STMPE811_ADDR1,
#endif
  .frequency = 1000000,

#ifdef CONFIG_STMPE811_MULTIPLE
  .irq       = NULL,
#endif
  .ctrl1     = 0,
  .ctrl2     = 0,
  .attach    = stmpe811_attach,
  .enable    = stmpe811_enable,
  .clear     = stmpe811_clear,
};

static struct stmpe811_dev_s *stmpe811_handle = NULL;
static struct stmpe811_pollwork_s stmpe811_pollwork;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the STMPE811 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.
 *
 * attach  - Attach the STMPE811 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

/* WARNING:  THIS IS A HACK RIGHT NOW TO IMPLEMENT POLLING FOR INTERRUPT
 *           THE KEYBOARD FEATHERWING DOES NOT HAVE A CONNECTED INT PIN
 *           FOR THIS :(
 */

static void stmpe811_pollworker(FAR void *arg)
{
  struct stmpe811_pollwork_s *priv = (struct stmpe811_pollwork_s *)arg;
  priv->isr(0, NULL, priv->arg);

  work_queue(HPWORK, &priv->work, stmpe811_pollworker, arg, priv->delay);
}

static int stmpe811_attach(FAR struct stmpe811_config_s *state, xcpt_t isr,
                           FAR void *arg)
{
  stmpe811_pollwork.isr = isr;
  stmpe811_pollwork.arg = arg;
  stmpe811_pollwork.delay = MSEC2TICK(100);
  return OK;
}

static void stmpe811_enable(FAR struct stmpe811_config_s *state, bool enable)
{
  int ret;
  if (enable)
    {
      if (work_available(&stmpe811_pollwork.work))
        {
          ret = work_queue(HPWORK, &stmpe811_pollwork.work,
                           stmpe811_pollworker, &stmpe811_pollwork, 0);
          if (ret != 0)
            {
              ierr("ERROR: Failed to queue work: %d\n", ret);
            }
        }
    }
  else
    {
      work_cancel(HPWORK, &stmpe811_pollwork.work);
    }
}

static void stmpe811_clear(FAR struct stmpe811_config_s *state)
{
  /* Nothing to do */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kb_backlightctl
 *
 * Description:
 *   This function is called to control the keyboard backlight attached
 *   to stmpe811
 *
 * Input Parameters:
 *   state - true or false to drive backlight
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kb_backlightctl(bool state)
{
  stmpe811_gpiowrite(stmpe811_handle, KB_FW_LCD_BL, state);
}

/****************************************************************************
 * Name: kb_card_state
 *
 * Description:
 *   This function returns the state of the sdcard
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True if card is inserted
 *
 ****************************************************************************/

bool kb_card_state(void)
{
  bool state;
  if (stmpe811_handle == NULL)
    {
      ierr("STMPE811 not configured yet.\n");
      return false;
    }

  stmpe811_gpioread(stmpe811_handle, KB_FW_CARD_DET, &state);
  state = state ? false : true;  /* Card inserted on low */
  if (state)
    {
      iinfo("Detected SD card inserted.");
    }
  else
    {
      iinfo("Detected SD card removed.");
    }

  return state;
}

/****************************************************************************
 * Name: fw_stmpe811_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int fw_stmpe811_setup(int minor)
{
  FAR struct spi_dev_s *spi;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Check if we are already initialized */

  if (!stmpe811_handle)
    {
      iinfo("Initializing\n");

      /* Get an instance of the SPI interface */

      spi = nrf52_spibus_initialize(KB_FW_STMP_SPI_DEV);
      if (!spi)
        {
          ierr("ERROR: Failed to initialize SPI bus %d\n",
               KB_FW_STMP_SPI_DEV);
          return -ENODEV;
        }

      /* Configure SPI */

      SPI_SETMODE(spi, SPIDEV_MODE0);
      SPI_SETBITS(spi, 8);
      SPI_HWFEATURES(spi, 0);
      SPI_SETFREQUENCY(spi, stmpe811_config.frequency);

      /* Instantiate the STMPE811 driver */

      stmpe811_handle = stmpe811_instantiate(spi, &stmpe811_config);
      if (!stmpe811_handle)
        {
          ierr("ERROR: Failed to instantiate the STMPE811 driver\n");
          return -ENODEV;
        }

      /* Initialize and register the I2C touchscreen device */

      ret = stmpe811_register(stmpe811_handle, minor);
      if (ret < 0)
        {
          ierr("ERROR: Failed to register STMPE driver: %d\n", ret);
          return -ENODEV;
        }

      stmpe811_gpioconfig(stmpe811_handle, KB_FW_LCD_BL);
#ifdef CONFIG_KB_FEATHERWING_SDCARD
      stmpe811_gpioconfig(stmpe811_handle, KB_FW_CARD_DET);
      stmpe811_gpioattach(stmpe811_handle, KB_FW_CARD_DET,
                          kb_feather_carddet);
#endif
#ifdef CONFIG_KB_FEATHERWING_KEYBOARD
      stmpe811_gpioconfig(stmpe811_handle, KB_FW_KBD_INT);
      stmpe811_gpioattach(stmpe811_handle, KB_FW_KBD_INT, kb_int);
#endif
    }

#ifdef CONFIG_KB_FEATHERWING_SDCARD
  /* Trigger a fake card detection event to insure mmcsd driver is
   * configured based on the real state now that it can be detected
   */

  kb_feather_carddet(KB_FW_CARD_DET_PIN);
#endif
  return OK;
}
