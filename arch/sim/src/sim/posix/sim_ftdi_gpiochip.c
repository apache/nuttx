/****************************************************************************
 * arch/sim/src/sim/posix/sim_ftdi_gpiochip.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/const.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <ftdi.h>

#include "sim_gpiochip.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOCHIP_NPINS            8

#define gpioerr(fmt, ...) \
        syslog(LOG_ERR, "sim_ftdi_gpio: " fmt "\n", ##__VA_ARGS__)
#define gpioinfo(fmt, ...) \
        syslog(LOG_ERR, "sim_ftdi_gpio: " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* Host GPIOCHIP device definition */

struct host_gpiochip_dev
{
  struct ftdi_context *ftdi;
  uint8_t port_dir;
  uint8_t port_value;
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_gpiochip_direction
 *
 * Description:
 *   Provide gpio pin direction config.
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux ft2232h_gpio.
 *   pin   - The pin number.
 *   input - The direction of the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

int host_gpiochip_direction(struct host_gpiochip_dev *priv,
                            uint8_t pin, bool input)
{
  struct ftdi_context *ftdi = priv->ftdi;
  uint8_t dir = priv->port_dir;

  if (pin >= GPIOCHIP_NPINS)
    {
      gpioerr("ERROR: Invalid pin %d config\n", pin);
      return -EINVAL;
    }

  /* FTDI uses 1 as output and 0 as input, so invert input first */

  input = !input;
  dir &= ~(input << pin);
  dir |= (input << pin);

  if (ftdi_set_bitmode(ftdi, dir, BITMODE_SYNCBB) < 0)
    {
      gpioerr("Failed to change pin %d direction\n", pin);
      return -EIO;
    }

  priv->port_dir = dir;

  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_irq_request
 *
 * Input Parameters:
 *   priv   - A pointer to instance of Linux gpiochip.
 *   pin    - The pin number.
 *   cfgset - The config set of the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

int host_gpiochip_irq_request(struct host_gpiochip_dev *priv, uint8_t pin,
                              uint16_t cfg)
{
  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_writepin
 *
 * Description:
 *   Write ft2232h_gpio pin value.
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux ft2232h_gpio.
 *   pin   - The pin number.
 *   value - The value write to the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

int host_gpiochip_writepin(struct host_gpiochip_dev *priv,
                           uint8_t pin, bool value)
{
  struct ftdi_context *ftdi = priv->ftdi;
  uint8_t pins = priv->port_value;

  if (pin >= GPIOCHIP_NPINS)
    {
      gpioerr("ERROR: Invalid pin %d config\n", pin);
      return -EINVAL;
    }

  pins &= ~(value << pin);
  pins |= (value << pin);
  ftdi_write_data(ftdi, &pins, 1);
  priv->port_value = pins;

  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_readpin
 *
 * Description:
 *   Read ft2232h_gpio pin value.
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux ft2232h_gpio.
 *   pin   - The pin number.
 *   value - The value write to the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

int host_gpiochip_readpin(struct host_gpiochip_dev *priv,
                          uint8_t pin, bool *value)
{
  struct ftdi_context *ftdi = priv->ftdi;
  uint8_t pins;

  if (pin >= GPIOCHIP_NPINS)
    {
      gpioerr("ERROR: Invalid pin %d config\n", pin);
      return -EINVAL;
    }

  ftdi_read_pins(ftdi, &pins);
  priv->port_value = pins;

  *value = !!(pins & (1 << pin));

  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_irq_active
 *
 * Description:
 *   register gpio for gpiochip device
 *
 * Input Parameters:
 *   priv - A pointer to instance of Linux gpiochip.
 *   pin  - gpio pin of Linux gpiochip device.
 *
 * Returned Value:
 *   0 for OK.
 *
 ****************************************************************************/

bool host_gpiochip_irq_active(struct host_gpiochip_dev *priv, uint8_t pin)
{
  return false;
}

/****************************************************************************
 * Name: host_gpiochip_get_line
 *
 * Description:
 *   Get line info from ft2232h_gpio device
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux ft2232h_gpio.
 *   pin   - gpio line of Linux ft2232h_gpio.
 *   input - A pointer to direction of gpioline.
 *
 * Returned Value:
 *   0 for OK.
 *
 ****************************************************************************/

int host_gpiochip_get_line(struct host_gpiochip_dev *priv,
                           uint8_t pin, bool *input)
{
  uint8_t dir = priv->port_dir;

  if (pin >= GPIOCHIP_NPINS)
    {
      gpioerr("ERROR: Invalid pin %d config\n", pin);
      return -EINVAL;
    }

  /* For FTDI Input is defined as 0, so we need to invert here */

  *input = !(dir & (1 << pin));

  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_alloc
 *
 * Description:
 *   Initialize one ft2232h_gpio device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The pointer to the instance of Linux ft2232h_gpio device.
 *
 ****************************************************************************/

struct host_gpiochip_dev *host_gpiochip_alloc(uint8_t pins_dir)
{
  struct host_gpiochip_dev *dev;

  dev = malloc(sizeof(struct host_gpiochip_dev));
  if (!dev)
    {
      gpioerr("Failed to allocate memory for ft2232h_gpio device");
      return NULL;
    }

  dev->ftdi = ftdi_new();
  if (dev->ftdi == NULL)
    {
      gpioerr("Failed to initialize the new FTDI device!\n");
      free(dev);
      return NULL;
    }

  /* Interface A controls AD0-AD7 pins on SYNCBB mode */

  ftdi_set_interface(dev->ftdi, INTERFACE_A);

  /* Open the device */

  if (ftdi_usb_open(dev->ftdi, CONFIG_SIM_FTDI_VID,
                    CONFIG_SIM_FTDI_PID) < 0)
    {
      gpioerr("Failed to open the FTDI FT2232H device!\n");
      ftdi_free(dev->ftdi);
      free(dev);
      return NULL;
    }

  /* Reset the Bitmode */

  ftdi_set_bitmode(dev->ftdi, 0x00, BITMODE_RESET);

  /* Configure SYNCBB mode with the pins direction */

  if (ftdi_set_bitmode(dev->ftdi, pins_dir, BITMODE_SYNCBB) < 0)
    {
      gpioerr("Failed to enter SYNCBB mode\n");
      ftdi_usb_close(dev->ftdi);
      ftdi_free(dev->ftdi);
      free(dev);
      return NULL;
    }

  /* Save the current pins direction */

  dev->port_dir   = pins_dir;
  dev->port_value = 0x00;

  return dev;
}

/****************************************************************************
 * Name: host_gpiochip_free
 *
 * Description:
 *   Uninitialize an ft2232h_gpio device
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void host_gpiochip_free(struct host_gpiochip_dev *priv)
{
  ftdi_set_bitmode(priv->ftdi, 0x00, BITMODE_RESET);
  ftdi_usb_close(priv->ftdi);
  ftdi_free(priv->ftdi);
  free(priv);
}

