/****************************************************************************
 * arch/sim/src/sim/posix/sim_linux_gpiochip.c
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

#include "sim_gpiochip.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define gpioerr(fmt, ...) \
        syslog(LOG_ERR, "sim_linux_gpiochip: " fmt "\n", ##__VA_ARGS__)
#define gpioinfo(fmt, ...) \
        syslog(LOG_ERR, "sim_linux_gpiochip: " fmt "\n", ##__VA_ARGS__)

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
 *   Provide gpiochip pin direction config.
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux gpiochip.
 *   pin   - The pin number.
 *   input - The direction of the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

int host_gpiochip_direction(struct host_gpiochip_dev *priv, uint8_t pin,
                            bool input)
{
  struct gpio_v2_line_request req;
  int nonblock = 1;
  int ret;

  memset(&req, 0, sizeof(req));
  req.num_lines = 1;
  req.offsets[0] = pin;

  if (priv->line_fd[pin] > 0)
    {
      close(priv->line_fd[pin]);
      priv->line_fd[pin] = -1;
    }

  if (input)
    {
      req.config.flags = GPIO_V2_LINE_FLAG_INPUT;
    }
  else
    {
      req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    }

  snprintf(req.consumer, sizeof(req.consumer) - 1, "gpio%d", pin);
  ret = host_uninterruptible(ioctl, priv->file, GPIO_V2_GET_LINE_IOCTL,
                             &req);
  if (ret < 0)
    {
      gpioerr("ERROR: pin %d set direction failed\n", pin);
      return -errno;
    }

  ret = host_uninterruptible(ioctl, req.fd, FIONBIO, &nonblock);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to set non-blocking: %s\n", strerror(errno));
      return -errno;
    }

  priv->line_fd[pin] = req.fd;

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
  struct gpio_v2_line_request req;
  int nonblock = 1;
  int ret;

  if (priv->line_fd[pin] > 0)
    {
      close(priv->line_fd[pin]);
      priv->line_fd[pin] = -1;
    }

  memset(&req, 0, sizeof(req));
  switch (cfg)
    {
      case GPIOCHIP_LINE_FLAG_FALLING:
        req.config.flags = GPIO_V2_LINE_FLAG_EDGE_FALLING;
        break;
      case GPIOCHIP_LINE_FLAG_RISING:
        req.config.flags = GPIO_V2_LINE_FLAG_EDGE_RISING;
        break;
      case GPIOCHIP_LINE_FLAG_BOTH:
        req.config.flags = GPIO_V2_LINE_FLAG_EDGE_FALLING |
                           GPIO_V2_LINE_FLAG_EDGE_RISING;
        break;
      default:
        req.config.flags = 0;
        break;
    }

  req.offsets[0] = pin;
  req.num_lines = 1;
  req.config.flags |= GPIO_V2_LINE_FLAG_INPUT;
  if (req.config.flags != GPIO_V2_LINE_FLAG_INPUT)
    {
      snprintf(req.consumer, sizeof(req.consumer) - 1, "gpio-irq%d", pin);
    }

  /* Warn only pin 10 can register in ch341A */

  ret = host_uninterruptible(ioctl, priv->file, GPIO_V2_GET_LINE_IOCTL,
                             &req);
  if (ret < 0)
    {
      gpioerr("ERROR: ioctl failed: %s \n", strerror(errno));
      return -errno;
    }

  ret = host_uninterruptible(ioctl, req.fd, FIONBIO, &nonblock);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to set non-blocking: %s\n", strerror(errno));
      return -errno;
    }

  priv->line_fd[pin] = req.fd;

  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_writepin
 *
 * Description:
 *   Write gpiochip pin value.
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux gpiochip.
 *   pin   - The pin number.
 *   value - The value write to the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

int host_gpiochip_writepin(struct host_gpiochip_dev *priv, uint8_t pin,
                           bool value)
{
  struct gpio_v2_line_values vals;
  int ret;

  if (priv->line_fd[pin] <= 0)
    {
      gpioerr("ERROR: Invalid pin %d config\n", pin);
      return -EINVAL;
    }

  memset(&vals, 0, sizeof(vals));
  vals.mask = 1;
  vals.bits = !!value;

  ret = host_uninterruptible(ioctl, priv->line_fd[pin],
                             GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to set pin %d value %d\n", pin, value);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_readpin
 *
 * Description:
 *   Read gpiochip pin value.
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux gpiochip.
 *   pin   - The pin number.
 *   value - The value write to the pin.
 *
 * Returned Value:
 *   0 for success, other for fail.
 ****************************************************************************/

int host_gpiochip_readpin(struct host_gpiochip_dev *priv, uint8_t pin,
                          bool *value)
{
  struct gpio_v2_line_values vals;
  int ret;

  if (priv->line_fd[pin] <= 0)
    {
      gpioerr("ERROR: Invalid pin %d config\n", pin);
      return -EINVAL;
    }

  memset(&vals, 0, sizeof(vals));
  vals.mask = 1;
  ret = host_uninterruptible(ioctl, priv->line_fd[pin],
                             GPIO_V2_LINE_GET_VALUES_IOCTL, &vals);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to get pin%d value, errno[%d]\n", pin, errno);
      return -errno;
    }

  *value = !!(vals.bits & 0x01);

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
  if (priv->line_fd[pin] > 0)
    {
      struct gpio_v2_line_event ev;
      int fd = priv->line_fd[pin];
      memset(&ev, 0, sizeof(ev));
      if (host_uninterruptible(read, fd, &ev, sizeof(ev)) == sizeof(ev))
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: host_gpiochip_get_line
 *
 * Description:
 *   Get line info from gpiochip device
 *
 * Input Parameters:
 *   priv  - A pointer to instance of Linux gpiochip.
 *   pin   - gpio line of Linux gpiochip.
 *   input - A pointer to direction of gpioline.
 *
 * Returned Value:
 *   0 for OK.
 *
 ****************************************************************************/

int host_gpiochip_get_line(struct host_gpiochip_dev *priv, uint8_t pin,
                           bool *input)
{
  struct gpio_v2_line_info info;
  int ret;

  memset(&info, 0, sizeof(info));
  info.offset = pin;
  ret = host_uninterruptible(ioctl, priv->file, GPIO_V2_GET_LINEINFO_IOCTL,
                             &info);
  if (ret < 0)
    {
      gpioerr("Failed to get line info: %d", ret);
      return -errno;
    }

  if (info.flags & GPIO_V2_LINE_FLAG_USED)
    {
      return 1;
    }

  if (info.flags & GPIO_V2_LINE_FLAG_OUTPUT)
    {
      *input = false;
    }
  else
    {
      *input = true;
    }

  return 0;
}

/****************************************************************************
 * Name: host_gpiochip_alloc
 *
 * Description:
 *   Initialize one gpiochip device
 *
 * Input Parameters:
 *   filename - the name of gpiochip device in Linux, e.g. "/dev/gpiochipN".
 *
 * Returned Value:
 *   The pointer to the instance of Linux gpiochip device.
 *
 ****************************************************************************/

struct host_gpiochip_dev *host_gpiochip_alloc(const char *filename)
{
  struct host_gpiochip_dev *dev;

  dev = malloc(sizeof(struct host_gpiochip_dev));
  if (!dev)
    {
      gpioerr("Failed to allocate memory for gpiochip device");
      return NULL;
    }

  dev->file = host_uninterruptible(open, filename, O_RDWR | O_CLOEXEC);
  if (dev->file < 0)
    {
      gpioerr("Failed to open %s: %d", filename, dev->file);
      free(dev);
      return NULL;
    }

  return dev;
}

/****************************************************************************
 * Name: host_gpiochip_free
 *
 * Description:
 *   Uninitialize an gpiochip device
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void host_gpiochip_free(struct host_gpiochip_dev *priv)
{
  host_uninterruptible(close, priv->file);
  free(priv);
}
