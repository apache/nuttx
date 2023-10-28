/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_gnss_addon.c
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

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/sensors/cxd5610_gnss.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/chip/pin.h>
#include <arch/board/board.h>
#include "cxd56_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C interface */

#define CXD5610_I2C_ADDR  0x24
#define CXD5610_I2C_FREQ  400000
#define CXD5610_INT_PIN   PIN_SEN_IRQ_IN

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cxd5610_gnss_i2csend(struct cxd5610_gnss_lowerhalf_s *lower,
                                uint8_t *buffer, int buflen);
static int cxd5610_gnss_i2crecv(struct cxd5610_gnss_lowerhalf_s *lower,
                                uint8_t *buffer, int buflen);
static int cxd5610_gnss_enableint(struct cxd5610_gnss_lowerhalf_s *lower,
                                  void (*handler)(void));
static int cxd5610_gnss_disableint(struct cxd5610_gnss_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cxd5610_gnss_lowerops_s g_gnss_addon_ops =
{
  cxd5610_gnss_i2csend,
  cxd5610_gnss_i2crecv,
  cxd5610_gnss_enableint,
  cxd5610_gnss_disableint
};

static struct cxd5610_gnss_lowerhalf_s g_gnss_addon_lowerhalf =
{
  &g_gnss_addon_ops
};

static struct i2c_master_s *g_i2c;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cxd5610_gnss_i2csend(struct cxd5610_gnss_lowerhalf_s *lower,
                                uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = CXD5610_I2C_FREQ;
  msg.addr      = CXD5610_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = buflen;

  ret = I2C_TRANSFER(g_i2c, &msg, 1);

  return ret;
}

static int cxd5610_gnss_i2crecv(struct cxd5610_gnss_lowerhalf_s *lower,
                                uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = CXD5610_I2C_FREQ;
  msg.addr      = CXD5610_I2C_ADDR;
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  ret = I2C_TRANSFER(g_i2c, &msg, 1);
  return ret;
}

static int cxd5610_gnss_enableint(struct cxd5610_gnss_lowerhalf_s *lower,
                                  void (*handler)(void))
{
  /* Enable interrupt from CXD5610 device */

  board_gpio_config(CXD5610_INT_PIN, 0, true, false, PIN_PULLDOWN);
  board_gpio_intconfig(CXD5610_INT_PIN, INT_RISING_EDGE, false,
                       (xcpt_t)handler);
  board_gpio_int(CXD5610_INT_PIN, true);

  return OK;
}

static int cxd5610_gnss_disableint(struct cxd5610_gnss_lowerhalf_s *lower)
{
  /* Disable interrupt */

  board_gpio_int(CXD5610_INT_PIN, false);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_gnss_addon_initialize(const char *devpath, int bus)
{
  int ret;

  sninfo("Initializing CXD5610 GNSS...\n");

  /* Initialize i2c device */

  g_i2c = cxd56_i2cbus_initialize(bus);
  if (!g_i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = cxd5610_gnss_register(devpath, &g_gnss_addon_lowerhalf);
  if (ret < 0)
    {
      snerr("ERROR: registering CXD5610 GNSS.\n");
    }

  return ret;
}
