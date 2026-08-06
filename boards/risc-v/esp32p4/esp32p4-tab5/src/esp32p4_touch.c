/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_touch.c
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

#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/st7123.h>

#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "espressif/esp_i2c.h"

#include "esp32p4-tab5.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int board_st7123_attach(FAR const struct st7123_config_s *config,
                               xcpt_t isr, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct st7123_config_s g_st7123_config =
{
  .attach = board_st7123_attach,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_st7123_attach
 *
 * Description:
 *   Configure TAB5_GPIO_TP_INT, wire it to the ST7123 driver interrupt
 *   handler, and enable the pin.
 *
 * Input Parameters:
 *   config - Pointer to the ST7123 configuration structure.
 *   isr - The interrupt service routine to call.
 *   arg - The argument to pass to the interrupt service routine.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

static int board_st7123_attach(FAR const struct st7123_config_s *config,
                               xcpt_t isr, FAR void *arg)
{
  int ret;

  UNUSED(config);

  /* Input with pull-up, falling-edge interrupt (active-low INT). */

  esp_configgpio(TAB5_GPIO_TP_INT, INPUT_FUNCTION_2 | PULLUP | FALLING);
  esp_gpioirqdisable(TAB5_GPIO_TP_INT);

  ret = esp_gpio_irq(TAB5_GPIO_TP_INT, isr, arg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to attach interrupt: %d\n", ret);
      return ret;
    }

  esp_gpioirqenable(TAB5_GPIO_TP_INT);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_touchscreen_init
 *
 * Description:
 *   Initialize the touch screen controller.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_touchscreen_init(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = esp_i2cbus_initialize(ESPRESSIF_I2C0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: failed to get I2C0 bus\n");
      return -ENODEV;
    }

  ret = st7123_register(i2c, 0, &g_st7123_config);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to register ST7123: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "ST7123 touchscreen controller initialized!\n");
  return OK;
}
