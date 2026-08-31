/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_touch_gt911.c
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
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>

#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "espressif/esp_i2c.h"

#include "esp32p4-tab5.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int board_gt911_attach(FAR const struct gt9xx_board_s *state,
                              xcpt_t isr, FAR void *arg);
static void board_gt911_enable(FAR const struct gt9xx_board_s *state,
                               bool enable);
static int board_gt911_power(FAR const struct gt9xx_board_s *state, bool on);
static void board_gt911_report_id(FAR struct i2c_master_s *i2c);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gt9xx_board_s g_gt911_config =
{
  .irq_attach = board_gt911_attach,
  .irq_enable = board_gt911_enable,
  .set_power  = board_gt911_power,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gt911_attach
 *
 * Description:
 *   Drive TAB5_GPIO_TP_INT low.  These units have a pull-up to 3V3 on the
 *   touch interrupt line that stops the GT911 from scanning, so the pin is
 *   held low and no interrupt is used; contacts are picked up on read.
 *
 * Input Parameters:
 *   state - Pointer to the GT9XX board configuration structure.
 *   isr - The interrupt service routine (unused).
 *   arg - The argument for the interrupt service routine (unused).
 *
 * Returned Value:
 *   Zero on success.
 *
 ****************************************************************************/

static int board_gt911_attach(FAR const struct gt9xx_board_s *state,
                              xcpt_t isr, FAR void *arg)
{
  UNUSED(state);
  UNUSED(isr);
  UNUSED(arg);

  esp_configgpio(TAB5_GPIO_TP_INT, OUTPUT_FUNCTION_2);
  esp_gpiowrite(TAB5_GPIO_TP_INT, false);

  return OK;
}

/****************************************************************************
 * Name: board_gt911_enable
 *
 * Description:
 *   No interrupt is used on this board (see board_gt911_attach).
 *
 * Input Parameters:
 *   state - Pointer to the GT9XX board configuration structure.
 *   enable - True to enable the interrupt, false to disable it.
 *
 ****************************************************************************/

static void board_gt911_enable(FAR const struct gt9xx_board_s *state,
                               bool enable)
{
  UNUSED(state);
  UNUSED(enable);
}

/****************************************************************************
 * Name: board_gt911_power
 *
 * Description:
 *   Nothing to do: the rails and the reset line are driven earlier, by
 *   tab5_hmi_power_init().
 *
 * Input Parameters:
 *   state - Pointer to the GT9XX board configuration structure.
 *   on - True to power on, false to power off.
 *
 * Returned Value:
 *   Zero on success.
 *
 ****************************************************************************/

static int board_gt911_power(FAR const struct gt9xx_board_s *state, bool on)
{
  UNUSED(state);
  UNUSED(on);

  return OK;
}

/****************************************************************************
 * Name: board_gt911_report_id
 *
 * Description:
 *   Read and log the controller identification: product ID (ASCII, "911"
 *   for the GT911), firmware version and maximum coordinates, from
 *   register 0x8140.  A failed read is logged but is not fatal.
 *
 * Input Parameters:
 *   i2c - The I2C bus the controller is attached to.
 *
 ****************************************************************************/

static void board_gt911_report_id(FAR struct i2c_master_s *i2c)
{
  struct i2c_config_s config;
  uint8_t reg[2];
  uint8_t buf[10];
  char product[5];
  int ret;

  config.frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY;
  config.address   = CONFIG_ESP32P4_TAB5_TOUCH_GT911_ADDR;
  config.addrlen   = 7;

  /* 0x8140: product ID (4) + firmware version (2) + x max (2) + y max (2) */

  reg[0] = 0x81;
  reg[1] = 0x40;

  ret = i2c_writeread(i2c, &config, reg, sizeof(reg), buf, sizeof(buf));
  if (ret < 0)
    {
      syslog(LOG_WARNING, "gt911: ID read failed: %d\n", ret);
      return;
    }

  memcpy(product, buf, 4);
  product[4] = '\0';

  syslog(LOG_INFO,
         "gt911: product \"%s\" (%02x %02x %02x %02x) fw %02x%02x "
         "res %dx%d\n",
         product, buf[0], buf[1], buf[2], buf[3], buf[5], buf[4],
         buf[6] | (buf[7] << 8), buf[8] | (buf[9] << 8));
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

  board_gt911_report_id(i2c);

  ret = gt9xx_register("/dev/input0", i2c,
                       CONFIG_ESP32P4_TAB5_TOUCH_GT911_ADDR,
                       &g_gt911_config);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to register GT911: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "GT911 touchscreen controller initialized!\n");

  return OK;
}
