/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-lcd-ev/src/esp32s3_touchscreen.c
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

#include <syslog.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/input/ft5x06.h>

#include "esp32s3_i2c.h"
#include "esp32s3-lcd-ev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_TOUCHSCREEN_I2C_PORT    I2C_PORT

#define BOARD_TOUCHSCREEN_I2C_ADDR    (0x38)
#define BOARD_TOUCHSCREEN_I2C_CLOCK   (400 * 1000)

/****************************************************************************
 * Private Function Ptototypes
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int  esp32s3_ft5x06_attach(const struct ft5x06_config_s *config,
                                  xcpt_t isr, void *arg);
static void esp32s3_ft5x06_enable(const struct ft5x06_config_s *config,
                                  bool enable);
static void esp32s3_ft5x06_clear(const struct ft5x06_config_s *config);
#endif
static void esp32s3_ft5x06_wakeup(const struct ft5x06_config_s *config);
static void esp32s3_ft5x06_nreset(const struct ft5x06_config_s *config,
                                  bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ft5x06_config_s g_ft5x06_config =
{
  .address   = BOARD_TOUCHSCREEN_I2C_ADDR,
  .frequency = BOARD_TOUCHSCREEN_I2C_CLOCK,
#ifndef CONFIG_FT5X06_POLLMODE
  .attach    = esp32s3_ft5x06_attach,
  .enable    = esp32s3_ft5x06_enable,
  .clear     = esp32s3_ft5x06_clear,
#endif
  .wakeup    = esp32s3_ft5x06_wakeup,
  .nreset    = esp32s3_ft5x06_nreset
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE

/****************************************************************************
 * Name: esp32s3_ft5x06_attach
 *
 * Description:
 *   Attach an FT5x06 interrupt handler to a GPIO interrupt
 *
 ****************************************************************************/

static int  esp32s3_ft5x06_attach(const struct ft5x06_config_s *config,
                                  xcpt_t isr, void *arg)
{
  /* We do not have interrupt pin in the implementation */

  return 0;
}

/****************************************************************************
 * Name: esp32s3_ft5x06_enable
 *
 * Description:
 *   Enable or disable a GPIO interrupt
 *
 ****************************************************************************/

static void esp32s3_ft5x06_enable(const struct ft5x06_config_s *config,
                                  bool enable)
{
  /* We do not have interrupt pin in the implementation */
}

/****************************************************************************
 * Name: esp32s3_ft5x06_clear
 *
 * Description:
 *   Acknowledge/clear any pending GPIO interrupt
 *
 ****************************************************************************/

static void esp32s3_ft5x06_clear(const struct ft5x06_config_s *config)
{
  /* We do not have interrupt pin in the implementation */
}
#endif

/****************************************************************************
 * Name: esp32s3_ft5x06_wakeup
 *
 * Description:
 *   Issue WAKE interrupt to FT5x06 to change the FT5x06 from Hibernate to
 *   Active mode.
 *
 ****************************************************************************/

static void esp32s3_ft5x06_wakeup(const struct ft5x06_config_s *config)
{
  /* We do not have access to the WAKE pin in the implementation */
}

/****************************************************************************
 * Name: esp32s3_ft5x06_nreset
 *
 * Description:
 *   Control the chip reset pin
 *
 ****************************************************************************/

static void esp32s3_ft5x06_nreset(const struct ft5x06_config_s *config,
                                  bool state)
{
  /* We do not have access to the RESET pin in the implementation */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_touchscreen_initialize
 *
 * Description:
 *   Initialize touchpad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_touchscreen_initialize(void)
{
  int ret;
  struct i2c_master_s *i2c;

  i2c = esp32s3_i2cbus_initialize(BOARD_TOUCHSCREEN_I2C_PORT);
  if (!i2c)
    {
      return -EINVAL;
    }

  ret = ft5x06_register(i2c, &g_ft5x06_config, 0);
  if (ret != 0)
    {
      return ret;
    }

  return 0;
}
