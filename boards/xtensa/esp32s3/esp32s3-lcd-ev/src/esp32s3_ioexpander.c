/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-lcd-ev/src/esp32s3_ioexpander.c
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

#include <unistd.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "esp32s3_i2c.h"
#include "esp32s3-lcd-ev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_IOEXPANDER_I2C_PORT       I2C_PORT

#define BOARD_IOEXPANDER_I2C_ADDR       (0x20)
#define BOARD_IOEXPANDER_I2C_CLOCK      (400 * 1000)

#define BOARD_IOEXPANDER_SET_DIRECTION  (3)
#define BOARD_IOEXPANDER_SET_OUTPUT     (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ioexpander
{
  struct i2c_master_s *i2c;
  uint8_t output_val;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_config_s g_i2c_config =
{
  .frequency = BOARD_IOEXPANDER_I2C_CLOCK,
  .address   = BOARD_IOEXPANDER_I2C_ADDR,
  .addrlen   = 7
};
static struct ioexpander g_ioexpander;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ioexpander_set_pin
 *
 * Description:
 *   Configure pin mode through the IO expander.
 *
 * Input Parameters:
 *   input_mask  - pin bit mask which need to be set input, if set pin 0 to
 *                 to be input, please make: input_mask = (1 << 0)
 *   output_mask - pin bit mask which need to be set output, if set pin 1 to
 *                 to be input, please make: output_mask = (1 << 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ioexpander_set_pin(uint8_t input_mask, uint8_t output_mask)
{
  uint8_t data[2] =
  {
    BOARD_IOEXPANDER_SET_DIRECTION, 0
  };

  if (input_mask & output_mask)
    {
      return -EINVAL;
    }

  data[1] |= input_mask;
  data[1] &= ~output_mask;

  return i2c_write(g_ioexpander.i2c, &g_i2c_config, data, 2);
}

/****************************************************************************
 * Name: board_ioexpander_output
 *
 * Description:
 *   Set pin output level through the IO expander.
 *
 * Input Parameters:
 *   pin   - pin number
 *   level - true for high level, false for low.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ioexpander_output(int pin, bool level)
{
  uint8_t data[2] =
  {
    BOARD_IOEXPANDER_SET_OUTPUT, 0
  };

  DEBUGASSERT(pin < 8);

  if (level)
    {
      g_ioexpander.output_val |= 1 << pin;
    }
  else
    {
      g_ioexpander.output_val &= ~(1 << pin);
    }

  data[1] = g_ioexpander.output_val;

  return i2c_write(g_ioexpander.i2c, &g_i2c_config, data, 2);
}

/****************************************************************************
 * Name: board_ioexpander_initialize
 *
 * Description:
 *   Initialize IO expander driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ioexpander_initialize(void)
{
  g_ioexpander.i2c = esp32s3_i2cbus_initialize(BOARD_IOEXPANDER_I2C_PORT);
  if (!g_ioexpander.i2c)
    {
      return -EINVAL;
    }

  return 0;
}
