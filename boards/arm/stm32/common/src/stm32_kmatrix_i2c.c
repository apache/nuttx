/****************************************************************************
 * boards/arm/stm32/common/src/stm32_kmatrix_i2c.c
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

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/input/kmatrix.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_i2c.h"

#ifdef CONFIG_IOEXPANDER_MCP23X08
#  include <nuttx/ioexpander/mcp23x08.h>
#endif

#ifdef CONFIG_IOEXPANDER_PCA9538
#  include <nuttx/ioexpander/pca9538.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef uint32_t kmatrix_pin_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Row and column pin definitions for 4x3 keypad matrix via I2C expander
 *
 * For MCP23X08/PCA9538 I2C expanders, pins are numbered 0-7.
 *
 * Example mapping:
 *   Rows (outputs):    Pins 0-3
 *   Columns (inputs):  Pins 4-6 (with pull-ups)
 */

static const kmatrix_pin_t g_km_rows[] =
{
  0, 1, 2, 3,  /* Row 0-3: Output pins on expander */
};

static const kmatrix_pin_t g_km_cols[] =
{
  4, 5, 6,  /* Col 0-2: Input pins on expander (with pull-up) */
};

/* Keymap for 4x3 matrix - Standard phone keypad layout
 * Rows: 0-3, Columns: 0-2
 */

static const uint32_t g_km_keymap[] =
{
  '1', '2', '3',  /* Row 0 */
  '4', '5', '6',  /* Row 1 */
  '7', '8', '9',  /* Row 2 */
  '*', '0', '#',  /* Row 3 */
};

/* Get callbacks from I2C driver */

extern FAR struct kmatrix_callbacks_s *kmatrix_i2c_get_callbacks(void);

/* Keyboard matrix configuration structure
 * Callbacks are set in board_kmatrix_i2c_initialize.
 */

static struct kmatrix_config_s g_km_i2c_config =
{
  .nrows              = 4,
  .ncols              = 3,
  .rows               = g_km_rows,
  .cols               = g_km_cols,
  .keymap             = g_km_keymap,
  .poll_interval_ms   = CONFIG_INPUT_KMATRIX_POLL_MS,
};

/* IO expander configuration */

#ifdef CONFIG_IOEXPANDER_MCP23X08
static struct mcp23x08_config_s g_mcp23x08_config =
{
  .address   = CONFIG_STM32_KMATRIX_I2C_ADDR,
  .frequency = CONFIG_STM32_KMATRIX_I2C_FREQ,
};
#endif

#ifdef CONFIG_IOEXPANDER_PCA9538
static struct pca9538_config_s g_pca9538_config =
{
  .address   = CONFIG_STM32_KMATRIX_I2C_ADDR,
  .frequency = CONFIG_STM32_KMATRIX_I2C_FREQ,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Name: board_kmatrix_i2c_initialize
 *
 * Description:
 *   Initialize keyboard matrix driver using I2C GPIO expander.
 *   This function is called by stm32_bringup.c during initialization.
 *
 * Input Parameters:
 *   devpath - Device path (e.g., "/dev/kbd0")
 *
 * Returned Value:
 *   Zero on success; negated errno on failure.
 */

int board_kmatrix_i2c_initialize(const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  FAR struct ioexpander_dev_s *ioe;
  FAR struct kmatrix_callbacks_s *callbacks;
  int ret;

  iinfo("Initializing keyboard matrix via I2C expander\n");

  /* Initialize I2C bus */

  i2c = stm32_i2cbus_initialize(CONFIG_STM32_KMATRIX_I2C_BUS);
  if (i2c == NULL)
    {
      ierr("ERROR: Failed to initialize I2C bus %d\n",
           CONFIG_STM32_KMATRIX_I2C_BUS);
      return -ENODEV;
    }

  /* Initialize IO expander */

#ifdef CONFIG_IOEXPANDER_MCP23X08
  ioe = mcp23x08_initialize(i2c, &g_mcp23x08_config);
  if (ioe == NULL)
    {
      ierr("ERROR: Failed to initialize MCP23X08\n");
      stm32_i2cbus_uninitialize(i2c);
      return -ENODEV;
    }

  iinfo("MCP23X08 initialized at 0x%02x\n", CONFIG_STM32_KMATRIX_I2C_ADDR);
#elif defined(CONFIG_IOEXPANDER_PCA9538)
  ioe = pca9538_initialize(i2c, &g_pca9538_config);
  if (ioe == NULL)
    {
      ierr("ERROR: Failed to initialize PCA9538\n");
      stm32_i2cbus_uninitialize(i2c);
      return -ENODEV;
    }

  iinfo("PCA9538 initialized at 0x%02x\n", CONFIG_STM32_KMATRIX_I2C_ADDR);
#else
#  error "No IO expander configured"
#endif

  /* Get callbacks from I2C driver */

  callbacks = kmatrix_i2c_get_callbacks();
  g_km_i2c_config.config_row = callbacks->config_row;
  g_km_i2c_config.config_col = callbacks->config_col;
  g_km_i2c_config.row_set = callbacks->row_set;
  g_km_i2c_config.col_get = callbacks->col_get;

  /* Register keyboard matrix driver */

  ret = kmatrix_i2c_register(ioe, &g_km_i2c_config, devpath);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register keyboard matrix: %d\n", ret);
      stm32_i2cbus_uninitialize(i2c);
      return ret;
    }

  iinfo("Keyboard matrix I2C driver registered at %s\n", devpath);
  return OK;
}
