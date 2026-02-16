/****************************************************************************
 * drivers/input/kmatrix_i2c.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/input/kmatrix.h>
#include <nuttx/ioexpander/ioexpander.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Verify IO expander support is enabled */

#if !defined(CONFIG_IOEXPANDER_PCA9538) && !defined(CONFIG_IOEXPANDER_MCP23X08)
#  error "Either CONFIG_IOEXPANDER_PCA9538 or " \
         "CONFIG_IOEXPANDER_MCP23X08 must be enabled"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uint32_t kmatrix_pin_t;

struct kmatrix_i2c_dev_s
{
  FAR struct ioexpander_dev_s *ioe;   /* IO expander device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void km_i2c_config_row(kmatrix_pin_t pin);
static void km_i2c_config_col(kmatrix_pin_t pin);
static void km_i2c_row_set(kmatrix_pin_t pin, bool active);
static bool km_i2c_col_get(kmatrix_pin_t pin);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global I2C device instance (simplified - one per board) */

static struct kmatrix_i2c_dev_s g_km_i2c_dev;

/****************************************************************************
 * I2C Keyboard Matrix Callbacks
 ****************************************************************************/

/**
 * Name: km_i2c_config_row
 *
 * Description:
 *   Configure row pins as outputs using IO expander API.
 */

static void km_i2c_config_row(kmatrix_pin_t pin)
{
  int ret;

  iinfo("I2C: Configuring pin %lu as output (row)\n", (unsigned long)pin);

  ret = IOEXP_SETDIRECTION(g_km_i2c_dev.ioe, (uint8_t)pin,
                           IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      ierr("ERROR: Failed to configure row pin %lu: %d\n",
           (unsigned long)pin, ret);
    }
}

/**
 * Name: km_i2c_config_col
 *
 * Description:
 *   Configure column pins as inputs with pull-up using IO expander API.
 */

static void km_i2c_config_col(kmatrix_pin_t pin)
{
  int ret;

  iinfo("I2C: Configuring pin %lu as input (column)\n", (unsigned long)pin);

  ret = IOEXP_SETDIRECTION(g_km_i2c_dev.ioe, (uint8_t)pin,
                           IOEXPANDER_DIRECTION_IN_PULLUP);
  if (ret < 0)
    {
      /* PCA9538 does not support IN_PULLUP; fall back to plain input. */

      iinfo("I2C: IN_PULLUP not supported for pin %lu, falling back to IN\n",
            (unsigned long)pin);

      ret = IOEXP_SETDIRECTION(g_km_i2c_dev.ioe, (uint8_t)pin,
                               IOEXPANDER_DIRECTION_IN);
      if (ret < 0)
        {
          ierr("ERROR: Failed to configure col pin %lu: %d\n",
               (unsigned long)pin, ret);
        }
    }
}

/**
 * Name: km_i2c_row_set
 *
 * Description:
 *   Control row output (active-low for matrix with diodes).
 */

static void km_i2c_row_set(kmatrix_pin_t pin, bool active)
{
  int ret;

  /* For active-low: active=true means write LOW (false) */

  ret = IOEXP_WRITEPIN(g_km_i2c_dev.ioe, (uint8_t)pin, !active);
  if (ret < 0)
    {
      ierr("ERROR: Failed to set row pin %lu: %d\n",
           (unsigned long)pin, ret);
    }

  iinfo("I2C: Row set pin %lu to %d\n", (unsigned long)pin, active ? 0 : 1);
}

/**
 * Name: km_i2c_col_get
 *
 * Description:
 *   Read column input (active-low with pull-up).
 */

static bool km_i2c_col_get(kmatrix_pin_t pin)
{
  bool value;
  int ret;

  ret = IOEXP_READPIN(g_km_i2c_dev.ioe, (uint8_t)pin, &value);
  if (ret < 0)
    {
      ierr("ERROR: Failed to read col pin %lu: %d\n",
           (unsigned long)pin, ret);
      return false;
    }

  /* Return inverted: true = active (low), false = inactive (high) */

  bool result = !value;

  iinfo("I2C: Col get pin %lu = %d\n", (unsigned long)pin, result);

  return result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Name: kmatrix_i2c_get_callbacks
 *
 * Description:
 *   Get the I2C callback functions to use in keyboard matrix config.
 *   This is called by board adapters to populate the callbacks.
 *
 * Returned Value:
 *   Structure with the callback function pointers.
 */

static struct kmatrix_callbacks_s g_km_i2c_callbacks =
{
  .config_row = km_i2c_config_row,
  .config_col = km_i2c_config_col,
  .row_set    = km_i2c_row_set,
  .col_get    = km_i2c_col_get,
};

FAR struct kmatrix_callbacks_s *kmatrix_i2c_get_callbacks(void)
{
  return &g_km_i2c_callbacks;
}

/**
 * Name: kmatrix_i2c_register
 *
 * Description:
 *   Register keyboard matrix driver using I2C GPIO expander.
 *   The IO expander device must already be initialized.
 *
 * Input Parameters:
 *   ioe_dev  - IO expander device (from mcp23x08_initialize or
 *              pca9538_initialize)
 *   config   - Keyboard matrix configuration (with callbacks set)
 *   devpath  - Device path (e.g., "/dev/kbd0")
 *
 * Returned Value:
 *   Zero on success; negated errno on failure.
 */

int kmatrix_i2c_register(FAR struct ioexpander_dev_s *ioe_dev,
                         FAR const struct kmatrix_config_s *config,
                         FAR const char *devpath)
{
  int ret;

  if (ioe_dev == NULL)
    {
      ierr("ERROR: IO expander device is NULL\n");
      return -EINVAL;
    }

  iinfo("Initializing keyboard matrix via I2C IO expander\n");

  /* Store IO expander device in global for callbacks */

  g_km_i2c_dev.ioe = ioe_dev;

  /* Register the keyboard matrix driver with provided config
   * (which must have callbacks already set by the board adapter)
   */

  ret = kmatrix_register(config, devpath);
  if (ret < 0)
    {
      ierr("ERROR: kmatrix_register failed: %d\n", ret);
      return ret;
    }

  iinfo("Keyboard matrix I2C driver registered successfully\n");
  return OK;
}
