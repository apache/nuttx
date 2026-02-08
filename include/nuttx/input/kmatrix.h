/****************************************************************************
 * include/nuttx/input/kmatrix.h
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

#ifndef __INCLUDE_NUTTX_INPUT_KMATRIX_H
#define __INCLUDE_NUTTX_INPUT_KMATRIX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/input/keyboard.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t kmatrix_pin_t;

/* Keyboard matrix configuration structure passed to kmatrix_register() */

struct kmatrix_config_s
{
  uint8_t nrows;                          /* Number of rows */
  uint8_t ncols;                          /* Number of columns */
  FAR const kmatrix_pin_t *rows;          /* Array of row GPIO pins */
  FAR const kmatrix_pin_t *cols;          /* Array of column GPIO pins */

  /* Keymap: keycode[row * cols + col] */

  FAR const uint32_t *keymap;
  uint16_t poll_interval_ms;              /* Polling interval in milliseconds */

  /* GPIO callback functions specific to the SoC/board */

  void (*config_row)(kmatrix_pin_t pin);
  void (*config_col)(kmatrix_pin_t pin);
  void (*row_set)(kmatrix_pin_t pin, bool active);
  bool (*col_get)(kmatrix_pin_t pin);
};

#ifdef CONFIG_INPUT_KMATRIX_I2C

/* Keyboard matrix callback structure for I2C expanders */

struct kmatrix_callbacks_s
{
  void (*config_row)(kmatrix_pin_t pin);
  void (*config_col)(kmatrix_pin_t pin);
  void (*row_set)(kmatrix_pin_t pin, bool active);
  bool (*col_get)(kmatrix_pin_t pin);
};

#endif /* CONFIG_INPUT_KMATRIX_I2C */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: kmatrix_register
 *
 * Description:
 *   Configure and register a keyboard matrix device.  This will create the
 *   /dev/keypadN device node and enable keyboard scanning.
 *
 * Input Parameters:
 *   config - The keyboard matrix configuration.  This structure is not
 *            copied; it must persist for the lifetime of the driver.
 *   devpath - The device path for the /dev/keypadN device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int kmatrix_register(FAR const struct kmatrix_config_s *config,
                     FAR const char *devpath);

#ifdef CONFIG_INPUT_KMATRIX_I2C

/* Forward declaration */

struct ioexpander_dev_s;

/****************************************************************************
 * Name: kmatrix_i2c_register
 *
 * Description:
 *   Register keyboard matrix driver using I2C GPIO expander.
 *   The IO expander device must already be initialized using
 *   mcp23x08_initialize() or pca9538_initialize().
 *
 * Input Parameters:
 *   ioe_dev  - IO expander device (from mcp23x08_initialize or
 *              pca9538_initialize)
 *   config   - The keyboard matrix configuration (with callbacks set)
 *   devpath  - The device path for the /dev/keypadN device
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int kmatrix_i2c_register(FAR struct ioexpander_dev_s *ioe_dev,
                         FAR const struct kmatrix_config_s *config,
                         FAR const char *devpath);

/****************************************************************************
 * Name: kmatrix_i2c_get_callbacks
 *
 * Description:
 *   Get the I2C callback functions to use in keyboard matrix config.
 *   This is called by board adapters to populate the callbacks.
 *
 * Returned Value:
 *   Structure with the callback function pointers.
 *
 ****************************************************************************/

FAR struct kmatrix_callbacks_s *kmatrix_i2c_get_callbacks(void);

#endif /* CONFIG_INPUT_KMATRIX_I2C */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_KMATRIX_H */
