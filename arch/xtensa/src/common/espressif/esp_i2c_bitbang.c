/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_i2c_bitbang.c
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

#ifdef CONFIG_ESPRESSIF_I2C_BITBANG
#include <assert.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_bitbang.h>
#include <nuttx/kmalloc.h>

#include "espressif/esp_i2c_bitbang.h"

#if defined(CONFIG_ARCH_CHIP_ESP32S3)
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#include "esp32s2_gpio.h"
#include "esp32s2_gpio_sigmap.h"
#else
#include "esp32_gpio.h"
#include "esp32_gpio_sigmap.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_ESP32S3)
#define CONFIG_GPIO(pin, attr)                 esp32s3_configgpio(pin, attr)
#define GPIO_MATRIX_OUT(pin, idx, inv, en_inv) esp32s3_gpio_matrix_out(pin, \
                                                          idx, inv, en_inv)
#define GPIO_WRITE(pin, value)                 esp32s3_gpiowrite(pin, value)
#define GPIO_READ(pin)                         esp32s3_gpioread(pin)
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#define CONFIG_GPIO(pin, attr)                 esp32s2_configgpio(pin, attr)
#define GPIO_MATRIX_OUT(pin, idx, inv, en_inv) esp32s2_gpio_matrix_out(pin, \
                                                          idx, inv, en_inv)
#define GPIO_WRITE(pin, value)                 esp32s2_gpiowrite(pin, value)
#define GPIO_READ(pin)                         esp32s2_gpioread(pin)
#else
#define CONFIG_GPIO(pin, attr)                 esp32_configgpio(pin, attr)
#define GPIO_MATRIX_OUT(pin, idx, inv, en_inv) esp32_gpio_matrix_out(pin,   \
                                                          idx, inv, en_inv)
#define GPIO_WRITE(pin, value)                 esp32_gpiowrite(pin, value)
#define GPIO_READ(pin)                         esp3_gpioread(pin)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_i2c_bitbang_dev_s
{
  struct i2c_bitbang_lower_dev_s lower;
  int sda_pin;
  int scl_pin;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_i2c_bitbang_init(struct i2c_bitbang_lower_dev_s *lower);
static void esp_i2c_bitbang_set_scl(struct i2c_bitbang_lower_dev_s *lower,
                                    bool value);
static void esp_i2c_bitbang_set_sda(struct i2c_bitbang_lower_dev_s *lower,
                                    bool value);
static bool esp_i2c_bitbang_get_scl(struct i2c_bitbang_lower_dev_s *lower);
static bool esp_i2c_bitbang_get_sda(struct i2c_bitbang_lower_dev_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower-half I2C bitbang data  */

const static struct i2c_bitbang_lower_ops_s g_ops =
{
  .initialize = esp_i2c_bitbang_init,
  .set_scl    = esp_i2c_bitbang_set_scl,
  .set_sda    = esp_i2c_bitbang_set_sda,
  .get_scl    = esp_i2c_bitbang_get_scl,
  .get_sda    = esp_i2c_bitbang_get_sda
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_i2c_bitbang_init
 *
 * Description:
 *   Initialize the I2C bit-bang driver
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_i2c_bitbang_init(struct i2c_bitbang_lower_dev_s *lower)
{
  struct esp_i2c_bitbang_dev_s *dev = lower->priv;

  GPIO_WRITE(dev->scl_pin, 1);
  GPIO_WRITE(dev->sda_pin, 1);

  CONFIG_GPIO(dev->scl_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  GPIO_MATRIX_OUT(dev->scl_pin, SIG_GPIO_OUT_IDX, 0, 0);

  CONFIG_GPIO(dev->sda_pin, INPUT_PULLUP | OUTPUT_OPEN_DRAIN);
  GPIO_MATRIX_OUT(dev->sda_pin, SIG_GPIO_OUT_IDX, 0, 0);
}

/****************************************************************************
 * Name: esp_i2c_bitbang_set_scl
 *
 * Description:
 *   Set SCL line value
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *   value - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_i2c_bitbang_set_scl(struct i2c_bitbang_lower_dev_s *lower,
                                    bool value)
{
  struct esp_i2c_bitbang_dev_s *dev =
    (struct esp_i2c_bitbang_dev_s *)lower->priv;

  GPIO_WRITE(dev->scl_pin, value);
}

/****************************************************************************
 * Name: esp_i2c_bitbang_set_sda
 *
 * Description:
 *   Set SDA line value
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *   value - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_i2c_bitbang_set_sda(struct i2c_bitbang_lower_dev_s *lower,
                                    bool value)
{
  struct esp_i2c_bitbang_dev_s *dev =
    (struct esp_i2c_bitbang_dev_s *)lower->priv;

  GPIO_WRITE(dev->sda_pin, value);
}

/****************************************************************************
 * Name: esp_i2c_bitbang_get_scl
 *
 * Description:
 *   Get value from SCL line
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   The boolean representation of the SCL line value (true/false).
 *
 ****************************************************************************/

static bool esp_i2c_bitbang_get_scl(struct i2c_bitbang_lower_dev_s *lower)
{
  struct esp_i2c_bitbang_dev_s *dev =
    (struct esp_i2c_bitbang_dev_s *)lower->priv;

  return GPIO_READ(dev->scl_pin);
}

/****************************************************************************
 * Name: esp_i2c_bitbang_get_sda
 *
 * Description:
 *   Get value from SDA line
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   The boolean representation of the SDA line value (true/false).
 *
 ****************************************************************************/

static bool esp_i2c_bitbang_get_sda(struct i2c_bitbang_lower_dev_s *lower)
{
  struct esp_i2c_bitbang_dev_s *dev =
    (struct esp_i2c_bitbang_dev_s *)lower->priv;

  return GPIO_READ(dev->sda_pin);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_i2cbus_bitbang_initialize
 *
 * Description:
 *   Initialize the I2C bitbang driver. And return a unique instance of
 *   struct struct i2c_master_s. This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with
 *   a different frequency and slave address.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *esp_i2cbus_bitbang_initialize(void)
{
  struct esp_i2c_bitbang_dev_s *dev =
      (struct esp_i2c_bitbang_dev_s *)
          kmm_malloc(sizeof(struct esp_i2c_bitbang_dev_s));

  DEBUGASSERT(dev);

  dev->lower.ops = &g_ops;
  dev->lower.priv = dev;
  dev->scl_pin = CONFIG_ESPRESSIF_I2C_BITBANG_SCLPIN;
  dev->sda_pin = CONFIG_ESPRESSIF_I2C_BITBANG_SDAPIN;

  return i2c_bitbang_initialize(&dev->lower);
}
#endif /* CONFIG_ESPRESSIF_I2C_BITBANG */
