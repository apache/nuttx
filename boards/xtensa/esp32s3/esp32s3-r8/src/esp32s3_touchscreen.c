/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-r8/src/esp32s3_touchscreen.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/input/ft5x06.h>

#include "esp32s3_gpio.h"
#include "esp32s3_i2c.h"

#include "esp32s3-r8.h"

#ifdef CONFIG_ESP32S3_R8_TOUCHSCREEN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_TOUCHSCREEN_I2C_ADDR      0x38
#define BOARD_TOUCHSCREEN_I2C_CLOCK     400000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
  .wakeup    = esp32s3_ft5x06_wakeup,
  .nreset    = esp32s3_ft5x06_nreset
};

static bool g_touch_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void esp32s3_ft5x06_wakeup(const struct ft5x06_config_s *config)
{
}

static void esp32s3_ft5x06_nreset(const struct ft5x06_config_s *config,
                                  bool state)
{
  esp32s3_gpiowrite(ESP32S3_R8_TOUCH_RST, state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_touchscreen_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret;

  if (g_touch_initialized)
    {
      return OK;
    }

  esp32s3_configgpio(ESP32S3_R8_TOUCH_RST, OUTPUT_FUNCTION_2);
  esp32s3_gpiowrite(ESP32S3_R8_TOUCH_RST, false);
  up_mdelay(10);
  esp32s3_gpiowrite(ESP32S3_R8_TOUCH_RST, true);
  up_mdelay(50);

  i2c = esp32s3_i2cbus_initialize(ESP32S3_R8_TOUCH_I2C);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = ft5x06_register(i2c, &g_ft5x06_config, 0);

  if (ret == OK)
    {
      g_touch_initialized = true;
    }

  return ret;
}
#endif /* CONFIG_ESP32S3_R8_TOUCHSCREEN */
