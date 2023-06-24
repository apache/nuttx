/****************************************************************************
 * boards/arm/nrf52/thingy52/src/nrf52_sx1509.c
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

#include <nuttx/irq.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/ioexpander/sx1509.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio.h>

#include "nrf52_i2c.h"

#include "thingy52.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SX1509_I2C_BUS  (0)
#define SX1509_I2C_ADDR (0x3e)
#define SX1509_I2C_FREQ (400000)

/* LEDs */

#define BOARD_LEDW_R 0
#define BOARD_LEDW_G 1
#define BOARD_LEDW_B 2
#define BOARD_LEDS_R 3
#define BOARD_LEDS_G 4
#define BOARD_LEDS_B 5

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf52_sx1509_set_nreset(bool state);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int  nrf52_sx1509_attach(struct sx1509_config_s *state, xcpt_t isr,
                                void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sx1509_config_s g_sx1509_cfg =
{
  .address        = SX1509_I2C_ADDR,
  .frequency      = SX1509_I2C_FREQ,
  .set_nreset     = nrf52_sx1509_set_nreset,
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  .attach         = nrf52_sx1509_attach,
#endif
#ifdef CONFIG_SX1509_LED_ENABLE
  .led_pre        = 2,
#endif
};
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_sx1509_set_nreset
 ****************************************************************************/

static void nrf52_sx1509_set_nreset(bool state)
{
  nrf52_gpio_write(GPIO_SX1509_RESET, state);
}

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/****************************************************************************
 * Name: nrf52_sx1509_attach
 ****************************************************************************/

static int nrf52_sx1509_attach(struct sx1509_config_s *state, xcpt_t isr,
                               void *arg)
{
#  error INT pin not supported
}
#endif

/****************************************************************************
 * Name: nrf52_sx1509_pinscfg
 ****************************************************************************/

static void nrf52_sx1509_pinscfg(struct ioexpander_dev_s *ioe)
{
  /* Pin 0: IOEXT0 */

  IOEXP_SETDIRECTION(ioe, 0, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  IOEXP_WRITEPIN(ioe, 0, false);
  gpio_lower_half(ioe, 0, GPIO_INPUT_PIN, 0);

  /* Pin 1: IOEXT1 */

  IOEXP_SETDIRECTION(ioe, 1, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 1, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 1, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  IOEXP_WRITEPIN(ioe, 1, false);
  gpio_lower_half(ioe, 1, GPIO_INPUT_PIN, 1);

  /* Pin 2: IOEXT2 */

  IOEXP_SETDIRECTION(ioe, 2, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  IOEXP_WRITEPIN(ioe, 2, false);
  gpio_lower_half(ioe, 2, GPIO_INPUT_PIN, 2);

  /* Pin 3: IOEXT3 */

  IOEXP_SETDIRECTION(ioe, 3, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  IOEXP_WRITEPIN(ioe, 3, false);
  gpio_lower_half(ioe, 3, GPIO_INPUT_PIN, 3);

  /* Pin 4: BAT_MON_EN */

  IOEXP_SETDIRECTION(ioe, 4, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 4, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 4, false);
  gpio_lower_half(ioe, 4, GPIO_OUTPUT_PIN, 4);

#ifdef CONFIG_SX1509_LED_ENABLE
  /* Pin 5: LIGHTWELL_G */

  IOEXP_SETDIRECTION(ioe, 5, IOEXPANDER_DIRECTION_OUT_LED);
  IOEXP_SETOPTION(ioe, 5, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_INVERT);
  IOEXP_SETOPTION(ioe, 5, IOEXPANDER_OPTION_LEDCFG,
                  (void *)BOARD_LEDW_G);
  IOEXP_WRITEPIN(ioe, 5, false);

  /* Pin 6: LIGHTWELL_B */

  IOEXP_SETDIRECTION(ioe, 6, IOEXPANDER_DIRECTION_OUT_LED);
  IOEXP_SETOPTION(ioe, 6, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_INVERT);
  IOEXP_SETOPTION(ioe, 6, IOEXPANDER_OPTION_LEDCFG,
                  (void *)BOARD_LEDW_B);
  IOEXP_WRITEPIN(ioe, 6, false);

  /* Pin 7: LIGHTWELL_R */

  IOEXP_SETDIRECTION(ioe, 7, IOEXPANDER_DIRECTION_OUT_LED);
  IOEXP_SETOPTION(ioe, 7, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_INVERT);
  IOEXP_SETOPTION(ioe, 7, IOEXPANDER_OPTION_LEDCFG,
                  (void *)BOARD_LEDW_R);
  IOEXP_WRITEPIN(ioe, 7, false);
#endif

  /* Pin 8: MPU_PWR_CTRL */

  IOEXP_SETDIRECTION(ioe, 8, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 8, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 8, false);
  gpio_lower_half(ioe, 8, GPIO_OUTPUT_PIN, 8);

  /* Pin 9: MIC_PWR_CTRL */

  IOEXP_SETDIRECTION(ioe, 9, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 9, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 9, false);
  gpio_lower_half(ioe, 9, GPIO_OUTPUT_PIN, 9);

  /* Pin 10: CSS_PWR_CTRL */

  IOEXP_SETDIRECTION(ioe, 10, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 10, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 10, false);
  gpio_lower_half(ioe, 10, GPIO_OUTPUT_PIN, 10);

  /* Pin 11: CSS_RESET */

  IOEXP_SETDIRECTION(ioe, 11, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 11, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 11, false);
  gpio_lower_half(ioe, 11, GPIO_OUTPUT_PIN, 11);

  /* Pin 12: CSS_WAKE */

  IOEXP_SETDIRECTION(ioe, 12, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 12, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 12, false);
  gpio_lower_half(ioe, 12, GPIO_OUTPUT_PIN, 12);

#ifdef CONFIG_SX1509_LED_ENABLE
  /* Pin 13: SENSE_LED_R */

  IOEXP_SETDIRECTION(ioe, 13, IOEXPANDER_DIRECTION_OUT_LED);
  IOEXP_SETOPTION(ioe, 13, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_INVERT);
  IOEXP_SETOPTION(ioe, 13, IOEXPANDER_OPTION_LEDCFG,
                  (void *)BOARD_LEDS_R);
  IOEXP_WRITEPIN(ioe, 13, false);

  /* Pin 14: SENSE_LED_G */

  IOEXP_SETDIRECTION(ioe, 14, IOEXPANDER_DIRECTION_OUT_LED);
  IOEXP_SETOPTION(ioe, 14, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_INVERT);
  IOEXP_SETOPTION(ioe, 14, IOEXPANDER_OPTION_LEDCFG,
                  (void *)BOARD_LEDS_G);
  IOEXP_WRITEPIN(ioe, 14, false);

  /* Pin 15: SENSE_LED_B */

  IOEXP_SETDIRECTION(ioe, 15, IOEXPANDER_DIRECTION_OUT_LED);
  IOEXP_SETOPTION(ioe, 15, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_INVERT);
  IOEXP_SETOPTION(ioe, 15, IOEXPANDER_OPTION_LEDCFG,
                  (void *)BOARD_LEDS_B);
  IOEXP_WRITEPIN(ioe, 15, false);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_sx1509_initialize
 *
 * Description:
 *   Configure the SX1509 io expander.
 *
 ****************************************************************************/

int nrf52_sx1509_initialize(void)
{
  struct i2c_master_s     *i2c = NULL;
  struct ioexpander_dev_s *ioe = NULL;
  int                      ret = OK;

  /* Configure sx1509 pins */

  nrf52_gpio_config(GPIO_SX1509_RESET);

  /* Get I2C bus */

  i2c = nrf52_i2cbus_initialize(SX1509_I2C_BUS);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to initialize I2C%d\n", SX1509_I2C_BUS);
      return -ENODEV;
    }

  /* Initialize sx1509 */

  ioe = sx1509_initialize(i2c, &g_sx1509_cfg);
  if (ioe == NULL)
    {
      _err("ERROR: Failed to initialize sx1509 %d\n", ret);
      return ret;
    }

  /* Register pins */

  nrf52_sx1509_pinscfg(ioe);

#ifdef CONFIG_SX1509_LED_ENABLE
  /* Initialize sx1509 LED driver */

  ret = sx1509_leds_initialize(ioe, "/dev/leds");
  if (ret < 0)
    {
      return ret;
    }
#endif

  return OK;
}
