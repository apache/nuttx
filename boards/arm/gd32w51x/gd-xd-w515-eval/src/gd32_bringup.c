/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/src/gd32_bringup.c
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

#include <sys/types.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_I2C_DRIVER
#  include <nuttx/i2c/i2c_master.h>
#endif
#ifdef CONFIG_GD32W51X_ADC
#  include <nuttx/analog/adc.h>
#endif
#ifdef CONFIG_GD32W51X_PWM
#  include <nuttx/timers/pwm.h>
#endif
#ifdef CONFIG_USERLED_LOWER
#  include <nuttx/leds/userled.h>
#endif
#ifdef CONFIG_INPUT_BUTTONS_LOWER
#  include <nuttx/input/buttons.h>
#endif
#if defined(CONFIG_MTD_PROGMEM) || defined(CONFIG_MTD_GD25)
#  include <nuttx/mtd/mtd.h>
#endif
#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif
#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#endif
#ifdef CONFIG_GD30BC2416
#  include <nuttx/power/battery_charger.h>
#  include <nuttx/power/gd30bc2416.h>
#endif

#include "gd32w51x_gpio.h"
#include "gd32w51x_exti.h"

#include "gd32w51x_i2c.h"
#include "gd32w51x_adc.h"
#include "gd32w51x_pwm.h"
#include "gd32w51x_fwdgt.h"
#include "gd32w51x_qspi.h"
#include "gd32w51x_spi.h"
#include "gd32w51x_wwdgt.h"

#include <arch/board/board.h>

#include "gd-xd-w515-eval.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_GD32W51X_I2C0) && defined(CONFIG_GD30BC2416)
static int gd32_charger_irq_attach(
             FAR const struct gd30bc2416_config_s *config,
             xcpt_t isr, FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_GD32W51X_I2C0) && defined(CONFIG_GD30BC2416)
static const struct gd30bc2416_config_s g_charger_config =
{
  .frequency  = GD32_CHARGER_I2C_FREQUENCY,
  .current    = GD32_CHARGER_ISET_CURRENT,
  .irq_attach = gd32_charger_irq_attach,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_GD32W51X_I2C0) && defined(CONFIG_GD30BC2416)
/****************************************************************************
 * Name: gd32_charger_irq_attach
 *
 * Description:
 *   Attach the IRQ of the GD30BC2416 (PB2).  The 8 ms low pulse of the
 *   charger turns the PNP Q3 on, so PB2 (pulled down by R41) goes high:
 *   the rising edge marks the event.
 *
 ****************************************************************************/

static int gd32_charger_irq_attach(
             FAR const struct gd30bc2416_config_s *config,
             xcpt_t isr, FAR void *arg)
{
  return gd32_gpio_setevent(GPIO_CHARGER_IRQ, isr != NULL, false, false,
                            isr, arg);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int gd32_bringup(void)
{
#ifdef CONFIG_GD32W51X_I2C0
  struct i2c_master_s *i2c;
#endif
#if defined(CONFIG_GD32W51X_SPI1) && defined(CONFIG_SPI_DRIVER)
  struct spi_dev_s *spi;
#endif
#if defined(CONFIG_GD32W51X_I2C0) && defined(CONFIG_GD30BC2416)
  struct battery_charger_dev_s *charger;
#endif
#ifdef CONFIG_GD32W51X_ADC
  /* The analog inputs: NTC_COIL (PA1), NTC_Battery (PA2), NTC_USB (PA3)
   * and the battery current (PC2, ADC_I_VBAT)
   */

  static const uint8_t chanlist[] =
  {
    1, 2, 3, 6
  };

  struct adc_dev_s *adc;
#endif
#ifdef CONFIG_GD32W51X_PWM
  struct pwm_lowerhalf_s *pwm;
#endif
#ifdef CONFIG_MTD_PROGMEM
  struct mtd_dev_s *mtd;
#endif
#if defined(CONFIG_GD32W51X_QSPI) && defined(CONFIG_MTD_GD25)
  struct qspi_dev_s *qspi;
  struct mtd_dev_s *qmtd;
#endif
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED_LOWER
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      ferr("ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* The touch key, as /dev/buttons */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      ferr("ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_GD32W51X_I2C0
  i2c = gd32_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      ferr("ERROR: failed to initialize I2C0\n");
    }
#ifdef CONFIG_I2C_DRIVER
  else if (i2c_register(i2c, 0) < 0)
    {
      ferr("ERROR: failed to register /dev/i2c0\n");
    }
#endif

#ifdef CONFIG_GD30BC2416
  /* The GD30BC2416 battery charger is on I2C0 */

  if (i2c != NULL)
    {
      gd32_gpio_config(GPIO_CHARGER_IRQ);
      charger = gd30bc2416_initialize(i2c, &g_charger_config);
      if (charger == NULL)
        {
          ferr("ERROR: failed to initialize the GD30BC2416\n");
        }
      else if (battery_charger_register("/dev/batt0", charger) < 0)
        {
          ferr("ERROR: failed to register /dev/batt0\n");
        }
    }
#endif
#endif

#ifdef CONFIG_GD32W51X_ADC
  adc = gd32_adc_initialize(0, chanlist, sizeof(chanlist));
  if (adc == NULL)
    {
      ferr("ERROR: failed to initialize the ADC\n");
    }
  else if (adc_register("/dev/adc0", adc) < 0)
    {
      ferr("ERROR: failed to register /dev/adc0\n");
    }
#endif

#ifdef CONFIG_GD32W51X_PWM
  /* /dev/pwm0: vibration motor (TIMER1_CH0) */

  pwm = gd32_pwminitialize(1);
  if (pwm == NULL || pwm_register("/dev/pwm0", pwm) < 0)
    {
      ferr("ERROR: failed to register /dev/pwm0 (TIMER1)\n");
    }
#endif

#ifdef CONFIG_GD32W51X_FWDGT
  /* Free watchdog (IRC32K) as /dev/watchdog0 */

  gd32_fwdgt_initialize("/dev/watchdog0", BOARD_IRC32K_FREQUENCY);
#endif

#ifdef CONFIG_GD32W51X_WWDGT
  /* Window watchdog (APB1) as /dev/watchdog1 */

  gd32_wwdgt_initialize("/dev/watchdog1");
#endif

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
  ret = gd32_gpio_initialize();
  if (ret < 0)
    {
      ferr("ERROR: gd32_gpio_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_GD32W51X_SPI1
  /* SPI1: the fingerprint sensor (no driver yet), kept deselected and out
   * of reset
   */

  gd32_spidev_initialize();

#ifdef CONFIG_SPI_DRIVER
  spi = gd32_spibus_initialize(1);
  if (spi == NULL)
    {
      ferr("ERROR: failed to initialize SPI1\n");
    }
  else if (spi_register(spi, 1) < 0)
    {
      ferr("ERROR: failed to register /dev/spi1\n");
    }
#endif
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      ferr("ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_MTD_PROGMEM
  /* The progmem region of the internal flash as an MTD device, see
   * CONFIG_GD32W51X_PROGMEM_START_ADDR.
   */

  mtd = progmem_initialize();
  if (mtd == NULL)
    {
      ferr("ERROR: progmem_initialize failed\n");
    }
  else
    {
      ret = register_mtddriver("/dev/gd32flash", mtd, 0750, NULL);
      if (ret < 0)
        {
          ferr("ERROR: failed to register /dev/gd32flash: %d\n", ret);
        }
#ifdef CONFIG_FS_LITTLEFS
      else
        {
          ret = nx_mount("/dev/gd32flash", "/data", "littlefs", 0,
                         "autoformat");
          if (ret < 0)
            {
              ferr("ERROR: failed to mount /data: %d\n", ret);
            }
        }
#endif
    }
#endif

#if defined(CONFIG_GD32W51X_QSPI) && defined(CONFIG_MTD_GD25)
  /* The 16 MB QSPI NOR flash as /dev/qspiflash, mounted as LittleFS on
   * /flash (formatted on the first boot).
   */

  qspi = gd32_qspi_initialize(0);
  qmtd = qspi != NULL ? gd25_initialize(qspi, true) : NULL;
  if (qmtd == NULL)
    {
      ferr("ERROR: failed to initialize the QSPI flash\n");
    }
  else
    {
      ret = register_mtddriver("/dev/qspiflash", qmtd, 0755, NULL);
      if (ret < 0)
        {
          ferr("ERROR: failed to register /dev/qspiflash: %d\n", ret);
        }
#ifdef CONFIG_FS_LITTLEFS
      else
        {
          ret = nx_mount("/dev/qspiflash", "/flash", "littlefs", 0,
                         "autoformat");
          if (ret < 0)
            {
              ferr("ERROR: failed to mount /flash: %d\n", ret);
            }
        }
#endif
    }
#endif

  UNUSED(ret);
  return OK;
}
