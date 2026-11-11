/****************************************************************************
 * boards/arm/stm32l0/npzero-g1s-dk/src/stm32_bringup.c
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
#include <sys/types.h>
#include <syslog.h>

#ifdef CONFIG_NPZERO
#  include <nuttx/i2c/i2c_master.h>
#  include <nuttx/power/npzero.h>
#  include "stm32_i2c.h"
#endif

#ifdef CONFIG_NPZERO_G1S_DK_PMOD_POWER
#  include <stdbool.h>
#  include <nuttx/arch.h>
#  include "stm32_gpio.h"
#endif

#include "npzero-g1s-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NPZERO_G1S_DK_PMOD_POWER
/* Half period of the bit-banged I2C clock (about 2 kHz): SDA can take up
 * to about 100 us to rise while the sensor boards are unpowered.
 */

#  define PMOD_BB_DELAY     250   /* us */

#  define PMOD_PSWINT       (NPZERO_PSWCTL_PSWINT(3) | \
                             NPZERO_PSWCTL_PSWINT(4))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NPZERO_G1S_DK_PMOD_POWER
/****************************************************************************
 * Name: pmod_scl / pmod_sda
 *
 * Description:
 *   Set the level of SCL or SDA (open drain) and wait a half period.  SCL
 *   waits for a slave stretching the clock.
 *
 ****************************************************************************/

static void pmod_scl(bool high)
{
  int n;

  stm32_gpiowrite(GPIO_NPZERO_SCL, high);
  for (n = 0; high && !stm32_gpioread(GPIO_NPZERO_SCL) && n < 100; n++)
    {
      up_udelay(PMOD_BB_DELAY);
    }

  up_udelay(PMOD_BB_DELAY);
}

static void pmod_sda(bool high)
{
  stm32_gpiowrite(GPIO_NPZERO_SDA, high);
  up_udelay(PMOD_BB_DELAY);
}

/****************************************************************************
 * Name: pmod_start / pmod_stop
 ****************************************************************************/

static void pmod_start(void)
{
  pmod_sda(true);
  pmod_scl(true);
  pmod_sda(false);
  pmod_scl(false);
}

static void pmod_stop(void)
{
  pmod_sda(false);
  pmod_scl(true);
  pmod_sda(true);
}

/****************************************************************************
 * Name: pmod_write
 *
 * Description:
 *   Send one byte, return true if the slave acknowledged it.
 *
 ****************************************************************************/

static bool pmod_write(uint8_t byte)
{
  bool ack;
  int i;

  for (i = 7; i >= 0; i--)
    {
      pmod_sda((byte >> i) & 1);
      pmod_scl(true);
      pmod_scl(false);
    }

  pmod_sda(true);
  pmod_scl(true);
  ack = !stm32_gpioread(GPIO_NPZERO_SDA);
  pmod_scl(false);
  return ack;
}

/****************************************************************************
 * Name: pmod_read
 *
 * Description:
 *   Read one byte and answer with a NACK (last byte).
 *
 ****************************************************************************/

static uint8_t pmod_read(void)
{
  uint8_t byte = 0;
  int i;

  pmod_sda(true);
  for (i = 0; i < 8; i++)
    {
      pmod_scl(true);
      byte = (byte << 1) | stm32_gpioread(GPIO_NPZERO_SDA);
      pmod_scl(false);
    }

  pmod_scl(true);
  pmod_scl(false);
  return byte;
}

/****************************************************************************
 * Name: stm32_pmod_power
 *
 * Description:
 *   Enable the power switches of PMOD 3 and 4 in Standby state.  After a
 *   reset of the nPZero they are off, and an unpowered sensor board slows
 *   down SDA too much for the STM32 I2C controller, so this is done with a
 *   slow bit-banged transfer before the I2C driver is initialized.  The
 *   other fields of PSWCTL (e.g. the host switch mode) are kept.
 *
 ****************************************************************************/

static int stm32_pmod_power(void)
{
  uint8_t addr = NPZERO_I2C_ADDR << 1;
  uint8_t pswctl;
  bool ok;

  stm32_configgpio(GPIO_NPZERO_SCL);
  stm32_configgpio(GPIO_NPZERO_SDA);

  pmod_start();
  ok = pmod_write(addr) && pmod_write(NPZERO_PSWCTL);
  pmod_start();
  ok = ok && pmod_write(addr | 1);
  pswctl = ok ? pmod_read() : 0;
  pmod_stop();

  if (ok && (pswctl & PMOD_PSWINT) != PMOD_PSWINT)
    {
      pmod_start();
      ok = pmod_write(addr) && pmod_write(NPZERO_PSWCTL) &&
           pmod_write(pswctl | PMOD_PSWINT);
      pmod_stop();

      /* Let the sensors power up */

      up_mdelay(2);
    }

  return ok ? OK : -EIO;
}
#endif

/****************************************************************************
 * Name: stm32_npzero_setup
 *
 * Description:
 *   Register the nPZero G1S driver.  The host is powered off while the
 *   nPZero is in Idle state, so this is done at each wake-up of the host.
 *
 ****************************************************************************/

#ifdef CONFIG_NPZERO
static int stm32_npzero_setup(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

#ifdef CONFIG_NPZERO_G1S_DK_PMOD_POWER
  ret = stm32_pmod_power();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: PMOD power failed: %d\n", ret);
    }
#endif

  i2c = stm32_i2cbus_initialize(NPZERO_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, NPZERO_I2C_BUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: i2c_register() failed: %d\n", ret);
    }
#endif

  ret = npzero_register(NPZERO_DEVPATH, i2c, NPZERO_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: npzero_register() failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_NPZERO
  /* Register the nPZero G1S driver */

  ret = stm32_npzero_setup();
#endif

  return ret;
}
