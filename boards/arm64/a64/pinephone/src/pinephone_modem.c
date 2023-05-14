/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_modem.c
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

/* Reference:
 *
 * "NuttX RTOS for PinePhone: 4G LTE Modem"
 * https://lupyuen.github.io/articles/lte
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include "chip.h"
#include "arm64_internal.h"
#include "a64_pio.h"
#include "pinephone_modem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeout for LTE Modem to be ready, in milliseconds */

#define MODEM_READY_TIMEOUT_MS 10000

/* Shortcut for Output Pin */

#define OUTPUT_PIN (PIO_OUTPUT | PIO_PULL_NONE | PIO_DRIVE_MEDLOW | \
                    PIO_INT_NONE | PIO_OUTPUT_SET)

/* LTE Modem Status at PH9 (Input) */

#define MODEM_STATUS    (PIO_INPUT | PIO_PORT_PIOH | PIO_PIN9)

/* LTE Modem Power at PL7 (Output) */

#define MODEM_PWR_BAT   (OUTPUT_PIN | PIO_PORT_PIOL | PIO_PIN7)

/* LTE Modem Reset at PC4 (Output, High-Low Inverted) */

#define MODEM_RESET_N   (OUTPUT_PIN | PIO_PORT_PIOC | PIO_PIN4)

/* LTE Modem AP Ready at PH7 (Output) */

#define MODEM_AP_READY  (OUTPUT_PIN | PIO_PORT_PIOH | PIO_PIN7)

/* LTE Modem DTR at PB2 (Output) */

#define MODEM_DTR       (OUTPUT_PIN | PIO_PORT_PIOB | PIO_PIN2)

/* LTE Modem Power Key at PB3 (Output, High-Low Inverted) */

#define MODEM_PWRKEY    (OUTPUT_PIN | PIO_PORT_PIOB | PIO_PIN3)

/* LTE Modem Wireless Disable at PH8 (Output) */

#define MODEM_W_DISABLE (OUTPUT_PIN | PIO_PORT_PIOH | PIO_PIN8)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_modem_wait
 *
 * Description:
 *   Wait for LTE Modem to be ready.  Poll the Modem Status until it
 *   becomes Low.  This takes about 3 seconds.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

static int pinephone_modem_wait(void)
{
  int i;
  int ret;

  ret = a64_pio_config(MODEM_STATUS);
  if (ret < 0)
    {
      _err("Configure Status failed: %d\n", ret);
      return ret;
    }

  for (i = 0; i < MODEM_READY_TIMEOUT_MS / 100; i++)
    {
      /* Modem Status is Low when ready */

      uint32_t status = a64_pio_read(MODEM_STATUS);

      if (status == 0)
        {
          return OK;
        }

      up_mdelay(100);
    }

  _err("Timeout waiting for Modem Ready\n");
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_modem_init
 *
 * Description:
 *   Initialize the Quectel EG25-G LTE Modem.  Power up the modem, deassert
 *   the Reset, toggle the Power Key and wait for the modem to be ready
 *   (3 seconds). Note that the Reset and Power Key Pins are High-Low
 *   Inverted for PinePhone.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_modem_init(void)
{
  int ret;

  /* Set PWR_BAT (4G-PWR-BAT) to High to Power On LTE Modem */

  ret = a64_pio_config(MODEM_PWR_BAT);
  if (ret < 0)
    {
      _err("Configure PWR_BAT failed: %d\n", ret);
      return ret;
    }

  a64_pio_write(MODEM_PWR_BAT, true);

  /* Set RESET_N (BB-RESET) to Low to Deassert LTE Modem Reset */

  ret = a64_pio_config(MODEM_RESET_N);
  if (ret < 0)
    {
      _err("Configure RESET_N failed: %d\n", ret);
      return ret;
    }

  a64_pio_write(MODEM_RESET_N, false);

  /* Set AP_READY to Low to wake up modem */

  ret = a64_pio_config(MODEM_AP_READY);
  if (ret < 0)
    {
      _err("Configure AP_READY failed: %d\n", ret);
      return ret;
    }

  a64_pio_write(MODEM_AP_READY, false);

  /* Set DTR to Low to wake up modem */

  ret = a64_pio_config(MODEM_DTR);
  if (ret < 0)
    {
      _err("Configure DTR failed: %d\n", ret);
      return ret;
    }

  a64_pio_write(MODEM_DTR, false);

  /* Wait 30 milliseconds for power to be stable */

  up_mdelay(30);

  /* Toggle PWRKEY (BB-PWRKEY) to Power On LTE Modem.
   * PWRKEY should be pulled up at least 500 ms, then pulled down.
   */

  ret = a64_pio_config(MODEM_PWRKEY);
  if (ret < 0)
    {
      _err("Configure PWRKEY failed: %d\n", ret);
      return ret;
    }

  a64_pio_write(MODEM_PWRKEY, true);
  up_mdelay(600);
  a64_pio_write(MODEM_PWRKEY, false);

  /* Set W_DISABLE# (BB-DISABLE) to High to Enable LTE Modem and
   * Disable Airplane Mode
   */

  ret = a64_pio_config(MODEM_W_DISABLE);
  if (ret < 0)
    {
      _err("Configure W_DISABLE failed: %d\n", ret);
      return ret;
    }

  a64_pio_write(MODEM_W_DISABLE, true);

  /* Wait for LTE Modem to be ready */

  ret = pinephone_modem_wait();
  if (ret < 0)
    {
      _err("Wait for Modem Ready failed: %d\n", ret);
      return ret;
    }

  return OK;
}
