/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_bcm20706.c
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

#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <arch/board/board.h>

#include "cxd56_gpio.h"
#include "cxd56_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCM20707_RST_N    PIN_SEN_IRQ_IN
#define BCM20707_DEV_WAKE PIN_EMMC_DATA3

#define BCM20707_RST_DELAY  (50 * 1000)  /* ms */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_bluetooth_pin_cfg
 *
 * Description:
 *   Setup pin configuration for bcm20706.
 *
 ****************************************************************************/

int board_bluetooth_pin_cfg(void)
{
  int ret = 0;

  ret = cxd56_gpio_config(BCM20707_RST_N, false);

  if (ret != 0)
    {
      return ret;
    }

  cxd56_gpio_write(BCM20707_RST_N, false);

  ret = cxd56_gpio_config(BCM20707_DEV_WAKE, false);

  if (ret != 0)
    {
      return ret;
    }

  cxd56_gpio_write(BCM20707_DEV_WAKE, true);

  return ret;
}

/****************************************************************************
 * Name: board_bluetooth_uart_pin_cfg
 *
 * Description:
 *   Setup UART pin configuration for bcm20706.
 *
 ****************************************************************************/

int board_bluetooth_uart_pin_cfg(void)
{
  int ret = 0;

  /* BCM20706 evaluation board might set pull-down.
   * Set float for UART2 CTS
   */

  ret = board_gpio_config(PIN_UART2_CTS, 1, 1, 0, 0);

  return ret;
}

/****************************************************************************
 * Name: board_bluetooth_reset
 *
 * Description:
 *   Reset BCM20706.
 *
 ****************************************************************************/

void board_bluetooth_reset(void)
{
  cxd56_gpio_write(BCM20707_RST_N, false);
  usleep(BCM20707_RST_DELAY);
  cxd56_gpio_write(BCM20707_RST_N, true);
  usleep(BCM20707_RST_DELAY);
}

/****************************************************************************
 * Name: board_bluetooth_power_control
 *
 * Description:
 *   Power ON/OFF BCM20706.
 *
 ****************************************************************************/

int board_bluetooth_power_control(bool en)
{
  int ret = 0;
  ret = board_power_control(POWER_BTBLE, en);
  return ret;
}

/****************************************************************************
 * Name: board_bluetooth_enable_sleep
 *
 * Description:
 *   Sleep mode ON/OFF BCM20706.
 *
 ****************************************************************************/

void board_bluetooth_enable_sleep(bool en)
{
  cxd56_gpio_write(BCM20707_DEV_WAKE, en);
}
