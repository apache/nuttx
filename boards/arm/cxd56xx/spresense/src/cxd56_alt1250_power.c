/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_alt1250_power.c
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

#if defined(CONFIG_MODEM_ALT1250)

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/wdog.h>
#include <arch/board/board.h>
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_alt1250_poweron
 *
 * Description:
 *   Power on the Altair modem device on the board.
 *
 ****************************************************************************/

void board_alt1250_poweron(void)
{
  board_power_control(POWER_LTE, true);
}

/****************************************************************************
 * Name: board_alt1250_poweroff
 *
 * Description:
 *   Power off the Altair modem device on the board.
 *
 ****************************************************************************/

void board_alt1250_poweroff(void)
{
  board_power_control(POWER_LTE, false);
}

/****************************************************************************
 * Name: board_alt1250_powerstatus
 *
 * Description:
 *   Get the power status for the Altair modem device on the board.
 *
 ****************************************************************************/

bool board_alt1250_powerstatus(void)
{
  return board_power_monitor(POWER_LTE);
}

/****************************************************************************
 * Name: board_alt1250_powerkeep
 *
 * Description:
 *   Set Modem power keep mode when turning off the board.
 *
 ****************************************************************************/

int board_alt1250_powerkeep(bool enable)
{
  if (enable)
    {
      return board_unset_reset_gpo(POWER_LTE);
    }
  else
    {
      return board_set_reset_gpo(POWER_LTE);
    }
}
#endif

