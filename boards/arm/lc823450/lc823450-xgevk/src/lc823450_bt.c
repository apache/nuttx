/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450_bt.c
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

#include <unistd.h>

#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "lc823450_gpio.h"
#include "lc823450-xgevk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_POWER (GPIO_PORT1 | GPIO_PIN7)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_bt_enable
 ****************************************************************************/

void up_bt_enable(int enable)
{
  if (enable)
    {
      lc823450_gpio_write(BT_POWER, 0);
      nxsig_usleep(100 * 1000);
      lc823450_gpio_write(BT_POWER, 1);
      nxsig_usleep(100 * 1000);
    }
  else
    {
      lc823450_gpio_write(BT_POWER, 0);
    }
}
