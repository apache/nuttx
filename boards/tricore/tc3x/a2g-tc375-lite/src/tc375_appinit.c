/****************************************************************************
 * boards/tricore/tc3x/a2g-tc375-lite/src/tc375_appinit.c
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

#include "tricore_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_aurix_setup_serial_pin(int idx)
{
  if (idx != 0)
    {
      return;
    }

  aurix_config_gpio(AURIX_GPIO(14, 0, GPIO_PERIPH, GPIO_ALT2));
  aurix_config_gpio(AURIX_GPIO(14, 1, GPIO_INPUT, GPIO_PULL_UP));
}

int board_app_initialize(uintptr_t arg)
{
  return OK;
}
