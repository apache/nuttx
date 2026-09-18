/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/src/nrf54l_boot.c
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

#include <stdint.h>

#include <nuttx/board.h>

#include "nrf54l_start.h"
#include "nrf54l15-tag.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_board_initialize
 *
 * Description:
 *   Initialize board hardware before starting the operating system.
 *
 ****************************************************************************/

void nrf54l_board_initialize(void)
{
#ifdef CONFIG_NRF54L_SPI2_MASTER
  nrf54l_spidev_initialize();
#endif

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   Initialize board devices after the operating system is available.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
  nrf54l_bringup();
}
#endif
