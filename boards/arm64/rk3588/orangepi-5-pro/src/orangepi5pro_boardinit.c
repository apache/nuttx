/****************************************************************************
 * boards/arm64/rk3588/orangepi-5-pro/src/orangepi5pro_boardinit.c
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

#include <nuttx/board.h>

#include "orangepi5pro.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rk3588_memory_initialize(void)
{
  /* DRAM is trained and initialised by the boot firmware before NuttX
   * receives control on the Orange Pi 5 Pro.
   */
}

void rk3588_board_initialize(void)
{
  /* No early board initialization is needed for this configuration. */
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  orangepi5pro_bringup();
}
#endif
