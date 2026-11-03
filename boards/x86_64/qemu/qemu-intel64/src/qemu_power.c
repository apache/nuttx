/****************************************************************************
 * boards/x86_64/qemu/qemu-intel64/src/qemu_power.c
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

#include <nuttx/board.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power off the board.  Support for this function is required by
 *   board-level logic if CONFIG_BOARDCTL_POWEROFF is selected.  QEMU
 *   stops when the ACPI sleep state S5 is entered (up_systempoweroff()).
 *
 * Input Parameters:
 *   status - Status information provided with the power off event.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value in this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

int board_power_off(int status)
{
  up_systempoweroff();
  return 0;
}
