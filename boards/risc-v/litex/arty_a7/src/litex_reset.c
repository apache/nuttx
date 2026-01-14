/****************************************************************************
 * boards/risc-v/litex/arty_a7/src/litex_reset.c
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

#include <assert.h>
#include <nuttx/board.h>
#include <nuttx/arch.h>

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  Support for this function is required by board-level
 *   logic if CONFIG_BOARDCTL_RESET is selected.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *            meaning of this status information is board-specific.  If not
 *            used by a board, the value zero may be provided in calls to
 *            board_reset().
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value in this case is a
 *   board-specific reason for the failure to shutdown.
 *
 * Additionally: This function is called from assert if
 *  CONFIG_BOARD_RESET_ON_ASSERT >= 1. In this case status is set to
 *  CONFIG_BOARD_ASSERT_RESET_VALUE
 *
 ****************************************************************************/

int board_reset(int status)
{
  switch (status)
    {
      case CONFIG_BOARD_ASSERT_RESET_VALUE:
      default:
        up_systemreset();
    }

  /* up_systemreset can't actually return. However, let's try to catch the
   * case that this changes in the future, or the logic is extended to a case
   * which can fail.
  */

  return -ENODEV;
}

#endif /* CONFIG_BOARDCTL_RESET */
