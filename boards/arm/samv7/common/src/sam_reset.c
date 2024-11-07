/****************************************************************************
 * boards/arm/samv7/common/src/sam_reset.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "sam_systemreset.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset_cause
 *
 * Description:
 *   Get the cause of last board reset. This should call architecture
 *   specific logic to handle the register read.
 *
 * Input Parameters:
 *   cause - Pointer to boardioc_reset_cause_s structure to which the
 *      reason (and potentially subreason) is saved.
 *
 * Returned Value:
 *   This functions should always return succesfully with 0. We save
 *   BOARDIOC_RESETCAUSE_UNKOWN in cause structure if we are
 *   not able to get last reset cause from HW (which is unlikely).
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
int board_reset_cause(struct boardioc_reset_cause_s *cause)
{
  int rst_cause;

  /* Get the reset cause from hardware */

  rst_cause = sam_get_reset_cause();

  switch (rst_cause)
    {
      case SAMV7_RESET_PWRUP:

        /* Power up */

        cause->cause = BOARDIOC_RESETCAUSE_SYS_CHIPPOR;
        break;
      case SAMV7_RESET_BACKUP:

        /* Wake up from backup (low power) mode */

        cause->cause = BOARDIOC_RESETCAUSE_LOWPOWER;
        break;
      case SAMV7_RESET_WDOG:

        /* Watchdog error */

        cause->cause = BOARDIOC_RESETCAUSE_CPU_RWDT;
        break;
      case SAMV7_RESET_SWRST:

        /* SW reset */

        cause->cause = BOARDIOC_RESETCAUSE_CPU_SOFT;
        break;
      case SAMV7_RESET_NRST:

        /* Reset from user by pressing reset button */

        cause->cause = BOARDIOC_RESETCAUSE_PIN;
        break;
      default:

        /* Unknown cause returned from HW */

        cause->cause = BOARDIOC_RESETCAUSE_UNKOWN;
        break;
    }

  return 0;
}
#endif /* CONFIG_BOARDCTL_RESET_CAUSE */

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
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status)
{
  up_systemreset();
  return 0;
}
#endif /* CONFIG_BOARDCTL_RESET */
