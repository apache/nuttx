/****************************************************************************
 * boards/risc-v/gd32vw55x/gd32vw553k-start/src/gd32_reset.c
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

#include "riscv_internal.h"
#include "gd32vw55x_memorymap.h"

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Nuclei SysTimer carries the software reset request register.  Writing
 * the key asks the SoC for a reset; the core is reset a few cycles later,
 * so the write never returns.
 */

#define GD32VW55X_SYSTIMER_MSFTRST  (GD32VW55X_SYSTIMER_BASE + 0x0ff0)
#define SYSTIMER_MSFTRST_KEY        0x80000a5f

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset the board.  Called by the "reset" NSH command and by
 *   boardctl(BOARDIOC_RESET).
 *
 * Input Parameters:
 *   status - Reset status, ignored here.
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

int board_reset(int status)
{
  putreg32(SYSTIMER_MSFTRST_KEY, GD32VW55X_SYSTIMER_MSFTRST);

  for (; ; )
    {
      /* Wait for the reset to arrive */
    }

  return 0;
}

#endif /* CONFIG_BOARDCTL_RESET */
