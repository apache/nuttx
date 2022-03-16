/****************************************************************************
 * arch/arm/src/eoss3/eoss3_clockconfig.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "eoss3.h"
#include "hardware/eoss3_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: eoss3_clockconfig
 *
 * Description:
 *   Called to initialize the EOS S3.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void eoss3_clockconfig(void)
{
  uint32_t clk_cfg;
  uint8_t check_cnt;

  /* Enable the OSC clock source */

  putreg32(AIP_OSC_CTRL_0_EN, EOSS3_AIP_OSC_CTRL_0);

  /* Set the frequency 79.79MHz (Update to use BOARD clock define */

  clk_cfg = getreg32(EOSS3_AIP_OSC_CTRL_1);
  clk_cfg &= ~AIP_OSC_CTRL_1_PROG_MASK;
  clk_cfg |= 0x980 << AIP_OSC_CTRL_1_PROG_SHIFT;  /* (prog + 3) ∗ 32,768Hz */
  putreg32(clk_cfg, EOSS3_AIP_OSC_CTRL_1);

  /* Wait for the lock, we need to wait for lock twice
   * This can be disabled for the emulator to function since it does not
   * implement the lock register and it will forever spin here.
   */

#if 1
  for (check_cnt = 0; check_cnt < 2; check_cnt++)
    {
      while ((getreg32(EOSS3_AIP_OSC_STA_0) & AIP_OSC_STA_0_LOCK) == 0);
    }
#endif

  /* Configure the M4 Clock. Directly connect to HFCLK for 79.79MHz */

  putreg32(0, EOSS3_CLK_CONTROL_A_1);  /* Use OSC */
  putreg32(0, EOSS3_CLK_CONTROL_A_0);  /* Disable, Output Directly */
  putreg32(0x5f, EOSS3_CLK_C10_GATE);  /* Enable Gates [6, 4:0] */

  /* Need to setup M4 peripheral clocks (UART, Timer, Watchdog)
   * CLK_SWITCH_FOR_D = 0
   * CLK_Control_D_0 = 0x206 (divide 8 [8-2=6] + enable)
   * MISC_LOCK_KEY_CTRL = 0x1acce551  (re-lock by writing any other val)
   * C11_CLK_GATE = 1
   */

  putreg32(0, EOSS3_CLK_SWITCH_FOR_D);
  putreg32((8 - 2) | (1 << 9), EOSS3_CLK_CONTROL_D_0);
  putreg32(MISC_LOCK_KEY_CTRL_UNLOCK, MISC_LOCK_KEY_CTRL);
  putreg32(1, EOSS3_CLK_C11_GATE);

#if 0
  /* Enable clock debug logic C11 is brought out to pad 13 */

  putreg32(0, 0x40005004);
  putreg32(1, 0x40005008);
  putreg32(2, 0x40004c34);
  putreg32(8, 0x40004108);
#endif
}
