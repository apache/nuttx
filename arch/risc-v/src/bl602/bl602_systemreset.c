/****************************************************************************
 * arch/risc-v/src/bl602/bl602_systemreset.c
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
#include "riscv_arch.h"

#include "hardware/bl602_glb.h"
#include "hardware/bl602_hbn.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_chip_reset
 *
 * Description:
 *   Control the different reset modes
 *
 * Input Parameters:
 *   mask - Reset bitmask use these defines
 *          SWRST_CFG2_CTRL_SYS_RESET, SWRST_CFG2_CTRL_CPU_RESET,
 *          SWRST_CFG2_CTRL_PWRON_RST
 *
 ****************************************************************************/

static void bl602_chip_reset(uint32_t mask)
{
  /* Reset the root clock */

  modifyreg32(BL602_HBN_GLB, HBN_GLB_HBN_ROOT_CLK_SEL_MASK, 0);

  /* Clear root clock dividers */

  modifyreg32(
      BL602_CLK_CFG0,
      CLK_CFG0_REG_BCLK_DIV_MASK | CLK_CFG0_REG_HCLK_DIV_MASK,
      0
  );

  /* This register should toggled on-off on changes to root clock.
   * details of this register are not documented, but is clear from ROM
   */

  putreg32(1, 0x40000ffc);
  putreg32(0, 0x40000ffc);

  /* Trigger reset
   * NOTE: The reset seems to be rising _edge_ triggered so the reset
   *       bit should be cleared first otherwise the reset will not
   *       trigger if it has previously fired.
   */

  modifyreg32(
      BL602_SWRST_CFG2,
      (SWRST_CFG2_CTRL_SYS_RESET | SWRST_CFG2_CTRL_CPU_RESET | \
       SWRST_CFG2_CTRL_PWRON_RST),
      0
  );

  modifyreg32(
      BL602_SWRST_CFG2,
      (SWRST_CFG2_CTRL_SYS_RESET | SWRST_CFG2_CTRL_CPU_RESET | \
       SWRST_CFG2_CTRL_PWRON_RST),
      mask
  );

  /* Wait for the reset */

  for (; ; );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  bl602_chip_reset(SWRST_CFG2_CTRL_SYS_RESET | SWRST_CFG2_CTRL_CPU_RESET);
}

/****************************************************************************
 * Name: bl602_cpu_reset
 *
 * Description:
 *   Reset only the CPU
 *
 ****************************************************************************/

void bl602_cpu_reset(void)
{
  bl602_chip_reset(SWRST_CFG2_CTRL_CPU_RESET);
}

/****************************************************************************
 * Name: bl602_por_reset
 *
 * Description:
 *   Trigger Power-on-Reset
 *
 ****************************************************************************/

void bl602_por_reset(void)
{
  bl602_chip_reset(
    SWRST_CFG2_CTRL_SYS_RESET | \
    SWRST_CFG2_CTRL_CPU_RESET | \
    SWRST_CFG2_CTRL_PWRON_RST);
}
