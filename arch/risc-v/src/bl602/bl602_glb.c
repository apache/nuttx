/****************************************************************************
 * arch/risc-v/src/bl602/bl602_glb.c
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
#include "riscv_internal.h"
#include "hardware/bl602_glb.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#define nop() asm volatile("nop")

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_swrst_ahb_slave1
 *
 * Description:
 *   SW Reset ahb slave.
 *
 * Input Parameters:
 *   slave1: reset signal
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_swrst_ahb_slave1(uint32_t slave1)
{
  /* To prevent glitch from accessing bus immediately
   * after certain register operations, so some nop delay is added
   */

  modifyreg32(BL602_SWRST_CFG1, slave1, 0);
  nop();
  nop();
  nop();
  nop();
  modifyreg32(BL602_SWRST_CFG1, 0, slave1);
  nop();
  nop();
  nop();
  nop();
  modifyreg32(BL602_SWRST_CFG1, slave1, 0);
}

/****************************************************************************
 * Name: bl602_glb_get_bclk_div
 *
 * Description:
 *   get bus clock div.
 *
 * Input Parameters:
 *   void
 *
 * Returned Value:
 *   bus clock div
 *
 ****************************************************************************/

uint8_t bl602_glb_get_bclk_div(void)
{
  uint32_t tmp_val;

  tmp_val = getreg32(BL602_CLK_CFG0);
  tmp_val =
    (CLK_CFG0_REG_BCLK_DIV_MASK & tmp_val) >> CLK_CFG0_REG_BCLK_DIV_SHIFT;

  return (uint8_t)tmp_val;
}

/****************************************************************************
 * Name: bl602_set_em_sel
 *
 * Description:
 *   Set how much wifi ram is allocated to ble.
 *
 * Input Parameters:
 *   em_type: memory size type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_set_em_sel(int em_type)
{
  modifyreg32(BL602_SEAM_MISC, SEAM_MISC_EM_SEL_MASK, em_type);
}
