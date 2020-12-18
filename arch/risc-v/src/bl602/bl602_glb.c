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

#include "hardware/bl602_glb.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: glb_uart_fun_sel
 *
 * Description:
 *   Select UART signal function.
 *
 * Input Parameters:
 *   sig: UART signal
 *   fun: UART function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void glb_uart_fun_sel(enum glb_uart_sig_e sig, enum glb_uart_sig_fun_e fun)
{
  uint32_t sig_pos = 0;
  uint32_t tmp_val = 0;

  tmp_val = BL_RD_REG(GLB_BASE, GLB_UART_SIG_SEL_0);
  sig_pos = (sig * 4);

  /* Clear original val */

  tmp_val = tmp_val & (~(0xf << sig_pos));

  /* Set new value */

  tmp_val = tmp_val | (fun << sig_pos);
  BL_WR_REG(GLB_BASE, GLB_UART_SIG_SEL_0, tmp_val);
}

/****************************************************************************
 * Name: glb_ahb_slave1_reset
 *
 * Description:
 *   Select UART signal function.
 *
 * Input Parameters:
 *   sig: UART signal
 *   fun: UART function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void glb_ahb_slave1_reset(enum bl_ahb_slave1_e slave1)
{
  uint32_t tmp_val = 0;

  tmp_val = BL_RD_REG(GLB_BASE, GLB_SWRST_CFG1);
  tmp_val &= (~(1 << slave1));
  BL_WR_REG(GLB_BASE, GLB_SWRST_CFG1, tmp_val);
  BL_DRV_DUMMY;
  tmp_val = BL_RD_REG(GLB_BASE, GLB_SWRST_CFG1);
  tmp_val |= (1 << slave1);
  BL_WR_REG(GLB_BASE, GLB_SWRST_CFG1, tmp_val);
  BL_DRV_DUMMY;
  tmp_val = BL_RD_REG(GLB_BASE, GLB_SWRST_CFG1);
  tmp_val &= (~(1 << slave1));
  BL_WR_REG(GLB_BASE, GLB_SWRST_CFG1, tmp_val);
}

