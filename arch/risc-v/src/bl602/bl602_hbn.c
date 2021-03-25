/****************************************************************************
 * arch/risc-v/src/bl602/bl602_hbn.c
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

#include "hardware/bl602_hbn.h"
#include "riscv_arch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_hbn_set_uart_clk_sel
 *
 * Description:
 *   Select uart clock source.
 *
 * Input Parameters:
 *   clk_sel: uart clock type selection, 0 for FCLK or 1 for 160MHz CLK
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

void bl602_set_uart_clk_sel(int clk_sel)
{
  if (clk_sel)
    {
      modifyreg32(BL602_HBN_GLB, 0, HBN_GLB_HBN_UART_CLK_SEL);
    }
  else
    {
      modifyreg32(BL602_HBN_GLB, HBN_GLB_HBN_UART_CLK_SEL, 0);
    }
}

void bl602_aon_pad_iesmt_cfg(uint8_t pad_cfg)
{
  modifyreg32(BL602_HBN_IRQ_MODE, HBN_IRQ_MODE_REG_AON_PAD_IE_SMT,
      pad_cfg << 8);
}
