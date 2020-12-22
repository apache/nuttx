/****************************************************************************
 * arch/risc-v/src/bl602/bl602_hbn.c
 *
 * Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 * Author: Gregory Nutt <gnutt@nuttx.org>
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
 *   clk_sel: uart clock type selection
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

void bl602_hbn_set_uart_clk_sel(int clk_sel)
{
  uint32_t tmp_val;

  tmp_val = getreg32(HBN_BASE + HBN_GLB_OFFSET);
  tmp_val &= ~(1 << 2);
  tmp_val |= (clk_sel << 2);
  putreg32(tmp_val, HBN_BASE + HBN_GLB_OFFSET);
}
