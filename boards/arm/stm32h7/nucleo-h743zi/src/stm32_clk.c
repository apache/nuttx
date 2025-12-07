/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_clk.c
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

#include <stddef.h>
#include <assert.h>
#include <nuttx/clk/clk.h>
#include <arch/board/board.h>

void up_clk_initialize(void)
{
  struct clk_s *clk;
  struct clk_s *parent_clk;

  clk = clk_get("pll1_q_ck");
  VERIFY(clk_disable(clk));

  clk = clk_get("pllsrc");
  parent_clk = clk_get_parent_by_index(clk, 2);
  VERIFY(clk_set_parent(clk, parent_clk));

  clk = clk_get("sys_ck");
  parent_clk = clk_get_parent_by_index(clk, 3);
  VERIFY(clk_set_parent(clk, parent_clk));

  clk = clk_get("divm1");
  VERIFY(clk_set_rate(clk, 4000000));

  clk = clk_get("pll1_p_ck");
  VERIFY(clk_set_rate(clk, 160000000));
  VERIFY(clk_enable(clk));

  clk = clk_get("gpioa_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpiob_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpioc_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpiod_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpioe_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpiof_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpiog_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpioh_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpioi_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpioj_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("gpiok_ker_ck");
  VERIFY(clk_enable(clk));

  clk = clk_get("usart234578sel");
  parent_clk = clk_get_parent_by_index(clk, 3);
  VERIFY(clk_set_parent(clk, parent_clk));
}