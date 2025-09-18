/****************************************************************************
 * arch/tricore/src/common/tricore_tcbinfo.c
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

#include <nuttx/sched.h>
#include <arch/irq.h>
#include <sys/param.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_reg_offs[] =
{
  TCB_REG_OFF(16 + REG_D0),
  TCB_REG_OFF(16 + REG_D1),
  TCB_REG_OFF(16 + REG_D2),
  TCB_REG_OFF(16 + REG_D3),
  TCB_REG_OFF(16 + REG_D4),
  TCB_REG_OFF(16 + REG_D5),
  TCB_REG_OFF(16 + REG_D6),
  TCB_REG_OFF(16 + REG_D7),
  TCB_REG_OFF(REG_D8),
  TCB_REG_OFF(REG_D9),
  TCB_REG_OFF(REG_D10),
  TCB_REG_OFF(REG_D11),
  TCB_REG_OFF(REG_D12),
  TCB_REG_OFF(REG_D13),
  TCB_REG_OFF(REG_D14),
  TCB_REG_OFF(REG_D15),
  UINT16_MAX,
  UINT16_MAX,
  TCB_REG_OFF(16 + REG_A2),
  TCB_REG_OFF(16 + REG_A3),
  TCB_REG_OFF(16 + REG_A4),
  TCB_REG_OFF(16 + REG_A5),
  TCB_REG_OFF(16 + REG_A6),
  TCB_REG_OFF(16 + REG_A7),
  UINT16_MAX,
  UINT16_MAX,
  TCB_REG_OFF(REG_A10),
  TCB_REG_OFF(REG_UA11),
  TCB_REG_OFF(REG_A12),
  TCB_REG_OFF(REG_A13),
  TCB_REG_OFF(REG_A14),
  TCB_REG_OFF(REG_A15),
  UINT16_MAX,
  UINT16_MAX,
  TCB_REG_OFF(REG_UPCXI),
  TCB_REG_OFF(REG_PSW),
  TCB_REG_OFF(16 + REG_LA11),
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct tcbinfo_s g_tcbinfo used_data =
{
  .pid_off        = TCB_PID_OFF,
  .state_off      = TCB_STATE_OFF,
  .pri_off        = TCB_PRI_OFF,
  .name_off       = TCB_NAME_OFF,
  .stack_off      = TCB_STACK_OFF,
  .stack_size_off = TCB_STACK_SIZE_OFF,
  .regs_off       = TCB_REGS_OFF,
  .regs_num       = nitems(g_reg_offs),
  {
    .p = g_reg_offs,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
