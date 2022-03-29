/****************************************************************************
 * arch/xtensa/src/common/xtensa_tcbinfo.c
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_reg_offs[] =
{
  TCB_REG_OFF(REG_PC),
  TCB_REG_OFF(REG_PS),
  TCB_REG_OFF(REG_A0),
  TCB_REG_OFF(REG_A1),
  TCB_REG_OFF(REG_A2),
  TCB_REG_OFF(REG_A3),
  TCB_REG_OFF(REG_A4),
  TCB_REG_OFF(REG_A5),
  TCB_REG_OFF(REG_A6),
  TCB_REG_OFF(REG_A7),
  TCB_REG_OFF(REG_A8),
  TCB_REG_OFF(REG_A9),
  TCB_REG_OFF(REG_A10),
  TCB_REG_OFF(REG_A11),
  TCB_REG_OFF(REG_A12),
  TCB_REG_OFF(REG_A13),
  TCB_REG_OFF(REG_A14),
  TCB_REG_OFF(REG_A15),
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct tcbinfo_s g_tcbinfo =
{
  .pid_off   = TCB_PID_OFF,
  .state_off = TCB_STATE_OFF,
  .pri_off   = TCB_PRI_OFF,
  .name_off  = TCB_NAME_OFF,
  .regs_off  = TCB_REGS_OFF,
  .basic_num = XCPTCONTEXT_REGS,
  .total_num = XCPTCONTEXT_REGS,
  {
    .p = g_reg_offs,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

