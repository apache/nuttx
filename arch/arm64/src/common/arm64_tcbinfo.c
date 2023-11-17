/****************************************************************************
 * arch/arm64/src/common/arm64_tcbinfo.c
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
  TCB_REG_OFF(REG_X0),
  TCB_REG_OFF(REG_X1),
  TCB_REG_OFF(REG_X2),
  TCB_REG_OFF(REG_X3),
  TCB_REG_OFF(REG_X4),
  TCB_REG_OFF(REG_X5),
  TCB_REG_OFF(REG_X6),
  TCB_REG_OFF(REG_X7),
  TCB_REG_OFF(REG_X8),
  TCB_REG_OFF(REG_X9),
  TCB_REG_OFF(REG_X10),
  TCB_REG_OFF(REG_X11),
  TCB_REG_OFF(REG_X12),
  TCB_REG_OFF(REG_X13),
  TCB_REG_OFF(REG_X14),
  TCB_REG_OFF(REG_X15),
  TCB_REG_OFF(REG_X16),
  TCB_REG_OFF(REG_X17),
  TCB_REG_OFF(REG_X18),
  TCB_REG_OFF(REG_X19),
  TCB_REG_OFF(REG_X20),
  TCB_REG_OFF(REG_X21),
  TCB_REG_OFF(REG_X22),
  TCB_REG_OFF(REG_X23),
  TCB_REG_OFF(REG_X24),
  TCB_REG_OFF(REG_X25),
  TCB_REG_OFF(REG_X26),
  TCB_REG_OFF(REG_X27),
  TCB_REG_OFF(REG_X28),
  TCB_REG_OFF(REG_X29),
  TCB_REG_OFF(REG_X30),
  TCB_REG_OFF(REG_SP_ELX),
  TCB_REG_OFF(REG_ELR),
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct tcbinfo_s g_tcbinfo =
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
