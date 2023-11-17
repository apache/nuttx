/****************************************************************************
 * arch/x86_64/src/common/x86_64_tcbinfo.c
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
  TCB_REG_OFF(REG_RAX),    /* RAX */
  TCB_REG_OFF(REG_RBX),    /* RBX */
  TCB_REG_OFF(REG_RCX),    /* RCX */
  TCB_REG_OFF(REG_RDX),    /* RDX */
  TCB_REG_OFF(REG_RSI),    /* RSI */
  TCB_REG_OFF(REG_RDI),    /* RDI */
  TCB_REG_OFF(REG_RBP),    /* RBP */
  TCB_REG_OFF(REG_RSP),    /* RSP */
  TCB_REG_OFF(REG_R8),     /* R8 */
  TCB_REG_OFF(REG_R9),     /* R9 */
  TCB_REG_OFF(REG_R10),    /* R10 */
  TCB_REG_OFF(REG_R11),    /* R11 */
  TCB_REG_OFF(REG_R12),    /* R12 */
  TCB_REG_OFF(REG_R13),    /* R13 */
  TCB_REG_OFF(REG_R14),    /* R14 */
  TCB_REG_OFF(REG_R15),    /* R15 */
  TCB_REG_OFF(REG_RIP),    /* RIP */
  TCB_REG_OFF(REG_RFLAGS), /* EFLAGS */
  TCB_REG_OFF(REG_CS),     /* CS */
  TCB_REG_OFF(REG_SS),     /* SS */
  TCB_REG_OFF(REG_DS),     /* DS */
  TCB_REG_OFF(REG_ES),     /* ES */
  TCB_REG_OFF(REG_FS),     /* FS */
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
