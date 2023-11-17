/****************************************************************************
 * arch/sim/src/sim/sim_tcbinfo.c
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

#if defined(CONFIG_HOST_X86_64) && !defined(CONFIG_SIM_M32)
static const uint16_t g_reg_offs[] =
{
  UINT16_MAX,            /* RAX */
  TCB_REG_OFF(JB_RBX),   /* RBX */
  UINT16_MAX,            /* RCX */
  UINT16_MAX,            /* RDX */
  UINT16_MAX,            /* RSI */
  UINT16_MAX,            /* RDI */
  TCB_REG_OFF(JB_RBP),   /* RBP */
  TCB_REG_OFF(JB_RSP),   /* RSP */
  UINT16_MAX,            /* R8 */
  UINT16_MAX,            /* R9 */
  UINT16_MAX,            /* R10 */
  UINT16_MAX,            /* R11 */
  TCB_REG_OFF(JB_R12),   /* R12 */
  TCB_REG_OFF(JB_R13),   /* R13 */
  TCB_REG_OFF(JB_R15),   /* R14 */
  TCB_REG_OFF(JB_R15),   /* R15 */
  TCB_REG_OFF(JB_RIP),   /* RIP */
  UINT16_MAX,            /* EFLAGS */
  UINT16_MAX,            /* CS */
  UINT16_MAX,            /* SS */
  UINT16_MAX,            /* DS */
  UINT16_MAX,            /* ES */
  UINT16_MAX,            /* FS */
};
#elif defined(CONFIG_HOST_X86) || defined(CONFIG_SIM_M32)
static const uint16_t g_reg_offs[] =
{
  UINT16_MAX,            /* RAX */
  UINT16_MAX,            /* RCX */
  UINT16_MAX,            /* RDX */
  TCB_REG_OFF(JB_EBX),   /* RBX */
  TCB_REG_OFF(JB_ESP),   /* ESP */
  TCB_REG_OFF(JB_EBP),   /* EBP */
  TCB_REG_OFF(JB_ESI),   /* ESI */
  TCB_REG_OFF(JB_EDI),   /* EDI */
  TCB_REG_OFF(JB_EIP),   /* EIP */
  UINT16_MAX,            /* EFLAGS */
  UINT16_MAX,            /* CS */
  UINT16_MAX,            /* SS */
  UINT16_MAX,            /* DS */
  UINT16_MAX,            /* ES */
  UINT16_MAX,            /* FS */
};
#elif defined(CONFIG_HOST_ARM64)
static const uint16_t g_reg_offs[] =
{
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  TCB_REG_OFF(JB_SP),
  TCB_REG_OFF(JB_PC),
  UINT16_MAX,
  UINT16_MAX,
};
#elif defined(CONFIG_HOST_ARM)
static const uint16_t g_reg_offs[] =
{
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
  UINT16_MAX,
};
#endif

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
