/****************************************************************************
 * arch/risc-v/src/common/riscv_tcbinfo.c
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

#ifdef CONFIG_DEBUG_TCBINFO

#include <nuttx/sched.h>
#include <arch/irq.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_reg_offs[] =
{
  UINT16_MAX,                       /* x0 */
  TCB_REG_OFF(REG_X1_NDX),
  TCB_REG_OFF(REG_X2_NDX),
  TCB_REG_OFF(REG_X3_NDX),
  TCB_REG_OFF(REG_X4_NDX),
  TCB_REG_OFF(REG_X5_NDX),
  TCB_REG_OFF(REG_X6_NDX),
  TCB_REG_OFF(REG_X7_NDX),
  TCB_REG_OFF(REG_X8_NDX),
  TCB_REG_OFF(REG_X9_NDX),
  TCB_REG_OFF(REG_X10_NDX),
  TCB_REG_OFF(REG_X11_NDX),
  TCB_REG_OFF(REG_X12_NDX),
  TCB_REG_OFF(REG_X13_NDX),
  TCB_REG_OFF(REG_X14_NDX),
  TCB_REG_OFF(REG_X15_NDX),
  TCB_REG_OFF(REG_X16_NDX),
  TCB_REG_OFF(REG_X17_NDX),
  TCB_REG_OFF(REG_X18_NDX),
  TCB_REG_OFF(REG_X19_NDX),
  TCB_REG_OFF(REG_X20_NDX),
  TCB_REG_OFF(REG_X21_NDX),
  TCB_REG_OFF(REG_X22_NDX),
  TCB_REG_OFF(REG_X23_NDX),
  TCB_REG_OFF(REG_X24_NDX),
  TCB_REG_OFF(REG_X25_NDX),
  TCB_REG_OFF(REG_X26_NDX),
  TCB_REG_OFF(REG_X27_NDX),
  TCB_REG_OFF(REG_X28_NDX),
  TCB_REG_OFF(REG_X29_NDX),
  TCB_REG_OFF(REG_X30_NDX),
  TCB_REG_OFF(REG_X31_NDX),
  TCB_REG_OFF(REG_EPC_NDX),

#ifdef CONFIG_ARCH_FPU
  TCB_REG_OFF(REG_F0_NDX),
  TCB_REG_OFF(REG_F1_NDX),
  TCB_REG_OFF(REG_F2_NDX),
  TCB_REG_OFF(REG_F3_NDX),
  TCB_REG_OFF(REG_F4_NDX),
  TCB_REG_OFF(REG_F5_NDX),
  TCB_REG_OFF(REG_F6_NDX),
  TCB_REG_OFF(REG_F7_NDX),
  TCB_REG_OFF(REG_F8_NDX),
  TCB_REG_OFF(REG_F9_NDX),
  TCB_REG_OFF(REG_F10_NDX),
  TCB_REG_OFF(REG_F11_NDX),
  TCB_REG_OFF(REG_F12_NDX),
  TCB_REG_OFF(REG_F13_NDX),
  TCB_REG_OFF(REG_F14_NDX),
  TCB_REG_OFF(REG_F15_NDX),
  TCB_REG_OFF(REG_F16_NDX),
  TCB_REG_OFF(REG_F17_NDX),
  TCB_REG_OFF(REG_F18_NDX),
  TCB_REG_OFF(REG_F19_NDX),
  TCB_REG_OFF(REG_F20_NDX),
  TCB_REG_OFF(REG_F21_NDX),
  TCB_REG_OFF(REG_F22_NDX),
  TCB_REG_OFF(REG_F23_NDX),
  TCB_REG_OFF(REG_F24_NDX),
  TCB_REG_OFF(REG_F25_NDX),
  TCB_REG_OFF(REG_F26_NDX),
  TCB_REG_OFF(REG_F27_NDX),
  TCB_REG_OFF(REG_F28_NDX),
  TCB_REG_OFF(REG_F29_NDX),
  TCB_REG_OFF(REG_F30_NDX),
  TCB_REG_OFF(REG_F31_NDX),
  UINT16_MAX,                      /* fflags */
  UINT16_MAX,                      /* frm */
  TCB_REG_OFF(REG_FCSR_NDX),
#endif
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
  .basic_num = 33,
  .total_num = sizeof(g_reg_offs) / sizeof(uint16_t),
  {
    .p = g_reg_offs,
  },
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

