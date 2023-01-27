/****************************************************************************
 * arch/arm/src/armv7-a/arm_tcbinfo.c
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
  TCB_REG_OFF(REG_R0),
  TCB_REG_OFF(REG_R1),
  TCB_REG_OFF(REG_R2),
  TCB_REG_OFF(REG_R3),
  TCB_REG_OFF(REG_R4),
  TCB_REG_OFF(REG_R5),
  TCB_REG_OFF(REG_R6),
  TCB_REG_OFF(REG_R7),
  TCB_REG_OFF(REG_R8),
  TCB_REG_OFF(REG_R9),
  TCB_REG_OFF(REG_R10),
  TCB_REG_OFF(REG_R11),
  TCB_REG_OFF(REG_R12),
  TCB_REG_OFF(REG_R13),
  TCB_REG_OFF(REG_R14),
  TCB_REG_OFF(REG_R15),
  TCB_REG_OFF(REG_CPSR),

#ifdef CONFIG_ARCH_FPU
  TCB_REG_OFF(REG_D0),
  TCB_REG_OFF(REG_D1),
  TCB_REG_OFF(REG_D2),
  TCB_REG_OFF(REG_D3),
  TCB_REG_OFF(REG_D4),
  TCB_REG_OFF(REG_D5),
  TCB_REG_OFF(REG_D6),
  TCB_REG_OFF(REG_D7),
  TCB_REG_OFF(REG_D8),
  TCB_REG_OFF(REG_D9),
  TCB_REG_OFF(REG_D10),
  TCB_REG_OFF(REG_D11),
  TCB_REG_OFF(REG_D12),
  TCB_REG_OFF(REG_D13),
  TCB_REG_OFF(REG_D14),
  TCB_REG_OFF(REG_D15),
#endif

#ifdef CONFIG_ARM_DPFPU32
  TCB_REG_OFF(REG_D16),
  TCB_REG_OFF(REG_D17),
  TCB_REG_OFF(REG_D18),
  TCB_REG_OFF(REG_D19),
  TCB_REG_OFF(REG_D20),
  TCB_REG_OFF(REG_D21),
  TCB_REG_OFF(REG_D22),
  TCB_REG_OFF(REG_D23),
  TCB_REG_OFF(REG_D24),
  TCB_REG_OFF(REG_D25),
  TCB_REG_OFF(REG_D26),
  TCB_REG_OFF(REG_D27),
  TCB_REG_OFF(REG_D28),
  TCB_REG_OFF(REG_D29),
  TCB_REG_OFF(REG_D30),
  TCB_REG_OFF(REG_D31),
#endif

#ifdef CONFIG_ARCH_FPU
  TCB_REG_OFF(REG_FPSCR),
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
  .basic_num = 17,
  .total_num = sizeof(g_reg_offs) / sizeof(g_reg_offs[0]),
  {
    .p = g_reg_offs,
  },
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

