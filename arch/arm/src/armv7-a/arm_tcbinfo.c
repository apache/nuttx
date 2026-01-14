/****************************************************************************
 * arch/arm/src/armv7-a/arm_tcbinfo.c
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
  TCB_REG_OFF(REG_R0),   /* 0 */
  TCB_REG_OFF(REG_R1),   /* 1 */
  TCB_REG_OFF(REG_R2),   /* 2 */
  TCB_REG_OFF(REG_R3),   /* 3 */
  TCB_REG_OFF(REG_R4),   /* 4 */
  TCB_REG_OFF(REG_R5),   /* 5 */
  TCB_REG_OFF(REG_R6),   /* 6 */
  TCB_REG_OFF(REG_R7),   /* 7 */
  TCB_REG_OFF(REG_R8),   /* 8 */
  TCB_REG_OFF(REG_R9),   /* 9 */
  TCB_REG_OFF(REG_R10),  /* 10 */
  TCB_REG_OFF(REG_R11),  /* 11 */
  TCB_REG_OFF(REG_R12),  /* 12 */
  TCB_REG_OFF(REG_R13),  /* 13 */
  TCB_REG_OFF(REG_R14),  /* 14 */
  TCB_REG_OFF(REG_R15),  /* 15 */
  UINT16_MAX,            /* 16 */
  UINT16_MAX,            /* 17 */
  UINT16_MAX,            /* 18 */
  UINT16_MAX,            /* 19 */
  UINT16_MAX,            /* 20 */
  UINT16_MAX,            /* 21 */
  UINT16_MAX,            /* 22 */
  UINT16_MAX,            /* 23 */
  UINT16_MAX,            /* 24 */
  UINT16_MAX,            /* 25 */
  UINT16_MAX,            /* 26 */
  UINT16_MAX,            /* 27 */
  UINT16_MAX,            /* 28 */
  UINT16_MAX,            /* 29 */
  UINT16_MAX,            /* 30 */
  UINT16_MAX,            /* 31 */
  UINT16_MAX,            /* 32 */
  UINT16_MAX,            /* 33 */
  UINT16_MAX,            /* 34 */
  UINT16_MAX,            /* 35 */
  UINT16_MAX,            /* 36 */
  UINT16_MAX,            /* 37 */
  UINT16_MAX,            /* 38 */
  UINT16_MAX,            /* 39 */
  UINT16_MAX,            /* 40 */
  TCB_REG_OFF(REG_CPSR), /* 41 */

#if 0
#  ifdef CONFIG_ARCH_FPU
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
#  endif

#  ifdef CONFIG_ARM_DPFPU32
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
#  endif

#  ifdef CONFIG_ARCH_FPU
  TCB_REG_OFF(REG_FPSCR),
#  endif
#endif
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
