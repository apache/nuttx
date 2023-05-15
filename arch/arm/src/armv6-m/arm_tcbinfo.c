/****************************************************************************
 * arch/arm/src/armv6-m/arm_tcbinfo.c
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
  TCB_REG_OFF(REG_XPSR),

  UINT16_MAX,                     /* msp */
  TCB_REG_OFF(REG_R13),
  TCB_REG_OFF(REG_PRIMASK),
  UINT16_MAX,                     /* basepri */
  UINT16_MAX,                     /* faultmask */
  UINT16_MAX,                     /* control */
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
  .basic_num      = 17,
  .total_num      = sizeof(g_reg_offs) / sizeof(g_reg_offs[0]),
  {
    .p = g_reg_offs,
  },
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

