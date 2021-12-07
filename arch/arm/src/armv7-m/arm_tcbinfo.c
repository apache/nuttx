/****************************************************************************
 * arch/arm/src/armv7-m/arm_tcbinfo.c
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
 * Public Data
 ****************************************************************************/

const struct tcbinfo_s g_tcbinfo =
{
  TCB_PID_OFF,
  TCB_STATE_OFF,
  TCB_PRI_OFF,
#if CONFIG_TASK_NAME_SIZE > 0
  TCB_NAME_OFF,
#endif

  XCPTCONTEXT_REGS,

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

  0,
  TCB_REG_OFF(REG_R13),
#ifdef CONFIG_ARMV7M_USEBASEPRI
  0,
  TCB_REG_OFF(REG_BASEPRI),
#else
  TCB_REG_OFF(REG_PRIMASK),
  0,
#endif
  0,
  0,

#ifdef CONFIG_ARCH_FPU
  TCB_REG_OFF(REG_S0),
  TCB_REG_OFF(REG_S1),
  TCB_REG_OFF(REG_S2),
  TCB_REG_OFF(REG_S3),
  TCB_REG_OFF(REG_S4),
  TCB_REG_OFF(REG_S5),
  TCB_REG_OFF(REG_S6),
  TCB_REG_OFF(REG_S7),
  TCB_REG_OFF(REG_S8),
  TCB_REG_OFF(REG_S9),
  TCB_REG_OFF(REG_S10),
  TCB_REG_OFF(REG_S11),
  TCB_REG_OFF(REG_S12),
  TCB_REG_OFF(REG_S13),
  TCB_REG_OFF(REG_S14),
  TCB_REG_OFF(REG_S15),
  TCB_REG_OFF(REG_S16),
  TCB_REG_OFF(REG_S17),
  TCB_REG_OFF(REG_S18),
  TCB_REG_OFF(REG_S19),
  TCB_REG_OFF(REG_S20),
  TCB_REG_OFF(REG_S21),
  TCB_REG_OFF(REG_S22),
  TCB_REG_OFF(REG_S23),
  TCB_REG_OFF(REG_S24),
  TCB_REG_OFF(REG_S25),
  TCB_REG_OFF(REG_S26),
  TCB_REG_OFF(REG_S27),
  TCB_REG_OFF(REG_S28),
  TCB_REG_OFF(REG_S29),
  TCB_REG_OFF(REG_S30),
  TCB_REG_OFF(REG_S31),
  0,
  TCB_REG_OFF(REG_FPSCR),
  0,
#endif
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

