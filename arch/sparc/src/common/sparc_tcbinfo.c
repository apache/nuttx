/****************************************************************************
 * arch/sparc/src/common/sparc_tcbinfo.c
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
  UINT16_MAX,            /* G0 */
  TCB_REG_OFF(REG_G1),   /* G1 */
  TCB_REG_OFF(REG_G2),   /* G2 */
  TCB_REG_OFF(REG_G3),   /* G3 */
  TCB_REG_OFF(REG_G4),   /* G4 */
  TCB_REG_OFF(REG_G5),   /* G5 */
  TCB_REG_OFF(REG_G6),   /* G6 */
  TCB_REG_OFF(REG_G7),   /* G7 */
  TCB_REG_OFF(REG_O0),   /* O0 */
  TCB_REG_OFF(REG_O1),   /* O1 */
  TCB_REG_OFF(REG_O2),   /* O2 */
  TCB_REG_OFF(REG_O3),   /* O3 */
  TCB_REG_OFF(REG_O4),   /* O4 */
  TCB_REG_OFF(REG_O5),   /* O5 */
  TCB_REG_OFF(REG_O6),   /* O6 */
  TCB_REG_OFF(REG_O7),   /* O7 */
  TCB_REG_OFF(REG_L0),   /* L0 */
  TCB_REG_OFF(REG_L1),   /* L1 */
  TCB_REG_OFF(REG_L2),   /* L2 */
  TCB_REG_OFF(REG_L3),   /* L3 */
  TCB_REG_OFF(REG_L4),   /* L4 */
  TCB_REG_OFF(REG_L5),   /* L5 */
  TCB_REG_OFF(REG_L6),   /* L6 */
  TCB_REG_OFF(REG_L7),   /* L7 */
  TCB_REG_OFF(REG_I0),   /* I0 */
  TCB_REG_OFF(REG_I1),   /* I1 */
  TCB_REG_OFF(REG_I2),   /* I2 */
  TCB_REG_OFF(REG_I3),   /* I3 */
  TCB_REG_OFF(REG_I4),   /* I4 */
  TCB_REG_OFF(REG_I5),   /* I5 */
  TCB_REG_OFF(REG_I6),   /* I6 */
  TCB_REG_OFF(REG_I7),   /* I7 */
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
