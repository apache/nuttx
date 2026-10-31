/****************************************************************************
 * arch/rh850/src/common/rh850_fpucmp.c
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

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fpucmp
 *
 * Description:
 *   The FPU works on the general registers; its own state is FPSR and FPEPC.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
bool up_fpucmp(FAR const void *saveregs1, FAR const void *saveregs2)
{
  FAR const uintptr_t *regs1 = saveregs1;
  FAR const uintptr_t *regs2 = saveregs2;

  return regs1[REG_FPSR] == regs2[REG_FPSR] &&
         regs1[REG_FPEPC] == regs2[REG_FPEPC];
}
#endif
