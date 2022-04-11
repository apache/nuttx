/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_mcall.c
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
#include <nuttx/irq.h>

#include <stdint.h>

#include "riscv_internal.h"

#include "sbi_mcall.h"
#include "sbi_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sbi_mcall_handle
 *
 * Description:
 *   Handle environment call to machine mode
 *
 * Input Parameters:
 *   regs - User context
 *
 ****************************************************************************/

void sbi_mcall_handle(uintptr_t *regs)
{
  /* Check the environment call number */

  switch (regs[REG_A0])
  {
    case MCALL_GET_TIMER:
#ifdef CONFIG_ARCH_RV64
      regs[REG_A0]    = sbi_get_mtime();
#else
      {
        uint64_t time = sbi_get_mtime();
        regs[REG_A0]  = (uint32_t)time;
        regs[REG_A1]  = (uint32_t)(time >> 32);
      }
#endif
      break;

    case MCALL_SET_TIMER:
#ifdef CONFIG_ARCH_RV64
      sbi_set_mtimecmp(regs[REG_A1]);
#else
      sbi_set_mtimecmp(regs[REG_A1] + ((uint64_t)regs[REG_A2] << 32));
#endif
      CLEAR_CSR(mip, MIP_STIP);
      SET_CSR(mie, MIE_MTIE);
      break;

    default:
      break;
  }
}
