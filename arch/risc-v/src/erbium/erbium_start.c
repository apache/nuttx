/****************************************************************************
 * arch/risc-v/src/erbium/erbium_start.c
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
#include <stdint.h>
#include <nuttx/init.h>

#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: erbium_start
 *
 * Description:
 *   Start NuttX on hart 0. The emulator loads initialized data from the ELF
 *   directly into MRAM, so no ROM-to-RAM data copy is necessary.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Does not return.
 *
 * Assumptions/Limitations:
 *   Assembly startup has initialized the stack, global pointer and trap
 *   vector, with interrupts disabled.
 *
 ****************************************************************************/

void erbium_start(void)
{
  uint32_t *dest;

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

#ifdef CONFIG_ARCH_FPU
  /* Erbium traps FENCE.I to microcode. The ELF loader has already installed
   * the image, so initialize the FPU without the common FENCE.I sequence.
   */

  SET_CSR(CSR_STATUS, MSTATUS_FS_INIT);
  WRITE_CSR(CSR_FCSR, 0);
#endif

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  nx_start();
}
