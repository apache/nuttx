/****************************************************************************
 * arch/risc-v/src/common/riscv_getnewintctx.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/csr.h>

#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_get_newintctx
 *
 * Description:
 *   Return initial mstatus when a task is created.
 *
 ****************************************************************************/

uintptr_t riscv_get_newintctx(void)
{
  /* Set machine previous privilege mode to machine mode. Reegardless of
   * how NuttX is configured and of what kind of thread is being started.
   * That is because all threads, even user-mode threads will start in
   * kernel trampoline at nxtask_start() or pthread_start().
   * The thread's privileges will be dropped before transitioning to
   * user code. Also set machine previous interrupt enable.
   *
   * Mask the bits which should be preserved (from ISA spec)
   */

  uintptr_t mstatus = READ_CSR(mstatus);

  mstatus &= MSTATUS_WPRI;

  return (mstatus | MSTATUS_MPPM | MSTATUS_MPIE
#ifdef CONFIG_ARCH_FPU
                  | MSTATUS_FS_INIT
#endif
  );
}
