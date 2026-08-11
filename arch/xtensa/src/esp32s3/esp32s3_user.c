/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_user.c
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

#include "xtensa.h"
#include <arch/xtensa/xtensa_corebits.h>
#ifdef CONFIG_ESP32S3_PAGEFAULT
#include "esp32s3_pagefault.h"
#endif
#include "esp32s3_userfault.h"
#ifdef CONFIG_ESPRESSIF_SPIFLASH
#include "esp_private/cache_utils.h"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_user
 *
 * Description:
 *   ESP32-S3-specific user exception handler.
 *
 * Input Parameters:
 *   exccause - Identifies the EXCCAUSE of the user exception.
 *   regs     - The register save are at the time of the interrupt.
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

uint32_t *xtensa_user(int exccause, uint32_t *regs)
{
#ifdef CONFIG_ESPRESSIF_SPIFLASH
  int cpu = this_cpu();

  if (!spi_flash_cache_enabled())
    {
      spi_flash_restore_cache(cpu, 0);
    }
#endif /* CONFIG_ESPRESSIF_SPIFLASH */

#ifdef CONFIG_ESP32S3_PAGEFAULT
  /* A cache-attribute permission violation raises a precise, restartable
   * exception: Load/Store/InstrFetch Prohibited (EXCCAUSE 28/29/20), with
   * EXCVADDR holding the exact faulting address.  This is proven on silicon
   * (see esp32s3_pagefault.c) and is the recoverable-fault primitive.  Offer
   * these to the dispatcher; if serviced, return the register frame so that
   * the RFE in the exception vector re-executes the faulting instruction.
   *
   * A PMS permission violation does not arrive as one of these causes.  It
   * raises the asynchronous DRAM0/IRAM0 PMS monitor interrupt, which
   * pms_violation_isr() in esp32s3_isolation.c serves.  So does an access
   * that no MMU entry translates, which the cache reports separately.  Both
   * are installed by esp32s3_pmsirqinitialize(), in a protected build and in
   * a kernel build alike.
   */

  if (exccause == EXCCAUSE_LOAD_PROHIBITED  ||
      exccause == EXCCAUSE_STORE_PROHIBITED ||
      exccause == EXCCAUSE_INSTR_PROHIBITED)
    {
      if (esp32s3_pagefault_dispatch(exccause, regs) == OK)
        {
          return regs;
        }
    }
#endif

#ifdef CONFIG_ESP32S3_USERFAULT_ABORT
  /* Nothing serviced it, so this fault is not going away by re-executing.
   * If the interruptee was unprivileged, terminate just that task.
   *
   * The test is on the interruptee's User Mode bit and NOT on the cause, and
   * that is the whole point.  A user task has many ways to raise a
   * synchronous exception that no dispatcher can service -- an unaligned
   * load (EXCCAUSE 9), a divide by zero (6), a privileged instruction (8), a
   * load/store error (3), a corrupt opcode (0) -- and gating on a list of
   * causes means every cause left off the list is a way for an unprivileged
   * task to stop the machine.  Whichever way it arose, a user task running
   * garbage must not take the system down with it.
   *
   * PS.UM also excludes the cases where there is no safe task to kill:  a
   * kernel thread, a fault inside a system call made on the user's behalf,
   * and a fault while handling an interrupt all run with it clear, and fall
   * through to the panic below.
   *
   * One cause deserves its own note.  A *denied* external-memory access does
   * not trap on this chip.  TRM v1.8 p.699: an access without permission is
   * "responded with 0 (for internal memory) or 0xdeadbeaf (for external
   * memory)".  So an unprivileged branch into kernel text in flash or PSRAM
   * is refused silently, the CPU executes the dummy word it was handed
   * instead, and the refusal surfaces here as EXCCAUSE_ILLEGAL at the
   * address that was branched to -- never as EXCCAUSE_INSTR_PROHIBITED.
   */

  if ((regs[REG_PS] & PS_UM) != 0)
    {
      return esp32s3_userfault_abort(exccause, regs);
    }
#endif

  /* xtensa_user_panic never returns. */

  xtensa_user_panic(exccause, regs);

  while (1)
    {
    }
}
