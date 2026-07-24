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
   * Note: ESP32-S3 PMS (World Controller) memory-protection violations are
   * NOT delivered as these precise causes; they raise the asynchronous
   * DRAM0/IRAM0 PMS-monitor interrupt instead (handled elsewhere).
   */

  if (exccause == EXCCAUSE_LOAD_PROHIBITED  ||
      exccause == EXCCAUSE_STORE_PROHIBITED ||
      exccause == EXCCAUSE_INSTR_PROHIBITED)
    {
      if (esp32s3_pagefault_dispatch(exccause, regs) == OK)
        {
          return regs;
        }

#ifdef CONFIG_ESP32S3_PAGEFAULT_ABORT
      /* Unrecoverable, but a WORLD1 (user-mode) fault need not take down the
       * whole system: terminate just the faulting task with SIGSEGV.  The
       * User Mode bit in the interruptee's saved PS distinguishes a user
       * fault from a privileged one (which still panics below).
       */

      if ((regs[REG_PS] & PS_UM) != 0)
        {
          return esp32s3_pagefault_abort(exccause, regs);
        }
#endif
    }
#endif

  /* xtensa_user_panic never returns. */

  xtensa_user_panic(exccause, regs);

  while (1)
    {
    }
}
