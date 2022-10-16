/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/dram_main.c
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

#include <stdio.h>
#include <debug.h>
#include <hex2bin.h>

#include <nuttx/cache.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "mmu.h"
#include "cp15_cacheops.h"

#include "sama5d4-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DRAM_ENTRY       ((dram_entry_t)SAM_DDRCS_VSECTION)

#define DRAM_WAIT        1
#define DRAM_NOWAIT      0

#ifdef CONFIG_SAMA5D4EK_DRAM_START
#  define DRAM_BOOT_MODE DRAM_NOWAIT
#else
#  define DRAM_BOOT_MODE DRAM_WAIT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*dram_entry_t)(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dram_main
 *
 * Description:
 *   dram_main is a tiny program that runs in ISRAM.  dram_main will
 *   configure DRAM, present a prompt, load an Intel HEX file into DRAM,
 *   and either start that program or wait for you to break in with the
 *   debugger.
 *
 ****************************************************************************/

int dram_main(int argc, char *argv[])
{
  /* Here we have a in memory value we can change in the debugger
   * to begin booting in NOR Flash
   */

  static volatile uint32_t wait = DRAM_BOOT_MODE;
  int ret;

  /* Disable the PMC.  This is necessary on the SAMA5D4-MB Rev C. board.  On
   * that board, the PMIC can lock up the I2C bus.  The work around is
   * awkward:
   *
   *   1. Open JP23 (disabling the WM8904 data line)
   *   2. Execute DRAMBOOT.  The WM8904 will be disabled while JP23 is open.
   *   3. At the prompt to "Send the Intel HEX file now", close JP23,
   *      enabling the WM8904.
   *   4. Send the NuttX file.  When NuttX starts, the WM8904 is initialized,
   *      JP23 will be closed and the PMIC will be initialized.
   */

  sam_pmic_initialize();

  /* DRAM was already initialized at boot time, so we are ready to load the
   * Intel HEX stream into DRAM.
   *
   * Hmm.. With no hardware handshake, there is a possibility of data loss
   * to overrunning incoming data buffer.  So far I have not seen this at
   * 115200 8N1, but still it is a possibility.
   */

  printf("Send Intel HEX file now\n");
  fflush(stdout);

  ret = hex2mem(0,                           /* Accept Intel HEX on stdin */
                (uint32_t)SAM_DDRCS_VSECTION,
                (uint32_t)(SAM_DDRCS_VSECTION + CONFIG_SAMA5_DDRCS_SIZE),
                0);
  if (ret < 0)
    {
      /* We failed the load */

      printf("ERROR: Intel HEX file load failed: %d\n", ret);
      fflush(stdout);
      for (; ; );
    }

  /* No success indication.. The following cache/MMU operations will clobber
   * any I/O that we attempt (Hmm.. unless, perhaps, if we delayed.  But who
   * wants a delay?).
   */

  /* Flush the entire data cache assure that everything is in memory before
   * we disable caching.
   */

  up_clean_dcache((uintptr_t)SAM_DDRCS_VSECTION,
                  (uintptr_t)(SAM_DDRCS_VSECTION + CONFIG_SAMA5_DDRCS_SIZE));

  /* Interrupts must be disabled through the following.  In this
   * configuration, there should only be timer interrupts.  Your NuttX
   * configuration must use CONFIG_SERIAL_LOWCONSOLE=y or printf() will
   * hang when the interrupts are disabled!
   */

  up_irq_save();

  /* Disable the caches and the MMU.  Disabling the MMU should be safe here
   * because there is a 1-to-1 identity mapping between the physical and
   * virtual addressing.
   */

  cp15_disable_mmu();
  up_disable_icache();
  up_disable_dcache();

  /* Invalidate caches and TLBs */

  cp15_invalidate_icache();
  cp15_invalidate_dcache_all();
  cp15_invalidate_tlbs();

  /* Then jump into NOR flash */

  while (wait)
    {
    }

  DRAM_ENTRY();

  return 0; /* We should not get here in either case */
}
