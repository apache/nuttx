/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_start.c
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
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "spacemit_k1x_rcpu_clockconfig.h"
#include "spacemit_k1x_rcpu_lowputc.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#define RSC_TABLE_VERSION 1

/* The resource table.
 *
 * Remote processors are allowed to publish their resources to the host, and
 * Linux refuses to boot a firmware whose ELF has no ".resource_table"
 * section at all: rproc_elf_load_rsc_table() looks the section up by name
 * before it will start the core.
 *
 * This port does not use any of those resources yet, so the table is the
 * bare header with zero entries.  The virtio devices that carry rpmsg are
 * added here (a fw_rsc_vdev entry per channel, with its vring children) when
 * the remote processor interface is brought up.  Until then the table exists
 * only to satisfy the loader and to reserve the carveout.
 */

struct spacemit_k1x_rcpu_rsc_s
{
  uint32_t ver;
  uint32_t num;
  uint32_t reserved[2];
};

__attribute__((section(".resource_table"), used))
const struct spacemit_k1x_rcpu_rsc_s g_spacemit_k1x_rcpu_rsc =
{
  .ver      = RSC_TABLE_VERSION,
  .num      = 0,
  .reserved =
  {
    0, 0
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __spacemit_k1x_rcpu_start
 *
 * Description: C part of the start up.  Entered from __start in
 * spacemit_k1x_rcpu_head.S with the stack set up, the caches configured and
 * the trap vector installed, and still with interrupts masked.
 *
 *   The image is RAM resident: it is linked for the address it runs at, so
 *   the load address and the run address are the same, so only .bss has to
 *   cleared here.
 *
 ****************************************************************************/

void __spacemit_k1x_rcpu_start(void)
{
  uint32_t *dest;

  /* Clear .bss.  Done inline rather than with memset so that there is no
   * dependency on the state of anything.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* The application processor has already brought the audio subsystem and
   * the RCPU clocks up before releasing this core, so this only has to
   * settle what the firmware owns.
   */

  spacemit_k1x_rcpu_clockconfig();

  /* The console has to be configured before the first showprogress().
   *
   * When CONFIG_DEBUG_FEATURES is set, showprogress() expands to
   * riscv_lowputc(), which polls the UART line status register.  The K1
   * console UART is a PXA variant held disabled until IER.UUE has been
   * programmed, so polling it before lowsetup() waits for a line status
   * that never becomes ready.  That is not theoretical: with the calls in
   * the other order the core was observed sitting in riscv_lowputc() and
   * nx_start() was never reached.
   */

  spacemit_k1x_rcpu_lowsetup();
  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif
  showprogress('B');

  /* Board level initialisation */

  spacemit_k1x_rcpu_boardinitialize();
  showprogress('C');

  /* Bring up the operating system */

  nx_start();

  /* nx_start() does not return */

  for (; ; )
    {
    }
}
