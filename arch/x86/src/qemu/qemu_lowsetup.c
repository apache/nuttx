/****************************************************************************
 * arch/x86/src/qemu/qemu_lowsetup.c
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "x86_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gdt_entry_s gdt_entries[5];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_gdtentry
 *
 * Description:
 *   Set the value of one GDT entry.
 *
 ****************************************************************************/

static void up_gdtentry(struct gdt_entry_s *entry, uint32_t base,
                        uint32_t limit, uint8_t access, uint8_t gran)
{
  entry->lowbase      = (base & 0xffff);
  entry->midbase      = (base >> 16) & 0xff;
  entry->hibase       = (base >> 24) & 0xff;

  entry->lowlimit     = (limit & 0xffff);
  entry->granularity  = (limit >> 16) & 0x0f;

  entry->granularity |= gran & 0xf0;
  entry->access       = access;
}

/****************************************************************************
 * Name: up_gdtinit
 *
 * Description:
 *   Initialize the GDT. The Global Descriptor Table or GDT is a data
 *   structure used by Intel x86-family processors starting with the 80286
 *   in order to define the characteristics of the various memory areas used
 *   during program execution, for example the base address, the size and
 *   access privileges like executability and writability. These memory areas
 *   are called segments in Intel terminology.
 *
 ****************************************************************************/

static void up_gdtinit(void)
{
  struct gdt_ptr_s gdt_ptr;

  up_gdtentry(&gdt_entries[0], 0, 0, 0, 0);                /* Null segment */
  up_gdtentry(&gdt_entries[1], 0, 0xffffffff, 0x9a, 0xcf); /* Code segment */
  up_gdtentry(&gdt_entries[2], 0, 0xffffffff, 0x92, 0xcf); /* Data segment */
  up_gdtentry(&gdt_entries[3], 0, 0xffffffff, 0xfa, 0xcf); /* User mode code segment */
  up_gdtentry(&gdt_entries[4], 0, 0xffffffff, 0xf2, 0xcf); /* User mode data segment */

  gdt_ptr.limit = (sizeof(struct gdt_entry_s) * 5) - 1;
  gdt_ptr.base  = (uint32_t)gdt_entries;
  gdt_flush((uint32_t)&gdt_ptr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   Called from qemu_head BEFORE starting the operating system in order
 *   perform any necessary, early initialization.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
  /* Initialize the Global descriptor table */

  up_gdtinit();

  /* Early serial driver initialization */

#ifdef USE_EARLYSERIALINIT
  x86_earlyserialinit();
#endif

  /* Now perform board-specific initializations */

  x86_boardinitialize();
}
