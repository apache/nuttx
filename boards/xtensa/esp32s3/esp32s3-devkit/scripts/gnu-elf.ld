/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/scripts/gnu-elf.ld
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

/* Link script for the user programs of a kernel build.  A fully linked
 * executable is loaded at the addresses it was linked for -- binfmt takes
 * the entry point straight out of the ELF header -- so these must match the
 * address environment the ESP32-S3 gives a user process:
 *
 *   .text  goes to CONFIG_ARCH_TEXT_VBASE, a slot of the instruction-bus
 *          cache-MMU window (0x42000000-0x44000000).
 *   .data  goes to CONFIG_ARCH_DATA_VBASE plus one page: up_addrenv_create()
 *          keeps the first page of the data window for the OS reserve (the
 *          heap bookkeeping and the signal trampoline pointer).  The window
 *          is a slot of the data-bus range (0x3C000000-0x3E000000).
 *
 * Both windows have to sit above the kernel's own PSRAM window, which the
 * two buses share entry for entry: 0x42C00000 and 0x3C000000+0xC00000 are
 * the same cache-MMU entry, so a user window that overlapped it would take
 * away the kernel's view of the pages underneath.  The kernel window starts
 * wherever the flash mappings end, so it moves as the image grows;
 * up_allocate_pgheap() checks the two against each other at boot.
 *
 * .rodata sits with the data, not with the text: the ESP32-S3 selects
 * CONFIG_ARCH_HAVE_TEXT_HEAP_WORD_ALIGNED_READ, so the loader places every
 * section that is not executable into the data region, and unaligned reads
 * through the instruction bus are not reliable anyway.
 *
 * The loader packs allocatable sections back to back in section-header
 * order, honouring only each section's own alignment, so the layout below
 * must not introduce gaps of its own.
 *
 * Keep these addresses in step with CONFIG_ARCH_TEXT_VBASE,
 * CONFIG_ARCH_DATA_VBASE and CONFIG_MM_PGSIZE in the board defconfig.
 */

SECTIONS
{
  . = 0x42c00000;

  .text :
    {
      _stext = . ;
      *(.literal .text .literal.* .text.*)
      *(.gnu.warning)
      *(.stub)
      *(.jcr)

      /* C++ support:  The .init and .fini sections contain specific logic
       * to manage static constructors and destructors.
       */

      *(.gnu.linkonce.t.*)
      *(.init)
      *(.fini)
      . = ALIGN(4);
      _etext = . ;
    }

  . = 0x3d010000;

  .rodata :
    {
      _srodata = . ;
      *(.rodata)
      *(.rodata1)
      *(.rodata.*)
      *(.gnu.linkonce.r*)
      . = ALIGN(4);
      _erodata = . ;
    }

  /* Placed explicitly, and here, because the loader assigns it to the data
   * region -- it is allocatable but not executable -- and packs it directly
   * after .rodata.
   */

  .eh_frame :
    {
      KEEP (*(.eh_frame))
      *(.eh_frame.*)
    }

  .data :
    {
      _sdata = . ;
      *(.data)
      *(.data1)
      *(.data.*)
      *(.gnu.linkonce.d*)
      . = ALIGN(4);
      _edata = . ;
    }

  .init_array :
    {
      _sinit = .;
      _sctors = .;
      KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      KEEP(*(.init_array .ctors))
      . = ALIGN(4);
      _einit = .;
      _ectors = .;
    }

  .fini_array :
    {
      _sfini = .;
      _sdtors = .;
      KEEP (*(.dtors))
      KEEP (*(.fini_array))
      KEEP (*(SORT(.fini_array.*)))
      . = ALIGN(4);
      _efini = .;
      _edtors = .;
    }

  .bss :
    {
      _sbss = . ;
      *(.bss)
      *(.bss.*)
      *(.sbss)
      *(.sbss.*)
      *(.gnu.linkonce.b*)
      *(COMMON)
      . = ALIGN(4);
      _ebss = . ;
    }

  .got :
    {
      *(.got*)
    }

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
