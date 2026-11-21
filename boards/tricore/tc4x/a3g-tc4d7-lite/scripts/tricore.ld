/****************************************************************************
 * boards/tricore/tc4x/a3g-tc4d7-lite/scripts/tricore.ld
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

#include <nuttx/config.h>

OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH("tricore")
ENTRY(__start)

/* CPU0 only: program flash bank A, the data scratchpad RAM and a window of the
 * DLMU of CPU0.
 * The CSA, the interrupt stack and the idle stack are at the top of the
 * data scratchpad RAM, the heap below them.
 */

#define DSPR_ORIGIN 0x70000000
#define DSPR_LENGTH 240K

MEMORY
{
  pfls (rx!p) : ORIGIN = 0x80000000, LENGTH = 1024K
  dspr (w!xp) : ORIGIN = DSPR_ORIGIN, LENGTH = DSPR_LENGTH
  dlmu (w!xp) : ORIGIN = 0x90020400, LENGTH = 171K
}

REGION_ALIAS("DATA", dlmu)

__csa_size    = 40K;
__istack_size = CONFIG_ARCH_INTERRUPTSTACK;
__ustack_size = CONFIG_IDLETHREAD_STACKSIZE;

__csa_end     = ORIGIN(dspr) + LENGTH(dspr);
__csa_begin   = __csa_end - __csa_size;
__istack      = __csa_begin;
__ustack      = __istack - __istack_size;

/* Boot stack and single boot CSA in the PSPR, which the startup code uses
 * to disable the virtualization before it links the CSAs
 */

__psram_ustack       = 0x7010eb00;
__psram_boot_csa     = 0x7010ec00;
__psram_boot_csa_end = 0x7010fc00;

/* The other cores start at the program flash of their own image */

PROVIDE(__start1 = 0x80200000);
PROVIDE(__start2 = 0x80580000);
PROVIDE(__start3 = 0x80680000);
PROVIDE(__start4 = 0x807e0000);
PROVIDE(__start5 = 0x8092c000);

#ifdef CONFIG_ARCH_IRQPRIO
#  define INTTAB_OFFSET 0x20
#else
#  define INTTAB_OFFSET 0x1fe0
#endif

SECTIONS
{
  /* Reset entry at +0, full BIV table at +0x20, single-entry BIV at
   * +0x1fe0, trap table at +0x2000, and text at +0x2100.
   */

  .start ORIGIN(pfls) :
  {
    KEEP(*(.start))
  } > pfls

  .traptab ORIGIN(pfls) + 0x2000 :
  {
    __TRAPTAB_CPU0 = .;
    KEEP(*(.traptab))
  } > pfls

  .intvec ORIGIN(pfls) + INTTAB_OFFSET :
  {
#ifdef CONFIG_ARCH_IRQPRIO
    __INTTAB_CPU0 = ORIGIN(pfls);
#else
    __INTTAB_CPU0 = ORIGIN(pfls) + INTTAB_OFFSET;
#endif
    KEEP(*(.inttab))
  } > pfls

  .text ORIGIN(pfls) + 0x2100 :
  {
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
    . = ALIGN(8);
  } > pfls

  .rodata :
  {
    *(.rodata .rodata.*)
    *(.zrodata .zrodata.*)
    *(.gnu.linkonce.r.*)
    . = ALIGN(4);
  } > pfls

  /* Small constants, addressed relative to A1 */

  .srodata :
  {
    *(.srodata .srodata.*)
  } > pfls

  __a1_mem = ADDR(.srodata) + 32K;
  __a8_mem = __a1_mem;

  .ctors :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array .ctors))
    _einit = ABSOLUTE(.);
    . = ALIGN(8);
  } > pfls

  .eh_frame :
  {
    KEEP(*(.eh_frame))
  } > pfls

  /* Initialized data, small data (A0) first, copied from flash at startup */

  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.sdata .sdata.*)
    *(.zdata .zdata.*)
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > DATA AT > pfls

  _data_flash = LOADADDR(.data);

  .bss (NOLOAD) :
  {
    _sbss = ABSOLUTE(.);
    *(.sbss .sbss.*)
    *(.zbss .zbss.*)
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
  } > DATA

  __a0_mem = _sdata + 32K;
  __a9_mem = __a0_mem;

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    _sheap = ABSOLUTE(.);
  } > dspr

  _eheap = __ustack - __ustack_size;

  .ustack __ustack - __ustack_size (NOLOAD) :
  {
    . += __ustack_size;
  } > dspr

  .istack __istack - __istack_size (NOLOAD) :
  {
    . += __istack_size;
  } > dspr

  .csa __csa_begin (NOLOAD) :
  {
    . += __csa_size;
  } > dspr

  /DISCARD/ :
  {
    *(.comment)
  }
}
