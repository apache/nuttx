/****************************************************************************
 * arch/tricore/src/common/tricore-llvm.ld
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
EXTERN(__start)

CONFIG_TRICORE_CSA_COUNT = 512;
CONFIG_TRICORE_INITIAL_STACK_SIZE = 512;
CONFIG_TRICORE_INITIAL_STACK_OFFSET = 0xEB00;

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
__HEAP_SIZE = (512 * 1024);
#else
__HEAP_SIZE = (64 * 1024);
#endif

MEMORY
{
  dsram  (rw): ORIGIN = 0x70000000, LENGTH = 240K
  psram  (rwx): ORIGIN = 0x70100000, LENGTH = 64K
#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
  dlmu   (rw): ORIGIN = 0x90000000, LENGTH = 512K
#else
  dlmu   (rw): ORIGIN = 0x90000000, LENGTH = 96K
#endif
  pflash (rx): ORIGIN = 0x80000000, LENGTH = 4M
}

REGION_ALIAS("CODE_MEM", pflash)
REGION_ALIAS("DATA_MEM", dsram)
REGION_ALIAS("BSS_MEM", dsram)
REGION_ALIAS("CSA_MEM", dsram)
REGION_ALIAS("HEAP_MEM", dlmu)
REGION_ALIAS("PSRAM_MEM", psram)

SECTIONS
{
  .start :
  {
    KEEP(*(.start))
    . = ALIGN(4);
  } > CODE_MEM

  .traptab :
  {
    KEEP(*(.traptab))
    . = ALIGN(256);
  } > CODE_MEM

  .ivtab :
  {
    KEEP(*(.ivtab))
    . = ALIGN(4);
  } > CODE_MEM

  .text :
  {
    KEEP(*(.text))
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
    *(.rodata .rodata.*)
    *(.gnu.linkonce.r.*)
    . = ALIGN(2);
  } > CODE_MEM

  .data : ALIGN(4)
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > DATA_MEM AT> CODE_MEM
  _data_flash = LOADADDR(.data);

  .psram_base ALIGN(8) (NOLOAD) :
  {
    . += CONFIG_TRICORE_INITIAL_STACK_OFFSET;
    __PSRAM_USTACK = .;
    . += CONFIG_TRICORE_INITIAL_STACK_SIZE;
    __PSRAM_BOOT_CSA = .;
  } > PSRAM_MEM

  .bss :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
  } > BSS_MEM

  .ustack ALIGN(8) (NOLOAD) :
  {
    . += CONFIG_IDLETHREAD_STACKSIZE;
    __USTACK = .;
  } > BSS_MEM

#if defined(CONFIG_ARCH_INTERRUPTSTACK)
  .istack ALIGN(8) (NOLOAD) :
  {
    . += CONFIG_ARCH_INTERRUPTSTACK;
    __istack = .;
  } > BSS_MEM
#endif

  .csa (NOLOAD) : ALIGN(64)
  {
    __CSA_BEGIN = .;
    . += CONFIG_TRICORE_CSA_COUNT * 64;
    __CSA_END = .;
  } > CSA_MEM
  __CSA_SIZE = __CSA_END - __CSA_BEGIN;

  .heap (NOLOAD) : ALIGN(8)
  {
    _sheap = .;
    . += __HEAP_SIZE;
    _eheap = .;
  } > HEAP_MEM
  _heap_size = _sheap - _eheap;
}
