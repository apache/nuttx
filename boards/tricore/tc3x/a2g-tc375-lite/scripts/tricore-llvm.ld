/****************************************************************************
 * boards/tricore/tc3x/a2g-tc375-lite/scripts/tricore-llvm.ld
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

__HEAP_SIZE = (48 * 1024);
CONFIG_TRICORE_CSA_COUNT = 512;
CONFIG_TRICORE_INITIAL_STACK_SIZE = 512;
CONFIG_TRICORE_INITIAL_STACK_OFFSET = 0xEB00;

MEMORY
{
#if CPU == 0
  dsram  (rw): ORIGIN = 0x70000000, LENGTH = 240K
  psram  (rw): ORIGIN = 0x70100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90000000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80000000, LENGTH = 1M
#endif
  pflash0 (rx): ORIGIN = 0xA0000000, LENGTH = 1M

#if CPU == 1
  dsram  (rw): ORIGIN = 0x60000000, LENGTH = 240K
  psram  (rw): ORIGIN = 0x60100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90010000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80100000, LENGTH = 1M
#endif
  pflash1 (rx): ORIGIN = 0x80100000, LENGTH = 1M

#if CPU == 2
  dsram  (rw): ORIGIN = 0x50000000, LENGTH = 96K
  psram  (rw): ORIGIN = 0x50100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90020000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80200000, LENGTH = 1M
#endif
  pflash2 (rx): ORIGIN = 0x80200000, LENGTH = 1M

}

REGION_ALIAS("CODE_MEM", pflash)
REGION_ALIAS("DATA_MEM", dsram)
REGION_ALIAS("BSS_MEM", dsram)
REGION_ALIAS("CSA_MEM", dsram)
REGION_ALIAS("HEAP_MEM", dsram)
REGION_ALIAS("PSRAM_MEM", psram)

pflash0_base = ORIGIN(pflash0);
pflash1_base = ORIGIN(pflash1);
pflash2_base = ORIGIN(pflash2);

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

  .flash_banks :
  {
    _flash_banks = . ;
    LONG(pflash0_base);
    LONG(pflash1_base);
    LONG(pflash2_base);
  } > CODE_MEM

  .psram_base :
  {
    . += CONFIG_TRICORE_INITIAL_STACK_OFFSET;
     __PSRAM_USTACK = . ;
    . += CONFIG_TRICORE_INITIAL_STACK_SIZE;
     __PSRAM_BOOT_CSA = . ;
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
    . +=  CONFIG_IDLETHREAD_STACKSIZE ;
    __USTACK =  . ;
  } > BSS_MEM

#if defined(CONFIG_ARCH_INTERRUPTSTACK)
  .istack ALIGN(8) (NOLOAD) :
  {
    . += CONFIG_ARCH_INTERRUPTSTACK;
    __istack =  . ;
  } > BSS_MEM
#endif

  .csa (NOLOAD) : ALIGN(64)
  {
    __CSA_BEGIN = . ;
    . +=  CONFIG_TRICORE_CSA_COUNT * 64 ;
    __CSA_END = .;
  } > CSA_MEM
  __CSA_SIZE = __CSA_END - __CSA_BEGIN;

  .heap (NOLOAD) : ALIGN(8)
  {
    _sheap =  . ;
    . +=  __HEAP_SIZE;
     _eheap =  . ;
  } > HEAP_MEM
  _heap_size = _sheap - _eheap;
}
