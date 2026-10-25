/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/boot.ld
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

/* Stage-1 boot loader: copied to 0x10800 and called by the mask ROM from
 * the boot header of the flash.  It must stay below 0x12000, where the
 * NuttX image starts, and is limited to 3 KB by the boot sector layout.
 * Its stack is at the top of AECRAM (the host tool keeps images out of the
 * last KB there).
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

MEMORY
{
  ram (rwx) : ORIGIN = 0x00010800, LENGTH = 0xc00
}

SECTIONS
{
  .text : {
    KEEP(*(.start))
    *(.text .text.*)
    *(.rodata .rodata.* .srodata .srodata.*)
  } > ram

  .data : ALIGN(4) {
    *(.data .data.* .sdata .sdata.*)
  } > ram

  .bss (NOLOAD) : ALIGN(4) {
    _sbss = .;
    *(.bss .bss.* .sbss .sbss.* COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > ram

  _stack_top = 0x00057a00;

  /DISCARD/ : { *(.comment) *(.riscv.attributes) *(.eh_frame) }
}
