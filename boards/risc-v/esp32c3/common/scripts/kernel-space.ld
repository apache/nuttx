/****************************************************************************
 * boards/risc-v/esp32c3/common/scripts/kernel-space.ld
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

/* Provide these so there is no need for using config files for this */

__uirom_start = ORIGIN(UIROM);
__uirom_size = LENGTH(UIROM);
__uirom_end = ORIGIN(UIROM) + LENGTH(UIROM);
__udrom_start = ORIGIN(UDROM);
__udrom_size = LENGTH(UDROM);
__udrom_end = ORIGIN(UDROM) + LENGTH(UDROM);
__uiram_start = ORIGIN(UIRAM);
__uiram_size = LENGTH(UIRAM);
__uiram_end = ORIGIN(UIRAM) + LENGTH(UIRAM);
__udram_start = ORIGIN(UDRAM);
__udram_size = LENGTH(UDRAM);
__udram_end = ORIGIN(UDRAM) + LENGTH(UDRAM);

/* Provide the kernel boundaries as well */

__kirom_start = ORIGIN(KIROM);
__kirom_size = LENGTH(KIROM);
__kdrom_start = ORIGIN(KDROM);
__kdrom_size = LENGTH(KDROM);
__kiram_start = ORIGIN(KIRAM);
__kiram_size = LENGTH(KIRAM);
__kiram_end = ORIGIN(KIRAM) + LENGTH(KIRAM);
__kdram_start = ORIGIN(KDRAM);
__kdram_size = LENGTH(KDRAM);
__kdram_end = ORIGIN(KDRAM) + LENGTH(KDRAM);

ENTRY(_stext)

SECTIONS
{
  .iram0.text :
  {
    _iram_start = ABSOLUTE(.);

    __vectors_start = ABSOLUTE(.);

    /* Vectors go to start of IRAM */

    KEEP(*(.exception_vectors.text));
    . = ALIGN(4);

    __vectors_end = ABSOLUTE(.);

    *(.iram1)
    *(.iram1.*)

    *libsched.a:irq_dispatch.*(.text .text.* .literal .literal.*)

    *(.wifi0iram .wifi0iram.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)

    _iram_end = ABSOLUTE(.);
  } >KIRAM

  /* This section is required to skip .iram0.text area because iram0_0_seg
   * and dram0_0_seg reflect the same address space on different buses.
   */

  .dram0.dummy (NOLOAD):
  {
    . = ORIGIN(KDRAM) + _iram_end - _iram_start;
  } >KDRAM

  /* Shared RAM */

  .dram0.bss (NOLOAD) :
  {
    . = ALIGN (8);
    _sbss = ABSOLUTE(.);
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.bss)
    *(.bss.*)
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)

    . = ALIGN (8);
    _ebss = ABSOLUTE(.);
  } >KDRAM

  .noinit (NOLOAD):
  {
    /* This section contains data that is not initialized during load,
     * or during the application's initialization sequence.
     */

    *(.noinit)
    *(.noinit.*)
  } >KDRAM

  .dram0.data :
  {
    _sdata = ABSOLUTE(.);
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    *(.data1)
    __global_pointer$ = . + 0x800;
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.sdata2)
    *(.sdata2.*)
    *(.gnu.linkonce.s2.*)
    *(.jcr)
    *(.dram1)
    *(.dram1.*)
    . = ALIGN(4);
    _edata = ABSOLUTE(.);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
  } >KDRAM

  .flash.text :
  {
    _stext = .;

    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    _etext = .;

    /* Similar to _iram_start, this symbol goes here so it is
     * resolved by addr2line in preference to the first symbol in
     * the flash.text segment.
     */

    _flash_cache_start = ABSOLUTE(0);
  } >KIROM

  .flash_rodata_dummy (NOLOAD):
  {
    . = ALIGN(ALIGNOF(.flash.text));
    . = . + SIZEOF(.flash.text);
    . = ALIGN(0x10000) + 0x20;
  } >KDROM

  .flash.rodata : ALIGN(0x10)
  {
    _srodata = ABSOLUTE(.);

    *(.rodata)
    *(.rodata.*)

    *(.srodata.*)

    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    . = (. + 3) & ~ 3;
    __eh_frame = ABSOLUTE(.);
    KEEP(*(.eh_frame))
    . = (. + 7) & ~ 3;

    /* C++ constructor and destructor tables:
     * RISC-V GCC is configured with --enable-initfini-array so it emits an
     * .init_array section instead.
     */

    _sinit = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .init_array.*))
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .init_array))
    _einit = ABSOLUTE(.);
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))

    /* C++ exception handlers table: */

    __XT_EXCEPTION_DESCS_ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    _erodata = ABSOLUTE(.);

    /* Literals are also RO data. */

    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    . = ALIGN(4);
  } >KDROM
}
