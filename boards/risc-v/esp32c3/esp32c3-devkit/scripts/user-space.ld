/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/scripts/user-space.ld
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

SECTIONS
{
  .metadata :
  {
    /* DROM metadata:
     * - Destination address (VMA) for DROM region
     * - Flash offset (LMA) for start of DROM region
     * - Size of DROM region
     */

    LONG(ADDR(.userspace))
    LONG(LOADADDR(.userspace))
    LONG(SIZEOF(.userspace) + SIZEOF(.rodata))

    /* IROM metadata:
     * - Destination address (VMA) for IROM region
     * - Flash offset (LMA) for start of IROM region
     * - Size of IROM region
     */

    LONG(ADDR(.text))
    LONG(LOADADDR(.text))
    LONG(SIZEOF(.text))
  } >metadata

  /* section info */

  __ld_uirom_start = ORIGIN(UIROM);
  __ld_uirom_size = LENGTH(UIROM);
  __ld_uirom_end = ORIGIN(UIROM) + LENGTH(UIROM);
  __ld_udrom_start = ORIGIN(UDROM);
  __ld_udrom_size = LENGTH(UDROM);
  __ld_udrom_end = ORIGIN(UDROM) + LENGTH(UDROM);
  __ld_uiram_start = ORIGIN(UIRAM);
  __ld_uiram_size = LENGTH(UIRAM);
  __ld_uiram_end = ORIGIN(UIRAM) + LENGTH(UIRAM);
  __ld_udram_start = ORIGIN(UDRAM);
  __ld_udram_size = LENGTH(UDRAM);
  __ld_udram_end = ORIGIN(UDRAM) + LENGTH(UDRAM);

  _eronly = LOADADDR(.data);

  .userspace : {
    *(.userspace)
  } >UDROM AT>ROM

  /* Output sections for the Userspace image are given standard names, so
   * instead of the Espressif-usual ".flash.text" we name it as ".text".
   * The motivation is to ease debugging with GDB when loading symbols from
   * both Kernel and User images since GDB's "add-symbol-file" command
   * expects to find a .text section at the provided offset.
   */

  .rodata :
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
  } >UDROM AT>ROM

  .iram0.text :
  {
    _iram_start = ABSOLUTE(.);

    *(.iram1)
    *(.iram1.*)

    _iram_end = ABSOLUTE(.);
  } >UIRAM AT>ROM

  /* This section is required to skip .iram0.text area because iram0_0_seg
   * and dram0_0_seg reflect the same address space on different buses.
   */

  .dram0.dummy (NOLOAD):
  {
    . = ORIGIN(UDRAM) + _iram_end - _iram_start;
  } >UDRAM

  /* Shared RAM */

  .bss (NOLOAD) :
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
  } >UDRAM

  .noinit (NOLOAD):
  {
    /* This section contains data that is not initialized during load,
     * or during the application's initialization sequence.
     */

    *(.noinit)
    *(.noinit.*)
  } >UDRAM

  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    *(.data1)
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
  } >UDRAM AT>ROM

  .flash_text_dummy (NOLOAD) : ALIGN(0x00010000)
  {
    . = SIZEOF(.userspace) + SIZEOF(.rodata);
  } >UIROM

  .text : ALIGN(0x00010000)
  {
    _stext = .;

    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    _etext = .;
  } >UIROM AT>ROM
}
