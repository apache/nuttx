/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/user-space.ld
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
    LONG(LOADADDR(.rodata) + SIZEOF(.rodata) - LOADADDR(.userspace))

    /* IRAM metadata:
     * - Destination address (VMA) for IRAM region
     * - Flash offset (LMA) for start of IRAM region
     * - Size of IRAM region
     */

    LONG(ADDR(.iram0.vectors))
    LONG(LOADADDR(.iram0.vectors))
    LONG(LOADADDR(.iram0.text) + SIZEOF(.iram0.text) - LOADADDR(.iram0.vectors))

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

  __kernel_vector_table = ORIGIN(KIRAM);

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

  .userspace :
  {
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
    . = ALIGN(4);
    _srodata = ABSOLUTE(.);

    *(.rodata)
    *(.rodata.*)

    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table)
    *(.gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    *(.eh_frame)

    . = (. + 3) & ~ 3;

    /* C++ constructor and destructor tables, properly ordered: */

    _sinit = ABSOLUTE(.);
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
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
    . = ALIGN(4);               /* This table MUST be 4-byte aligned */

    _erodata = ABSOLUTE(.);

    /* Literals are also RO data. */
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    . = ALIGN(4);
  } >UDROM AT>ROM

  .iram0.vectors :
  {
    _iram_start = ABSOLUTE(.);

    /* Vectors go to IRAM */

    _init_start = ABSOLUTE(.);

    __vectors_start = ABSOLUTE(.);

    /* Vectors according to builds/RF-2015.2-win32/esp108_v1_2_s5_512int_2/config.html */

    . = 0x0;
    KEEP (*(.window_vectors.text));
    . = 0x180;
    KEEP (*(.xtensa_level2_vector.text));
    . = 0x1c0;
    KEEP (*(.xtensa_level3_vector.text));
    . = 0x200;
    KEEP (*(.xtensa_level4_vector.text));
    . = 0x240;
    KEEP (*(.xtensa_level5_vector.text));
    . = 0x280;
    KEEP (*(.debug_exception_vector.text));
    . = 0x2c0;
    KEEP (*(.nmi_exception_vector.text));
    . = 0x300;
    KEEP (*(.kernel_exception_vector.text));
    . = 0x340;
    KEEP (*(.user_exception_vector.text));
    . = 0x3c0;
    KEEP (*(.double_exception_vector.text));
    . = 0x400;
    *(.*_vector.literal)

    . = ALIGN (16);

    __vectors_end = ABSOLUTE(.);

    *(.entry.text)
    *(.init.literal)
    *(.init)

    _init_end = ABSOLUTE(.);
  } >UIRAM AT>ROM

  .iram0.text :
  {

    *(.iram1)
    *(.iram1.*)

    _iram_end = ABSOLUTE(.);
  } >UIRAM AT>ROM

  /* Shared RAM */

  .bss (NOLOAD) :
  {
    /* .bss initialized on power-up */

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
    KEEP (*(.bss))
    *(.bss.*)
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)

    . = ALIGN(8);

    _ebss = ABSOLUTE(.);
  } >UDRAM

  .noinit (NOLOAD):
  {
    /* This section contains data that is not initialized during load,
     * or during the application's initialization sequence.
     */

    . = ALIGN(8);
    *(.noinit)
    *(.noinit.*)
    . = ALIGN(8);
  } >UDRAM

  .data :
  {
    _sdata = ABSOLUTE(.);
    KEEP (*(.data))
    KEEP (*(.data.*))
    KEEP (*(.gnu.linkonce.d.*))
    KEEP (*(.data1))
    KEEP (*(.sdata))
    KEEP (*(.sdata.*))
    KEEP (*(.gnu.linkonce.s.*))
    KEEP (*(.sdata2))
    KEEP (*(.sdata2.*))
    KEEP (*(.gnu.linkonce.s2.*))
    KEEP (*(.jcr))
    *(.dram1 .dram1.*)
    _edata = ABSOLUTE(.);
    . = ALIGN(4);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
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
    . = ALIGN(4);

    _etext = .;
  } >UIROM AT>ROM
}
