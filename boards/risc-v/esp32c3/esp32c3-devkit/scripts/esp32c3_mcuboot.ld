/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/scripts/esp32c3_mcuboot.ld
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

/* Default entry point: */

ENTRY(__start);

SECTIONS
{
  .metadata :
  {
    /* Magic for load header */

    LONG(0xace637d3)

    /* Application entry point address */

    KEEP(*(.entry_addr))

    /* IRAM metadata:
     * - Destination address (VMA) for IRAM region
     * - Flash offset (LMA) for start of IRAM region
     * - Size of IRAM region
     */

    LONG(ADDR(.iram0.text))
    LONG(LOADADDR(.iram0.text))
    LONG(SIZEOF(.iram0.text))

    /* DRAM metadata:
     * - Destination address (VMA) for DRAM region
     * - Flash offset (LMA) for start of DRAM region
     * - Size of DRAM region
     */

    LONG(ADDR(.dram0.data))
    LONG(LOADADDR(.dram0.data))
    LONG(SIZEOF(.dram0.data))
  } >metadata

  _image_drom_vma = ADDR(.flash.rodata);
  _image_drom_lma = LOADADDR(.flash.rodata);
  _image_drom_size = LOADADDR(.flash.rodata) + SIZEOF(.flash.rodata) - _image_drom_lma;

  .flash.rodata :
  {
    _srodata = ABSOLUTE(.);
    *(EXCLUDE_FILE (*libarch.a:esp32c3_spiflash.* esp32c3_head.* esp32c3_start.*) .rodata)
    *(EXCLUDE_FILE (*libarch.a:esp32c3_spiflash.* esp32c3_head.* esp32c3_start.*) .rodata.*)

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
  } >drom0_0_seg AT>ROM

  .iram0.text :
  {
    _iram_start = ABSOLUTE(.);

    /* Vectors go to start of IRAM */

    KEEP(*(.exception_vectors.text));
    . = ALIGN(4);

    *(.iram1)
    *(.iram1.*)
    *libarch.a:esp32c3_spiflash.*(.literal .text .literal.* .text.*)
    esp32c3_head.*(.literal .text .literal.* .text.*)
    esp32c3_start.*(.literal .text .literal.* .text.*)
    *(.wifi0iram .wifi0iram.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
  } >iram0_0_seg AT>ROM

  .dram0.dummy (NOLOAD):
  {
    /* This section is required to skip .iram0.text area because iram0_0_seg
     * and dram0_0_seg reflect the same address space on different buses.
     */

    . = ORIGIN(dram0_0_seg) + _iram_end - _iram_start;
  } >dram0_0_seg

  /* Shared RAM */

  .dram0.bss (NOLOAD) :
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
  } >dram0_0_seg

  .noinit (NOLOAD):
  {
    /* This section contains data that is not initialized during load,
     * or during the application's initialization sequence.
     */

    . = ALIGN(8);
    *(.noinit)
    *(.noinit.*)
    . = ALIGN(8);
  } >dram0_0_seg

  .dram0.data :
  {
    /* .data initialized on power-up in ROMed configurations. */

    _sdata = ABSOLUTE(.);
    KEEP (*(.data))
    KEEP (*(.data.*))
    KEEP (*(.gnu.linkonce.d.*))
    KEEP (*(.data1))
    __global_pointer$ = . + 0x800;
    KEEP (*(.sdata))
    KEEP (*(.sdata.*))
    KEEP (*(.gnu.linkonce.s.*))
    KEEP (*(.sdata2))
    KEEP (*(.sdata2.*))
    KEEP (*(.gnu.linkonce.s2.*))
    KEEP (*(.jcr))
    *(.dram1 .dram1.*)
    *libarch.a:esp32c3_spiflash.*(.rodata .rodata.*)
    esp32c3_head.*(.rodata .rodata.*)
    esp32c3_start.*(.rodata .rodata.*)
    _edata = ABSOLUTE(.);
    . = ALIGN(4);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
  } >dram0_0_seg AT>ROM

  /* Marks the end of IRAM code segment */

  .iram0.text_end (NOLOAD) :
  {
    . = ALIGN (16);
  } >iram0_0_seg

  .iram0.data :
  {
    . = ALIGN(16);
    *(.iram.data)
    *(.iram.data*)
  } >iram0_0_seg AT>ROM

  .iram0.bss (NOLOAD) :
  {
    . = ALIGN(16);
    *(.iram.bss)
    *(.iram.bss*)

    . = ALIGN(16);
    _iram_end = ABSOLUTE(.);
  } >iram0_0_seg

  _image_irom_vma = ADDR(.flash.text);
  _image_irom_lma = LOADADDR(.flash.text);
  _image_irom_size = LOADADDR(.flash.text) + SIZEOF(.flash.text) - _image_irom_lma;

  /* The alignment of the ".flash.text" output section is forced to
   * 0x0000FFFF (64KB) to ensure that it will be allocated at the beginning
   * of the next available Flash block.
   * This is required to meet the following constraint from the external
   * flash MMU:
   *    VMA % 64KB == LMA % 64KB
   * i.e. the lower 16 bits of both the virtual address (address seen by the
   * CPU) and the load address (physical address of the external flash) must
   * be equal.
   */

  .flash_text_dummy (NOLOAD) : ALIGN(0x0000FFFF)
  {
    /* This section is required to skip .flash.rodata area because irom0_0_seg
     * and drom0_0_seg reflect the same address space on different buses.
     */

    . = SIZEOF(.flash.rodata);
  } >irom0_0_seg

  .flash.text : ALIGN(0x0000FFFF)
  {
    _stext = .;

    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    . = ALIGN(4);

    _etext = .;
  } >irom0_0_seg AT>ROM

  .rtc.text :
  {
    . = ALIGN(4);
    *(.rtc.literal .rtc.text)
  } >rtc_seg AT>ROM

  .rtc.dummy (NOLOAD) :
  {
    /* This section is required to skip .rtc.text area because the text and
     * data segments reflect the same address space on different buses.
     */

    . = SIZEOF(.rtc.text);
  } >rtc_seg

  /* RTC BSS section. */

  .rtc.bss (NOLOAD) :
  {
    *(.rtc.bss)
  } >rtc_seg

  .rtc.data :
  {
    *(.rtc.data)
    *(.rtc.rodata)

   /* Whatever is left from the RTC memory is used as a special heap. */

    . = ALIGN (4);
    _srtcheap = ABSOLUTE(.);
  } >rtc_seg AT>ROM
}
