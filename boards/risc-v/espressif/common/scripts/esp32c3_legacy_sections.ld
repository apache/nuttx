/****************************************************************************
 * boards/risc-v/espressif/common/scripts/esp32c3_legacy_sections.ld
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
  .iram0.text :
  {
    _iram_start = ABSOLUTE(.);

    /* Vectors go to start of IRAM */

    KEEP(*(.exception_vectors.text));
    . = ALIGN(4);

    *(.iram1)
    *(.iram1.*)

    *libarch.a:brownout.*(.text .text.* .literal .literal.*)
    *libarch.a:cpu.*(.text .text.* .literal .literal.*)
    *libarch.a:gpio_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:periph_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:rtc_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:rtc_sleep.*(.text .text.* .literal .literal.*)
    *libarch.a:rtc_time.*(.text .text.* .literal .literal.*)
    *libarch.a:systimer.*(.text .text.* .literal .literal.*)
    *libarch.a:systimer_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:uart_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:wdt_hal_iram.*(.text .text.* .literal .literal.*)
    *libsched.a:irq_dispatch.*(.text .text.* .literal .literal.*)
  } >iram0_0_seg

  /* This section is required to skip .iram0.text area because iram0_0_seg
   * and dram0_0_seg reflect the same address space on different buses.
   */

  .dram0.dummy (NOLOAD):
  {
    . = ORIGIN(dram0_0_seg) + _iram_end - _iram_start;
  } >dram0_0_seg

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
  } >dram0_0_seg

  .noinit (NOLOAD):
  {
    /* This section contains data that is not initialized during load,
     * or during the application's initialization sequence.
     */

    *(.noinit)
    *(.noinit.*)
  } >dram0_0_seg

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

    *libarch.a:brownout.*(.rodata .rodata.*)
    *libarch.a:cpu.*(.rodata .rodata.*)
    *libarch.a:gpio_hal.*(.rodata .rodata.*)
    *libarch.a:periph_ctrl.*(.rodata .rodata.*)
    *libarch.a:rtc_clk.*(.rodata .rodata.*)
    *libarch.a:rtc_sleep.*(.rodata .rodata.*)
    *libarch.a:rtc_time.*(.rodata .rodata.*)
    *libarch.a:systimer.*(.rodata .rodata.*)
    *libarch.a:systimer_hal.*(.rodata .rodata.*)
    *libarch.a:uart_hal_iram.*(.rodata .rodata.*)
    *libarch.a:wdt_hal_iram.*(.rodata .rodata.*)
    *libsched.a:irq_dispatch.*(.rodata .rodata.*)
    . = ALIGN(4);
    _edata = ABSOLUTE(.);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
  } >dram0_0_seg

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
  } >default_code_seg

  .flash_rodata_dummy (NOLOAD):
  {
    /* Start at the same alignment constraint than .flash.text */

    . = ALIGN(ALIGNOF(.flash.text));

    /* Create an empty gap as big as .flash.text section */

    . = . + SIZEOF(.flash.text);

    /* Prepare the alignment of the section above. Few bytes (0x20) must be
     * added for the mapping header.
     */

    . = ALIGN(0x10000) + 0x20;
  } >default_rodata_seg

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
  } >default_rodata_seg

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
  } >iram0_0_seg

  .iram0.bss (NOLOAD) :
  {
    . = ALIGN(16);
    *(.iram.bss)
    *(.iram.bss*)

    . = ALIGN(16);
    _iram_end = ABSOLUTE(.);
  } >iram0_0_seg

  /* RTC fast memory holds RTC wake stub code !*/

  .rtc.text :
  {
    . = ALIGN(4);
    *(.rtc.literal .rtc.text)
  } >rtc_seg

  /* This section is required to skip rtc.text area because the text and
   * data segments reflect the same address space on different buses.
   */

  .rtc.dummy :
  {
    . = SIZEOF(.rtc.text);
  } >rtc_seg

  /* RTC BSS section. */

  .rtc.bss (NOLOAD) :
  {
    *(.rtc.bss)
  } >rtc_seg

  /* RTC data section holds RTC wake stub data/rodata. */

  .rtc.data :
  {
    *(.rtc.data)
    *(.rtc.rodata)

   /* Whatever is left from the RTC memory is used as a special heap. */

    _srtcheap = ABSOLUTE(.);

  } >rtc_seg
}

