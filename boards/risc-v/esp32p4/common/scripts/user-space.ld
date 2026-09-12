/****************************************************************************
 * boards/risc-v/esp32p4/common/scripts/user-space.ld
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

/****************************************************************************
 * User image layout for the protected build.
 *
 * Two differences from the equivalent ESP32-C3 script are worth noting:
 *
 * 1. There is no UIRAM region.  On the C3, IRAM and DRAM are two views of
 *    the same internal SRAM at different addresses, so the user image needs
 *    an .iram0.text section and a .dram0.dummy section to skip past it.
 *    SRAM is unified here, and user code executes in place from flash, so
 *    .iram1 input sections are collected into the flash text output section
 *    and neither the IRAM output section nor the dummy is required.
 *
 * 2. The metadata block is 0x80 bytes rather than 0x18.  UDROM starts
 *    immediately after it and that address becomes a PMP TOR boundary, so
 *    it has to respect this SoC's 128-byte PMP granularity.
 ****************************************************************************/

#include <nuttx/config.h>

SECTIONS
{
  /* Consumed by esp_userspace.c in the kernel before any user code is
   * reachable.  The layout must match struct user_image_load_header_s.
   */

  .metadata :
  {
    /* DROM: destination address, flash offset, size */

    LONG(ADDR(.userspace))
    LONG(LOADADDR(.userspace))
    LONG(SIZEOF(.userspace) + SIZEOF(.rodata))

    /* IROM: destination address, flash offset, size */

    LONG(ADDR(.text))
    LONG(LOADADDR(.text))
    LONG(SIZEOF(.text))
  } >metadata

  /* Region boundaries, for the user image's own use */

  __ld_uirom_start = ORIGIN(UIROM);
  __ld_uirom_size  = LENGTH(UIROM);
  __ld_uirom_end   = ORIGIN(UIROM) + LENGTH(UIROM);
  __ld_udrom_start = ORIGIN(UDROM);
  __ld_udrom_size  = LENGTH(UDROM);
  __ld_udrom_end   = ORIGIN(UDROM) + LENGTH(UDROM);
  __ld_udram_start = ORIGIN(UDRAM);
  __ld_udram_size  = LENGTH(UDRAM);
  __ld_udram_end   = ORIGIN(UDRAM) + LENGTH(UDRAM);

  _eronly = LOADADDR(.data);

  .userspace : {
    *(.userspace)
  } >UDROM AT>ROM

  /* Output sections carry standard names rather than the Espressif-usual
   * ".flash.text" so that GDB's "add-symbol-file" can find a .text section
   * when symbols from both images are loaded together.
   */

  .rodata :
  {
    _srodata = ABSOLUTE(.);

    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)
    *(.rodata1)

    /* Collect the small read-only sections here too.  Left to themselves the
     * linker emits them as orphan output sections after .rodata, and their
     * unaligned sizes then push the load address of .data to an odd offset.
     */

    *(.srodata)
    *(.srodata.*)
    *(.sdata2)
    *(.sdata2.*)
    __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table)
    *(.gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    . = (. + 3) & ~ 3;
    __eh_frame = ABSOLUTE(.);
    KEEP(*(.eh_frame))
    . = (. + 7) & ~ 3;

    /* C++ constructor and destructor tables: classes would be sorted by
     * init priority, but NuttX does not use priorities here.
     */

    _sinit = ABSOLUTE(.);
    KEEP (*(.init_array .init_array.*))
    _einit = ABSOLUTE(.);

    KEEP (*(.fini_array .fini_array.*))

    *(.lit4)
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);

    /* esp_userspace() copies .data out of flash with bootloader_flash_read(),
     * which requires an aligned source address; .data's load address follows
     * the end of this section, so pad to 16 here and again at the end of
     * .data so that both the offset and the length are aligned.
     */

    . = ALIGN(16);
    _erodata = ABSOLUTE(.);
  } >UDROM AT>ROM

  .noinit (NOLOAD):
  {
    /* Data that is neither loaded nor initialised at startup */

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
    . = ALIGN(16);
    _edata = ABSOLUTE(.);
  } >UDRAM AT>ROM

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

    . = ALIGN (32);
    _ebss = ABSOLUTE(.);
  } >UDRAM

  /* Skip the flash space already consumed by .userspace and .rodata so that
   * the text output section's load address follows them in the image while
   * its virtual address sits in UIROM.
   */

  .flash_text_dummy (NOLOAD) : ALIGN(0x00010000)
  {
    . = SIZEOF(.userspace) + SIZEOF(.rodata);
  } >UIROM

  .text : ALIGN(0x00010000)
  {
    _stext = .;

    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */

    /* No UIRAM region on this SoC: anything the user image marked for
     * internal instruction RAM is executed in place from flash instead.
     */

    *(.iram1)
    *(.iram1.*)

    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    _etext = .;
  } >UIROM AT>ROM
}
