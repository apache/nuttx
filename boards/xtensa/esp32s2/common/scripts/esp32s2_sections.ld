/****************************************************************************
 * boards/xtensa/esp32s2/common/scripts/esp32s2_sections.ld
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

/* Default entry point: */

ENTRY(__start);

SECTIONS
{
#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
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

    LONG(ADDR(.iram0.vectors))
    LONG(LOADADDR(.iram0.vectors))
    LONG(LOADADDR(.iram0.text) + SIZEOF(.iram0.text) - LOADADDR(.iram0.vectors))

    /* DRAM metadata:
     * - Destination address (VMA) for DRAM region
     * - Flash offset (LMA) for start of DRAM region
     * - Size of DRAM region
     */

    LONG(ADDR(.dram0.data))
    LONG(LOADADDR(.dram0.data))
    LONG(SIZEOF(.dram0.data))
  } >metadata
#endif

  /* Send .iram0 code to iram */

  .iram0.vectors : ALIGN(4)
  {
    _iram_start = ABSOLUTE(.);

    /* Vectors go to IRAM. */

    _init_start = ABSOLUTE(.);

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
    KEEP (*(.nmi_vector.text));
    . = 0x300;
    KEEP (*(.kernel_exception_vector.text));
    . = 0x340;
    KEEP (*(.user_exception_vector.text));
    . = 0x3c0;
    KEEP (*(.double_exception_vector.text));
    . = 0x400;
    *(.*_vector.literal)

    . = ALIGN (16);
    *(.entry.text)
    *(.init.literal)
    *(.init)
    _init_end = ABSOLUTE(.);
  } >iram0_0_seg AT>ROM

  .iram0.text : ALIGN(4)
  {
    /* Code marked as running out of IRAM */

    *(.iram1 .iram1.*)
    . = ALIGN (4);
    esp32s2_start.*(.literal .text .literal.* .text.*)
    esp32s2_region.*(.text .text.* .literal .literal.*)

    *libarch.a:*esp_loader.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s2_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:*brownout_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*cpu.*(.text .text.* .literal .literal.*)
    *libarch.a:*gpio_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*periph_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*efuse_hal.*(.literal.is_eco0 .text.is_eco0)
    *libarch.a:*efuse_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree_common.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk_tree_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_sleep.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_time.*(.text .text.* .literal .literal.*)
    *libarch.a:*regi2c_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:*wdt_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_banner_wrap.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_common.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_common_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_console.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_console_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_esp32s2.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash_config_esp32s2.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_efuse.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_panic.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_mem.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random*.*(.literal.bootloader_random_disable .text.bootloader_random_disable)
    *libarch.a:*bootloader_random*.*(.literal.bootloader_random_enable .text.bootloader_random_enable)
    *libarch.a:*bootloader_random_esp32s2.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_image_format.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_soc.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_sha.*(.text .text.* .literal .literal.*)
    *libarch.a:*flash_encrypt.*(.text .text.* .literal .literal.*)
    *libarch.a:*cache_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*mpu_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*mmu_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_periph.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_uart.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_sys.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_spiflash.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_wdt.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_efuse_fields.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_efuse_api_key.*(.text .text.* .literal .literal.*)
    *libarch.a:*log.*(.text .text.* .literal .literal.*)
    *libarch.a:*log_noos.*(.text .text.* .literal .literal.*)
    *libarch.a:*cpu_region_protect.*(.text .text.* .literal .literal.*)

    *libc.a:sq_remlast.*(.literal .text .literal.* .text.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifi0iram  .wifi0iram.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *(.phyiram .phyiram.*)
    *(.wifiextrairam .wifiextrairam.*)

    /* align + add 16B for CPU dummy speculative instr. fetch */

    . = ALIGN(4) + 16;
    _iram_text = ABSOLUTE(.);
  } >iram0_0_seg AT>ROM

  /* Marks the end of IRAM code segment */

  .iram0.text_end (NOLOAD) :
  {
    . = ALIGN (4);
    _iram_end = ABSOLUTE(.);
  } >iram0_0_seg

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
    _bss_start = ABSOLUTE(.);
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
    _bss_end = ABSOLUTE(.);
  } >dram0_0_seg

  .noinit (NOLOAD):
  {
    /* This section contains data that is not initialized during load,
     * or during the application's initialization sequence.
     */

    . = ALIGN(4);
    *(.noinit)
    *(.noinit.*)
    . = ALIGN(4);
  } >dram0_0_seg

  .dram0.data : ALIGN(4)
  {
    /* .data initialized on power-up in ROMed configurations. */

    _data_start = ABSOLUTE(.);
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
    esp32s2_start.*(.rodata .rodata.*)
    esp32s2_region.*(.rodata .rodata.*)

    *libarch.a:*esp_loader.*(.rodata .rodata.*)
    *libarch.a:esp32s2_spiflash.*(.rodata .rodata.*)
    *libarch.a:*brownout.*(.rodata .rodata.*)
    *libarch.a:*cpu.*(.rodata .rodata.*)
    *libarch.a:*gpio_hal.*(.rodata .rodata.*)
    *libarch.a:*periph_ctrl.*(.rodata .rodata.*)
    *libarch.a:*clk.*(.rodata .rodata.*)
    *libarch.a:*esp_clk.*(.rodata .rodata.*)
    *libarch.a:*esp_clk_tree.*(.rodata .rodata.*)
    *libarch.a:*esp_clk_tree_common.*(.rodata .rodata.*)
    *libarch.a:*clk_tree_hal.*(.rodata .rodata.*)
    *libarch.a:*rtc_init.*(.rodata .rodata.*)
    *libarch.a:*rtc_clk.*(.rodata .rodata.*)
    *libarch.a:*rtc_clk_init.*(.rodata .rodata.*)
    *libarch.a:*rtc_sleep.*(.rodata .rodata.*)
    *libarch.a:*rtc_time.*(.rodata .rodata.*)
    *libarch.a:*regi2c_ctrl.*(.rodata .rodata.*)
    *libarch.a:*uart_hal_iram.*(.rodata .rodata.*)
    *libarch.a:*wdt_hal_iram.*(.rodata .rodata.*)
    *libarch.a:*bootloader_banner_wrap.*(.rodata .rodata.*)
    *libarch.a:*bootloader_init.*(.rodata .rodata.*)
    *libarch.a:*bootloader_common.*(.rodata .rodata.*)
    *libarch.a:*bootloader_common_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_console.*(.rodata .rodata.*)
    *libarch.a:*bootloader_console_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_esp32s2.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash_config_esp32s2.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_init.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_efuse.*(.rodata .rodata.*)
    *libarch.a:*bootloader_panic.*(.rodata .rodata.*)
    *libarch.a:*bootloader_mem.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random_esp32s2.*(.rodata .rodata.*)
    *libarch.a:*esp_image_format.*(.rodata .rodata.*)
    *libarch.a:*bootloader_soc.*(.rodata .rodata.*)
    *libarch.a:*bootloader_sha.*(.rodata .rodata.*)
    *libarch.a:*flash_encrypt.*(.rodata .rodata.*)
    *libarch.a:*cache_hal.*(.rodata .rodata.*)
    *libarch.a:*uart_hal.*(.rodata .rodata.*)
    *libarch.a:*mpu_hal.*(.rodata .rodata.*)
    *libarch.a:*mmu_hal.*(.rodata .rodata.*)
    *libarch.a:*uart_periph.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_uart.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_sys.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_spiflash.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_wdt.*(.rodata .rodata.*)
    *libarch.a:*esp_efuse_fields.*(.rodata .rodata.*)
    *libarch.a:*esp_efuse_api_key.*(.rodata .rodata.*)
    *libarch.a:*log.*(.rodata .rodata.*)
    *libarch.a:*log_noos.*(.rodata .rodata.*)
    *libarch.a:*cpu_region_protect.*(.rodata .rodata.*)

    . = ALIGN(4);
    _edata = ABSOLUTE(.);
    _data_end = ABSOLUTE(.);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
  } >dram0_0_seg AT>ROM

  _image_drom_vma = ADDR(.flash.rodata);
  _image_drom_lma = LOADADDR(.flash.rodata);
  _image_drom_size = LOADADDR(.flash.rodata) + SIZEOF(.flash.rodata) - _image_drom_lma;

  /* The alignment of the ".flash.rodata" output section is forced to
   * 0x00010000 (64KB) to ensure that it will be allocated at the beginning
   * of the next available Flash block.
   * This is required to meet the following constraint from the external
   * flash MMU:
   *    VMA % 64KB == LMA % 64KB
   * i.e. the lower 16 bits of both the virtual address (address seen by the
   * CPU) and the load address (physical address of the external flash) must
   * be equal.
   */

  .flash.rodata_dummy (NOLOAD) :
  {
    . = ALIGN(0x10000);
  } > ROM

  .flash.rodata : ALIGN(0x10000)
  {
    _rodata_reserved_start = ABSOLUTE(.);
    _srodata = ABSOLUTE(.);
    *(EXCLUDE_FILE (esp32s2_start.* esp32s2_region.*
                    *libarch.a:*esp_loader.*
                    *libarch.a:esp32s2_spiflash.*
                    *libarch.a:*cache_hal.* *libarch.a:*mmu_hal.*
                    *libarch.a:*mpu_hal.*) .rodata)
    *(EXCLUDE_FILE (esp32s2_start.* esp32s2_region.*
                    *libarch.a:*esp_loader.*
                    *libarch.a:esp32s2_spiflash.*
                    *libarch.a:*cache_hal.* *libarch.a:*mmu_hal.*
                    *libarch.a:*mpu_hal.*) .rodata.*)

#ifdef CONFIG_ESPRESSIF_WIRELESS
    *(.rodata_wlog_verbose.*)
    *(.rodata_wlog_debug.*)
    *(.rodata_wlog_info.*)
    *(.rodata_wlog_warning.*)
    *(.rodata_wlog_error.*)
#endif

    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    KEEP(*(.eh_frame))
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

    /* TLS data. */

    . = ALIGN(4);
    _thread_local_start = ABSOLUTE(.);
    _stdata = ABSOLUTE(.);
    *(.tdata .tdata.* .gnu.linkonce.td.*);
    _etdata = ABSOLUTE(.);
    _stbss = ABSOLUTE(.);
    *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
    _etbss = ABSOLUTE(.);

    _thread_local_end = ABSOLUTE(.);
    _rodata_reserved_end = ABSOLUTE(.);
  } >drom0_0_seg AT>ROM
  _rodata_reserved_align = ALIGNOF(.flash.rodata);

  _image_irom_vma = ADDR(.flash.text);
  _image_irom_lma = LOADADDR(.flash.text);
  _image_irom_size = LOADADDR(.flash.text) + SIZEOF(.flash.text) - _image_irom_lma;

  /* The alignment of the ".flash.text" output section is forced to
   * 0x00010000 (64KB) to ensure that it will be allocated at the beginning
   * of the next available Flash block.
   * This is required to meet the following constraint from the external
   * flash MMU:
   *    VMA % 64KB == LMA % 64KB
   * i.e. the lower 16 bits of both the virtual address (address seen by the
   * CPU) and the load address (physical address of the external flash) must
   * be equal.
   */

#ifndef CONFIG_ESP32S2_RUN_IRAM
  .flash.text_dummy (NOLOAD) :
  {
    /* This section is required to skip .flash.rodata area because irom0_0_seg
     * and drom0_0_seg reflect the same address space on different buses.
     */

    . += SIZEOF(.flash.rodata);
    . = ALIGN(0x10000);
  } >default_code_seg AT> ROM
#endif

  .flash.text : ALIGN(0x00010000)
  {
    _stext = .;

    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    . = ALIGN(4);

    . += 16;

    _etext = .;
  } >default_code_seg AT>ROM

  .rtc.text :
  {
    . = ALIGN(4);
    *(.rtc.literal .rtc.text)
  } >rtc_iram_seg AT>ROM

  .rtc.dummy (NOLOAD) :
  {
    /* This section is required to skip .rtc.text area because the text and
     * data segments reflect the same address space on different buses.
     */

    . = SIZEOF(.rtc.text);
  } >rtc_data_seg

  /* RTC BSS section. */

  .rtc.bss (NOLOAD) :
  {
    *(.rtc.bss)
  } >rtc_slow_seg

  .rtc.data :
  {
    . = ALIGN(4);
    *(.rtc.data)
    *(.rtc.data.*)
    *(.rtc.rodata)
    *(.rtc.rodata.*)

   /* Whatever is left from the RTC memory is used as a special heap. */

    . = ALIGN (4);
    _srtcheap = ABSOLUTE(.);
  } >rtc_slow_seg AT>ROM
}
