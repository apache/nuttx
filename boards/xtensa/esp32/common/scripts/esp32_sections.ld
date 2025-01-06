/****************************************************************************
 * boards/xtensa/esp32/common/scripts/simple_boot_sections.ld
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
#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
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

  .iram0.vectors :
  {
    /* Vectors go to IRAM */

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

    _iram_text_start = ABSOLUTE(.);
    *(.iram1 .iram1.*)
    esp32_start.*(.literal .text .literal.* .text.*)
    *libphy.a:(.literal .text .literal.* .text.*)
    *librtc.a:(.literal .text .literal.* .text.*)
    *libpp.a:(.literal .text .literal.* .text.*)
    *libhal.a:(.literal .text .literal.* .text.*)
    *libarch.a:*esp_loader.*(.literal .text .literal.* .text.*)
    *libarch.a:*brownout_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*cpu.*(.text .text.* .literal .literal.*)
    *libarch.a:*gpio_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*periph_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*efuse_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree_common.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk_tree_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk_init.*(.text .text.* .literal .literal.*)
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
    *libarch.a:*bootloader_esp32.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash_config_esp32.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_efuse.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_panic.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_mem.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random*.*(.literal.bootloader_random_disable .text.bootloader_random_disable)
    *libarch.a:*bootloader_random*.*(.literal.bootloader_random_enable .text.bootloader_random_enable)
    *libarch.a:*bootloader_random_esp32.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_image_format.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_soc.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_sha.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*mpu_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*mmu_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_periph.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_uart.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_sys.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_spiflash.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_wdt.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_efuse_fields.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_efuse_api_key.*(.text .text.* .literal .literal.*)
    *libarch.a:*log.*(.text .text.* .literal .literal.*)
    *libarch.a:*log_noos.*(.text .text.* .literal .literal.*)
    *libarch.a:*cpu_region_protect.*(.text .text.* .literal .literal.*)

#ifdef CONFIG_STACK_CANARIES
    *libc.a:lib_stackchk.*(.literal .text .literal.* .text.*)
#endif
    *libarch.a:esp32_cpuindex.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_irq.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_spicache.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_user.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_assert.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpuint.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpupause.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_irqdispatch.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_modifyreg32.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_saveusercontext.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_testset.*(.literal .text .literal.* .text.*)

    *libc.a:lib_assert.*(.literal .text .literal.* .text.*)
    *libc.a:lib_utsname.*(.literal .text .literal.* .text.*)
    *libc.a:sq_remlast.*(.literal .text .literal.* .text.*)

    *libdrivers.a:syslog_flush.*(.literal .text .literal.* .text.*)

    *libsched.a:assert.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_csection.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_dispatch.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:panic_notifier.*(.literal.panic_notifier_call_chain .text.panic_notifier_call_chain)
    *libsched.a:sched_gettcb.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_lock.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_note.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_suspendscheduler.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_thistask.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_unlock.*(.literal .text .literal.* .text.*)
    *libsched.a:spinlock.*(.literal .text .literal.* .text.*)

    *(.wifirxiram .wifirxiram.*)
    *(.wifi0iram .wifi0iram.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *(.phyiram .phyiram.*)

    . = ALIGN(4) + 16;

    _iram_text_end = ABSOLUTE(.);

    /* IRAM heap starts at the end of iram0_0_seg */

    . = ALIGN (4);
    _siramheap = ABSOLUTE(.);
  } >iram0_0_seg AT>ROM

  /* Shared RAM */

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

  .dram0.bss (NOLOAD) :
  {
    /* .bss initialized on power-up */

    . = ALIGN (8);
    _bss_start = ABSOLUTE(.);
    _sbss = ABSOLUTE(.);
    *(EXCLUDE_FILE(*libble_app.a *libbt.a *libbtdm_app.a *libnimble.a) .bss EXCLUDE_FILE(*libble_app.a *libbt.a *libbtdm_app.a *libnimble.a) .bss.*)
    *(.ext_ram.bss*)
    *(EXCLUDE_FILE(*libble_app.a *libbt.a *libbtdm_app.a *libnimble.a) COMMON)
    . = ALIGN(4);
    _bt_controller_bss_start = ABSOLUTE(.);
    *libble_app.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_controller_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_common_start = ABSOLUTE(.);
    *libble_app.a:(COMMON)
    . = ALIGN(4);
    _bt_controller_common_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_bss_start = ABSOLUTE(.);
    *libbt.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_common_start = ABSOLUTE(.);
    *libbt.a:(COMMON)
    . = ALIGN(4);
    _bt_common_end = ABSOLUTE(.);
    . = ALIGN(4);
    _btdm_bss_start = ABSOLUTE(.);
    *libbtdm_app.a:(.bss .bss.*)
    . = ALIGN(4);
    _btdm_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _btdm_common_start = ABSOLUTE(.);
    *libbtdm_app.a:(COMMON)
    . = ALIGN(4);
    _btdm_common_end = ABSOLUTE(.);
    . = ALIGN (8);
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
#ifdef CONFIG_STACK_CANARIES
    *libc.a:lib_stackchk.*(.bss  .bss.*  COMMON)
#endif
    *libarch.a:esp32_spicache.*(.bss  .bss.*  COMMON)
    *libarch.a:esp32_spiflash.*(.bss  .bss.*  COMMON)
    *libarch.a:xtensa_cpupause.*(.bss  .bss.*  COMMON)
    *libarch.a:xtensa_copystate.*(.bss  .bss.*  COMMON)
    *libarch.a:xtensa_interruptcontext.*(.bss  .bss.*  COMMON)
    *libarch.a:xtensa_testset.*(.bss  .bss.*  COMMON)

    *libsched.a:sched_suspendscheduler.*(.bss  .bss.*  COMMON)
    *libsched.a:sched_thistask.*(.bss  .bss.*  COMMON)
    *libsched.a:sched_note.*(.bss  .bss.*  COMMON)
    *libsched.a:spinlock.*(.bss  .bss.*  COMMON)
    *libsched.a:irq_csection.*(.bss  .bss.*  COMMON)
    *libsched.a:irq_dispatch.*(.bss  .bss.*  COMMON)

    . = ALIGN(8);
    _ebss = ABSOLUTE(.);
    _bss_end = ABSOLUTE(.);
  } >dram0_0_seg

  .dram0.data :
  {
    /* .data initialized on power-up in ROMed configurations. */

    _sdata = ABSOLUTE(.);
    _data_start = ABSOLUTE(.);
    _bt_data_start = ABSOLUTE(.);
    *libbt.a:(.data .data.*)
    . = ALIGN (4);
    _bt_data_end = ABSOLUTE(.);
    _btdm_data_start = ABSOLUTE(.);
    *libbtdm_app.a:(.data .data.*)
    . = ALIGN (4);
    _btdm_data_end = ABSOLUTE(.);
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
    esp32_start.*(.rodata  .rodata.*)
    *libphy.a:(.rodata  .rodata.*)
#ifdef CONFIG_STACK_CANARIES
    *libc.a:lib_stackchk.*(.rodata  .rodata.*)
#endif
    *libarch.a:xtensa_context.*(.rodata  .rodata.*)
    *libarch.a:xtensa_copystate.*(.rodata  .rodata.*)
    *libarch.a:xtensa_cpupause.*(.rodata  .rodata.*)
    *libarch.a:xtensa_testset.*(.rodata  .rodata.*)

    *libdrivers.a:syslog_channel.*(.rodata  .rodata.*)

    *libsched.a:sched_suspendscheduler.*(.rodata  .rodata.*)
    *libsched.a:sched_thistask.*(.rodata  .rodata.*)
    *libsched.a:sched_note.*(.rodata  .rodata.*)
    *libsched.a:spinlock.*(.rodata  .rodata.*)
    *libsched.a:irq_csection.*(.rodata  .rodata.*)
    *libsched.a:irq_dispatch.*(.rodata  .rodata.*)

    *libarch.a:esp32_spicache.*(.rodata  .rodata.*)
    *libarch.a:esp32_spiflash.*(.rodata  .rodata.*)
    *libarch.a:*esp_loader.*(.rodata .rodata.*)
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
    *libarch.a:*bootloader_esp32.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash_config_esp32.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_init.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_efuse.*(.rodata .rodata.*)
    *libarch.a:*bootloader_panic.*(.rodata .rodata.*)
    *libarch.a:*bootloader_mem.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random_esp32.*(.rodata .rodata.*)
    *libarch.a:*esp_image_format.*(.rodata .rodata.*)
    *libarch.a:*bootloader_soc.*(.rodata .rodata.*)
    *libarch.a:*bootloader_sha.*(.rodata .rodata.*)
    *libarch.a:*uart_hal.*(.rodata .rodata.*)
    *libarch.a:*mpu_hal.*(.rodata .rodata.*)
    *libarch.a:*mmu_hal.*(.rodata .rodata.*)
    *libarch.a:*efuse_hal.*(.rodata .rodata.*)
    *libarch.a:*uart_periph.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_uart.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_sys.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_spiflash.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_wdt.*(.rodata .rodata.*)
    *libarch.a:*esp_efuse_fields.*(.rodata .rodata.*)
    *libarch.a:*esp_efuse_api_key.*(.rodata .rodata.*)
    *libarch.a:*log.*(.rodata .rodata.*)
    *libarch.a:*log_noos.*(.rodata .rodata.*)
    *libarch.a:*cpu_region_protect.*(.rodata .rodata.*)
    _edata = ABSOLUTE(.);
    _data_end = ABSOLUTE(.);

    . = ALIGN(4);

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

#ifndef CONFIG_ESP32_APP_FORMAT_MCUBOOT
  .flash.rodata_dummy (NOLOAD) :
  {
    . = ALIGN(0x10000);
  } > ROM
#endif

  .flash.rodata : ALIGN(0x10000)
  {
    _rodata_reserved_start = ABSOLUTE(.);
    . = ALIGN(4);
    _srodata = ABSOLUTE(.);
    *(EXCLUDE_FILE (*libarch.a:esp32_spiflash.* esp32_start.*
                    *libarch.a:*esp_loader.*
                    *libarch.a:*uart_hal.*
                    *libarch.a:*mmu_hal.*
                    ) .rodata)
    *(EXCLUDE_FILE (*libarch.a:esp32_spiflash.* esp32_start.*
                    *libarch.a:*esp_loader.*
                    *libarch.a:*uart_hal.*
                    *libarch.a:*mmu_hal.*
                    ) .rodata.*)
    *(.srodata.*)
    *(.rodata)
    *(.rodata.*)
#ifdef CONFIG_ESP32_WIRELESS
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

    /* Addresses of memory regions reserved via SOC_RESERVE_MEMORY_REGION() */

    soc_reserved_memory_region_start = ABSOLUTE(.);
    KEEP (*(.reserved_memory_address))
    soc_reserved_memory_region_end = ABSOLUTE(.);

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
  } >default_rodata_seg AT>ROM
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

#ifndef CONFIG_ESP32_APP_FORMAT_MCUBOOT
  .flash.text_dummy (NOLOAD) :
  {
    . += SIZEOF(.flash.rodata);
    . = ALIGN(0x10000);
  } >default_code_seg AT> ROM
#endif

  .flash.text : ALIGN(0x00010000)
  {
    _stext = .;
    _text_start = ABSOLUTE(.);
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)

    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifi0iram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifi0iram.*)
    *(.wifiextrairam .wifiextrairam.*)
    *(EXCLUDE_FILE(*libpp.a) .wifiorslpiram EXCLUDE_FILE(*libpp.a) .wifiorslpiram.*)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifirxiram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifislprxiram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifislprxiram.*)

    . = ALIGN(4);

    . += 16;

    _text_end = ABSOLUTE(.);
    _etext = .;
  } >default_code_seg AT>ROM

  /* External memory bss, from any global variable with EXT_RAM_ATTR attribute */

  .extmem.bss (NOLOAD) :
  {
    _sbss_extmem = ABSOLUTE(.);
    *(.extmem.bss .extmem.bss.*)
    . = ALIGN(4);
    _ebss_extmem = ABSOLUTE(.);
  } >extmem_seg

  .rtc.text :
  {
    . = ALIGN(4);
    *(.rtc.literal .rtc.text)
  } >rtc_iram_seg AT>ROM

  .rtc.data :
  {
    *(.rtc.data)
    *(.rtc.data.*)
    *(.rtc.rodata)
    *(.rtc.rodata.*)

   /* Whatever is left from the RTC memory is used as a special heap. */

    . = ALIGN (4);
    _srtcheap = ABSOLUTE(.);
  } >rtc_slow_seg AT>ROM
}
