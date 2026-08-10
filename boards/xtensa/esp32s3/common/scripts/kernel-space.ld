/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/kernel-space.ld
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

/* Provide these so there is no need for using config files for this */

#include "esp32s3_aliases.ld"

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
  /* Send .iram0 code to iram */

  .iram0.vectors :
  {
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

    __vectors_end = ABSOLUTE(.);

    *(.entry.text)
    *(.init.literal)
    *(.init)

    _init_end = ABSOLUTE(.);
  } >KIRAM

  .iram0.text :
  {
    /* Code marked as running out of IRAM */

    _iram_text_start = ABSOLUTE(.);
    *(.iram1 .iram1.*)

    *librtc.a:(.literal .text .literal.* .text.*)
    *libkarch.a:esp32s3_spiflash.*(.literal .text .literal.* .text.*)
    *libkarch.a:cache_hal.*(.literal .text .literal.* .text.*)
    *libkarch.a:esp_rom_cache_esp32s2_esp32s3.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_cpupause.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_copystate.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_interruptcontext.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_testset.*(.literal .text .literal.* .text.*)
    *libkarch.a:esp_app_desc.*(.literal .text .literal.* .text.*)
    *libkarch.a:intr_alloc.*(.literal.esp_intr_get_intno .text.esp_intr_get_intno)
    *libkarch.a:xtensa_intr.*(.literal.xt_get_interrupt_handler .text.xt_get_interrupt_handler)
    *libkarch.a:xtensa_intr.*(.literal.xt_get_interrupt_handler_arg .text.xt_get_interrupt_handler_arg)
    *libkarch.a:esp_spiflash.*(.literal .text .literal.* .text.*)
    *libkarch.a:esp_flash_api.*(.text .text.* .literal .literal.*)
    *libkarch.a:esp_flash_spi_init.*(.text .text.* .literal .literal.*)
    *libkarch.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_encrypt_hal_iram.*(.text .text.* .literal .literal.*)
    *libkarch.a:spi_flash_hal_gpspi.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_chip*.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_os_func_noos.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_os_func_app.*(.literal .literal.* .text .text.*)
    *libkarch.a:flash_brownout_hook.*(.literal .literal.* .text .text.*)
    *libkarch.a:esp_cache_utils.*(.literal .literal.* .text .text.*)
    *libkarch.a:esp_cache.*(.literal .literal.* .text .text.*)
    *libkarch.a:cache_utils.*(.literal .literal.* .text .text.*)
    *libkarch.a:memspi_host_driver.*(.literal .literal.* .text .text.*)

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
    /* Simple boot runs bootloader_init() and map_rom_segments() before any
     * flash mapping exists, so everything they reach has to be resident.
     */

    *libkarch.a:*esp_loader.*(.literal .text .literal.* .text.*)
    *libkarch.a:*brownout_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*cpu.*(.text .text.* .literal .literal.*)
    *libkarch.a:*gpio_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*periph_ctrl.*(.text .text.* .literal .literal.*)
    *libkarch.a:*clk.*(.text .text.* .literal .literal.*)
    *libkarch.a:critical_section.*(.literal .literal.* .text .text.*)
    *libkarch.a:os.*(.literal.nuttx_enter_critical .text.nuttx_enter_critical)
    *libkarch.a:os.*(.literal.nuttx_exit_critical .text.nuttx_exit_critical)
    *libkarch.a:*sleep_modes.*(.literal.esp_sleep_sub_mode_force_disable* .text.esp_sleep_sub_mode_force_disable*)
    *libkarch.a:*efuse_hal.*(.literal.is_eco0 .text.is_eco0)
    *libkarch.a:*efuse_utility.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_clk.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_clk_tree.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_clk_tree_common.*(.text .text.* .literal .literal.*)
    *libkarch.a:*clk_tree_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*rtc_init.*(.text .text.* .literal .literal.*)
    *libkarch.a:*rtc_clk.*(.text .text.* .literal .literal.*)
    *libkarch.a:*rtc_clk_init.*(.text .text.* .literal .literal.*)
    *libkarch.a:*rtc_sleep.*(.text .text.* .literal .literal.*)
    *libkarch.a:*rtc_time.*(.text .text.* .literal .literal.*)
    *libkarch.a:*regi2c_ctrl.*(.text .text.* .literal .literal.*)
    *libkarch.a:*uart_hal_iram.*(.text .text.* .literal .literal.*)
    *libkarch.a:*wdt_hal_iram.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_banner_wrap.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_init.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_common.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_common_loader.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_console.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_console_loader.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_esp32s3.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_flash.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_flash_config_esp32s3.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_clock_init.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_clock_loader.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_efuse.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_panic.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_mem.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_random.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_random*.*(.literal.bootloader_random_disable .text.bootloader_random_disable)
    *libkarch.a:*bootloader_random*.*(.literal.bootloader_random_enable .text.bootloader_random_enable)
    *libkarch.a:*bootloader_random_esp32s3.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_image_format.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_soc.*(.text .text.* .literal .literal.*)
    *libkarch.a:*bootloader_sha.*(.text .text.* .literal .literal.*)
    *libkarch.a:*flash_encrypt.*(.text .text.* .literal .literal.*)
    *libkarch.a:*cache_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*uart_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*mpu_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*mmu_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*efuse_hal.*(.text .text.* .literal .literal.*)
    *libkarch.a:*uart_periph.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_rom_uart.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_rom_sys.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_rom_spiflash.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_rom_wdt.*(.text .text.* .literal .literal.*)
    *libkarch.a:*log.*(.text .text.* .literal .literal.*)
    *libkarch.a:*log_lock.*(.literal .literal.* .text .text.*)
    *libkarch.a:*log_print.*(.literal .literal.* .text .text.*)
    *libkarch.a:*log_timestamp.*(.literal.esp_log_early_timestamp .text.esp_log_early_timestamp)
    *libkarch.a:*log_timestamp.*(.literal.esp_log_timestamp .text.esp_log_timestamp)
    *libkarch.a:*log_timestamp_common.*(.literal .literal.* .text .text.*)
    *libkarch.a:*log_write.*(.literal.esp_log_write .text.esp_log_write)
    *libkarch.a:*log_write.*(.literal.esp_log_writev .text.esp_log_writev)
    *libkarch.a:*cpu_region_protect.*(.text .text.* .literal .literal.*)
    *libkarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.literal .text .literal.* .text.*)
    *libkarch.a:*flash_qio_mode.*(.text .text.* .literal .literal.*)
    *libkarch.a:*spi_flash_wrap.*(.text .text.* .literal .literal.*)

#ifdef CONFIG_ESP32S3_FLASH_MODE_OCT
    /* Octal (OPI) bring-up only.  A board with quad flash never runs these,
     * and pinning them would cost it IRAM for nothing.
     */

    *libkarch.a:*mspi_timing_tuning.*(.text .text.* .literal .literal.*)
    *libkarch.a:*mspi_timing_config.*(.text .text.* .literal .literal.*)
    *libkarch.a:*spi_flash_oct_flash_init.*(.text .text.* .literal .literal.*)
    *libkarch.a:*spi_flash_hpm_enable.*(.text .text.* .literal .literal.*)
#endif

    *libkarch.a:esp_spiflash.*(.literal .text .literal.* .text.*)
    *libkarch.a:esp_flash_api.*(.text .text.* .literal .literal.*)
    *libkarch.a:esp_flash_spi_init.*(.text .text.* .literal .literal.*)
    *libkarch.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_encrypt_hal_iram.*(.text .text.* .literal .literal.*)
    *libkarch.a:spi_flash_hal_gpspi.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_chip*.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_os_func_noos.*(.literal .literal.* .text .text.*)
    *libkarch.a:spi_flash_os_func_app.*(.literal .literal.* .text .text.*)
#endif

    *libsched.a:sched_suspendscheduler.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_note.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_thistask.*(.literal .text .literal.* .text.*)
    *libsched.a:spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_csection.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_dispatch.*(.literal .text .literal.* .text.*)

    *libc.a:*lib_instrument.*(.text .text.* .literal .literal.*)

    *(.wifirxiram .wifirxiram.*)
    *(.wifi0iram  .wifi0iram.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)

    /* align + add 16B for CPU dummy speculative instr. fetch */

    . = ALIGN(4) + 16;

    _iram_text_end = ABSOLUTE(.);

    _iram_end = ABSOLUTE(.);
  } >KIRAM

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
    *(.bss)
    *(.bss.*)
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    *libkarch.a:esp32s3_spiflash.*(.bss  .bss.*  COMMON)
    *libkarch.a:xtensa_cpupause.*(.bss  .bss.*  COMMON)
    *libkarch.a:xtensa_copystate.*(.bss  .bss.*  COMMON)
    *libkarch.a:xtensa_interruptcontext.*(.bss  .bss.*  COMMON)
    *libkarch.a:xtensa_testset.*(.bss  .bss.*  COMMON)

    *libsched.a:sched_suspendscheduler.*(.bss  .bss.*  COMMON)
    *libsched.a:sched_thistask.*(.bss  .bss.*  COMMON)
    *libsched.a:sched_note.*(.bss  .bss.*  COMMON)
    *libsched.a:spinlock.*(.bss  .bss.*  COMMON)
    *libsched.a:irq_csection.*(.bss  .bss.*  COMMON)
    *libsched.a:irq_dispatch.*(.bss  .bss.*  COMMON)

    *libc.a:*lib_instrument.*(.bss  .bss.*  COMMON)

    . = ALIGN(8);
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
    /* .data initialized on power-up in ROMed configurations. */

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
    *libphy.a:(.rodata  .rodata.*)
    *libkarch.a:esp32s3_spiflash.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_cpupause.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_copystate.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_interruptcontext.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_testset.*(.rodata  .rodata.*)
    *libkarch.a:esp_spiflash.*(.rodata .rodata.*)
    *libkarch.a:esp_flash_api.*(.rodata .rodata.*)
    *libkarch.a:esp_flash_spi_init.*(.rodata .rodata.*)
    *libkarch.a:spi_flash_hal_iram.*(.rodata .rodata.*)
    *libkarch.a:spi_flash_encrypt_hal_iram.*(.rodata .rodata.*)
    *libkarch.a:spi_flash_hal_gpspi.*(.rodata .rodata.*)
    *libkarch.a:spi_flash_chip*.*(.rodata .rodata.*)
    *libkarch.a:spi_flash_wrap.*(.rodata .rodata.*)
    *libkarch.a:spi_flash_os_func_noos.*(.rodata .rodata.*)
    *libkarch.a:spi_flash_os_func_app.*(.rodata .rodata.*)
    *libkarch.a:flash_brownout_hook.*(.rodata .rodata.*)
    *libkarch.a:esp_cache.*(.rodata .rodata.*)
    *libkarch.a:cache_utils.*(.rodata .rodata.*)
    *libkarch.a:memspi_host_driver.*(.rodata .rodata.*)
    *libkarch.a:esp_mmu_map.*(.rodata .rodata.*)
    *libkarch.a:ext_mem_layout.*(.rodata .rodata.*)

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
    *libkarch.a:*esp_loader.*(.rodata .rodata.*)
    *libkarch.a:*brownout.*(.rodata .rodata.*)
    *libkarch.a:*cpu.*(.rodata .rodata.*)
    *libkarch.a:*gpio_hal.*(.rodata .rodata.*)
    *libkarch.a:*periph_ctrl.*(.rodata .rodata.*)
    *libkarch.a:*clk.*(.rodata .rodata.*)
    *libkarch.a:*efuse_utility.*(.rodata .rodata.*)
    *libkarch.a:critical_section.*(.rodata .rodata.*)
    *libkarch.a:os.*(.rodata.g_int_flags_count .rodata.g_int_flags)
    *libkarch.a:*sleep_modes.*(.rodata.esp_sleep_sub_mode_force_disable*)
    *libkarch.a:*rtc_time.*(.rodata .rodata.*)
    *libkarch.a:*regi2c_ctrl.*(.rodata .rodata.*)
    *libkarch.a:*uart_hal_iram.*(.rodata .rodata.*)
    *libkarch.a:*wdt_hal_iram.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_banner_wrap.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_init.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_common.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_common_loader.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_console.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_console_loader.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_esp32s3.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_flash.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_flash_config_esp32s3.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_clock_init.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_clock_loader.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_efuse.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_panic.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_mem.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_random.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_random_esp32s3.*(.rodata .rodata.*)
    *libkarch.a:*esp_image_format.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_soc.*(.rodata .rodata.*)
    *libkarch.a:*bootloader_sha.*(.rodata .rodata.*)
    *libkarch.a:*flash_encrypt.*(.rodata .rodata.*)
    *libkarch.a:*cache_hal.*(.rodata .rodata.*)
    *libkarch.a:*uart_hal.*(.rodata .rodata.*)
    *libkarch.a:*mpu_hal.*(.rodata .rodata.*)
    *libkarch.a:*mmu_hal.*(.rodata .rodata.*)
    *libkarch.a:*uart_periph.*(.rodata .rodata.*)
    *libkarch.a:*esp_rom_uart.*(.rodata .rodata.*)
    *libkarch.a:*esp_rom_sys.*(.rodata .rodata.*)
    *libkarch.a:*esp_rom_spiflash.*(.rodata .rodata.*)
    *libkarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.rodata .rodata.*)
    *libkarch.a:*esp_rom_wdt.*(.rodata .rodata.*)
    *libkarch.a:*efuse_hal.*(.rodata .rodata.*)
    *libkarch.a:*log.*(.rodata .rodata.*)
    *libkarch.a:*log_noos.*(.rodata .rodata.*)
    *libkarch.a:*cpu_region_protect.*(.rodata .rodata.*)
#ifdef CONFIG_ESP32S3_FLASH_MODE_OCT
    *libkarch.a:*mspi_timing_tuning.*(.rodata .rodata.*)
    *libkarch.a:*mspi_timing_config.*(.rodata .rodata.*)
#endif
    *libkarch.a:esp32s3_psram_quad.*(.rodata .rodata.*)
    *libkarch.a:esp32s3_psram_octal.*(.rodata .rodata.*)
#endif

    *libsched.a:sched_suspendscheduler.*(.rodata  .rodata.*)
    *libsched.a:sched_thistask.*(.rodata  .rodata.*)
    *libsched.a:sched_note.*(.rodata  .rodata.*)
    *libsched.a:spinlock.*(.rodata  .rodata.*)
    *libsched.a:irq_csection.*(.rodata  .rodata.*)
    *libsched.a:irq_dispatch.*(.rodata  .rodata.*)

    *libc.a:*lib_instrument.*(.rodata  .rodata.*)
    *libc.a:arch_atomic.*(.rodata .rodata.*)

    . = ALIGN(4);
    _edata = ABSOLUTE(.);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
  } >KDRAM

  .flash.text :
  {
    _stext = .;
    _instruction_reserved_start = ABSOLUTE(.);

    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)

    /* CPU will try to prefetch up to 16 bytes of instructions.
     * This means that any configuration (e.g. MMU, PMS) must allow
     * safe access to up to 16 bytes after the last real instruction, add
     * dummy bytes to ensure this
     */

    . += 16;

    _instruction_reserved_end = ABSOLUTE(.);
    _etext = .;
  } >KIROM

  .flash_rodata_dummy (NOLOAD) :
  {
    /* This dummy section represents the .flash.text section but in default_rodata_seg.
     * Thus, it must have its alignment and (at least) its size.
     */

    /* Start at the same alignment constraint than .flash.text */

    . = ALIGN(ALIGNOF(.flash.text));

    /* Create an empty gap as big as .flash.text section */

    . = SIZEOF(.flash.text);

    /* Prepare the alignment of the section above. Few bytes (0x20) must be
     * added for the mapping header.
     */

    . = ALIGN(0x10000) + 0x20;
    _rodata_reserved_start = .;
  } >KDROM

  .flash.rodata : ALIGN(0x10)
  {
    _rodata_reserved_start = ABSOLUTE(.);
    _srodata = ABSOLUTE(.);

    /* !DO NOT PUT ANYTHING BEFORE THIS! */
    /* Should be the first.  App version info. */
    *(.rodata_desc .rodata_desc.*)

    *(.rodata)
    *(.rodata.*)
    *(.rodata_wlog_verbose.*)
    *(.rodata_wlog_debug.*)
    *(.rodata_wlog_info.*)
    *(.rodata_wlog_warning.*)
    *(.rodata_wlog_error.*)
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

    . = ALIGN(4);

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
    _rodata_reserved_end = ABSOLUTE(.);
    . = ALIGN(4);

    /* C++ constructor tables. Those are initialized by lib_cxx_initialize
     * which calls constructor functions that may be on common source code.
     */

    . = ALIGN(4);
    _sinit = ABSOLUTE(.);
    __init_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .ctors SORT(.ctors.*)))
    __init_array_end = ABSOLUTE(.);
    _einit = ABSOLUTE(.);

    /* Addresses of memory regions reserved via SOC_RESERVE_MEMORY_REGION() */

    . = ALIGN(4);
    soc_reserved_memory_region_start = ABSOLUTE(.);
    KEEP (*(.reserved_memory_address))
    soc_reserved_memory_region_end = ABSOLUTE(.);

    /* System init functions registered via ESP_SYSTEM_INIT_FN */

    . = ALIGN(4);
    _esp_system_init_fn_array_start = ABSOLUTE(.);
    KEEP (*(SORT_BY_INIT_PRIORITY(.esp_system_init_fn.*)))
    _esp_system_init_fn_array_end = ABSOLUTE(.);

    . = ALIGN(4);
  } >KDROM

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
  /* Simple boot has no second-stage bootloader:  __start() maps the flash
   * itself, from these, and bootloader_init() clears the BSS.
   */

  _image_irom_vma = ADDR(.flash.text);
  _image_irom_lma = LOADADDR(.flash.text);
  _image_irom_size = LOADADDR(.flash.text) + SIZEOF(.flash.text) -
                     _image_irom_lma;

  _image_drom_vma = ADDR(.flash.rodata);
  _image_drom_lma = LOADADDR(.flash.rodata);
  _image_drom_size = LOADADDR(.flash.rodata) + SIZEOF(.flash.rodata) -
                     _image_drom_lma;

  _bss_start = _sbss;
  _bss_end = _ebss;
#endif

  /*
   * This section holds RTC data that should have fixed addresses.
   * The data are not initialized at power-up and are retained during deep sleep.
   */

  .rtc_reserved (NOLOAD):
  {
    . = ALIGN(4);
    _rtc_reserved_start = ABSOLUTE(.);

    /* New data can only be added here to ensure existing data are not moved.
     * Because data have adhered to the end of the segment and code is relied
     * on it.
     * >> put new data here <<
     */

    *(.rtc_timer_data_in_rtc_mem .rtc_timer_data_in_rtc_mem.*)
    KEEP(*(.bootloader_data_rtc_mem .bootloader_data_rtc_mem.*))
    _rtc_reserved_end = ABSOLUTE(.);
  } > rtc_reserved_seg
}
