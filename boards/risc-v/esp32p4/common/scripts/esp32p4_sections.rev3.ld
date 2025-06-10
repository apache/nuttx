/****************************************************************************
 * boards/risc-v/esp32p4/common/scripts/esp32p4_sections.rev3.ld
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

/* Default entry point: */

ENTRY(__start);

SECTIONS
{

  /**
   * RTC fast memory holds RTC wake stub code,
   * including from any source file named rtc_wake_stub*.c
   */
  .rtc.text :
  {
    /* Align the start of RTC code region as per PMP granularity
     * this ensures we do not overwrite the permissions for the previous
     * region (ULP mem/RTC reserved) regardless of their end alignment
     */

 . = ALIGN(128);
 _rtc_fast_start = ABSOLUTE(.);

 . = ALIGN(128);
 _rtc_text_start = ABSOLUTE(.);
    *(.rtc.literal .rtc.text .rtc.text.*)
    *rtc_wake_stub*.*(.text .text.*)
    *(.rtc_text_end_test)
    /* Align the end of RTC code region as per PMP granularity */
    . = ALIGN(128);

 . = ALIGN(4);
 _rtc_text_end = ABSOLUTE(.);
  } > lp_ram_seg

#if CONFIG_P4_REV3_MSPI_CRASH_AFTER_POWER_UP_WORKAROUND
  .rtc.p4_rev3_mspi_workaround :
  {
    ALIGNED_SYMBOL(4, _rtc_p4_rev3_mspi_workaround_start)
    KEEP (*(.p4_rev3_mspi_workaround.rtc_text .p4_rev3_mspi_workaround.rtc_text.*))
    ALIGNED_SYMBOL(4, _rtc_p4_rev3_mspi_workaround_end)
  } > rev3_mspi_workaround_seg
#endif

  /**
   * This section located in RTC FAST Memory area.
   * It holds data marked with RTC_FAST_ATTR attribute.
   * See the file "esp_attr.h" for more information.
   */
  .rtc.force_fast :
  {

 . = ALIGN(4);
 _rtc_force_fast_start = ABSOLUTE(.);
    *(.rtc.force_fast .rtc.force_fast.*)

 . = ALIGN(4);
 _rtc_force_fast_end = ABSOLUTE(.);
  } > lp_ram_seg

  /**
   * RTC data section holds RTC wake stub
   * data/rodata, including from any source file
   * named rtc_wake_stub*.c and the data marked with
   * RTC_DATA_ATTR, RTC_RODATA_ATTR attributes.
   */
  .rtc.data :
  {
    _rtc_data_start = ABSOLUTE(.);
    *(.rtc.data .rtc.data.*)
    *(.rtc.rodata .rtc.rodata.*)
    *rtc_wake_stub*.*(.data .rodata .data.* .rodata.* .srodata.*)
    _rtc_data_end = ABSOLUTE(.);
  } > lp_ram_seg

  /* RTC bss, from any source file named rtc_wake_stub*.c */
  .rtc.bss (NOLOAD) :
  {
    _rtc_bss_start = ABSOLUTE(.);
    *rtc_wake_stub*.*(.bss .bss.* .sbss .sbss.*)
    *rtc_wake_stub*.*(COMMON)
    *(.rtc.bss)
    _rtc_bss_end = ABSOLUTE(.);
  } > lp_ram_seg

  /**
   * This section holds data that should not be initialized at power up
   * and will be retained during deep sleep.
   * User data marked with RTC_NOINIT_ATTR will be placed
   * into this section. See the file "esp_attr.h" for more information.
   */
  .rtc_noinit (NOLOAD):
  {

 . = ALIGN(4);
 _rtc_noinit_start = ABSOLUTE(.);
    *(.rtc_noinit .rtc_noinit.*)

 . = ALIGN(4);
 _rtc_noinit_end = ABSOLUTE(.);
  } > lp_ram_seg

  /**
   * This section located in RTC SLOW Memory area.
   * It holds data marked with RTC_SLOW_ATTR attribute.
   * See the file "esp_attr.h" for more information.
   */
  .rtc.force_slow :
  {

 . = ALIGN(4);
 _rtc_force_slow_start = ABSOLUTE(.);
    *(.rtc.force_slow .rtc.force_slow.*)

 . = ALIGN(4);
 _rtc_force_slow_end = ABSOLUTE(.);
  } > lp_ram_seg

  /**
   * This section holds RTC data that should have fixed addresses.
   * The data are not initialized at power-up and are retained during deep
   * sleep.
   */
  .rtc_reserved (NOLOAD):
  {

 . = ALIGN(4);
 _rtc_reserved_start = ABSOLUTE(.);
    KEEP(*(.bootloader_data_rtc_mem .bootloader_data_rtc_mem.*))
    *(.rtc_timer_data_in_rtc_mem .rtc_timer_data_in_rtc_mem.*)
    /**
     * New data can only be added here to ensure existing data are not moved.
     * Because data have adhered to the beginning of the segment and code is relied
     * on it.
     * >> put new data here <<
     */
    _rtc_reserved_end = ABSOLUTE(.);
  } > rtc_reserved_seg
  _rtc_reserved_length = _rtc_reserved_end - _rtc_reserved_start;
  _rtc_ulp_memory_start = _rtc_reserved_start + LENGTH(rtc_reserved_seg);
  ASSERT((_rtc_reserved_length <= LENGTH(rtc_reserved_seg)),
          "RTC reserved segment data does not fit.")
  /* Get size of rtc slow data based on rtc_data_location alias */
  _rtc_slow_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_slow_end - _rtc_data_start)
                        : (_rtc_force_slow_end - _rtc_force_slow_start);
  _rtc_fast_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_fast_end - _rtc_fast_start)
                        : (_rtc_noinit_end - _rtc_fast_start);
  ASSERT((_rtc_slow_length <= LENGTH(rtc_slow_seg)),
          "RTC_SLOW segment data does not fit.")
  ASSERT((_rtc_fast_length <= LENGTH(rtc_data_seg)),
          "RTC_FAST segment data does not fit.")

  .tcm.text :
  {
    /* Code marked as running out of TCM */
    _tcm_text_start = ABSOLUTE(.);
    *(.tcm.text .tcm.text.*)
    _tcm_text_end = ABSOLUTE(.);
  } > tcm_idram_seg
  .tcm.data :
  {
    _tcm_data_start = ABSOLUTE(.);
    *(.tcm.data .tcm.data.*)
    _tcm_data_end = ABSOLUTE(.);
  } > tcm_idram_seg

  .iram0.text :
  {
    _iram_start = ABSOLUTE(.);

    /* Vectors go to start of IRAM */

    ASSERT(ABSOLUTE(.) % 0x40 == 0, "vector address must be 64 byte aligned");

    KEEP(*(.exception_vectors_table.text));
    KEEP(*(.exception_vectors.text));

    /* Code marked as running out of IRAM */
    _iram_text_start = ABSOLUTE(.);

    *(.iram1)
    *(.iram1.*)

    *libcoexist.a:(.coexiram .coexiram.*)

    *libsched.a:irq_dispatch.*(.text .text.* .literal .literal.*)
    *libsched.a:sched_lock.*(.text .text.* .literal .literal.*)
    *libsched.a:sched_unlock.*(.text .text.* .literal .literal.*)

    *libarch.a:*(.text.esprv_intc_int_get_type .literal.esprv_intc_int_get_type)
    *libarch.a:*riscv_doirq.*(.text .text.* .literal .literal.*)
    *libarch.a:*brownout.*(.text .text.* .literal .literal.*)
    *libarch.a:*cpu.*(.text .text.* .literal .literal.*)
    *libarch.a:*gpio_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*periph_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree_common.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk_tree_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*pmu_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*modem_clock.*(.text .text.* .literal .literal.*)
    *libarch.a:*regi2c_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:*gpio_periph.*(.text .text.* .literal .literal.*)
    *libarch.a:*modem_clock_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_systimer.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_wdt.*(.text .text.* .literal .literal.*)
    *libarch.a:*ocode_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_regi2c_esp32p4.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*pmu_sleep.*(.text .text.* .literal .literal.*)
    *libarch.a:*pmu_param.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_time.*(.text .text.* .literal .literal.*)
    *libarch.a:*systimer.*(.text .text.* .literal .literal.*)
    *libarch.a:*systimer_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:*wdt_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_banner_wrap.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_common.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_common_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_console.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_console_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_esp32p4.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash_config_esp32p4.*(.text .text.* .literal .literal.*)
    *libarch.a:*flash_qio_mode.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_efuse.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_panic.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_mem.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random_esp32p4.*(.text .text.* .literal .literal.*)
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
    *libarch.a:*esp_efuse_fields.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_efuse_api_key.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_efuse_utility.*(.text .text.* .literal .literal.*)
    *libarch.a:*efuse_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*apm_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*log.*(.text .text.* .literal .literal.*)
    *libarch.a:*cpu_region_protect.*(.text .text.* .literal .literal.*)
    *libarch.a:*log_lock.*(.literal .literal.* .text .text.*)
    *libarch.a:*log_print.*(.literal .literal.* .text .text.*)
    *libarch.a:*log_timestamp.*(.literal.esp_log_early_timestamp .text.esp_log_early_timestamp)
    *libarch.a:*log_timestamp.*(.literal.esp_log_timestamp .text.esp_log_timestamp)
    *libarch.a:*log_timestamp_common.*(.literal .literal.* .text .text.*)
    *libarch.a:*log_write.*(.literal.esp_log_write .text.esp_log_write)
    *libarch.a:*log_write.*(.literal.esp_log_writev .text.esp_log_writev)
    *libarch.a:*rv_utils.*(.literal .literal.* .text .text.*)
    *libarch.a:*libarch.*(.literal .literal.* .text .text.*)
    *libarch.a:*riscv_modifyreg32.*(.literal .literal.* .text .text.*)
    *libarch.a:critical_section.*(.literal .literal.* .text .text.*)
    *libarch.a:os.*(.literal.nuttx_enter_critical .text.nuttx_enter_critical)
    *libarch.a:os.*(.literal.nuttx_exit_critical .text.nuttx_exit_critical)
    *libarch.a:spi_flash_hpm_enable.*(.literal .literal.* .text .text.*)
    *libarch.a:*sleep_modes.*(.literal.esp_sleep_pd_config* .text.esp_sleep_pd_config*)
    *libarch.a:esp_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:esp_flash_api.*(.text .text.* .literal .literal.*)
    *libarch.a:esp_flash_spi_init.*(.text .text.* .literal .literal.*)
    *libarch.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_encrypt_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:spi_flash_hal_gpspi.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_chip*.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_os_func_noos.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_os_func_app.*(.literal .literal.* .text .text.*)
    *libarch.a:flash_brownout_hook.*(.literal .literal.* .text .text.*)
    *libarch.a:esp_cache.*(.literal .literal.* .text .text.*)
    *libarch.a:cache_utils.*(.literal .literal.* .text .text.*)
    *libarch.a:memspi_host_driver.*(.literal .literal.* .text .text.*)

    *libc.a:sq_remlast.*(.literal .text .literal.* .text.*)

    esp_head.*(.literal .text .literal.* .text.*)
    esp_start.*(.literal .text .literal.* .text.*)
  } > sram_seg

  /* Marks the end of IRAM code segment */
  .iram0.text_end (NOLOAD) :
  {
    /* Align the end of code region as per PMP region granularity */
    . = ALIGN(128);

 . = ALIGN(4);
 _iram_text_end = ABSOLUTE(.);
  } > sram_seg

  .iram0.data :
  {
    . = ALIGN(16);
    _iram_data_start = ABSOLUTE(.);
    *(.iram.data)
    *(.iram.data*)
    _iram_data_end = ABSOLUTE(.);
  } >sram_seg

  .iram0.bss (NOLOAD) :
  {
    _iram_bss_start = ABSOLUTE(.);
    *(.iram.bss .iram.bss.*)
    _iram_bss_end = ABSOLUTE(.);

    . = ALIGN(16);
    _iram_end = ABSOLUTE(.);
  } >sram_seg

  /**
   * This section is required to skip .iram0.text area because sram_seg and
   * sram_seg reflect the same address space on different buses.
   */

  .dram0.dummy (NOLOAD):
  {
    . = ORIGIN(sram_seg) + _iram_end - _iram_start;
  } > sram_seg

  .dram0.data :
  {
    _data_start = ABSOLUTE(.);
    _dram_start = ABSOLUTE(.);
    _sdata = ABSOLUTE(.);
    _data_start_low = ABSOLUTE(.);
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
    *libarch.a:interrupt.*(.rodata .rodata.*)
    *libarch.a:periph_ctrl.*(.rodata .rodata.*)
    *libarch.a:rtc_clk.*(.rodata .rodata.*)
    *libarch.a:rtc_sleep.*(.rodata .rodata.*)
    *libarch.a:rtc_time.*(.rodata .rodata.*)
    *libarch.a:systimer.*(.rodata .rodata.*)
    *libarch.a:systimer_hal.*(.rodata .rodata.*)
    *libarch.a:uart_hal_iram.*(.rodata .rodata.*)
    *libarch.a:wdt_hal_iram.*(.rodata .rodata.*)
    *libsched.a:irq_dispatch.*(.rodata .rodata.*)
    *libsched.a:irq_dispatch.*(.rodata .rodata.*)
    *libsched.a:sched_lock.*(.rodata .rodata.*)
    *libsched.a:sched_unlock.*(.rodata .rodata.*)

    *libarch.a:*(.rodata.esprv_intc_int_get_type)
    *libarch.a:*riscv_doirq.*(.rodata .rodata.*)
    *libarch.a:*brownout.*(.rodata .rodata.*)
    *libarch.a:*cpu.*(.rodata .rodata.*)
    *libarch.a:*gpio_hal.*(.rodata .rodata.*)
    *libarch.a:*periph_ctrl.*(.rodata .rodata.*)
    *libarch.a:*clk.*(.rodata .rodata.*)
    *libarch.a:*esp_clk.*(.rodata .rodata.*)
    *libarch.a:*esp_clk_tree.*(.rodata .rodata.*)
    *libarch.a:*esp_clk_tree_common.*(.rodata .rodata.*)
    *libarch.a:*clk_tree_hal.*(.rodata .rodata.*)
    *libarch.a:*pmu_init.*(.rodata .rodata.*)
    *libarch.a:*modem_clock.*(.rodata .rodata.*)
    *libarch.a:*modem_clock_hal.*(.rodata .rodata.*)
    *libarch.a:*regi2c_ctrl.*(.rodata .rodata.*)
    *libarch.a:*gpio_periph.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_systimer.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_wdt.*(.rodata .rodata.*)
    *libarch.a:*ocode_init.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_regi2c_esp32p4.*(.rodata .rodata.*)
    *libarch.a:*rtc_clk.*(.rodata .rodata.*)
    *libarch.a:*rtc_clk_init.*(.rodata .rodata.*)
    *libarch.a:*pmu_param.*(.rodata .rodata.*)
    *libarch.a:*pmu_sleep.*(.rodata .rodata.*)
    *libarch.a:*rtc_time.*(.rodata .rodata.*)
    *libarch.a:*systimer.*(.rodata .rodata.*)
    *libarch.a:*systimer_hal.*(.rodata .rodata.*)
    *libarch.a:*uart_hal_iram.*(.rodata .rodata.*)
    *libarch.a:*wdt_hal_iram.*(.rodata .rodata.*)
    *libarch.a:*bootloader_banner_wrap.*(.rodata .rodata.*)
    *libarch.a:*bootloader_init.*(.rodata .rodata.*)
    *libarch.a:*bootloader_common.*(.rodata .rodata.*)
    *libarch.a:*bootloader_common_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_console.*(.rodata .rodata.*)
    *libarch.a:*bootloader_console_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_esp32p4.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash_config_esp32p4.*(.rodata .rodata.*)
    *libarch.a:*flash_qio_mode.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_init.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_efuse.*(.rodata .rodata.*)
    *libarch.a:*bootloader_panic.*(.rodata .rodata.*)
    *libarch.a:*bootloader_mem.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random_esp32p4.*(.rodata .rodata.*)
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
    *libarch.a:*esp_efuse_fields.*(.rodata .rodata.*)
    *libarch.a:*esp_efuse_api_key.*(.rodata .rodata.*)
    *libarch.a:*esp_efuse_utility.*(.rodata .rodata.*)
    *libarch.a:*efuse_hal.*(.rodata .rodata.*)
    *libarch.a:*log.*(.rodata .rodata.*)
    *libarch.a:*log_noos.*(.rodata .rodata.*)
    *libarch.a:esp_spiflash.*(.rodata .rodata.*)
    *libarch.a:esp_chip_rev.*(.rodata .rodata.*)
    *libarch.a:critical_section.*(.rodata .rodata.*)
    *libarch.a:os.*(.rodata.g_int_flags_count .rodata.g_int_flags)
    *libarch.a:spi_flash_hpm_enable.*(.rodata .rodata.*)
    *libarch.a:*sleep_modes.*(.rodata.esp_sleep_pd_config*)
    *libarch.a:esp_flash_api.*(.rodata .rodata.*)
    *libarch.a:esp_flash_spi_init.*(.rodata .rodata.*)
    *libarch.a:spi_flash_hal_iram.*(.rodata .rodata.*)
    *libarch.a:spi_flash_encrypt_hal_iram.*(.rodata .rodata.*)
    *libarch.a:spi_flash_hal_gpspi.*(.rodata .rodata.*)
    *libarch.a:spi_flash_chip*.*(.rodata .rodata.*)
    *libarch.a:spi_flash_wrap.*(.rodata .rodata.*)
    *libarch.a:spi_flash_os_func_noos.*(.rodata .rodata.*)
    *libarch.a:spi_flash_os_func_app.*(.rodata .rodata.*)
    *libarch.a:flash_brownout_hook.*(.rodata .rodata.*)
    *libarch.a:esp_cache.*(.rodata .rodata.*)
    *libarch.a:cache_utils.*(.rodata .rodata.*)
    *libarch.a:memspi_host_driver.*(.rodata .rodata.*)

    esp_head.*(.rodata .rodata.*)
    esp_start.*(.rodata .rodata.*)

    _data_end_low = ABSOLUTE(.);
    _edata = ABSOLUTE(.);
    _data_end = ABSOLUTE(.);
  } >sram_seg

  /**
   * This section holds data that should not be initialized at power up.
   * The section located in Internal SRAM memory region. The macro _NOINIT
   * can be used as attribute to place data into this section.
   * See the "esp_attr.h" file for more information.
   */
  .noinit (NOLOAD):
  {

 . = ALIGN(4);
 _noinit_start = ABSOLUTE(.);
    *(.noinit .noinit.*)

 . = ALIGN(4);
 _noinit_end = ABSOLUTE(.);
  } > sram_seg

  .dram0.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _bss_start = ABSOLUTE(.);

    /**
     * ldgen places all bss-related data to mapping[dram0_bss]
     * (See components/esp_system/app.lf).
     */
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

    *(.dynbss .dynsbss .gnu.linkonce.b .gnu.linkonce.b.* .gnu.linkonce.sb .gnu.linkonce.sb.* .gnu.linkonce.sb2 .gnu.linkonce.sb2.* .sbss .sbss.* .sbss2 .sbss2.* .scommon .share.mem)
    *(.ext_ram.bss .ext_ram.bss.*)

    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
    _bss_end = ABSOLUTE(.);
    _bss_end_low = ABSOLUTE(.);
    _dram_end = ABSOLUTE(.);

    . = ALIGN(4);
    _bss_end = ABSOLUTE(.);
  } > sram_seg

  .flash.text :
  {
    _stext = .;
    _instruction_reserved_start = ABSOLUTE(.);
    _text_start = ABSOLUTE(.);

    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    /**
     * CPU will try to prefetch up to 16 bytes of of instructions.
     * This means that any configuration (e.g. MMU, PMS) must allow
     * safe access to up to 16 bytes after the last real instruction, add
     * dummy bytes to ensure this
     */
    . += 16;
    _text_end = ABSOLUTE(.);
    /**
     * Mark the flash.text end.
     * This can be used for MMU driver to maintain virtual address.
     */
    _instruction_reserved_end = ABSOLUTE(.);
    _etext = .;

    /**
     * Similar to _iram_start, this symbol goes here so it is
     * resolved by addr2line in preference to the first symbol in
     * the flash.text segment.
     */
    _flash_cache_start = ABSOLUTE(0);
  } > text_seg_low

  /**
   * Dummy section represents the .flash.text section but in default_rodata_seg.
   * Thus, it must have its alignment and (at least) its size.
   */
  .flash_rodata_dummy (NOLOAD):
  {
    _flash_rodata_dummy_start = .;
    . = ALIGN(ALIGNOF(.flash.text)) + SIZEOF(.flash.text);
    /* Add alignment of MMU page size + 0x20 bytes for the mapping header. */
    . = ALIGN(0x10000) + 0x20;
  } > rodata_seg_low

  .flash.appdesc : ALIGN(0x10)
  {
    /**
     * Mark flash.rodata start.
     * This can be used for mmu driver to maintain virtual address
     */
    _rodata_reserved_start = ABSOLUTE(.);
    _rodata_start = ABSOLUTE(.);
    /* !DO NOT PUT ANYTHING BEFORE THIS! */
    /* Should be the first.  App version info. */
    *(.rodata_desc .rodata_desc.*)
    /* Should be the second. Custom app version info. */
    *(.rodata_custom_desc .rodata_custom_desc.*)
    /**
     * Create an empty gap within this section. Thanks to this, the end of this
     * section will match .flash.rodata's begin address. Thus, both sections
     * will be merged when creating the final bin image.
     */
    . = ALIGN(ALIGNOF(.flash.rodata));
  } > rodata_seg_low
  ASSERT((ADDR(.flash.rodata) == ADDR(.flash.appdesc) + SIZEOF(.flash.appdesc)), "The gap between .flash.appdesc and .flash.rodata must not exist to produce the final bin image.")

  .flash.rodata : ALIGN(0x10)
  {
    _flash_rodata_start = ABSOLUTE(.);
    *(EXCLUDE_FILE(*libgcov.a *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_by_dqs.* *libesp_hw_support.a:mspi_timing_by_flash_delay.* *libesp_hw_support.a:mspi_timing_config.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:pmu_sleep.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sleep_console.* *libesp_hw_support.a:sleep_usb.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_rom.a:esp_rom_wdt.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:pmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *liblog.a:log.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libsoc.a:temperature_sensor_periph.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_hpm_enable.* *libspi_flash.a:spi_flash_wrap.*) .rodata EXCLUDE_FILE(*libgcov.a *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_by_dqs.* *libesp_hw_support.a:mspi_timing_by_flash_delay.* *libesp_hw_support.a:mspi_timing_config.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:pmu_sleep.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sleep_console.* *libesp_hw_support.a:sleep_usb.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_rom.a:esp_rom_wdt.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:pmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *liblog.a:log.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libsoc.a:temperature_sensor_periph.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_hpm_enable.* *libspi_flash.a:spi_flash_wrap.*) .rodata.* EXCLUDE_FILE(*libgcov.a *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_by_dqs.* *libesp_hw_support.a:mspi_timing_by_flash_delay.* *libesp_hw_support.a:mspi_timing_config.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:pmu_sleep.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sleep_console.* *libesp_hw_support.a:sleep_usb.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_rom.a:esp_rom_wdt.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:pmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *liblog.a:log.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libsoc.a:temperature_sensor_periph.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_hpm_enable.* *libspi_flash.a:spi_flash_wrap.*) .sdata2 EXCLUDE_FILE(*libgcov.a *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_by_dqs.* *libesp_hw_support.a:mspi_timing_by_flash_delay.* *libesp_hw_support.a:mspi_timing_config.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:pmu_sleep.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sleep_console.* *libesp_hw_support.a:sleep_usb.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_rom.a:esp_rom_wdt.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:pmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *liblog.a:log.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libsoc.a:temperature_sensor_periph.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_hpm_enable.* *libspi_flash.a:spi_flash_wrap.*) .sdata2.* EXCLUDE_FILE(*libgcov.a *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_by_dqs.* *libesp_hw_support.a:mspi_timing_by_flash_delay.* *libesp_hw_support.a:mspi_timing_config.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:pmu_sleep.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sleep_console.* *libesp_hw_support.a:sleep_usb.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_rom.a:esp_rom_wdt.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:pmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *liblog.a:log.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libsoc.a:temperature_sensor_periph.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_hpm_enable.* *libspi_flash.a:spi_flash_wrap.*) .srodata EXCLUDE_FILE(*libgcov.a *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:mspi_timing_by_dqs.* *libesp_hw_support.a:mspi_timing_by_flash_delay.* *libesp_hw_support.a:mspi_timing_config.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:pmu_sleep.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sleep_console.* *libesp_hw_support.a:sleep_usb.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_rom.a:esp_rom_wdt.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:pmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *liblog.a:log.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libsoc.a:temperature_sensor_periph.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_hpm_enable.* *libspi_flash.a:spi_flash_wrap.*) .srodata.*)
    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    . = ALIGN(ALIGNOF(.flash.init_array));
  } > rodata_seg_low

  ASSERT((ADDR(.flash.init_array) == ADDR(.flash.rodata) + SIZEOF(.flash.rodata)), "The gap between .flash.rodata and .flash.init_array must not exist to produce the final bin image.")

  .flash.init_array :
  {
    /**
     * C++ constructor tables.
     *
     * Excluding crtbegin.o/crtend.o since IDF doesn't use the toolchain crt.
     *
     * RISC-V gcc is configured with --enable-initfini-array so it emits
     * .init_array section instead. But the init_priority sections will be
     * sorted for iteration in ascending order during startup.
     * The rest of the init_array sections is sorted for iteration in descending
     * order during startup, however. Hence a different section is generated for
     * the init_priority functions which is iterated in ascending order during
     * startup. The corresponding code can be found in startup.c.
     */

 . = ALIGN(4);
 __init_priority_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .init_array.*))
    __init_priority_array_end = ABSOLUTE(.);

 . = ALIGN(4);
 __init_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .init_array))
    __init_array_end = ABSOLUTE(.);
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
    _rodata_end = ABSOLUTE(.);
    . = ALIGN(ALIGNOF(.flash.tdata));
  } > rodata_seg_low
  ASSERT((ADDR(.flash.tdata) == ADDR(.flash.init_array) + SIZEOF(.flash.init_array)), "The gap between .flash.init_array and .flash.tdata must not exist to produce the final bin image.")

  .flash.tdata :
  {
    _thread_local_data_start = ABSOLUTE(.);
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    . = ALIGN(ALIGNOF(.flash.tbss));
    _thread_local_data_end = ABSOLUTE(.);
  } > rodata_seg_low

  ASSERT((ADDR(.flash.tbss) == ADDR(.flash.tdata) + SIZEOF(.flash.tdata)), "The gap between .flash.tdata and .flash.tbss must not exist to produce the final bin image.")

  .flash.tbss (NOLOAD) :
  {
    _thread_local_bss_start = ABSOLUTE(.);
    *(.tbss .tbss.* .gnu.linkonce.tb.*)
    *(.tcommon .tcommon.*)
    _thread_local_bss_end = ABSOLUTE(.);
  } > rodata_seg_low

  /**
   * This section contains all the rodata that is not used
   * at runtime, helping to avoid an increase in binary size.
   */
  .flash.rodata_noload (NOLOAD) :
  {
    /**
     * This symbol marks the end of flash.rodata. It can be utilized by the MMU
     * driver to maintain the virtual address.
     * NOLOAD rodata may not be included in this section.
     */
    _rodata_reserved_end = ADDR(.flash.tbss);
  } > rodata_seg_low

  /* Shared RAM */

  /* DWARF 1 */
  .debug 0 : { *(.debug) }
  .line 0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo 0 : { *(.debug_srcinfo) }
  .debug_sfnames 0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges 0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_line 0 : { *(.debug_line) }
  .debug_frame 0 : { *(.debug_frame) }
  .debug_str 0 : { *(.debug_str) }
  .debug_loc 0 : { *(.debug_loc) }
  .debug_macinfo 0 : { *(.debug_macinfo) }
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  /* DWARF 3 */
  .debug_ranges 0 : { *(.debug_ranges) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames 0 : { *(.debug_varnames) }
  /* GNU DWARF 2 extensions */
  .debug_gnu_pubnames 0 : { *(.debug_gnu_pubnames) }
  .debug_gnu_pubtypes 0 : { *(.debug_gnu_pubtypes) }
  /* DWARF 4 */
  .debug_types 0 : { *(.debug_types) }
  /* DWARF 5 */
  .debug_addr 0 : { *(.debug_addr) }
  .debug_line_str 0 : { *(.debug_line_str) }
  .debug_loclists 0 : { *(.debug_loclists) }
  .debug_macro 0 : { *(.debug_macro) }
  .debug_names 0 : { *(.debug_names) }
  .debug_rnglists 0 : { *(.debug_rnglists) }
  .debug_str_offsets 0 : { *(.debug_str_offsets) }
  .comment 0 : { *(.comment) }
  .note.GNU-stack 0: { *(.note.GNU-stack) }
  .riscv.attributes 0: { *(.riscv.attributes) }
  /DISCARD/ :
  {
   /**
    * Discarding .rela.* sections results in the following mapping:
    * .rela.text.* -> .text.*
    * .rela.data.* -> .data.*
    * And so forth...
    */
   *(.rela.*)
   *(.got .got.plt) /* TODO: GCC-382 */
   *(.eh_frame_hdr)
   *(.eh_frame)
  }

  _image_irom_vma = ADDR(.flash.text);
  _image_irom_lma = LOADADDR(.flash.text);
  _image_irom_size = SIZEOF(.flash.text);

  _image_drom_vma = ADDR(.flash.rodata);
  _image_drom_lma = LOADADDR(.flash.rodata);
  _image_drom_size = SIZEOF(.flash.rodata);
}
