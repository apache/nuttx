/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/legacy_sections.ld
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

_diram_i_start = 0x40378000;

SECTIONS
{
  /* Send .iram0 code to iram */

  .iram0.vectors :
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

    . = ALIGN(16);

    *(.entry.text)
    *(.init.literal)
    *(.init)
  } >iram0_0_seg

  .iram0.text :
  {
    /* Code marked as running out of IRAM */

    *(.iram1 .iram1.*)

    *libarch.a:esp32s3_cpuindex.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_irq.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_user.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_assert.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpuint.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpupause.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_irqdispatch.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_modifyreg32.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_testset.*(.literal .text .literal.* .text.*)

    *libarch.a:*esp_rom_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.literal .text .literal.* .text.*)

    *libc.a:sq_remlast.*(.literal .text .literal.* .text.*)

    *libdrivers.a:syslog_flush.*(.literal .text .literal.* .text.*)

    *libsched.a:assert.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_csection.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_dispatch.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_note.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_suspendscheduler.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_thistask.*(.literal .text .literal.* .text.*)
    *libsched.a:spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:stack_record.*(.literal .text .literal.* .text.*)

    *libc.a:*lib_instrument.*(.literal .text .literal.* .text.*)

#ifdef CONFIG_ESP32S3_SPEED_UP_ISR
    *libarch.a:xtensa_switchcontext.*(.literal.up_switch_context .text.up_switch_context)

    *libarch.a:esp32s3_timerisr.*(.literal.systimer_isr .text.systimer_isr)
    *libarch.a:esp32s3_idle.*(.literal.up_idle .text.up_idle)
    *libarch.a:esp32s3_dma.*(.literal.esp32s3_dma_load .text.esp32s3_dma_load \
                             .literal.esp32s3_dma_enable .text.esp32s3_dma_enable)

    *libsched.a:sched_processtimer.*(.literal.nxsched_process_timer .text.nxsched_process_timer)
    *libsched.a:clock_initialize.*(.literal.clock_timer .text.clock_timer)
    *libsched.a:wd_start.*(.literal.wd_timer .text.wd_timer)
    *libsched.a:sched_roundrobin.*(.literal.nxsched_process_roundrobin .text.nxsched_process_roundrobin)
    *libsched.a:sched_reprioritizertr.*(.literal.nxsched_reprioritize_rtr .text.nxsched_reprioritize_rtr)
    *libsched.a:sched_removereadytorun.*(.literal.nxsched_remove_readytorun .text.nxsched_remove_readytorun)
    *libsched.a:sched_addreadytorun.*(.literal.nxsched_add_readytorun .text.nxsched_add_readytorun)
    *libsched.a:sched_addprioritized.*(.literal.nxsched_add_prioritized .text.nxsched_add_prioritized)
    *libsched.a:sched_mergepending.*(.literal.nxsched_merge_pending .text.nxsched_merge_pending)
    *libsched.a:sched_resumescheduler.*(.literal.nxsched_resume_scheduler .text.nxsched_resume_scheduler)

    *libc.a:sq_remfirst.*(.literal.sq_remfirst .text.sq_remfirst)
#endif

    *libarch.a:esp32s3_spi_timing.*(.literal .text .literal.* .text.*)
#ifdef CONFIG_ESP32S3_SPIRAM_MODE_QUAD
    *libarch.a:esp32s3_psram_quad.*(.literal .text .literal.* .text.*)
#endif
#ifdef CONFIG_ESP32S3_SPIRAM_MODE_OCT
    *libarch.a:esp32s3_psram_octal.*(.literal .text .literal.* .text.*)
#endif
#if defined(CONFIG_STACK_CANARIES) && \
    (defined(CONFIG_ESP32S3_SPIFLASH) || \
     defined(CONFIG_ESP32S3_SPIRAM))
    *libc.a:lib_stackchk.*(.literal .text .literal.* .text.*)
#endif
    *libarch.a:*cache_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.literal .text .literal.* .text.*)

    *(.wifirxiram .wifirxiram.*)
    *(.wifi0iram  .wifi0iram.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *(.phyiram .phyiram.*)

    /* align + add 16B for CPU dummy speculative instr. fetch */

    . = ALIGN(4) + 16;

    _iram_text = ABSOLUTE(.);
  } >iram0_0_seg

  .dram0.dummy (NOLOAD) :
  {
    /* This section is required to skip .iram0.text area because iram0_0_seg
     * and dram0_0_seg reflect the same address space on different buses.
     */

    . = ORIGIN(dram0_0_seg) + MAX(_iram_end, _diram_i_start) - _diram_i_start;
  } >dram0_0_seg

  /* Shared RAM */

  .dram0.bss (NOLOAD) :
  {
    /* .bss initialized on power-up */

    . = ALIGN(8);
    _sbss = ABSOLUTE(.);

    *(.bss .bss.*)
    *(COMMON)
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.share.mem)
    *(.gnu.linkonce.b.*)

    . = ALIGN(8);
    _ebss = ABSOLUTE(.);
  } >dram0_0_seg

  .noinit (NOLOAD) :
  {
    /* This section contains data that is not initialized during load,
     * or during the application's initialization sequence.
     */

    . = ALIGN(4);

    *(.noinit .noinit.*)

    . = ALIGN(4);
  } >dram0_0_seg

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
    *libarch.a:xtensa_context.*(.rodata  .rodata.*)
    *libarch.a:*cache_hal.*(.rodata .rodata.*)
#if defined(CONFIG_STACK_CANARIES) && \
    (defined(CONFIG_ESP32S3_SPIFLASH) || \
     defined(CONFIG_ESP32S3_SPIRAM))
    *libc.a:lib_stackchk.*(.rodata  .rodata.*)
#endif

    _edata = ABSOLUTE(.);
    . = ALIGN(4);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
  } >dram0_0_seg

  .flash.text :
  {
    _stext = .;
    _instruction_reserved_start = ABSOLUTE(.);

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

    /* CPU will try to prefetch up to 16 bytes of instructions.
     * This means that any configuration (e.g. MMU, PMS) must allow
     * safe access to up to 16 bytes after the last real instruction, add
     * dummy bytes to ensure this
     */

    . += 16;

    _instruction_reserved_end = ABSOLUTE(.);
    _etext = .;
  } >default_code_seg

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
  } >default_rodata_seg

  .flash.rodata : ALIGN(0x10)
  {
    _srodata = ABSOLUTE(.);

    *(.rodata)
    *(.rodata.*)
#ifdef CONFIG_ESP32S3_WIRELESS
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

    . = ALIGN(4);

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
    _erodata = ABSOLUTE(.);

    /* Literals are also RO data. */

    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    _rodata_reserved_end = ABSOLUTE(.);
    . = ALIGN(4);
  } >default_rodata_seg

  /* Marks the end of IRAM code segment */

  .iram0.text_end (NOLOAD) :
  {
    /* ESP32-S3 memprot requires 16B padding for possible CPU prefetch and
     * 256B alignment for PMS split lines.
     */

    . += 16;
    . = ALIGN(256);
  } >iram0_0_seg

  .iram0.data :
  {
    . = ALIGN(4);

    *(.iram.data)
    *(.iram.data.*)
  } >iram0_0_seg

  .iram0.bss (NOLOAD) :
  {
    . = ALIGN(4);

    *(.iram.bss)
    *(.iram.bss.*)

    . = ALIGN(4);
    _iram_end = ABSOLUTE(.);
  } >iram0_0_seg

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
  } > rtc_slow_seg
}
