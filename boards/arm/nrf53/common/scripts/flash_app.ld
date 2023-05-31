/****************************************************************************
 * boards/arm/nrf53/common/scripts/flash_app.ld
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

#define FLASH_CPUAPP_START_ADDR 0x00000000
#define RAM_CPUAPP_START_ADDR   0x20000000

/* Image format configuration */

#ifdef CONFIG_NRF53_APP_FORMAT_MCUBOOT
#  define MCUBOOT_HEADER_SIZE   CONFIG_NRF53_MCUBOOT_HEADER_SIZE
#  ifdef CONFIG_BOARDCTL_BOOT_IMAGE
#    define FLASH_OFFSET        0
#    define FLASH_IMG_SIZE      CONFIG_NRF53_OTA_PRIMARY_SLOT_OFFSET
#  else
#    define FLASH_OFFSET        (CONFIG_NRF53_OTA_PRIMARY_SLOT_OFFSET + MCUBOOT_HEADER_SIZE)
#    define FLASH_IMG_SIZE      (CONFIG_NRF53_OTA_SLOT_SIZE - MCUBOOT_HEADER_SIZE)
#  endif
#else
#  define FLASH_OFFSET          0
#  define FLASH_IMG_SIZE        CONFIG_NRF53_CPUAPP_MEM_FLASH_SIZE
#endif

/* Shared memory configuration */

#ifdef CONFIG_RPTUN
#  define RPTUN_SHMEM 0x8000
#else
#  define RPTUN_SHMEM 0
#endif

MEMORY
{
    progmem (rx)  : ORIGIN = FLASH_CPUAPP_START_ADDR + FLASH_OFFSET,
                    LENGTH = FLASH_IMG_SIZE
    datamem (rwx) : ORIGIN = RAM_CPUAPP_START_ADDR,
                    LENGTH = CONFIG_NRF53_CPUAPP_MEM_RAM_SIZE - RPTUN_SHMEM
}

OUTPUT_ARCH(arm)
ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
EXTERN(_vectors)       /* Force the vectors to be included in the output */
SECTIONS
{
    .text :
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > progmem

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > progmem

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > progmem

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } > progmem
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > datamem AT > progmem

    /* BSS */

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > datamem

    /* Stabs debugging sections. */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
