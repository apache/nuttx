/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/kernel-space.ld
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
    esp32s3_userspace.*(.literal .text .literal.* .text.*)
    *librtc.a:(.literal .text .literal.* .text.*)
    *libkarch.a:esp32s3_spiflash.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_cpupause.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_copystate.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_interruptcontext.*(.literal .text .literal.* .text.*)
    *libkarch.a:xtensa_testset.*(.literal .text .literal.* .text.*)

    *libsched.a:sched_suspendscheduler.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_note.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_thistask.*(.literal .text .literal.* .text.*)
    *libsched.a:spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_csection.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_dispatch.*(.literal .text .literal.* .text.*)

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
    esp32s3_userspace.*(.rodata  .rodata.*)
    *libkarch.a:esp32s3_spiflash.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_cpupause.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_copystate.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_interruptcontext.*(.rodata  .rodata.*)
    *libkarch.a:xtensa_testset.*(.rodata  .rodata.*)

    *libsched.a:sched_suspendscheduler.*(.rodata  .rodata.*)
    *libsched.a:sched_thistask.*(.rodata  .rodata.*)
    *libsched.a:sched_note.*(.rodata  .rodata.*)
    *libsched.a:spinlock.*(.rodata  .rodata.*)
    *libsched.a:irq_csection.*(.rodata  .rodata.*)
    *libsched.a:irq_dispatch.*(.rodata  .rodata.*)

    . = ALIGN(4);
    _edata = ABSOLUTE(.);

    /* Heap starts at the end of .data */

    _sheap = ABSOLUTE(.);
  } >KDRAM

  .flash.text :
  {
    _stext = .;
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
  } >KDROM
}
