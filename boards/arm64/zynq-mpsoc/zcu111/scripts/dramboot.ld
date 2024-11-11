/****************************************************************************
 * boards/arm64/zynq-mpsoc/zcu111/scripts/dramboot.ld
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

OUTPUT_ARCH(aarch64)

ENTRY(__start)

/* We don't use Uboot on Zynq UltraScale+ MPSoC besause XilinX SDK provided
 * FSBL(First Stage Boot Loader) which will boot ATF(Trusted Firmware-A)
 * and ATF will boot NuttX on BL31 stage. Memory is organized as follows:
 * - ATF debug reserved area is 0x00000000 - 0x00100000
 * - NuttX is loaded to 0x00100000 by ATF, ATF expects us here
 * - NuttX ROM and RAM are one continuous region, starting from 0x00100000
     with a size of 2046MB
 * - Heap memory is allocated from dram end to idlestack top
 */

MEMORY
{
   axi_bram_ctrl_0_Mem0 : ORIGIN = 0xB0000000, LENGTH = 0x1000
   psu_ddr_0_MEM_0 : ORIGIN = 0x0, LENGTH = 0x100000
   dram (rwx) : ORIGIN = 0x00100000, LENGTH = 0x7FE00000
   psu_ddr_1_MEM_0 : ORIGIN = 0x800000000, LENGTH = 0x80000000
   psu_ocm_ram_0_MEM_0 : ORIGIN = 0xFFFC0000, LENGTH = 0x40000
   psu_qspi_linear_0_MEM_0 : ORIGIN = 0xC0000000, LENGTH = 0x20000000
}

PHDRS
{
  /* R = 100, W = 010, X = 001 */

  text   PT_LOAD FLAGS(5); /* RX */
  rodata PT_LOAD FLAGS(4); /* R  */
  data   PT_LOAD FLAGS(6); /* RW */
}

SECTIONS
{
  .text  :
    {
      _stext = ABSOLUTE(.);  /* Text section */
      *(.start .start.*)     /* Place __start here */
      *(.text .text.*)
      *(.text.cold)
      *(.text.unlikely)
      *(.fixup)
      *(.gnu.warning)
    } > dram :text

  .init_section :
    {
      _sinit = ABSOLUTE(.);
      KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      KEEP(*(.init_array .ctors))
      _einit = ABSOLUTE(.);
    } > dram :text

  /* Vector table must be page aligned */

  .vector : ALIGN(4096)
    {
      _vector_start = ABSOLUTE(.);
      KEEP(*(.exc_vector_table))
      KEEP(*(".exc_vector_table.*"))
      KEEP(*(.vectors))
      _vector_end = ABSOLUTE(.);
    } > dram :text

  /* End of text data must be aligned to page boundary */

  . = ALIGN(4096);
  _etext = .;
  _sztext = _etext - _stext;

  /* Start of RO data must be page aligned (mapped as read only) */

  .rodata : ALIGN(4096)
    {
      _srodata = ABSOLUTE(.);          /* Read-only data */
      *(.rodata .rodata.*)
      *(.data.rel.ro)
      *(.data.rel.ro.*)
    } > dram :rodata

  /* End of RO data must be page aligned */

  . = ALIGN(4096);

  _erodata = .;                /* End of read-only data */
  _szrodata = _erodata - _srodata;
  _eronly = .;  /* End of read-only data */

  .data : ALIGN(4096)
    {
      _sdata = ABSOLUTE(.);
      *(.data.page_aligned)
      *(.data .data.*)
      . = ALIGN(8);
      *(.data.rel)
      *(.data.rel.*)
      CONSTRUCTORS
      . = ALIGN(8);
      _edata = ABSOLUTE(.);
    } > dram :data

  .bss :
    {
      . = ALIGN(8);
      _sbss = ABSOLUTE(.);
      *(.bss .bss.*)
      . = ALIGN(8);
      _ebss = ABSOLUTE(.);
    } > dram :data

  _szbss = _ebss - _sbss;

  .initstack :
    {
      _s_initstack = ABSOLUTE(.);
      *(.initstack)
    } > dram :data

  /* End of data must be page aligned */
  . = ALIGN(4096);

  g_idle_topstack = .;
  _e_initstack = .;
  _szdata = _e_initstack - _sdata;

  /* Sections to be discarded */
  /DISCARD/ : {
       *(.exit.text)
       *(.exit.data)
       *(.exitcall.exit)
       *(.eh_frame)
  }

  /* Stabs debugging sections.  */
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }
}

