/****************************************************************************
 * boards/arm64/imx9/imx93-evk/scripts/ocramboot.ld
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
EXTERN(__start)

/* Memory is organized as follows:
 * - ahab reserves memory from 2048k to 0x2049a000
 * - NuttX is compiled into 0x2049a000, ahab expects NuttX here
 * - Heap memory is allocated from ocram end to idlestack top
 */

MEMORY
{
  ocram (rx) : ORIGIN = 0x2049a000, LENGTH = 0x46000
  ocram_data (rw) : ORIGIN = 0x204e0000, LENGTH = 0x6000
  ocram_noload (rw) : ORIGIN = 0x204f0000, LENGTH = 0x2c000
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
    } > ocram :text

  .init_section :
    {
      _sinit = ABSOLUTE(.);
      KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      KEEP(*(.init_array .ctors))
      _einit = ABSOLUTE(.);
    } > ocram :text

  /* Vector table must be page aligned */

  .vector : ALIGN(4096)
    {
      _vector_start = ABSOLUTE(.);
      KEEP(*(.exc_vector_table))
      KEEP(*(".exc_vector_table.*"))
      KEEP(*(.vectors))
      _vector_end = ABSOLUTE(.);
    } > ocram :text

  _etext = .;

  .rodata : ALIGN(4096)
    {
      _srodata = ABSOLUTE(.);          /* Read-only data */
      *(.rodata .rodata.*)
      *(.data.rel.ro)
      *(.data.rel.ro.*)
    } > ocram :rodata

  _erodata = .;            /* End of read-only data */
  _eronly = .;             /* End of read-only data */

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
    } > ocram_data :data

  .bss (NOLOAD) :
    {
      . = ALIGN(8);
      _sbss = ABSOLUTE(.);
      *(.bss .bss.*)
      . = ALIGN(8);
      _ebss = ABSOLUTE(.);
    } > ocram_noload :data

  .initstack :
    {
      _s_initstack = ABSOLUTE(.);
      *(.initstack)
    } > ocram_noload :data

  /* End of data must be page aligned */

  . = ALIGN(4096);

  g_idle_topstack = .;
  _e_initstack = .;

  _sztext = _srodata - _stext;   /* _erodata is aligned, after _etext */
  _szrodata = _sdata - _srodata; /* _sdata is aligned, after _erodata */

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

_heapend = ORIGIN(ocram_noload) + LENGTH(ocram_noload);
_szdata = _heapend - _sdata;
