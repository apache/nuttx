/****************************************************************************
 * boards/arm/qemu/qemu-armv7a/scripts/dramboot_armv7a.ld
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
OUTPUT_ARCH(arm)
ENTRY(__start)
PHDRS
{
  text PT_LOAD ;
}
SECTIONS
{
  . = 0x40101000;

  /* where the global variable out-of-bounds detection information located */
#ifdef CONFIG_MM_KASAN_GLOBAL
  .kasan.unused :
  {
    *(.data..LASANLOC*)
  } > ROM
  .kasan.global :
  {
    KEEP (*(.data..LASAN0))
    KEEP (*(.data.rel.local..LASAN0))
  } > ROM
#endif

  .text : {
        _stext = .;            /* Text section */
       *(.vectors)
       *(.text)
       *(.text.cold)
       *(.text.unlikely)
       *(.fixup)
       *(.gnu.warning)
  } :text
  . = ALIGN(4096);
  .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array .ctors))
        _einit = ABSOLUTE(.);
  } :text
  . = ALIGN(4096);
  _etext = .; /* End_1 of .text */
  _sztext = _etext - _stext;
  .rodata : {
        _srodata = .;          /* Read-only data */
       *(.rodata)
       *(.rodata.*)
       *(.data.rel.ro)
       *(.data.rel.ro.*)
  } :text
  . = ALIGN(4096);
  _erodata = .;                /* End of read-only data */
  _szrodata = _erodata - _srodata;
  _eronly = .;  /* End of read-only data */
  .data : {                    /* Data */
        _sdata = .;
       *(.data.page_aligned)
       *(.data)
       . = ALIGN(8);
       __start_impls = .;
        *(.impls)
        KEEP(*(.impls))
        . = ALIGN(4);
        __stop_impls = .;
       *(.data.rel)
       *(.data.rel.*)
       CONSTRUCTORS
  } :text
  _edata = .; /* End+1 of .data */
  .bss : {                     /* BSS */
       _sbss = .;
       *(.bss)
       . = ALIGN(1 << 3);
  } :text
  . = ALIGN(4096);
  _ebss = .;
  _szbss = _ebss - _sbss;
  .initstack : {             /* INIT STACK */
       _s_initstack = .;
       *(.initstack)
       . = ALIGN(16);
  } :text
  . = ALIGN(4096);
  _e_initstack = . ;
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

