/****************************************************************************
 * binfmt/libnxflat/gnu-nxflat-gotoff.ld
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

MEMORY
{
  ISPACE : ORIGIN = 0x0, LENGTH = 2097152
  DSPACE : ORIGIN = 0x0, LENGTH = 2097152
}

/****************************************************************************
 * The XFLAT program image is divided into two segments:
 *
 * (1) ISpace (Instruction Space).  This is the segment that contains
 *     code (.text).  Everything in the segment should be fetch-able
 *     machine PC instructions (jump, branch, call, etc.).
 * (2) DSpace (Data Space).  This is the segment that contains both
 *     read-write data (.data, .bss) as well as read-only data (.rodata).
 *     Everything in this segment should be access-able with machine
 *     PIC load and store instructions.
 *
 * Older versions of GCC (at least up to GCC 4.3.3), use GOT-relative
 * addressing to access RO data. In that case, read-only data (.rodata) must
 * reside in D-Space and this linker script should be used.
 *
 * Newer versions of GCC (at least as of GCC 4.6.3), use PC-relative
 * addressing to access RO data. In that case, read-only data (.rodata) must
 * reside in I-Space and this linker script should NOT be used with those
 * newer tools.
 *
 ****************************************************************************/

SECTIONS
{
  .text 0x00000000 :
    {
      /* ISpace is located at address 0.  Every (unrelocated) ISpace
       * address is an offset from the beginning of this segment.
       */

      text_start = . ;

      *(.text)
      *(.text.*)
      *(.gnu.warning)
      *(.stub)
      *(.glue_7)
      *(.glue_7t)
      *(.jcr)

      /* C++ support:  The .init and .fini sections contain XFLAT-
       * specific logic to manage static constructors and destructors.
       */

      *(.gnu.linkonce.t.*)
      *(.init)
      *(.fini)

      /* This is special code area at the end of the normal
         text section.  It contains a small lookup table at
         the start followed by the code pointed to by entries
         in the lookup table.  */

      . = ALIGN (4) ;
      PROVIDE(__ctbp = .);
      *(.call_table_data)
      *(.call_table_text)

      _etext = . ;

    } > ISPACE

    /* DSpace is also located at address 0.  Every (unrelocated) DSpace
     * address is an offset from the beginning of this segment.
     */

  .data 0x00000000 :
    {
      /* In this model, .rodata is access using GOT-relative addressing
       * and, hence, must also reside in the .data section.
       */

      __data_start = . ;
      *(.rodata)
      *(.rodata1)
      *(.rodata.*)
      *(.gnu.linkonce.r*)

      *(.data)
      *(.data1)
      *(.data.*)
      *(.gnu.linkonce.d*)
      *(.data1)
      *(.eh_frame)
      *(.gcc_except_table)

      *(.gnu.linkonce.s.*)
      *(__libc_atexit)
      *(__libc_subinit)
      *(__libc_subfreeres)
      *(.note.ABI-tag)

      /* C++ support. For each global and static local C++ object,
       * GCC creates a small subroutine to construct the object. Pointers
       * to these routines (not the routines themselves) are stored as
       * simple, linear arrays in the .ctors section of the object file.
       * Similarly, pointers to global/static destructor routines are
       * stored in .dtors.
       */

      *(.gnu.linkonce.d.*)

      _ctors_start = . ;
      *(.ctors)
      _ctors_end = . ;
      _dtors_start = . ;
      *(.dtors)
      _dtors_end = . ;

      . = ALIGN(4);
      _edata = . ;
      edata = ALIGN( 0x10 ) ;
    } > DSPACE

  .bss :
    {
      __bss_start = _edata ;
      *(.dynsbss)
      *(.sbss)
      *(.sbss.*)
      *(.scommon)
      *(.dynbss)
      *(.bss)
      *(.bss.*)
      *(.bss*)
      *(.gnu.linkonce.b*)
      *(COMMON)
      end = ALIGN( 0x10 ) ;
      _end = ALIGN( 0x10 ) ;
    } > DSPACE

    .got 0 : { *(.got.plt) *(.got) }
    .junk 0 : { *(.rel*) *(.rela*) }
    /* Stabs debugging sections.    */
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
