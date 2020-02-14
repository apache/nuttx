/****************************************************************************
 * boards/arm/tiva/lm3s6965-ek/scripts/gnu-elf.ld
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

SECTIONS
{
  .text 0x00000000 :
    {
      _stext = . ;
      *(.text)
      *(.text.*)
      *(.gnu.warning)
      *(.stub)
      *(.glue_7)
      *(.glue_7t)
      *(.jcr)

      /* C++ support:  The .init and .fini sections contain specific logic
       * to manage static constructors and destructors.
       */

      *(.gnu.linkonce.t.*)
      *(.init)             /* Old ABI */
      *(.fini)             /* Old ABI */
      _etext = . ;
    }

  .ARM.extab :
    {
      *(.ARM.extab*)
    }

  .ARM.exidx :
    {
      *(.ARM.exidx*)
    }

  .rodata :
    {
      _srodata = . ;
      *(.rodata)
      *(.rodata1)
      *(.rodata.*)
      *(.gnu.linkonce.r*)
      _erodata = . ;
    }

  .data :
    {
      _sdata = . ;
      *(.data)
      *(.data1)
      *(.data.*)
      *(.gnu.linkonce.d*)
      . = ALIGN(4);
      _edata = . ;
    }

  /* C++ support. For each global and static local C++ object,
   * GCC creates a small subroutine to construct the object. Pointers
   * to these routines (not the routines themselves) are stored as
   * simple, linear arrays in the .ctors section of the object file.
   * Similarly, pointers to global/static destructor routines are
   * stored in .dtors.
   */

  .ctors :
    {
      _sctors = . ;
      *(.ctors)       /* Old ABI:  Unallocated */
      *(.init_array)  /* New ABI:  Allocated */
      _edtors = . ;
    }

  .dtors :
    {
      _sdtors = . ;
      *(.dtors)       /* Old ABI:  Unallocated */
      *(.fini_array)  /* New ABI:  Allocated */
      _edtors = . ;
    }

  .bss :
    {
      _sbss = . ;
      *(.bss)
      *(.bss.*)
      *(.sbss)
      *(.sbss.*)
      *(.gnu.linkonce.b*)
      *(COMMON)
      . = ALIGN(4);
      _ebss = . ;
    }

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
