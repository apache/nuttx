/****************************************************************************
 * boards/renesas/sh1/us7032evb1/scripts/sram.ld
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

OUTPUT_ARCH(sh)
ENTRY(_stext)
SECTIONS
{
    /* The us7032evb1 has CMON in PROM beginning at address 0x00000000 and
     * either 64Kb or 256Kb of SRAM beginning at 0x0a000000.  Neither the
     * PROM nor the first 8Kb of SRAM are available to the devoleper as these
     * are used by CMON.  The next 1Kb of SRAM is dedicated to relocated
     * interrupt vectors.
     */

    . = 0x0a002000;
    .text : {
        _svect = ABSOLUTE(.);
        *(.vects);      /* Relocated interrupt vectors  */
        _evect = ABSOLUTE(.);
        _stext = ABSOLUTE(.);
        *(.reset)       /* Reset/IRQ code       */
        *(.text)        /* Code             */
        *(.fixup)
        *(.gnu.warning)
        *(.rodata)      /* Read-only data       */
        *(.rodata.str1.4)
        *(.glue_7)
        *(.glue_7t)
        *(.got)         /* Global offset table      */
        _etext = ABSOLUTE(.);
    }

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data)        /* Modifiable data      */
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    }

    .bss : {            /* BSS              */
        _sbss = ABSOLUTE(.);
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
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
