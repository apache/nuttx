/****************************************************************************
 * boards/renesas/m16c/skp16c26/scripts/skp16c26.ld
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

OUTPUT_ARCH(m32c)
ENTRY(_stext)
SECTIONS
{
    /* "Far" flash memory begins at address 0xf0000 for the M20262F8 part and
     * ends at address 0xfffff (all parts).  The program entry point is
     * the first address in flash
     */

    . = 0xf0000;
    .text : {
        _stext = ABSOLUTE(.);
        *(.text)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata)
        *(.glue_7)
        *(.glue_7t)
        *(.got)         /* Global offset table      */
        _etext = ABSOLUTE(.);
    }

    _enronly = ABSOLUTE(.);     /* End of read-only values      */
    /* .ndata will be relocated     */
    /* from this address            */

    /* The "variable" vector table will be fixed at the following address */

    . = 0xffd00;
    .varvect : {
        _svarvect = ABSOLUTE(.);
        *(.varvect)
        _evarvect = ABSOLUTE(.);
    }

    /* Followed by the special page/fixed vector table. */

    . = 0xffe00;
    .specpg : {
        _sspecpg = ABSOLUTE(.);
        *(.specpg)
        _especpg = ABSOLUTE(.);
    }

    . = 0xfffdc;
    .fixvect : {
        _sfixvect = ABSOLUTE(.);
        *(.fixvect)
        _efixvect = ABSOLUTE(.);
    }

    /* Internal "near" RAM begins at address 0x00400 (all parts) and ends at
     * address 0x00bff (M20262F6 and M20262F8 parts).  With the RAM
     * region, used is .data followed by .bss.  The remainder of RAM
     * carved up by the start-up code into stacks and heaps.
     */

    . = 0x00400;

    .ndata : {
        _sndata = ABSOLUTE(.);
        *(.data)
        CONSTRUCTORS
        _endata = ABSOLUTE(.);
    }

    .nbss : {           /* BSS              */
        _snbss = ABSOLUTE(.);
        *(.bss)
        *(COMMON)
        _enbss = ABSOLUTE(.);
    }

    /* "Far" RAM begins at 0x10000.  The SKP16C26 has no far RAM */

    . = 0x10000;

    .fdata : {
        _sfdata = ABSOLUTE(.);
        *(.fdata)
        CONSTRUCTORS
        _efdata = ABSOLUTE(.);
    }

    .fbss : {           /* BSS              */
        _sfbss = ABSOLUTE(.);
        *(.fbss)
        *(COMMON)
        _efbss = ABSOLUTE(.);
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
