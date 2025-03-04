/****************************************************************************
 * boards/tricore/tc3xx/tc397/scripts/Lcf_Tasking_Tricore_Tc.lsl
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

#include "include/nuttx/config.h"

#define LCF_CSA0_SIZE 40k
#define LCF_USTACK0_SIZE CONFIG_IDLETHREAD_STACKSIZE
#define LCF_ISTACK0_SIZE CONFIG_ARCH_INTERRUPTSTACK

#define LCF_CSA1_SIZE 40k
#define LCF_USTACK1_SIZE CONFIG_IDLETHREAD_STACKSIZE
#define LCF_ISTACK1_SIZE CONFIG_ARCH_INTERRUPTSTACK

#define LCF_CSA2_SIZE 40k
#define LCF_USTACK2_SIZE CONFIG_IDLETHREAD_STACKSIZE
#define LCF_ISTACK2_SIZE CONFIG_ARCH_INTERRUPTSTACK

#define LCF_CSA3_SIZE 40k
#define LCF_USTACK3_SIZE CONFIG_IDLETHREAD_STACKSIZE
#define LCF_ISTACK3_SIZE CONFIG_ARCH_INTERRUPTSTACK

#define LCF_CSA4_SIZE 40k
#define LCF_USTACK4_SIZE CONFIG_IDLETHREAD_STACKSIZE
#define LCF_ISTACK4_SIZE CONFIG_ARCH_INTERRUPTSTACK

#define LCF_CSA5_SIZE 40k
#define LCF_USTACK5_SIZE CONFIG_IDLETHREAD_STACKSIZE
#define LCF_ISTACK5_SIZE CONFIG_ARCH_INTERRUPTSTACK

#define LCF_HEAP_SIZE  4k

#define LCF_CPU0 0
#define LCF_CPU1 1
#define LCF_CPU2 2
#define LCF_CPU3 3
#define LCF_CPU4 4
#define LCF_CPU5 5


/*Un comment one of the below statements to enable CpuX DMI RAM to hold global variables*/
#define LCF_DEFAULT_HOST LCF_CPU0
/*#define LCF_DEFAULT_HOST LCF_CPU1*/
/*#define LCF_DEFAULT_HOST LCF_CPU2*/
/*#define LCF_DEFAULT_HOST LCF_CPU3*/
/*#define LCF_DEFAULT_HOST LCF_CPU4*/
/*#define LCF_DEFAULT_HOST LCF_CPU5*/

#define LCF_DSPR5_START 0x10000000
#define LCF_DSPR5_SIZE  96k

#define LCF_DSPR4_START 0x30000000
#define LCF_DSPR4_SIZE  96k

#define LCF_DSPR3_START 0x40000000
#define LCF_DSPR3_SIZE  96k

#define LCF_DSPR2_START 0x50000000
#define LCF_DSPR2_SIZE  96k

#define LCF_DSPR1_START 0x60000000
#define LCF_DSPR1_SIZE  240k

#define LCF_DSPR0_START 0x70000000
#define LCF_DSPR0_SIZE  240k

#define LCF_CSA5_OFFSET     (LCF_DSPR5_SIZE - 1k - LCF_CSA5_SIZE)
#define LCF_ISTACK5_OFFSET  (LCF_CSA5_OFFSET - 256 - LCF_ISTACK5_SIZE)
#define LCF_USTACK5_OFFSET  (LCF_ISTACK5_OFFSET - 256 - LCF_USTACK5_SIZE)

#define LCF_CSA4_OFFSET     (LCF_DSPR4_SIZE - 1k - LCF_CSA4_SIZE)
#define LCF_ISTACK4_OFFSET  (LCF_CSA4_OFFSET - 256 - LCF_ISTACK4_SIZE)
#define LCF_USTACK4_OFFSET  (LCF_ISTACK4_OFFSET - 256 - LCF_USTACK4_SIZE)

#define LCF_CSA3_OFFSET     (LCF_DSPR3_SIZE - 1k - LCF_CSA3_SIZE)
#define LCF_ISTACK3_OFFSET  (LCF_CSA3_OFFSET - 256 - LCF_ISTACK3_SIZE)
#define LCF_USTACK3_OFFSET  (LCF_ISTACK3_OFFSET - 256 - LCF_USTACK3_SIZE)

#define LCF_CSA2_OFFSET     (LCF_DSPR2_SIZE - 1k - LCF_CSA2_SIZE)
#define LCF_ISTACK2_OFFSET  (LCF_CSA2_OFFSET - 256 - LCF_ISTACK2_SIZE)
#define LCF_USTACK2_OFFSET  (LCF_ISTACK2_OFFSET - 256 - LCF_USTACK2_SIZE)

#define LCF_CSA1_OFFSET     (LCF_DSPR1_SIZE - 1k - LCF_CSA1_SIZE)
#define LCF_ISTACK1_OFFSET  (LCF_CSA1_OFFSET - 256 - LCF_ISTACK1_SIZE)
#define LCF_USTACK1_OFFSET  (LCF_ISTACK1_OFFSET - 256 - LCF_USTACK1_SIZE)

#define LCF_CSA0_OFFSET     (LCF_DSPR0_SIZE - 1k - LCF_CSA0_SIZE)
#define LCF_ISTACK0_OFFSET  (LCF_CSA0_OFFSET - 256 - LCF_ISTACK0_SIZE)
#define LCF_USTACK0_OFFSET  (LCF_ISTACK0_OFFSET - 256 - LCF_USTACK0_SIZE)

#define LCF_HEAP0_OFFSET    (LCF_USTACK0_OFFSET - LCF_HEAP_SIZE)
#define LCF_HEAP1_OFFSET    (LCF_USTACK1_OFFSET - LCF_HEAP_SIZE)
#define LCF_HEAP2_OFFSET    (LCF_USTACK2_OFFSET - LCF_HEAP_SIZE)
#define LCF_HEAP3_OFFSET    (LCF_USTACK3_OFFSET - LCF_HEAP_SIZE)
#define LCF_HEAP4_OFFSET    (LCF_USTACK4_OFFSET - LCF_HEAP_SIZE)
#define LCF_HEAP5_OFFSET    (LCF_USTACK5_OFFSET - LCF_HEAP_SIZE)

#define LCF_INTVEC0_START 0x802FE000
#define LCF_INTVEC1_START 0x805FE000
#define LCF_INTVEC2_START 0x808FE000
#define LCF_INTVEC3_START 0x80BFE000
#define LCF_INTVEC4_START 0x80EFE000
#define LCF_INTVEC5_START 0x80FFE000

#define LCF_TRAPVEC0_START 0x80000100
#define LCF_TRAPVEC1_START 0x80300000
#define LCF_TRAPVEC2_START 0x80600000
#define LCF_TRAPVEC3_START 0x80900000
#define LCF_TRAPVEC4_START 0x80C00000
#define LCF_TRAPVEC5_START 0x80F00000

#define LCF_STARTPTR_CPU0 0x80000000
#define LCF_STARTPTR_CPU1 0x80300100
#define LCF_STARTPTR_CPU2 0x80600100
#define LCF_STARTPTR_CPU3 0x80900100
#define LCF_STARTPTR_CPU4 0x80C00100
#define LCF_STARTPTR_CPU5 0x80F00100

#define LCF_STARTPTR_NC_CPU0 0xA0000000
#define LCF_STARTPTR_NC_CPU1 0xA0300100
#define LCF_STARTPTR_NC_CPU2 0xA0600100
#define LCF_STARTPTR_NC_CPU3 0xA0900100
#define LCF_STARTPTR_NC_CPU4 0xA0C00100
#define LCF_STARTPTR_NC_CPU5 0xA0F00100

#define INTTAB0             (LCF_INTVEC0_START)
#define INTTAB1             (LCF_INTVEC1_START)
#define INTTAB2             (LCF_INTVEC2_START)
#define INTTAB3             (LCF_INTVEC3_START)
#define INTTAB4             (LCF_INTVEC4_START)
#define INTTAB5             (LCF_INTVEC5_START)
#define TRAPTAB0            (LCF_TRAPVEC0_START)
#define TRAPTAB1            (LCF_TRAPVEC1_START)
#define TRAPTAB2            (LCF_TRAPVEC2_START)
#define TRAPTAB3            (LCF_TRAPVEC3_START)
#define TRAPTAB4            (LCF_TRAPVEC4_START)
#define TRAPTAB5            (LCF_TRAPVEC5_START)

#define RESET LCF_STARTPTR_NC_CPU0

#include "tc1v1_6_2.lsl"

// Specify a multi-core processor environment (mpe)

processor mpe
{
    derivative = tc39;
}

derivative tc39
{
    core tc0
    {
        architecture = TC1V1.6.2;
        space_id_offset = 100;            // add 100 to all space IDs in the architecture definition
        copytable_space = vtc:linear;     // use the copy table in the virtual core for 'bss' and initialized data sections
    }

    core tc1 // core 1 TC16E
    {
        architecture = TC1V1.6.2;
        space_id_offset = 200;            // add 200 to all space IDs in the architecture definition
        copytable_space = vtc:linear;     // use the copy table in the virtual core for 'bss' and initialized data sections
    }

    core tc2 // core 2 TC16P
    {
        architecture = TC1V1.6.2;
        space_id_offset = 300;            // add 300 to all space IDs in the architecture definition
        copytable_space = vtc:linear;     // use the copy table in the virtual core for 'bss' and initialized data sections
    }

    core tc3 // core 3 TC16P
    {
        architecture = TC1V1.6.2;
        space_id_offset = 400;            // add 300 to all space IDs in the architecture definition
        copytable_space = vtc:linear;     // use the copy table in the virtual core for 'bss' and initialized data sections
    }

    core tc4 // core 4 TC16P
    {
        architecture = TC1V1.6.2;
        space_id_offset = 500;            // add 300 to all space IDs in the architecture definition
        copytable_space = vtc:linear;     // use the copy table in the virtual core for 'bss' and initialized data sections
    }

    core tc5 // core 5 TC16P
    {
        architecture = TC1V1.6.2;
        space_id_offset = 600;            // add 300 to all space IDs in the architecture definition
        copytable_space = vtc:linear;     // use the copy table in the virtual core for 'bss' and initialized data sections
    }

    core vtc
    {
        architecture = TC1V1.6.2;
        import tc0;                     // add all address spaces of core tc0 to core vtc for linking and locating
        import tc1;                     //                                tc1
        import tc2;                     //                                tc2
        import tc3;                     //                                tc3
        import tc4;                     //                                tc4
        import tc5;                     //                                tc5
    }

    bus sri
    {
        mau = 8;
        width = 32;

        // map shared addresses one-to-one to real cores and virtual cores
        map (dest=bus:tc0:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:tc1:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:tc2:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:tc3:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:tc4:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:tc5:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
        map (dest=bus:vtc:fpi_bus, src_offset=0, dest_offset=0, size=0xc0000000);
    }

    memory dsram5 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 96k;
        type = ram;
        map (dest=bus:tc5:fpi_bus, dest_offset=0xd0000000, size=96k, priority=8);
        map (dest=bus:sri, dest_offset=0x10000000, size=96k);
    }

    memory psram5 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 64k;
        type = ram;
        map (dest=bus:tc5:fpi_bus, dest_offset=0xc0000000, size=64k, priority=8);
        map (dest=bus:sri, dest_offset=0x10100000, size=64k);
    }

    memory dsram4 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 96k;
        type = ram;
        map (dest=bus:tc4:fpi_bus, dest_offset=0xd0000000, size=96k, priority=8);
        map (dest=bus:sri, dest_offset=0x30000000, size=96k);
    }

    memory psram4 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 64k;
        type = ram;
        map (dest=bus:tc4:fpi_bus, dest_offset=0xc0000000, size=64k, priority=8);
        map (dest=bus:sri, dest_offset=0x30100000, size=64k);
    }

    memory dsram3 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 96k;
        type = ram;
        map (dest=bus:tc3:fpi_bus, dest_offset=0xd0000000, size=96k, priority=8);
        map (dest=bus:sri, dest_offset=0x40000000, size=96k);
    }

    memory psram3 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 64k;
        type = ram;
        map (dest=bus:tc3:fpi_bus, dest_offset=0xc0000000, size=64k, priority=8);
        map (dest=bus:sri, dest_offset=0x40100000, size=64k);
    }

    memory dsram2 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 96k;
        type = ram;
        map (dest=bus:tc2:fpi_bus, dest_offset=0xd0000000, size=96k, priority=8);
        map (dest=bus:sri, dest_offset=0x50000000, size=96k);
    }

    memory psram2 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 64k;
        type = ram;
        map (dest=bus:tc2:fpi_bus, dest_offset=0xc0000000, size=64k, priority=8);
        map (dest=bus:sri, dest_offset=0x50100000, size=64k);
    }

    memory dsram1 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 240k;
        type = ram;
        map (dest=bus:tc1:fpi_bus, dest_offset=0xd0000000, size=240k, priority=8);
        map (dest=bus:sri, dest_offset=0x60000000, size=240k);
    }

    memory psram1 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 64k;
        type = ram;
        map (dest=bus:tc1:fpi_bus, dest_offset=0xc0000000, size=64k, priority=8);
        map (dest=bus:sri, dest_offset=0x60100000, size=64k);
    }

    memory dsram0 // Data Scratch Pad Ram
    {
        mau = 8;
        size = 240k;
        type = ram;
        map (dest=bus:tc0:fpi_bus, dest_offset=0xd0000000, size=240k, priority=8);
        map (dest=bus:sri, dest_offset=0x70000000, size=240k);
    }

    memory psram0 // Program Scratch Pad Ram
    {
        mau = 8;
        size = 64k;
        type = ram;
        map (dest=bus:tc0:fpi_bus, dest_offset=0xc0000000, size=64k, priority=8);
        map (dest=bus:sri, dest_offset=0x70100000, size=64k);
    }

    memory pfls0
    {
        mau = 8;
        size = 3M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80000000,           size=3M);
        map not_cached (dest=bus:sri, dest_offset=0xa0000000, reserved, size=3M);
    }

    memory pfls1
    {
        mau = 8;
        size = 3M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80300000,           size=3M);
        map not_cached (dest=bus:sri, dest_offset=0xa0300000, reserved, size=3M);
    }

    memory pfls2
    {
        mau = 8;
        size = 3M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80600000,           size=3M);
        map not_cached (dest=bus:sri, dest_offset=0xa0600000, reserved, size=3M);
    }

    memory pfls3
    {
        mau = 8;
        size = 3M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80900000,           size=3M);
        map not_cached (dest=bus:sri, dest_offset=0xa0900000, reserved, size=3M);
    }

    memory pfls4
    {
        mau = 8;
        size = 3M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80c00000,           size=3M);
        map not_cached (dest=bus:sri, dest_offset=0xa0c00000, reserved, size=3M);
    }

    memory pfls5
    {
        mau = 8;
        size = 1M;
        type = rom;
        map     cached (dest=bus:sri, dest_offset=0x80f00000,           size=1M);
        map not_cached (dest=bus:sri, dest_offset=0xa0f00000, reserved, size=1M);
    }

    memory dfls0
    {
        mau = 8;
        size = 1M;
        type = reserved nvram;
        map (dest=bus:sri, dest_offset=0xaf000000, size=1M  );
    }

    memory ucb
    {
        mau = 8;
        size = 24k;
        type = rom;
        map (dest=bus:sri, dest_offset=0xaf400000, reserved, size=24k);
    }

    memory cpu0_dlmu
    {
        mau = 8;
        size = 64k;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90000000,           size=64k);
        map not_cached (dest=bus:sri, dest_offset=0xb0000000, reserved, size=64k);
    }

    memory cpu1_dlmu
    {
        mau = 8;
        size = 64k;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90010000,           size=64k);
        map not_cached (dest=bus:sri, dest_offset=0xb0010000, reserved, size=64k);
    }

    memory cpu2_dlmu
    {
        mau = 8;
        size = 64k;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90020000,           size=64k);
        map not_cached (dest=bus:sri, dest_offset=0xb0020000, reserved, size=64k);
    }

    memory cpu3_dlmu
    {
        mau = 8;
        size = 64k;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90030000,           size=64k);
        map not_cached (dest=bus:sri, dest_offset=0xb0030000, reserved, size=64k);
    }

    memory lmuram
    {
        mau = 8;
        size = 768K;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90040000,           size=768K);
        map not_cached (dest=bus:sri, dest_offset=0xb0040000, reserved, size=768K);
    }

    memory cpu4_dlmu
    {
        mau = 8;
        size = 64k;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90100000,           size=64k);
        map not_cached (dest=bus:sri, dest_offset=0xb0100000, reserved, size=64k);
    }

    memory cpu5_dlmu
    {
        mau = 8;
        size = 64k;
        type = ram;
        map     cached (dest=bus:sri, dest_offset=0x90110000,           size=64k);
        map not_cached (dest=bus:sri, dest_offset=0xb0110000, reserved, size=64k);
    }

    memory edmem
    {
        mau = 8;
        size = 4M;
        type = ram;
        map (dest=bus:sri, dest_offset=0x99000000, size=4M);
        map (dest=bus:sri, dest_offset=0xb9000000, reserved, size=4M);
    }

#if (__VERSION__ >= 6003)
    section_setup :vtc:linear
    {
        heap "heap" (min_size = (1k), fixed, align = 8);
    }
#endif

    section_setup :vtc:linear
    {
        start_address
        (
            symbol = "_START"
        );
    }

    section_setup :vtc:linear
    {
        stack "ustack_tc0" (min_size = 1k, fixed, align = 8);
        stack "istack_tc0" (min_size = 1k, fixed, align = 8);
        stack "ustack_tc1" (min_size = 1k, fixed, align = 8);
        stack "istack_tc1" (min_size = 1k, fixed, align = 8);
        stack "ustack_tc2" (min_size = 1k, fixed, align = 8);
        stack "istack_tc2" (min_size = 1k, fixed, align = 8);
        stack "ustack_tc3" (min_size = 1k, fixed, align = 8);
        stack "istack_tc3" (min_size = 1k, fixed, align = 8);
        stack "ustack_tc4" (min_size = 1k, fixed, align = 8);
        stack "istack_tc4" (min_size = 1k, fixed, align = 8);
        stack "ustack_tc5" (min_size = 1k, fixed, align = 8);
        stack "istack_tc5" (min_size = 1k, fixed, align = 8);
    }

    /*Section setup for the copy table*/
    section_setup :vtc:linear
    {
        copytable
        (
            align = 4,
            dest = linear,
            table
            {
                symbol = "_lc_ub_table_tc0";
                space = :tc0:linear, :tc0:abs24, :tc0:abs18, :tc0:csa;
            },
            table
            {
                symbol = "_lc_ub_table_tc1";
                space = :tc1:linear, :tc1:abs24, :tc1:abs18, :tc1:csa;
            },
            table
            {
                symbol = "_lc_ub_table_tc2";
                space = :tc2:linear, :tc2:abs24, :tc2:abs18, :tc2:csa;
            },
            table
            {
                symbol = "_lc_ub_table_tc3";
                space = :tc3:linear, :tc3:abs24, :tc3:abs18, :tc3:csa;
            },
            table
            {
                symbol = "_lc_ub_table_tc4";
                space = :tc4:linear, :tc4:abs24, :tc4:abs18, :tc4:csa;
            },
            table
            {
                symbol = "_lc_ub_table_tc5";
                space = :tc5:linear, :tc5:abs24, :tc5:abs18, :tc5:csa;
            }
        );
    }

    /*Sections located at absolute fixed address*/

    section_layout :vtc:linear
    {
        /*Fixed memory Allocations for stack memory and CSA*/
        group (ordered)
        {
            group ustack5(align = 8, run_addr = mem:dsram5[LCF_USTACK5_OFFSET])
            {
                stack "ustack_tc5" (size = LCF_USTACK5_SIZE);
            }
            "__USTACK5":= sizeof(group:ustack5) > 0  ? "_lc_ue_ustack_tc5" : 0;
            "__USTACK5_END"="_lc_gb_ustack5";

            group istack5(align = 8, run_addr = mem:dsram5[LCF_ISTACK5_OFFSET])
            {
                stack "istack_tc5" (size = LCF_ISTACK5_SIZE);
            }
            "__ISTACK5":= sizeof(group:istack5) > 0  ? "_lc_ue_istack_tc5" : 0;
            "__ISTACK5_END"="_lc_gb_istack5";

            group (align = 64, attributes=rw, run_addr=mem:dsram5[LCF_CSA5_OFFSET])
                reserved "csa_tc5" (size = LCF_CSA5_SIZE);
            "__CSA5":=        "_lc_ub_csa_tc5";
            "__CSA5_END":=    "_lc_ue_csa_tc5";
        }
        group (ordered)
        {
            group ustack4(align = 8, run_addr = mem:dsram4[LCF_USTACK4_OFFSET])
            {
                stack "ustack_tc4" (size = LCF_USTACK4_SIZE);
            }
            "__USTACK4":= sizeof(group:ustack4) > 0  ? "_lc_ue_ustack_tc4" : 0;
            "__USTACK4_END"="_lc_gb_ustack4";

            group istack4(align = 8, run_addr = mem:dsram4[LCF_ISTACK4_OFFSET])
            {
                stack "istack_tc4" (size = LCF_ISTACK4_SIZE);
            }
            "__ISTACK4":= sizeof(group:istack4) > 0  ? "_lc_ue_istack_tc4" : 0;
            "__ISTACK4_END"="_lc_gb_istack4";

            group (align = 64, attributes=rw, run_addr=mem:dsram4[LCF_CSA4_OFFSET])
                reserved "csa_tc4" (size = LCF_CSA4_SIZE);
            "__CSA4":=        "_lc_ub_csa_tc4";
            "__CSA4_END":=    "_lc_ue_csa_tc4";
        }
        group (ordered)
        {
            group ustack3(align = 8, run_addr = mem:dsram3[LCF_USTACK3_OFFSET])
            {
                stack "ustack_tc3" (size = LCF_USTACK3_SIZE);
            }
            "__USTACK3":= sizeof(group:ustack3) > 0  ? "_lc_ue_ustack_tc3" : 0;
            "__USTACK3_END"="_lc_gb_ustack3";

            group istack3(align = 8, run_addr = mem:dsram3[LCF_ISTACK3_OFFSET])
            {
                stack "istack_tc3" (size = LCF_ISTACK3_SIZE);
            }
            "__ISTACK3":= sizeof(group:istack3) > 0  ? "_lc_ue_istack_tc3" : 0;
            "__ISTACK3_END"="_lc_gb_istack3";

            group (align = 64, attributes=rw, run_addr=mem:dsram3[LCF_CSA3_OFFSET])
                reserved "csa_tc3" (size = LCF_CSA3_SIZE);
            "__CSA3":=        "_lc_ub_csa_tc3";
            "__CSA3_END":=    "_lc_ue_csa_tc3";
        }
        group (ordered)
        {
            group ustack2(align = 8, run_addr = mem:dsram2[LCF_USTACK2_OFFSET])
            {
                stack "ustack_tc2" (size = LCF_USTACK2_SIZE);
            }
            "__USTACK2":= sizeof(group:ustack2) > 0  ? "_lc_ue_ustack_tc2" : 0;
            "__USTACK2_END"="_lc_gb_ustack2";

            group istack2(align = 8, run_addr = mem:dsram2[LCF_ISTACK2_OFFSET])
            {
                stack "istack_tc2" (size = LCF_ISTACK2_SIZE);
            }
            "__ISTACK2":= sizeof(group:istack2) > 0  ? "_lc_ue_istack_tc2" : 0;
            "__ISTACK2_END"="_lc_gb_istack2";

            group (align = 64, attributes=rw, run_addr=mem:dsram2[LCF_CSA2_OFFSET])
                reserved "csa_tc2" (size = LCF_CSA2_SIZE);
            "__CSA2":=        "_lc_ub_csa_tc2";
            "__CSA2_END":=    "_lc_ue_csa_tc2";
        }
        group (ordered)
        {
            group ustack1(align = 8, run_addr = mem:dsram1[LCF_USTACK1_OFFSET])
            {
                stack "ustack_tc1" (size = LCF_USTACK1_SIZE);
            }
            "__USTACK1":= sizeof(group:ustack1) > 0  ? "_lc_ue_ustack_tc1" : 0;
            "__USTACK1_END"="_lc_gb_ustack1";

            group istack1(align = 8, run_addr = mem:dsram1[LCF_ISTACK1_OFFSET])
            {
                stack "istack_tc1" (size = LCF_ISTACK1_SIZE);
            }
            "__ISTACK1":= sizeof(group:istack1) > 0  ? "_lc_ue_istack_tc1" : 0;
            "__ISTACK1_END"="_lc_gb_istack1";

            group (align = 64, attributes=rw, run_addr=mem:dsram1[LCF_CSA1_OFFSET])
                reserved "csa_tc1" (size = LCF_CSA1_SIZE);
            "__CSA1":=        "_lc_ub_csa_tc1";
            "__CSA1_END":=    "_lc_ue_csa_tc1";
        }
        group (ordered)
        {
            group ustack0(align = 8, run_addr = mem:dsram0[LCF_USTACK0_OFFSET])
            {
                stack "ustack_tc0" (size = LCF_USTACK0_SIZE);
            }
            "__USTACK0":= sizeof(group:ustack0) > 0  ? "_lc_ue_ustack_tc0" : 0;
            "__USTACK0_END"="_lc_gb_ustack0";

            group istack0(align = 8, run_addr = mem:dsram0[LCF_ISTACK0_OFFSET])
            {
                stack "istack_tc0" (size = LCF_ISTACK0_SIZE);
            }
            "__ISTACK0":= sizeof(group:istack0) > 0  ? "_lc_ue_istack_tc0" : 0;
            "__ISTACK0_END"="_lc_gb_istack0";

            group (align = 64, attributes=rw, run_addr=mem:dsram0[LCF_CSA0_OFFSET])
                reserved "csa_tc0" (size = LCF_CSA0_SIZE);
            "__CSA0":=        "_lc_ub_csa_tc0";
            "__CSA0_END":=    "_lc_ue_csa_tc0";
        }

        /*Fixed memory Allocations for _START*/
        group (ordered)
        {
            group  reset (run_addr=RESET)
            {
                section "reset" ( size = 0x20, fill = 0x0800, attributes = r )
                {
                    select ".text.start";
                }
            }
            group  interface_const (run_addr=mem:pfls0[0x0020])
            {
                select "*.interface_const";
            }
            "__IF_CONST" := addressof(group:interface_const);
            "__START0" := LCF_STARTPTR_NC_CPU0;
            "__START1" := LCF_STARTPTR_NC_CPU1;
            "__START2" := LCF_STARTPTR_NC_CPU2;
            "__START3" := LCF_STARTPTR_NC_CPU3;
            "__START4" := LCF_STARTPTR_NC_CPU4;
            "__START5" := LCF_STARTPTR_NC_CPU5;
        }

        /*Fixed memory Allocations for Trap Vector Table*/
        group (ordered)
        {
            group trapvec_tc0 (align = 8, run_addr=LCF_TRAPVEC0_START)
            {
                section "trapvec_tc0" (size=0x100, attributes=rx, fill=0)
                {
                    select "(.text.traptab_cpu0*)";
                }
            }
            group trapvec_tc1 (align = 8, run_addr=LCF_TRAPVEC1_START)
            {
                section "trapvec_tc1" (size=0x100, attributes=rx, fill=0)
                {
                    select "(.text.traptab_cpu1*)";
                }
            }
            group trapvec_tc2 (align = 8, run_addr=LCF_TRAPVEC2_START)
            {
                section "trapvec_tc2" (size=0x100, attributes=rx, fill=0)
                {
                    select "(.text.traptab_cpu2*)";
                }
            }
            group trapvec_tc3 (align = 8, run_addr=LCF_TRAPVEC3_START)
            {
                section "trapvec_tc3" (size=0x100, attributes=rx, fill=0)
                {
                    select "(.text.traptab_cpu3*)";
                }
            }
            group trapvec_tc4 (align = 8, run_addr=LCF_TRAPVEC4_START)
            {
                section "trapvec_tc4" (size=0x100, attributes=rx, fill=0)
                {
                    select "(.text.traptab_cpu4*)";
                }
            }
            group trapvec_tc5 (align = 8, run_addr=LCF_TRAPVEC5_START)
            {
                section "trapvec_tc5" (size=0x100, attributes=rx, fill=0)
                {
                    select "(.text.traptab_cpu5*)";
                }
            }
            "__TRAPTAB_CPU0" := TRAPTAB0;
            "__TRAPTAB_CPU1" := TRAPTAB1;
            "__TRAPTAB_CPU2" := TRAPTAB2;
            "__TRAPTAB_CPU3" := TRAPTAB3;
            "__TRAPTAB_CPU4" := TRAPTAB4;
            "__TRAPTAB_CPU5" := TRAPTAB5;
        }

        /*Fixed memory Allocations for Start up code*/
        group (ordered)
        {
            group start_tc0 (run_addr=LCF_STARTPTR_NC_CPU0)
            {
                select "(.text.start_cpu0*)";
            }
            group start_tc1 (run_addr=LCF_STARTPTR_NC_CPU1)
            {
                select "(.text.start_cpu1*)";
            }
            group start_tc2 (run_addr=LCF_STARTPTR_NC_CPU2)
            {
                select "(.text.start_cpu2*)";
            }
            group start_tc3 (run_addr=LCF_STARTPTR_NC_CPU3)
            {
                select "(.text.start_cpu3*)";
            }
            group start_tc4 (run_addr=LCF_STARTPTR_NC_CPU4)
            {
                select "(.text.start_cpu4*)";
            }
            group start_tc5 (run_addr=LCF_STARTPTR_NC_CPU5)
            {
                select "(.text.start_cpu5*)";
            }
            "__ENABLE_INDIVIDUAL_C_INIT_CPU0" := 0; /* Not used */
            "__ENABLE_INDIVIDUAL_C_INIT_CPU1" := 0;
            "__ENABLE_INDIVIDUAL_C_INIT_CPU2" := 0;
            "__ENABLE_INDIVIDUAL_C_INIT_CPU3" := 0;
            "__ENABLE_INDIVIDUAL_C_INIT_CPU4" := 0;
            "__ENABLE_INDIVIDUAL_C_INIT_CPU5" := 0;
        }

        /*Fixed memory Allocations for Interrupt Vector Table*/
        group (ordered)
        {
            group int_tab_tc0 (ordered)
            {
#                include "inttab0.lsl"
            }
            group int_tab_tc1 (ordered)
            {
#                include "inttab1.lsl"
            }
            group int_tab_tc2 (ordered)
            {
#                include "inttab2.lsl"
            }
            group int_tab_tc3 (ordered)
            {
#                include "inttab3.lsl"
            }
            group int_tab_tc4 (ordered)
            {
#                include "inttab4.lsl"
            }
            group int_tab_tc5 (ordered)
            {
#                include "inttab5.lsl"
            }
            "_lc_u_int_tab" = (LCF_INTVEC0_START);
            "__INTTAB_CPU0" = (LCF_INTVEC0_START);
            "__INTTAB_CPU1" = (LCF_INTVEC1_START);
            "__INTTAB_CPU2" = (LCF_INTVEC2_START);
            "__INTTAB_CPU3" = (LCF_INTVEC3_START);
            "__INTTAB_CPU4" = (LCF_INTVEC4_START);
            "__INTTAB_CPU5" = (LCF_INTVEC5_START);
        }

        /*Fixed memory Allocations for BMHD*/
        group (ordered)
        {
            group  bmh_0_orig (run_addr=mem:ucb[0x0000])
            {
                select ".rodata.bmhd_0_orig";
            }
            group  bmh_1_orig (run_addr=mem:ucb[0x0200])
            {
                select ".rodata.bmhd_1_orig";
            }
            group  bmh_2_orig (run_addr=mem:ucb[0x0400])
            {
                select ".rodata.bmhd_2_orig";
            }
            group  bmh_3_orig (run_addr=mem:ucb[0x0600])
            {
                select ".rodata.bmhd_3_orig";
            }
            group  bmh_blank (run_addr=mem:ucb[0x0800])
            {
            }
            group  bmh_0_copy (run_addr=mem:ucb[0x1000])
            {
                select ".rodata.bmhd_0_copy";
            }
            group  bmh_1_copy (run_addr=mem:ucb[0x1200])
            {
                select ".rodata.bmhd_1_copy";
            }
            group  bmh_2_copy (run_addr=mem:ucb[0x1400])
            {
                select ".rodata.bmhd_2_copy";
            }
            group  bmh_3_copy (run_addr=mem:ucb[0x1600])
            {
                select ".rodata.bmhd_3_copy";
            }
        }
    }

    /*Near Abbsolute Addressable Data Sections*/
    section_layout :vtc:abs18
    {
        /*Near Absolute Data, selectable with patterns and user defined sections*/
        group
        {
            group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram5)
            {
                select "(.zdata.zdata_cpu5|.zdata.zdata_cpu5.*)";
                select "(.zbss.zbss_cpu5|.zbss.zbss_cpu5.*)";
            }

            group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram4)
            {
                select "(.zdata.zdata_cpu4|.zdata.zdata_cpu4.*)";
                select "(.zbss.zbss_cpu4|.zbss.zbss_cpu4.*)";
            }

            group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram3)
            {
                select "(.zdata.zdata_cpu3|.zdata.zdata_cpu3.*)";
                select "(.zbss.zbss_cpu3|.zbss.zbss_cpu3.*)";
            }

            group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
            {
                select "(.zdata.zdata_cpu2|.zdata.zdata_cpu2.*)";
                select "(.zbss.zbss_cpu2|.zbss.zbss_cpu2.*)";
            }

            group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
            {
                select "(.zdata.zdata_cpu1|.zdata.zdata_cpu1.*)";
                select "(.zbss.zbss_cpu1|.zbss.zbss_cpu1.*)";
            }

            group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
            {
                select "(.zdata.zdata_cpu0|.zdata.zdata_cpu0.*)";
                select "(.zbss.zbss_cpu0|.zbss.zbss_cpu0.*)";
            }

            group (ordered, attributes=rw, run_addr = mem:cpu0_dlmu)
            {
                select "(.zdata.zlmudata|.zdata.zlmudata.*)";
                select "(.zbss.zlmubss|.zbss.zlmubss.*)";
            }
        }

        /*Near Absolute Data, selectable by toolchain*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram5)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram4)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram3)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
#        endif
        {
            group zdata_mcal(attributes=rw)
            {
                select ".zdata.dsprInit.cpu0.32bit";
                select ".zdata.dsprInit.cpu0.16bit";
                select ".zdata.dsprInit.cpu0.8bit";
            }

            group zdata_powerOn(attributes=rw)
            {
                select ".zdata.dsprPowerOnInit.cpu0.32bit";
                select ".zdata.dsprPowerOnInit.cpu0.16bit";
                select ".zdata.dsprPowerOnInit.cpu0.8bit";
            }

            group zbss_mcal(attributes=rw)
            {
                select ".zbss.dsprClearOnInit.cpu0.32bit";
                select ".zbss.dsprClearOnInit.cpu0.16bit";
                select ".zbss.dsprClearOnInit.cpu0.8bit";
            }

            group zbss_noClear(attributes=rw)
            {
                select ".zbss.dsprNoInit.cpu0.32bit";
                select ".zbss.dsprNoInit.cpu0.16bit";
                select ".zbss.dsprNoInit.cpu0.8bit";
            }

            group zbss_powerOn(attributes=rw)
            {
                select ".zbss.dsprPowerOnClear.cpu0.32bit";
                select ".zbss.dsprPowerOnClear.cpu0.16bit";
                select ".zbss.dsprPowerOnClear.cpu0.8bit";
            }

            group zdata(attributes=rw)
            {
                select "(.zdata|.zdata.*)";
                select "(.zbss|.zbss.*)";
            }
        }

        /*Near Absolute Const, selectable with patterns and user defined sections*/
        group
        {
            group (ordered, align = 4, contiguous, run_addr=mem:pfls0)
            {
                select ".zrodata.Ifx_Ssw_Tc0.*";
                select ".zrodata.Ifx_Ssw_Tc1.*";
                select ".zrodata.Ifx_Ssw_Tc2.*";
                select ".zrodata.Ifx_Ssw_Tc3.*";
                select ".zrodata.Ifx_Ssw_Tc4.*";
                select ".zrodata.Ifx_Ssw_Tc5.*";
                select ".zrodata.Cpu0_Main.*";
                select ".zrodata.Cpu1_Main.*";
                select ".zrodata.Cpu2_Main.*";
                select ".zrodata.Cpu3_Main.*";
                select ".zrodata.Cpu4_Main.*";
                select ".zrodata.Cpu5_Main.*";

                /*Near Absolute Const, selectable by toolchain*/
                select ".zrodata.const.cpu0.32bit";
                select ".zrodata.const.cpu0.16bit";
                select ".zrodata.const.cpu0.8bit";
                select ".zrodata.config.cpu0.32bit";
                select ".zrodata.config.cpu0.16bit";
                select ".zrodata.config.cpu0.8bit";
                select "(.zrodata|.zrodata.*)";
            }
        }
    }

    /*Relative A0/A1/A8/A9 Addressable Sections*/
    section_layout :vtc:linear
    {
        /*Relative A0 Addressable Data, selectable by toolchain*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram5)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram4)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram3)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group a0 (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
#        endif
        {
            select "(.data_a0.sdata|.data_a0.sdata.*)";
            select "(.bss_a0.sbss|.bss_a0.sbss.*)";
        }
        "_SMALL_DATA_" := sizeof(group:a0) > 0 ? addressof(group:a0) : addressof(group:a0) & 0xF0000000 + 32k;
        "__A0_MEM" = "_SMALL_DATA_";

        /*Relative A1 Addressable Const, selectable by toolchain*/
        /*Small constant sections, No option given for CPU specific user sections to make generated code portable across Cpus*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group  a1 (ordered, align = 4, run_addr=mem:pfls5)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group  a1 (ordered, align = 4, run_addr=mem:pfls4)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group  a1 (ordered, align = 4, run_addr=mem:pfls3)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group  a1 (ordered, align = 4, run_addr=mem:pfls2)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group  a1 (ordered, align = 4, run_addr=mem:pfls1)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group  a1 (ordered, align = 4, run_addr=mem:pfls0)
#        endif
        {
            select "(.rodata_a1.srodata|.rodata_a1.srodata.*)";
            select "(.ldata|.ldata.*)";
        }
        "_LITERAL_DATA_" := sizeof(group:a1) > 0 ? addressof(group:a1) : addressof(group:a1) & 0xF0000000 + 32k;
        "__A1_MEM" = "_LITERAL_DATA_";

        /*Relative A9 Addressable Data, selectable with patterns and user defined sections*/
        group a9 (ordered, align = 4, run_addr=mem:lmuram)
        {
            select "(.data_a9.a9sdata|.data_a9.a9sdata.*)";
            select "(.bss_a9.a9sbss|.bss_a9.a9sbss.*)";
        }
        "_A9_DATA_" := sizeof(group:a9) > 0 ? addressof(group:a9) : addressof(group:a9) & 0xF0000000 + 32k;
        "__A9_MEM" = "_A9_DATA_";

        /*Relative A8 Addressable Const, selectable with patterns and user defined sections*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group  a8 (ordered, align = 4, run_addr=mem:pfls5)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group  a8 (ordered, align = 4, run_addr=mem:pfls4)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group  a8 (ordered, align = 4, run_addr=mem:pfls3)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group  a8 (ordered, align = 4, run_addr=mem:pfls2)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group  a8 (ordered, align = 4, run_addr=mem:pfls1)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group  a8 (ordered, align = 4, run_addr=mem:pfls0)
#        endif
        {
            select "(.rodata_a8.a8srodata|.rodata_a8.a8srodata.*)";
        }
        "_A8_DATA_" := sizeof(group:a8) > 0 ? addressof(group:a8) : addressof(group:a8) & 0xF0000000 + 32k;
        "__A8_MEM" = "_A8_DATA_";
    }

    /*Far Data / Far Const Sections, selectable with patterns and user defined sections*/
    section_layout :vtc:linear
    {
        /*Far Data Sections, selectable with patterns and user defined sections*/
        group
        {
            /*DSRAM sections*/
            group
            {
                group (ordered, attributes=rw, run_addr=mem:dsram5)
                {
                    select ".data.Ifx_Ssw_Tc5.*";
                    select ".data.Cpu5_Main.*";
                    select "(.data.data_cpu5|.data.data_cpu5.*)";
                    select ".bss.Ifx_Ssw_Tc5.*";
                    select ".bss.Cpu5_Main.*";
                    select "(.bss.bss_cpu5|.bss.bss_cpu5.*)";
                }
                group (ordered, attributes=rw, run_addr=mem:dsram4)
                {
                    select ".data.Ifx_Ssw_Tc4.*";
                    select ".data.Cpu4_Main.*";
                    select "(.data.data_cpu4|.data.data_cpu4.*)";
                    select ".bss.Ifx_Ssw_Tc4.*";
                    select ".bss.Cpu4_Main.*";
                    select "(.bss.bss_cpu4|.bss.bss_cpu4.*)";
                }
                group (ordered, attributes=rw, run_addr=mem:dsram3)
                {
                    select ".data.Ifx_Ssw_Tc3.*";
                    select ".data.Cpu3_Main.*";
                    select "(.data.data_cpu3|.data.data_cpu3.*)";
                    select ".bss.Ifx_Ssw_Tc3.*";
                    select ".bss.Cpu3_Main.*";
                    select "(.bss.bss_cpu3|.bss.bss_cpu3.*)";
                }
                group (ordered, attributes=rw, run_addr=mem:dsram2)
                {
                    select ".data.Ifx_Ssw_Tc2.*";
                    select ".data.Cpu2_Main.*";
                    select "(.data.data_cpu2|.data.data_cpu2.*)";
                    select ".bss.Ifx_Ssw_Tc2.*";
                    select ".bss.Cpu2_Main.*";
                    select "(.bss.bss_cpu2|.bss.bss_cpu2.*)";
                }
                group (ordered, attributes=rw, run_addr=mem:dsram1)
                {
                    select ".data.Ifx_Ssw_Tc1.*";
                    select ".data.Cpu1_Main.*";
                    select "(.data.data_cpu1|.data.data_cpu1.*)";
                    select ".bss.Ifx_Ssw_Tc1.*";
                    select ".bss.Cpu1_Main.*";
                    select "(.bss.bss_cpu1|.bss.bss_cpu1.*)";
                }
                group (ordered, attributes=rw, run_addr=mem:dsram0)
                {
                    select ".data.Ifx_Ssw_Tc0.*";
                    select ".data.Cpu0_Main.*";
                    select "(.data.data_cpu0|.data.data_cpu0.*)";
                    select ".bss.Ifx_Ssw_Tc0.*";
                    select ".bss.Cpu0_Main.*";
                    select "(.bss.bss_cpu0|.bss.bss_cpu0.*)";
                }
            }

            /*LMU Data sections*/
            group
            {
                group (ordered, attributes=rw, run_addr = mem:cpu0_dlmu)
                {
                    select "(.data.lmudata_cpu0|.data.lmudata_cpu0.*)";
                    select "(.bss.lmubss_cpu0|.bss.lmubss_cpu0.*)";
                }
                group (ordered, attributes=rw, run_addr = mem:cpu1_dlmu)
                {
                    select "(.data.lmudata_cpu1|.data.lmudata_cpu1.*)";
                    select "(.bss.lmubss_cpu1|.bss.lmubss_cpu1.*)";
                }
                group (ordered, attributes=rw, run_addr = mem:cpu2_dlmu)
                {
                    select "(.data.lmudata_cpu2|.data.lmudata_cpu2.*)";
                    select "(.bss.lmubss_cpu2|.bss.lmubss_cpu2.*)";
                }
                group (ordered, attributes=rw, run_addr = mem:cpu3_dlmu)
                {
                    select "(.data.lmudata_cpu3|.data.lmudata_cpu3.*)";
                    select "(.bss.lmubss_cpu3|.bss.lmubss_cpu3.*)";
                }
                group (ordered, attributes=rw, run_addr = mem:cpu4_dlmu)
                {
                    select "(.data.lmudata_cpu4|.data.lmudata_cpu4.*)";
                    select "(.bss.lmubss_cpu4|.bss.lmubss_cpu4.*)";
                }
                group (ordered, attributes=rw, run_addr = mem:cpu5_dlmu)
                {
                    select "(.data.lmudata_cpu5|.data.lmudata_cpu5.*)";
                    select "(.bss.lmubss_cpu5|.bss.lmubss_cpu5.*)";
                }
                group (ordered, attributes=rw, run_addr=mem:lmuram)
                {
                    select "(.data.lmudata|.data.lmudata.*)";
                    select "(.bss.lmubss|.bss.lmubss.*)";
                }
            }
        }

        /*Far Data Sections, selectable by toolchain*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram5)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram4)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram3)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram2)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram1)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, contiguous, align = 4, attributes=rw, run_addr = mem:dsram0)
#        endif
        {
            group data_mcal(attributes=rw)
            {
                select ".data.farDsprInit.cpu0.32bit";
                select ".data.farDsprInit.cpu0.16bit";
                select ".data.farDsprInit.cpu0.8bit";
            }

            group bss_mcal(attributes=rw)
            {
                select ".bss.farDsprClearOnInit.cpu0.32bit";
                select ".bss.farDsprClearOnInit.cpu0.16bit";
                select ".bss.farDsprClearOnInit.cpu0.8bit";
            }

            group bss_noInit(attributes=rw)
            {
                select ".bss.farDsprNoInit.cpu0.32bit";
                select ".bss.farDsprNoInit.cpu0.16bit";
                select ".bss.farDsprNoInit.cpu0.8bit";
            }

            group data(attributes=rw)
            {
                select "(.data|.data.*)";
                select "(.bss|.bss.*)";
            }
        }

        /*Heap allocation*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group (ordered, align = 4, run_addr = mem:dsram5[LCF_HEAP5_OFFSET])
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group (ordered, align = 4, run_addr = mem:dsram4[LCF_HEAP4_OFFSET])
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group (ordered, align = 4, run_addr = mem:dsram3[LCF_HEAP3_OFFSET])
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, align = 4, run_addr = mem:dsram2[LCF_HEAP2_OFFSET])
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, align = 4, run_addr = mem:dsram1[LCF_HEAP1_OFFSET])
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, align = 4, run_addr = mem:dsram0[LCF_HEAP0_OFFSET])
#        endif
        {
            heap "heap" (size = LCF_HEAP_SIZE);
        }

        /*Far Const Sections, selectable with patterns and user defined sections*/
        group
        {
            group (ordered, align = 4, run_addr=mem:pfls0)
            {
                select ".rodata.Ifx_Ssw_Tc0.*";
                select ".rodata.Cpu0_Main.*";
                select "(.rodata.rodata_cpu0|.rodata.rodata_cpu0.*)";
            }
            group (ordered, align = 4, run_addr=mem:pfls1)
            {
                select ".rodata.Cpu1_Main.*";
                select ".rodata.Ifx_Ssw_Tc1.*";
                select "(.rodata.rodata_cpu1|.rodata.rodata_cpu1.*)";
            }
            group (ordered, align = 4, run_addr=mem:pfls2)
            {
                select ".rodata.Ifx_Ssw_Tc2.*";
                select ".rodata.Cpu2_Main.*";
                select "(.rodata.rodata_cpu2|.rodata.rodata_cpu2.*)";
            }
            group (ordered, align = 4, run_addr=mem:pfls3)
            {
                select ".rodata.Ifx_Ssw_Tc3.*";
                select ".rodata.Cpu3_Main.*";
                select "(.rodata.rodata_cpu3|.rodata.rodata_cpu3.*)";
            }
            group (ordered, align = 4, run_addr=mem:pfls4)
            {
                select ".rodata.Ifx_Ssw_Tc4.*";
                select ".rodata.Cpu4_Main.*";
                select "(.rodata.rodata_cpu4|.rodata.rodata_cpu4.*)";
            }
            group (ordered, align = 4, run_addr=mem:pfls5)
            {
                select ".rodata.Ifx_Ssw_Tc5.*";
                select ".rodata.Cpu5_Main.*";
                select "(.rodata.rodata_cpu5|.rodata.rodata_cpu5.*)";
            }
        }

        /*Far Const Sections, selectable by toolchain*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group (ordered, align = 4, run_addr=mem:pfls5)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group  ordered, align = 4, run_addr=mem:pfls4)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group (ordered, align = 4, run_addr=mem:pfls3)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, align = 4, run_addr=mem:pfls2)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, align = 4, run_addr=mem:pfls1)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, align = 4, run_addr=mem:pfls0)
#        endif
        {
            select ".rodata.farConst.cpu0.32bit";
            select ".rodata.farConst.cpu0.16bit";
            select ".rodata.farConst.cpu0.8bit";
            select "(.rodata|.rodata.*)";
        }
    }

    /* PSRAM Code selections*/
    section_layout :vtc:linear
    {
        /*Code Sections, selectable with patterns and user defined sections*/
        group
        {
            /*Program Scratchpad Sections*/
            group
            {
                group code_psram0 (ordered, attributes=rwx, copy, run_addr=mem:psram0)
                {
                    select "(.text.cpu0_psram|.text.cpu0_psram.*)";
                    select "(.text.psram_text_cpu0|.text.psram_text_cpu0.*)";
                }
                group code_psram1 (ordered, attributes=rwx, copy, run_addr=mem:psram1)
                {
                    select "(.text.cpu1_psram|.text.cpu1_psram.*)";
                    select "(.text.psram_text_cpu1|.text.psram_text_cpu1.*)";
                }
                group code_psram2 (ordered, attributes=rwx, copy, run_addr=mem:psram2)
                {
                    select "(.text.cpu2_psram|.text.cpu2_psram.*)";
                    select "(.text.psram_text_cpu2|.text.psram_text_cpu2.*)";
                }
                group code_psram3 (ordered, attributes=rwx, copy, run_addr=mem:psram3)
                {
                    select "(.text.cpu3_psram|.text.cpu3_psram.*)";
                    select "(.text.psram_text_cpu3|.text.psram_text_cpu3.*)";
                }
                group code_psram4 (ordered, attributes=rwx, copy, run_addr=mem:psram4)
                {
                    select "(.text.cpu4_psram|.text.cpu4_psram.*)";
                    select "(.text.psram_text_cpu4|.text.psram_text_cpu4.*)";
                }
                group code_psram5 (ordered, attributes=rwx, copy, run_addr=mem:psram5)
                {
                    select "(.text.cpu5_psram|.text.cpu5_psram.*)";
                    select "(.text.psram_text_cpu5|.text.psram_text_cpu5.*)";
                }
            }
        }
    }

    /* FLS Code selections*/
    section_layout :vtc:linear
    {
        /*Code Sections, selectable with patterns and user defined sections*/
        group
        {
            /*Cpu specific PFLASH Sections*/
            group
            {
                group (ordered, align = 4, run_addr=mem:pfls0)
                {
                    select ".text.Ifx_Ssw_Tc0.*";
                    select ".text.Cpu0_Main.*";
                    select ".text.CompilerTasking.Ifx_C_Init";
                    select "(.text.text_cpu0|.text.text_cpu0.*)";
                }
                group (ordered, align = 4, run_addr=mem:pfls1)
                {
                    select ".text.Ifx_Ssw_Tc1.*";
                    select ".text.Cpu1_Main.*";
                    select "(.text.text_cpu1|.text.text_cpu1.*)";
                }
                group (ordered, align = 4, run_addr=mem:pfls2)
                {
                    select ".text.Ifx_Ssw_Tc2.*";
                    select ".text.Cpu2_Main.*";
                    select "(.text.text_cpu2|.text.text_cpu2.*)";
                }
                group (ordered, align = 4, run_addr=mem:pfls3)
                {
                    select ".text.Ifx_Ssw_Tc3.*";
                    select ".text.Cpu3_Main.*";
                    select "(.text.text_cpu3|.text.text_cpu3.*)";
                }
                group (ordered, align = 4, run_addr=mem:pfls4)
                {
                    select ".text.Ifx_Ssw_Tc4.*";
                    select ".text.Cpu4_Main.*";
                    select ".text.text_cpu4*";
                    select "(.text.text_cpu4|.text.text_cpu4.*)";
                }
                group (ordered, align = 4, run_addr=mem:pfls5)
                {
                    select ".text.Ifx_Ssw_Tc5.*";
                    select ".text.Cpu5_Main.*";
                    select ".text.text_cpu5*";
                    select "(.text.text_cpu5|.text.text_cpu5.*)";
                }
            }
        }

        /*Code Sections, selectable by toolchain*/
#        if LCF_DEFAULT_HOST == LCF_CPU5
        group (ordered, run_addr=mem:pfls5)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU4
        group  ordered, run_addr=mem:pfls4)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU3
        group (ordered, run_addr=mem:pfls3)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU2
        group (ordered, run_addr=mem:pfls2)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU1
        group (ordered, run_addr=mem:pfls1)
#        endif
#        if LCF_DEFAULT_HOST == LCF_CPU0
        group (ordered, run_addr=mem:pfls0)
#        endif
        {
            select ".text.fast.pfls.cpu0";
            select ".text.slow.pfls.cpu0";
            select ".text.5ms.pfls.cpu0";
            select ".text.10ms.pfls.cpu0";
            select ".text.callout.pfls.cpu0";
            select "(.text|.text.*)";
        }
    }
}
