/****************************************************************************
 * arch/x86_64/src/common/multiboot1.S
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

/* Multiboot1 NuttX Naive Loader */

.balign 16
.code32

.globl _start

/* NuttX binary with multiboot1 header */

.section                ".bin" , "ax"
bin_start:
    .incbin             "../../../nuttx.bin"

multiboot_info_struct:
    .long               0
multiboot_magic:
    .long               0

.section                ".text" , "ax"
_start:

    /* We should manually copy .realmode section */

    /* Saving multiboot args */

    movl                %ebx, multiboot_info_struct
    movl                %eax, multiboot_magic

    /* memcpy(0, realmode_start, realmode_size) */

    movl                (realmode_size), %ecx
    movl                $realmode_start, %esi
    movl                $0, %edi

copy_loop:

    /* Copy by bytes, make sure the addresses are not overlapped */

    movb                (%esi), %al
    movb                %al, (%edi)
    inc                 %esi
    inc                 %edi
    loop                copy_loop

    /* Jump to bin_start + 0x30, skip the multiboot1 header */

    movl                (multiboot_info_struct), %ebx
    movl                (multiboot_magic), %eax
    movl                $bin_start, %ecx
    addl                $0x30, %ecx
    jmp                 *%ecx


/* NuttX realmode section */

.section                ".realmode", "ax"
realmode_start:
    .incbin             "../../../nuttx_realmode.bin"

.align 8
realmode_size:
    .long               . - realmode_start


