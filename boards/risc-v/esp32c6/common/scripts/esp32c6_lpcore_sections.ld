/****************************************************************************
 * boards/risc-v/esp32c6/common/scripts/esp32c6_lpcore_sections.ld
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

#include <nuttx/config.h>

ENTRY(reset_vector)

MEMORY
{
    /*first 128byte for exception/interrupt vectors*/

    vector_table(RX) : ORIGIN = (0x50000000) , LENGTH = 0x80
    ram(RWX) : ORIGIN = (0x50000000) + 0x80, LENGTH = (CONFIG_ESPRESSIF_ULP_COPROC_RESERVE_MEM_SIZE & ~(0x8 - 1)) - 0x80 - CONFIG_ESPRESSIF_ULP_SHARED_MEM_SIZE
    shared_mem_ram(RW) : ORIGIN = (0x50000000) + (CONFIG_ESPRESSIF_ULP_COPROC_RESERVE_MEM_SIZE & ~(0x8 - 1)) - CONFIG_ESPRESSIF_ULP_SHARED_MEM_SIZE, LENGTH = CONFIG_ESPRESSIF_ULP_SHARED_MEM_SIZE
}

SECTIONS
{
    .vector.text :
    {
        /*exception/interrupt vectors*/
        __mtvec_base = .;
        KEEP (*(.init.vector .init.vector.*))
    } > vector_table

    . = ORIGIN(ram);

    .text ALIGN(4):
    {
        *(.text.vectors) /* Default reset vector must link to offset 0x80 */
        *(.text)
        *(.text*)
    } >ram

    .rodata ALIGN(4):
    {
        *(.rodata)
        *(.rodata*)
    } > ram

    .data ALIGN(4):
    {
        *(.data)
        *(.data*)
        *(.sdata)
        *(.sdata*)
    } > ram

    .bss ALIGN(4) :
    {
        *(.bss)
        *(.bss*)
        *(.sbss)
        *(.sbss*)
        PROVIDE(end = .);
    } >ram

    __stack_top = ORIGIN(ram) + LENGTH(ram);

    . = ORIGIN(shared_mem_ram);
    .shared_mem (ALIGN(4)) :
    {
        KEEP(*(.shared_mem))
    } > shared_mem_ram
}
