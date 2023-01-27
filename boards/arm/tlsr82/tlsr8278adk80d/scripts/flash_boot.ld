/****************************************************************************
 * board/arm/src/tlsr82/tlsr8278adk80d/scripts/flash_boot.ld
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

/* to tell the linker the program begin from __start label in cstartup.s,
 * thus do not treat it as a unused symbol
 */

ENTRY(__start)

SECTIONS
{
    . = 0x0;
    .vectors :
    {
        *(.vectors)

        /* MUST as follows, when compile with -ffunction-sections
         * -fdata-sections, session name may changed
         */

        *(.vectors.*)
    }

    .ram_code :
    {
        PROVIDE(_sramcode = ABSOLUTE(.));
        *(.ram_code)
        *(.ram_code.*)
        PROVIDE(_eramcode = ABSOLUTE(.));
    }

    PROVIDE(_ramcode_size_ = . );
    PROVIDE(_ramcode_size_div_16_ = (. + 15 ) / 16);
    PROVIDE(_ramcode_size_div_256_ = (. + 255) / 256);
    PROVIDE(_ramcode_size_div_16_align_256_ = ( (. + 255) / 256) * 16);
    PROVIDE(_ramcode_size_align_256_ = ( (. + 255) / 256) * 256);

    . =  _ramcode_size_align_256_ ; /* >= (_ictag_start_ - 0x840000)*/

    .text :
    {
        PROVIDE(_stext = ABSOLUTE(.));
        *(.text)
        *(.text.*)
        PROVIDE(_etext = ABSOLUTE(.));
    }

    .rodata :
    {
        *(.rodata)
        *(.rodata.*)
    }

    . = ALIGN(4);
    PROVIDE(_rstored_ = . );
    PROVIDE(_code_size_ = .);

    /* __LOAD_FLASH for MCU RUN IN Flash 0x100 aligned, must greater
     * than or equal to : 0x808000 + ram_code_size + irq_vector(0x100) +
     * IC_tag(0x100) + IC_cache(0x800) == 0x808a00 + ram_code_size
     *
     * __LOAD_DUT for MCU RUN IN SRAM, this setting is used for DUT
     * Test and bin size is limited to 8k
     *
     * __LOAD_RAM for MCU RUN IN SRAM
     */

    . = (0x840000 + ((0x900 + _ramcode_size_align_256_) * __LOAD_FLASH) + (0x400 * __LOAD_DUT) + (_rstored_ * __LOAD_RAM));

    .retention_data : AT ( _rstored_ )
    {
        . = ALIGN(4);
        PROVIDE(_retention_data_start_ = . );
        *(.retention_data)
        *(.retention_data.*)
        . = ALIGN(4);
        PROVIDE(_retention_data_end_ = . );
    }
    PROVIDE(_dstored_ = _rstored_ + _retention_data_end_ - _retention_data_start_);

    .my_code(NOLOAD) :
    {
        *(.spi_slave_buff)
    }

    .data : AT ( _dstored_ )
    {
        . = ALIGN(4);
        PROVIDE(_start_data_ = . );
        *(.data);
        *(.data.*);
        . = ALIGN(4);
        PROVIDE(_end_data_ = . );
    }

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_sbss = .);
        *(.sbss)
        *(.sbss.*)
        *(.bss)
        *(.bss.*)
    }

    /* data in ram but no need to clean in .s*/

    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.noinit)
        *(.noinit.*)
    }
    . = ALIGN(4);

    PROVIDE(_ebss = .);
    PROVIDE(_bin_size_ = _code_size_ + _end_data_ - _start_data_ + _retention_data_end_ - _retention_data_start_ );
    PROVIDE(_bin_size_div_16 = ( _bin_size_ + 15 ) / 16 );
    PROVIDE(_ictag_start_ = 0x840000 + _ramcode_size_align_256_ );
    PROVIDE(_ictag_end_ = 0x840000 + _ramcode_size_align_256_ + 0x100);
}
