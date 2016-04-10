/*
 * Linker script for flashed applications on the Compal E86
 *
 * This script creates a binary that can be linked at 0xFFFF, starting
 * with the second flash page. This is what a phone application or
 * pure layer1 device uses.
 *
 * XXX: interrupts?
 *
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
MEMORY
{
    LOADR (rx) : ORIGIN = 0x00000000, LENGTH = 0x10000
    /* 4 MBytes of external flash memory (minus loader) */
    FLASH (rx) : ORIGIN = 0x00010000, LENGTH = 0x3F0000
    /* 256 kBytes of internal zero-waitstate sram */
    IRAM  (rw) : ORIGIN = 0x00800000, LENGTH = 0x040000
    /* 256 kBytes of external slow sram */
    ERAM  (rw) : ORIGIN = 0x01000000, LENGTH = 0x040000
}
SECTIONS
{
    /* entrypoint */
    .text.start : {
        PROVIDE(_start = .);
        KEEP(*(.text.start))
        *(.text.start)
    } > FLASH

    /* exception vectors from 0x80001c to 0x800034 */
    .text.exceptions 0x80001c : {
        KEEP(*(.text.exceptions))
        * (.text.exceptions)
        . = ALIGN(4);
    } > IRAM AT> FLASH
    PROVIDE(_exceptions = LOADADDR(.text.exceptions));

    /* code */
    .text : {
    _stext = ABSOLUTE(.) ;
        /* regular code */
        *(.text*)
        /* gcc voodoo */
        *(.glue_7t) *(.glue_7) *(.vfp11_veneer) *(.v4_bx)
    _etext = ABSOLUTE(.) ;
    } > FLASH
    PROVIDE(_text_start = ADDR(.text));
    PROVIDE(_text_end = ADDR(.text) + SIZEOF(.text));

    /* constructor pointers */
    .ctors : {
        /* ctor count */
        LONG(SIZEOF(.ctors) / 4 - 2)
        /* ctor pointers */
        KEEP(*(SORT(.ctors)))
        /* end of list */
        LONG(0)
    } > FLASH
    PROVIDE(_ctor_start = LOADADDR(.ctors));
    PROVIDE(_ctor_end = LOADADDR(.ctors) + SIZEOF(.ctors));

    /* destructor pointers */
    .dtors : {
        /* dtor count */
        LONG(SIZEOF(.dtors) / 4 - 2)
        /* dtor pointers */
        KEEP(*(SORT(.dtors)))
        /* end of list */
        LONG(0)
    } > FLASH
    PROVIDE(_dtor_start = LOADADDR(.dtors));
    PROVIDE(_dtor_end = LOADADDR(.dtors) + SIZEOF(.dtors));

    /* read-only data */
    .rodata : {
        *(.rodata*)
    _eronly = ABSOLUTE(.) ;
    } > FLASH
    PROVIDE(_rodata_start = ADDR(.rodata));
    PROVIDE(_rodata_end = ADDR(.rodata) + SIZEOF(.rodata));

    /* pic offset tables */
    .got : {
        . = ALIGN(4);
        *(.got)
        *(.got.plt) *(.igot.plt) *(.got) *(.igot)
        . = ALIGN(4);
    } > FLASH
    PROVIDE(_got_start = ADDR(.got));
    PROVIDE(_got_end = ADDR(.got) + SIZEOF(.got));

    /* reserved ram  */
    .compal.reservedram 0x800000 (NOLOAD) : {
        . = 0xff;
    } > IRAM

    /* initialized data */
    .data : AT (LOADADDR(.got) + SIZEOF(.got)) {
        . = ALIGN(4);
    _sdata = ABSOLUTE(.);
        *(.data)
    _edata = ABSOLUTE(.);
        . = ALIGN(4);
    } > IRAM
    PROVIDE(__data_start = LOADADDR(.data));
    PROVIDE(__data_end = LOADADDR(.data) + SIZEOF(.data));
    PROVIDE(_data_start = ADDR(.data));
    PROVIDE(_data_end = ADDR(.data) + SIZEOF(.data));

    /* ram code */
    .ramtext : AT (LOADADDR(.data) + SIZEOF(.data)) {
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
    } > IRAM
    PROVIDE(__ramtext_start = LOADADDR(.ramtext));
    PROVIDE(__ramtext_end = LOADADDR(.ramtext) + SIZEOF(.ramtext));
    PROVIDE(_ramtext_start = ADDR(.ramtext));
    PROVIDE(_ramtext_end = ADDR(.ramtext) + SIZEOF(.ramtext));

    /* uninitialized data */
    .bss (NOLOAD) : {
        . = ALIGN(4);
    _sbss = ABSOLUTE(.);
        *(.bss)
    _ebss = ABSOLUTE(.);
        . = ALIGN(4);
    } > IRAM
    PROVIDE(__bss_start = ADDR(.bss));
    PROVIDE(__bss_end = ADDR(.bss) + SIZEOF(.bss));
    PROVIDE(_bss_start = __bss_start);
    PROVIDE(_bss_end = __bss_end);

    /* end of image */
    . = ALIGN(4);
    _end = .;
    PROVIDE(end = .);
}
