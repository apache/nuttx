/* Specify the memory areas */
MEMORY
{
ITCMRAM  (xrw) : ORIGIN = 0x00000000, LENGTH = 896K
DTCMRAM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
AXI_RAM  (xrw) : ORIGIN = 0x24000000, LENGTH = 128K
AHB_RAM  (xrw) : ORIGIN = 0x30000000, LENGTH = 352K
FLASH    (rx ) : ORIGIN = 0x15000000, LENGTH = 896K /* 896K for code, 1024K for mtd */
}

/* Entry Point */
OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

/* Define output sections */
SECTIONS
{
    _sivectors = LOADADDR(.vectors);

    .vectors :
    {
        _svectors = ABSOLUTE(.);
        *(.vectors)
        _evectors = ABSOLUTE(.);
    } > ITCMRAM AT > FLASH

    .startup :
    {
        *(.rodata.g_idle_topstack)
        *(.text.start)
        *(.text.__start)
        *(.text.n32_tcmenable)
        *(.text.n32_tcm_init)
    } > FLASH

    _sitext = LOADADDR(.text);

    .text :
    {
        _stext = ABSOLUTE(.);
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > ITCMRAM AT > FLASH

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > ITCMRAM AT > FLASH

    .ARM.extab :
    {
        __extab_start = ABSOLUTE(.);
        *(.ARM.extab*)
        __extab_end = ABSOLUTE(.);
    } > ITCMRAM AT > FLASH

    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > ITCMRAM AT > FLASH

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > DTCMRAM AT > FLASH

    _eronly = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        _sbss_rel = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss_rel = ABSOLUTE(.);
    } > DTCMRAM

    /* Emit the the D3 power domain section for locating BDMA data
     *
     * Static data with locate_data(".axi_ram") will be located
     * at start of AXI_RAM; the rest of AXI_RAM will be added to the heap.
     */
    .ahb_ram_reserve (NOLOAD) :
    {
        *(.ahb_ram)
        . = ALIGN(4);
        _ahbram_heap_start = ABSOLUTE(.);
    } > AHB_RAM

    .axi_ram_reserve (NOLOAD) :
    {
        _sbss = ABSOLUTE(.);
        *(.axi_ram)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
        _axiram_heap_start = ABSOLUTE(.);
    } > AXI_RAM

    /* Stabs debugging sections. */

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
