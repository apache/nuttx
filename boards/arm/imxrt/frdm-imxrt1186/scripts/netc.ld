/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/scripts/netc.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

MEMORY
{
  itcm (rwx) : ORIGIN = 0x0ffe0000, LENGTH = 128K
  ocram (rw) : ORIGIN = 0x20480000, LENGTH = 512K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
  .vectors :
  {
    _stext = ABSOLUTE(.);
    KEEP(*(.vectors))
  } > itcm

  .reset :
  {
    *(.text.start)
    *(.text.__start)
  } > itcm

  .text :
  {
    *(.text .text.*)
    *(.fixup)
    *(.gnu.warning)
    *(.gnu.linkonce.t.*)
    *(.glue_7)
    *(.glue_7t)
    *(.got)
    *(.gcc_except_table)
  } > itcm

  .rodata :
  {
    *(.rodata .rodata.*)
    *(.gnu.linkonce.r.*)
    _etext = ABSOLUTE(.);
  } > ocram

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(.init_array .init_array.*))
    _einit = ABSOLUTE(.);
  } > ocram

  .ARM.extab : { *(.ARM.extab*) } > ocram
  .ARM.exidx :
  {
    __exidx_start = ABSOLUTE(.);
    *(.ARM.exidx*)
    __exidx_end = ABSOLUTE(.);
  } > ocram

  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > ocram

  _eronly = LOADADDR(.data);

  .ramfunc ALIGN(4) :
  {
    _sramfuncs = ABSOLUTE(.);
    *(.ramfunc .ramfunc.*)
    _eramfuncs = ABSOLUTE(.);
  } > ocram
  _framfuncs = LOADADDR(.ramfunc);

  .bss :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
  } > ocram

  .clockstatus (NOLOAD) :
  {
    *(.clockstatus)
    . = ALIGN(4);
    _eclockstatus = ABSOLUTE(.);
  } > ocram

  /DISCARD/ : { *(.note .note.*) }

  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .comment 0 : { *(.comment) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_info 0 : { *(.debug_info) }
  .debug_line 0 : { *(.debug_line) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  .debug_aranges 0 : { *(.debug_aranges) }
}
