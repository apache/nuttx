/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/scripts/xip.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

MEMORY
{
  /* Secure FlexSPI2 AHB window — Boot ROM container load/entry must use
   * 0x14xxxxxx (Zephyr/NXP FRDM-IMXRT1186). Non-secure alias 0x04xxxxxx
   * maps the same flash but ROM handoff stays in Boot ROM if container
   * points there.
   */
  flash (rx)  : ORIGIN = 0x14000000, LENGTH = 16M
  ocram (rwx) : ORIGIN = 0x20480000, LENGTH = 512K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(g_flash_config)
EXTERN(g_boot_container)
EXTERN(g_xmcd_data)
ENTRY(_stext)

SECTIONS
{
  /* RT1180 Boot ROM layout in the FlexSPI2 secure AHB window:
   *
   *   0x14000400  FlexSPI configuration block
   *   0x14000800  XMCD (HyperRAM option words for Boot ROM; match Zephyr)
   *   0x14001000  AHAB-style container
   *   0x1400b000  Cortex-M33 XIP image
   */

  .boot_hdr :
  {
    FILL(0xff)
    . = 0x400;
    KEEP(*(.boot_hdr.conf))
    . = 0x800;
    KEEP(*(.boot_hdr.xmcd_data))
    . = 0x1000;
    KEEP(*(.boot_hdr.container))
    . = 0xb000;
  } > flash

  __flash_image_start = .;

  .text : ALIGN(4)
  {
    _stext = ABSOLUTE(.);
    KEEP(*(.vectors))
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.fixup)
    *(.gnu.warning)
    *(.gnu.linkonce.t.*)
    *(.gnu.linkonce.r.*)
    *(.glue_7)
    *(.glue_7t)
    *(.got)
    *(.gcc_except_table)
    _etext = ABSOLUTE(.);
  } > flash

  .init_section : ALIGN(4)
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(.init_array .init_array.*))
    _einit = ABSOLUTE(.);
  } > flash

  .ARM.extab : ALIGN(4)
  {
    *(.ARM.extab*)
  } > flash

  .ARM.exidx : ALIGN(4)
  {
    __exidx_start = ABSOLUTE(.);
    *(.ARM.exidx*)
    __exidx_end = ABSOLUTE(.);
  } > flash

  .data : ALIGN(4)
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > ocram AT > flash

  _eronly = LOADADDR(.data);

  .ramfunc : ALIGN(4)
  {
    _sramfuncs = ABSOLUTE(.);
    *(.ramfunc .ramfunc.*)
    _eramfuncs = ABSOLUTE(.);
  } > ocram AT > flash

  _framfuncs = LOADADDR(.ramfunc);

  .bss : ALIGN(4)
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

  __flash_image_end = LOADADDR(.ramfunc) + SIZEOF(.ramfunc);
  __container_image_offset =
    __flash_image_start - (ORIGIN(flash) + 0x1000);
  __container_image_size = __flash_image_end - __flash_image_start;
  __container_load_address = __flash_image_start;
  __container_entry = __start;

  ASSERT(__container_image_offset == 0xa000,
         "RT1180 container payload must start at offset 0xa000")
  ASSERT(__flash_image_end <= ORIGIN(flash) + LENGTH(flash),
         "FRDM-IMXRT1186 XIP image exceeds FlexSPI2 flash")

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
