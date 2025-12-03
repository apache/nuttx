#include <nuttx/config.h>

OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH("tricore")
ENTRY(__start)
EXTERN(__start)

__HEAP_SIZE = (512 * 1024);
CONFIG_TRICORE_CSA_COUNT = 512;
CONFIG_TRICORE_INITIAL_STACK_SIZE = 512;
CONFIG_TRICORE_INITIAL_STACK_OFFSET = 0xEB00;

MEMORY
{
#if CPU == 0
	dsram (w!xp): ORIGIN = 0x70000000, LENGTH = 240K
	psram (w!xp): ORIGIN = 0x70100000, LENGTH = 64K
	lmu (wr!xp): ORIGIN = 0x90400000, LENGTH = 512K
	dlmu (wr!xp): ORIGIN = 0x90000000, LENGTH = 512K
	pflash (rx!p): ORIGIN = 0x80000000, LENGTH = 4M
#endif
	pflash0 (rx!p): ORIGIN = 0xA0000000, LENGTH = 4M

#if CPU == 1
	dsram (w!xp): ORIGIN = 0x60000000, LENGTH = 240K
	psram (w!xp): ORIGIN = 0x60100000, LENGTH = 64K
	lmu (wr!xp): ORIGIN = 0x90480000, LENGTH = 512K
	dlmu (wr!xp): ORIGIN = 0x90080000, LENGTH = 512K
	pflash (rx!p): ORIGIN = 0x80400000, LENGTH = 4M
#endif
	pflash1 (rx!p): ORIGIN = 0xA0400000, LENGTH = 4M

#if CPU == 2
	dsram (w!xp): ORIGIN = 0x50000000, LENGTH = 240K
	psram (w!xp): ORIGIN = 0x50100000, LENGTH = 64K
	lmu (wr!xp): ORIGIN = 0x90500000, LENGTH = 512K
	dlmu (wr!xp): ORIGIN = 0x90100000, LENGTH = 512K
	pflash (rx!p): ORIGIN = 0x80800000, LENGTH = 2M
#endif
	pflash2 (rx!p): ORIGIN = 0xA0800000, LENGTH = 2M

#if CPU == 3
	dsram (w!xp): ORIGIN = 0x40000000, LENGTH = 240K
	psram (w!xp): ORIGIN = 0x40100000, LENGTH = 64K
	lmu (wr!xp): ORIGIN = 0x90580000, LENGTH = 512K
	dlmu (wr!xp): ORIGIN = 0x90180000, LENGTH = 512K
	pflash (rx!p): ORIGIN = 0x80A00000, LENGTH = 4M
#endif
	pflash3 (rx!p): ORIGIN = 0xA0A00000, LENGTH = 4M

#if CPU == 4
	dsram (w!xp): ORIGIN = 0x30000000, LENGTH = 240K
	psram (w!xp): ORIGIN = 0x30100000, LENGTH = 64K
	lmu (wr!xp): ORIGIN = 0x90600000, LENGTH = 512K
	dlmu (wr!xp): ORIGIN = 0x90200000, LENGTH = 512K
	pflash (rx!p): ORIGIN = 0x80E00000, LENGTH = 4M
#endif
	pflash4 (rx!p): ORIGIN = 0x80E00000, LENGTH = 4M

#if CPU == 5
	dsram (w!xp): ORIGIN = 0x20000000, LENGTH = 240K
	psram (w!xp): ORIGIN = 0x20100000, LENGTH = 64K
	lmu (wr!xp): ORIGIN = 0x90680000, LENGTH = 512K
	dlmu (wr!xp): ORIGIN = 0x90280000, LENGTH = 512K
	pflash (rx!p): ORIGIN = 0x81200000, LENGTH = 2M
#endif
	pflash5 (rx!p): ORIGIN = 0x81200000, LENGTH = 2M

}

REGION_ALIAS("CODE_MEM", pflash)
REGION_ALIAS("DATA_MEM", dsram)
REGION_ALIAS("BSS_MEM", dsram)
REGION_ALIAS("CSA_MEM", dsram)
REGION_ALIAS("HEAP_MEM", dlmu)
REGION_ALIAS("PSRAM_MEM", psram)

pflash0_base = ORIGIN(pflash0);
pflash1_base = ORIGIN(pflash1);
pflash2_base = ORIGIN(pflash2);
pflash3_base = ORIGIN(pflash3);
pflash4_base = ORIGIN(pflash4);
pflash5_base = ORIGIN(pflash5);

SECTIONS
{
	.start :
	{
		KEEP(*(.start))
		. = ALIGN(4);
	} > CODE_MEM

	.traptab :
	{
		KEEP(*(.traptab))
		. = ALIGN(256);
	} > CODE_MEM

	.ivtab :
	{
		KEEP(*(.ivtab))
		. = ALIGN(4);
	} > CODE_MEM

	.text :
	{
		KEEP(*(.text))
		. = ALIGN(2);
	} > CODE_MEM

	.data : ALIGN(4)
	{
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		. = ALIGN(4);
		_edata = ABSOLUTE(.);
	} > DATA_MEM AT> CODE_MEM
	_data_flash = LOADADDR(.data);

	.flash_banks :
	{
		_flash_banks = . ;
		LONG(pflash0_base);
		LONG(pflash1_base);
		LONG(pflash2_base);
		LONG(pflash3_base);
		LONG(pflash4_base);
		LONG(pflash5_base);
	} > CODE_MEM

	.psram_base :
	{
		. += CONFIG_TRICORE_INITIAL_STACK_OFFSET;
		 __PSRAM_USTACK = . ;
		. += CONFIG_TRICORE_INITIAL_STACK_SIZE;
		 __PSRAM_BOOT_CSA = . ;
	} > PSRAM_MEM

	.bss :
	{
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > BSS_MEM

	.ustack . : ALIGN(8) FLAGS(aw)
	{
		. +=  CONFIG_IDLETHREAD_STACKSIZE ;
		__USTACK =  . ;
	} > BSS_MEM

#if defined(CONFIG_ARCH_INTERRUPTSTACK)
	.istack . : ALIGN(8) FLAGS(aw)
	{
		. += CONFIG_ARCH_INTERRUPTSTACK;
		__istack =  . ;
	} > BSS_MEM
#endif
	.csa : ALIGN(64) FLAGS(aw)
	{
		__CSA_BEGIN = . ;
		. +=  CONFIG_TRICORE_CSA_COUNT * 64 ;
		__CSA_END = .;
	} > CSA_MEM
	__CSA_SIZE = __CSA_END - __CSA_BEGIN;

	.heap : ALIGN(8) FLAGS(aw)
	{
		_sheap =  . ;
		. +=  __HEAP_SIZE;
		_eheap =  . ;
	} > HEAP_MEM
	_heap_size = _sheap - _eheap;

#if 0
	/DISCARD/ : { *(.comment) }
	/DISCARD/ : { *(.callinfo) }
#endif
}
