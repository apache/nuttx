#include <nuttx/config.h>

#include "tc1v1_8.lsl"

#define HEAP_SIZE               (512 * 1024)
#define CSA_COUNT               512
#define INITIAL_STACK_SIZE      512
#define INITIAL_STACK_OFFSET    0xEB00

#if CPU == 0
#  define DSRAM_BASE            0x70000000
#  define PSRAM_BASE            0x70100000
#  define DLMU_BASE             0x90000000
#  define DLMU_SIZE             512k
#  define PFLASH_BASE           0x80000000
#  define PFLASH_SIZE           4M
#elif CPU == 1
#  define DSRAM_BASE            0x60000000
#  define PSRAM_BASE            0x60100000
#  define DLMU_BASE             0x90080000
#  define DLMU_SIZE             512k
#  define PFLASH_BASE           0x80400000
#  define PFLASH_SIZE           4M
#elif CPU == 2
#  define DSRAM_BASE            0x50000000
#  define PSRAM_BASE            0x50100000
#  define DLMU_BASE             0x90100000
#  define DLMU_SIZE             512k
#  define PFLASH_BASE           0x80800000
#  define PFLASH_SIZE           2M
#elif CPU == 3
#  define DSRAM_BASE            0x40000000
#  define PSRAM_BASE            0x40100000
#  define DLMU_BASE             0x90180000
#  define DLMU_SIZE             512k
#  define PFLASH_BASE           0x80A00000
#  define PFLASH_SIZE           4M
#elif CPU == 4
#  define DSRAM_BASE            0x30000000
#  define PSRAM_BASE            0x30100000
#  define DLMU_BASE             0x90200000
#  define DLMU_SIZE             512k
#  define PFLASH_BASE           0x80E00000
#  define PFLASH_SIZE           4M
#elif CPU == 5
#  define DSRAM_BASE            0x20000000
#  define PSRAM_BASE            0x20100000
#  define DLMU_BASE             0x90280000
#  define DLMU_SIZE             512k
#  define PFLASH_BASE           0x81200000
#  define PFLASH_SIZE           2M
#endif

derivative nuttx_tc4dx
{
  core tc0
  {
    architecture = TC1V1.8_DPFPU;
  }

  memory dsram
  {
    mau = 8;
    type = ram;
    size = 240k;
    map (dest = bus:tc0:fpi_bus, dest_offset = DSRAM_BASE, size = 240k);
  }

  memory psram
  {
    mau = 8;
    type = ram;
    size = 64k;
    map (dest = bus:tc0:fpi_bus, dest_offset = PSRAM_BASE, size = 64k);
  }

  memory dlmu
  {
    mau = 8;
    type = ram;
    size = DLMU_SIZE;
    map (dest = bus:tc0:fpi_bus, dest_offset = DLMU_BASE, size = DLMU_SIZE);
  }

  memory pflash
  {
    mau = 8;
    type = rom;
    size = PFLASH_SIZE;
    map (dest = bus:tc0:fpi_bus, dest_offset = PFLASH_BASE, size = PFLASH_SIZE);
  }
}

processor mpe
{
  derivative = nuttx_tc4dx;
}

/* NuttX initializes .data and .bss itself, the copy table only has to
 * exist for the linker.
 */

section_setup mpe:tc0:linear
{
  copytable (align = 4, dest = linear);
}

section_layout mpe:tc0:linear
{
  group start (ordered, run_addr = PFLASH_BASE)
  {
    select ".start";
  }

  group traptab (ordered, align = 256, run_addr = mem:mpe:pflash)
  {
    select ".traptab";
  }

  group ivtab (ordered, align = 32, run_addr = mem:mpe:pflash)
  {
    select ".ivtab";
  }

  group text (run_addr = mem:mpe:pflash)
  {
    select ".text*";
    select ".rodata*";
  }

  struct "flash_banks"
  {
    0xA0000000 : 4;
    0xA0400000 : 4;
    0xA0800000 : 4;
    0xA0A00000 : 4;
    0x80E00000 : 4;
    0x81200000 : 4;
  }

  group data (ordered, contiguous, align = 4, attributes = rwu, run_addr = mem:mpe:dsram)
  {
    select ".data*";
  }

  /* The startup code copies .data as one block, so the ROM copies are
   * selected by name to get the same order as in RAM, a group selector
   * sorts them by alignment.
   */

  group data_rom (ordered, contiguous, align = 4, attributes = ru, load_addr)
  {
    select ".data*";
  }

  group bss (ordered, contiguous, align = 4, attributes = rwu, run_addr = mem:mpe:dsram)
  {
    select ".bss*";
  }

  group psram_area (ordered, run_addr = PSRAM_BASE)
  {
    reserved "psram" (size = 64k);
  }

  group ustack_area (align = 8, run_addr = mem:mpe:dsram)
  {
    reserved "ustack" (size = CONFIG_IDLETHREAD_STACKSIZE);
  }

#ifdef CONFIG_ARCH_INTERRUPTSTACK
  group istack_area (align = 8, run_addr = mem:mpe:dsram)
  {
    reserved "istack" (size = CONFIG_ARCH_INTERRUPTSTACK);
  }

  "__istack" = "_lc_ue_istack";
#endif

  group csa_area (align = 64, run_addr = mem:mpe:dsram)
  {
    reserved "csa" (size = CSA_COUNT * 64);
  }

  group heap_area (align = 8, run_addr = mem:mpe:dlmu)
  {
    reserved "heap" (size = HEAP_SIZE);
  }

  "_sdata" = "_lc_gb_data";
  "_edata" = "_lc_ge_data";
  "_data_flash" = "_lc_gb_data_rom";
  "_sbss" = "_lc_gb_bss";
  "_ebss" = "_lc_ge_bss";
  "_flash_banks" = "_lc_ub_flash_banks";
  "__PSRAM_USTACK" = PSRAM_BASE + INITIAL_STACK_OFFSET;
  "__PSRAM_BOOT_CSA" = PSRAM_BASE + INITIAL_STACK_OFFSET + INITIAL_STACK_SIZE;
  "__USTACK" = "_lc_ue_ustack";
  "__CSA_BEGIN" = "_lc_ub_csa";
  "__CSA_END" = "_lc_ue_csa";
  "_sheap" = "_lc_ub_heap";
  "_eheap" = "_lc_ue_heap";

  /* ctc refers to one of these per printf/scanf call to pick a formatter
   * from its own C library, the NuttX one needs none of them.
   */

  "__printf_noarg" := 0;
  "__printf_simple" := 0;
  "__printf_int" := 0;
  "__printf_long" := 0;
  "__printf_llong" := 0;
  "__printf_float" := 0;
  "__wprintf_noarg" := 0;
  "__wprintf_simple" := 0;
  "__wprintf_int" := 0;
  "__wprintf_long" := 0;
  "__wprintf_llong" := 0;
  "__wprintf_float" := 0;
  "__scanf_int" := 0;
  "__scanf_long" := 0;
  "__scanf_llong" := 0;
  "__scanf_float" := 0;
  "__wscanf_int" := 0;
  "__wscanf_long" := 0;
  "__wscanf_llong" := 0;
  "__wscanf_float" := 0;
}
