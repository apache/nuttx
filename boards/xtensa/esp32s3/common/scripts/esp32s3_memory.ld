/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/esp32s3_memory.ld
 * ESP32-S3 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * esp32s3_sections.ld contains output sections to link compiler output
 * into these memory blocks.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#define SRAM_IRAM_START     0x40370000
#define SRAM_DIRAM_I_START  0x40378000
#define SRAM_IRAM_END       0x403ba000
#define I_D_SRAM_OFFSET     (SRAM_DIRAM_I_START - SRAM_DRAM_START)

#define SRAM_DRAM_START     0x3fc88000

/* 2nd stage bootloader iram_loader_seg start address */

#define SRAM_DRAM_END       (SRAM_IRAM_END - I_D_SRAM_OFFSET)
#define I_D_SRAM_SIZE       (SRAM_DRAM_END - SRAM_DRAM_START)

#define ICACHE_SIZE         0x8000

#define SRAM_IRAM_ORG       (SRAM_IRAM_START + CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE)
#define SRAM_IRAM_SIZE      (I_D_SRAM_SIZE + ICACHE_SIZE - CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE)

#define DCACHE_SIZE         0x10000
#define SRAM_DRAM_ORG       (SRAM_DRAM_START)

#ifdef CONFIG_ESP32S3_FLASH_4M
#  define FLASH_SIZE        0x400000
#elif defined (CONFIG_ESP32S3_FLASH_8M)
#  define FLASH_SIZE        0x800000
#elif defined (CONFIG_ESP32S3_FLASH_16M)
#  define FLASH_SIZE        0x1000000
#endif

MEMORY
{
  /* Below values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and eg allow bytewise access.
   */

  /* IRAM for CPU */

  iram0_0_seg (RX) :                 org = SRAM_IRAM_ORG, len = SRAM_IRAM_SIZE

  /* Flash mapped instruction data. */

  /* The 0x20 offset is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  irom0_0_seg (RX) :                 org = 0x42000020, len = FLASH_SIZE - 0x20

  /* Shared data RAM, excluding memory reserved for bootloader and ROM
   * bss/data/stack.
   */

  dram0_0_seg (RW) :                 org = SRAM_DRAM_ORG, len = I_D_SRAM_SIZE

  /* Flash mapped constant data */

  /* The 0x20 offset is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  drom0_0_seg (R) :                  org = 0x3c000020, len = FLASH_SIZE - 0x20

  /* RTC fast memory (executable). Persists over deep sleep. */

  rtc_iram_seg(RWX) :                org = 0x600fe000, len = 0x2000

  /* RTC fast memory (same block as above), viewed from data bus */

  rtc_data_seg(RW)  :                org = 0x600fe000, len = 0x2000

  /* RTC slow memory (data accessible). Persists over deep sleep.
   * Start of RTC slow memory is reserved for ULP co-processor code + data,
   * if enabled.
   */

  rtc_slow_seg(RW)  :    org = 0x50000000 + CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM,
                         len = 0x2000 - CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM
}

#ifdef CONFIG_ESP32S3_RUN_IRAM
  REGION_ALIAS("default_rodata_seg", dram0_0_seg);
  REGION_ALIAS("default_code_seg", iram0_0_seg);
#else
  REGION_ALIAS("default_rodata_seg", drom0_0_seg);
  REGION_ALIAS("default_code_seg", irom0_0_seg);
#endif /* CONFIG_ESP32S3_RUN_IRAM */

