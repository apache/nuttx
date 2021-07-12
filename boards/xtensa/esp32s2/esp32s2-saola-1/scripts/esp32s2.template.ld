/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-saola-1/scripts/esp32s2.template.ld
 * ESP32S2 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * esp32s2.common.ld contains output sections to link compiler output
 * into these memory blocks.
 *
 * NOTE: That this is not the actual linker script but rather a "template"
 * for the elf32_out.ld script.  This template script is passed through
 * the C preprocessor to include selected configuration options.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESP32S2_INSTRUCTION_CACHE_8KB
#  define CONFIG_ESP32S2_INSTRUCTION_CACHE_SIZE 0x2000
#else
#  define CONFIG_ESP32S2_INSTRUCTION_CACHE_SIZE 0x4000
#endif

#ifdef CONFIG_ESP32S2_DATA_CACHE_0KB
#  define CONFIG_ESP32S2_DATA_CACHE_SIZE 0
#elif defined CONFIG_ESP32S2_DATA_CACHE_8KB
#  define CONFIG_ESP32S2_DATA_CACHE_SIZE 0x2000
#else
#  define CONFIG_ESP32S2_DATA_CACHE_SIZE 0x4000
#endif

#define RAM_IRAM_START    0x40020000
#define RAM_DRAM_START    0x3ffb0000

#define DATA_RAM_END      0x3ffe0000  /* 2nd stage bootloader iram_loader_seg
                                       * starts at SRAM block 14 (reclaimed
                                       * after app boots)
                                       */

#define IRAM_ORG    (RAM_IRAM_START + CONFIG_ESP32S2_INSTRUCTION_CACHE_SIZE \
                                    + CONFIG_ESP32S2_DATA_CACHE_SIZE)

#define DRAM_ORG    (RAM_DRAM_START + CONFIG_ESP32S2_INSTRUCTION_CACHE_SIZE \
                                    + CONFIG_ESP32S2_DATA_CACHE_SIZE)

#define I_D_RAM_SIZE   DATA_RAM_END - DRAM_ORG

MEMORY
{
  /* All these values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and eg allow bytewise access.
   */

  /* IRAM for CPU */

  iram0_0_seg (RX) :                 org = IRAM_ORG, len = I_D_RAM_SIZE

  /* Even though the segment name is iram, it is actually mapped to flash */

  iram0_2_seg (RX) :                 org = 0x40080020, len = 0x780000-0x20

  /* (0x20 offset above is a convenience for the app binary image generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).)
   */

  /* Shared data RAM, excluding memory reserved for bootloader and ROM
   * bss/data/stack.
   */

  dram0_0_seg (RW) :                 org = DRAM_ORG, len = I_D_RAM_SIZE

  /* Flash mapped constant data */

  drom0_0_seg (R) :                  org = 0x3f000020, len = 0x3f0000-0x20

  /* RTC fast memory (executable). Persists over deep sleep. */

  rtc_iram_seg(RWX) :                org = 0x40070000, len = 0x2000

  /* RTC slow memory (data accessible). Persists over deep sleep.
   * Start of RTC slow memory is reserved for ULP co-processor code + data,
   * if enabled.
   */

  rtc_slow_seg(RW)  :    org = 0x50000000 + CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM,
                         len = 0x2000 - CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM
}

