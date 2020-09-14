/****************************************************************************
 * boards/xtensa/esp32/esp32-core/scripts/esp32.template.ld
 * ESP32 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * esp32.common.ld contains output sections to link compiler output
 * into these memory blocks.
 *
 * NOTE: That this is not the actual linker script but rather a "template"
 * for the elf32_out.ld script.  This template script is passed through
 * the C preprocessor to include selected configuration options.
 *
 ****************************************************************************/

#include <nuttx/config.h>

MEMORY
{
  /* All these values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and eg allow bytewise access.
   */

  /* IRAM for PRO cpu. Not sure if happy with this, this is MMU area... */

  iram0_0_seg (RX) :     org = 0x40080000, len = 0x20000

  /* Even though the segment name is iram, it is actually mapped to flash.
   *
   * NOTE: (0x20 offset above is a convenience for the app binary image
   * generation.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).)
   */

  iram0_2_seg (RX) :     org = 0x400d0020, len = 0x330000 - 0x20

  /* Shared data RAM, excluding memory reserved for ROM bss/data/stack.
   * Enabling Bluetooth & Trace Memory features in menuconfig will decrease
   * the amount of RAM available.
   */

  dram0_0_seg (RW) :     org = 0x3ffb0000 + CONFIG_ESP32_BT_RESERVE_DRAM,
                         len = 0x50000 - CONFIG_ESP32_TRACEMEM_RESERVE_DRAM - CONFIG_ESP32_BT_RESERVE_DRAM

  /* Flash mapped constant data */

  drom0_0_seg (R) :      org = 0x3f400020, len = 0x400000 - 0x20

  /* RTC fast memory (executable). Persists over deep sleep. */

  rtc_iram_seg(RWX) :    org = 0x400c0000, len = 0x2000

  /* RTC slow memory (data accessible). Persists over deep sleep.
   * Start of RTC slow memory is reserved for ULP co-processor code + data,
   * if enabled.
   */

  rtc_slow_seg(RW)  :    org = 0x50000000 + CONFIG_ESP32_ULP_COPROC_RESERVE_MEM,
                         len = 0x1000 - CONFIG_ESP32_ULP_COPROC_RESERVE_MEM
}

/* Heap ends at top of dram0_0_seg */

_eheap = 0x40000000 - CONFIG_ESP32_TRACEMEM_RESERVE_DRAM;

/* Module text area ends at top of dram0_0_seg */

_emodtext = 0x400a0000;
