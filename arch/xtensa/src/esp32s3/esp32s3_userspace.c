/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_userspace.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <stdlib.h>

#include <nuttx/userspace.h>

#include <arch/board/board_memorymap.h>

#include "chip.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp32s3_irq.h"
#include "esp32s3_userspace.h"
#include "hardware/esp32s3_apb_ctrl.h"
#include "hardware/esp32s3_cache_memory.h"
#include "hardware/esp32s3_extmem.h"
#include "hardware/esp32s3_rom_layout.h"
#include "hardware/esp32s3_sensitive.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_wcl_core.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USER_IMAGE_OFFSET   CONFIG_ESP32S3_USER_IMAGE_OFFSET

#define MMU_BLOCK0_VADDR    SOC_DROM_LOW
#define MMU_SIZE            0x3f0000
#define MMU_BLOCK63_VADDR   (MMU_BLOCK0_VADDR + MMU_SIZE)

/* Cache MMU address mask (MMU tables ignore bits which are zero) */

#define MMU_FLASH_MASK      (~(MMU_PAGE_SIZE - 1))

/* Helper just for shortening */

#define VALUE_TO_PMS_FIELD(v, f)  VALUE_TO_FIELD(v, SENSITIVE_CORE_X_ ## f)

/* Total addressable space is 1GB for the External Memories */

#define EXTMEM_MAX_LENGTH   0x40000000

/* Maximum number of supported entry addresses */

#define WCL_ENTRY_MAX       13

/* Last value of the agreed sequence to be written to the address configured
 * in WCL_CORE_0_MESSAGE_ADDR register.
 */

#define WCL_SEQ_LAST_VAL    6

#define I_D_SRAM_OFFSET           (SOC_DIRAM_IRAM_LOW - SOC_DIRAM_DRAM_LOW)
#define MAP_IRAM_TO_DRAM(addr)    ((addr) - I_D_SRAM_OFFSET)

/* Categories bits for split line configuration */

#define PMS_SRAM_CATEGORY_BELOW   0x0
#define PMS_SRAM_CATEGORY_EQUAL   0x2
#define PMS_SRAM_CATEGORY_ABOVE   0x3

/* Offsets for helping setting values to register fields */

#define ICACHE_PMS_W0_BASE        12
#define ICACHE_PMS_W1_BASE        12
#define ICACHE_PMS_S              3
#define ICACHE_PMS_V              7

#define IRAM_PMS_W0_BASE          0
#define IRAM_PMS_W1_BASE          0
#define IRAM_PMS_S                3
#define IRAM_PMS_V                7

#define DRAM_PMS_W0_BASE          0
#define DRAM_PMS_W1_BASE          12
#define DRAM_PMS_S                2
#define DRAM_PMS_V                3

#define FLASH_CACHE_S             3
#define FLASH_CACHE_V             7

#define PIF_PMS_MAX_REG_ENTRY     16
#define PIF_PMS_V                 3

#ifndef ALIGN_UP
#  define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
#endif

#ifndef ALIGN_DOWN
#  define ALIGN_DOWN(num, align)  ((num) & ~((align) - 1))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct user_image_load_header_s
{
  uintptr_t drom_vma;      /* Destination address (VMA) for DROM region */
  uintptr_t drom_lma;      /* Flash offset (LMA) for start of DROM region */
  uintptr_t drom_size;     /* Size of DROM region */
  uintptr_t iram_vma;      /* Destination address (VMA) for IRAM region */
  uintptr_t iram_lma;      /* Flash offset (LMA) for start of IRAM region */
  uintptr_t iram_size;     /* Size of IRAM region */
  uintptr_t irom_vma;      /* Destination address (VMA) for IROM region */
  uintptr_t irom_lma;      /* Flash offset (LMA) for start of IROM region */
  uintptr_t irom_size;     /* Size of IROM region */
};

enum pms_world_e
{
  PMS_WORLD_0 = 0,
  PMS_WORLD_1
};

enum pms_split_line_e
{
  PMS_SPLIT_LINE_0 = 0,
  PMS_SPLIT_LINE_1,
  PMS_SPLIT_LINE_2,
  PMS_SPLIT_LINE_3
};

enum pms_area_e
{
  PMS_AREA_0 = 0,    /* Area between BASE and split_line 0 */
  PMS_AREA_1,        /* Area between split_line 0 and split_line 1 */
  PMS_AREA_2,        /* Area between split_line 1 and END */
  PMS_AREA_3,
  PMS_AREA_INVALID
};

enum pms_flags_e
{
  PMS_ACCESS_NONE = 0,
  PMS_ACCESS_R = 1,
  PMS_ACCESS_W = 2,
  PMS_ACCESS_X = 4,
  PMS_ACCESS_ALL = PMS_ACCESS_X | PMS_ACCESS_W | PMS_ACCESS_R
};

/* There are 55 peripherals, each having 2 bits for permission configuration.
 * These are spread across 4 registers, each register having maximum of 16
 * peripheral entries.
 *
 * Enum defined as per the bit field position of the peripheral in the
 * register:
 *    FIELD = 30 - 2 * (ENUM % 16)
 */

enum pms_peripheral_e
{
  PMS_UART1 = 0,
  PMS_I2S0,
  PMS_I2C,
  PMS_MISC,
  PMS_HINF = 5,
  PMS_IO_MUX = 7,
  PMS_RTC,
  PMS_FE = 10,
  PMS_FE2 = 11,
  PMS_GPIO,
  PMS_G0SPI_0,
  PMS_G0SPI_1,
  PMS_UART,
  PMS_SYSTIMER,
  PMS_TIMERGROUP1,
  PMS_TIMERGROUP,
  PMS_PWM0,
  PMS_BB,
  PMS_BACKUP = 22,
  PMS_LEDC,
  PMS_SLC,
  PMS_PCNT,
  PMS_RMT,
  PMS_SLCHOST,
  PMS_UHCI0,
  PMS_I2C_EXT0,
  PMS_BT = 31,
  PMS_PWR = 33,
  PMS_WIFIMAC,
  PMS_RWBT = 36,
  PMS_UART2 = 39,
  PMS_I2S1,
  PMS_PWM1,
  PMS_CAN,
  PMS_SDIO_HOST,
  PMS_I2C_EXT1,
  PMS_APB_CTRL,
  PMS_SPI_3,
  PMS_SPI_2,
  PMS_WORLD_CONTROLLER,
  PMS_DIO,
  PMS_AD,
  PMS_CACHE_CONFIG,
  PMS_DMA_COPY,
  PMS_INTERRUPT,
  PMS_SENSITIVE,
  PMS_SYSTEM,
  PMS_USB,
  PMS_BT_PWR,
  PMS_LCD_CAM,
  PMS_APB_ADC,
  PMS_CRYPTO_DMA,
  PMS_CRYPTO_PERI,
  PMS_USB_WRAP,
  PMS_USB_DEVICE,
  PMS_MAX
};

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_dcache(uint32_t val);
extern void cache_invalidate_dcache_all(void);
extern void cache_invalidate_icache_all(void);
extern void cache_writeback_all(void);
extern int cache_dbus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
extern int cache_ibus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct user_image_load_header_s g_header;

static const intptr_t g_sram_rg3_level_hlimits[] =
{
  0x4037ffff, /* Block 2 (32KB) */
  0x4038ffff, /* Block 3 (64KB) */
  0x4039ffff, /* Block 4 (64KB) */
  0x403affff, /* Block 5 (64KB) */
  0x403bffff, /* Block 6 (64KB) */
  0x403cffff, /* Block 7 (64KB) */
  0x403dffff  /* Block 8 (64KB) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dcache_suspend
 *
 * Description:
 *   Helper function for suspending the data cache access for the CPU.
 *
 * Input Parameters:
 *   needs_wb      - Flag indicating whether the CPU should perform the
 *                   write-back of the data cache contents prior to
 *                   invalidation.
 *
 * Returned Value:
 *   Current cache state.
 *
 ****************************************************************************/

static inline uint32_t dcache_suspend(bool needs_wb)
{
  uint32_t dcache_state = cache_suspend_dcache();

  if (needs_wb)
    {
      cache_writeback_all();
    }

  cache_invalidate_dcache_all();

  return dcache_state;
}

/****************************************************************************
 * Name: dcache_resume
 *
 * Description:
 *   Helper function for resuming the data cache access for the CPU.
 *
 * Input Parameters:
 *   cache_state   - Previously saved data cache state to be restored.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void dcache_resume(uint32_t cache_state)
{
  uint32_t regval;

  regval = getreg32(EXTMEM_DCACHE_CTRL1_REG);
  regval &= ~EXTMEM_DCACHE_SHUT_CORE0_BUS;
#ifdef CONFIG_SMP
  regval &= ~EXTMEM_DCACHE_SHUT_CORE1_BUS;
#endif
  putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);

  cache_resume_dcache(cache_state);
}

/****************************************************************************
 * Name: calc_mmu_pages
 *
 * Description:
 *   Calculate the required number of MMU pages for mapping a given region
 *   from External Flash into Internal RAM.
 *
 * Input Parameters:
 *   size          - Length of the region to map
 *   vaddr         - Starting External Flash offset to map to Internal RAM
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline uint32_t calc_mmu_pages(uint32_t size, uint32_t vaddr)
{
  return (size + (vaddr - (vaddr & MMU_FLASH_MASK)) + MMU_PAGE_SIZE - 1) /
    MMU_PAGE_SIZE;
}

/****************************************************************************
 * Name: configure_flash_mmu
 *
 * Description:
 *   Configure the External Flash MMU and Cache for enabling access to code
 *   and read-only data of the userspace image.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static noinline_function IRAM_ATTR void configure_flash_mmu(void)
{
  uint32_t drom_lma_aligned;
  uint32_t drom_vma_aligned;
  uint32_t drom_page_count;
  uint32_t irom_lma_aligned;
  uint32_t irom_vma_aligned;
  uint32_t irom_page_count;

  size_t partition_offset = USER_IMAGE_OFFSET;
  uint32_t app_drom_lma = partition_offset + g_header.drom_lma;
  uint32_t app_drom_size = g_header.drom_size;
  uint32_t app_drom_vma = g_header.drom_vma;
  uint32_t app_irom_lma = partition_offset + g_header.irom_lma;
  uint32_t app_irom_size = g_header.irom_size;
  uint32_t app_irom_vma = g_header.irom_vma;

  uint32_t cache_state = dcache_suspend(false);

  drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
  drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
  drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
  ASSERT(cache_dbus_mmu_set(MMU_ACCESS_FLASH, drom_vma_aligned,
                            drom_lma_aligned, 64,
                            (int)drom_page_count, 0) == 0);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
  ASSERT(cache_ibus_mmu_set(MMU_ACCESS_FLASH, irom_vma_aligned,
                            irom_lma_aligned, 64,
                            (int)irom_page_count, 0) == 0);

  dcache_resume(cache_state);
}

/****************************************************************************
 * Name: map_flash
 *
 * Description:
 *   Map a region of the External Flash memory to Internal RAM.
 *
 * Input Parameters:
 *   src_addr      - Starting External Flash offset to map to Internal RAM
 *   size          - Length of the region to map
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static noinline_function IRAM_ATTR const void *map_flash(uint32_t src_addr,
                                                         uint32_t size)
{
  uint32_t src_addr_aligned;
  uint32_t page_count;

  uint32_t cache_state = dcache_suspend(false);

  src_addr_aligned = src_addr & MMU_FLASH_MASK;
  page_count = calc_mmu_pages(size, src_addr);

  ASSERT(cache_dbus_mmu_set(MMU_ACCESS_FLASH, MMU_BLOCK63_VADDR,
                            src_addr_aligned, 64, (int)page_count, 0) == 0);

  dcache_resume(cache_state);

  return (void *)(MMU_BLOCK63_VADDR + (src_addr - src_addr_aligned));
}

/****************************************************************************
 * Name: load_header
 *
 * Description:
 *   Load metadata information from image header to enable the correct
 *   configuration of the Flash MMU and Cache and initialization of the
 *   code and data located in Internal RAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void load_header(void)
{
  size_t length = sizeof(struct user_image_load_header_s);
  const uint8_t *data =
    (const uint8_t *)map_flash(USER_IMAGE_OFFSET, length);

  DEBUGASSERT(data != NULL);

  memcpy(&g_header, data, length);
}

/****************************************************************************
 * Name: initialize_dram
 *
 * Description:
 *   Initialize data sections of the userspace image.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void initialize_dram(void)
{
  uint8_t *dest;
  uint8_t *end;

  /* Clear all of userspace .bss */

  ASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
         USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint8_t *)USERSPACE->us_bssstart;
  end  = (uint8_t *)USERSPACE->us_bssend;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of userspace .data */

  ASSERT(USERSPACE->us_datasource != 0 && USERSPACE->us_datastart != 0 &&
         USERSPACE->us_dataend != 0 &&
         USERSPACE->us_datastart <= USERSPACE->us_dataend);

  size_t length = USERSPACE->us_dataend - USERSPACE->us_datastart;
  const uint8_t *src =
    (const uint8_t *)map_flash(USER_IMAGE_OFFSET + USERSPACE->us_datasource,
                               length);

  ASSERT(src != NULL);

  dest = (uint8_t *)USERSPACE->us_datastart;
  end  = (uint8_t *)USERSPACE->us_dataend;

  while (dest != end)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Name: initialize_iram
 *
 * Description:
 *   Initialize instruction sections of the userspace image.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void initialize_iram(void)
{
  uint32_t *dest;
  uint32_t *end;
  const uint32_t *src =
    (const uint32_t *)map_flash(USER_IMAGE_OFFSET + g_header.iram_lma,
                                g_header.iram_size);

  ASSERT(src != NULL);

  /* This routine will initialize the IRAM region located in Internal SRAM0,
   * which can be accessible using 32-bit loads and stores and if the source
   * and destiny addresses and length are 32-bit aligned.
   */

  dest = (uint32_t *)g_header.iram_vma;
  end  = (uint32_t *)g_header.iram_vma + g_header.iram_size;

  while (dest != end)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Name: wcl_set_vecbase
 *
 * Description:
 *   Override Vector Table base address via World Controller.
 *
 * Input Parameters:
 *   world         - World to which the vector table base address will apply
 *                   to.
 *   vecbase       - Vector table base address to set.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void wcl_set_vecbase(enum pms_world_e world, uintptr_t vecbase)
{
  switch (world)
    {
      case PMS_WORLD_0:
        {
          modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_1_REG,
                      SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD0_VALUE_M,
                      VALUE_TO_FIELD(vecbase >> 10,
                            SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD0_VALUE));
        }
        break;
      case PMS_WORLD_1:
        {
          modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_2_REG,
                      SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD1_VALUE_M,
                      VALUE_TO_FIELD(vecbase >> 10,
                            SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD1_VALUE));
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }

  modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_1_REG,
              SENSITIVE_CORE_0_VECBASE_OVERRIDE_SEL_M,
              VALUE_TO_FIELD(0x3, SENSITIVE_CORE_0_VECBASE_OVERRIDE_SEL));

  modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_0_REG,
              SENSITIVE_CORE_0_VECBASE_WORLD_MASK_M, 0);
}

/****************************************************************************
 * Name: wcl_set_world0_entry
 *
 * Description:
 *   Configure the World Controller to switch to World 0 whenever the CPU
 *   performs an instruction fetch from a given address.
 *
 * Input Parameters:
 *   entry         - Entry number. Up to 13 entry addresses are supported.
 *                   Entry 0 is reserved and must be skipped.
 *   addr          - Interrupt vector address that will trigger the change to
 *                   World 0.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void wcl_set_world0_entry(uint32_t entry, uintptr_t addr)
{
  ASSERT(entry > 0 && entry <= WCL_ENTRY_MAX);

  /* Configure registers required for cleaning the World Controller write
   * buffer upon World0 entry.
   *
   * Refer to ESP32-S3 Technical Reference Manual, section 16.4.3, for a
   * detailed description of the write buffer clearing process.
   */

  putreg32(SOC_RTC_DATA_LOW, WCL_CORE_0_MESSAGE_ADDR_REG);
  putreg32(WCL_SEQ_LAST_VAL, WCL_CORE_0_MESSAGE_MAX_REG);

  uint32_t reg = WCL_CORE_0_ENTRY_1_ADDR_REG + ((entry - 1) * 4);

  /* Write ENTRY address */

  putreg32(addr, reg);

  /* Enable check for that particular address. When fetched, World will
   * switch to World 0.
   */

  modifyreg32(WCL_CORE_0_ENTRY_CHECK_REG, 0, BIT(entry));
}

/****************************************************************************
 * Name: pms_violation_isr
 *
 * Description:
 *   This is the common PMS interrupt handler. It will be invoked the PMS
 *   detects an access violation.
 *
 * Parameters:
 *   cpuint        - CPU interrupt index
 *   context       - Context data from the ISR
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int IRAM_ATTR pms_violation_isr(int cpuint, void *context, void *arg)
{
  PANIC();

  return OK;
}

/****************************************************************************
 * Name: pms_enable_interrupts
 *
 * Description:
 *   Configure top level permission violation interrupts and set World
 *   Controller monitor registers.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_enable_interrupts(void)
{
  /* We use two separate vector tables for WORLD0 and WORLD1,
   * this allows us to handle Windowed exceptions in WORLD1 itself, thus
   * saving CPU cycles.
   * For other vectors, we jump to WORLD0 vector table from WORLD1 vector
   * table.
   * The vector table for WORLD1 is placed right at the start of WORLD1 IRAM.
   */

  wcl_set_vecbase(PMS_WORLD_0, VECTORS_START);
  wcl_set_vecbase(PMS_WORLD_1, UIRAM_START);

  extern void _user_exception_vector(void);
  wcl_set_world0_entry(1, (uintptr_t)_user_exception_vector);
  extern void _xtensa_level3_vector(void);
  wcl_set_world0_entry(2, (uintptr_t)_xtensa_level3_vector);

  /* Enable IRAM0 permission violation interrupt */

  modifyreg32(SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_EN);

  /* Enable DRAM0 permission violation interrupt */

  modifyreg32(SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_EN);

  /* Enable Flash Instruction Cache permission violation interrupt */

  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_CLR_REG,
              EXTMEM_CORE0_IBUS_REJECT_INT_CLR_M,
              0);
  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_ENA_REG,
              EXTMEM_CORE0_IBUS_REJECT_INT_ENA_M,
              EXTMEM_CORE0_IBUS_REJECT_INT_ENA);

  /* Enable Flash Data Cache permission violation interrupt */

  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_CLR_REG,
              EXTMEM_CORE0_DBUS_REJECT_INT_CLR_M,
              0);
  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_ENA_REG,
              EXTMEM_CORE0_DBUS_REJECT_INT_ENA_M,
              EXTMEM_CORE0_DBUS_REJECT_INT_ENA);

  /* Enable Flash Data Cache permission violation interrupt */

  modifyreg32(SENSITIVE_CORE_0_PIF_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_EN);
}

/****************************************************************************
 * Name: set_iram_split_line
 *
 * Description:
 *   Split the IRAM region into two sub regions.
 *
 * Input Parameters:
 *   addr          - Address for the split line.
 *   sensitive_reg - Register to which the split line configuration will be
 *                   applied.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_iram_split_line(uintptr_t addr, const uint32_t sensitive_reg)
{
  /* Set category bits for a given split line */

  uint32_t cat[7] =
    {
      [0 ... 6] = PMS_SRAM_CATEGORY_ABOVE
    };

  for (size_t x = 0; x < 7; x++)
    {
      if (addr <= g_sram_rg3_level_hlimits[x])
        {
          cat[x] = PMS_SRAM_CATEGORY_EQUAL;
          break;
        }
      else
        {
          cat[x] = PMS_SRAM_CATEGORY_BELOW;
        }
    }

  /* Resolve split address' significant bits.
   * Split address must be aligned to 256 bytes.
   */

  uint32_t regval =
    VALUE_TO_PMS_FIELD((addr >> 8), IRAM0_DRAM0_DMA_SRAM_SPLITADDR) |
    VALUE_TO_PMS_FIELD(cat[6], IRAM0_DRAM0_DMA_SRAM_CATEGORY_6) |
    VALUE_TO_PMS_FIELD(cat[5], IRAM0_DRAM0_DMA_SRAM_CATEGORY_5) |
    VALUE_TO_PMS_FIELD(cat[4], IRAM0_DRAM0_DMA_SRAM_CATEGORY_4) |
    VALUE_TO_PMS_FIELD(cat[3], IRAM0_DRAM0_DMA_SRAM_CATEGORY_3) |
    VALUE_TO_PMS_FIELD(cat[2], IRAM0_DRAM0_DMA_SRAM_CATEGORY_2) |
    VALUE_TO_PMS_FIELD(cat[1], IRAM0_DRAM0_DMA_SRAM_CATEGORY_1) |
    VALUE_TO_PMS_FIELD(cat[0], IRAM0_DRAM0_DMA_SRAM_CATEGORY_0);

  putreg32(regval, sensitive_reg);
}

/****************************************************************************
 * Name: pms_set_sram_main_split_line
 *
 * Description:
 *   Configure the Internal SRAM1 Instruction and Data regions.
 *
 * Input Parameters:
 *   addr          - Address for the split line.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void pms_set_sram_main_split_line(uintptr_t addr)
{
  set_iram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_1_REG);
}

/****************************************************************************
 * Name: pms_set_iram_split_line
 *
 * Description:
 *   Helper function for setting the a split line into IRAM region.
 *
 * Input Parameters:
 *   line          - Split line to be set.
 *   addr          - Address for the split line.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void pms_set_iram_split_line(enum pms_split_line_e line,
                                           uintptr_t addr)
{
  switch (line)
    {
      case PMS_SPLIT_LINE_0:
        {
          set_iram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_2_REG);
        }
        break;
      case PMS_SPLIT_LINE_1:
        {
          set_iram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_3_REG);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: pms_configure_iram_split_region
 *
 * Description:
 *   Configure access permissions to a given split region in IRAM.
 *
 * Input Parameters:
 *   area          - A given region created after setting a split line.
 *   world         - World to which the flags will apply to.
 *   flags         - Attributes representing the operations allowed to be
 *                   performed within the target area.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_iram_split_region(enum pms_area_e area,
                                            enum pms_world_e world,
                                            enum pms_flags_e flags)
{
  uint32_t reg;
  uint32_t offset;

  if (world == PMS_WORLD_0)
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_2_REG;
      offset = IRAM_PMS_W0_BASE;
    }
  else
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_1_REG;
      offset = IRAM_PMS_W1_BASE;
    }

  uint32_t shift = offset + (area * IRAM_PMS_S);
  uint32_t mask = IRAM_PMS_V << shift;
  uint32_t val = (flags & IRAM_PMS_V) << shift;

  modifyreg32(reg, mask, val);
}

/****************************************************************************
 * Name: pms_configure_icache_permission
 *
 * Description:
 *   Configure access permissions to the Internal SRAM 0 blocks that won't be
 *   used as Instruction Cache.
 *
 * Input Parameters:
 *   area          - Which of the two SRAM blocks used as Instruction Cache.
 *   world         - World to which the flags will apply to.
 *   flags         - Attributes representing the operations allowed to be
 *                   performed within the target area.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_icache_permission(enum pms_area_e area,
                                            enum pms_world_e world,
                                            enum pms_flags_e flags)
{
  uint32_t reg;
  uint32_t offset;

  if (world == PMS_WORLD_0)
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_2_REG;
      offset = ICACHE_PMS_W0_BASE;
    }
  else
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_1_REG;
      offset = ICACHE_PMS_W1_BASE;
    }

  uint32_t shift = offset + (area * ICACHE_PMS_S);
  uint32_t mask = ICACHE_PMS_V << shift;
  uint32_t val = (flags & ICACHE_PMS_V) << shift;

  modifyreg32(reg, mask, val);
}

/****************************************************************************
 * Name: pms_configure_dcache_permission
 *
 * Description:
 *   Configure access permissions to the Internal SRAM 2 blocks that won't be
 *   used as Data Cache.
 *
 * Input Parameters:
 *   world         - World to which the flags will apply to.
 *   flags         - Attributes representing the operations allowed to be
 *                   performed within the target area.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_dcache_permission(enum pms_world_e world,
                                            enum pms_flags_e flags)
{
  switch (world)
    {
      case PMS_WORLD_0:
        {
          modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
    SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_0_M
  | SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_1_M,
    VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_0)
  | VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_1));
        }
        break;
      case PMS_WORLD_1:
        {
          modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
    SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_0_M
  | SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_1_M,
    VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_0)
  | VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_1));
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: set_dram_split_line
 *
 * Description:
 *   Split the DRAM region into two sub regions.
 *
 * Input Parameters:
 *   addr          - Address for the split line.
 *   sensitive_reg - Register to which the split line configuration will be
 *                   applied.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void set_dram_split_line(uintptr_t addr, const uint32_t sensitive_reg)
{
  /* Set category bits for a given split line */

  uint32_t cat[7] =
    {
      [0 ... 6] = PMS_SRAM_CATEGORY_ABOVE
    };

  for (size_t x = 0; x < 7; x++)
    {
      if (addr <= MAP_IRAM_TO_DRAM(g_sram_rg3_level_hlimits[x]))
        {
          cat[x] = PMS_SRAM_CATEGORY_EQUAL;
          break;
        }
      else
        {
          cat[x] = PMS_SRAM_CATEGORY_BELOW;
        }
    }

  /* Resolve split address' significant bits.
   * Split address must be aligned to 256 bytes.
   */

  uint32_t regval =
    VALUE_TO_PMS_FIELD((addr >> 8), DRAM0_DMA_SRAM_LINE_0_SPLITADDR) |
    VALUE_TO_PMS_FIELD(cat[6], DRAM0_DMA_SRAM_LINE_0_CATEGORY_6) |
    VALUE_TO_PMS_FIELD(cat[5], DRAM0_DMA_SRAM_LINE_0_CATEGORY_5) |
    VALUE_TO_PMS_FIELD(cat[4], DRAM0_DMA_SRAM_LINE_0_CATEGORY_4) |
    VALUE_TO_PMS_FIELD(cat[3], DRAM0_DMA_SRAM_LINE_0_CATEGORY_3) |
    VALUE_TO_PMS_FIELD(cat[2], DRAM0_DMA_SRAM_LINE_0_CATEGORY_2) |
    VALUE_TO_PMS_FIELD(cat[1], DRAM0_DMA_SRAM_LINE_0_CATEGORY_1) |
    VALUE_TO_PMS_FIELD(cat[0], DRAM0_DMA_SRAM_LINE_0_CATEGORY_0);

  putreg32(regval, sensitive_reg);
}

/****************************************************************************
 * Name: pms_set_dram_split_line
 *
 * Description:
 *   Helper function for setting the a split line into DRAM region.
 *
 * Input Parameters:
 *   line          - Split line to be set.
 *   addr          - Address for the split line.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_set_dram_split_line(enum pms_split_line_e line,
                                    uintptr_t addr)
{
  switch (line)
    {
      case PMS_SPLIT_LINE_0:
        {
          set_dram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_4_REG);
        }
        break;
      case PMS_SPLIT_LINE_1:
        {
          set_dram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_5_REG);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: pms_configure_dram_split_region
 *
 * Description:
 *   Configure access permissions to a given split region in DRAM.
 *
 * Input Parameters:
 *   area          - A given region created after setting a split line.
 *   world         - World to which the flags will apply to.
 *   flags         - Attributes representing the operations allowed to be
 *                   performed within the target area.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_dram_split_region(enum pms_area_e area,
                                            enum pms_world_e world,
                                            enum pms_flags_e flags)
{
  uint32_t offset;

  if (world == PMS_WORLD_0)
    {
      offset = DRAM_PMS_W0_BASE;
    }
  else
    {
      offset = DRAM_PMS_W1_BASE;
    }

  uint32_t shift = offset + (area * DRAM_PMS_S);
  uint32_t mask = DRAM_PMS_V << shift;
  uint32_t val = (flags & DRAM_PMS_V) << shift;

  modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG, mask, val);
}

/****************************************************************************
 * Name: pms_set_flash_cache_split_line
 *
 * Description:
 *   Split the External Flash region into sub regions.
 *
 * Input Parameters:
 *   line          - Split line to be set.
 *   addr          - Starting address for the new region.
 *   length        - Length of the new region in bytes.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_set_flash_cache_split_line(enum pms_split_line_e line,
                                           uintptr_t addr, size_t length)
{
  /* The starting address of each region should be aligned to 64 KB */

  uintptr_t aligned_addr = ALIGN_DOWN(addr, MMU_PAGE_SIZE);

  /* The length of each region should be the integral multiples of 64 KB */

  size_t length_pages = length / MMU_PAGE_SIZE;

  switch (line)
    {
      case PMS_SPLIT_LINE_0:
        {
          modifyreg32(APB_CTRL_FLASH_ACE0_ADDR_REG,
                      APB_CTRL_FLASH_ACE0_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE0_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE0_SIZE_REG,
                      APB_CTRL_FLASH_ACE0_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE0_SIZE));
        }
        break;
      case PMS_SPLIT_LINE_1:
        {
          modifyreg32(APB_CTRL_FLASH_ACE1_ADDR_REG,
                      APB_CTRL_FLASH_ACE1_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE1_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE1_SIZE_REG,
                      APB_CTRL_FLASH_ACE1_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE1_SIZE));
        }
        break;
      case PMS_SPLIT_LINE_2:
        {
          modifyreg32(APB_CTRL_FLASH_ACE2_ADDR_REG,
                      APB_CTRL_FLASH_ACE2_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE2_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE2_SIZE_REG,
                      APB_CTRL_FLASH_ACE2_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE2_SIZE));
        }
        break;
      case PMS_SPLIT_LINE_3:
        {
          modifyreg32(APB_CTRL_FLASH_ACE3_ADDR_REG,
                      APB_CTRL_FLASH_ACE3_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE3_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE3_SIZE_REG,
                      APB_CTRL_FLASH_ACE3_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE3_SIZE));
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: pms_configure_flash_cache_split_region
 *
 * Description:
 *   Configure access permissions to a given split region in External Flash.
 *
 * Input Parameters:
 *   area          - A given region created after setting a split line.
 *   world         - World to which the flags will apply to.
 *   flags         - Attributes representing the operations allowed to be
 *                   performed within the target area.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void
pms_configure_flash_cache_split_region(enum pms_area_e area,
                                       enum pms_world_e world,
                                       enum pms_flags_e flags)
{
  const uint32_t shift = (FLASH_CACHE_S * world);
  const uint32_t mask = FLASH_CACHE_V << shift;
  uint32_t attr;

  if (flags == PMS_ACCESS_ALL)
    {
      attr = 0b11;
    }
  else if ((flags & PMS_ACCESS_W) != 0)
    {
      PANIC();
    }
  else if ((flags & PMS_ACCESS_X) != 0)
    {
      attr = flags | 0b1;
    }
  else
    {
      attr = flags;
    }

  uint32_t val = 0x40 | (attr & FLASH_CACHE_V) << shift;

  switch (area)
    {
      case PMS_AREA_0:
        {
          modifyreg32(APB_CTRL_FLASH_ACE0_ATTR_REG, mask, val);
        }
        break;
      case PMS_AREA_1:
        {
          modifyreg32(APB_CTRL_FLASH_ACE1_ATTR_REG, mask, val);
        }
        break;
      case PMS_AREA_2:
        {
          modifyreg32(APB_CTRL_FLASH_ACE2_ATTR_REG, mask, val);
        }
        break;
      case PMS_AREA_3:
        {
          modifyreg32(APB_CTRL_FLASH_ACE3_ATTR_REG, mask, val);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: pms_configure_peripheral_permission
 *
 * Description:
 *   Configure access permissions to a given peripheral.
 *
 * Input Parameters:
 *   periph        - A given peripheral to be configured.
 *   world         - World to which the flags will apply to.
 *   flags         - Attributes representing the operations allowed to be
 *                   performed with the selected peripheral.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_peripheral_permission(enum pms_peripheral_e periph,
                                                enum pms_world_e world,
                                                enum pms_flags_e flags)
{
  uint32_t reg = 0;
  uint32_t reg_off = periph / PIF_PMS_MAX_REG_ENTRY;
  uint32_t bit_field_base = 30 - (2 * (periph % PIF_PMS_MAX_REG_ENTRY));

  switch (world)
    {
      case PMS_WORLD_0:
        {
          reg = SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_1_REG + (4 * reg_off);
        }
        break;
      case PMS_WORLD_1:
        {
          reg = SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_5_REG + (4 * reg_off);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }

  uint32_t mask = PIF_PMS_V << bit_field_base;
  uint32_t val = (flags & PIF_PMS_V) << bit_field_base;

  modifyreg32(reg, mask, val);
}

/****************************************************************************
 * Name: pms_configure_irom_access
 *
 * Description:
 *   Configure the access permissions to the IROM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_irom_access(void)
{
  /* Kernel permission to the IROM region */

  modifyreg32(SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_2_REG,
              SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_ALL,
                                 IRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS));

  /* User permission to the IROM region */

  modifyreg32(SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_1_REG,
              SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_NONE,
                                 IRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS));
}

/****************************************************************************
 * Name: pms_configure_drom_access
 *
 * Description:
 *   Configure the access permissions to the DROM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_drom_access(void)
{
  /* Kernel permission to the DROM region */

  modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
              SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_ALL,
                                 DRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS));

  /* User permission to the DROM region */

  modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
              SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_NONE,
                                 DRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS));
}

/****************************************************************************
 * Name: pms_configure_iram_access
 *
 * Description:
 *   Configure the access permissions to the IRAM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_iram_access(void)
{
  /* Kernel permission to the Instruction Cache */

  pms_configure_icache_permission(PMS_AREA_0, PMS_WORLD_0, PMS_ACCESS_ALL);
  pms_configure_icache_permission(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);

  /* User permission to the Instruction Cache */

  pms_configure_icache_permission(PMS_AREA_0, PMS_WORLD_1, PMS_ACCESS_NONE);
#ifdef CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB
  /* In case the Instruction Cache size is configured to 16KB, the other 16KB
   * block from Internal SRAM0 (Block1) will be used as IRAM.
   */

  pms_configure_icache_permission(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_ALL);
#else /* CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB */
  /* In case the Instruction Cache size is configured to 32KB, the WORLD1
   * access permissions to the Internal SRAM0 must be revoked.
   */

  pms_configure_icache_permission(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_NONE);
#endif

  /* Set split lines to partition the IRAM into regions */

  pms_set_iram_split_line(PMS_SPLIT_LINE_0, UIRAM_END);
  pms_set_iram_split_line(PMS_SPLIT_LINE_1, KIRAM_END);

  /* Configure Kernel access permissions to each split region */

  pms_configure_iram_split_region(PMS_AREA_0, PMS_WORLD_0, PMS_ACCESS_ALL);
  pms_configure_iram_split_region(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);
  pms_configure_iram_split_region(PMS_AREA_2, PMS_WORLD_0, PMS_ACCESS_ALL);

  /* Configure User access permissions to each split region */

#ifdef CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB
  /* In case the Instruction Cache size is configured to 16KB, the IRAM
   * allocation to WORLD1 will be entirely restricted to Internal SRAM0, so
   * we can safely revoke permissions to the shared Internal SRAM1.
   */

  pms_configure_iram_split_region(PMS_AREA_0, PMS_WORLD_1, PMS_ACCESS_NONE);
#else /* CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB */
  /* In case the Instruction Cache size is configured to 32KB, the first
   * block from the shared Internal SRAM1 (Block2) will be allocated to
   * WORLD1.
   */

  pms_configure_iram_split_region(PMS_AREA_0, PMS_WORLD_1, PMS_ACCESS_ALL);
#endif
  pms_configure_iram_split_region(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_NONE);
  pms_configure_iram_split_region(PMS_AREA_2, PMS_WORLD_1, PMS_ACCESS_NONE);

  /* PMS_AREA_3 corresponds to the region after the main split line,
   * i.e. the entire DRAM.
   */

  pms_configure_iram_split_region(PMS_AREA_3, PMS_WORLD_0, PMS_ACCESS_NONE);
  pms_configure_iram_split_region(PMS_AREA_3, PMS_WORLD_1, PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: pms_configure_dram_access
 *
 * Description:
 *   Configure the access permissions to the DRAM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_dram_access(void)
{
  /* Kernel permission to the Data Cache */

  pms_configure_dcache_permission(PMS_WORLD_0, PMS_ACCESS_ALL);

  /* User permission to the Data Cache */

  pms_configure_dcache_permission(PMS_WORLD_1, PMS_ACCESS_NONE);

  /* Split line to protect DRAM area reserved for ROM functions.
   * ALIGN_DOWN macro is used to align the address to 256 bit boundary.
   */

  uintptr_t rom_reserve_aligned =
    ALIGN_DOWN(ets_rom_layout_p->dram0_rtos_reserved_start, 256);

  /* Set split lines to partition the DRAM into regions */

  pms_set_dram_split_line(PMS_SPLIT_LINE_0, UDRAM_START);
  pms_set_dram_split_line(PMS_SPLIT_LINE_1, rom_reserve_aligned);

  /* PMS_AREA_0 corresponds to the region before the main split line,
   * i.e entire IRAM.
   */

  pms_configure_dram_split_region(PMS_AREA_0, PMS_WORLD_0, PMS_ACCESS_NONE);
  pms_configure_dram_split_region(PMS_AREA_0, PMS_WORLD_1, PMS_ACCESS_NONE);

  /* Configure Kernel access permissions to each split region */

  pms_configure_dram_split_region(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);
  pms_configure_dram_split_region(PMS_AREA_2, PMS_WORLD_0, PMS_ACCESS_ALL);
  pms_configure_dram_split_region(PMS_AREA_3, PMS_WORLD_0, PMS_ACCESS_ALL);

  /* Configure User access permissions to each split region */

  pms_configure_dram_split_region(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_NONE);
  pms_configure_dram_split_region(PMS_AREA_2, PMS_WORLD_1, PMS_ACCESS_ALL);
  pms_configure_dram_split_region(PMS_AREA_3, PMS_WORLD_1, PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: pms_configure_flash_cache_access
 *
 * Description:
 *   Configure the access permissions to the region dedicated to the
 *   Instruction and Data Caches.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void pms_configure_flash_cache_access(void)
{
  /* Invalidate Cache */

  uint32_t cache_state = dcache_suspend(true);
  cache_invalidate_icache_all();

  size_t partition_offset = USER_IMAGE_OFFSET;

  /* Total image size must be aligned to the MMU page size (64 KB) to match
   * the required granularity of the PMS split regions for the External
   * Flash.
   * IROM region offset in External Flash plus its size provides an accurate
   * estimate about the User Image size.
   */

  size_t total_size = ALIGN_UP(g_header.irom_lma + g_header.irom_size,
                               MMU_PAGE_SIZE);
  size_t remaining_size = EXTMEM_MAX_LENGTH - partition_offset - total_size;

  uint32_t region0_start_addr = 0x0;
  uint32_t region0_size = partition_offset;
  uint32_t region1_start_addr = region0_start_addr + region0_size;
  uint32_t region1_size = total_size;
  uint32_t region2_start_addr = region1_start_addr + region1_size;
  uint32_t region2_size = remaining_size / 2;
  uint32_t region3_start_addr = region2_start_addr + region2_size;
  uint32_t region3_size = remaining_size / 2;

  pms_set_flash_cache_split_line(PMS_SPLIT_LINE_0, region0_start_addr,
                                 region0_size);
  pms_set_flash_cache_split_line(PMS_SPLIT_LINE_1, region1_start_addr,
                                 region1_size);
  pms_set_flash_cache_split_line(PMS_SPLIT_LINE_2, region2_start_addr,
                                 region2_size);
  pms_set_flash_cache_split_line(PMS_SPLIT_LINE_3, region3_start_addr,
                                 region3_size);

  /* Configure Kernel access permissions to each split region */

  pms_configure_flash_cache_split_region(PMS_AREA_0, PMS_WORLD_0,
                                         PMS_ACCESS_ALL);

  /* WORLD0 requires access to WORLD1 to load the cache when returning to
   * WORLD1 from WORLD0.
   */

  pms_configure_flash_cache_split_region(PMS_AREA_1, PMS_WORLD_0,
                                         PMS_ACCESS_ALL);
  pms_configure_flash_cache_split_region(PMS_AREA_2, PMS_WORLD_0,
                                         PMS_ACCESS_ALL);
  pms_configure_flash_cache_split_region(PMS_AREA_3, PMS_WORLD_0,
                                         PMS_ACCESS_ALL);

  /* Configure User access permissions to each split region */

  pms_configure_flash_cache_split_region(PMS_AREA_0, PMS_WORLD_1,
                                         PMS_ACCESS_NONE);
  pms_configure_flash_cache_split_region(PMS_AREA_1, PMS_WORLD_1,
                                         PMS_ACCESS_ALL);
  pms_configure_flash_cache_split_region(PMS_AREA_2, PMS_WORLD_1,
                                         PMS_ACCESS_NONE);
  pms_configure_flash_cache_split_region(PMS_AREA_3, PMS_WORLD_1,
                                         PMS_ACCESS_NONE);

  dcache_resume(cache_state);
}

/****************************************************************************
 * Name: pms_configure_peripheral_access
 *
 * Description:
 *   Configure Kernel and Userspace permissions for accessing the chip's
 *   peripherals.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_peripheral_access(void)
{
  /* Revoke User access permission to every peripheral */

  pms_configure_peripheral_permission(PMS_UART1, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_I2C, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_MISC, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_IO_MUX, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_RTC, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_FE, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_FE2, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_GPIO, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_G0SPI_0, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_G0SPI_1, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_UART, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SYSTIMER, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_TIMERGROUP1, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_TIMERGROUP, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_BB, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_LEDC, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_RMT, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_UHCI0, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_I2C_EXT0, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_BT, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_PWR, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_WIFIMAC, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_RWBT, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_I2S1, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_CAN, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_APB_CTRL, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SPI_2, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_WORLD_CONTROLLER, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_DIO, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_AD, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_CACHE_CONFIG, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_DMA_COPY, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_INTERRUPT, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SENSITIVE, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SYSTEM, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_BT_PWR, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_APB_ADC, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_CRYPTO_DMA, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_CRYPTO_PERI, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_USB_WRAP, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_USB_DEVICE, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_I2S0, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_HINF, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_PWM0, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_BACKUP, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SLC, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_PCNT, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SLCHOST, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_UART2, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_PWM1, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SDIO_HOST, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_I2C_EXT1, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_SPI_3, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
  pms_configure_peripheral_permission(PMS_USB, PMS_WORLD_1,
                                      PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   Configure the MPU for kernel/userspace separation.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void configure_mpu(void)
{
  /* Configure interrupts for permission violation */

  pms_enable_interrupts();

  /* Define the Internal SRAM1 regions for code and data by configuring the
   * split lines.
   */

  pms_set_sram_main_split_line(KIRAM_END);

  /* Configure Kernel and Userspace permissions for accessing the internal
   * memories.
   */

  /* IROM */

  pms_configure_irom_access();

  /* DROM */

  pms_configure_drom_access();

  /* IRAM */

  pms_configure_iram_access();

  /* DRAM */

  pms_configure_dram_access();

  /* Instruction and Data Caches */

  pms_configure_flash_cache_access();

  /* Configure Kernel and Userspace permissions for accessing the chip's
   * peripherals.
   */

  pms_configure_peripheral_access();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_userspace
 *
 * Description:
 *   For the case of the separate user/kernel space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the userspace .data and .bss
 *   segments.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_userspace(void)
{
  /* Load metadata information from image header */

  load_header();

  /* Configure the Flash MMU for enabling access to the userspace image */

  configure_flash_mmu();

  /* Initialize userspace DRAM */

  initialize_dram();

  /* Initialize userspace IRAM */

  initialize_iram();

  /* Configure MPU to grant access to the userspace */

  configure_mpu();
}

/****************************************************************************
 * Name: esp32s3_pmsirqinitialize
 *
 * Description:
 *   Initialize interrupt handler for the PMS violation ISR.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_pmsirqinitialize(void)
{
  VERIFY(esp32s3_setup_irq(0,
                           ESP32S3_PERIPH_CORE_0_IRAM0_PMS_MONITOR_VIOLATE,
                           1, ESP32S3_CPUINT_LEVEL));
  VERIFY(esp32s3_setup_irq(0,
                           ESP32S3_PERIPH_CORE_0_DRAM0_PMS_MONITOR_VIOLATE,
                           1, ESP32S3_CPUINT_LEVEL));
  VERIFY(esp32s3_setup_irq(0, ESP32S3_PERIPH_CACHE_CORE0_ACS, 1,
                           ESP32S3_CPUINT_LEVEL));
  VERIFY(esp32s3_setup_irq(0, ESP32S3_PERIPH_CORE_0_PIF_PMS_MONITOR_VIOLATE,
                           1, ESP32S3_CPUINT_LEVEL));

  VERIFY(irq_attach(ESP32S3_IRQ_CORE_0_IRAM0_PMS_MONITOR_VIOLATE,
                    pms_violation_isr, NULL));
  VERIFY(irq_attach(ESP32S3_IRQ_CORE_0_DRAM0_PMS_MONITOR_VIOLATE,
                    pms_violation_isr, NULL));
  VERIFY(irq_attach(ESP32S3_IRQ_CACHE_CORE0_ACS, pms_violation_isr, NULL));
  VERIFY(irq_attach(ESP32S3_IRQ_CORE_0_PIF_PMS_MONITOR_VIOLATE,
                    pms_violation_isr, NULL));

  up_enable_irq(ESP32S3_IRQ_CORE_0_IRAM0_PMS_MONITOR_VIOLATE);
  up_enable_irq(ESP32S3_IRQ_CORE_0_DRAM0_PMS_MONITOR_VIOLATE);
  up_enable_irq(ESP32S3_IRQ_CACHE_CORE0_ACS);
  up_enable_irq(ESP32S3_IRQ_CORE_0_PIF_PMS_MONITOR_VIOLATE);
}

#endif /* CONFIG_BUILD_PROTECTED */
