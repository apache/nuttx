/****************************************************************************
 * arch/arm/src/imxrt/imxrt106x_flash.c
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

/* Provides standard flash access functions, to be used by the flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <debug.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "barriers.h"

#include "hardware/rt106x/imxrt106x_memorymap.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_IMXRT_PROGMEM_FLEXSPI_INSTANCE == 0
#define FLEXSPI_INSTANCE        (0)
#else
#define FLEXSPI_INSTANCE        (1)
#endif

#define FLASH_PAGE_SIZE         (256UL)
#define FLASH_SECTOR_SIZE       (4096UL)
#define FLASH_ERASEDVALUE       (0xffu)
#define FLASH_ERASEDVALUE_DW    (0xffffffffu)
#define PROGMEM_NBLOCKS         (1024UL)
#define FLASH_NPAGES            ((4UL * 1024UL * 1024UL) / FLASH_PAGE_SIZE)

#define FLEXSPI_CFG_BLK_TAG     (0x42464346UL)
#define FLEXSPI_CFG_BLK_VERSION (0x56010400UL)
#define FLEXSPI_CFG_BLK_SIZE    (512)

#define CMD_SDR 0x01
#define CMD_DDR 0x21
#define RADDR_SDR 0x02
#define RADDR_DDR 0x22
#define CADDR_SDR 0x03
#define CADDR_DDR 0x23
#define MODE1_SDR 0x04
#define MODE1_DDR 0x24
#define MODE2_SDR 0x05
#define MODE2_DDR 0x25
#define MODE4_SDR 0x06
#define MODE4_DDR 0x26
#define MODE8_SDR 0x07
#define MODE8_DDR 0x27
#define WRITE_SDR 0x08
#define WRITE_DDR 0x28
#define READ_SDR 0x09
#define READ_DDR 0x29
#define LEARN_SDR 0x0a
#define LEARN_DDR 0x2a
#define DATSZ_SDR 0x0b
#define DATSZ_DDR 0x2b
#define DUMMY_SDR 0x0c
#define DUMMY_DDR 0x2c
#define DUMMY_RWDS_SDR 0x0d
#define DUMMY_RWDS_DDR 0x2d
#define JMP_ON_CS 0x1f
#define STOP 0

#define FLEXSPI_1PAD 0
#define FLEXSPI_2PAD 1
#define FLEXSPI_4PAD 2
#define FLEXSPI_8PAD 3

#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1) \
    (FLEXSPI_LUT_OPERAND0(op0) | \
     FLEXSPI_LUT_NUM_PADS0(pad0) | \
     FLEXSPI_LUT_OPCODE0(cmd0) | \
     FLEXSPI_LUT_OPERAND1(op1) | \
     FLEXSPI_LUT_NUM_PADS1(pad1) | \
     FLEXSPI_LUT_OPCODE1(cmd1))

typedef enum
{
  FLEXSPI_SERIAL_CLK_NO_CHANGE = 0,
  FLEXSPI_SERIAL_CLK_30MHz = 1,
  FLEXSPI_SERIAL_CLK_50MHz = 2,
  FLEXSPI_SERIAL_CLK_60MHz = 3,
  FLEXSPI_SERIAL_CLK_75MHz = 4,
  FLEXSPI_SERIAL_CLK_80MHz = 5,
  FLEXSPI_SERIAL_CLK_100MHz = 6,
  FLEXSPI_SERIAL_CLK_120MHz = 7,
  FLEXSPI_SERIAL_CLK_133MHz = 8,
  FLEXSPI_SERIAL_CLK_166MHz = 9,
} flexspi_serial_clk_freq_t;

enum
{
  FLEXSPI_CLK_SDR,
  FLEXSPI_CLK_DDR,
};

typedef enum
{
  FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_INTERNALLY = 0,
  FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_DQS_PAD = 1,
  FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_SCK_PAD = 2,
  FLEXSPI_READ_SAMPLE_CLK_EXTERNAL_INPUT_FROM_DQS_PAD = 3,
} flexspi_read_sample_clk_t;

enum
{
  FLEXSPI_MISC_OFFSET_DIFF_CLK_ENABLE = 0,
  FLEXSPI_MISC_OFFSET_CK2_ENABLE = 1,
  FLEXSPI_MISC_OFFSET_PARALLEL_ENABLE = 2,
  FLEXSPI_MISC_OFFSET_WORD_ADDRESSABLE_ENABLE = 3,
  FLEXSPI_MISC_OFFSET_SAFE_CONFIG_FREQ_ENABLE = 4,
  FLEXSPI_MISC_OFFSET_PAD_SETTING_OVERRIDE_ENABLE = 5,
  FLEXSPI_MISC_OFFSET_DDR_MODE_ENABLE = 6,
  FLEXSPI_MISC_OFFSET_USE_VALID_TIME_FOR_ALL_FREQ = 7,
  FLEXSPI_MISC_OFFSET_SECOND_PINMUX = 8,
};

enum
{
  FLEXSPI_DEVICE_TYPE_SERIAL_NOR = 1,
  FLEXSPI_DEVICE_TYPE_SERIAL_NAND = 2,
};

enum
{
  SERIAL_FLASH_1PAD = 1,
  SERIAL_FLASH_2PADS = 2,
  SERIAL_FLASH_4PADS = 4,
  SERIAL_FLASH_8PADS = 8,
};

typedef struct _lut_sequence
{
  uint8_t seq_num;
  uint8_t seq_id;
  uint16_t reserved;
} flexspi_lut_seq_t;

enum
{
  DEVICE_CONFIG_CMD_TYPE_GENERIC,
  DEVICE_CONFIG_CMD_TYPE_QUADENABLE,
  DEVICE_CONFIG_CMD_TYPE_SPI2XPI,
  DEVICE_CONFIG_CMD_TYPE_XPI2SPI,
  DEVICE_CONFIG_CMD_TYPE_SPI2NOCMD,
  DEVICE_CONFIG_CMD_TYPE_RESET,
};

typedef struct
{
  uint8_t time_100ps;
  uint8_t delay_cells;
} flexspi_dll_time_t;

typedef struct
{
  uint32_t tag;
  uint32_t version;
  uint32_t reserved0;
  uint8_t read_sample_clk_src;
  uint8_t data_hold_time;
  uint8_t data_setup_time;
  uint8_t column_address_width;
  uint8_t device_mode_cfg_enable;
  uint8_t device_mode_type;
  uint16_t wait_time_cfg_commands;
  flexspi_lut_seq_t device_mode_seq;
  uint32_t device_mode_arg;
  uint8_t config_cmd_enable;
  uint8_t config_mode_type[3];
  flexspi_lut_seq_t config_cmd_seqs[3];
  uint32_t reserved1;
  uint32_t config_cmd_args[3];
  uint32_t reserved2;
  uint32_t controller_misc_option;
  uint8_t device_type;
  uint8_t sflash_pad_type;
  uint8_t serial_clk_freq;
  uint8_t lut_custom_seq_enable;
  uint32_t reserved3[2];
  uint32_t sflash_a1_size;
  uint32_t sflash_a2_size;
  uint32_t sflash_b1_size;
  uint32_t sflash_b2_size;
  uint32_t cs_pad_setting_override;
  uint32_t sclk_pad_setting_override;
  uint32_t data_pad_setting_override;
  uint32_t dqs_pad_setting_override;
  uint32_t timeout_in_ms;
  uint32_t command_interval;
  flexspi_dll_time_t data_valid_time[2];
  uint16_t busy_offset;
  uint16_t busy_bit_polarity;
  uint32_t lookup_table[64];
  flexspi_lut_seq_t lut_custom_seq[12];
  uint32_t reserved4[4];
} flexspi_mem_config_t;

typedef enum
{
  FLEXSPI_OPERATION_COMMAND,
  FLEXSPI_OPERATION_CONFIG,
  FLEXSPI_OPERATION_WRITE,
  FLEXSPI_OPERATION_READ,
  FLEXSPI_OPERATION_END = FLEXSPI_OPERATION_READ,
} flexspi_operation_t;

typedef struct
{
  flexspi_operation_t operation;
  uint32_t base_address;
  uint32_t seq_id;
  uint32_t seq_num;
  bool is_parallel_mode_enable;
  uint32_t *tx_buffer;
  uint32_t tx_size;
  uint32_t *rx_buffer;
  uint32_t rx_size;
} flexspi_xfer_t;

typedef enum
{
  FLEXSPI_CLOCK_CORE_CLOCK,
  FLEXSPI_CLOCK_AHB_CLOCK,
  FLEXSPI_CLOCK_SERIAL_ROOT_CLOCK,
  FLEXSPI_CLOCK_IPG_CLOCK,
} flexspi_clock_type_t;

#define FLEXSPI_BITMASK(bit_offset) (1u << (bit_offset))

enum
{
  STATUS_GROUP_GENERIC = 0,
  STATUS_GROUP_FLEXSPINOR = 201,
};

typedef int32_t status_t;

#define MAKE_STATUS(group, code) ((((group)*100) + (code)))

enum
{
  STATUS_SUCCESS         = MAKE_STATUS(STATUS_GROUP_GENERIC, 0),
  STATUS_FAIL            = MAKE_STATUS(STATUS_GROUP_GENERIC, 1),
  STATUS_READONLY        = MAKE_STATUS(STATUS_GROUP_GENERIC, 2),
  STATUS_OUTOFRANGE      = MAKE_STATUS(STATUS_GROUP_GENERIC, 3),
  STATUS_INVALIDARGUMENT = MAKE_STATUS(STATUS_GROUP_GENERIC, 4),
  STATUS_TIMEOUT         = MAKE_STATUS(STATUS_GROUP_GENERIC, 5),
  STATUS_NOTRANSFERINPROGRESS =
    MAKE_STATUS(STATUS_GROUP_GENERIC, 6),
  STATUS_BUSY = MAKE_STATUS(STATUS_GROUP_GENERIC, 7),
  STATUS_NODATA =
    MAKE_STATUS(STATUS_GROUP_GENERIC, 8),
};

enum _flexspi_nor_status
{
  STATUS_FLEXSPINOR_PROGRAMFAIL = MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 0),
  STATUS_FLEXSPINOR_ERASESECTORFAIL =
    MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 1),
  STATUS_FLEXSPINOR_ERASEALLFAIL = MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 2),
  STATUS_FLEXSPINOR_WAITTIMEOUT = MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 3),
  STATUS_FLEXSPINOR_NOTSUPPORTED = MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 4),
  STATUS_FLEXSPINOR_WRITEALIGNMENTERROR =
    MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 5),
  STATUS_FLEXSPINOR_COMMANDFAILURE =
    MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 6),
  STATUS_FLEXSPINOR_SFDP_NOTFOUND = MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 7),
  STATUS_FLEXSPINOR_UNSUPPORTED_SFDP_VERSION =
    MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 8),
  STATUS_FLEXSPINOR_FLASH_NOTFOUND = MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 9),
  STATUS_FLEXSPINOR_DTRREAD_DUMMYPROBEFAILED =
    MAKE_STATUS(STATUS_GROUP_FLEXSPINOR, 10),
};

enum
{
  SERIAL_NOR_CFG_OPTION_TAG = 0x0c,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_READSFDP_SDR = 0,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_READSFDP_DDR = 1,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_HYPERFLASH1V8 = 2,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_HYPERFLASH3V0 = 3,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_MACRONIXOCTALDDR = 4,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_MACRONIXOCTALSDR = 5,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_MICRONOCTALDDR = 6,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_MICRONOCTALSDR = 7,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_ADESTOOCTALDDR = 8,
  SERIAL_NOR_CFG_OPTION_DEVICETYPE_ADESTOOCTALSDR = 9,
};

enum
{
  SERIAL_NOR_QUAD_MODE_NOTCONFIG = 0,
  SERIAL_NOR_QUAD_MODE_STATUSREG1_BIT6 = 1,
  SERIAL_NOR_QUAD_MODE_STATUSREG2_BIT1 = 2,
  SERIAL_NOR_QUAD_MODE_STATUSREG2_BIT7 = 3,
  SERIAL_NOR_QUAD_MODE_STATUSREG2_BIT1_0X31 = 4,
};

enum
{
  SERIAL_NOR_ENHANCE_MODE_DISABLED = 0,
  SERIAL_NOR_ENHANCE_MODE_0_4_4_MODE = 1,
  SERIAL_NOR_ENHANCE_MODE_0_8_8_MODE = 2,
  SERIAL_NOR_ENHANCE_MODE_DATAORDERSWAPPED = 3,
  SERIAL_NOR_ENHANCE_MODE_2NDPINMUX = 4,
};

typedef struct _serial_nor_config_option
{
  union
  {
    struct
    {
      uint32_t max_freq : 4;
      uint32_t misc_mode : 4;
      uint32_t quad_mode_setting : 4;
      uint32_t cmd_pads : 4;
      uint32_t query_pads : 4;
      uint32_t device_type : 4;
      uint32_t option_size : 4;
      uint32_t tag : 4;
    } B;
    uint32_t U;
  } option0;

  union
  {
    struct
    {
      uint32_t dummy_cycles : 8;
      uint32_t status_override : 8;
      uint32_t is_pinmux_group2 : 4;
      uint32_t reserved : 12;
    } B;
    uint32_t U;
  } option1;
} serial_nor_config_option_t;

typedef struct _flexspi_nor_config
{
  flexspi_mem_config_t mem_config;
  uint32_t page_size;
  uint32_t sector_size;
  uint8_t ipcmd_serial_clk_freq;
  uint8_t is_uniform_block_size;
  uint8_t is_data_order_swapped;
  uint8_t reserved0[1];
  uint8_t serial_nor_type;
  uint8_t need_exit_no_cmd_mode;
  uint8_t half_clk_for_non_read_cmd;
  uint8_t need_restore_no_cmd_mode;
  uint32_t block_size;
  uint32_t reserve2[11];
} flexspi_nor_config_t;

typedef struct
{
  uint32_t version;
  status_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
  status_t (*program)(uint32_t instance, flexspi_nor_config_t *config,
                      uint32_t dst_addr, const uint32_t *src);
  status_t (*erase_all)(uint32_t instance, flexspi_nor_config_t *config);
  status_t (*erase)(uint32_t instance, flexspi_nor_config_t *config,
                    uint32_t start, uint32_t length_in_bytes);
  status_t (*read)(uint32_t instance, flexspi_nor_config_t *config,
                   uint32_t *dst, uint32_t addr, uint32_t length_in_bytes);
  void (*clear_cache)(uint32_t instance);
  status_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
  status_t (*update_lut)(uint32_t instance, uint32_t seq_index,
                         const uint32_t *lut_base, uint32_t seq_number);
  status_t (*get_config)(uint32_t instance, flexspi_nor_config_t *config,
                         serial_nor_config_option_t *option);
} flexspi_nor_driver_interface_t;

typedef struct
{
  const uint32_t version;
  const char *copyright;
  void (*run_bootloader)(void *arg);
  const uint32_t *reserved0;
  const flexspi_nor_driver_interface_t *flexspi_nor_driver;
  const uint32_t *reserved1[2];
  const void *rtwdog_driver;
  const void *wdog_driver;
  const uint32_t *reserved2;
} bootloader_api_entry_t;

enum
{
  ENTER_BOOTLOADER_TAG = 0xeb,
  ENTER_BOOTLOADER_MODE_DEFAULT = 0,
  ENTER_BOOTLOADER_MODE_SERIALDOWNLOADER = 1,

  ENTER_BOOTLOADER_SERIALINTERFACE_AUTO = 0,
  ENTER_BOOTLOADER_SERIALINTERFACE_USB = 1,
  ENTER_BOOTLOADER_SERIALINTERFACE_UART = 2,

  ENTER_BOOTLOADER_IMAGEINDEX_MAX = 3,
};

typedef union
{
  struct
  {
    uint32_t image_index : 4;
    uint32_t reserved : 12;
    uint32_t serial_boot_interface : 4;
    uint32_t boot_mode : 4;
    uint32_t tag : 8;
  } B;
  uint32_t U;
} run_bootloader_ctx_t;

#define g_bootloader_tree (*(bootloader_api_entry_t**)0x0020001c)

static inline void run_bootloader(run_bootloader_ctx_t *ctx)
{
  g_bootloader_tree->run_bootloader(ctx);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

status_t flexspi_nor_flash_init(uint32_t instance,
                                flexspi_nor_config_t *config)
{
  return g_bootloader_tree->flexspi_nor_driver->init(instance, config);
}

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t dst_addr,
                                        const uint32_t *src)
{
  return g_bootloader_tree->flexspi_nor_driver->program(instance, config,
         dst_addr, src);
}

status_t flexspi_nor_flash_erase_all(uint32_t instance,
                                     flexspi_nor_config_t *config)
{
  return g_bootloader_tree->flexspi_nor_driver->erase_all(instance, config);
}

status_t flexspi_nor_get_config(uint32_t instance,
                                flexspi_nor_config_t *config,
                                serial_nor_config_option_t *option)
{
  return g_bootloader_tree->flexspi_nor_driver->get_config(instance, config,
         option);
}

status_t flexspi_nor_flash_erase(uint32_t instance,
                                 flexspi_nor_config_t *config,
                                 uint32_t start,
                                 uint32_t length)
{
  return g_bootloader_tree->flexspi_nor_driver->erase(instance, config,
         start, length);
}

status_t flexspi_nor_flash_read(uint32_t instance,
                                flexspi_nor_config_t *config,
                                uint32_t *dst,
                                uint32_t start,
                                uint32_t bytes)
{
  return g_bootloader_tree->flexspi_nor_driver->read(instance, config,
         dst, start, bytes);
}

status_t flexspi_update_lut(uint32_t instance,
                            uint32_t seq_index,
                            const uint32_t *lut_base,
                            uint32_t number_of_seq)
{
  return g_bootloader_tree->flexspi_nor_driver->update_lut(instance,
                                                           seq_index,
                                                           lut_base,
                                                           number_of_seq);
}

status_t flexspi_command_xfer(uint32_t instance, flexspi_xfer_t *xfer)
{
  return g_bootloader_tree->flexspi_nor_driver->xfer(instance, xfer);
}

void flexspi_clear_cache(uint32_t instance)
{
  g_bootloader_tree->flexspi_nor_driver->clear_cache(instance);
}

struct imxrt106x_flash_priv_s
{
  mutex_t  lock;
  uint32_t ifbase;
  uint32_t base;
  uint32_t stblock;
  uint32_t stpage;
};

#if CONFIG_IMXRT_PROGMEM_FLEXSPI_INSTANCE == 0
static struct imxrt106x_flash_priv_s imxrt106x_flash_priv =
{
  .lock    = NXMUTEX_INITIALIZER,
  .ifbase  = IMXRT_FLEXSPIC_BASE,
  .base    = IMXRT_FLEXCIPHER_BASE,
  .stblock = 0,
  .stpage  = 0,
};
#else
static struct imxrt106x_flash_priv_s imxrt106x_flash_priv =
{
  .lock    = NXMUTEX_INITIALIZER,
  .ifbase  = IMXRT_FLEXSPI2C_BASE,
  .base    = IMXRT_FLEX2CIPHER_BASE,
  .stblock = 0,
  .stpage  = 0,
};
#endif

static flexspi_nor_config_t flash_config;
static serial_nor_config_option_t config_option;

static int imxrt106x_israngeerased(size_t startaddress, size_t size)
{
  uint32_t *addr;
  uint8_t *baddr;
  size_t count = 0;
  size_t bwritten = 0;

  addr = (uint32_t *)startaddress;
  while (count + 4 <= size)
    {
      if (getreg32(addr) != FLASH_ERASEDVALUE_DW)
        {
          bwritten++;
        }

      addr++;
      count += 4;
    }

  baddr = (uint8_t *)addr;
  while (count < size)
    {
      if (getreg8(baddr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }

      baddr++;
      count++;
    }

  return bwritten;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t up_progmem_erasesize(size_t block)
{
  return FLASH_SECTOR_SIZE;
}

size_t up_progmem_pagesize(size_t page)
{
  return up_progmem_erasesize(page);
}

ssize_t up_progmem_getpage(size_t addr)
{
  struct imxrt106x_flash_priv_s *priv = &imxrt106x_flash_priv;

  return  priv->stpage + ((addr - priv->base) / FLASH_SECTOR_SIZE);
}

size_t up_progmem_getaddress(size_t page)
{
  struct imxrt106x_flash_priv_s *priv = &imxrt106x_flash_priv;

  if (page >= PROGMEM_NBLOCKS)
    {
      return SIZE_MAX;
    }

  return priv->base + (page - priv->stpage) * FLASH_SECTOR_SIZE;
}

size_t up_progmem_neraseblocks(void)
{
  return PROGMEM_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= PROGMEM_NBLOCKS)
    {
      return -EFAULT;
    }

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }
    }

  return bwritten;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  struct imxrt106x_flash_priv_s *priv = &imxrt106x_flash_priv;
  int ret;
  status_t status;
  size_t block_address = (block * FLASH_SECTOR_SIZE);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  config_option.option0.U = 0xc0000008;

  up_irq_disable();
  up_disable_icache();
  up_disable_dcache();

  status = flexspi_nor_get_config(FLEXSPI_INSTANCE, &flash_config,
                                  &config_option);
  if (status != STATUS_SUCCESS)
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  status = flexspi_nor_flash_init(FLEXSPI_INSTANCE, &flash_config);
  if (status != STATUS_SUCCESS)
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  status = flexspi_nor_flash_erase(FLEXSPI_INSTANCE, &flash_config,
                                   block_address, FLASH_SECTOR_SIZE);
  if (status != STATUS_SUCCESS)
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  ret = 0;

exit_with_lock:
  nxmutex_unlock(&priv->lock);

  /* Verify */

  if (ret == 0 &&
      imxrt106x_israngeerased(priv->base + block_address,
                              up_progmem_erasesize(block)) == 0)
    {
      ret = up_progmem_erasesize(block); /* success */
    }
  else
    {
      ret = -EIO; /* failure */
    }

  up_enable_dcache();
  up_enable_icache();
  up_irq_enable();

  return ret;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  struct imxrt106x_flash_priv_s *priv = &imxrt106x_flash_priv;
  size_t written = count;
  uint32_t *ll = (uint32_t *)buf;
  size_t faddr;
  status_t status;
  int ret;
  const size_t pagesize = FLASH_PAGE_SIZE;
  const size_t llperpage  = pagesize / sizeof(uint32_t);
  size_t pcount = count / pagesize;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Check address and count alignment */

  DEBUGASSERT(!(addr % pagesize));
  DEBUGASSERT(!(count % pagesize));

  config_option.option0.U = 0xc0000008;

  up_irq_disable();
  up_disable_icache();

  status = flexspi_nor_get_config(FLEXSPI_INSTANCE, &flash_config,
                                  &config_option);
  if (status != STATUS_SUCCESS)
    {
      written = -EIO;
      goto exit_with_lock;
    }

  status = flexspi_nor_flash_init(FLEXSPI_INSTANCE, &flash_config);
  if (status != STATUS_SUCCESS)
    {
      written = -EIO;
      goto exit_with_lock;
    }

  for (ll = (uint32_t *)buf, faddr = addr - priv->base; pcount;
       pcount -= 1, ll += llperpage, faddr += pagesize)
    {
      status = flexspi_nor_flash_page_program(FLEXSPI_INSTANCE,
                                              &flash_config, faddr, ll);
      if (status != STATUS_SUCCESS)
        {
          written = -EIO;
          break;
        }
    }

exit_with_lock:
  nxmutex_unlock(&priv->lock);
  up_enable_icache();
  up_irq_enable();

  return written;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}
