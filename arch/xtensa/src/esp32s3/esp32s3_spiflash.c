/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiflash.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <sys/types.h>
#include <sys/param.h>
#include <inttypes.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/esp32s3_spi_mem_reg.h"
#include "rom/esp32s3_spiflash.h"
#include "rom/esp32s3_opi_flash.h"
#include "esp32s3_irq.h"
#include "esp32s3_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RO data page in MMU index */

#define DROM0_PAGES_START           (2)
#define DROM0_PAGES_END             (128)

/* MMU invalid value */

#define INVALID_MMU_VAL             (0x100)

/* MMU page size */

#define SPI_FLASH_MMU_PAGE_SIZE     (0x10000)

/* MMU base virtual mapped address */

#define VADDR0_START_ADDR           (0x3c020000)

/* Flash MMU table for CPU */

#define MMU_TABLE                   ((volatile uint32_t *)DR_REG_MMU_TABLE)

#define MMU_ADDR2PAGE(_addr)        ((_addr) / SPI_FLASH_MMU_PAGE_SIZE)
#define MMU_ADDR2OFF(_addr)         ((_addr) % SPI_FLASH_MMU_PAGE_SIZE)
#define MMU_BYTES2PAGES(_n)         (((_n) + SPI_FLASH_MMU_PAGE_SIZE - 1) / \
                                     SPI_FLASH_MMU_PAGE_SIZE)

#ifdef CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE

/* SPI port number */

#  define SPI_PORT                  (1)

/* SPI buffer size */

#  define SPI_BUFFER_WORDS          (16)
#  define SPI_BUFFER_BYTES          (SPI_BUFFER_WORDS * 4)

/* SPI flash hardware definition */

#  define FLASH_SECTOR_SIZE         (4096)

/* SPI flash command */

#  define FLASH_CMD_WRDI            ROM_FLASH_CMD_WRDI
#  define FLASH_CMD_WREN            ROM_FLASH_CMD_WREN
#  define FLASH_CMD_RDSR            ROM_FLASH_CMD_RDSR
#  define FLASH_CMD_SE4B            ROM_FLASH_CMD_SE4B
#  define FLASH_CMD_SE              ROM_FLASH_CMD_ERASE_SEC
#  define FLASH_CMD_PP4B            ROM_FLASH_CMD_PP4B
#  define FLASH_CMD_PP              0x02
#  define FLASH_CMD_FSTRD4B         ROM_FLASH_CMD_FSTRD4B
#  define FLASH_CMD_FSTRD           0x0B

/* SPI flash SR1 bits */

#  define FLASH_SR1_BUSY            ESP_ROM_SPIFLASH_BUSY_FLAG
#  define FLASH_SR1_WREN            ESP_ROM_SPIFLASH_WRENABLE_FLAG

/* SPI flash operation */

#  ifdef CONFIG_ESP32S3S_SPI_FLASH_USE_32BIT_ADDRESS
#    define ADDR_BITS(addr)         (((addr) & 0xff000000) ? 32 : 24)
#    define READ_CMD(addr)          (ADDR_BITS(addr) == 32 ? FLASH_CMD_FSTRD4B : \
                                                             FLASH_CMD_FSTRD)
#    define WRITE_CMD(addr)         (ADDR_BITS(addr) == 32 ? FLASH_CMD_PP4B : \
                                                             FLASH_CMD_PP)
#    define ERASE_CMD(addr)         (ADDR_BITS(addr) == 32 ? FLASH_CMD_SE4B : \
                                                             FLASH_CMD_SE)
#    define READ_DUMMY(addr)        (8)
#  else
#    define ADDR_BITS(addr)         24
#    define READ_CMD(addr)          FLASH_CMD_FSTRD
#    define WRITE_CMD(addr)         FLASH_CMD_PP
#    define ERASE_CMD(addr)         FLASH_CMD_SE
#    define READ_DUMMY(addr)        (8)
#  endif

#  define SEND_CMD8_TO_FLASH(cmd)                           \
    esp32s3_spi_trans((cmd), 8,                             \
                      0, 0,                                 \
                      NULL, 0,                              \
                      NULL, 0,                              \
                      0,                                    \
                      false)

#  define READ_SR1_FROM_FLASH(cmd, status)                  \
    esp32s3_spi_trans((cmd), 8,                             \
                      0, 0,                                 \
                      NULL, 0,                              \
                      (status), 1,                          \
                      0,                                    \
                      false)

#  define ERASE_FLASH_SECTOR(addr)                          \
    esp32s3_spi_trans(ERASE_CMD(addr), 8,                   \
                      (addr), ADDR_BITS(addr),              \
                      NULL, 0,                              \
                      NULL, 0,                              \
                      0,                                    \
                      true)

#  define WRITE_DATA_TO_FLASH(addr, buffer, size)           \
    esp32s3_spi_trans(WRITE_CMD(addr), 8,                   \
                      (addr), ADDR_BITS(addr),              \
                      buffer, size,                         \
                      NULL, 0,                              \
                      0,                                    \
                      true) 

#  define READ_DATA_FROM_FLASH(addr, buffer, size)          \
    esp32s3_spi_trans(READ_CMD(addr), 8,                    \
                      (addr), ADDR_BITS(addr),              \
                      NULL, 0,                              \
                      buffer, size,                         \
                      READ_DUMMY(addr),                     \
                      false) 

#endif /* CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Flash map request data */

struct spiflash_map_req_s
{
  /* Request mapping SPI Flash base address */

  uint32_t  src_addr;

  /* Request mapping SPI Flash size */

  uint32_t  size;

  /* Mapped memory pointer */

  void      *ptr;

  /* Mapped started MMU page index */

  uint32_t  start_page;

  /* Mapped MMU page count */

  uint32_t  page_cnt;
};

struct spiflash_cachestate_s
{
  uint32_t value;
  irqstate_t flags;
  int cpu;
#ifdef CONFIG_SMP
  int other;
#endif
};

/****************************************************************************
 * Private Functions Declaration
 ****************************************************************************/

static void spiflash_start(void);
static void spiflash_end(void);

/****************************************************************************
 * Public Functions Declaration
 ****************************************************************************/

extern uint32_t cache_suspend_icache(void);
extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_resume_dcache(uint32_t val);
extern int cache_invalidate_addr(uint32_t addr, uint32_t size);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spiflash_guard_funcs g_spi_flash_guard_funcs =
{
  .start           = spiflash_start,
  .end             = spiflash_end,
  .op_lock         = NULL,
  .op_unlock       = NULL,
  .address_is_safe = NULL,
  .yield           = NULL,
};

static struct spiflash_cachestate_s g_state;

/****************************************************************************
 * Name: spiflash_opstart
 *
 * Description:
 *   Prepare for an SPIFLASH operation.
 *
 ****************************************************************************/

static IRAM_ATTR void spiflash_start(void)
{
  g_state.flags = enter_critical_section();
  g_state.cpu = up_cpu_index();
#ifdef CONFIG_SMP
  g_state.other = g_state.cpu ? 0 : 1;
#endif

  DEBUGASSERT(g_state.cpu == 0 || g_state.cpu == 1);
#ifdef CONFIG_SMP
  DEBUGASSERT(g_state.other == 0 || g_state.other == 1);
  DEBUGASSERT(g_state.other != g_state.cpu);
  up_cpu_pause(g_state.other);
#endif

  g_state.value = cache_suspend_icache() << 16;
  g_state.value |= cache_suspend_dcache();
}

/****************************************************************************
 * Name: spiflash_opdone
 *
 * Description:
 *   Undo all the steps of opstart.
 *
 ****************************************************************************/

static IRAM_ATTR void spiflash_end(void)
{
  DEBUGASSERT(g_state.cpu == 0 || g_state.cpu == 1);

#ifdef CONFIG_SMP
  DEBUGASSERT(g_state.other == 0 || g_state.other == 1);
  DEBUGASSERT(g_state.other != g_state.cpu);
#endif

  cache_resume_icache(g_state.value >> 16);
  cache_resume_dcache(g_state.value & 0xffff);

#ifdef CONFIG_SMP
  up_cpu_resume(g_state.other);
#endif

  leave_critical_section(g_state.flags);
}

/****************************************************************************
 * Name: esp32s3_spi_trans
 *
 * Description:
 *   Transmit given command, address and data.
 *
 * Input Parameters:
 *   command      - command value
 *   command_bits - command bits
 *   address      - address value
 *   address_bits - address bits
 *   tx_buffer    - write buffer
 *   tx_bytes     - write buffer size
 *   rx_buffer    - read buffer
 *   rx_bytes     - read buffer size
 *   dummy_bits   - dummy bits
 *   is_program   - true if operation is program or erase
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE
static IRAM_ATTR void esp32s3_spi_trans(uint32_t command,
                                        uint32_t command_bits,
                                        uint32_t address,
                                        uint32_t address_bits,
                                        uint32_t *tx_buffer,
                                        uint32_t tx_bytes,
                                        uint32_t *rx_buffer,
                                        uint32_t rx_bytes,
                                        uint32_t dummy_bits,
                                        bool is_program)
{
  uint32_t regval;
  uint32_t cmd_reg;
  uint32_t user1_reg = getreg32(SPI_MEM_USER1_REG(SPI_PORT));
  uint32_t user_reg = getreg32(SPI_MEM_USER_REG(SPI_PORT));

  /* Initiliaze SPI user register */

  user_reg &= ~(SPI_MEM_USR_ADDR_M | SPI_MEM_USR_DUMMY_M |
                SPI_MEM_USR_MOSI_M | SPI_MEM_USR_MISO_M);
  user_reg |= SPI_MEM_USR_COMMAND_M;

  /* Wait until SPI is idle */

  do
    {
      cmd_reg = getreg32(SPI_MEM_CMD_REG(SPI_PORT));
    }
  while ((cmd_reg & SPI_MEM_USR_M) != 0);

  /* Set command bits and value, and command is always needed */

  regval  = getreg32(SPI_MEM_USER2_REG(SPI_PORT));
  regval &= ~(SPI_MEM_USR_COMMAND_BITLEN_M | SPI_MEM_USR_COMMAND_VALUE_M);
  regval |= ((command_bits - 1) << SPI_MEM_USR_COMMAND_BITLEN_S) |
            (command << SPI_MEM_USR_COMMAND_VALUE_S);
  putreg32(regval, SPI_MEM_USER2_REG(SPI_PORT));

  /* Set address bits and value */

  if (address_bits)
    {
      user1_reg &= ~SPI_MEM_USR_ADDR_BITLEN_M;
      user1_reg |= (address_bits - 1) << SPI_MEM_USR_ADDR_BITLEN_S;

      putreg32(address, SPI_MEM_ADDR_REG(SPI_PORT));

      user_reg |= SPI_MEM_USR_ADDR_M;

      regval  = getreg32(SPI_MEM_CACHE_FCTRL_REG(SPI_PORT));
      if (address_bits > 24)
        {
          regval |= SPI_MEM_CACHE_USR_CMD_4BYTE_M;
        }
      else
        {
          regval &= ~SPI_MEM_CACHE_USR_CMD_4BYTE_M;
        }

      putreg32(regval, SPI_MEM_CACHE_FCTRL_REG(SPI_PORT));
    }

  /* Set dummy */

  if (dummy_bits)
    {
      user1_reg &= ~SPI_MEM_USR_DUMMY_CYCLELEN_M;
      user1_reg |= (dummy_bits - 1) << SPI_MEM_USR_DUMMY_CYCLELEN_S;

      user_reg |= SPI_MEM_USR_DUMMY_M;
    }

  /* Set TX data */

  if (tx_bytes)
    {
      putreg32(tx_bytes * 8 - 1, SPI_MEM_MOSI_DLEN_REG(SPI_PORT));
      for (uint32_t i = 0; i < tx_bytes; i += 4)
        {
          putreg32(tx_buffer[i / 4], SPI_MEM_W0_REG(SPI_PORT) + i);
        }

      user_reg |= SPI_MEM_USR_MOSI_M;
    }

  /* Set RX data */

  if (rx_bytes)
    {
      putreg32(rx_bytes * 8 - 1, SPI_MEM_MISO_DLEN_REG(SPI_PORT));

      user_reg |= SPI_MEM_USR_MISO_M;
    }

  putreg32(user_reg,  SPI_MEM_USER_REG(SPI_PORT));
  putreg32(user1_reg, SPI_MEM_USER1_REG(SPI_PORT));

  /* Set I/O mode */

  regval = getreg32(SPI_MEM_CTRL_REG(SPI_PORT));
  regval &= ~(SPI_MEM_FREAD_QIO_M | SPI_MEM_FREAD_DIO_M |
              SPI_MEM_FREAD_QUAD_M | SPI_MEM_FREAD_DUAL_M |
              SPI_MEM_FCMD_OCT_M | SPI_MEM_FCMD_QUAD_M |
              SPI_MEM_FCMD_DUAL_M | SPI_MEM_FADDR_OCT_M |
              SPI_MEM_FDIN_OCT_M | SPI_MEM_FDOUT_OCT_M |
              SPI_MEM_FDUMMY_OUT_M | SPI_MEM_RESANDRES_M |
              SPI_MEM_WP_REG_M | SPI_MEM_WRSR_2B_M);
  regval |= SPI_MEM_FASTRD_MODE_M;
  putreg32(regval, SPI_MEM_CTRL_REG(SPI_PORT));

  /* Set clock and delay */

  regval = SPI_MEM_FLASH_PES_WAIT_EN_M |
           SPI_MEM_FLASH_PER_WAIT_EN_M;
  putreg32(regval, SPI_MEM_FLASH_SUS_CMD_REG(SPI_PORT));
  putreg32(0, SPI_MEM_CLOCK_GATE_REG(SPI_PORT));

  /* Set if this is program or erase operation */

  if (is_program)
    {
      cmd_reg |= SPI_MEM_FLASH_PE_M;
    }

  /* Start transmision */

  cmd_reg |= SPI_MEM_USR_M;
  putreg32(cmd_reg, SPI_MEM_CMD_REG(SPI_PORT));

  /* Wait until transmission is done */

  while ((getreg32(SPI_MEM_CMD_REG(SPI_PORT)) & SPI_MEM_USR_M) != 0)
    {
      ;
    }

  /* Get read data */

  if (rx_bytes)
    {
      for (uint32_t i = 0; i < rx_bytes; i += 4)
        {
          rx_buffer[i / 4] = getreg32(SPI_MEM_W0_REG(SPI_PORT) + i);
        }
    }
}

/****************************************************************************
 * Name: wait_flash_idle
 *
 * Description:
 *   Wait until flash enters idle state
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void wait_flash_idle(void)
{
  uint32_t status;

  do
    {
      READ_SR1_FROM_FLASH(FLASH_CMD_RDSR, &status);
      if ((status & FLASH_SR1_BUSY) == 0)
        {
          break;
        }
    }
  while (1);
}

/****************************************************************************
 * Name: enable_flash_write
 *
 * Description:
 *   Enable Flash write mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void enable_flash_write(void)
{
  uint32_t status;

  do
    {
      SEND_CMD8_TO_FLASH(FLASH_CMD_WREN);
      READ_SR1_FROM_FLASH(FLASH_CMD_RDSR, &status);
      if ((status & FLASH_SR1_WREN) != 0)
        {
          break;
        }
    }
  while (1);
}

/****************************************************************************
 * Name: disable_flash_write
 *
 * Description:
 *   Disable Flash write mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void disable_flash_write(void)
{
  uint32_t status;

  do
    {
      SEND_CMD8_TO_FLASH(FLASH_CMD_WRDI);
      READ_SR1_FROM_FLASH(FLASH_CMD_RDSR, &status);
      if ((status & FLASH_SR1_WREN) == 0)
        {
          break;
        }
    }
  while (1);
}
#endif /* CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE */

/****************************************************************************
 * Name: esp32s3_mmap
 *
 * Description:
 *   Mapped SPI Flash address to ESP32-S3's address bus, so that software
 *   can read SPI Flash data by reading data from memory access.
 *
 *   If SPI Flash hardware encryption is enable, the read from mapped
 *   address is decrypted.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static IRAM_ATTR int esp32s3_mmap(struct spiflash_map_req_s *req)
{
  int ret;
  int i;
  int start_page;
  int flash_page;
  int page_cnt;
  uint32_t mapped_addr;

  spiflash_start();

  for (start_page = DROM0_PAGES_START;
       start_page < DROM0_PAGES_END;
       ++start_page)
    {
      if (MMU_TABLE[start_page] == INVALID_MMU_VAL)
        {
          break;
        }
    }

  flash_page = MMU_ADDR2PAGE(req->src_addr);
  page_cnt   = MMU_BYTES2PAGES(MMU_ADDR2OFF(req->src_addr) + req->size);

  if (start_page + page_cnt < DROM0_PAGES_END)
    {
      mapped_addr = (start_page - DROM0_PAGES_START) *
                    SPI_FLASH_MMU_PAGE_SIZE +
                    VADDR0_START_ADDR;

      for (i = 0; i < page_cnt; i++)
        {
          MMU_TABLE[start_page + i] = flash_page + i;
          cache_invalidate_addr(mapped_addr + i * SPI_FLASH_MMU_PAGE_SIZE,
                                SPI_FLASH_MMU_PAGE_SIZE);
        }

      req->start_page = start_page;
      req->page_cnt = page_cnt;
      req->ptr = (void *)(mapped_addr + MMU_ADDR2OFF(req->src_addr));
      ret = OK;
    }
  else
    {
      ret = -ENOBUFS;
    }

  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: esp32s3_ummap
 *
 * Description:
 *   Unmap SPI Flash address in ESP32-S3's address bus, and free resource.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void esp32s3_ummap(const struct spiflash_map_req_s *req)
{
  int i;

  spiflash_start();

  for (i = req->start_page; i < req->start_page + req->page_cnt; ++i)
    {
      MMU_TABLE[i] = INVALID_MMU_VAL;
    }

  spiflash_end();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_flash_read_encrypted
 *
 * Description:
 *   Read decrypted data from SPI Flash at designated address when
 *   enable SPI Flash hardware encryption.
 *
 * Input Parameters:
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int spi_flash_read_encrypted(uint32_t addr, void *buffer, uint32_t size)
{
  int ret;
  struct spiflash_map_req_s req =
    {
      .src_addr = addr,
      .size = size
    };

  ret = esp32s3_mmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, req.ptr, size);

  esp32s3_ummap(&req);

  return OK;
}

/****************************************************************************
 * Name: spi_flash_erase_sector
 *
 * Description:
 *   Erase the Flash sector.
 *
 * Parameters:
 *   sector - Sector number, the count starts at sector 0, 4KB per sector.
 *
 * Returned Values: esp_err_t
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE
IRAM_ATTR int spi_flash_erase_sector(uint32_t sector)
{
  int ret = OK;
  uint32_t addr = sector * FLASH_SECTOR_SIZE;

  spiflash_start();

  wait_flash_idle();
  enable_flash_write();

  ERASE_FLASH_SECTOR(addr);

  wait_flash_idle();
  disable_flash_write();

  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: spi_flash_erase_range
 *
 * Description:
 *   Erase a range of flash sectors
 *
 * Parameters:
 *   start_address - Address where erase operation has to start.
 *                   Must be 4kB-aligned
 *   size          - Size of erased range, in bytes. Must be divisible by
 *                   4kB.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

IRAM_ATTR int spi_flash_erase_range(uint32_t start_address, uint32_t size)
{
  int ret = OK;
  uint32_t addr = start_address;

  spiflash_start();

  for (uint32_t i = 0; i < size; i += FLASH_SECTOR_SIZE)
    {
      wait_flash_idle();
      enable_flash_write();

      ERASE_FLASH_SECTOR(addr);
      addr += FLASH_SECTOR_SIZE;
    }

  wait_flash_idle();
  disable_flash_write();

  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: spi_flash_write
 *
 * Description:
 *   Write data to Flash.
 *
 * Parameters:
 *   dest_addr - Destination address in Flash.
 *   src       - Pointer to the source buffer.
 *   size      - Length of data, in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

IRAM_ATTR int spi_flash_write(uint32_t dest_addr,
                              const void *buffer,
                              uint32_t size)
{
  int ret = OK;
  const uint8_t *tx_buf = (const uint8_t *)buffer;
  uint32_t tx_bytes = size;
  uint32_t tx_addr = dest_addr;

  spiflash_start();

  for (int i = 0; i < size; i += SPI_BUFFER_BYTES)
    {
      uint32_t spi_buffer[SPI_BUFFER_WORDS];
      uint32_t n = MIN(tx_bytes, SPI_BUFFER_BYTES);

      memcpy(spi_buffer, tx_buf, n);

      wait_flash_idle();
      enable_flash_write();

      WRITE_DATA_TO_FLASH(tx_addr, spi_buffer, n);

      tx_bytes -= n;
      tx_buf += n;
      tx_addr += n;
    }

  wait_flash_idle();
  disable_flash_write();

  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: spi_flash_read
 *
 * Description:
 *   Read data from Flash.
 *
 * Parameters:
 *   src_addr - source address of the data in Flash.
 *   dest     - pointer to the destination buffer
 *   size     - length of data
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

IRAM_ATTR int spi_flash_read(uint32_t src_addr, void *dest, uint32_t size)
{
  int ret = OK;
  uint8_t *rx_buf = (uint8_t *)dest;
  uint32_t rx_bytes = size;
  uint32_t rx_addr = src_addr;

  spiflash_start();

  for (uint32_t i = 0; i < size; i += SPI_BUFFER_BYTES)
    {
      uint32_t spi_buffer[SPI_BUFFER_WORDS];
      uint32_t n = MIN(rx_bytes, SPI_BUFFER_BYTES);

      READ_DATA_FROM_FLASH(rx_addr, spi_buffer, n);

      memcpy(rx_buf, spi_buffer, n);
      rx_bytes -= n;
      rx_buf += n;
      rx_addr += n;
    }

  spiflash_end();

  return ret;
}
#endif /* CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE */

/****************************************************************************
 * Name: esp32s3_spiflash_init
 *
 * Description:
 *   Initialize ESP32-S3 SPI flash driver.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_spiflash_init(void)
{
  spi_flash_guard_set(&g_spi_flash_guard_funcs);

  return OK;
}
