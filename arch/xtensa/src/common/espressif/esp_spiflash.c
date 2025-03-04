/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_spiflash.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <inttypes.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include "esp_spiflash.h"
#include "esp_attr.h"
#include "memspi_host_driver.h"
#include "spi_flash_defs.h"
#include "hal/spimem_flash_ll.h"
#include "hal/spi_flash_ll.h"
#include "esp_rom_spiflash.h"
#include "esp32s2_irq.h"
#ifndef CONFIG_ARCH_CHIP_ESP32S3
#include "esp_private/spi_flash_os.h"
#endif

#ifdef CONFIG_ARCH_CHIP_ESP32S3
#define esp_intr_noniram_enable esp32s3_irq_noniram_disable
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ESPRESSIF_SPI_FLASH_USE_ROM_CODE
/* SPI buffer size */

#  define SPI_BUFFER_WORDS          (16)
#  define SPI_BUFFER_BYTES          (SPI_BUFFER_WORDS * 4)

/* SPI flash hardware definition */

#  define FLASH_SECTOR_SIZE         (4096)

/* SPI flash SR1 bits */

#  define FLASH_SR1_BUSY            ESP_ROM_SPIFLASH_BUSY_FLAG
#  define FLASH_SR1_WREN            ESP_ROM_SPIFLASH_WRENABLE_FLAG

/* SPI flash operation */

#  define FLASH_CMD_WRDI            CMD_WRDI
#  define FLASH_CMD_WREN            CMD_WREN
#  define FLASH_CMD_RDSR            CMD_RDSR
#ifdef CONFIG_ESPRESSIF_SPI_FLASH_USE_32BIT_ADDRESS
#  define ADDR_BITS(addr)         (((addr) & 0xff000000) ? 32 : 24)
#  define READ_CMD(addr)          (ADDR_BITS(addr) == 32 ?                  \
                                   CMD_FASTRD_4B : CMD_FASTRD)
#  define WRITE_CMD(addr)         (ADDR_BITS(addr) == 32 ? CMD_PROGRAM_PAGE_4B : \
                                   CMD_PROGRAM_PAGE)
#  define ERASE_CMD(addr)         (ADDR_BITS(addr) == 32 ? CMD_SECTOR_ERASE_4B : \
                                   CMD_SECTOR_ERASE)
#  define READ_DUMMY(addr)        (8)
#else
#  define ADDR_BITS(addr)         (24)
#  define READ_CMD(addr)          CMD_FASTRD
#  define WRITE_CMD(addr)         CMD_PROGRAM_PAGE
#  define ERASE_CMD(addr)         CMD_SECTOR_ERASE
#  define READ_DUMMY(addr)        (8)
#endif

#  define SEND_CMD8_TO_FLASH(cmd)                           \
    esp_spi_trans((cmd), 8,                                 \
                  0, 0,                                     \
                  NULL, 0,                                  \
                  NULL, 0,                                  \
                  0)

#  define READ_SR1_FROM_FLASH(cmd, status)                  \
    esp_spi_trans((cmd), 8,                                 \
                  0, 0,                                     \
                  NULL, 0,                                  \
                  (status), 1,                              \
                  0)

#  define ERASE_FLASH_SECTOR(addr)                          \
    esp_spi_trans(ERASE_CMD(addr), 8,                       \
                  (addr), ADDR_BITS(addr),                  \
                  NULL, 0,                                  \
                  NULL, 0,                                  \
                  0)

#  define WRITE_DATA_TO_FLASH(addr, buffer, size)           \
    esp_spi_trans(WRITE_CMD(addr), 8,                       \
                  (addr), ADDR_BITS(addr),                  \
                  buffer, size,                             \
                  NULL, 0,                                  \
                  0)

#  define READ_DATA_FROM_FLASH(addr, buffer, size)          \
    esp_spi_trans(READ_CMD(addr), 8,                        \
                  (addr), ADDR_BITS(addr),                  \
                  NULL, 0,                                  \
                  buffer, size,                             \
                  READ_DUMMY(addr))

/****************************************************************************
 * Private Types
 ****************************************************************************/

spi_mem_dev_t *dev = spimem_flash_ll_get_hw(SPI1_HOST);
#endif /* CONFIG_ESPRESSIF_SPI_FLASH_USE_ROM_CODE */

/****************************************************************************
 * Private Functions Declaration
 ****************************************************************************/

void spiflash_start(void);
void spiflash_end(void);
#if !CONFIG_ESPRESSIF_SPI_FLASH_USE_ROM_CODE && CONFIG_ARCH_CHIP_ESP32S3
extern bool spi_flash_check_and_flush_cache(size_t start_addr,
                                            size_t length);
#endif /* CONFIG_ESPRESSIF_SPI_FLASH_USE_ROM_CODE */

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

static mutex_t s_flash_op_mutex;
static uint32_t s_flash_op_cache_state[CONFIG_ESPRESSIF_NUM_CPUS];
static volatile bool s_sched_suspended[CONFIG_ESPRESSIF_NUM_CPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiflash_start
 *
 * Description:
 *   Prepare for an SPIFLASH operation.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void spiflash_start(void)
{
  extern uint32_t cache_suspend_icache(void);
  int cpu;
  irqstate_t flags;
  uint32_t regval;

  nxmutex_lock(&s_flash_op_mutex);
  flags = enter_critical_section();
  cpu = this_cpu();
  s_sched_suspended[cpu] = true;

#ifndef CONFIG_ARCH_CHIP_ESP32S2
  esp_intr_noniram_disable();
#endif

  s_flash_op_cache_state[cpu] = cache_suspend_icache() << 16;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: spiflash_end
 *
 * Description:
 *   Undo all the steps of opstart.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void spiflash_end(void)
{
  extern void cache_resume_icache(uint32_t);
  extern void cache_invalidate_icache_all(void);

  int cpu;
  irqstate_t flags;

  flags = enter_critical_section();

  cpu = this_cpu();

  cache_invalidate_icache_all();
  cache_resume_icache(s_flash_op_cache_state[cpu] >> 16);

#ifndef CONFIG_ARCH_CHIP_ESP32S2
  esp_intr_noniram_enable();
#endif
  s_sched_suspended[cpu] = false;

  leave_critical_section(flags);
  nxmutex_unlock(&s_flash_op_mutex);
}

/****************************************************************************
 * Name: esp_spi_trans
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
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

#ifndef CONFIG_ESPRESSIF_SPI_FLASH_USE_ROM_CODE
static IRAM_ATTR void esp_spi_trans(uint32_t command,
                                    uint32_t command_bits,
                                    uint32_t address,
                                    uint32_t address_bits,
                                    uint32_t *tx_buffer,
                                    uint32_t tx_bytes,
                                    uint32_t *rx_buffer,
                                    uint32_t rx_bytes,
                                    uint32_t dummy_bits)
{
  /* Initiliaze SPI user register */

  spi_flash_ll_reset(dev);

  while (!spi_flash_ll_host_idle(dev));

  /* Set command bits and value, and command is always needed */

  spi_flash_ll_set_command(dev, command, command_bits);

  /* Set address bits and value */

  if (address_bits)
    {
      spi_flash_ll_set_addr_bitlen(dev, address_bits);
      spi_flash_ll_set_address(dev, address);
    }

  /* Set dummy */

  if (dummy_bits)
    {
      spi_flash_ll_set_dummy(dev, dummy_bits);
    }

  /* Set TX data */

  if (tx_bytes)
    {
      spi_flash_ll_set_mosi_bitlen(dev, tx_bytes * 8);
      spi_flash_ll_set_buffer_data(dev, tx_buffer, tx_bytes);
    }

  /* Set RX data */

  if (rx_bytes)
    {
      spi_flash_ll_set_miso_bitlen(dev, rx_bytes * 8);
    }

  /* Set I/O mode */

  spi_flash_ll_set_read_mode(dev, SPI_FLASH_FASTRD);

  /* Set clock and delay */

  spimem_flash_ll_suspend_cmd_setup(dev, 0);

  /* Start transmision */

  spi_flash_ll_user_start(dev);

  /* Wait until transmission is done */

  while (!spi_flash_ll_cmd_is_done(dev));

  /* Get read data */

  if (rx_bytes)
    {
      spi_flash_ll_get_buffer_data(dev, rx_buffer, rx_bytes);
    }
}

/****************************************************************************
 * Name: wait_flash_idle
 *
 * Description:
 *   Wait until flash enters idle state
 *
 * Input Parameters:
 *   None.
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
 * Input Parameters:
 *   None.
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
 * Input Parameters:
 *   None.
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

/****************************************************************************
 * Name: spi_flash_read
 *
 * Description:
 *   Read data from Flash.
 *
 * Parameters:
 *   address - source address of the data in Flash.
 *   buffer  - pointer to the destination buffer
 *   length  - length of data
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

IRAM_ATTR int spi_flash_read(uint32_t address, void *buffer, uint32_t length)
{
  int ret = OK;
  uint8_t *rx_buf = (uint8_t *)buffer;
  uint32_t rx_bytes = length;
  uint32_t rx_addr = address;

  spiflash_start();

  for (uint32_t i = 0; i < length; i += SPI_BUFFER_BYTES)
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
#ifdef CONFIG_ARCH_CHIP_ESP32S3
  spi_flash_check_and_flush_cache(start_address, size);
#endif

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
 *   buffer    - Pointer to the source buffer.
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
#ifdef CONFIG_ARCH_CHIP_ESP32S3
  spi_flash_check_and_flush_cache(start_address, size);
#endif

  spiflash_end();

  return ret;
}
#endif /* CONFIG_ESPRESSIF_SPI_FLASH_USE_ROM_CODE */

/****************************************************************************
 * Name: esp_spiflash_init
 *
 * Description:
 *   Initialize ESP SPI flash driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_spiflash_init(void)
{
#ifdef CONFIG_ARCH_CHIP_ESP32S3
  extern void spi_flash_guard_set(const struct spiflash_guard_funcs *);
#endif

  nxmutex_init(&s_flash_op_mutex);

  spi_flash_guard_set((spi_flash_guard_funcs_t *)&g_spi_flash_guard_funcs);

  return OK;
}
