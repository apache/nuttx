/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spiflash.c
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

#ifdef CONFIG_ESP32_SPIFLASH

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/mtd.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "rom/esp32_spiflash.h"

#include "hardware/esp32_soc.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_dport.h"

#ifdef CONFIG_ESP32_SPIRAM
#include "esp32_spiram.h"
#endif

#include "esp32_spicache.h"
#include "esp32_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Used in spiflash_cachestate_s structure even when SMP is disabled. */

#define SPI_FLASH_WRITE_BUF_SIZE    (32)
#define SPI_FLASH_READ_BUF_SIZE     (64)

#define SPI_FLASH_WRITE_WORDS       (SPI_FLASH_WRITE_BUF_SIZE / 4)
#define SPI_FLASH_READ_WORDS        (SPI_FLASH_READ_BUF_SIZE / 4)

#define SPI_FLASH_MMU_PAGE_SIZE     (0x10000)

#define SPI_FLASH_ENCRYPT_UNIT_SIZE (32)
#define SPI_FLASH_ENCRYPT_WORDS     (32 / 4)
#define SPI_FLASH_ERASED_STATE      (0xff)

#define MTD2PRIV(_dev)              ((struct esp32_spiflash_s *)_dev)
#define MTD_SIZE(_priv)             ((_priv)->chip->chip_size)
#define MTD_BLKSIZE(_priv)          ((_priv)->chip->page_size)
#define MTD_ERASESIZE(_priv)        ((_priv)->chip->sector_size)
#define MTD_BLK2SIZE(_priv, _b)     (MTD_BLKSIZE(_priv) * (_b))
#define MTD_SIZE2BLK(_priv, _s)     ((_s) / MTD_BLKSIZE(_priv))

#define MMU_ADDR2PAGE(_addr)        ((_addr) / SPI_FLASH_MMU_PAGE_SIZE)
#define MMU_ADDR2OFF(_addr)         ((_addr) % SPI_FLASH_MMU_PAGE_SIZE)
#define MMU_BYTES2PAGES(_n)         (((_n) + SPI_FLASH_MMU_PAGE_SIZE - 1) \
                                     / SPI_FLASH_MMU_PAGE_SIZE)
#define MMU_ALIGNUP_SIZE(_s)        (((_s) + SPI_FLASH_MMU_PAGE_SIZE - 1) \
                                     & ~(SPI_FLASH_MMU_PAGE_SIZE - 1))
#define MMU_ALIGNDOWN_SIZE(_s)      ((_s) & ~(SPI_FLASH_MMU_PAGE_SIZE - 1))

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/* Flash MMU table for PRO CPU */

#define PRO_MMU_TABLE ((volatile uint32_t *)DPORT_PRO_FLASH_MMU_TABLE_REG)

/* Flash MMU table for APP CPU */

#define APP_MMU_TABLE ((volatile uint32_t *)DPORT_APP_FLASH_MMU_TABLE_REG)

#define PRO_IRAM0_FIRST_PAGE  ((SOC_IRAM_LOW - SOC_DRAM_HIGH) /\
                               (SPI_FLASH_MMU_PAGE_SIZE + IROM0_PAGES_START))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Flash device hardware configuration */

struct esp32_spiflash_config_s
{
  /* SPI register base address */

  uint32_t reg_base;
};

/* SPI Flash device private data  */

struct esp32_spiflash_s
{
  struct mtd_dev_s mtd;

  /* Port configuration */

  struct esp32_spiflash_config_s *config;

  /* SPI Flash data */

  esp32_spiflash_chip_t *chip;

  /* SPI Flash communication dummy number */

  uint8_t *dummies;
};

/* SPI Flash map request data */

struct spiflash_map_req
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
  int cpu;
#ifdef CONFIG_SMP
  int other;
#endif
  irqstate_t flags;
  uint32_t val[CONFIG_SMP_NCPUS];
};

/****************************************************************************
 * ROM function prototypes
 ****************************************************************************/

extern void cache_flush(int cpu);

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* SPI helpers */

static inline void spi_set_reg(struct esp32_spiflash_s *priv,
                               int offset, uint32_t value);
static inline uint32_t spi_get_reg(struct esp32_spiflash_s *priv,
                                   int offset);
static inline void spi_set_regbits(struct esp32_spiflash_s *priv,
                                   int offset, uint32_t bits);
static inline void spi_reset_regbits(struct esp32_spiflash_s *priv,
                                     int offset, uint32_t bits);

/* Misc. helpers */

static inline void IRAM_ATTR
esp32_spiflash_opstart(struct spiflash_cachestate_s *state);
static inline void IRAM_ATTR
esp32_spiflash_opdone(const struct spiflash_cachestate_s *state);

static bool IRAM_ATTR spiflash_pagecached(uint32_t phypage);
static void IRAM_ATTR spiflash_flushmapped(size_t start, size_t size);

/* Flash helpers */

static void IRAM_ATTR esp32_set_read_opt(struct esp32_spiflash_s *priv);
static void IRAM_ATTR esp32_set_write_opt(struct esp32_spiflash_s *priv);
static int  IRAM_ATTR  esp32_read_status(struct esp32_spiflash_s *priv,
                                         uint32_t *status);
static int IRAM_ATTR esp32_wait_idle(struct esp32_spiflash_s *priv);
static int IRAM_ATTR esp32_enable_write(struct esp32_spiflash_s *priv);
static int IRAM_ATTR esp32_erasesector(struct esp32_spiflash_s *priv,
                                       uint32_t addr, uint32_t size);
static int IRAM_ATTR esp32_writedata(struct esp32_spiflash_s *priv,
                                     uint32_t addr,
                                     const uint8_t *buffer, uint32_t size);
static int IRAM_ATTR esp32_readdata(struct esp32_spiflash_s *priv,
                                    uint32_t addr,
                                    uint8_t *buffer, uint32_t size);
#if 0
static int esp32_read_highstatus(struct esp32_spiflash_s *priv,
                                 uint32_t *status);
#endif
#if 0
static int esp32_write_status(struct esp32_spiflash_s *priv,
                              uint32_t status);
#endif

/* MTD driver methods */

static int esp32_erase(struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks);
static ssize_t esp32_read(struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, uint8_t *buffer);
static ssize_t esp32_read_decrypt(struct mtd_dev_s *dev,
                                  off_t offset,
                                  size_t nbytes,
                                  uint8_t *buffer);
static ssize_t esp32_bread(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, uint8_t *buffer);
static ssize_t esp32_bread_decrypt(struct mtd_dev_s *dev,
                                   off_t startblock,
                                   size_t nblocks,
                                   uint8_t *buffer);
static ssize_t esp32_write(struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, const uint8_t *buffer);
static ssize_t esp32_bwrite(struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, const uint8_t *buffer);
static ssize_t esp32_bwrite_encrypt(struct mtd_dev_s *dev,
                                    off_t startblock,
                                    size_t nblocks,
                                    const uint8_t *buffer);
static int esp32_ioctl(struct mtd_dev_s *dev, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32_spiflash_config_s g_esp32_spiflash1_config =
{
  .reg_base = REG_SPI_BASE(1)
};

static struct esp32_spiflash_s g_esp32_spiflash1 =
{
  .mtd =
          {
            .erase  = esp32_erase,
            .bread  = esp32_bread,
            .bwrite = esp32_bwrite,
            .read   = esp32_read,
            .ioctl  = esp32_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = esp32_write,
#endif
            .name   = "esp32_mainflash"
          },
  .config = &g_esp32_spiflash1_config,
  .chip = &g_rom_flashchip,
  .dummies = g_rom_spiflash_dummy_len_plus
};

static struct esp32_spiflash_s g_esp32_spiflash1_encrypt =
{
  .mtd =
          {
            .erase  = esp32_erase,
            .bread  = esp32_bread_decrypt,
            .bwrite = esp32_bwrite_encrypt,
            .read   = esp32_read_decrypt,
            .ioctl  = esp32_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = NULL,
#endif
            .name   = "esp32_mainflash_encrypt"
          },
  .config = &g_esp32_spiflash1_config,
  .chip = &g_rom_flashchip,
  .dummies = g_rom_spiflash_dummy_len_plus
};

/* Enxusre exculisve access to the driver */

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_set_reg
 *
 * Description:
 *   Set the content of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   value  - Value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_set_reg(struct esp32_spiflash_s *priv,
                               int offset, uint32_t value)
{
  putreg32(value, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: spi_get_reg
 *
 * Description:
 *   Get the content of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *
 * Returned Value:
 *   The content of the register
 *
 ****************************************************************************/

static inline uint32_t spi_get_reg(struct esp32_spiflash_s *priv,
                                   int offset)
{
  return getreg32(priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: spi_set_regbits
 *
 * Description:
 *   Set the bits of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   bits   - Bits to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void IRAM_ATTR spi_set_regbits(struct esp32_spiflash_s *priv,
                                             int offset, uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp | bits, priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: spi_reset_regbits
 *
 * Description:
 *   Clear the bits of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   offset - Offset to the register of interest
 *   bits   - Bits to be cleared
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_reset_regbits(struct esp32_spiflash_s *priv,
                                     int offset, uint32_t bits)
{
  uint32_t tmp = getreg32(priv->config->reg_base + offset);

  putreg32(tmp & (~bits), priv->config->reg_base + offset);
}

/****************************************************************************
 * Name: esp32_spiflash_opstart
 *
 * Description:
 *   Prepare for an SPIFLASH operartion.
 *
 ****************************************************************************/

static inline void IRAM_ATTR
esp32_spiflash_opstart(struct spiflash_cachestate_s *state)
{
  state->flags = enter_critical_section();

  state->cpu = up_cpu_index();
#ifdef CONFIG_SMP
  state->other = state->cpu ? 0 : 1;
#endif

  DEBUGASSERT(state->cpu == 0 || state->cpu == 1);
#ifdef CONFIG_SMP
  DEBUGASSERT(state->other == 0 || state->other == 1);
  DEBUGASSERT(state->other != state->cpu);
  up_cpu_pause(state->other);
#endif

  spi_disable_cache(state->cpu, &state->val[state->cpu]);
#ifdef CONFIG_SMP
  spi_disable_cache(state->other, &state->val[state->other]);
#endif
}

/****************************************************************************
 * Name: esp32_spiflash_opdone
 *
 * Description:
 *   Undo all the steps of opstart.
 *
 ****************************************************************************/

static inline void IRAM_ATTR
  esp32_spiflash_opdone(const struct spiflash_cachestate_s *state)
{
  DEBUGASSERT(state->cpu == 0 || state->cpu == 1);
#ifdef CONFIG_SMP
  DEBUGASSERT(state->other == 0 || state->other == 1);
  DEBUGASSERT(state->other != state->cpu);
#endif

  spi_enable_cache(state->cpu, state->val[state->cpu]);
#ifdef CONFIG_SMP
  spi_enable_cache(state->other, state->val[state->other]);
  up_cpu_resume(state->other);
#endif

  leave_critical_section(state->flags);
}

/****************************************************************************
 * Name: spiflash_pagecached
 *
 * Description:
 *   Check if the given page is cached.
 *
 ****************************************************************************/

static bool IRAM_ATTR spiflash_pagecached(uint32_t phypage)
{
  int start[2];
  int end[2];
  int i;
  int j;

  /* Data ROM start and end pages */

  start[0] = DROM0_PAGES_START;
  end[0]   = DROM0_PAGES_END;

  /* Instruction RAM start and end pages */

  start[1] = PRO_IRAM0_FIRST_PAGE;
  end[1]   = IROM0_PAGES_END;

  for (i = 0; i < 2; i++)
    {
      for (j = start[i]; j < end[i]; j++)
        {
          if (PRO_MMU_TABLE[j] == phypage)
            {
              return true;
            }
        }
    }

  return false;
}

/****************************************************************************
 * Name: spiflash_flushmapped
 *
 * Description:
 *   Writeback PSRAM data and invalidate the cache if the address is mapped.
 *
 ****************************************************************************/

static void IRAM_ATTR spiflash_flushmapped(size_t start, size_t size)
{
  uint32_t page_start;
  uint32_t addr;
  uint32_t page;

  page_start = MMU_ALIGNDOWN_SIZE(start);
  size += (start - page_start);
  size = MMU_ALIGNUP_SIZE(size);

  for (addr = page_start; addr < page_start + size;
       addr += SPI_FLASH_MMU_PAGE_SIZE)
    {
      page = addr / SPI_FLASH_MMU_PAGE_SIZE;

      if (page >= 256)
        {
          return;
        }

      if (spiflash_pagecached(page))
        {
#ifdef CONFIG_ESP32_SPIRAM
          esp_spiram_writeback_cache();
#endif
          cache_flush(0);
#ifdef CONFIG_SMP
          cache_flush(1);
#endif
        }
    }
}

/****************************************************************************
 * Name: esp32_set_read_opt
 *
 * Description:
 *   Set SPI Flash to be direct read mode. Due to different SPI I/O mode
 *   including DIO, QIO and so on. Different command and communication
 *   timing sequence are needed.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_set_read_opt(struct esp32_spiflash_s *priv)
{
  uint32_t regval;
  uint32_t ctrl;
  uint32_t mode;
  uint32_t cmd;
  uint32_t cycles = 0;
  uint32_t addrbits = 0;
  uint32_t dummy = 0;

  ctrl = spi_get_reg(priv, SPI_CTRL_OFFSET);
  mode = ctrl & (SPI_FREAD_QIO | SPI_FASTRD_MODE);
  if (mode == (SPI_FREAD_QIO | SPI_FASTRD_MODE))
    {
      cycles = SPI1_R_QIO_DUMMY_CYCLELEN + priv->dummies[1];
      dummy = 1;
      addrbits = SPI1_R_QIO_ADDR_BITSLEN;
      cmd = (0x7 << SPI_USR_COMMAND_BITLEN_S) | 0xeb;
    }
  else if (mode == SPI_FASTRD_MODE)
    {
      if (ctrl & SPI_FREAD_DIO)
        {
          if (priv->dummies[1] == 0)
            {
              addrbits = SPI1_R_DIO_ADDR_BITSLEN;
              cmd = (0x7 << SPI_USR_COMMAND_BITLEN_S) | 0xbb;
            }
          else
            {
              cycles = priv->dummies[1] - 1;
              dummy = 1;
              addrbits = SPI1_R_DIO_ADDR_BITSLEN;
              cmd = 0xbb;
            }
        }
      else
        {
          if (ctrl & SPI_FREAD_QUAD)
            {
              cmd = (0x7 << SPI_USR_COMMAND_BITLEN_S) | 0x6b;
            }
          else if (ctrl & SPI_FREAD_DUAL)
            {
              cmd = (0x7 << SPI_USR_COMMAND_BITLEN_S) | 0x3b;
            }
          else
            {
              cmd = (0x7 << SPI_USR_COMMAND_BITLEN_S) | 0x0b;
            }

          cycles = SPI1_R_FAST_DUMMY_CYCLELEN + priv->dummies[1];
          dummy = 1;
          addrbits = SPI1_R_DIO_ADDR_BITSLEN;
        }
    }
  else
    {
      if (priv->dummies[1] != 0)
        {
          cycles = priv->dummies[1] - 1;
          dummy = 1;
        }

      addrbits = SPI1_R_SIO_ADDR_BITSLEN ;
      cmd = (0x7 << SPI_USR_COMMAND_BITLEN_S) | 0x03;
    }

  regval = spi_get_reg(priv, SPI_USER_OFFSET);
  regval &= ~SPI_USR_MOSI;
  regval = SPI_USR_MISO | SPI_USR_ADDR;
  if (dummy)
    {
      regval |= SPI_USR_DUMMY;
    }
  else
    {
      regval &= ~SPI_USR_DUMMY;
    }

  spi_set_regbits(priv, SPI_USER_OFFSET, regval);

  regval = spi_get_reg(priv, SPI_USER1_OFFSET);
  regval &= ~SPI_USR_DUMMY_CYCLELEN_M;
  regval |= cycles << SPI_USR_DUMMY_CYCLELEN_S;
  regval &= ~SPI_USR_ADDR_BITLEN_M;
  regval |= addrbits << SPI_USR_ADDR_BITLEN_S;
  spi_set_reg(priv, SPI_USER1_OFFSET, regval);

  regval = spi_get_reg(priv, SPI_USER2_OFFSET);
  regval &= ~SPI_USR_COMMAND_VALUE;
  regval |= cmd;
  spi_set_reg(priv, SPI_USER2_OFFSET, regval);
}

/****************************************************************************
 * Name: esp32_set_write_opt
 *
 * Description:
 *   Set SPI Flash to be direct read mode. Due to different SPI I/O mode
 *   including DIO, QIO and so on. Different command and communication
 *   timing sequence are needed.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_set_write_opt(struct esp32_spiflash_s *priv)
{
  uint32_t addrbits;
  uint32_t regval;

  spi_reset_regbits(priv, SPI_USER_OFFSET, SPI_USR_DUMMY);

  addrbits = ESP_ROM_SPIFLASH_W_SIO_ADDR_BITSLEN;
  regval = spi_get_reg(priv, SPI_USER1_OFFSET);
  regval &= ~SPI_USR_ADDR_BITLEN_M;
  regval |= addrbits << SPI_USR_ADDR_BITLEN_S;
  spi_set_reg(priv, SPI_USER1_OFFSET, regval);
}

/****************************************************************************
 * Name: esp32_read_status
 *
 * Description:
 *   Read SPI Flash status register value.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   status - status buffer pointer
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_read_status(struct esp32_spiflash_s *priv,
                                       uint32_t *status)
{
  esp32_spiflash_chip_t *chip = priv->chip;
  uint32_t regval;
  uint32_t flags;
  bool direct = (priv->dummies[1] == 0);

  do
    {
      if (direct)
        {
          spi_set_reg(priv, SPI_RD_STATUS_OFFSET, 0);
          spi_set_reg(priv, SPI_CMD_OFFSET, SPI_FLASH_RDSR);
          while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
            {
              ;
            }

          regval = spi_get_reg(priv, SPI_RD_STATUS_OFFSET);
          regval &= chip->status_mask;
          flags = regval & ESP_ROM_SPIFLASH_BUSY_FLAG;
        }
      else
        {
          if (esp_rom_spiflash_read_user_cmd(&regval, 0x05))
            {
              return -EIO;
            }

          flags = regval & ESP_ROM_SPIFLASH_BUSY_FLAG;
        }
    }
  while (flags == ESP_ROM_SPIFLASH_BUSY_FLAG);

  *status = regval;

  return OK;
}

/****************************************************************************
 * Name: esp32_wait_idle
 *
 * Description:
 *   Wait for SPI Flash to be in an idle state.
 *
 * Input Parameters:
 *   spi - ESP32 SPI Flash chip data
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_wait_idle(struct esp32_spiflash_s *priv)
{
  uint32_t status;

  while (spi_get_reg(priv, SPI_EXT2_OFFSET) & SPI_ST)
    {
      ;
    }

  while (getreg32(SPI_EXT2_REG(0)) & SPI_ST)
    {
      ;
    }

  if (esp32_read_status(priv, &status) != OK)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_read_highstatus
 *
 * Description:
 *   Read SPI Flash high status register value.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   status - status buffer pointer
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

#if 0
static int esp32_read_highstatus(struct esp32_spiflash_s *priv,
                                 uint32_t *status)
{
  uint32_t regval;

  if (esp32_wait_idle(priv))
    {
      return -EIO;
    }

  if (esp_rom_spiflash_read_user_cmd(&regval, 0x35))
    {
      return -EIO;
    }

  *status = regval << 8;

  return 0;
}
#endif

/****************************************************************************
 * Name: esp32_write_status
 *
 * Description:
 *   Write status value to SPI Flash status register.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   status - status data
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

#if 0
static int esp32_write_status(struct esp32_spiflash_s *priv,
                              uint32_t status)
{
  if (esp32_wait_idle(priv))
    {
      return -EIO;
    }

  spi_set_reg(priv, SPI_RD_STATUS_OFFSET, status);
  spi_set_reg(priv, SPI_CMD_OFFSET, SPI_FLASH_WRSR);
  while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
    {
      ;
    }

  if (esp32_wait_idle(priv))
    {
      return -EIO;
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: esp32_enable_write
 *
 * Description:
 *   Drive SPI flash entering into write mode.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_enable_write(struct esp32_spiflash_s *priv)
{
  uint32_t flags;
  uint32_t regval;

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  spi_set_reg(priv, SPI_RD_STATUS_OFFSET, 0);
  spi_set_reg(priv, SPI_CMD_OFFSET, SPI_FLASH_WREN);
  while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
    {
      ;
    }

  do
    {
      if (esp32_read_status(priv, &regval) != OK)
        {
          return -EIO;
        }

      flags = regval & ESP_ROM_SPIFLASH_WRENABLE_FLAG;
    }
  while (flags != ESP_ROM_SPIFLASH_WRENABLE_FLAG);

  return OK;
}

/****************************************************************************
 * Name: esp32_erasesector
 *
 * Description:
 *   Erase SPI Flash sector at designated address.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   addr   - erasing address
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_erasesector(struct esp32_spiflash_s *priv,
                                       uint32_t addr, uint32_t size)
{
  uint32_t offset;
  struct spiflash_cachestate_s state;

  esp32_set_write_opt(priv);

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  for (offset = 0; offset < size; offset += MTD_ERASESIZE(priv))
    {
      esp32_spiflash_opstart(&state);

      if (esp32_enable_write(priv) != OK)
        {
          esp32_spiflash_opdone(&state);
          return -EIO;
        }

      spi_set_reg(priv, SPI_ADDR_OFFSET, (addr + offset) & 0xffffff);
      spi_set_reg(priv, SPI_CMD_OFFSET, SPI_FLASH_SE);
      while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
        {
          ;
        }

      if (esp32_wait_idle(priv) != OK)
        {
          esp32_spiflash_opdone(&state);
          return -EIO;
        }

      esp32_spiflash_opdone(&state);
    }

  esp32_spiflash_opstart(&state);
  spiflash_flushmapped(addr, size);
  esp32_spiflash_opdone(&state);

  return 0;
}

/****************************************************************************
 * Name: esp32_writeonce
 *
 * Description:
 *   Write max 32 byte data to SPI Flash at designated address.
 *
 *   ESP32 can write max 32 byte once transmission by hardware.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number by bytes
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_writeonce(struct esp32_spiflash_s *priv,
                                     uint32_t addr,
                                     const uint32_t *buffer,
                                     uint32_t size)
{
  uint32_t regval;
  uint32_t i;

  if (size > SPI_FLASH_WRITE_BUF_SIZE)
    {
      return -EINVAL;
    }

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  if (esp32_enable_write(priv) != OK)
    {
      return -EIO;
    }

  regval = addr & 0xffffff;
  regval |= size << ESP_ROM_SPIFLASH_BYTES_LEN;
  spi_set_reg(priv, SPI_ADDR_OFFSET, regval);

  for (i = 0; i < (size / 4); i++)
    {
      spi_set_reg(priv, SPI_W0_OFFSET + i * 4, buffer[i]);
    }

  if (size & 0x3)
    {
      memcpy(&regval, &buffer[i], size & 0x3);
      spi_set_reg(priv, SPI_W0_OFFSET + i * 4, regval);
    }

  spi_set_reg(priv, SPI_RD_STATUS_OFFSET, 0);
  spi_set_reg(priv, SPI_CMD_OFFSET, SPI_FLASH_PP);
  while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
    {
      ;
    }

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_writedata
 *
 * Description:
 *   Write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_writedata(struct esp32_spiflash_s *priv,
                                     uint32_t addr,
                                     const uint8_t *buffer,
                                     uint32_t size)
{
  int ret;
  uint32_t off = 0;
  uint32_t bytes;
  uint32_t tmp_buf[SPI_FLASH_WRITE_WORDS];
  struct spiflash_cachestate_s state;

  esp32_set_write_opt(priv);

  while (size > 0)
    {
      bytes = MTD_BLKSIZE(priv) - addr % MTD_BLKSIZE(priv) ;
      if (!bytes)
        {
          bytes = MIN(size, SPI_FLASH_WRITE_BUF_SIZE);
        }
      else
        {
          bytes = MIN(bytes, size);
          bytes = MIN(bytes, SPI_FLASH_WRITE_BUF_SIZE);
        }

      memcpy(tmp_buf, &buffer[off], bytes);

      esp32_spiflash_opstart(&state);
      ret = esp32_writeonce(priv, addr, tmp_buf, bytes);
      esp32_spiflash_opdone(&state);

      if (ret)
        {
          return ret;
        }

      addr += bytes;
      size -= bytes;
      off += bytes;
    }

  esp32_spiflash_opstart(&state);
  spiflash_flushmapped(addr, size);
  esp32_spiflash_opdone(&state);

  return OK;
}

/****************************************************************************
 * Name: esp32_writedata
 *
 * Description:
 *   Write plaintext data to SPI Flash at designated address by SPI Flash
 *   hardware encryption, and written data in SPI Flash is ciphertext.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_writedata_encrypted(
  struct esp32_spiflash_s *priv,
  uint32_t addr,
  const uint8_t *buffer,
  uint32_t size)
{
  int i;
  int blocks;
  int ret = OK;
  uint32_t tmp_buf[SPI_FLASH_ENCRYPT_WORDS];
  struct spiflash_cachestate_s state;

  if (addr % SPI_FLASH_ENCRYPT_UNIT_SIZE)
    {
      ferr("ERROR: address=0x%x is not %d-byte align\n",
           addr, SPI_FLASH_ENCRYPT_UNIT_SIZE);
      return -EINVAL;
    }

  if (size % SPI_FLASH_ENCRYPT_UNIT_SIZE)
    {
      ferr("ERROR: size=%u is not %d-byte align\n",
           size, SPI_FLASH_ENCRYPT_UNIT_SIZE);
      return -EINVAL;
    }

  blocks = size / SPI_FLASH_ENCRYPT_UNIT_SIZE;

  for (i = 0; i < blocks; i++)
    {
      memcpy(tmp_buf, buffer, SPI_FLASH_ENCRYPT_UNIT_SIZE);

      esp32_spiflash_opstart(&state);
      esp_rom_spiflash_write_encrypted_enable();

      ret = esp_rom_spiflash_prepare_encrypted_data(addr, tmp_buf);
      if (ret)
        {
          ferr("ERROR: Failed to prepare encrypted data\n");
          goto exit;
        }

      ret = esp32_writeonce(priv, addr, tmp_buf,
                            SPI_FLASH_ENCRYPT_UNIT_SIZE);
      if (ret)
        {
          ferr("ERROR: Failed to write encrypted data @ 0x%x\n", addr);
          goto exit;
        }

      esp_rom_spiflash_write_encrypted_disable();
      esp32_spiflash_opdone(&state);

      addr += SPI_FLASH_ENCRYPT_UNIT_SIZE;
      buffer += SPI_FLASH_ENCRYPT_UNIT_SIZE;
      size -= SPI_FLASH_ENCRYPT_UNIT_SIZE;
    }

  esp32_spiflash_opstart(&state);
  spiflash_flushmapped(addr, size);
  esp32_spiflash_opdone(&state);

  return 0;

exit:
  esp_rom_spiflash_write_encrypted_disable();
  esp32_spiflash_opdone(&state);

  return ret;
}

/****************************************************************************
 * Name: esp32_readdata
 *
 * Description:
 *   Read max 64 byte data data from SPI Flash at designated address.
 *
 *   ESP32 can read max 64 byte once transmission by hardware.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number by bytes
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_readonce(struct esp32_spiflash_s *priv,
                                    uint32_t addr,
                                    uint32_t *buffer,
                                    uint32_t size)
{
  uint32_t regval;
  uint32_t i;

  if (size > SPI_FLASH_READ_BUF_SIZE)
    {
      return -EINVAL;
    }

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  regval = ((size << 3) - 1) << SPI_USR_MISO_DBITLEN_S;
  spi_set_reg(priv, SPI_MISO_DLEN_OFFSET, regval);

  regval = addr << 8;
  spi_set_reg(priv, SPI_ADDR_OFFSET, regval);

  spi_set_reg(priv, SPI_RD_STATUS_OFFSET, 0);
  spi_set_reg(priv, SPI_CMD_OFFSET, SPI_USR);
  while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
    {
      ;
    }

  for (i = 0; i < (size / 4); i++)
    {
      buffer[i] = spi_get_reg(priv, SPI_W0_OFFSET + i * 4);
    }

  if (size & 0x3)
    {
      regval = spi_get_reg(priv, SPI_W0_OFFSET + i * 4);
      memcpy(&buffer[i], &regval, size & 0x3);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_readdata
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_readdata(struct esp32_spiflash_s *priv,
                                    uint32_t addr,
                                    uint8_t *buffer,
                                    uint32_t size)
{
  int ret;
  uint32_t off = 0;
  uint32_t bytes;
  uint32_t tmp_buf[SPI_FLASH_READ_WORDS];
  struct spiflash_cachestate_s state;

  while (size > 0)
    {
      bytes = MIN(size, SPI_FLASH_READ_BUF_SIZE);

      esp32_spiflash_opstart(&state);
      ret = esp32_readonce(priv, addr, tmp_buf, bytes);
      esp32_spiflash_opdone(&state);

      if (ret)
        {
          return ret;
        }

      memcpy(&buffer[off], tmp_buf, bytes);

      addr += bytes;
      size -= bytes;
      off += bytes;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_mmap
 *
 * Description:
 *   Mapped SPI Flash address to ESP32's address bus, so that software
 *   can read SPI Flash data by reading data from memory access.
 *
 *   If SPI Flash hardware encryption is enable, the read from mapped
 *   address is decrypted.
 *
 * Input Parameters:
 *   spi - ESP32 SPI Flash chip data
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_mmap(struct esp32_spiflash_s *priv,
                                struct spiflash_map_req *req)
{
  int ret;
  int i;
  int start_page;
  int flash_page;
  int page_cnt;
  struct spiflash_cachestate_s state;
  bool flush = false;

  esp32_spiflash_opstart(&state);

  for (start_page = DROM0_PAGES_START;
       start_page < DROM0_PAGES_END;
       ++start_page)
    {
      if (PRO_MMU_TABLE[start_page] == INVALID_MMU_VAL
#ifdef CONFIG_SMP
          && APP_MMU_TABLE[start_page] == INVALID_MMU_VAL
#endif
          )
        {
          break;
        }
    }

  flash_page = MMU_ADDR2PAGE(req->src_addr);
  page_cnt = MMU_BYTES2PAGES(MMU_ADDR2OFF(req->src_addr) + req->size);

  if (start_page + page_cnt < DROM0_PAGES_END)
    {
      for (i = 0; i < page_cnt; i++)
        {
          PRO_MMU_TABLE[start_page + i] = flash_page + i;
#ifdef CONFIG_SMP
          APP_MMU_TABLE[start_page + i] = flash_page + i;
#endif
        }

      req->start_page = start_page;
      req->page_cnt = page_cnt;
      req->ptr = (void *)(VADDR0_START_ADDR +
                          start_page * SPI_FLASH_MMU_PAGE_SIZE +
                          MMU_ADDR2OFF(req->src_addr));
      flush = true;
      ret = 0;
    }
  else
    {
      ret = -ENOBUFS;
    }

  if (flush)
    {
#ifdef CONFIG_ESP32_SPIRAM
      esp_spiram_writeback_cache();
#endif
      cache_flush(0);
#ifdef CONFIG_SMP
      cache_flush(1);
#endif
    }

  esp32_spiflash_opdone(&state);

  return ret;
}

/****************************************************************************
 * Name: esp32_ummap
 *
 * Description:
 *   Unmap SPI Flash address in ESP32's address bus, and free resource.
 *
 * Input Parameters:
 *   spi - ESP32 SPI Flash chip data
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_ummap(struct esp32_spiflash_s *priv,
                                  const struct spiflash_map_req *req)
{
  int i;
  struct spiflash_cachestate_s state;

  esp32_spiflash_opstart(&state);

  for (i = req->start_page; i < req->start_page + req->page_cnt; ++i)
    {
      PRO_MMU_TABLE[i] = INVALID_MMU_VAL;
#ifdef CONFIG_SMP
      APP_MMU_TABLE[i] = INVALID_MMU_VAL;
#endif
    }

#ifdef CONFIG_ESP32_SPIRAM
  esp_spiram_writeback_cache();
#endif
  cache_flush(0);
#ifdef CONFIG_SMP
  cache_flush(1);
#endif
  esp32_spiflash_opdone(&state);
}

/****************************************************************************
 * Name: esp32_readdata_encrypted
 *
 * Description:
 *   Read decrypted data from SPI Flash at designated address when
 *   enable SPI Flash hardware encryption.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_readdata_encrypted(
  struct esp32_spiflash_s *priv,
  uint32_t addr,
  uint8_t *buffer,
  uint32_t size)
{
  int ret;
  struct spiflash_map_req req =
    {
      .src_addr = addr,
      .size = size
    };

  ret = esp32_mmap(priv, &req);
  if (ret)
    {
      return ret;
    }

  memcpy(buffer, req.ptr, size);

  esp32_ummap(priv, &req);

  return OK;
}

/****************************************************************************
 * Name: esp32_erase
 *
 * Description:
 *   Erase SPI Flash designated sectors.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32_erase(struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks)
{
  int ret;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);
  uint32_t addr = startblock * MTD_ERASESIZE(priv);
  uint32_t size = nblocks * MTD_ERASESIZE(priv);

  if ((addr >= MTD_SIZE(priv)) || (addr + size > MTD_SIZE(priv)))
    {
      return -EINVAL;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_erase(%p, %d, %d)\n", dev, startblock, nblocks);
#endif

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = esp32_erasesector(priv, addr, size);

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_erase()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32_read
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - ESP32 MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_read(struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, uint8_t *buffer)
{
  ssize_t ret;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_read(%p, 0x%x, %d, %p)\n", dev, offset, nbytes, buffer);
#endif

  /* Acquire the mutex. */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  esp32_set_read_opt(priv);
  ret = esp32_readdata(priv, offset, buffer, nbytes);

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_read()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32_bread
 *
 * Description:
 *   Read data from designated blocks.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_bread(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, uint8_t *buffer)
{
  int ret;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);
  uint32_t addr = MTD_BLK2SIZE(priv, startblock);
  uint32_t size = MTD_BLK2SIZE(priv, nblocks);

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bread(%p, 0x%x, %d, %p)\n",
        dev, startblock, nblocks, buffer);
#endif

  ret = esp32_read(dev, addr, size, buffer);
  if (ret == size)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bread()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32_read_decrypt
 *
 * Description:
 *   Read encrypted data and decrypt automatically from SPI Flash
 *   at designated address.
 *
 * Input Parameters:
 *   dev    - ESP32 MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_read_decrypt(struct mtd_dev_s *dev,
                                  off_t offset,
                                  size_t nbytes,
                                  uint8_t *buffer)
{
  ssize_t ret;
  uint8_t *tmpbuff = buffer;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_read_decrypt(%p, 0x%x, %d, %p)\n",
        dev, offset, nbytes, buffer);
#endif

  /* Acquire the mutex. */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = esp32_readdata_encrypted(priv, offset, tmpbuff, nbytes);

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_read_decrypt()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32_bread_decrypt
 *
 * Description:
 *   Read encrypted data and decrypt automatically from designated blocks.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_bread_decrypt(struct mtd_dev_s *dev,
                                   off_t startblock,
                                   size_t nblocks,
                                   uint8_t *buffer)
{
  int ret;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);
  uint32_t addr = MTD_BLK2SIZE(priv, startblock);
  uint32_t size = MTD_BLK2SIZE(priv, nblocks);

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bread_decrypt(%p, 0x%x, %d, %p)\n",
        dev, startblock, nblocks, buffer);
#endif

  ret = esp32_read_decrypt(dev, addr, size, buffer);
  if (ret == size)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bread_decrypt()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32_write
 *
 * Description:
 *   write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - ESP32 MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Written bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_write(struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, const uint8_t *buffer)
{
  ssize_t ret;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);

  ASSERT(buffer);

  if ((offset > MTD_SIZE(priv)) || ((offset + nbytes) > MTD_SIZE(priv)))
    {
      return -EINVAL;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_write(%p, 0x%x, %d, %p)\n", dev, offset, nbytes, buffer);
#endif

  /* Acquire the mutex. */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = esp32_writedata(priv, offset, buffer, nbytes);

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_write()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32_bwrite
 *
 * Description:
 *   Write data to designated blocks.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Written block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_bwrite(struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, const uint8_t *buffer)
{
  ssize_t ret;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);
  uint32_t addr = MTD_BLK2SIZE(priv, startblock);
  uint32_t size = MTD_BLK2SIZE(priv, nblocks);

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bwrite(%p, 0x%x, %d, %p)\n",
        dev, startblock, nblocks, buffer);
#endif

  ret = esp32_write(dev, addr, size, buffer);
  if (ret == size)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bwrite()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32_bwrite_encrypt
 *
 * Description:
 *   Write data to designated blocks by SPI Flash hardware encryption.
 *
 * Input Parameters:
 *   dev        - ESP32 MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Written block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_bwrite_encrypt(struct mtd_dev_s *dev,
                                    off_t startblock,
                                    size_t nblocks,
                                    const uint8_t *buffer)
{
  ssize_t ret;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);
  uint32_t addr = MTD_BLK2SIZE(priv, startblock);
  uint32_t size = MTD_BLK2SIZE(priv, nblocks);

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bwrite_encrypt(%p, 0x%x, %d, %p)\n",
        dev, startblock, nblocks, buffer);
#endif

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = esp32_writedata_encrypted(priv, addr, buffer, size);

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_bwrite_encrypt()=%d\n", ret);
#endif
  return ret;
}

/****************************************************************************
 * Name: esp32_ioctl
 *
 * Description:
 *   Set/Get option to/from ESP32 SPI Flash MTD device data.
 *
 * Input Parameters:
 *   dev - ESP32 MTD device data
 *   cmd - operation command
 *   arg - operation argument
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32_ioctl(struct mtd_dev_s *dev, int cmd,
                       unsigned long arg)
{
  int ret = -EINVAL;
  struct esp32_spiflash_s *priv = MTD2PRIV(dev);

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              geo->blocksize    = MTD_BLKSIZE(priv);
              geo->erasesize    = MTD_ERASESIZE(priv);
              geo->neraseblocks = MTD_SIZE(priv) / MTD_ERASESIZE(priv);
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          struct partition_info_s *info =
            (struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = MTD_SIZE(priv) / MTD_BLKSIZE(priv);
              info->sectorsize  = MTD_BLKSIZE(priv);
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          uint8_t *result = (uint8_t *)arg;
          *result = SPI_FLASH_ERASED_STATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spiflash_alloc_mtdpart
 *
 * Description:
 *   Allocate an MTD partition from the ESP32 SPI Flash.
 *
 * Input Parameters:
 *   mtd_offset - MTD Partition offset from the base address in SPI Flash.
 *   mtd_size   - Size for the MTD partition.
 *   encrypted  - Flag indicating whether the newly allocated partition will
 *                have its content encrypted.
 *
 * Returned Value:
 *   ESP32 SPI Flash MTD data pointer if success or NULL if fail.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32_spiflash_alloc_mtdpart(uint32_t mtd_offset,
                                               uint32_t mtd_size,
                                               bool encrypted)
{
  struct esp32_spiflash_s *priv;
  esp32_spiflash_chip_t *chip;
  struct mtd_dev_s *mtd_part;
  uint32_t blocks;
  uint32_t startblock;
  uint32_t size;

  if (encrypted)
    {
      priv = &g_esp32_spiflash1_encrypt;
    }
  else
    {
      priv = &g_esp32_spiflash1;
    }

  chip = priv->chip;

  finfo("ESP32 SPI Flash information:\n");
  finfo("\tID = 0x%x\n", chip->device_id);
  finfo("\tStatus mask = %x\n", chip->status_mask);
  finfo("\tChip size = %d KB\n", chip->chip_size / 1024);
  finfo("\tPage size = %d B\n", chip->page_size);
  finfo("\tSector size = %d KB\n", chip->sector_size / 1024);
  finfo("\tBlock size = %d KB\n", chip->block_size / 1024);

  ASSERT((mtd_offset + mtd_size) <= chip->chip_size);
  ASSERT((mtd_offset % chip->sector_size) == 0);
  ASSERT((mtd_size % chip->sector_size) == 0);

  if (mtd_size == 0)
    {
      size = chip->chip_size - mtd_offset;
    }
  else
    {
      size = mtd_size;
    }

  finfo("\tMTD offset = 0x%x\n", mtd_offset);
  finfo("\tMTD size = 0x%x\n", size);

  startblock = MTD_SIZE2BLK(priv, mtd_offset);
  blocks = MTD_SIZE2BLK(priv, size);

  mtd_part = mtd_partition(&priv->mtd, startblock, blocks);
  if (!mtd_part)
    {
      ferr("ERROR: create MTD partition");
      return NULL;
    }

  return mtd_part;
}

/****************************************************************************
 * Name: esp32_spiflash_get_mtd
 *
 * Description:
 *   Get ESP32 SPI Flash raw MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32 SPI Flash raw MTD data pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32_spiflash_get_mtd(void)
{
  struct esp32_spiflash_s *priv = &g_esp32_spiflash1;

  return &priv->mtd;
}

/****************************************************************************
 * Name: esp32_spiflash_get_mtd
 *
 * Description:
 *   Get ESP32 SPI Flash encryption raw MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32 SPI Flash encryption raw MTD data pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32_spiflash_encrypt_get_mtd(void)
{
  struct esp32_spiflash_s *priv = &g_esp32_spiflash1_encrypt;

  return &priv->mtd;
}

#endif /* CONFIG_ESP32_SPIFLASH */
