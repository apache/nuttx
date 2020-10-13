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
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/semaphore.h>
#include <nuttx/mtd/mtd.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "hardware/esp32_soc.h"
#include "hardware/esp32_spi.h"
#include "hardware/esp32_dport.h"
#include "rom/esp32_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_FLASH_WRITE_BUF_SIZE    (32)
#define SPI_FLASH_READ_BUF_SIZE     (64)

#define ESP32_MTD_OFFSET            CONFIG_ESP32_MTD_OFFSET
#define ESP32_MTD_SIZE              CONFIG_ESP32_MTD_SIZE

#define MTD2PRIV(_dev)              ((FAR struct esp32_spiflash_s *)_dev)
#define MTD_SIZE(_priv)             ((_priv)->chip->chip_size)
#define MTD_BLKSIZE(_priv)          ((_priv)->chip->page_size)
#define MTD_ERASESIZE(_priv)        ((_priv)->chip->sector_size)
#define MTD_BLK2SIZE(_priv, _b)     (MTD_BLKSIZE(_priv) * (_b))
#define MTD_SIZE2BLK(_priv, _s)     ((_s) / MTD_BLKSIZE(_priv))

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

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

  /* Enxusre exculisve access to the driver */

  sem_t exclsem;
};

/****************************************************************************
 * ROM function prototypes
 ****************************************************************************/

void Cache_Flush(int cpu);
void Cache_Read_Enable(int cpu);
void Cache_Read_Disable(int cpu);

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
static inline void IRAM_ATTR spi_memcpy(void *d, const void *s, uint32_t n);

/* Misc. helpers */

static inline irqstate_t IRAM_ATTR esp32_spiflash_opstart(
  FAR struct esp32_spiflash_s *priv, int *cpu);
static inline void IRAM_ATTR esp32_spiflash_opdone(
  FAR struct esp32_spiflash_s *priv, irqstate_t flags, int cpu);

/* Flash helpers */

static void IRAM_ATTR esp32_set_read_opt(FAR struct esp32_spiflash_s *priv);
static void IRAM_ATTR esp32_set_write_opt(struct esp32_spiflash_s *priv);
static int  IRAM_ATTR  esp32_read_status(FAR struct esp32_spiflash_s *priv,
                                         uint32_t *status);
static int IRAM_ATTR esp32_wait_idle(FAR struct esp32_spiflash_s *priv);
static int IRAM_ATTR esp32_enable_write(FAR struct esp32_spiflash_s *priv);
static int IRAM_ATTR esp32_erasesector(FAR struct esp32_spiflash_s *priv,
                                       uint32_t addr, uint32_t size);
static int IRAM_ATTR esp32_writedata(FAR struct esp32_spiflash_s *priv,
                                     uint32_t addr,
                                     const uint8_t *buffer, uint32_t size);
static int IRAM_ATTR esp32_readdata(FAR struct esp32_spiflash_s *priv,
                                    uint32_t addr,
                                    uint8_t *buffer, uint32_t size);
#if 0
static int esp32_read_highstatus(FAR struct esp32_spiflash_s *priv,
                                 uint32_t *status);
#endif
#if 0
static int esp32_write_status(FAR struct esp32_spiflash_s *priv,
                              uint32_t status);
#endif

/* MTD driver methods */

static int esp32_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks);
static ssize_t esp32_read(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR uint8_t *buffer);
static ssize_t esp32_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buffer);
static ssize_t esp32_write(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR const uint8_t *buffer);
static ssize_t esp32_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buffer);
static int esp32_ioctl(FAR struct mtd_dev_s *dev, int cmd,
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
 * Name: spi_memcpy
 *
 * Description:
 *   Copy data from one block of memory to another block of memory.
 *   The function must be linked in IRAM not in flash, so add this function
 *   instead of libc memcpy.
 *
 * Input Parameters:
 *   d - destination address
 *   s - source address
 *   n - data bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void IRAM_ATTR spi_memcpy(void *d, const void *s, uint32_t n)
{
  uint8_t *dest = (uint8_t *)d;
  const uint8_t *src = (const uint8_t *)s;

  while (n--)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Name: esp32_spiflash_opstart
 *
 * Description:
 *   Prepare for an SPIFLASH operartion.
 *
 ****************************************************************************/

static inline irqstate_t IRAM_ATTR esp32_spiflash_opstart(
  FAR struct esp32_spiflash_s *priv, int *cpu)
{
  irqstate_t flags;
#ifdef CONFIG_SMP
  int other;
#endif

  flags = enter_critical_section();

  *cpu = up_cpu_index();
#ifdef CONFIG_SMP
  other = *cpu ? 0 : 1;
#endif

  DEBUGASSERT(*cpu == 0 || *cpu == 1);
#ifdef CONFIG_SMP
  DEBUGASSERT(other == 0 || other == 1);
#endif

  Cache_Read_Disable(*cpu);
#ifdef CONFIG_SMP
  Cache_Read_Disable(other);
#endif

  return flags;
}

/****************************************************************************
 * Name: esp32_spiflash_opdone
 *
 * Description:
 *   Undo all the steps of opstart.
 *
 ****************************************************************************/

static inline void IRAM_ATTR esp32_spiflash_opdone(
  FAR struct esp32_spiflash_s *priv, irqstate_t flags, int cpu)
{
#ifdef CONFIG_SMP
  int other;
#endif

#ifdef CONFIG_SMP
  other = cpu ? 0 : 1;
#endif

  DEBUGASSERT(cpu == 0 || cpu == 1);
#ifdef CONFIG_SMP
  DEBUGASSERT(other == 0 || other == 1);
#endif

  Cache_Flush(cpu);
  Cache_Read_Enable(cpu);
#ifdef CONFIG_SMP
  Cache_Flush(other);
  Cache_Read_Enable(other);
#endif

  leave_critical_section(flags);
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

static void IRAM_ATTR esp32_set_read_opt(FAR struct esp32_spiflash_s *priv)
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
 *   Read SPI Flash status regitser value.
 *
 * Input Parameters:
 *   spi    - ESP32 SPI Flash chip data
 *   status - status buffer pointer
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_read_status(FAR struct esp32_spiflash_s *priv,
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

static int IRAM_ATTR esp32_wait_idle(FAR struct esp32_spiflash_s *priv)
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
 *   Read SPI Flash high status regitser value.
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
static int esp32_read_highstatus(FAR struct esp32_spiflash_s *priv,
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
 *   Write status value to SPI Flash status regitser.
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
static int esp32_write_status(FAR struct esp32_spiflash_s *priv,
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

static int esp32_enable_write(FAR struct esp32_spiflash_s *priv)
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

static int IRAM_ATTR esp32_erasesector(FAR struct esp32_spiflash_s *priv,
                                       uint32_t addr, uint32_t size)
{
  uint32_t offset;
  int me;
  uint32_t flags;

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  for (offset = 0; offset < size; offset += MTD_ERASESIZE(priv))
    {
      flags = esp32_spiflash_opstart(priv, &me);

      if (esp32_enable_write(priv) != OK)
        {
          esp32_spiflash_opdone(priv, flags, me);
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
          esp32_spiflash_opdone(priv, flags, me);
          return -EIO;
        }

      esp32_spiflash_opdone(priv, flags, me);
    }

  return 0;
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

static int esp32_writedata(FAR struct esp32_spiflash_s *priv, uint32_t addr,
                           const uint8_t *buffer, uint32_t size)
{
  uint32_t regval;
  uint32_t i;
  uint32_t bytes;
  uint32_t tmp;
  uint32_t res;
  const uint8_t  *tmpbuff = buffer;
  int ret = OK;
  int me;
  uint32_t flags;

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  while (size > 0)
    {
      flags = esp32_spiflash_opstart(priv, &me);

      if (esp32_enable_write(priv) != OK)
        {
          esp32_spiflash_opdone(priv, flags, me);
          return -EIO;
        }

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

      regval = addr & 0xffffff;
      regval |= bytes << ESP_ROM_SPIFLASH_BYTES_LEN;
      spi_set_reg(priv, SPI_ADDR_OFFSET, regval);

      for (i = 0; i < bytes; i += 4)
        {
          res = MIN(4, bytes - i);

          spi_memcpy(&tmp, tmpbuff, res);
          spi_set_reg(priv, SPI_W0_OFFSET + i, tmp);
          tmpbuff += res;
        }

      spi_set_reg(priv, SPI_RD_STATUS_OFFSET, 0);
      spi_set_reg(priv, SPI_CMD_OFFSET, SPI_FLASH_PP);
      while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
        {
          ;
        }

      if (esp32_wait_idle(priv) != OK)
        {
          esp32_spiflash_opdone(priv, flags, me);
          return -EIO;
        }

      addr += bytes;
      size -= bytes;

      esp32_spiflash_opdone(priv, flags, me);
    }

  return ret;
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

static int esp32_readdata(FAR struct esp32_spiflash_s *priv, uint32_t addr,
                          uint8_t *buffer, uint32_t size)
{
  uint32_t regval;
  uint32_t i;
  uint32_t bytes;
  uint32_t tmp;
  uint32_t res;
  uint8_t  *tmpbuff = buffer;
  int me;
  uint32_t flags;

  if (esp32_wait_idle(priv) != OK)
    {
      return -EIO;
    }

  while (size > 0)
    {
      flags = esp32_spiflash_opstart(priv, &me);

      bytes = MIN(size, SPI_FLASH_READ_BUF_SIZE);
      regval = ((bytes << 3) - 1) << SPI_USR_MISO_DBITLEN_S;
      spi_set_reg(priv, SPI_MISO_DLEN_OFFSET, regval);

      regval = addr << 8;
      spi_set_reg(priv, SPI_ADDR_OFFSET, regval);

      spi_set_reg(priv, SPI_RD_STATUS_OFFSET, 0);
      spi_set_reg(priv, SPI_CMD_OFFSET, SPI_USR);
      while (spi_get_reg(priv, SPI_CMD_OFFSET) != 0)
        {
          ;
        }

      for (i = 0; i < bytes; i += 4)
        {
          res = MIN(4, bytes - i);

          tmp = spi_get_reg(priv, SPI_W0_OFFSET + i);
          spi_memcpy(tmpbuff, &tmp, res);
          tmpbuff += res;
        }

      addr += bytes;
      size -= bytes;

      esp32_spiflash_opdone(priv, flags, me);
    }

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

static int esp32_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks)
{
  int ret;
  FAR struct esp32_spiflash_s *priv = MTD2PRIV(dev);
  uint32_t addr = startblock * MTD_ERASESIZE(priv);
  uint32_t size = nblocks * MTD_ERASESIZE(priv);

  if ((addr >= MTD_SIZE(priv)) || (addr + size > MTD_SIZE(priv)))
    {
      return -EINVAL;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_erase(%p, %d, %d)\n", dev, startblock, nblocks);
#endif

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  esp32_set_write_opt(priv);
  ret = esp32_erasesector(priv, addr, size);

  nxsem_post(&priv->exclsem);

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

static ssize_t esp32_read(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR uint8_t *buffer)
{
  int ret;
  uint8_t *tmpbuff = buffer;
  FAR struct esp32_spiflash_s *priv = MTD2PRIV(dev);

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_read(%p, 0x%x, %d, %p)\n", dev, offset, nbytes, buffer);
#endif

#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(buffer))
    {
      tmpbuff = xtensa_imm_malloc(nbytes);
      if (tmpbuff == NULL)
        {
          return (ssize_t)-ENOMEM;
        }
    }
#endif

  /* Acquire the semaphore. */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      goto error_with_buffer;
    }

  esp32_set_read_opt(priv);
  ret = esp32_readdata(priv, offset, tmpbuff, nbytes);

  nxsem_post(&priv->exclsem);

  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_read()=%d\n", ret);
#endif

error_with_buffer:
#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(buffer))
    {
      memcpy(buffer, tmpbuff, (ret == OK) ? nbytes : 0);
      xtensa_imm_free(tmpbuff);
    }
#endif

  return (ssize_t)ret;
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

static ssize_t esp32_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buffer)
{
  int ret;
  FAR struct esp32_spiflash_s *priv = MTD2PRIV(dev);
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
 *   Writen bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_write(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR const uint8_t *buffer)
{
  int ret;
  uint8_t *tmpbuff = (uint8_t *)buffer;
  FAR struct esp32_spiflash_s *priv = MTD2PRIV(dev);

  ASSERT(buffer);

  if ((offset > MTD_SIZE(priv)) || ((offset + nbytes) > MTD_SIZE(priv)))
    {
      return -EINVAL;
    }

#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(buffer))
    {
      tmpbuff = xtensa_imm_malloc(nbytes);
      if (tmpbuff == NULL)
        {
          return (ssize_t)-ENOMEM;
        }

      memcpy(tmpbuff, buffer, nbytes);
    }
#endif

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_write(%p, 0x%x, %d, %p)\n", dev, offset, nbytes, buffer);
#endif

  /* Acquire the semaphore. */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      goto error_with_buffer;
    }

  esp32_set_write_opt(priv);
  ret = esp32_writedata(priv, offset, tmpbuff, nbytes);

  nxsem_post(&priv->exclsem);

  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32_SPIFLASH_DEBUG
  finfo("esp32_write()=%d\n", ret);
#endif

error_with_buffer:
#ifdef CONFIG_XTENSA_USE_SEPERATE_IMEM
  if (esp32_ptr_extram(buffer))
    {
      xtensa_imm_free(tmpbuff);
    }
#endif

  return (ssize_t)ret;
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
 *   Writen block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buffer)
{
  ssize_t ret;
  FAR struct esp32_spiflash_s *priv = MTD2PRIV(dev);
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

static int esp32_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                       unsigned long arg)
{
  int ret = -EINVAL;
  FAR struct esp32_spiflash_s *priv = MTD2PRIV(dev);
  FAR struct mtd_geometry_s *geo;

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          geo = (FAR struct mtd_geometry_s *)arg;
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
 *   Alloc ESP32 SPI Flash MTD
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32 SPI Flash MTD data pointer if success or NULL if fail
 *
 ****************************************************************************/

FAR struct mtd_dev_s *esp32_spiflash_alloc_mtdpart(void)
{
  struct esp32_spiflash_s *priv = &g_esp32_spiflash1;
  esp32_spiflash_chip_t *chip = priv->chip;
  FAR struct mtd_dev_s *mtd_part;
  uint32_t blocks;
  uint32_t startblock;
  uint32_t size;

  /* Initiliaze the mutex */

  nxsem_init(&priv->exclsem, 0, 1);

  ASSERT((ESP32_MTD_OFFSET + ESP32_MTD_SIZE) <= chip->chip_size);
  ASSERT((ESP32_MTD_OFFSET % chip->sector_size) == 0);
  ASSERT((ESP32_MTD_SIZE % chip->sector_size) == 0);

  finfo("ESP32 SPI Flash information:\n");
  finfo("\tID = 0x%x\n", chip->device_id);
  finfo("\tStatus mask = %x\n", chip->status_mask);
  finfo("\tChip size = %d KB\n", chip->chip_size / 1024);
  finfo("\tPage size = %d B\n", chip->page_size);
  finfo("\tSector size = %d KB\n", chip->sector_size / 1024);
  finfo("\tBlock size = %d KB\n", chip->block_size / 1024);

#if ESP32_MTD_SIZE == 0
  size = chip->chip_size - ESP32_MTD_OFFSET;
#else
  size = ESP32_MTD_SIZE;
#endif

  finfo("\tMTD offset = 0x%x\n", ESP32_MTD_OFFSET);
  finfo("\tMTD size = 0x%x\n", size);

  startblock = MTD_SIZE2BLK(priv, ESP32_MTD_OFFSET);
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

FAR struct mtd_dev_s *esp32_spiflash_get_mtd(void)
{
  struct esp32_spiflash_s *priv = &g_esp32_spiflash1;

  return &priv->mtd;
}

#endif /* CONFIG_ESP32_SPIFLASH */
