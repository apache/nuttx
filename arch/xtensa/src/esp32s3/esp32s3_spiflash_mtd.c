/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiflash_mtd.c
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
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <inttypes.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/mtd.h>

#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_cache_memory.h"

#include "esp_attr.h"
#include "esp32s3_spiflash.h"
#include "esp32s3_spiram.h"

#include "rom/esp32s3_spiflash.h"
#include "esp32s3_spiflash_mtd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MTD_BLK_SIZE                CONFIG_ESP32S3_SPIFLASH_MTD_BLKSIZE
#define MTD_ERASE_SIZE              4096
#define MTD_ERASED_STATE            (0xff)

#define MTD2PRIV(_dev)              ((struct esp32s3_mtd_dev_s *)_dev)
#define MTD_SIZE(_priv)             ((*(_priv)->data)->chip.chip_size)
#define MTD_BLK2SIZE(_priv, _b)     (MTD_BLK_SIZE * (_b))
#define MTD_SIZE2BLK(_priv, _s)     ((_s) / MTD_BLK_SIZE)

#define SPI_FLASH_ENCRYPT_UNIT_SIZE (64)
#define SPI_FLASH_ENCRYPT_MIN_SIZE  (16)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
/* SPI flash work operation code */

enum spiflash_op_code_e
{
  SPIFLASH_OP_CODE_WRITE = 0,
  SPIFLASH_OP_CODE_READ,
  SPIFLASH_OP_CODE_ERASE,
  SPIFLASH_OP_CODE_SET_BANK,
  SPIFLASH_OP_CODE_ENCRYPT_READ,
  SPIFLASH_OP_CODE_ENCRYPT_WRITE
};
#endif

/* ESP32-S3 SPI Flash device private data  */

struct esp32s3_mtd_dev_s
{
  struct mtd_dev_s mtd;

  /* SPI Flash data */

  esp_rom_spiflash_legacy_data_t **data;
};

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
/* SPI flash work operation arguments */

struct spiflash_work_arg
{
  enum spiflash_op_code_e op_code;

  struct
  {
    uint32_t addr;
    uint8_t *buffer;
    uint32_t size;
    uint32_t paddr;
  } op_arg;

  volatile int ret;

  sem_t sem;
};
#endif

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int esp32s3_erase(struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks);
static ssize_t esp32s3_read(struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, uint8_t *buffer);
static ssize_t esp32s3_read_decrypt(struct mtd_dev_s *dev,
                                    off_t offset,
                                    size_t nbytes,
                                    uint8_t *buffer);
static ssize_t esp32s3_bread(struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, uint8_t *buffer);
static ssize_t esp32s3_bread_decrypt(struct mtd_dev_s *dev,
                                     off_t startblock,
                                     size_t nblocks,
                                     uint8_t *buffer);
static ssize_t esp32s3_write(struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, const uint8_t *buffer);
static ssize_t esp32s3_bwrite(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, const uint8_t *buffer);
static int esp32s3_writedata_encrypt(struct mtd_dev_s *dev, off_t offset,
                                     uint32_t size, const uint8_t *buffer);
static ssize_t esp32s3_write_encrypt(struct mtd_dev_s *dev, off_t offset,
                                     size_t nbytes, const uint8_t *buffer);
static ssize_t esp32s3_bwrite_encrypt(struct mtd_dev_s *dev,
                                      off_t startblock,
                                      size_t nblocks,
                                      const uint8_t *buffer);
static int esp32s3_ioctl(struct mtd_dev_s *dev, int cmd,
                         unsigned long arg);

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
static inline bool IRAM_ATTR stack_is_psram(void);
static void esp32s3_spiflash_work(void *arg);
static int esp32s3_async_op(enum spiflash_op_code_e opcode,
                            uint32_t addr,
                            const uint8_t *buffer,
                            uint32_t size,
                            uint32_t paddr);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct esp32s3_mtd_dev_s g_esp32s3_spiflash =
{
  .mtd =
          {
            .erase  = esp32s3_erase,
            .bread  = esp32s3_bread,
            .bwrite = esp32s3_bwrite,
            .read   = esp32s3_read,
            .ioctl  = esp32s3_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = esp32s3_write,
#endif
            .name   = "esp32s3_spiflash"
          },
  .data = (esp_rom_spiflash_legacy_data_t **)
          (&rom_spiflash_legacy_data),
};

static const struct esp32s3_mtd_dev_s g_esp32s3_spiflash_encrypt =
{
  .mtd =
          {
            .erase  = esp32s3_erase,
            .bread  = esp32s3_bread_decrypt,
            .bwrite = esp32s3_bwrite_encrypt,
            .read   = esp32s3_read_decrypt,
            .ioctl  = esp32s3_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
            .write  = esp32s3_write_encrypt,
#endif
            .name   = "esp32s3_spiflash_encrypt"
          },
  .data = (esp_rom_spiflash_legacy_data_t **)
          (&rom_spiflash_legacy_data),
};

/* Ensure exclusive access to the driver */

static mutex_t g_lock = NXMUTEX_INITIALIZER;

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
static struct work_s g_work;
static mutex_t g_work_lock = NXMUTEX_INITIALIZER;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stack_is_psram
 *
 * Description:
 *   Check if current task's stack space is in PSRAM.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if it is in PSRAM or false if not.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
static inline bool IRAM_ATTR stack_is_psram(void)
{
  void *sp = (void *)up_getsp();

  return esp32s3_ptr_extram(sp);
}
#endif

/**
 * Choose type of chip you want to encrypt manully
 *
 * esp-idf/components/hal/esp32s3/include/hal/spi_flash_encrypted_ll.h
 *
 */

typedef enum
{
  FLASH_ENCRYPTION_MANU = 0, /* !< Manually encrypt the flash chip. */
  PSRAM_ENCRYPTION_MANU = 1  /* !< Manually encrypt the psram chip. */
} flash_encrypt_ll_type_t;

/****************************************************************************
 * Name: spi_flash_encrypt_ll_enable
 *
 * Description:
 *   Enable the flash encryption function under spi boot mode and
 *   download boot mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_enable(void)
{
  REG_SET_BIT(SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL_REG,
              SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT |
              SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT);
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_disable
 *
 * Description:
 *   Disable the flash encryption mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_disable(void)
{
  REG_CLR_BIT(SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL_REG,
              SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT);
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_type
 *
 * Description:
 *   Choose type of chip you want to encrypt manully
 *
 * Input Parameters:
 *   type - The type of chip to be encrypted
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_type(flash_encrypt_ll_type_t type)
{
  DEBUGASSERT(type == FLASH_ENCRYPTION_MANU); /* Our hardware only support flash encryption */
  REG_WRITE(AES_XTS_DESTINATION_REG, type);
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_buffer_length
 *
 * Description:
 *   Configure the data size of a single encryption.
 *
 * Input Parameters:
 *   size - Size of the desired block.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_buffer_length(uint32_t size)
{
  REG_WRITE(AES_XTS_SIZE_REG, size >> 5); /* Desired block should not be larger than the block size. */
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_plaintext_save
 *
 * Description:
 *   Save 32-bit piece of plaintext.
 *
 * Input Parameters:
 *   address - the address of written flash partition.
 *   buffer  - Buffer to store the input data.
 *   size    - Buffer size.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline
void spi_flash_encrypt_ll_plaintext_save(uint32_t address,
                                         const uint32_t * buffer,
                                         uint32_t size)
{
  uint32_t plaintext_offs =
      (address % SOC_FLASH_ENCRYPTED_XTS_AES_BLOCK_MAX);
  DEBUGASSERT(
      plaintext_offs + size <= SOC_FLASH_ENCRYPTED_XTS_AES_BLOCK_MAX);
  memcpy((void *)(AES_XTS_PLAIN_BASE + plaintext_offs), buffer, size);
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_address_save
 *
 * Description:
 *   Copy the flash address to XTS_AES physical address
 *
 * Input Parameters:
 *   flash_addr - flash address to write.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_address_save(uint32_t flash_addr)
{
  REG_WRITE(AES_XTS_PHYSICAL_ADDR_REG, flash_addr);
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_calculate_start
 *
 * Description:
 *   Start flash encryption
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_calculate_start(void)
{
  REG_WRITE(AES_XTS_TRIGGER_REG, 1);
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_calculate_wait_idle
 *
 * Description:
 *   Wait for flash encryption termination
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_calculate_wait_idle(void)
{
  while (REG_READ(AES_XTS_STATE_REG) == 0x1)
    {
    }
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_done
 *
 * Description:
 *   Finish the flash encryption and make encrypted result accessible to SPI.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_done(void)
{
  REG_WRITE(AES_XTS_RELEASE_REG, 1);
  while (REG_READ(AES_XTS_STATE_REG) != 0x3)
    {
    }
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_destroy
 *
 * Description:
 *   Set to destroy encrypted result
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flash_encrypt_ll_destroy(void)
{
  REG_WRITE(AES_XTS_DESTROY_REG, 1);
}

/****************************************************************************
 * Name: spi_flash_encrypt_ll_check
 *
 * Description:
 *   Check if is qualified to encrypt the buffer
 *
 * Input Parameters:
 *   address - the address of written flash partition.
 *   length  - Buffer size.
 *
 * Returned Value:
 *   True if the address is qualified.
 *
 ****************************************************************************/

static inline bool spi_flash_encrypt_ll_check(uint32_t address,
                                              uint32_t length)
{
  return ((address % length) == 0) ? true : false;
}

/**
 * from: esp-idf/components/hal/spi_flash_encrypt_hal_iram.c
 */

/****************************************************************************
 * Name: spi_flash_encryption_hal_enable
 *
 * Description:
 *   Enable flash encryption
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spi_flash_encryption_hal_enable(void)
{
  spi_flash_encrypt_ll_enable();
  spi_flash_encrypt_ll_type(FLASH_ENCRYPTION_MANU);
}

/****************************************************************************
 * Name: spi_flash_encryption_hal_disable
 *
 * Description:
 *   Disable flash encryption
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spi_flash_encryption_hal_disable(void)
{
  spi_flash_encrypt_ll_disable();
}

/****************************************************************************
 * Name: spi_flash_encryption_hal_prepare
 *
 * Description:
 *   Prepare flash encryption
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spi_flash_encryption_hal_prepare(uint32_t address,
                                      const uint32_t * buffer, uint32_t size)
{
  spi_flash_encrypt_ll_buffer_length(size);
  spi_flash_encrypt_ll_address_save(address);
  spi_flash_encrypt_ll_plaintext_save(address, buffer, size);
  spi_flash_encrypt_ll_calculate_start();
}

/****************************************************************************
 * Name: spi_flash_encryption_hal_done
 *
 * Description:
 *   wait flash encryption and mark down
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spi_flash_encryption_hal_done(void)
{
  spi_flash_encrypt_ll_calculate_wait_idle();
  spi_flash_encrypt_ll_done();
}

/****************************************************************************
 * Name: spi_flash_encryption_hal_destroy
 *
 * Description:
 *   Set to destroy encrypted result
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spi_flash_encryption_hal_destroy(void)
{
  spi_flash_encrypt_ll_destroy();
}

/****************************************************************************
 * Name: spi_flash_encryption_hal_destroy
 *
 * Description:
 *   Check if is qualified to encrypt the buffer
 *
 * Input Parameters:
 *   address - the address of written flash partition.
 *   length  - Buffer size.
 *
 * Returned Value:
 *   True if the address is qualified.
 *
 ****************************************************************************/

bool spi_flash_encryption_hal_check(uint32_t address, uint32_t length)
{
  return spi_flash_encrypt_ll_check(address, length);
}

/**
 * These are the pointer to HW flash encryption.
 * Default using hardware encryption.
 *
 * esp-idf/components/spi_flash/spi_flash_chip_generic.c
 *
 */

DRAM_ATTR static spi_flash_encryption_t esp_flash_encryption_default
__attribute__((__unused__)) =
{
  .flash_encryption_enable = spi_flash_encryption_hal_enable,
  .flash_encryption_disable = spi_flash_encryption_hal_disable,
  .flash_encryption_data_prepare = spi_flash_encryption_hal_prepare,
  .flash_encryption_done = spi_flash_encryption_hal_done,
  .flash_encryption_destroy = spi_flash_encryption_hal_destroy,
  .flash_encryption_check = spi_flash_encryption_hal_check,
};

/****************************************************************************
 * Name: spi_flash_write_encrypted_manu
 *
 * Description:
 *   Workaround of spi_flash_write_encrypted
 *   by using of spi_flash_chip_generic_write_encrypted
 *
 * Input Parameters:
 *   address - Destination address in Flash. Must be a multiple of 16
 *             bytes.
 *   buffer  - Pointer to the source buffer.
 *   length  - Length of data, in bytes. Must be a multiple of 16 bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_write_encrypted_manu(uint32_t address, const void *buffer,
                                   uint32_t length)
{
  spi_flash_encryption_t *esp_flash_encryption =
                                             &esp_flash_encryption_default;
  esp_err_t err = ESP_OK;

  /* Check if the buffer and length can qualify the requirements */

  if (esp_flash_encryption->flash_encryption_check(address, length) != true)
    {
      return ESP_ERR_NOT_SUPPORTED;
    }

  const uint8_t *data_bytes = (const uint8_t *)buffer;
  esp_flash_encryption->flash_encryption_enable();
  while (length > 0)
    {
      int block_size;

      /* Write the largest block if possible */

      if (address % 64 == 0 && length >= 64)
        {
          block_size = 64;
        }
      else if (address % 32 == 0 && length >= 32)
        {
          block_size = 32;
        }
      else
        {
          block_size = 16;
        }

      /**
       * Prepare the flash chip (same time as AES operation, for performance)
       */

      esp_flash_encryption->flash_encryption_data_prepare(address,
                                                          (uint32_t *)
                                                          data_bytes,
                                                          block_size);

      /* err = chip->chip_drv->set_chip_write_protect(chip, false); */

      if (err != ESP_OK)
        {
          return err;
        }

      /**
       * Waiting for encrypting buffer to finish and
       * making result visible for SPI1
       */

      esp_flash_encryption->flash_encryption_done();

      /**
       * Note: For encryption function, after write flash command is sent.
       * The hardware will write the encrypted buffer
       * prepared in XTS_FLASH_ENCRYPTION register
       * in function `flash_encryption_data_prepare`, instead of the origin
       * buffer named `data_bytes`.
       */

      /**
       * err = chip->chip_drv->write(chip, (uint32_t *)data_bytes,
       *                             address, length);
       */

      if (err != ESP_OK)
        {
          return err;
        }

      /**
       * err =
       *     chip->chip_drv->wait_idle(chip,
       *                               chip->chip_drv->timeout->
       *                               page_program_timeout);
       */

      if (err != ESP_OK)
        {
          return err;
        }

      spi_flash_write(address, data_bytes, block_size);

      /**
       * Note: we don't wait for idle status here, because this way
       * the AES peripheral can start encrypting the next
       * block while the SPI flash chip is busy completing the write
       */

      esp_flash_encryption->flash_encryption_destroy();

      length -= block_size;
      data_bytes += block_size;
      address += block_size;
    }

  esp_flash_encryption->flash_encryption_disable();
  return err;
}

/****************************************************************************
 * Name: esp32s3_spiflash_work
 *
 * Description:
 *   Do SPI Flash operation, cache result and send semaphore to wake up
 *   blocked task.
 *
 * Input Parameters:
 *   arg - Reference to SPI flash work arguments structure (cast to void*)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
static void esp32s3_spiflash_work(void *arg)
{
  struct spiflash_work_arg *work_arg = (struct spiflash_work_arg *)arg;

  if (work_arg->op_code == SPIFLASH_OP_CODE_WRITE)
    {
      work_arg->ret = spi_flash_write(work_arg->op_arg.addr,
                                      work_arg->op_arg.buffer,
                                      work_arg->op_arg.size);
    }
  else if (work_arg->op_code == SPIFLASH_OP_CODE_READ)
    {
      work_arg->ret = spi_flash_read(work_arg->op_arg.addr,
                                     work_arg->op_arg.buffer,
                                     work_arg->op_arg.size);
    }
  else if (work_arg->op_code == SPIFLASH_OP_CODE_ERASE)
    {
      work_arg->ret = spi_flash_erase_range(work_arg->op_arg.addr,
                                            work_arg->op_arg.size);
    }
  else if (work_arg->op_code == SPIFLASH_OP_CODE_SET_BANK)
    {
      work_arg->ret = cache_dbus_mmu_map(work_arg->op_arg.addr,
                                         work_arg->op_arg.paddr,
                                         work_arg->op_arg.size);
    }
  else if (work_arg->op_code == SPIFLASH_OP_CODE_ENCRYPT_READ)
    {
      work_arg->ret = spi_flash_read_encrypted(work_arg->op_arg.addr,
                                               work_arg->op_arg.buffer,
                                               work_arg->op_arg.size);
    }
  else if (work_arg->op_code == SPIFLASH_OP_CODE_ENCRYPT_WRITE)
    {
      work_arg->ret = spi_flash_write_encrypted_manu(work_arg->op_arg.addr,
                                                     work_arg->op_arg.buffer,
                                                     work_arg->op_arg.size);
    }
  else
    {
      ferr("ERROR: op_code=%d is not supported\n", work_arg->op_code);
    }

  nxsem_post(&work_arg->sem);
}

/****************************************************************************
 * Name: esp32s3_async_op
 *
 * Description:
 *   Send operation code and arguments to workqueue so that workqueue do SPI
 *   Flash operation actually.
 *
 * Input Parameters:
 *   opcode - SPI flash work operation code
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32s3_async_op(enum spiflash_op_code_e opcode,
                            uint32_t addr,
                            const uint8_t *buffer,
                            uint32_t size,
                            uint32_t paddr)
{
  int ret;
  struct spiflash_work_arg work_arg =
  {
    .op_code = opcode,
    .op_arg =
    {
      .addr = addr,
      .buffer = (uint8_t *)buffer,
      .size = size,
      .paddr = paddr,
    },
    .sem = NXSEM_INITIALIZER(0, 0)
  };

  ret = nxmutex_lock(&g_work_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = work_queue(LPWORK, &g_work, esp32s3_spiflash_work, &work_arg, 0);
  if (ret == 0)
    {
      nxsem_wait(&work_arg.sem);
      ret = work_arg.ret;
    }

  nxmutex_unlock(&g_work_lock);

  return ret;
}
#endif

/****************************************************************************
 * Name: esp32s3_erase
 *
 * Description:
 *   Erase SPI Flash designated sectors.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - Number of blocks
 *
 * Returned Value:
 *   Erased blocks if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32s3_erase(struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks)
{
  ssize_t ret;
  uint32_t offset = startblock * MTD_ERASE_SIZE;
  uint32_t nbytes = nblocks * MTD_ERASE_SIZE;
  struct esp32s3_mtd_dev_s *priv = (struct esp32s3_mtd_dev_s *)dev;

  if ((offset > MTD_SIZE(priv)) || ((offset + nbytes) > MTD_SIZE(priv)))
    {
      return -EINVAL;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu)\n", __func__, dev, startblock, nblocks);

  finfo("spi_flash_erase_range(0x%x, %d)\n", offset, nbytes);
#endif

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
  if (stack_is_psram())
    {
      ret = esp32s3_async_op(SPIFLASH_OP_CODE_ERASE, offset, NULL,
                             nbytes, 0);
    }
  else
#endif
    {
      ret = spi_flash_erase_range(offset, nbytes);
    }

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nblocks;
    }
  else
    {
#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
      finfo("Failed to erase the flash range!\n");
#endif
      ret = -1;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32s3_read
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_read(struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, uint8_t *buffer)
{
  ssize_t ret;

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n",
        __func__, dev, offset, nbytes, buffer);

  finfo("spi_flash_read(0x%" PRIxOFF ", %p, %zu)\n", offset, buffer, nbytes);
#endif

  /* Acquire the mutex. */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
  if (stack_is_psram())
    {
      ret = esp32s3_async_op(SPIFLASH_OP_CODE_READ, offset,
                             buffer, nbytes, 0);
    }
  else
#endif
    {
      ret = spi_flash_read(offset, buffer, nbytes);
    }

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32s3_bread
 *
 * Description:
 *   Read data from designated blocks.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_bread(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * MTD_BLK_SIZE;
  uint32_t size = nblocks * MTD_BLK_SIZE;

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n",
        __func__, dev, startblock, nblocks, buffer);
#endif

  ret = esp32s3_read(dev, addr, size, buffer);
  if (ret == size)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32s3_read_decrypt
 *
 * Description:
 *   Read encrypted data and decrypt automatically from SPI Flash
 *   at designated address.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Read data bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_read_decrypt(struct mtd_dev_s *dev,
                                  off_t offset,
                                  size_t nbytes,
                                  uint8_t *buffer)
{
  ssize_t ret;

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n",
        __func__, dev, offset, nbytes, buffer);

  finfo("spi_flash_read_encrypted(0x%" PRIxOFF ", %p, %zu)\n",
        offset, buffer, nbytes);
#endif

  /* Acquire the mutex. */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
  if (stack_is_psram())
    {
      ret = esp32s3_async_op(SPIFLASH_OP_CODE_ENCRYPT_READ, offset,
                             buffer, nbytes, 0);
    }
  else
#endif
    {
      ret = spi_flash_read_encrypted(offset, buffer, nbytes);
    }

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32s3_bread_decrypt
 *
 * Description:
 *   Read encrypted data and decrypt automatically from designated blocks.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start block number, it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Read block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_bread_decrypt(struct mtd_dev_s *dev,
                                     off_t startblock,
                                     size_t nblocks,
                                     uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * MTD_BLK_SIZE;
  uint32_t size = nblocks * MTD_BLK_SIZE;

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n",
        __func__, dev, startblock, nblocks, buffer);
#endif

  ret = esp32s3_read_decrypt(dev, addr, size, buffer);
  if (ret == size)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif
  return ret;
}

/****************************************************************************
 * Name: esp32s3_writedata_encrypt
 *
 * Description:
 *   Write plaintext data to SPI Flash at designated address by SPI Flash
 *   hardware encryption, and written data in SPI Flash is ciphertext.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset, must be 32Bytes-aligned
 *   size   - data number, must be 32Bytes-aligned
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32s3_writedata_encrypt(struct mtd_dev_s *dev, off_t offset,
                                     uint32_t size, const uint8_t *buffer)
{
  ssize_t ret;

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n", __func__, dev, offset,
        size, buffer);
#endif

  DEBUGASSERT((offset % SPI_FLASH_ENCRYPT_MIN_SIZE) == 0);
  DEBUGASSERT((size % SPI_FLASH_ENCRYPT_MIN_SIZE) == 0);
  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
  if (stack_is_psram())
    {
      ret = esp32s3_async_op(SPIFLASH_OP_CODE_ENCRYPT_WRITE, offset,
                             buffer, size, 0);
    }
  else
#endif
    {
      ret = spi_flash_write_encrypted_manu(offset, buffer, size);
    }

  nxmutex_unlock(&g_lock);
#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif
  return ret;
}

/****************************************************************************
 * Name: esp32s3_write_encrypt
 *
 * Description:
 *   Write data to SPI Flash at designated address by SPI Flash hardware
 *   encryption.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset, must be 16Bytes-aligned
 *   nbytes - data number, must be 16Bytes-aligned
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Writen bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_write_encrypt(struct mtd_dev_s *dev, off_t offset,
                                     size_t nbytes, const uint8_t *buffer)
{
  ssize_t ret;
  size_t n;
  off_t addr;
  size_t wbytes;
  uint32_t step;
  uint8_t enc_buf[SPI_FLASH_ENCRYPT_UNIT_SIZE];

  if ((offset % SPI_FLASH_ENCRYPT_MIN_SIZE) ||
      (nbytes % SPI_FLASH_ENCRYPT_MIN_SIZE))
    {
      return -EINVAL;
    }
  else if (nbytes == 0)
    {
      return 0;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%x, %d, %p)\n", __func__, dev, offset, nbytes, buffer);
#endif

  for (n = 0; n < nbytes; n += step)
    {
      /* The temporary buffer need to be seperated into
       * 16-bytes, 32-bytes, 64-bytes(if supported).
       */

      addr = offset + n;
      if ((addr % 64) == 0 && (nbytes - n) >= 64)
        {
          wbytes = 64;
        }
      else if ((addr % 32) == 0 && (nbytes - n) >= 32)
        {
          wbytes = 32;
        }
      else
        {
          wbytes = 16;
        }

      memcpy(enc_buf, buffer + n, wbytes);
      step = wbytes;
      ret = esp32s3_writedata_encrypt(dev, addr, wbytes, enc_buf);
      if (ret < 0)
        {
          break;
        }
    }

  if (ret >= 0)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("esp32s3_write_encrypt()=%d\n", ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32s3_write
 *
 * Description:
 *   write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   dev    - MTD device data
 *   offset - target address offset
 *   nbytes - data number
 *   buffer - data buffer pointer
 *
 * Returned Value:
 *   Writen bytes if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_write(struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, const uint8_t *buffer)
{
  ssize_t ret;
  struct esp32s3_mtd_dev_s *priv = (struct esp32s3_mtd_dev_s *)dev;

  ASSERT(buffer);

  if ((offset > MTD_SIZE(priv)) || ((offset + nbytes) > MTD_SIZE(priv)))
    {
      return -EINVAL;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n",
        __func__, dev, offset, nbytes, buffer);

  finfo("spi_flash_write(0x%" PRIxOFF ", %p, %zu)\n",
        offset, buffer, nbytes);
#endif

  /* Acquire the mutex. */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
  if (stack_is_psram())
    {
      ret = esp32s3_async_op(SPIFLASH_OP_CODE_WRITE, offset,
                             buffer, nbytes, 0);
    }
  else
#endif
    {
      ret = spi_flash_write(offset, buffer, nbytes);
    }

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nbytes;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif
  return ret;
}

/****************************************************************************
 * Name: esp32s3_bwrite_encrypt
 *
 * Description:
 *   Write data to designated blocks by SPI Flash hardware encryption.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Writen block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_bwrite_encrypt(struct mtd_dev_s *dev,
                                      off_t startblock,
                                      size_t nblocks,
                                      const uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * MTD_BLK_SIZE;
  uint32_t size = nblocks * MTD_BLK_SIZE;

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n", __func__, dev, startblock,
        nblocks, buffer);

  finfo("spi_flash_write_encrypted(0x%x, %p, %" PRIu32 ")\n",
        addr, buffer, size);
#endif

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
  if (stack_is_psram())
    {
      ret = esp32s3_async_op(SPIFLASH_OP_CODE_ENCRYPT_WRITE, addr,
                             buffer, size, 0);
    }
  else
#endif
    {
      ret = spi_flash_write_encrypted_manu(addr, buffer, size);
    }

  nxmutex_unlock(&g_lock);
  if (ret == OK)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif
  return ret;
}

/****************************************************************************
 * Name: esp32s3_bwrite
 *
 * Description:
 *   Write data to designated blocks.
 *
 * Input Parameters:
 *   dev        - MTD device data
 *   startblock - start MTD block number,
 *                it is not equal to SPI Flash's block
 *   nblocks    - blocks number
 *   buffer     - data buffer pointer
 *
 * Returned Value:
 *   Writen block number if success or a negative value if fail.
 *
 ****************************************************************************/

static ssize_t esp32s3_bwrite(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, const uint8_t *buffer)
{
  ssize_t ret;
  uint32_t addr = startblock * MTD_BLK_SIZE;
  uint32_t size = nblocks * MTD_BLK_SIZE;

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s(%p, 0x%" PRIxOFF ", %zu, %p)\n", __func__, dev, startblock,
        nblocks, buffer);
#endif

  ret = esp32s3_write(dev, addr, size, buffer);
  if (ret == size)
    {
      ret = nblocks;
    }

#ifdef CONFIG_ESP32S3_STORAGE_MTD_DEBUG
  finfo("%s()=%d\n", __func__, ret);
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32s3_ioctl
 *
 * Description:
 *   Set/Get option to/from ESP32-S3 SPI Flash MTD device data.
 *
 * Input Parameters:
 *   dev - ESP32-S3 MTD device data
 *   cmd - operation command
 *   arg - operation argument
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static int esp32s3_ioctl(struct mtd_dev_s *dev, int cmd,
                         unsigned long arg)
{
  int ret = OK;
  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct esp32s3_mtd_dev_s *priv = (struct esp32s3_mtd_dev_s *)dev;
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              geo->blocksize    = MTD_BLK_SIZE;
              geo->erasesize    = MTD_ERASE_SIZE;
              geo->neraseblocks = MTD_SIZE(priv) / MTD_ERASE_SIZE;
              ret               = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32 \
                    " neraseblocks: %" PRId32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          struct esp32s3_mtd_dev_s *priv = (struct esp32s3_mtd_dev_s *)dev;
          struct partition_info_s *info = (struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = MTD_SIZE(priv) / MTD_BLK_SIZE;
              info->sectorsize  = MTD_BLK_SIZE;
              info->startsector = 0;
              info->parent[0]   = '\0';
            }
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          uint8_t *result = (uint8_t *)arg;
          *result = MTD_ERASED_STATE;

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
 * Name: esp32s3_set_bank
 *
 * Description:
 *   Set Ext-SRAM-Cache mmu mapping.
 *
 * Input Parameters:
 *   virt_bank - Beginning of the virtual bank
 *   phys_bank - Beginning of the physical bank
 *   ct        - Number of banks
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_set_bank(int virt_bank, int phys_bank, int ct)
{
  int ret;
  uint32_t vaddr = SOC_EXTRAM_DATA_LOW + MMU_PAGE_SIZE * virt_bank;
  uint32_t paddr = phys_bank * MMU_PAGE_SIZE;
#ifdef CONFIG_ESP32S3_SPI_FLASH_SUPPORT_PSRAM_STACK
  if (stack_is_psram())
    {
      ret = esp32s3_async_op(SPIFLASH_OP_CODE_SET_BANK, vaddr, NULL, ct,
                             paddr);
    }
  else
#endif
    {
      ret = cache_dbus_mmu_map(vaddr, paddr, ct);
    }

  DEBUGASSERT(ret == 0);
  UNUSED(ret);
}

/****************************************************************************
 * Name: esp32s3_spiflash_alloc_mtdpart
 *
 * Description:
 *   Allocate an MTD partition from the ESP32-S3 SPI Flash.
 *
 * Input Parameters:
 *   mtd_offset - MTD Partition offset from the base address in SPI Flash.
 *   mtd_size   - Size for the MTD partition.
 *   encrypted  - Flag indicating whether the newly allocated partition will
 *                have its content encrypted.
 *
 * Returned Value:
 *   ESP32-S3 SPI Flash MTD data pointer if success or NULL if fail.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32s3_spiflash_alloc_mtdpart(uint32_t mtd_offset,
                                                 uint32_t mtd_size,
                                                 bool encrypted)
{
  const struct esp32s3_mtd_dev_s *priv;
  const esp_rom_spiflash_chip_t *chip;
  struct mtd_dev_s *mtd_part;
  uint32_t blocks;
  uint32_t startblock;
  uint32_t size;

  if (encrypted)
    {
      priv = &g_esp32s3_spiflash_encrypt;
    }
  else
    {
      priv = &g_esp32s3_spiflash;
    }

  chip = &(*priv->data)->chip;

  finfo("ESP32-S3 SPI Flash information:\n");
  finfo("\tID = 0x%" PRIx32 "\n", chip->device_id);
  finfo("\tStatus mask = 0x%" PRIx32 "\n", chip->status_mask);
  finfo("\tChip size = %" PRId32 " KB\n", chip->chip_size / 1024);
  finfo("\tPage size = %" PRId32 " B\n", chip->page_size);
  finfo("\tSector size = %" PRId32 " KB\n", chip->sector_size / 1024);
  finfo("\tBlock size = %" PRId32 " KB\n", chip->block_size / 1024);

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

  finfo("\tMTD offset = 0x%" PRIx32 "\n", mtd_offset);
  finfo("\tMTD size = 0x%" PRIx32 "\n", size);

  startblock = MTD_SIZE2BLK(priv, mtd_offset);
  blocks = MTD_SIZE2BLK(priv, size);

  mtd_part = mtd_partition((struct mtd_dev_s *)&priv->mtd, startblock,
                           blocks);
  if (!mtd_part)
    {
      ferr("ERROR: Failed to create MTD partition\n");
      return NULL;
    }

  return mtd_part;
}

/****************************************************************************
 * Name: esp32s3_spiflash_mtd
 *
 * Description:
 *   Get SPI Flash MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32-S3 SPI Flash MTD pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32s3_spiflash_mtd(void)
{
  struct esp32s3_mtd_dev_s *priv =
      (struct esp32s3_mtd_dev_s *)&g_esp32s3_spiflash;

  return &priv->mtd;
}

/****************************************************************************
 * Name: esp32s3_spiflash_encrypt_mtd
 *
 * Description:
 *   Get SPI Flash encryption MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   SPI Flash encryption MTD pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32s3_spiflash_encrypt_mtd(void)
{
  struct esp32s3_mtd_dev_s *priv =
      (struct esp32s3_mtd_dev_s *)&g_esp32s3_spiflash_encrypt;

  return &priv->mtd;
}
