/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_core.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "bcmf_core.h"
#include "bcmf_sdio.h"

#include "bcmf_sdio_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Agent registers (common for every core) */

#define BCMA_IOCTL           0x0408 /* IO control */
#define BCMA_IOST            0x0500 /* IO status */
#define BCMA_RESET_CTL       0x0800 /* Reset control */
#define BCMA_RESET_ST        0x0804

#define BCMA_IOCTL_CLK       0x0001
#define BCMA_IOCTL_FGC       0x0002
#define BCMA_IOCTL_CORE_BITS 0x3FFC
#define BCMA_IOCTL_PME_EN    0x4000
#define BCMA_IOCTL_BIST_EN   0x8000

#define BCMA_IOST_CORE_BITS  0x0FFF
#define BCMA_IOST_DMA64      0x1000
#define BCMA_IOST_GATED_CLK  0x2000
#define BCMA_IOST_BIST_ERROR 0x4000
#define BCMA_IOST_BIST_DONE  0x8000

#define BCMA_RESET_CTL_RESET 0x0001

/* SOCSRAM core registers */

#define SOCSRAM_BANKX_INDEX  ((uint32_t) (0x18004000 + 0x10) )
#define SOCSRAM_BANKX_PDA    ((uint32_t) (0x18004000 + 0x44) )

/* Transfer size properties */

#define BCMF_UPLOAD_TRANSFER_SIZE  (64 * 256)

/* Define this to validate uploaded materials */

/*  #define DBG_VALIDATE_UPLOAD */

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef DBG_VALIDATE_UPLOAD
static uint8_t compare_buffer[BCMF_UPLOAD_TRANSFER_SIZE];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcmf_core_set_backplane_window(FAR struct bcmf_sdio_dev_s *sbus,
                                          uint32_t address);
static int bcmf_upload_binary(FAR struct bcmf_sdio_dev_s *sbusv,
                              uint32_t address, uint8_t *buf,
                              unsigned int len);
static int bcmf_upload_nvram(FAR struct bcmf_sdio_dev_s *sbus);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_core_set_backplane_window(FAR struct bcmf_sdio_dev_s *sbus,
                                   uint32_t address)
{
  int ret;
  int i;

  address &= ~SBSDIO_SB_OFT_ADDR_MASK;

  for (i = 1; i < 4; i++)
    {
      uint8_t addr_part = (address >> (8*i)) & 0xff;
      uint8_t cur_addr_part = (sbus->backplane_current_addr >> (8*i)) & 0xff;

      if (addr_part != cur_addr_part)
        {
          /* Update current backplane base address */

          ret = bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_SBADDRLOW + i - 1,
                  addr_part);

          if (ret != OK)
            {
              return ret;
            }

          sbus->backplane_current_addr &= ~(0xff << (8*i));
          sbus->backplane_current_addr |= addr_part << (8*i);
        }
    }

  return OK;
}

int bcmf_upload_binary(FAR struct bcmf_sdio_dev_s *sbus, uint32_t address,
                       uint8_t *buf, unsigned int len)
{
  unsigned int size;

#ifdef DBG_VALIDATE_UPLOAD
  uint32_t      validate_address = address;
  uint8_t       *validate_buffer = buf;
  unsigned int  validate_len     = len;
#endif

  while (len > 0)
    {
      /* Set the backplane window to include the start address */

      int ret = bcmf_core_set_backplane_window(sbus, address);
      if (ret != OK)
        {
          wlerr("Backplane setting failed at %08" PRIx32 "\n", address);
          return ret;
        }

      if (len > BCMF_UPLOAD_TRANSFER_SIZE)
        {
          size = BCMF_UPLOAD_TRANSFER_SIZE;
        }
      else
        {
           size = len;
        }

      /* Transfer firmware data */

      ret = bcmf_transfer_bytes(sbus, true, 1,
                                address & SBSDIO_SB_OFT_ADDR_MASK, buf,
                                size);
      if (ret != OK)
        {
          wlerr("transfer failed %d %" PRIx32 " %d\n", ret, address, size);
          return ret;
        }

      len       -= size;
      address   += size;
      buf       += size;
    }

#ifdef DBG_VALIDATE_UPLOAD
  wlwarn("Validating....\n");
  while (validate_len > 0)
    {
      /* Set the backplane window to include the start address */

      int ret = bcmf_core_set_backplane_window(sbus, validate_address);
      if (ret != OK)
        {
          wlerr("Backplane setting failed at %08x\n", validate_address);
          return ret;
        }

      if (validate_len > BCMF_UPLOAD_TRANSFER_SIZE)
        {
          size = BCMF_UPLOAD_TRANSFER_SIZE;
        }
      else
        {
          size = validate_len;
        }

      /* Transfer firmware data */

      ret = bcmf_transfer_bytes(sbus, false, 1,
                                validate_address & SBSDIO_SB_OFT_ADDR_MASK,
                                compare_buffer, size);
      if (ret != OK)
        {
          wlerr("validate transfer failed %d %x %d\n", ret, validate_address,
                size);
          return ret;
        }

      if (memcmp(validate_buffer, compare_buffer, size))
        {
          wlerr("Match failed at address base %08x\n", validate_address);
          return -EILSEQ;
        }

      validate_len     -= size;
      validate_address += size;
      validate_buffer  += size;
    }

  wlwarn("Validation passed\n");
  #endif

  return OK;
}

#ifdef CONFIG_IEEE80211_BROADCOM_FWFILES
int bcmf_upload_file(FAR struct bcmf_sdio_dev_s *sbus, uint32_t address,
                     FAR const char *path)
{
  struct file finfo;
  FAR uint8_t *buf;
  size_t total_read;
  ssize_t nread;
  int ret;

  /* Open the file in the detached state */

  ret = file_open(&finfo, path, O_RDONLY | O_BINARY);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to open the FILE MTD file %s: %d\n", path, ret);
      return ret;
    }

  /* Allocate an I/O buffer */

  buf = (FAR uint8_t *)kmm_malloc(BCMF_UPLOAD_TRANSFER_SIZE);
  if (buf == NULL)
    {
      wlerr("ERROR: Failed allocate an I/O buffer\n");
      ret = -ENOMEM;
      goto errout_with_file;
    }

  /* Loop until the firmware has been loaded */

  do
    {
      /* Set the backplane window to include the start address */

      nread = file_read(&finfo, buf, BCMF_UPLOAD_TRANSFER_SIZE);
      if (nread < 0)
        {
          ret = (int)nread;
          wlerr("ERROR: Failed to read file: %d\n", ret);
          goto errout_with_buf;
        }

      if (nread == 0)
        {
          break;
        }

      wlinfo("Read %ld bytes\n", (long)nread);

      ret = bcmf_core_set_backplane_window(sbus, address);
      if (ret < 0)
        {
          wlerr("ERROR: bcmf_core_set_backplane_window() failed: %d\n", ret);
          goto errout_with_buf;
        }

      total_read = nread;

      /* Transfer firmware data */

      ret = bcmf_transfer_bytes(sbus, true, 1,
                                address & SBSDIO_SB_OFT_ADDR_MASK, buf,
                                total_read);
      if (ret < 0)
        {
          wlerr("ERROR: Transfer failed address=%lx total_read=%lu: %d\n",
                (unsigned long)address, (unsigned long)total_read, ret);
          goto errout_with_buf;
        }

      address += total_read;
    }
  while (nread == BCMF_UPLOAD_TRANSFER_SIZE);

  file_close(&finfo);
  kmm_free(buf);

  wlinfo("Upload complete\n");
  return OK;

errout_with_buf:
  kmm_free(buf);

errout_with_file:
  file_close(&finfo);
  return ret;
}
#endif

int bcmf_upload_nvram(FAR struct bcmf_sdio_dev_s *sbus)
{
  int ret;
  uint32_t nvram_sz;
  uint32_t token;

  /* Round up the size of the image */

  nvram_sz = (*sbus->chip->nvram_image_size + 63) & (-64);

  wlinfo("nvram size is %" PRId32 " %d bytes\n",
         nvram_sz, *sbus->chip->nvram_image_size);

  /* Write image */

  ret = bcmf_upload_binary(sbus, sbus->chip->ram_size - 4 - nvram_sz,
                           sbus->chip->nvram_image,
                           *sbus->chip->nvram_image_size);
  if (ret != OK)
    {
      return ret;
    }

  /* Generate length token */

  token = nvram_sz / 4;
  token = (~token << 16) | (token & 0x0000ffff);

  /* Write the length token to the last word */

  ret = bcmf_write_sbreg(sbus, sbus->chip->ram_size - 4,
                         (FAR uint8_t *)&token, 4);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_read_sbreg
 ****************************************************************************/

int bcmf_read_sbreg(FAR struct bcmf_sdio_dev_s *sbus, uint32_t address,
                    FAR uint8_t *reg, unsigned int len)
{
  int ret = bcmf_core_set_backplane_window(sbus, address);
  if (ret != OK)
    {
      return ret;
    }

  address &= SBSDIO_SB_OFT_ADDR_MASK;

  /* Map to 32-bit access if len == 4 */

  if (len == 4)
    {
      address |= SBSDIO_SB_ACCESS_2_4B_FLAG;
    }

  return bcmf_transfer_bytes(sbus, false, 1, address, reg, len);
}

/****************************************************************************
 * Name: bcmf_write_sbreg
 ****************************************************************************/

int bcmf_write_sbreg(FAR struct bcmf_sdio_dev_s *sbus, uint32_t address,
                     FAR uint8_t *reg, unsigned int len)
{
  int ret = bcmf_core_set_backplane_window(sbus, address);
  if (ret != OK)
    {
      return ret;
    }

  address &= SBSDIO_SB_OFT_ADDR_MASK;

  /* Map to 32-bit access if len == 4 */

  if (len == 4)
    {
      address |= SBSDIO_SB_ACCESS_2_4B_FLAG;
    }

  return bcmf_transfer_bytes(sbus, true, 1, address, reg, len);
}

/****************************************************************************
 * Name: bcmf_core_upload_firmware
 ****************************************************************************/

int bcmf_core_upload_firmware(FAR struct bcmf_sdio_dev_s *sbus)
{
  int ret;

  wlinfo("upload firmware\n");

  /* Disable ARMCM3 core and reset SOCRAM core to set device in firmware
   * upload mode
   */

  bcmf_core_disable(sbus, WLAN_ARMCM3_CORE_ID);
  bcmf_core_reset(sbus, SOCSRAM_CORE_ID);

  /* Do chip specific initialization */

  if (sbus->cur_chip_id == SDIO_DEVICE_ID_BROADCOM_43430)
    {
      /* Disable remap for SRAM_3. Only for 4343x */

      bcmf_write_sbregw(sbus, SOCSRAM_BANKX_INDEX, 0x3);
      bcmf_write_sbregw(sbus, SOCSRAM_BANKX_PDA, 0);
    }

  up_mdelay(50);

  /* Flash chip firmware */

#ifdef CONFIG_IEEE80211_BROADCOM_FWFILES
  ret = bcmf_upload_file(sbus, 0, CONFIG_IEEE80211_BROADCOM_FWFILENAME);
#else
  wlinfo("firmware size is %d bytes\n", *sbus->chip->firmware_image_size);

  ret = bcmf_upload_binary(sbus, 0, sbus->chip->firmware_image,
                           *sbus->chip->firmware_image_size);
#endif

  if (ret < 0)
    {
      wlerr("ERROR: Failed to upload firmware\n");
      return ret;
    }

  /* Flash NVRAM configuration file */

  wlinfo("upload nvram configuration\n");
  ret = bcmf_upload_nvram(sbus);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to upload NVRAM\n");
      return ret;
    }

  /* Firmware upload done, restart ARMCM3 core */

  up_mdelay(10);
  bcmf_core_reset(sbus, WLAN_ARMCM3_CORE_ID);

  /* Check ARMCM3 core is running */

  up_mdelay(10);
  if (!bcmf_core_isup(sbus, WLAN_ARMCM3_CORE_ID))
    {
      wlerr("Cannot start ARMCM3 core\n");
      return -ETIMEDOUT;
    }

  return OK;
}

bool bcmf_core_isup(FAR struct bcmf_sdio_dev_s *sbus, unsigned int core)
{
  uint32_t value = 0;
  uint32_t base;

  if (core >= MAX_CORE_ID)
    {
      wlerr("Invalid core id %d\n", core);
      return false;
    }

  base = sbus->chip->core_base[core];

  bcmf_read_sbregw(sbus, base + BCMA_IOCTL, &value);

  if ((value & (BCMA_IOCTL_FGC | BCMA_IOCTL_CLK)) != BCMA_IOCTL_CLK)
    {
      return false;
    }

  bcmf_read_sbregw(sbus, base + BCMA_RESET_CTL, &value);

  return (value & BCMA_RESET_CTL_RESET) == 0;
}

void bcmf_core_disable(FAR struct bcmf_sdio_dev_s *sbus, unsigned int core)
{
  uint8_t value;

  if (core >= MAX_CORE_ID)
    {
      wlerr("Invalid core id %d\n", core);
      return;
    }

  uint32_t base = sbus->chip->core_base[core];

  /* Check if core is already in reset state */

  bcmf_read_sbregb(sbus, base + BCMA_RESET_CTL, &value);

  if ((value & BCMA_RESET_CTL_RESET) != 0)
    {
      /* Core already disabled */

      return;
    }

  /*  Ensure no backplane operation is pending */

  up_mdelay(10);

  /* Set core in reset state */

  bcmf_write_sbregb(sbus, base + BCMA_RESET_CTL, BCMA_RESET_CTL_RESET);
  up_udelay(1);

  /* Write 0 to the IO control and read it back */

  bcmf_write_sbregb(sbus, base + BCMA_IOCTL, 0);
  bcmf_read_sbregb(sbus, base + BCMA_IOCTL, &value);
  up_udelay(10);
}

void bcmf_core_reset(FAR struct bcmf_sdio_dev_s *sbus, unsigned int core)
{
  uint32_t value;
  uint32_t base;

  if (core >= MAX_CORE_ID)
    {
      wlerr("Invalid core id %d\n", core);
      return;
    }

  base = sbus->chip->core_base[core];

  /* Put core in reset state */

  bcmf_core_disable(sbus, core);

  /* Run initialization sequence */

  bcmf_write_sbregb(sbus, base + BCMA_IOCTL,
                    BCMA_IOCTL_FGC | BCMA_IOCTL_CLK);
  bcmf_read_sbregw(sbus, base + BCMA_IOCTL, &value);

  bcmf_write_sbregb(sbus, base + BCMA_RESET_CTL, 0);
  bcmf_read_sbregw(sbus, base + BCMA_RESET_CTL, &value);

  up_udelay(1);

  bcmf_write_sbregb(sbus, base + BCMA_IOCTL, BCMA_IOCTL_CLK);
  bcmf_read_sbregw(sbus, base + BCMA_IOCTL, &value);

  up_udelay(1);
}
