/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_boot_image.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <unistd.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>

#include "xtensa_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_APP_LOAD_HEADER_MAGIC 0xace637d3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_load_header_s
{
  uint32_t header_magic;
  uint32_t entry_addr;
  uint32_t iram_dest_addr;
  uint32_t iram_flash_offset;
  uint32_t iram_size;
  uint32_t dram_dest_addr;
  uint32_t dram_flash_offset;
  uint32_t dram_size;
  uint32_t lp_rtc_iram_dest_addr;
  uint32_t lp_rtc_iram_flash_offset;
  uint32_t lp_rtc_iram_size;
  uint32_t lp_rtc_dram_dest_addr;
  uint32_t lp_rtc_dram_flash_offset;
  uint32_t lp_rtc_dram_size;
  uint32_t irom_map_addr;
  uint32_t irom_flash_offset;
  uint32_t irom_size;
  uint32_t drom_map_addr;
  uint32_t drom_flash_offset;
  uint32_t drom_size;
  uint32_t reserved[4];
};

struct esp32_boot_loader_args_s
{
  uint32_t entry_addr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32_load_ram_segment(int fd, uint32_t flash_offset,
                                  uint32_t dest_addr, uint32_t size,
                                  FAR const char *name);
static void IRAM_ATTR esp32_boot_loader_stub(void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_load_ram_segment
 *
 * Description:
 *   Load a RAM-resident segment from the application image into its
 *   destination address.
 *
 ****************************************************************************/

static int esp32_load_ram_segment(int fd, uint32_t flash_offset,
                                  uint32_t dest_addr, uint32_t size,
                                  FAR const char *name)
{
  uint8_t *dest;
  off_t read_offset;
  ssize_t nread;
  size_t remaining;

  if (size == 0)
    {
      return OK;
    }

  if (dest_addr == 0)
    {
      ferr("ERROR: Invalid %s destination address\n", name);
      return -EINVAL;
    }

  finfo("Loading %-12s: dst=0x%08" PRIx32 " off=0x%08" PRIx32
        " size=0x%08" PRIx32 "\n",
        name, dest_addr, flash_offset, size);

  dest = (uint8_t *)(uintptr_t)dest_addr;
  read_offset = (off_t)flash_offset;
  remaining = (size_t)size;

  while (remaining > 0)
    {
      nread = pread(fd, dest, remaining, read_offset);
      if (nread < 0)
        {
          ferr("ERROR: Failed to read %s segment: %d\n", name, errno);
          return -errno;
        }

      if (nread == 0)
        {
          ferr("ERROR: Unexpected EOF while reading %s segment\n", name);
          return -EIO;
        }

      dest += nread;
      read_offset += nread;
      remaining -= nread;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_boot_loader_stub
 *
 * Description:
 *   This function resides in IRAM and is responsible for switching MMU
 *   mappings and jumping to the new application.
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_boot_loader_stub(void *arg)
{
  struct esp32_boot_loader_args_s *args =
      (struct esp32_boot_loader_args_s *)arg;
  void (*entry_point)(void) = (void (*)(void))args->entry_addr;

  /* Disable interrupts */

  up_irq_disable();

  /* For location-fixed MCUboot images, ROM mappings are handled by
   * the image startup path.
   */

  /* Jump to entry point */

  entry_point();

  /* Should never reach here */

  PANIC();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_boot_image
 *
 * Description:
 *   Boot a new application image.
 *
 * Input Parameters:
 *   path     - Path to the image file/partition
 *   hdr_size - Size of the prepended image header (e.g. MCUboot/nxboot)
 *
 * Returned Value:
 *   Does not return on success; returns error code on failure.
 *
 ****************************************************************************/

int board_boot_image(FAR const char *path, uint32_t hdr_size)
{
  int fd;
  int ret;
  struct esp32_load_header_s load_header;
  struct esp32_boot_loader_args_s args =
    {
      0
    };

  /* Legacy/simple boot image formats are not supported */

#if defined(CONFIG_ESP32_APP_FORMAT_LEGACY) || \
    defined(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  ferr("ERROR: Unsupported image format for board_boot_image\n");
  return -ENOTSUP;
#endif

  /* Open the image file */

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", path, errno);
      return -errno;
    }

  /* Skip image header if present (e.g. MCUboot/nxboot header) */

  if (hdr_size > 0)
    {
      if (lseek(fd, hdr_size, SEEK_SET) < 0)
        {
          ferr("ERROR: Failed to seek load header: %d\n", errno);
          close(fd);
          return -errno;
        }
    }

  /* Read image load header generated for MCUboot app format */

  ret = read(fd, &load_header, sizeof(struct esp32_load_header_s));
  if (ret != sizeof(struct esp32_load_header_s))
    {
      ferr("ERROR: Failed to read image load header: %d\n", errno);
      close(fd);
      return -errno;
    }

  if (load_header.header_magic != ESP32_APP_LOAD_HEADER_MAGIC)
    {
      ferr("ERROR: Invalid load header magic: 0x%08" PRIx32 "\n",
           load_header.header_magic);
      close(fd);
      return -EINVAL;
    }

  /* RAM sections are normally loaded by the ROM bootloader. Since
   * board_boot_image() chain-boots directly, load these sections here.
   */

  ret = esp32_load_ram_segment(fd, load_header.iram_flash_offset,
                               load_header.iram_dest_addr,
                               load_header.iram_size, "iram");
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_load_ram_segment(fd, load_header.dram_flash_offset,
                               load_header.dram_dest_addr,
                               load_header.dram_size, "dram");
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_load_ram_segment(fd, load_header.lp_rtc_iram_flash_offset,
                               load_header.lp_rtc_iram_dest_addr,
                               load_header.lp_rtc_iram_size, "lp_rtc_iram");
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_load_ram_segment(fd, load_header.lp_rtc_dram_flash_offset,
                               load_header.lp_rtc_dram_dest_addr,
                               load_header.lp_rtc_dram_size, "lp_rtc_dram");
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  args.entry_addr = load_header.entry_addr;

  close(fd);

  finfo("Booting image: entry=0x%08" PRIx32 "\n", args.entry_addr);

  /* Invoke IRAM loader stub */

  esp32_boot_loader_stub(&args);

  return 0;
}
