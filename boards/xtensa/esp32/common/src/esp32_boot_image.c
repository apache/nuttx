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
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include "xtensa_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_APP_LOAD_HEADER_MAGIC 0xace637d3
#define ESP32_LOAD_SEGMENT_COUNT    4
#define ESP32_BOOTLOADER_STACK_SIZE 2048

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

struct esp32_boot_loader_segment_s
{
  uint32_t dest_addr;
  uint32_t size;
  FAR uint8_t *buffer;
};

struct esp32_boot_loader_args_s
{
  uint32_t entry_addr;
  uintptr_t stack_top;
  struct esp32_boot_loader_segment_s segments[ESP32_LOAD_SEGMENT_COUNT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool esp32_ranges_overlap(uintptr_t start1, size_t size1,
                                 uintptr_t start2, size_t size2);
static int esp32_prepare_ram_segment(int fd, uint32_t flash_offset,
                                     uint32_t dest_addr, uint32_t size,
                                     FAR const char *name,
                                     FAR struct esp32_boot_loader_args_s
                                     *args,
                                     int index);
static void esp32_release_ram_segments(FAR struct esp32_boot_loader_args_s
                                       *args);
static void RTC_IRAM_ATTR esp32_copy_segment(FAR const void *src,
                                             uint32_t dest_addr,
                                             uint32_t size);
static void RTC_IRAM_ATTR esp32_boot_loader_stub(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32_boot_loader_args_s g_boot_loader_args;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_ranges_overlap
 *
 * Description:
 *   Return true when two address ranges overlap.
 *
 ****************************************************************************/

static bool esp32_ranges_overlap(uintptr_t start1, size_t size1,
                                 uintptr_t start2, size_t size2)
{
  uintptr_t end1 = start1 + size1;
  uintptr_t end2 = start2 + size2;

  return (start1 < end2) && (start2 < end1);
}

/****************************************************************************
 * Name: esp32_prepare_ram_segment
 *
 * Description:
 *   Read one RAM segment into a temporary buffer that is copied by the final
 *   RTC loader stage.
 *
 ****************************************************************************/

static int esp32_prepare_ram_segment(int fd, uint32_t flash_offset,
                                     uint32_t dest_addr, uint32_t size,
                                     FAR const char *name,
                                     FAR struct esp32_boot_loader_args_s
                                     *args,
                                     int index)
{
  FAR uint8_t *buffer;
  off_t read_offset;
  ssize_t nread;
  size_t remaining;

  DEBUGASSERT(index >= 0 && index < ESP32_LOAD_SEGMENT_COUNT);

  if (size == 0)
    {
      args->segments[index].dest_addr = 0;
      args->segments[index].size      = 0;
      args->segments[index].buffer    = NULL;
      return OK;
    }

  if (dest_addr == 0)
    {
      ferr("ERROR: Invalid %s destination address\n", name);
      return -EINVAL;
    }

  buffer = kmm_malloc(size);
  if (buffer == NULL)
    {
      ferr("ERROR: Failed to allocate %s preload buffer (%" PRIu32 ")\n",
           name, size);
      return -ENOMEM;
    }

  finfo("Preloading %-9s: dst=0x%08" PRIx32 " off=0x%08" PRIx32
        " size=0x%08" PRIx32 "\n",
        name, dest_addr, flash_offset, size);

  read_offset = (off_t)flash_offset;
  remaining = (size_t)size;

  while (remaining > 0)
    {
      nread = pread(fd, buffer + (size - remaining), remaining, read_offset);
      if (nread < 0)
        {
          ferr("ERROR: Failed to read %s segment: %d\n", name, errno);
          kmm_free(buffer);
          return -errno;
        }

      if (nread == 0)
        {
          ferr("ERROR: Unexpected EOF while reading %s segment\n", name);
          kmm_free(buffer);
          return -EIO;
        }

      read_offset += nread;
      remaining -= nread;
    }

  if (esp32_ranges_overlap((uintptr_t)buffer, size,
                           (uintptr_t)dest_addr, size))
    {
      ferr("ERROR: %s preload buffer overlaps destination\n", name);
      kmm_free(buffer);
      return -EFAULT;
    }

  args->segments[index].dest_addr = dest_addr;
  args->segments[index].size      = size;
  args->segments[index].buffer    = buffer;
  return OK;
}

/****************************************************************************
 * Name: esp32_release_ram_segments
 *
 * Description:
 *   Free preloaded segment buffers used by the boot loader handoff.
 *
 ****************************************************************************/

static void esp32_release_ram_segments(FAR struct esp32_boot_loader_args_s
                                       *args)
{
  int i;

  for (i = 0; i < ESP32_LOAD_SEGMENT_COUNT; i++)
    {
      if (args->segments[i].buffer != NULL)
        {
          kmm_free(args->segments[i].buffer);
          args->segments[i].buffer = NULL;
        }
    }
}

/****************************************************************************
 * Name: esp32_copy_segment
 *
 * Description:
 *   Copy one preloaded segment into its destination address.
 *
 ****************************************************************************/

static void RTC_IRAM_ATTR esp32_copy_segment(FAR const void *src,
                                             uint32_t dest_addr,
                                             uint32_t size)
{
  FAR const uint8_t *s = (FAR const uint8_t *)src;
  FAR uint8_t *d = (FAR uint8_t *)(uintptr_t)dest_addr;

  while (size-- > 0)
    {
      *d++ = *s++;
    }
}

/****************************************************************************
 * Name: esp32_boot_loader_stub
 *
 * Description:
 *   Final loader stage. The routine executes from RTC fast memory and copies
 *   preloaded RAM segments into their target addresses just before jumping
 *   to the new image entry point.
 *
 ****************************************************************************/

static void RTC_IRAM_ATTR esp32_boot_loader_stub(void)
{
  FAR const struct esp32_boot_loader_args_s *args = &g_boot_loader_args;
  register uintptr_t stack_top = args->stack_top;
  register uint32_t entry_addr = args->entry_addr;
  register FAR const void *dram_src = args->segments[3].buffer;
  register uint32_t dram_dst = args->segments[3].dest_addr;
  register uint32_t dram_size = args->segments[3].size;

  /* Disable interrupts */

  up_irq_disable();

  /* Use a dedicated handoff stack that does not overlap the DRAM segment
   * destination copied in the final step.
   */

  __asm__ __volatile__("mov sp, %0" : : "r"(stack_top));

  /* Copy IRAM and RTC sections first, then DRAM last. This avoids executing
   * from memory that can be overwritten during the chain-boot transition.
   */

  esp32_copy_segment(args->segments[0].buffer, args->segments[0].dest_addr,
                     args->segments[0].size);
  esp32_copy_segment(args->segments[1].buffer, args->segments[1].dest_addr,
                     args->segments[1].size);
  esp32_copy_segment(args->segments[2].buffer, args->segments[2].dest_addr,
                     args->segments[2].size);
  esp32_copy_segment(dram_src, dram_dst, dram_size);

  /* Jump using a register-held entry address to avoid relying on DRAM. */

  __asm__ __volatile__("jx %0" : : "r"(entry_addr));

  /* Should never return */

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
  uintptr_t stack_buf;
  struct esp32_load_header_s load_header;

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

  memset(&g_boot_loader_args, 0, sizeof(g_boot_loader_args));
  g_boot_loader_args.entry_addr = load_header.entry_addr;

  /* Preload segments into temporary buffers. They are copied to destination
   * addresses by the final RTC-resident loader stage.
   */

  ret = esp32_prepare_ram_segment(fd, load_header.iram_flash_offset,
                                  load_header.iram_dest_addr,
                                  load_header.iram_size, "iram",
                                  &g_boot_loader_args, 0);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_prepare_ram_segment(fd, load_header.lp_rtc_iram_flash_offset,
                                  load_header.lp_rtc_iram_dest_addr,
                                  load_header.lp_rtc_iram_size,
                                  "lp_rtc_iram", &g_boot_loader_args, 1);
  if (ret < 0)
    {
      esp32_release_ram_segments(&g_boot_loader_args);
      close(fd);
      return ret;
    }

  ret = esp32_prepare_ram_segment(fd, load_header.lp_rtc_dram_flash_offset,
                                  load_header.lp_rtc_dram_dest_addr,
                                  load_header.lp_rtc_dram_size,
                                  "lp_rtc_dram", &g_boot_loader_args, 2);
  if (ret < 0)
    {
      esp32_release_ram_segments(&g_boot_loader_args);
      close(fd);
      return ret;
    }

  ret = esp32_prepare_ram_segment(fd, load_header.dram_flash_offset,
                                  load_header.dram_dest_addr,
                                  load_header.dram_size, "dram",
                                  &g_boot_loader_args, 3);
  if (ret < 0)
    {
      esp32_release_ram_segments(&g_boot_loader_args);
      close(fd);
      return ret;
    }

  stack_buf = (uintptr_t)kmm_malloc(ESP32_BOOTLOADER_STACK_SIZE);
  if (stack_buf == 0)
    {
      ferr("ERROR: Failed to allocate boot loader stack\n");
      esp32_release_ram_segments(&g_boot_loader_args);
      close(fd);
      return -ENOMEM;
    }

  if (esp32_ranges_overlap(stack_buf, ESP32_BOOTLOADER_STACK_SIZE,
                           load_header.dram_dest_addr,
                           load_header.dram_size))
    {
      ferr("ERROR: Boot loader stack overlaps DRAM destination\n");
      esp32_release_ram_segments(&g_boot_loader_args);
      kmm_free((FAR void *)stack_buf);
      close(fd);
      return -EFAULT;
    }

  g_boot_loader_args.stack_top =
    (stack_buf + ESP32_BOOTLOADER_STACK_SIZE) & ~(uintptr_t)0xf;

  close(fd);

  finfo("Booting image: entry=0x%08" PRIx32 "\n",
        g_boot_loader_args.entry_addr);

  /* Invoke final RTC loader stub. */

  esp32_boot_loader_stub();

  return 0;
}
