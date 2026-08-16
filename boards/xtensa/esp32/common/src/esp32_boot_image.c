/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_boot_image.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/board.h>
#include <nuttx/cache.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>

#include "xtensa_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_APP_LOAD_HEADER_MAGIC   0xace637d3
#define ESP32_BOOTLOADER_STACK_SIZE   2048
#define ESP32_BOOTLOADER_STUB_ALIGN   16
#define ESP32_DRAM_LOW                0x3ffae000
#define ESP32_DRAM_HIGH               0x40000000
#define ESP32_IRAM_LOW                0x40080000
#define ESP32_IRAM_HIGH               0x400aa000
#define ESP32_RTC_IRAM_LOW            0x400c0000
#define ESP32_RTC_IRAM_HIGH           0x400c2000
#define ESP32_RTC_DATA_LOW            0x50000000
#define ESP32_RTC_DATA_HIGH           0x50002000
#define ESP32_BOOTLOADER_STUB_SECTION \
  __attribute__((section(".iram1.bootstub")))

#define ESP32_ALIGN_UP(value, align) \
  (((uint32_t)(value) + ((uint32_t)(align) - 1)) & ~((uint32_t)(align) - 1))
#define ESP32_ALIGN_DOWN(value, align) \
  ((uint32_t)(value) & ~((uint32_t)(align) - 1))

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
  uint32_t stack_top;
  uint32_t iram_src;
  uint32_t iram_dest_addr;
  uint32_t iram_size;
  uint32_t dram_src;
  uint32_t dram_dest_addr;
  uint32_t dram_size;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR const void *bootloader_mmap(uint32_t src_addr, uint32_t size);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool esp32_region_contains(uint32_t low, uint32_t high,
                                  uint32_t start, uint32_t size);
static int esp32_get_partition_info(int fd, FAR uint32_t *offset,
                                    FAR uint32_t *size);
static int esp32_flash_offset_add(uint32_t base, uint32_t offset,
                                  FAR uint32_t *result);
static int esp32_include_flash_segment(uint32_t offset, uint32_t size,
                                       FAR uint32_t *map_start,
                                       FAR uint32_t *map_end);
static int esp32_read_load_header(int fd, uint32_t image_offset,
                                  FAR struct esp32_load_header_s *header);
static int esp32_prepare_handoff_layout(
              FAR const struct esp32_load_header_s *load_header,
              uint32_t stub_size,
              FAR uint32_t *stub_dst,
              FAR uint32_t *args_addr,
              FAR uint32_t *stack_top);
static void IRAM_ATTR esp32_copy_segment(FAR const void *src,
                                         uint32_t dest_addr,
                                         uint32_t size);
static void ESP32_BOOTLOADER_STUB_SECTION __attribute__((used, noinline,
                                                         naked))
            esp32_boot_loader_stub(
              FAR const struct esp32_boot_loader_args_s *args);
static void ESP32_BOOTLOADER_STUB_SECTION __attribute__((used, noinline,
                                                         naked))
            esp32_boot_loader_stub_end(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_region_contains
 *
 * Description:
 *   Return true if [start, start + size) is fully inside [low, high).
 *
 ****************************************************************************/

static bool esp32_region_contains(uint32_t low, uint32_t high,
                                  uint32_t start, uint32_t size)
{
  uint64_t end;

  if (size == 0)
    {
      return true;
    }

  if (start < low)
    {
      return false;
    }

  end = (uint64_t)start + (uint64_t)size;
  if (end > (uint64_t)high)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: esp32_get_partition_info
 *
 * Description:
 *   Return the byte offset and size of the opened partition node.
 *
 ****************************************************************************/

static int esp32_get_partition_info(int fd, FAR uint32_t *offset,
                                    FAR uint32_t *size)
{
  struct partition_info_s partinfo;
  uint64_t partition_offset;
  uint64_t partition_size;

  if (offset == NULL || size == NULL)
    {
      return -EINVAL;
    }

  if (ioctl(fd, BIOC_PARTINFO, (unsigned long)((uintptr_t)&partinfo)) < 0)
    {
      ferr("ERROR: BIOC_PARTINFO failed: %d\n", errno);
      return -errno;
    }

  partition_offset = (uint64_t)partinfo.startsector *
                     (uint64_t)partinfo.sectorsize;
  partition_size = (uint64_t)partinfo.numsectors *
                   (uint64_t)partinfo.sectorsize;
  if (partition_offset > UINT32_MAX || partition_size > UINT32_MAX)
    {
      ferr("ERROR: Partition geometry exceeds 32-bit flash offsets\n");
      return -EOVERFLOW;
    }

  *offset = (uint32_t)partition_offset;
  *size = (uint32_t)partition_size;
  return OK;
}

/****************************************************************************
 * Name: esp32_flash_offset_add
 *
 * Description:
 *   Safely compute a 32-bit flash offset sum.
 *
 ****************************************************************************/

static int esp32_flash_offset_add(uint32_t base, uint32_t offset,
                                  FAR uint32_t *result)
{
  uint64_t sum;

  if (result == NULL)
    {
      return -EINVAL;
    }

  sum = (uint64_t)base + (uint64_t)offset;
  if (sum > UINT32_MAX)
    {
      ferr("ERROR: Flash offset overflow: 0x%08" PRIx32 " + 0x%08" PRIx32
           "\n", base, offset);
      return -EOVERFLOW;
    }

  *result = (uint32_t)sum;
  return OK;
}

/****************************************************************************
 * Name: esp32_include_flash_segment
 *
 * Description:
 *   Add a non-empty segment to the flash mapping span.
 *
 ****************************************************************************/

static int esp32_include_flash_segment(uint32_t offset, uint32_t size,
                                       FAR uint32_t *map_start,
                                       FAR uint32_t *map_end)
{
  uint64_t end;

  if (map_start == NULL || map_end == NULL)
    {
      return -EINVAL;
    }

  if (size == 0)
    {
      return OK;
    }

  end = (uint64_t)offset + (uint64_t)size;
  if (end > UINT32_MAX)
    {
      ferr("ERROR: Flash segment range overflow\n");
      return -EOVERFLOW;
    }

  if (offset < *map_start)
    {
      *map_start = offset;
    }

  if ((uint32_t)end > *map_end)
    {
      *map_end = (uint32_t)end;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_read_load_header
 *
 * Description:
 *   Read the MCUboot load header from the opened partition.
 *
 ****************************************************************************/

static int esp32_read_load_header(int fd, uint32_t image_offset,
                                  FAR struct esp32_load_header_s *header)
{
  ssize_t nread;

  if (fd < 0 || header == NULL)
    {
      return -EINVAL;
    }

  nread = pread(fd, header, sizeof(*header), image_offset);
  if (nread < 0)
    {
      ferr("ERROR: Failed to read image load header: %d\n", errno);
      return -errno;
    }

  if (nread != sizeof(*header))
    {
      ferr("ERROR: Incomplete image load header\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_prepare_handoff_layout
 *
 * Description:
 *   Reserve DRAM region for new stack+args and IRAM region for stub code,
 *   then validate image layout against reserved regions.
 *
 ****************************************************************************/

static int esp32_prepare_handoff_layout(
              FAR const struct esp32_load_header_s *load_header,
              uint32_t stub_size,
              FAR uint32_t *stub_dst,
              FAR uint32_t *args_addr,
              FAR uint32_t *stack_top)
{
  uint32_t reserved_dram;
  uint32_t dram_reserved_start;
  uint32_t iram_reserved_start;
  uint32_t args_size;

  if (load_header == NULL || stub_dst == NULL || args_addr == NULL ||
      stack_top == NULL)
    {
      return -EINVAL;
    }

  if (!esp32_region_contains(ESP32_IRAM_LOW, ESP32_IRAM_HIGH,
                             load_header->entry_addr, 1))
    {
      ferr("ERROR: Entry point is not in IRAM: 0x%08" PRIx32 "\n",
           load_header->entry_addr);
      return -EINVAL;
    }

  args_size = ESP32_ALIGN_UP(sizeof(struct esp32_boot_loader_args_s),
                             ESP32_BOOTLOADER_STUB_ALIGN);
  reserved_dram = args_size + ESP32_BOOTLOADER_STACK_SIZE;

  if (reserved_dram >= (ESP32_DRAM_HIGH - ESP32_DRAM_LOW))
    {
      ferr("ERROR: Reserved DRAM region too large\n");
      return -EINVAL;
    }

  dram_reserved_start = ESP32_ALIGN_DOWN(ESP32_DRAM_HIGH - reserved_dram,
                                         ESP32_BOOTLOADER_STUB_ALIGN);
  iram_reserved_start = ESP32_ALIGN_DOWN(ESP32_IRAM_HIGH - stub_size,
                                         ESP32_BOOTLOADER_STUB_ALIGN);

  if (iram_reserved_start < ESP32_IRAM_LOW)
    {
      ferr("ERROR: Reserved IRAM region too large\n");
      return -EINVAL;
    }

  if (!esp32_region_contains(ESP32_IRAM_LOW, iram_reserved_start,
                             load_header->iram_dest_addr,
                             load_header->iram_size))
    {
      ferr("ERROR: IRAM segment overlaps reserved IRAM stub region\n");
      return -EINVAL;
    }

  if (load_header->iram_size == 0 ||
      !esp32_region_contains(load_header->iram_dest_addr,
                             load_header->iram_dest_addr +
                             load_header->iram_size,
                             load_header->entry_addr, 1))
    {
      ferr("ERROR: Entry point is outside the loaded IRAM segment\n");
      return -EINVAL;
    }

  if (!esp32_region_contains(ESP32_DRAM_LOW, dram_reserved_start,
                             load_header->dram_dest_addr,
                             load_header->dram_size))
    {
      ferr("ERROR: DRAM segment overlaps reserved DRAM stack region\n");
      return -EINVAL;
    }

  if (!esp32_region_contains(ESP32_RTC_IRAM_LOW, ESP32_RTC_IRAM_HIGH,
                             load_header->lp_rtc_iram_dest_addr,
                             load_header->lp_rtc_iram_size))
    {
      ferr("ERROR: Invalid RTC IRAM segment\n");
      return -EINVAL;
    }

  if (!esp32_region_contains(ESP32_RTC_DATA_LOW, ESP32_RTC_DATA_HIGH,
                             load_header->lp_rtc_dram_dest_addr,
                             load_header->lp_rtc_dram_size))
    {
      ferr("ERROR: Invalid RTC DRAM segment\n");
      return -EINVAL;
    }

  *stub_dst = iram_reserved_start;
  *args_addr = dram_reserved_start;
  *stack_top = ESP32_ALIGN_DOWN(ESP32_DRAM_HIGH,
                                ESP32_BOOTLOADER_STUB_ALIGN);

  return OK;
}

/****************************************************************************
 * Name: esp32_copy_segment
 *
 * Description:
 *   Copy one segment into its destination address.
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_copy_segment(FAR const void *src,
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
 *   Final loader stage copied to the reserved IRAM area before handoff.
 *
 ****************************************************************************/

static void ESP32_BOOTLOADER_STUB_SECTION __attribute__((used, noinline,
                                                         naked))
esp32_boot_loader_stub(FAR const struct esp32_boot_loader_args_s *args)
{
  __asm__ __volatile__(
    "l32i a3, a2, 4\n"        /* stack_top */
    "mov sp, a3\n"
    "l32i a4, a2, 8\n"        /* iram_src */
    "l32i a5, a2, 12\n"       /* iram_dest_addr */
    "l32i a6, a2, 16\n"       /* iram_size */
    "1:\n"
    "beqz a6, 2f\n"
    "l8ui a7, a4, 0\n"
    "s8i a7, a5, 0\n"
    "addi a4, a4, 1\n"
    "addi a5, a5, 1\n"
    "addi a6, a6, -1\n"
    "j 1b\n"
    "2:\n"
    "l32i a4, a2, 20\n"       /* dram_src */
    "l32i a5, a2, 24\n"       /* dram_dest_addr */
    "l32i a6, a2, 28\n"       /* dram_size */
    "3:\n"
    "beqz a6, 4f\n"
    "l8ui a7, a4, 0\n"
    "s8i a7, a5, 0\n"
    "addi a4, a4, 1\n"
    "addi a5, a5, 1\n"
    "addi a6, a6, -1\n"
    "j 3b\n"
    "4:\n"
    "l32i a3, a2, 0\n"        /* entry_addr */
    "jx a3\n"
    :
    :
    : "a3", "a4", "a5", "a6", "a7", "memory");
}

/****************************************************************************
 * Name: esp32_boot_loader_stub_end
 *
 * Description:
 *   Marker used to determine the boot stub size.
 *
 ****************************************************************************/

static void ESP32_BOOTLOADER_STUB_SECTION __attribute__((used, noinline,
                                                         naked))
esp32_boot_loader_stub_end(void)
{
  __asm__ __volatile__("ret.n\n");
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
  uint32_t partition_offset = 0;
  uint32_t partition_size = 0;
  uint32_t iram_offset;
  uint32_t dram_offset;
  uint32_t lp_rtc_iram_offset;
  uint32_t lp_rtc_dram_offset;
  uint32_t map_start = UINT32_MAX;
  uint32_t map_end = 0;
  uint32_t map_size;
  uint32_t stub_dst;
  uint32_t stub_size;
  uint32_t args_addr;
  uint32_t stack_top;
  irqstate_t flags;
  FAR const uint8_t *mapping;
  FAR const void *iram_src;
  FAR const void *dram_src;
  FAR const void *lp_rtc_iram_src;
  FAR const void *lp_rtc_dram_src;
  FAR struct esp32_boot_loader_args_s *stub_args;
  FAR struct esp32_load_header_s load_header;

  if (path == NULL)
    {
      return -EINVAL;
    }

  stub_size = (uint32_t)((uintptr_t)esp32_boot_loader_stub_end -
                         (uintptr_t)esp32_boot_loader_stub);
  if (stub_size == 0)
    {
      ferr("ERROR: Invalid boot loader stub size\n");
      return -EINVAL;
    }

  stub_size = ESP32_ALIGN_UP(stub_size, ESP32_BOOTLOADER_STUB_ALIGN);

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", path, errno);
      return -errno;
    }

  ret = esp32_get_partition_info(fd, &partition_offset, &partition_size);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  if (!esp32_region_contains(0, partition_size, hdr_size,
                             sizeof(load_header)))
    {
      ferr("ERROR: Image load header is outside the partition\n");
      close(fd);
      return -EINVAL;
    }

  ret = esp32_read_load_header(fd, hdr_size, &load_header);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  if (load_header.header_magic != ESP32_APP_LOAD_HEADER_MAGIC)
    {
      ferr("ERROR: Invalid load header magic: 0x%08" PRIx32 "\n",
           load_header.header_magic);
      close(fd);
      return -EINVAL;
    }

  ret = esp32_prepare_handoff_layout(&load_header, stub_size,
                                     &stub_dst, &args_addr, &stack_top);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  if (!esp32_region_contains(0, partition_size,
                             load_header.iram_flash_offset,
                             load_header.iram_size) ||
      !esp32_region_contains(0, partition_size,
                             load_header.dram_flash_offset,
                             load_header.dram_size) ||
      !esp32_region_contains(0, partition_size,
                             load_header.lp_rtc_iram_flash_offset,
                             load_header.lp_rtc_iram_size) ||
      !esp32_region_contains(0, partition_size,
                             load_header.lp_rtc_dram_flash_offset,
                             load_header.lp_rtc_dram_size))
    {
      ferr("ERROR: Image RAM segment is outside the partition\n");
      close(fd);
      return -EINVAL;
    }

  ret = esp32_flash_offset_add(partition_offset,
                               load_header.iram_flash_offset,
                               &iram_offset);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_flash_offset_add(partition_offset,
                               load_header.dram_flash_offset,
                               &dram_offset);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_flash_offset_add(partition_offset,
                               load_header.lp_rtc_iram_flash_offset,
                               &lp_rtc_iram_offset);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_flash_offset_add(partition_offset,
                               load_header.lp_rtc_dram_flash_offset,
                               &lp_rtc_dram_offset);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_include_flash_segment(iram_offset, load_header.iram_size,
                                    &map_start, &map_end);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_include_flash_segment(dram_offset, load_header.dram_size,
                                    &map_start, &map_end);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_include_flash_segment(lp_rtc_iram_offset,
                                    load_header.lp_rtc_iram_size,
                                    &map_start, &map_end);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_include_flash_segment(lp_rtc_dram_offset,
                                    load_header.lp_rtc_dram_size,
                                    &map_start, &map_end);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  if (map_end <= map_start)
    {
      ferr("ERROR: Image has no RAM segments\n");
      close(fd);
      return -EINVAL;
    }

  map_size = map_end - map_start;
  close(fd);

  /* Flash mapping can suspend the instruction cache. */

  flags = up_irq_save();
  mapping = bootloader_mmap(map_start, map_size);
  if (mapping == NULL)
    {
      up_irq_restore(flags);
      ferr("ERROR: Failed to mmap image RAM segments\n");
      return -EIO;
    }

  iram_src = load_header.iram_size == 0 ? NULL :
             mapping + iram_offset - map_start;
  dram_src = load_header.dram_size == 0 ? NULL :
             mapping + dram_offset - map_start;
  lp_rtc_iram_src = load_header.lp_rtc_iram_size == 0 ? NULL :
                    mapping + lp_rtc_iram_offset - map_start;
  lp_rtc_dram_src = load_header.lp_rtc_dram_size == 0 ? NULL :
                    mapping + lp_rtc_dram_offset - map_start;

  /* From this point on we cannot safely return to NuttX. */

  if (load_header.lp_rtc_iram_size > 0)
    {
      esp32_copy_segment(lp_rtc_iram_src,
                         load_header.lp_rtc_iram_dest_addr,
                         load_header.lp_rtc_iram_size);
    }

  if (load_header.lp_rtc_dram_size > 0)
    {
      esp32_copy_segment(lp_rtc_dram_src,
                         load_header.lp_rtc_dram_dest_addr,
                         load_header.lp_rtc_dram_size);
    }

  esp32_copy_segment((FAR const void *)(uintptr_t)esp32_boot_loader_stub,
                     stub_dst, stub_size);
  up_invalidate_icache((uintptr_t)stub_dst, (uintptr_t)stub_dst + stub_size);

  stub_args = (FAR struct esp32_boot_loader_args_s *)(uintptr_t)args_addr;
  stub_args->entry_addr     = load_header.entry_addr;
  stub_args->stack_top      = stack_top;
  stub_args->iram_src       = (uint32_t)(uintptr_t)iram_src;
  stub_args->iram_dest_addr = load_header.iram_dest_addr;
  stub_args->iram_size      = load_header.iram_size;
  stub_args->dram_src       = (uint32_t)(uintptr_t)dram_src;
  stub_args->dram_dest_addr = load_header.dram_dest_addr;
  stub_args->dram_size      = load_header.dram_size;

  ((void (*)(FAR const struct esp32_boot_loader_args_s *))
    (uintptr_t)stub_dst)(stub_args);

  PANIC();
  return 0;
}
