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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/board.h>
#include <nuttx/cache.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>

#include "bootloader_flash_priv.h"
#include "soc/soc.h"
#include "xtensa_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_APP_LOAD_HEADER_MAGIC   0xace637d3
#define ESP32_BOOTLOADER_STACK_SIZE   2048
#define ESP32_BOOTLOADER_STUB_ALIGN   16
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
 * Private Function Prototypes
 ****************************************************************************/

static bool esp32_region_contains(uint32_t low, uint32_t high,
                                  uint32_t start, uint32_t size);
static int esp32_get_partition_offset(int fd, FAR uint32_t *offset);
static int esp32_flash_offset_add(uint32_t base, uint32_t offset,
                                  FAR uint32_t *result);
static int esp32_read_load_header(uint32_t flash_offset,
                                  FAR struct esp32_load_header_s *header);
static int esp32_map_flash_segment(uint32_t flash_offset, uint32_t size,
                                   FAR const char *name,
                                   FAR const void **mapping);
static void esp32_unmap_flash_segment(FAR const void **mapping);
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
 * Name: esp32_get_partition_offset
 *
 * Description:
 *   Return the byte offset in flash of the opened partition node.
 *
 ****************************************************************************/

static int esp32_get_partition_offset(int fd, FAR uint32_t *offset)
{
  struct partition_info_s partinfo;
  uint64_t partition_offset;

  if (offset == NULL)
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
  if (partition_offset > UINT32_MAX)
    {
      ferr("ERROR: Partition offset overflow: %" PRIu64 "\n",
           partition_offset);
      return -EOVERFLOW;
    }

  *offset = (uint32_t)partition_offset;
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
 * Name: esp32_read_load_header
 *
 * Description:
 *   Read the MCUboot load header from flash using ROM mmap routines.
 *
 ****************************************************************************/

static int esp32_read_load_header(uint32_t flash_offset,
                                  FAR struct esp32_load_header_s *header)
{
  FAR const void *mapping;

  if (header == NULL)
    {
      return -EINVAL;
    }

  mapping = bootloader_mmap(flash_offset, sizeof(*header));
  if (mapping == NULL)
    {
      ferr("ERROR: Failed to mmap image load header at 0x%08" PRIx32 "\n",
           flash_offset);
      return -EIO;
    }

  memcpy(header, mapping, sizeof(*header));
  bootloader_munmap(mapping);
  return OK;
}

/****************************************************************************
 * Name: esp32_map_flash_segment
 *
 * Description:
 *   Map one segment from flash for later copy.
 *
 ****************************************************************************/

static int esp32_map_flash_segment(uint32_t flash_offset, uint32_t size,
                                   FAR const char *name,
                                   FAR const void **mapping)
{
  if (mapping == NULL)
    {
      return -EINVAL;
    }

  *mapping = NULL;

  if (size == 0)
    {
      return OK;
    }

  *mapping = bootloader_mmap(flash_offset, size);
  if (*mapping == NULL)
    {
      ferr("ERROR: Failed to mmap %s segment\n", name);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_unmap_flash_segment
 *
 * Description:
 *   Unmap one mapped flash segment.
 *
 ****************************************************************************/

static void esp32_unmap_flash_segment(FAR const void **mapping)
{
  if (mapping != NULL && *mapping != NULL)
    {
      bootloader_munmap(*mapping);
      *mapping = NULL;
    }
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

  if (!esp32_region_contains(SOC_IRAM_LOW, SOC_IRAM_HIGH,
                             load_header->entry_addr, 1))
    {
      ferr("ERROR: Entry point is not in IRAM: 0x%08" PRIx32 "\n",
           load_header->entry_addr);
      return -EINVAL;
    }

  args_size = ESP32_ALIGN_UP(sizeof(struct esp32_boot_loader_args_s),
                             ESP32_BOOTLOADER_STUB_ALIGN);
  reserved_dram = args_size + ESP32_BOOTLOADER_STACK_SIZE;

  if (reserved_dram >= (SOC_DRAM_HIGH - SOC_DRAM_LOW))
    {
      ferr("ERROR: Reserved DRAM region too large\n");
      return -EINVAL;
    }

  dram_reserved_start = ESP32_ALIGN_DOWN(SOC_DRAM_HIGH - reserved_dram,
                                         ESP32_BOOTLOADER_STUB_ALIGN);
  iram_reserved_start = ESP32_ALIGN_DOWN(SOC_IRAM_HIGH - stub_size,
                                         ESP32_BOOTLOADER_STUB_ALIGN);

  if (iram_reserved_start < SOC_IRAM_LOW)
    {
      ferr("ERROR: Reserved IRAM region too large\n");
      return -EINVAL;
    }

  if (!esp32_region_contains(SOC_IRAM_LOW, iram_reserved_start,
                             load_header->iram_dest_addr,
                             load_header->iram_size))
    {
      ferr("ERROR: IRAM segment overlaps reserved IRAM stub region\n");
      return -EINVAL;
    }

  if (!esp32_region_contains(SOC_DRAM_LOW, dram_reserved_start,
                             load_header->dram_dest_addr,
                             load_header->dram_size))
    {
      ferr("ERROR: DRAM segment overlaps reserved DRAM stack region\n");
      return -EINVAL;
    }

  if (load_header->lp_rtc_iram_size > 0 &&
      load_header->lp_rtc_iram_dest_addr == 0)
    {
      ferr("ERROR: Invalid lp_rtc_iram destination\n");
      return -EINVAL;
    }

  if (load_header->lp_rtc_dram_size > 0 &&
      load_header->lp_rtc_dram_dest_addr == 0)
    {
      ferr("ERROR: Invalid lp_rtc_dram destination\n");
      return -EINVAL;
    }

  *stub_dst = iram_reserved_start;
  *args_addr = dram_reserved_start;
  *stack_top = ESP32_ALIGN_DOWN(SOC_DRAM_HIGH, ESP32_BOOTLOADER_STUB_ALIGN);

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
  uint32_t partition_offset;
  uint32_t load_header_offset;
  uint32_t iram_offset;
  uint32_t dram_offset;
  uint32_t lp_rtc_iram_offset;
  uint32_t lp_rtc_dram_offset;
  uint32_t stub_dst;
  uint32_t stub_size;
  uint32_t args_addr;
  uint32_t stack_top;
  FAR const void *iram_src = NULL;
  FAR const void *dram_src = NULL;
  FAR const void *lp_rtc_iram_src = NULL;
  FAR const void *lp_rtc_dram_src = NULL;
  FAR struct esp32_boot_loader_args_s *stub_args;
  FAR struct esp32_load_header_s load_header;

  /* Legacy/simple boot image formats are not supported */

#if defined(CONFIG_ESP32_APP_FORMAT_LEGACY) || \
    defined(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  ferr("ERROR: Unsupported image format for board_boot_image\n");
  return -ENOTSUP;
#endif

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

  ret = esp32_get_partition_offset(fd, &partition_offset);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_flash_offset_add(partition_offset, hdr_size,
                               &load_header_offset);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_read_load_header(load_header_offset, &load_header);
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

  ret = esp32_map_flash_segment(iram_offset, load_header.iram_size,
                                "iram", &iram_src);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_map_flash_segment(dram_offset, load_header.dram_size,
                                "dram", &dram_src);
  if (ret < 0)
    {
      esp32_unmap_flash_segment(&iram_src);
      close(fd);
      return ret;
    }

  ret = esp32_map_flash_segment(lp_rtc_iram_offset,
                                load_header.lp_rtc_iram_size,
                                "lp_rtc_iram", &lp_rtc_iram_src);
  if (ret < 0)
    {
      esp32_unmap_flash_segment(&dram_src);
      esp32_unmap_flash_segment(&iram_src);
      close(fd);
      return ret;
    }

  ret = esp32_map_flash_segment(lp_rtc_dram_offset,
                                load_header.lp_rtc_dram_size,
                                "lp_rtc_dram", &lp_rtc_dram_src);
  if (ret < 0)
    {
      esp32_unmap_flash_segment(&lp_rtc_iram_src);
      esp32_unmap_flash_segment(&dram_src);
      esp32_unmap_flash_segment(&iram_src);
      close(fd);
      return ret;
    }

  /* Copy the final handoff stub into the reserved IRAM region. */

  esp32_copy_segment((FAR const void *)(uintptr_t)esp32_boot_loader_stub,
                     stub_dst, stub_size);
  up_invalidate_icache((uintptr_t)stub_dst, (uintptr_t)stub_dst + stub_size);

  /* Prepare handoff args in reserved DRAM region below the new stack. */

  stub_args = (FAR struct esp32_boot_loader_args_s *)(uintptr_t)args_addr;
  stub_args->entry_addr     = load_header.entry_addr;
  stub_args->stack_top      = stack_top;
  stub_args->iram_src       = (uint32_t)(uintptr_t)iram_src;
  stub_args->iram_dest_addr = load_header.iram_dest_addr;
  stub_args->iram_size      = load_header.iram_size;
  stub_args->dram_src       = (uint32_t)(uintptr_t)dram_src;
  stub_args->dram_dest_addr = load_header.dram_dest_addr;
  stub_args->dram_size      = load_header.dram_size;

  close(fd);

  /* From this point on we cannot safely return to NuttX. */

  up_irq_disable();

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

  ((void (*)(FAR const struct esp32_boot_loader_args_s *))
    (uintptr_t)stub_dst)(stub_args);

  PANIC();
  return 0;
}
