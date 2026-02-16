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
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/board.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include "bootloader_flash_priv.h"
#include "xtensa_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_APP_LOAD_HEADER_MAGIC 0xace637d3
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

struct esp32_boot_loader_args_s
{
  uint32_t entry_addr;
  uintptr_t stack_top;
  uint32_t dram_dest_addr;
  uint32_t dram_size;
  FAR const void *dram_src;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool esp32_ranges_overlap(uintptr_t start1, size_t size1,
                                 uintptr_t start2, size_t size2);
static int esp32_get_partition_offset(int fd, FAR uint32_t *offset);
static int esp32_flash_offset_add(uint32_t base, uint32_t offset,
                                  FAR uint32_t *result);
static int esp32_read_load_header(uint32_t flash_offset,
                                  FAR struct esp32_load_header_s *header);
static int esp32_copy_flash_segment(uint32_t flash_offset,
                                    uint32_t dest_addr, uint32_t size,
                                    FAR const char *name);
static int esp32_prepare_dram_handoff(uint32_t flash_offset,
                                      FAR const struct esp32_load_header_s
                                      *load_header);
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
 * Name: esp32_copy_flash_segment
 *
 * Description:
 *   Load one segment from flash to RAM via ROM mmap routines.
 *
 ****************************************************************************/

static int esp32_copy_flash_segment(uint32_t flash_offset,
                                    uint32_t dest_addr, uint32_t size,
                                    FAR const char *name)
{
  FAR const void *mapping;

  if (size == 0)
    {
      return OK;
    }

  if (dest_addr == 0)
    {
      ferr("ERROR: Invalid %s destination address\n", name);
      return -EINVAL;
    }

  mapping = bootloader_mmap(flash_offset, size);
  if (mapping == NULL)
    {
      ferr("ERROR: Failed to mmap %s segment\n", name);
      return -EIO;
    }

  finfo("Loading %-9s: dst=0x%08" PRIx32 " off=0x%08" PRIx32
        " size=0x%08" PRIx32 "\n",
        name, dest_addr, flash_offset, size);

  esp32_copy_segment(mapping, dest_addr, size);
  bootloader_munmap(mapping);

  return OK;
}

/****************************************************************************
 * Name: esp32_prepare_dram_handoff
 *
 * Description:
 *   Prepare final DRAM copy handoff data. DRAM is copied in the RTC loader
 *   just before jumping to the new entry point.
 *
 ****************************************************************************/

static int esp32_prepare_dram_handoff(uint32_t flash_offset,
                                      FAR const struct esp32_load_header_s
                                      *load_header)
{
  uintptr_t stack_buf;

  DEBUGASSERT(load_header != NULL);

  memset(&g_boot_loader_args, 0, sizeof(g_boot_loader_args));

  g_boot_loader_args.entry_addr     = load_header->entry_addr;
  g_boot_loader_args.dram_dest_addr = load_header->dram_dest_addr;
  g_boot_loader_args.dram_size      = load_header->dram_size;

  if (g_boot_loader_args.dram_size == 0)
    {
      return -EINVAL;
    }

  if (g_boot_loader_args.dram_dest_addr == 0)
    {
      return -EINVAL;
    }

  g_boot_loader_args.dram_src =
    bootloader_mmap(flash_offset, g_boot_loader_args.dram_size);
  if (g_boot_loader_args.dram_src == NULL)
    {
      ferr("ERROR: Failed to mmap dram segment\n");
      return -EIO;
    }

  stack_buf = (uintptr_t)kmm_malloc(ESP32_BOOTLOADER_STACK_SIZE);
  if (stack_buf == 0)
    {
      ferr("ERROR: Failed to allocate boot loader stack\n");
      bootloader_munmap(g_boot_loader_args.dram_src);
      g_boot_loader_args.dram_src = NULL;
      return -ENOMEM;
    }

  if (esp32_ranges_overlap(stack_buf, ESP32_BOOTLOADER_STACK_SIZE,
                           g_boot_loader_args.dram_dest_addr,
                           g_boot_loader_args.dram_size))
    {
      ferr("ERROR: Boot loader stack overlaps DRAM destination\n");
      kmm_free((FAR void *)stack_buf);
      bootloader_munmap(g_boot_loader_args.dram_src);
      g_boot_loader_args.dram_src = NULL;
      return -EFAULT;
    }

  g_boot_loader_args.stack_top =
    (stack_buf + ESP32_BOOTLOADER_STACK_SIZE) & ~(uintptr_t)0xf;

  return OK;
}

/****************************************************************************
 * Name: esp32_copy_segment
 *
 * Description:
 *   Copy one segment into its destination address.
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
 *   the DRAM segment just before jumping to the new image entry point.
 *
 ****************************************************************************/

static void RTC_IRAM_ATTR esp32_boot_loader_stub(void)
{
  FAR const struct esp32_boot_loader_args_s *args = &g_boot_loader_args;
  register uintptr_t stack_top = args->stack_top;
  register uint32_t entry_addr = args->entry_addr;
  register FAR const void *dram_src = args->dram_src;
  register uint32_t dram_dst = args->dram_dest_addr;
  register uint32_t dram_size = args->dram_size;

  /* Disable interrupts */

  up_irq_disable();

  /* Use a dedicated handoff stack that does not overlap the DRAM segment
   * destination copied in the final step.
   */

  __asm__ __volatile__("mov sp, %0" : : "r"(stack_top));

  /* DRAM must be copied in the final stage to avoid self-overwrite while
   * still executing from the currently running image.
   */

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
  uint32_t partition_offset;
  uint32_t load_header_offset;
  uint32_t iram_offset;
  uint32_t dram_offset;
  uint32_t lp_rtc_iram_offset;
  uint32_t lp_rtc_dram_offset;
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

  ret = esp32_copy_flash_segment(iram_offset, load_header.iram_dest_addr,
                                 load_header.iram_size, "iram");
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_copy_flash_segment(lp_rtc_iram_offset,
                                 load_header.lp_rtc_iram_dest_addr,
                                 load_header.lp_rtc_iram_size,
                                 "lp_rtc_iram");
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_copy_flash_segment(lp_rtc_dram_offset,
                                 load_header.lp_rtc_dram_dest_addr,
                                 load_header.lp_rtc_dram_size,
                                 "lp_rtc_dram");
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  ret = esp32_prepare_dram_handoff(dram_offset, &load_header);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  close(fd);

  finfo("Booting image: entry=0x%08" PRIx32 "\n",
        g_boot_loader_args.entry_addr);

  /* Invoke final RTC loader stub. */

  esp32_boot_loader_stub();

  return 0;
}
