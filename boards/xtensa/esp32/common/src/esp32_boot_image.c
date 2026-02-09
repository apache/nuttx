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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/esp32/partition.h>
#include <nuttx/board.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>

#include "esp_app_format.h"
#include "esp_loader.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_APP_IMAGE_MAGIC 0xE9

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_boot_loader_args_s {
  uint32_t entry_addr;
  uint32_t drom_addr;
  uint32_t drom_vaddr;
  uint32_t drom_size;
  uint32_t irom_addr;
  uint32_t irom_vaddr;
  uint32_t irom_size;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void IRAM_ATTR esp32_boot_loader_stub(void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_boot_loader_stub
 *
 * Description:
 *   This function resides in IRAM and is responsible for switching MMU
 *   mappings and jumping to the new application.
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_boot_loader_stub(void *arg) {
  struct esp32_boot_loader_args_s *args =
      (struct esp32_boot_loader_args_s *)arg;
  void (*entry_point)(void) = (void (*)(void))args->entry_addr;

  /* Disable interrupts */

  up_irq_disable();

  /* Disable cache */

#ifdef CONFIG_ARCH_CHIP_ESP32
  cache_read_disable(0);
  cache_flush(0);
#else
  cache_hal_disable(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
#endif

  /* Map new segments */

  map_rom_segments(args->drom_addr, args->drom_vaddr, args->drom_size,
                   args->irom_addr, args->irom_vaddr, args->irom_size);

  /* Jump to entry point */

  entry_point();

  /* Should never reach here */

  while (1) {
  }
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
 *   hdr_size - Size of the image header (unused for ESP32)
 *
 * Returned Value:
 *   Does not return on success; returns error code on failure.
 *
 ****************************************************************************/

int board_boot_image(FAR const char *path, uint32_t hdr_size) {
  int fd;
  int ret;
  uint32_t offset;
  esp_image_header_t image_header;
  esp_image_segment_header_t segment_hdr;
  struct esp32_boot_loader_args_s args = {0};
  uint32_t current_offset;
  int i;

  /* Check for legacy format (not supported) */

#ifdef CONFIG_ESP32_APP_FORMAT_LEGACY
  ferr("ERROR: Legacy format not supported for board_boot_image\n");
  return -ENOTSUP;
#endif

  /* Open the image file */

  fd = open(path, O_RDONLY);
  if (fd < 0) {
    ferr("ERROR: Failed to open %s: %d\n", path, errno);
    return -errno;
  }

  /* Get partition offset */

  ret = ioctl(fd, OTA_IMG_GET_OFFSET, (unsigned long)&offset);
  if (ret < 0) {
    ferr("ERROR: Failed to get partition offset: %d\n", errno);
    close(fd);
    return -errno;
  }

  /* Read image header */

  ret = read(fd, &image_header, sizeof(esp_image_header_t));
  if (ret != sizeof(esp_image_header_t)) {
    ferr("ERROR: Failed to read image header: %d\n", errno);
    close(fd);
    return -errno;
  }

  if (image_header.magic != ESP32_APP_IMAGE_MAGIC) {
    ferr("ERROR: Invalid image magic: 0x%02x\n", image_header.magic);
    close(fd);
    return -EINVAL;
  }

  args.entry_addr = image_header.entry_addr;
  current_offset = sizeof(esp_image_header_t);

  /* Parse segments */

  for (i = 0; i < image_header.segment_count; i++) {
    ret = read(fd, &segment_hdr, sizeof(esp_image_segment_header_t));
    if (ret != sizeof(esp_image_segment_header_t)) {
      ferr("ERROR: Failed to read segment header: %d\n", errno);
      close(fd);
      return -errno;
    }

    /* Check for IROM/DROM segments */

    if (segment_hdr.load_addr >= 0x3f400000 &&
        segment_hdr.load_addr < 0x3f800000) /* DROM */
    {
      args.drom_addr =
          offset + current_offset + sizeof(esp_image_segment_header_t);
      args.drom_vaddr = segment_hdr.load_addr;
      args.drom_size = segment_hdr.data_len;
    } else if (segment_hdr.load_addr >= 0x400d0000 &&
               segment_hdr.load_addr < 0x40400000) /* IROM */
    {
      args.irom_addr =
          offset + current_offset + sizeof(esp_image_segment_header_t);
      args.irom_vaddr = segment_hdr.load_addr;
      args.irom_size = segment_hdr.data_len;
    }

    current_offset += sizeof(esp_image_segment_header_t) + segment_hdr.data_len;

    /* Advance file pointer to next segment */

    lseek(fd, segment_hdr.data_len, SEEK_CUR);
  }

  close(fd);

  finfo("Booting image: entry=0x%08" PRIx32 ", drom=0x%08" PRIx32
        " (0x%08" PRIx32 "), irom=0x%08" PRIx32 " (0x%08" PRIx32 ")\n",
        args.entry_addr, args.drom_vaddr, args.drom_size, args.irom_vaddr,
        args.irom_size);

  /* Invoke IRAM loader stub */

  esp32_boot_loader_stub(&args);

  return 0;
}
