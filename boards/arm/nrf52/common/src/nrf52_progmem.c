/****************************************************************************
 * boards/arm/nrf52/common/src/nrf52_progmem.c
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

#include <sys/mount.h>
#include <sys/param.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/progmem.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>

#include "nrf52_progmem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PARTITION_LABEL_LEN         16

/* Configuration ************************************************************/

/* Make sure that support for MTD partitions is enabled */

#ifndef CONFIG_MTD_PARTITION
#  error "CONFIG_MTD_PARTITION is required"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_NRF52_PROGMEM_OTA_PARTITION
struct ota_partition_s
{
  uint32_t    offset;          /* Partition offset from the beginning of MTD */
  uint32_t    size;            /* Partition size in bytes */
  const char *devpath;         /* Partition device path */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NRF52_PROGMEM_OTA_PARTITION
static struct mtd_dev_s *progmem_alloc_mtdpart(uint32_t mtd_offset,
                                               uint32_t mtd_size);
static int init_ota_partitions(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mtd_dev_s *g_progmem_mtd;

#ifdef CONFIG_NRF52_PROGMEM_OTA_PARTITION
static const struct ota_partition_s g_ota_partition_table[] =
{
  {
    .offset  = CONFIG_NRF52_OTA_PRIMARY_SLOT_OFFSET,
    .size    = CONFIG_NRF52_OTA_SLOT_SIZE,
    .devpath = CONFIG_NRF52_OTA_PRIMARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_NRF52_OTA_SECONDARY_SLOT_OFFSET,
    .size    = CONFIG_NRF52_OTA_SLOT_SIZE,
    .devpath = CONFIG_NRF52_OTA_SECONDARY_SLOT_DEVPATH
  },
#ifdef CONFIG_NRF52_MCUBOOT_HAVE_SCRACH
  {
    .offset  = CONFIG_NRF52_OTA_SCRATCH_OFFSET,
    .size    = CONFIG_NRF52_OTA_SCRATCH_SIZE,
    .devpath = CONFIG_NRF52_OTA_SCRATCH_DEVPATH
  }
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NRF52_PROGMEM_OTA_PARTITION
/****************************************************************************
 * Name: progmem_alloc_mtdpart
 *
 * Description:
 *   Allocate an MTD partition from FLASH.
 *
 * Input Parameters:
 *   mtd_offset - MTD Partition offset from the base address in FLASH.
 *   mtd_size   - Size for the MTD partition.
 *
 * Returned Value:
 *   MTD partition data pointer on success, NULL on failure.
 *
 ****************************************************************************/

static struct mtd_dev_s *progmem_alloc_mtdpart(uint32_t mtd_offset,
                                               uint32_t mtd_size)
{
  uint32_t blocks;
  ssize_t startblock;

  ASSERT((mtd_offset % up_progmem_pagesize(0)) == 0);
  ASSERT((mtd_size % up_progmem_pagesize(0)) == 0);

  finfo("\tMTD offset = 0x%"PRIx32"\n", mtd_offset);
  finfo("\tMTD size = 0x%"PRIx32"\n", mtd_size);

  startblock = up_progmem_getpage(mtd_offset + up_progmem_getaddress(0));
  if (startblock < 0)
    {
      return NULL;
    }

  blocks = mtd_size / up_progmem_pagesize(0);

  return mtd_partition(g_progmem_mtd, startblock, blocks);
}

/****************************************************************************
 * Name: init_ota_partitions
 *
 * Description:
 *   Initialize partitions that are dedicated to firmware OTA update.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int init_ota_partitions(void)
{
  char              path[PARTITION_LABEL_LEN + 1];
  struct mtd_dev_s *mtd = NULL;
  int               i   = 0;
  int               ret = 0;

  for (i = 0; i < nitems(g_ota_partition_table); ++i)
    {
      const struct ota_partition_s *part = &g_ota_partition_table[i];
      mtd = progmem_alloc_mtdpart(part->offset, part->size);

      strlcpy(path, (char *)part->devpath, PARTITION_LABEL_LEN);

      finfo("INFO: [label]:   %s\n", path);
      finfo("INFO: [offset]:  0x%08" PRIx32 "\n", part->offset);
      finfo("INFO: [size]:    0x%08" PRIx32 "\n", part->size);

      if (!mtd)
        {
          ferr("ERROR: Failed to create MTD partition\n");
          ret = -1;
        }

      ret = register_mtddriver(path, mtd, 0777, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register MTD @ %s\n", path);
          ret = -1;
        }
    }

  return ret;
}
#endif /* CONFIG_NRF52_PROGMEM_OTA_PARTITION */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_progmem_init
 *
 *   Initialize Progmem partition. Read partition information, and use
 *   these data for creating MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int nrf52_progmem_init(void)
{
  int ret = 0;

  g_progmem_mtd = progmem_initialize();
  if (g_progmem_mtd == NULL)
    {
      ferr("ERROR: Failed to get progmem flash MTD\n");
      ret = -EIO;
    }

#ifdef CONFIG_NRF52_PROGMEM_OTA_PARTITION
  ret = init_ota_partitions();
  if (ret < 0)
    {
      ferr("ERROR: Failed to create OTA partition from MTD\n");
      ret = -EIO;
    }
#endif

  return ret;
}
