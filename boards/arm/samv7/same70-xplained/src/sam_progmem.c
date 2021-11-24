/****************************************************************************
 * boards/arm/samv7/same70-xplained/src/sam_progmem.c
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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#ifdef CONFIG_BCH
#include <nuttx/drivers/drivers.h>
#endif

#include "sam_progmem.h"
#include "same70-xplained.h"

#ifdef HAVE_PROGMEM_CHARDEV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x)                (sizeof((x)) / sizeof((x)[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(SAME70XPLAINED_PROGMEM_OTA_PARTITION)

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

#if defined(SAME70XPLAINED_PROGMEM_OTA_PARTITION)
static struct mtd_dev_s *sam_progmem_alloc_mtdpart(uint32_t mtd_offset,
                                                   uint32_t mtd_size);
static int init_ota_partitions(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct mtd_dev_s *g_samv7_progmem_mtd;

#if defined(SAME70XPLAINED_PROGMEM_OTA_PARTITION)
static const struct ota_partition_s g_ota_partition_table[] =
{
  {
    .offset  = CONFIG_SAME70XPLAINED_OTA_PRIMARY_SLOT_OFFSET,
    .size    = CONFIG_SAME70XPLAINED_OTA_SLOT_SIZE,
    .devpath = CONFIG_SAME70XPLAINED_OTA_PRIMARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_SAME70XPLAINED_OTA_SECONDARY_SLOT_OFFSET,
    .size    = CONFIG_SAME70XPLAINED_OTA_SLOT_SIZE,
    .devpath = CONFIG_SAME70XPLAINED_OTA_SECONDARY_SLOT_DEVPATH
  },
  {
    .offset  = CONFIG_SAME70XPLAINED_OTA_SCRATCH_OFFSET,
    .size    = CONFIG_SAME70XPLAINED_OTA_SCRATCH_SIZE,
    .devpath = CONFIG_SAME70XPLAINED_OTA_SCRATCH_DEVPATH
  }
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(SAME70XPLAINED_PROGMEM_OTA_PARTITION)

/****************************************************************************
 * Name: sam_progmem_alloc_mtdpart
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

static struct mtd_dev_s *sam_progmem_alloc_mtdpart(uint32_t mtd_offset,
                                                   uint32_t mtd_size)
{
  uint32_t blocks;
  uint32_t startblock;

  ASSERT((mtd_offset + mtd_size) <= up_progmem_neraseblocks() *
          up_progmem_pagesize(0));
  ASSERT((mtd_offset % up_progmem_pagesize(0)) == 0);
  ASSERT((mtd_size % up_progmem_pagesize(0)) == 0);

  finfo("\tMTD offset = 0x%"PRIx32"\n", mtd_offset);
  finfo("\tMTD size = 0x%"PRIx32"\n", mtd_size);

  startblock = up_progmem_getpage(mtd_offset);
  if (startblock < 0)
    {
      return NULL;
    }

  blocks = mtd_size / up_progmem_pagesize(0);

  return mtd_partition(g_samv7_progmem_mtd, startblock, blocks);
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
  FAR struct mtd_dev_s *mtd;
#ifdef CONFIG_BCH
  char blockdev[18];
#endif
  int ret = OK;

  for (int i = 0; i < ARRAYSIZE(g_ota_partition_table); ++i)
    {
      const struct ota_partition_s *part = &g_ota_partition_table[i];
      mtd = sam_progmem_alloc_mtdpart(part->offset, part->size);

      ret = ftl_initialize(i, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#ifdef CONFIG_BCH
      snprintf(blockdev, 18, "/dev/mtdblock%d", i);

      ret = bchdev_register(blockdev, part->devpath, false);
      if (ret < 0)
        {
          ferr("ERROR: bchdev_register %s failed: %d\n", part->devpath, ret);
          return ret;
        }
#endif
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_progmem_init
 *
 * Description:
 *   Initialize the FLASH and register the MTD device.
 ****************************************************************************/

int sam_progmem_init(void)
{
  int ret = OK;

  /* Initialize the SAME70 FLASH programming memory library */

  sam_progmem_initialize();

  /* Create an instance of the SAME70 FLASH program memory device driver */

  g_samv7_progmem_mtd = progmem_initialize();
  if (g_samv7_progmem_mtd == NULL)
    {
      return -EFAULT;
    }

#if defined(SAME70XPLAINED_PROGMEM_OTA_PARTITION)
  ret = init_ota_partitions();
  if (ret < 0)
    {
      return ret;
    }
#else
  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(PROGMEM_MTD_MINOR, g_samv7_progmem_mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n",
             ret);
      return ret;
    }

#ifdef CONFIG_BCH
  char blockdev[18];
  char chardev[12];

  /* Use the minor number to create device paths */

  snprintf(blockdev, 18, "/dev/mtdblock%d", PROGMEM_MTD_MINOR);
  snprintf(chardev, 12, "/dev/mtd%d", PROGMEM_MTD_MINOR);

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n",
             chardev, ret);
      return ret;
    }
#endif /* CONFIG_BCH */
#endif

  return ret;
}
#endif /* HAVE_PROGMEM_CHARDEV */
