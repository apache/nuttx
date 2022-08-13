/****************************************************************************
 * boards/arm/samv7/common/src/sam_progmem.c
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

#include <nuttx/mtd/mtd.h>
#ifdef CONFIG_BCH
#include <nuttx/drivers/drivers.h>
#endif

#include "sam_progmem.h"
#include "board_progmem.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_progmem_register_driver
 *
 * Description:
 *   Initialize the FLASH and register MTD devices.
 *
 * Input Parameters:
 *   minor   - The minor number for progmem MTD block driver.
 *   mtd     - MTD partition data pointer
 *   devpath - Character device path
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_progmem_register_driver(int minor, struct mtd_dev_s *mtd,
                                       const char *devpath)
{
#ifdef CONFIG_BCH
  char blockdev[18];
  char chardev[12];
#endif
  int ret = OK;

  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_BCH
  /* Use the minor number to create device paths */

  snprintf(blockdev, sizeof(blockdev), "/dev/mtdblock%d", minor);
  if (devpath == NULL)
    {
      snprintf(chardev, sizeof(chardev), "/dev/mtd%d", minor);
      devpath = chardev;
    }

  /* Now create a character device on the block device */

  ret = bchdev_register(blockdev, devpath, false);
  if (ret < 0)
    {
      ferr("ERROR: bchdev_register %s failed: %d\n", devpath, ret);
      return ret;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: sam_progmem_alloc_mtdpart
 *
 * Description:
 *   Allocate an MTD partition from FLASH.
 *
 * Input Parameters:
 *   mtd        - The MTD device to be partitioned
 *   mtd_offset - MTD Partition offset from the base address in FLASH.
 *   mtd_size   - Size for the MTD partition.
 *
 * Returned Value:
 *   MTD partition data pointer on success, NULL on failure.
 *
 ****************************************************************************/

static struct mtd_dev_s *sam_progmem_alloc_mtdpart(struct mtd_dev_s *mtd,
                                                   size_t mtd_offset,
                                                   size_t mtd_size)
{
  ssize_t startblock;
  size_t  blocks;

  ASSERT((mtd_offset + mtd_size) <= up_progmem_neraseblocks() *
          up_progmem_pagesize(0));
  ASSERT((mtd_offset % up_progmem_pagesize(0)) == 0);
  ASSERT((mtd_size % up_progmem_pagesize(0)) == 0);

  finfo("\tMTD offset = 0x%zx\n", mtd_offset);
  finfo("\tMTD size = %zu\n", mtd_size);

  startblock = up_progmem_getpage(mtd_offset);
  if (startblock < 0)
    {
      return NULL;
    }

  blocks = mtd_size / up_progmem_pagesize(0);

  return mtd_partition(mtd, startblock, blocks);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_progmem_init
 *
 * Description:
 *   Initialize the FLASH and register MTD devices.
 *
 * Input Parameters:
 *   minor - The starting minor number for progmem MTD partitions.
 *   table - Progmem MTD partition table
 *   count - Number of element in progmem MTD partition table
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_progmem_init(int minor, struct mtd_partition_s *table,
                       size_t count)
{
  struct mtd_dev_s *progmem_mtd;
  size_t i;
  int ret = OK;

  /* Initialize the SAMV7 FLASH programming memory library */

  sam_progmem_initialize();

  /* Create an instance of the SAMV7 FLASH program memory device driver */

  progmem_mtd = progmem_initialize();
  if (progmem_mtd == NULL)
    {
      return -EFAULT;
    }

  if (table == NULL || count == 0)
    {
      ret = sam_progmem_register_driver(minor, progmem_mtd, NULL);
    }
  else
    {
      for (i = 0; i < count; ++i)
        {
          struct mtd_partition_s *part = &table[i];

          part->mtd = sam_progmem_alloc_mtdpart(progmem_mtd, part->offset,
                                                part->size);
          if (part->mtd != NULL)
            {
              if (part->devpath != NULL)
                {
                  ret = sam_progmem_register_driver(minor + i, part->mtd,
                                                    part->devpath);
                  if (ret < 0)
                    {
                      break;
                    }
                }
            }
          else
            {
              ferr("Failed to allocate MTD partition %d", i);
              ret = -ENOMEM;
              break;
            }
        }
    }

  return ret;
}
