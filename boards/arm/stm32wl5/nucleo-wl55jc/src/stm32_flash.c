/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_flash.c
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
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/smart.h>
#include <nuttx/fs/fs.h>

#include <debug.h>
#include <stdio.h>

#include "nucleo-wl55jc.h"
#include "hardware/stm32wl5_flash.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Define default values to silent compiler warning about undefined macro */

#ifndef CONFIG_ARCH_BOARD_FLASH_CPU1_PROG_SIZE
#define CONFIG_ARCH_BOARD_FLASH_CPU1_PROG_SIZE 0
#endif

#ifndef CONFIG_ARCH_BOARD_FLASH_CPU2_PROG_SIZE
#define CONFIG_ARCH_BOARD_FLASH_CPU2_PROG_SIZE 0
#endif

#ifndef CONFIG_ARCH_BOARD_FLASH_PART1_SIZE
#define CONFIG_ARCH_BOARD_FLASH_PART1_SIZE 0
#endif

#ifndef CONFIG_ARCH_BOARD_FLASH_PART2_SIZE
#define CONFIG_ARCH_BOARD_FLASH_PART2_SIZE 0
#endif

#ifndef CONFIG_ARCH_BOARD_FLASH_PART3_SIZE
#define CONFIG_ARCH_BOARD_FLASH_PART3_SIZE 0
#endif

#ifndef CONFIG_ARCH_BOARD_FLASH_PART4_SIZE
#define CONFIG_ARCH_BOARD_FLASH_PART4_SIZE 0
#endif

#if (CONFIG_ARCH_BOARD_FLASH_CPU1_PROG_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_CPU2_PROG_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART1_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART2_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART3_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART4_SIZE) > 128
#   error "Sum of all flash pertitions cannot be bigger than 128"
#endif

#if (CONFIG_ARCH_BOARD_FLASH_CPU1_PROG_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_CPU2_PROG_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART1_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART2_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART3_SIZE + \
     CONFIG_ARCH_BOARD_FLASH_PART4_SIZE) < 128
#   warning "There is unused space on flash"
#endif

#define FLASH_PAGE_SIZE    STM32WL5_FLASH_PAGESIZE

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

struct part_table
{
    int         size; /* partition size in pages */
    const char *name; /* name of the partition */
    const char *mnt;  /* mount point for the partition */
    const char *fs;   /* fs type (smart, raw, nxffs) */
};

/* partition table, first entry *must always* be program flash memory */

static const struct part_table part_table[] =
{
  {
    .size   = CONFIG_ARCH_BOARD_FLASH_CPU1_PROG_SIZE,
    .name   = "cpu1-progmem",
    .mnt    = NULL,
    .fs     = "rawfs"
  },

#if CONFIG_ARCH_BOARD_FLASH_CPU2_PROG_SIZE > 0
    .size   = CONFIG_ARCH_BOARD_FLASH_CPU2_PROG_SIZE,
    .name   = "cpu2-progmem",
    .mnt    = NULL,
    .fs     = "rawfs"
#endif

  {
    .size   = CONFIG_ARCH_BOARD_FLASH_PART1_SIZE,
    .name   = CONFIG_ARCH_BOARD_FLASH_PART1_NAME,
    .mnt    = CONFIG_ARCH_BOARD_FLASH_PART1_MNT,
    .fs     = CONFIG_ARCH_BOARD_FLASH_PART1_FS
  },

#if CONFIG_ARCH_BOARD_FLASH_PART2_SIZE > 0
  {
    .size   = CONFIG_ARCH_BOARD_FLASH_PART2_SIZE,
    .name   = CONFIG_ARCH_BOARD_FLASH_PART2_NAME,
    .mnt    = CONFIG_ARCH_BOARD_FLASH_PART2_MNT,
    .fs     = CONFIG_ARCH_BOARD_FLASH_PART2_FS
  },
#endif /* CONFIG_ARCH_BOARD_FLASH_PART2_SIZE */

#if CONFIG_ARCH_BOARD_FLASH_PART3_SIZE > 0
  {
    .size   = CONFIG_ARCH_BOARD_FLASH_PART3_SIZE,
    .name   = CONFIG_ARCH_BOARD_FLASH_PART3_NAME,
    .mnt    = CONFIG_ARCH_BOARD_FLASH_PART3_MNT,
    .fs     = CONFIG_ARCH_BOARD_FLASH_PART3_FS
  },
#endif /* CONFIG_ARCH_BOARD_FLASH_PART3_SIZE */

#if CONFIG_ARCH_BOARD_FLASH_PART4_SIZE > 0
  {
    .size   = CONFIG_ARCH_BOARD_FLASH_PART4_SIZE,
    .name   = CONFIG_ARCH_BOARD_FLASH_PART4_NAME,
    .mnt    = CONFIG_ARCH_BOARD_FLASH_PART4_MNT,
    .fs     = CONFIG_ARCH_BOARD_FLASH_PART4_FS
  },
#endif /* CONFIG_ARCH_BOARD_FLASH_PART4_SIZE */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32wl5_flash_init(void)
{
  FAR struct mtd_dev_s *mtd;
  FAR struct mtd_dev_s *mtd_part;
  int offset;
  int mtdconfig_minor;
  int mtdblk_minor;
  int smart_minor;
  int nxf_minor;
  int ret;
  int i;

  /* Silent compiler warning in case these filesystems are not enabled */

  UNUSED(nxf_minor);
  UNUSED(smart_minor);
  UNUSED(mtdconfig_minor);

  /* create an instance of the stm32 flash program memory device driver */

  if ((mtd = progmem_initialize()) == NULL)
    {
      ferr("ERROR: progmem_initialize failed %d\n", errno);
      return -1;
    }

  /* create partitions, this does not actually write anything
   * to flash memory, we only create partition table in ram
   */

  mtdconfig_minor = 0;
  mtdblk_minor = 0;
  smart_minor = 0;
  nxf_minor = 0;
  offset = 0;

  for (i = 0; i != sizeof(part_table) / sizeof(struct part_table); i++)
    {
      int         size;
      const char *name;
      const char *mnt;
      const char *fs;

      /* Silent compiler warning in case where only non mountable
       * partitions are defined.
       */

      UNUSED(mnt);

      size = part_table[i].size;
      name = part_table[i].name;
      mnt  = part_table[i].mnt;
      fs   = part_table[i].fs;

      finfo("[%s] creating partition, size: %d, fs: %s, offset: %d\n",
            name, size, fs, offset);

      /* create mtd parition */

      mtd_part = mtd_partition(mtd, offset, size);

      /* calculate offset for next partition */

      offset += size;

      if (mtd_part == NULL)
        {
          ferr("[%s]ERROR: mtd_partition() failed %d\n", name, errno);
          continue;
        }

      if (mtd_setpartitionname(mtd_part, name))
        {
          ferr("[%s]ERROR: mtd_setpartitionname() failed %d\n", name, errno);
          continue;
        }

      /* initialize filesystems */

      if (strcmp(fs, "rawfs") == 0)
        {
          /* for raw fs just create mtdblock using ftl */

          if ((ret = ftl_initialize(mtdblk_minor, mtd_part)))
            {
              ferr("[%s]ERROR: ftl_initialize failed %d\n", name, ret);
              continue;
            }

          mtdblk_minor++;
        }

#if defined(CONFIG_FS_NXFFS)
      else if (strcmp(fs, "nxffs") == 0)
        {
          if (nxf_minor)
            {
              ferr("[%s]ERROR: only 1 nxffs is allowed, ignoring\n", name);
              continue;
            }

          /* attach mtd to nxffs */

          if ((ret = nxffs_initialize(mtd_part)))
            {
              ferr("[%s]ERROR: nxffs_initialize failed %d\n", name, ret);
              continue;
            }

          /* mount nxffs */

          if ((ret = nx_mount(NULL, mnt, "nxffs", 0, NULL)))
            {
              ferr("[%s]ERROR: nx_mount failed: %d", name, ret);
              continue;
            }

          nxf_minor++;
        }
#endif

#if defined(CONFIG_FS_SMARTFS)
      else if (strcmp(fs, "smartfs") == 0)
        {
          /* attach mtd to smartfs */

          char src[32];
          snprintf(src, sizeof(src), "/dev/smart%d", smart_minor);

          if ((ret = smart_initialize(smart_minor, mtd_part, NULL)))
            {
              ferr("[%s]ERROR: smart_initialize() failed %d\n", name, ret);
              continue;
            }

          /* mount smartfs */

          if ((ret = nx_mount(src, mnt, "smartfs", 0, NULL)))
            {
              ferr("[%s]ERROR: nx_mount failed: %d\n", name, ret);
              if (ret == ENODEV)
                {
                  syslog(LOG_INFO, "[%s] mtd, smartfs seems unformated. "
                         "Did you run 'mksmartfs %s'?\n", name, src);
                }

              continue;
            }

          smart_minor++;
        }
#endif

#if defined(CONFIG_MTD_CONFIG)
      else if (strcmp(fs, "mtdconfig") == 0)
        {
          if (mtdconfig_minor)
            {
              ferr("[%s]ERROR: only 1 mtdconfig is allowed, ignoring\n",
                   name);
              continue;
            }

          /* attach mtd to mtdconfig driver */

          if (mtdconfig_register(mtd_part) != 0)
            {
              ferr("[%s]ERROR: mtdconfig_register() failed %d\n",
                   name, errno);
              continue;
            }

          mtdconfig_minor++;
        }
#endif
    }

  return 0;
}
