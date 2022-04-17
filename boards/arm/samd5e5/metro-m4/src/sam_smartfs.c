/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_smartfs.c
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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "chip.h"
#include <arch/board/board.h>

#include "metro-m4.h"

#ifdef CONFIG_MTD
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/mtd/configdata.h>
#  include <nuttx/drivers/drivers.h>
#  include "sam_progmem.h"
#endif

#ifdef CONFIG_FS_SMARTFS
#  include <nuttx/fs/smart.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_SMARTFS
  int ret;
  struct mtd_dev_s *mtd;
  struct mtd_geometry_s geo;
#endif

#if defined(CONFIG_MTD_PARTITION_NAMES)
  const char *partname = "mnta";
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_FS_SMARTFS
int sam_smartfs_initialize(void)
{
  /* Initialize the SAMD5E5 FLASH programming memory library */

  sam_progmem_initialize();

  /* Create an instance of the SAMD5E5 FLASH program memory device driver */

  mtd = progmem_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize failed\n");
    }

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  #ifdef CONFIG_MTD_PARTITION
    {
      int partno;
      int partsize;
      int partoffset;
      int partszbytes;
      int erasesize;
      const char *partstring = "256";
      const char *ptr;
      struct mtd_dev_s *mtd_part;
      char  partref[16];

      /* Now create a partition on the FLASH device */

      partno = 0;
      ptr = partstring;
      partoffset = 0;

          /* Get the Flash erase size */

          erasesize = geo.erasesize;

          while (*ptr != '\0')
            {
              /* Get the partition size */

              partsize = atoi(ptr);
              partszbytes = (partsize << 10); /* partsize is defined in KB */
              printf("partsize %d partszbytes %d\n", partsize, partszbytes);

              /* Check if partition size is bigger then erase block */

              if (partszbytes < erasesize)
                {
                  syslog(LOG_ERR,
                    "ERROR: Partition size is lesser than erasesize!\n");
                  return -1;
                }

              /* Check if partition size is multiple of erase block */

              if ((partszbytes % erasesize) != 0)
                {
                  syslog(LOG_ERR,
                    "ERROR: Partition size is not multiple of erasesize!\n");
                  return -1;
                }

              mtd_part = mtd_partition(mtd, partoffset,
                                       partszbytes / erasesize);
              partoffset += partszbytes / erasesize;

              /* Test if this is the config partition */

            #if defined  CONFIG_MTD_CONFIG
              if (partno == 0)
                {
                  /* Register the partition as the config device */

                  mtdconfig_register(mtd_part);
                }
                else
            #endif
                {
                  /* Now initialize a SMART Flash block device
                   * and bind it to the MTD device.
                   */

            #if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
                      snprintf(partref, sizeof(partref), "p%d", partno);
                      smart_initialize(0, mtd_part, partref);
            #endif
                }

                  /* Set the partition name */

            #ifdef CONFIG_MTD_PARTITION_NAMES
                  if (!mtd_part)
                    {
                      syslog(LOG_ERR,
                            "Error: failed to create partition %s\n",
                            partname);
                      return -1;
                    }

                  mtd_setpartitionname(mtd_part, partname);

                  /* Now skip to next name.
                   * We don't need to split the string here
                   * because the MTD partition logic will only
                   * display names up to the comma,
                   * thus allowing us to use a single static name
                   * in the code.
                   */

                  while (*partname != ',' && *partname != '\0')
                    {
                      /* Skip to next ',' */

                      partname++;
                    }

                  if (*partname == ',')
                    {
                      partname++;
                    }
            #endif

          /* Update the pointer to point to the next size in the list */

          while ((*ptr >= '0') && (*ptr <= '9'))
            {
              ptr++;
            }

          if (*ptr == ',')
            {
              ptr++;
            }

          /* Increment the part number */

          partno++;
        }
    }
  #else /* CONFIG_MTD_PARTITION */

  /* Configure the device with no partition support */

  smart_initialize(0, mtd, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: SmartFS initialization failed: %d\n", ret);
      return ret;
    }

    #endif

  return OK;
}

#endif /* CONFIG_FS_SMARTFS */