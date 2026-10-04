/****************************************************************************
 * boards/mips/pic32mz/ev49n51a/src/pic32mz_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_MTD_SST26
#  include <nuttx/mtd/mtd.h>
#  include "pic32mz_spi.h"
#endif

#include "ev49n51a.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int pic32mz_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n",
             ret);
    }
#endif

#if defined(CONFIG_MTD_SST26) && defined(CONFIG_PIC32MZ_SPI1)
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;

  /* SST26VF032B serial flash on SPI1 */

  spi = pic32mz_spibus_initialize(1);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI1\n");
    }
  else
    {
      mtd = sst26_initialize_spi(spi, 0);
      if (mtd == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to bind SPI1 to the SST26\n");
        }
      else
        {
          ret = ftl_initialize(SST26_MTD_MINOR, mtd);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: ftl_initialize() failed: %d\n", ret);
            }
        }
    }
#endif

  UNUSED(ret);
  return OK;
}
