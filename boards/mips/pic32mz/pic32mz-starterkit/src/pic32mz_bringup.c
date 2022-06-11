/****************************************************************************
 * boards/mips/pic32mz/pic32mz-starterkit/src/pic32mz_bringup.c
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

#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "pic32mz-starterkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_sdinitialize
 *
 * Description:
 *   Initialize SPI-based microSD.
 *
 ****************************************************************************/

#ifdef PIC32MZ_HAVE_MMCSD
static int nsh_sdinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  spi = pic32mz_spibus_initialize(PIC32MZ_MMCSDSPIPORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             PIC32MZ_MMCSDSPIPORTNO);
      ret = -ENODEV;
      goto errout;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port %d\n",
         PIC32MZ_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  ret = mmcsd_spislotinitialize(PIC32MZ_MMCSDMINOR, PIC32MZ_MMCSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
             PIC32MZ_MMCSDSPIPORTNO, PIC32MZ_MMCSLOTNO, ret);
      goto errout;
    }

  syslog(LOG_INFO, "Successfully bound SPI port %d to MMC/SD slot %d\n",
         PIC32MZ_MMCSDSPIPORTNO, PIC32MZ_MMCSLOTNO);
  return OK;

errout:
  return ret;
}
#else
#  define nsh_sdinitialize() (OK)
#endif

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
  int ret;

  /* Initialize SPI-based microSD */

  ret = nsh_sdinitialize();

  return ret;
}
