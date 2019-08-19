/****************************************************************************
 * boards/arm/stm32l4/b-l475e-iot01a/src/stm32_bringup.c
 *
 *   Copyright (C) 2017-2019 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mount.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32l4_qspi.h"
#include "b-l475e-iot01a.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_bringup
 *
 * Description:
 *   Called either by board_initialize() if CONFIG_BOARD_LATE_INITIALIZE or by
 *   board_app_initialize if CONFIG_LIB_BOARDCTL is selected.  This function
 *   initializes and configures all on-board features appropriate for the
 *   selected configuration.
 *
 ****************************************************************************/

int stm32l4_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
#ifdef CONFIG_USERLED_LOWER
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#else
  /* Enable USER LED support for some other purpose */

  board_userled_initialize();
#endif /* CONFIG_USERLED_LOWER */
#endif /* CONFIG_USERLED && !CONFIG_ARCH_LEDS */

#ifdef HAVE_MX25R6435F
  {
    /* Create an instance of the STM32L4 QSPI device driver */

    FAR struct qspi_dev_s *g_qspi;
    FAR struct mtd_dev_s *g_mtd_fs;

    g_qspi = stm32l4_qspi_initialize(0);
    if (g_qspi == NULL)
      {
        syslog(LOG_ERR, "ERROR: stm32l4_qspi_initialize failed\n");
        return -EIO;
      }

    /* Use the QSPI device instance to initialize the
     * MX25R6435F flash device.
     */

    g_mtd_fs = mx25rxx_initialize(g_qspi, true);
    if (!g_mtd_fs)
      {
        syslog(LOG_ERR, "ERROR: mx25rxx_initialize failed\n");
        return -EIO;
      }

#ifdef CONFIG_B_L475E_IOT01A_MTD_PART
      {
        /* Create partitions on external flash memory */

        int partno;
        int partsize;
        int partoffset;
        int partszbytes;
        int erasesize;
        FAR struct mtd_geometry_s geo;
        const char *ptr = CONFIG_B_L475E_IOT01A_MTD_PART_LIST;
        FAR struct mtd_dev_s *mtd_part;
        char  partref[4];

        /* Now create a partition on the FLASH device */

        partno = 0;
        partoffset = 0;

        /* Query MTD geometry */

        ret = MTD_IOCTL(g_mtd_fs, MTDIOC_GEOMETRY,
                        (unsigned long)(uintptr_t)&geo);
        if (ret < 0)
          {
            syslog(LOG_ERR, "ERROR: MTDIOC_GEOMETRY failed\n");
            return ret;
          }

        /* Get the Flash erase size */

        erasesize = geo.erasesize;

        while (*ptr != '\0')
          {
            /* Get the partition size */

            partsize = atoi(ptr);
            if (partsize <= 0)
              {
                syslog(LOG_ERR, "Error while processing <%s>\n", ptr);
                goto process_next_part;
              }

            partszbytes = (partsize << 10); /* partsize is defined in KB */

            if ((partszbytes < erasesize) || ((partszbytes % erasesize) != 0))
              {
                syslog(LOG_ERR, "Invalid partition size: %d bytes\n",
                       partszbytes);
                partszbytes = (partszbytes+erasesize) & -erasesize;
              }

            mtd_part = mtd_partition(g_mtd_fs, partoffset,
                                     partszbytes / geo.blocksize);
            partoffset += partszbytes / geo.blocksize;

            if (!mtd_part)
              {
                syslog(LOG_ERR, "Failed to create part %d, size=%d\n",
                       partno, partsize);
                goto process_next_part;
              }

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
            /* Now initialize a SMART Flash block device and bind it to the MTD
             * device */

            sprintf(partref, "p%d", partno);
            smart_initialize(CONFIG_B_L475E_IOT01A_MTD_FLASH_MINOR,
                             mtd_part, partref);
#endif

process_next_part:
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
#else /* CONFIG_B_L475E_IOT01A_MTD_PART */

#ifdef HAVE_MX25R6435F_SMARTFS
      /* Configure the device with no partition support */

      smart_initialize(CONFIG_B_L475E_IOT01A_MTD_FLASH_MINOR, g_mtd_fs, NULL);
#endif /* HAVE_MX25R6435F_SMARTFS */

#endif /* CONFIG_B_L475E_IOT01A_MTD_PART */
  }
#endif /* HAVE_MX25R6435F */

#ifdef HAVE_SPSGRF
  /* Configure Spirit/SPSGRF wireless */

  ret = stm32l4_spirit_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_spirit_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_TIMER
/* Register timer drivers */

  ret = stm32l4_timer_driver_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup TIM1 at /dev/timer0: %d\n", ret);
    }
#endif

  return ret;
}
