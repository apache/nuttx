/****************************************************************************
 * boards/mips/pic32mz/flipnclick-pic32mz/src/pic32mz_bringup.c
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

#include "flipnclick-pic32mz.h"

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

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n",
            ret);
    }
#endif

#if defined(HAVE_SSD1306) && !defined(CONFIG_NXSTART_EXTERNINIT)
  /* Configure the SSD1306 OLED */

  if (pic32mz_graphics_setup(0) == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure the SSD1306 OLED\n");
    }
#endif

  UNUSED(ret);
  return OK;
}
