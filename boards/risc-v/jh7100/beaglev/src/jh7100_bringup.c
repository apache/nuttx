/****************************************************************************
<<<<<<< HEAD
 * boards/risc-v/jh7100/beaglev/src/jh7100_bringup.c
=======
 * boards/risc-v/jh7100/smartl-jh7100/src/jh7100_bringup.c
>>>>>>> 577374631f (Add rest of BeagleV Starlight JH7100 and JH7110 work.)
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
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/drivers/ramdisk.h>

#include "jh7100.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jh7100_bringup
 ****************************************************************************/

int jh7100_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
<<<<<<< HEAD
      /* serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret); */
=======
//      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
>>>>>>> 577374631f (Add rest of BeagleV Starlight JH7100 and JH7110 work.)
    }
#endif

  return ret;
}
