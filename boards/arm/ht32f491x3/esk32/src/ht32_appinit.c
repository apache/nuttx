/****************************************************************************
 * boards/arm/ht32f491x3/esk32/src/ht32_appinit.c
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

#include <errno.h>
#include <sys/types.h>
#include <syslog.h>

#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ht32_bringup(void)
{
  int ret = OK;
  int tmp;

#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
#  ifdef CONFIG_USERLED_LOWER
  tmp = userled_lower_initialize("/dev/userleds");
  if (tmp < 0 && tmp != -EEXIST)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/userleds: %d\n", tmp);
      ret = tmp;
    }
#  else
  board_userled_initialize();
#  endif
#endif

#ifdef CONFIG_PWM
  tmp = ht32_pwm_setup();
  if (tmp < 0 && tmp != -EEXIST)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm0: %d\n", tmp);
      ret = tmp;
    }
#endif

#ifdef CONFIG_FS_BINFS
  tmp = nx_mount(NULL, "/bin", "binfs", 0, NULL);
  if (tmp < 0 && tmp != -EBUSY)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount binfs at /bin: %d\n", tmp);
      ret = tmp;
    }
#endif

#ifdef CONFIG_FS_PROCFS
  tmp = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (tmp < 0 && tmp != -EBUSY)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", tmp);
      ret = tmp;
    }
#endif

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  (void)arg;

#ifdef CONFIG_BOARD_LATE_INITIALIZE
  return OK;
#else
  return ht32_bringup();
#endif
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  ht32_bringup();
}
#endif
