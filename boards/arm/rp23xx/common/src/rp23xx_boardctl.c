/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_boardctl.c
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

#include <nuttx/board.h>
#include <arch/chip/pm.h>

#include "rp23xx_pm.h"

#ifdef CONFIG_BOARDCTL_IOCTL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ioctl
 *
 * Description:
 *   Handle the rp23xx boardctl() commands of arch/chip/pm.h.
 *
 ****************************************************************************/

int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  switch (cmd)
    {
#ifdef CONFIG_RP23XX_PM_SUSPEND
      case BOARDIOC_RP23XX_SUSPEND:
        {
          FAR struct rp23xx_suspend_s *suspend =
            (FAR struct rp23xx_suspend_s *)arg;
          int ret;

          if (suspend == NULL)
            {
              return -EINVAL;
            }

          ret = rp23xx_pm_suspend(suspend->wake_ms);
          suspend->wake_source = rp23xx_pm_wake_source();
          return ret;
        }
#endif

      default:
        return -ENOTTY;
    }
}

#endif /* CONFIG_BOARDCTL_IOCTL */
