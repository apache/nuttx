/****************************************************************************
 * mm/iob/iob_navail.c
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

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_navail
 *
 * Description:
 *   Return the number of available IOBs.
 *
 ****************************************************************************/

int iob_navail(bool throttled)
{
  int navail = 0;
  int ret;

#if CONFIG_IOB_NBUFFERS > 0
  /* Get the value of the IOB counting semaphores */

  ret = nxsem_get_value(&g_iob_sem, &navail);
  if (ret >= 0)
    {
      ret = navail;

#if CONFIG_IOB_THROTTLE > 0
      /* Subtract the throttle value is so requested */

      if (throttled)
        {
          ret -= CONFIG_IOB_THROTTLE;
        }
#endif

      if (ret < 0)
        {
          ret = 0;
        }
    }

#else
  ret = navail;
#endif

  return ret;
}

/****************************************************************************
 * Name: iob_qentry_navail
 *
 * Description:
 *   Return the number of available IOB chains.
 *
 ****************************************************************************/

int iob_qentry_navail(void)
{
  int navail = 0;
  int ret;

#if CONFIG_IOB_NCHAINS > 0
  /* Get the value of the IOB chain qentry counting semaphores */

  ret = nxsem_get_value(&g_qentry_sem, &navail);
  if (ret >= 0)
    {
      ret = navail;
      if (ret < 0)
        {
          ret = 0;
        }
    }

#else
  ret = navail;
#endif

  return ret;
}
