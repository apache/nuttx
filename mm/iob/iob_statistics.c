/****************************************************************************
 * mm/iob/iob_statistics.c
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

#include <nuttx/mm/iob.h>

#include "iob.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_IOBINFO)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_getuserstats
 *
 * Description:
 *   Return a reference to the IOB usage statistics for the IOB
 *   consumer/producer
 *
 * Input Parameters:
 *   stats - point to IOB usage statistics
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void iob_getstats(FAR struct iob_stats_s *stats)
{
  stats->ntotal = CONFIG_IOB_NBUFFERS;

  nxsem_get_value(&g_iob_sem, &stats->nfree);
  if (stats->nfree < 0)
    {
      stats->nwait = -stats->nfree;
      stats->nfree = 0;
    }
  else
    {
      stats->nwait = 0;
    }

#if CONFIG_IOB_THROTTLE > 0
  nxsem_get_value(&g_throttle_sem, &stats->nthrottle);
  if (stats->nthrottle < 0)
    {
      stats->nthrottle = -stats->nthrottle;
    }
  else
#endif
    {
      stats->nthrottle = 0;
    }
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * !CONFIG_FS_PROCFS_EXCLUDE_IOBINFO */
