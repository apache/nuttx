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

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_IOBINFO)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct iob_userstats_s g_iobuserstats[IOBUSER_NENTRIES];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_stats_onalloc
 *
 * Description:
 *   An IOB has just been allocated for the consumer. This is a hook for the
 *   IOB statistics to be updated when /proc/iobinfo is enabled.
 *
 * Input Parameters:
 *   consumerid - id representing who is consuming the IOB
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void iob_stats_onalloc(enum iob_user_e consumerid)
{
  DEBUGASSERT(consumerid < IOBUSER_NENTRIES);
  g_iobuserstats[consumerid].totalconsumed++;

  /* Increment the global statistic as well */

  g_iobuserstats[IOBUSER_GLOBAL].totalconsumed++;
}

/****************************************************************************
 * Name: iob_stats_onfree
 *
 * Description:
 *   An IOB has just been freed by the producer. This is a hook for the
 *   IOB statistics to be updated when /proc/iobinfo is enabled.
 *
 * Input Parameters:
 *   consumerid - id representing who is consuming the IOB
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void iob_stats_onfree(enum iob_user_e producerid)
{
  DEBUGASSERT(producerid < IOBUSER_NENTRIES);
  g_iobuserstats[producerid].totalproduced++;

  /* Increment the global statistic as well */

  g_iobuserstats[IOBUSER_GLOBAL].totalproduced++;
}

/****************************************************************************
 * Name: iob_getuserstats
 *
 * Description:
 *   Return a reference to the IOB usage statistics for the IOB
 *   consumer/producer
 *
 * Input Parameters:
 *   userid - id representing the IOB producer/consumer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

FAR struct iob_userstats_s * iob_getuserstats(enum iob_user_e userid)
{
  DEBUGASSERT(userid < IOBUSER_NENTRIES);
  return &g_iobuserstats[userid];
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * !CONFIG_FS_PROCFS_EXCLUDE_IOBINFO */
