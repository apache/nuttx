/****************************************************************************
 * net/procfs/netdev_statistics.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Anthony Merlino <anthony@vergeeaero.com>
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

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
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
 *   Return a reference to the IOB usage statistics for the IOB consumer/producer
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
