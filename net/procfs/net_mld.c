/****************************************************************************
 * net/procfs/net_mld.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* Output format:
 *
 *   Joins: xxxx Leaves: xxxx
 *   Sent       Sched Sent
 *     Queries: xxxx  xxxx
 *     Reports:
 *       Ver 1: ----  xxxx
 *       Ver 2: xxxx  xxxx
 *     Done:    xxxx  xxxx
 *   Received:
 *     Queries:
 *       Gen:   xxxx
 *       MAS:   xxxx
 *       MASSQ: xxxx
 *       Ucast: xxxx
 *       Bad:   xxxx
 *     Reports:
 *       Ver 1: xxxx
 *       Ver 2: xxxx
 *     Done:    xxxx
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netstats.h>

#include "procfs/procfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_NET) && defined(CONFIG_NET_STATISTICS)

#if defined(CONFIG_NET_ICMPv6) || defined(CONFIG_NET_MLD)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Line generating functions */

static int netprocfs_joinleave(FAR struct netprocfs_file_s *netfile);
static int netprocfs_queries_sent(FAR struct netprocfs_file_s *netfile);
static int netprocfs_reports_sent(FAR struct netprocfs_file_s *netfile);
static int netprocfs_done_sent(FAR struct netprocfs_file_s *netfile);
static int netprocfs_queries_received_1(FAR struct netprocfs_file_s *netfile);
static int netprocfs_queries_received_2(FAR struct netprocfs_file_s *netfile);
static int netprocfs_reports_received(FAR struct netprocfs_file_s *netfile);
static int netprocfs_done_received(FAR struct netprocfs_file_s *netfile);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Line generating functions */

static const linegen_t g_mld_linegen[] =
{
  netprocfs_joinleave,
  netprocfs_queries_sent,
  netprocfs_reports_sent,
  netprocfs_done_sent,
  netprocfs_queries_received_1,
  netprocfs_queries_received_2,
  netprocfs_reports_received,
  netprocfs_done_received
};

#define NSTAT_LINES (sizeof(g_mld_linegen) / sizeof(linegen_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_joinleave
 ****************************************************************************/

static int netprocfs_joinleave(FAR struct netprocfs_file_s *netfile)
{
  int len;

  len  = snprintf(netfile->line, NET_LINELEN, "Joins: %04x ",
                  g_netstats.mld.njoins);
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "Leaves: %04x\n",
                  g_netstats.mld.nleaves);
  return len;
}

/****************************************************************************
 * Name: netprocfs_queries_sent
 ****************************************************************************/

static int netprocfs_queries_sent(FAR struct netprocfs_file_s *netfile)
{
  int len;

  len = snprintf(netfile->line, NET_LINELEN, "Sent       Sched Sent\n");
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "  Queries: %04x  %04x\n",
                  g_netstats.mld.query_sched, g_netstats.mld.query_sent);
  return len;
}

/****************************************************************************
 * Name: netprocfs_reports_sent
 ****************************************************************************/

static int netprocfs_reports_sent(FAR struct netprocfs_file_s *netfile)
{
  int len;

  len  = snprintf(netfile->line, NET_LINELEN, "  Reports:\n");
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    Ver 1: ----  %04x\n",
                  g_netstats.mld.v1report_sent);
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    Ver 2: %04x  %04x\n",
                  g_netstats.mld.report_sched, g_netstats.mld.v2report_sent);
  return len;
}

/****************************************************************************
 * Name: netprocfs_done_sent
 ****************************************************************************/

static int netprocfs_done_sent(FAR struct netprocfs_file_s *netfile)
{
  return snprintf(netfile->line, NET_LINELEN, "  Done:    %04x  %04x\n",
                  g_netstats.mld.done_sched, g_netstats.mld.done_sent);
}

/****************************************************************************
 * Name: netprocfs_queries_received_1 and _2
 ****************************************************************************/

static int netprocfs_queries_received_1(FAR struct netprocfs_file_s *netfile)
{
  int len;

  len  = snprintf(netfile->line, NET_LINELEN, "Received:\n");
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "  Queries:\n");
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    Gen:   %04x\n",
                  g_netstats.mld.gm_query_received);
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    MAS:   %04x\n",
                  g_netstats.mld.mas_query_received);
  return len;
}

static int netprocfs_queries_received_2(FAR struct netprocfs_file_s *netfile)
{
  int len;

  len  = snprintf(netfile->line, NET_LINELEN,
                  "    MASS:  %04x\n",
                  g_netstats.mld.mass_query_received);
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    Ucast: %04x\n",
                  g_netstats.mld.ucast_query_received);
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    Bad:   %04x\n",
                  g_netstats.mld.bad_query_received);
  return len;
}

/****************************************************************************
 * Name: netprocfs_reports_received
 ****************************************************************************/

static int netprocfs_reports_received(FAR struct netprocfs_file_s *netfile)
{
  int len;

  len  = snprintf(netfile->line, NET_LINELEN, "  Reports:\n");
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    Ver 1: %04x\n",
                  g_netstats.mld.v1report_received);
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "    Ver 2: %04x\n",
                  g_netstats.mld.v2report_received);
  return len;
}

/****************************************************************************
 * Name: netprocfs_done_received
 ****************************************************************************/

static int netprocfs_done_received(FAR struct netprocfs_file_s *netfile)
{
  return snprintf(netfile->line, NET_LINELEN , "  Done:    %04x\n",
                  g_netstats.mld.done_received);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_read_mldstats
 *
 * Description:
 *   Read and format MLD statistics.
 *
 * Input Parameters:
 *   priv - A reference to the network procfs file structure
 *   buffer - The user-provided buffer into which network status will be
 *            returned.
 *   bulen  - The size in bytes of the user provided buffer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

ssize_t netprocfs_read_mldstats(FAR struct netprocfs_file_s *priv,
                                FAR char *buffer, size_t buflen)
{
  return netprocfs_read_linegen(priv, buffer, buflen, g_mld_linegen, NSTAT_LINES);
}

#endif /* CONFIG_NET_ICMPv6 || CONFIG_NET_MLD */
#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * !CONFIG_FS_PROCFS_EXCLUDE_NET */
