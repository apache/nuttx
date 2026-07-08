/****************************************************************************
 * binfmt/binfmt_checkexec.c
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

#include <sys/stat.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_checkexecperm
 *
 * Description:
 *   Verify that the calling task has execute permission on the file
 *   described by 'bin'.  The file owner, group, and mode must already be
 *   populated in the binary_s structure before calling this function.
 *
 * Input Parameters:
 *   bin - Pointer to the binary descriptor with uid, gid, and mode set.
 *
 * Returned Value:
 *   Zero (OK) on success; -EACCES if execute permission is denied.
 *
 ****************************************************************************/

int binfmt_checkexecperm(FAR struct binary_s *bin)
{
  FAR struct tcb_s *rtcb;
  mode_t xbits;

  rtcb = nxsched_self();

  if (bin == NULL || rtcb == NULL || rtcb->group == NULL)
    {
      return OK;
    }

  if (rtcb->group->tg_euid == 0)
    {
      /* Root can execute any file that has at least one execute bit set */

      if ((bin->mode & (S_IXUSR | S_IXGRP | S_IXOTH)) == 0)
        {
          return -EACCES;
        }

      return OK;
    }

  if (rtcb->group->tg_euid == bin->uid)
    {
      xbits = S_IXUSR;
    }
  else if (rtcb->group->tg_egid == bin->gid)
    {
      xbits = S_IXGRP;
    }
  else
    {
      xbits = S_IXOTH;
    }

  if ((bin->mode & xbits) == 0)
    {
      return -EACCES;
    }

  return OK;
}
