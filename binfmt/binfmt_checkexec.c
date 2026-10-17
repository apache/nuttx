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
 *   Verify that the calling task has execute permission on a file with
 *   the given mode, owner and group.
 *
 * Input Parameters:
 *   mode - The mode of the file
 *   uid  - The owner of the file
 *   gid  - The group of the file
 *
 * Returned Value:
 *   Zero (OK) on success; -EACCES if execute permission is denied.
 *
 ****************************************************************************/

int binfmt_checkexecperm(mode_t mode, uid_t uid, gid_t gid)
{
#ifdef CONFIG_SCHED_USER_IDENTITY
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;
#endif
  mode_t xbits = S_IXUSR | S_IXGRP | S_IXOTH;

#ifdef CONFIG_SCHED_USER_IDENTITY
  rtcb = nxsched_self();
  if (rtcb == NULL || rtcb->group == NULL)
    {
      return OK;
    }

  rgroup = rtcb->group;

  /* Root can execute any file that has at least one execute bit set */

  if (rgroup->tg_euid != 0)
    {
      if (rgroup->tg_euid == uid)
        {
          xbits = S_IXUSR;
        }
      else if (nxsched_has_gid(rtcb, gid))
        {
          xbits = S_IXGRP;
        }
      else
        {
          xbits = S_IXOTH;
        }
    }
#else
  /* Without user identities, any execute bit is enough */

  UNUSED(uid);
  UNUSED(gid);
#endif

  if ((mode & xbits) == 0)
    {
      return -EACCES;
    }

  return OK;
}
