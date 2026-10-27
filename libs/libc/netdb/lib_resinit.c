/****************************************************************************
 * libs/libc/netdb/lib_resinit.c
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

#include <assert.h>
#include <errno.h>
#include <resolv.h>

#include <nuttx/lib/lib.h>
#include <nuttx/mutex.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: res_init
 *
 * Description:
 *   Initialize the resolver.  It sets the defaults of the resolver state
 *   (_res) and RES_INIT, as BIND does.  The DNS client gets the name
 *   servers at each query: from CONFIG_NETDB_RESOLVCONF_PATH with
 *   CONFIG_NETDB_RESOLVCONF, otherwise from the list that
 *   dns_add_nameserver() changes.  The resolver state does not change
 *   the queries.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; -1 with errno ENOMEM if the state cannot be
 *   allocated.
 *
 ****************************************************************************/

int res_init(void)
{
  FAR struct __res_state *res = __res_state();

  if (res == NULL)
    {
      set_errno(ENOMEM);
      return -1;
    }

  /* The defaults of BIND for the fields that the program did not set.
   * NuttX's resolver does not use them.
   */

  if (res->retrans == 0)
    {
      res->retrans = RES_TIMEOUT;
    }

  if (res->retry == 0)
    {
      res->retry = RES_DFLRETRY;
    }

  if ((res->options & RES_INIT) == 0)
    {
      res->options = RES_DEFAULT;
      res->ndots = 1;
    }

  res->options |= RES_INIT;
  return 0;
}

/****************************************************************************
 * Name: __res_state
 *
 * Description:
 *   Give the resolver state of the process (the _res of <resolv.h>): the
 *   state of the task group, which is allocated at the first call.  NULL
 *   if it cannot be allocated.
 *
 ****************************************************************************/

FAR struct __res_state *__res_state(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct __res_state *res;

  DEBUGASSERT(info != NULL);

  nxrmutex_lock(&info->ta_lock);
  if (info->ta_res == NULL)
    {
      info->ta_res = lib_zalloc(sizeof(struct __res_state));
    }

  res = info->ta_res;
  nxrmutex_unlock(&info->ta_lock);

  return res;
}
