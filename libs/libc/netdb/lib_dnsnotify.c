/****************************************************************************
 * libs/libc/netdb/lib_dnsnotify.c
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

#include <nuttx/net/dns.h>
#include <queue.h>

#include "libc.h"
#include "netdb/lib_dns.h"

#ifdef CONFIG_NETDB_DNSCLIENT

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct dns_notify_s
{
  struct dq_entry_s entry;   /* Supports a doubly linked list */
  dns_callback_t callback;
  FAR void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dq_queue_t g_dns_notify;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_register_notify
 *
 * Description:
 *   This function is called in order to receive the nameserver change.
 *
 ****************************************************************************/

int dns_register_notify(dns_callback_t callback, FAR void *arg)
{
  FAR struct dns_notify_s *notify;

  notify = lib_malloc(sizeof(*notify));
  if (notify == NULL)
    {
      return -ENOMEM;
    }

  notify->callback = callback;
  notify->arg      = arg;

  dns_semtake();
  dq_addlast(&notify->entry, &g_dns_notify);
  dns_semgive();

  /* Notify the existed nameserver */

  dns_foreach_nameserver(callback, arg);
  return OK;
}

/****************************************************************************
 * Name: dns_unregister_notify
 *
 * Description:
 *   This function is called in order to unsubscribe the notification.
 *
 ****************************************************************************/

int dns_unregister_notify(dns_callback_t callback, FAR void *arg)
{
  FAR dq_entry_t *entry;

  dns_semtake();
  for (entry = dq_peek(&g_dns_notify); entry; entry = dq_next(entry))
    {
      FAR struct dns_notify_s *notify = (FAR struct dns_notify_s *)entry;

      if (notify->callback == callback && notify->arg == arg)
        {
          dq_rem(&notify->entry, &g_dns_notify);
          dns_semgive();
          lib_free(notify);
          return OK;
        }
    }

  dns_semgive();
  return -EINVAL;
}

/****************************************************************************
 * Name: dns_notify_nameserver
 ****************************************************************************/

void dns_notify_nameserver(FAR const struct sockaddr *addr,
                           socklen_t addrlen)
{
  FAR dq_entry_t *entry;

  dns_semtake();
  for (entry = dq_peek(&g_dns_notify); entry; entry = dq_next(entry))
    {
      FAR struct dns_notify_s *notify = (FAR struct dns_notify_s *)entry;
      notify->callback(notify->arg, (FAR struct sockaddr *)addr, addrlen);
    }

  dns_semgive();
}

#endif /* CONFIG_NETDB_DNSCLIENT */
