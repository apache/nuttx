/****************************************************************************
 * libs/libc/netdb/lib_dnsnotify.c
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

void dns_notify_nameserver(FAR const struct sockaddr *addr, socklen_t addrlen)
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
