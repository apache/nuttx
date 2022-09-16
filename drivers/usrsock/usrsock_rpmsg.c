/****************************************************************************
 * drivers/usrsock/usrsock_rpmsg.c
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

#include <nuttx/net/dns.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/semaphore.h>

#include "usrsock_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usrsock_rpmsg_s
{
  sem_t                 wait;
  struct rpmsg_endpoint ept;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT
static int usrsock_rpmsg_send_dns_request(FAR void *arg,
                                          FAR struct sockaddr *addr,
                                          socklen_t addrlen);
#endif

static void usrsock_rpmsg_ns_bound(FAR struct rpmsg_endpoint *ept);
static void usrsock_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept);

static int usrsock_rpmsg_dns_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR struct usrsock_rpmsg_dns_event_s *dns);

static int usrsock_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv);

static void usrsock_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_);
static void usrsock_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usrsock_rpmsg_s g_usrsock_rpmsg =
{
  NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NETDB_DNSCLIENT
static int usrsock_rpmsg_send_dns_request(FAR void *arg,
                                          FAR struct sockaddr *addr,
                                          socklen_t addrlen)
{
  FAR struct usrsock_rpmsg_s *priv = arg;
  FAR struct rpmsg_endpoint *ept = &priv->ept;
  FAR struct usrsock_rpmsg_dns_request_s *dns;
  uint32_t len;

  dns = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (dns == NULL)
    {
      return -ENOMEM;
    }

  dns->head.reqid = USRSOCK_RPMSG_DNS_REQUEST;
  dns->head.xid = 0;
  dns->head.reserved = 0;
  dns->addrlen = addrlen;
  memcpy(dns + 1, addr, addrlen);

  return rpmsg_send_nocopy(ept, dns, sizeof(*dns) + addrlen);
}
#endif

static void usrsock_rpmsg_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct usrsock_rpmsg_s *priv = ept->priv;

#ifdef CONFIG_NETDB_DNSCLIENT
  dns_register_notify(usrsock_rpmsg_send_dns_request, priv);
#endif
  nxsem_post(&priv->wait);
}

static void usrsock_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct usrsock_rpmsg_s *priv = ept->priv;

#ifdef CONFIG_NETDB_DNSCLIENT
  dns_unregister_notify(usrsock_rpmsg_send_dns_request, priv);
#endif
  nxsem_wait_uninterruptible(&priv->wait);
}

static int usrsock_rpmsg_dns_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR struct usrsock_rpmsg_dns_event_s *dns)
{
#ifdef CONFIG_NETDB_DNSCLIENT
  dns_add_nameserver((FAR struct sockaddr *)(dns + 1), dns->addrlen);
#endif
  return 0;
}

static int usrsock_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct usrsock_message_common_s *common = data;

  if (common->msgid == USRSOCK_RPMSG_DNS_EVENT)
    {
      return usrsock_rpmsg_dns_handler(ept, data);
    }
  else
    {
      return usrsock_response(data, len, NULL);
    }
}

static void usrsock_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_)
{
  FAR struct usrsock_rpmsg_s *priv = priv_;

  if (!strcmp(CONFIG_NET_USRSOCK_RPMSG_CPUNAME, rpmsg_get_cpuname(rdev)))
    {
      priv->ept.priv = priv;
      priv->ept.ns_bound_cb = usrsock_rpmsg_ns_bound;

      rpmsg_create_ept(&priv->ept, rdev, USRSOCK_RPMSG_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       usrsock_rpmsg_ept_cb, usrsock_rpmsg_ns_unbind);
    }
}

static void usrsock_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_)
{
  FAR struct usrsock_rpmsg_s *priv = priv_;

  if (!strcmp(CONFIG_NET_USRSOCK_RPMSG_CPUNAME, rpmsg_get_cpuname(rdev)))
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_request
 ****************************************************************************/

int usrsock_request(FAR struct iovec *iov, unsigned int iovcnt)
{
  FAR struct usrsock_rpmsg_s *priv = &g_usrsock_rpmsg;
  size_t pos = 0;
  int ret = 0;

  nxsem_get_value(&priv->wait, &ret);
  if (ret <= 0)
    {
      net_lockedwait_uninterruptible(&priv->wait);
      nxsem_post(&priv->wait);
    }

  for (; ; )
    {
      FAR void *buf;
      uint32_t len;
      bool done;

      buf = rpmsg_get_tx_payload_buffer(&priv->ept, &len, true);
      if (buf == NULL)
        {
          ret = -ENOMEM;
          break;
        }

      ret = usrsock_iovec_get(buf, len, iov, iovcnt, pos, &done);
      if (ret < 0)
        {
          break;
        }

      pos += ret;
      ret = rpmsg_send_nocopy(&priv->ept, buf, ret);
      if (ret < 0)
        {
          break;
        }

      if (done)
        {
          break;
        }
    }

  return ret;
}

void usrsock_register(void)
{
  rpmsg_register_callback(&g_usrsock_rpmsg,
                          usrsock_rpmsg_device_created,
                          usrsock_rpmsg_device_destroy,
                          NULL,
                          NULL);
}
