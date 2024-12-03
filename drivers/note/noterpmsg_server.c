/****************************************************************************
 * drivers/note/noterpmsg_server.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/sched_note.h>

#include "noterpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct noterpmsg_server_s
{
  struct rpmsg_endpoint ept;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int noterpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                            FAR void *data, size_t len,
                            uint32_t src, FAR void *priv)
{
  /* The client ensures that the packet sent is the correct note
   * data packet.
   */

  sched_note_add(data, len);

  return 0;
}

static void noterpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct noterpmsg_server_s *srv = ept->priv;

  rpmsg_destroy_ept(ept);
  kmm_free(srv);
}

static bool noterpmsg_ns_match(FAR struct rpmsg_device *rdev,
                               FAR void *priv, FAR const char *name,
                               uint32_t dest)
{
  return !strcmp(name, NOTERPMSG_EPT_NAME);
}

static void noterpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                              FAR void *priv, FAR const char *name,
                              uint32_t dest)
{
  FAR struct noterpmsg_server_s *srv;
  int ret;

  srv = kmm_zalloc(sizeof(struct noterpmsg_server_s));
  if (srv == NULL)
    {
      return;
    }

  srv->ept.priv = srv;

  ret = rpmsg_create_ept(&srv->ept, rdev, NOTERPMSG_EPT_NAME,
                         RPMSG_ADDR_ANY, dest,
                         noterpmsg_ept_cb, noterpmsg_ns_unbind);
  if (ret < 0)
    {
      kmm_free(srv);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noterpmsg_server_init
 *
 * Description:
 *   Register a rmpsg channel to note.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int noterpmsg_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 noterpmsg_ns_match,
                                 noterpmsg_ns_bind);
}
