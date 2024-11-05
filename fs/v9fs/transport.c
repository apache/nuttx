/****************************************************************************
 * fs/v9fs/transport.c
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

#include <string.h>

#include "client.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct v9fs_transport_ops_map_s
{
  FAR const char *type;
  FAR const struct v9fs_transport_ops_s *ops;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_V9FS_VIRTIO_9P
extern const struct v9fs_transport_ops_s g_virtio_9p_transport_ops;
#endif

static const struct v9fs_transport_ops_map_s g_transport_ops_map[] =
{
#ifdef CONFIG_V9FS_VIRTIO_9P
  { "virtio", &g_virtio_9p_transport_ops },
#endif
  { NULL, NULL},
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: v9fs_transport_request
 ****************************************************************************/

int v9fs_transport_request(FAR struct v9fs_transport_s *transport,
                           FAR struct v9fs_payload_s *payload)
{
  return transport->ops->request(transport, payload);
}

/****************************************************************************
 * Name: v9fs_transport_destroy
 ****************************************************************************/

void v9fs_transport_destroy(FAR struct v9fs_transport_s *transport)
{
  transport->ops->destroy(transport);
}

/****************************************************************************
 * Name: v9fs_register_transport
 ****************************************************************************/

int v9fs_transport_create(FAR struct v9fs_transport_s **transport,
                          FAR const char *trans_type, FAR const char *data)
{
  FAR const struct v9fs_transport_ops_map_s *map;

  /* Find the corresponding driver layer */

  for (map = g_transport_ops_map; map->type; map++)
    {
      if (strcmp(trans_type, map->type) == 0)
        {
          break;
        }
    }

  if (map->ops == NULL)
    {
      return -ENOENT;
    }

  return map->ops->create(transport, data);
}
