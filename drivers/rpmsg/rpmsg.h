/****************************************************************************
 * drivers/rpmsg/rpmsg.h
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

#ifndef __DRIVERS_RPMSG_RPMSG_H
#define __DRIVERS_RPMSG_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_RPMSG

#include <nuttx/rpmsg/rpmsg.h>

#include "rpmsg_ping.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rpmsg_s
{
  bool                         init;
  struct metal_list            bind;
  rmutex_t                     lock;
  struct metal_list            node;
  char                         local_cpuname[RPMSG_NAME_SIZE];
  char                         cpuname[RPMSG_NAME_SIZE];
  FAR const struct rpmsg_ops_s *ops;
#ifdef CONFIG_RPMSG_PING
  struct rpmsg_endpoint        ping;
#endif
#ifdef CONFIG_RPMSG_TEST
  struct rpmsg_endpoint        test;
#endif
  atomic_t                     signals;
};

/**
 * struct rpmsg_ops_s - Rpmsg device operations
 * wait: wait sem.
 * post: post sem.
 * get_cpuname: get cpu name.
 */

struct rpmsg_ops_s
{
  CODE int (*wait)(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
  CODE int (*post)(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
  CODE int (*ioctl)(FAR struct rpmsg_s *rpmsg, int cmd, unsigned long arg);
  CODE void (*panic)(FAR struct rpmsg_s *rpmsg);
  CODE void (*dump)(FAR struct rpmsg_s *rpmsg);
  CODE int (*get_timestamp)(FAR struct rpmsg_s *rpmsg, FAR const void *data,
                            FAR struct rpmsg_timestamp_s *ts);
};

CODE typedef int (*rpmsg_foreach_t)(FAR struct rpmsg_s *rpmsg,
                                    FAR void *args);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

static inline FAR struct rpmsg_device *
rpmsg_get_rdev_by_rpmsg(FAR struct rpmsg_s *rpmsg)
{
  if (!rpmsg)
    {
      return NULL;
    }

  return (FAR struct rpmsg_device *)(rpmsg + 1);
}

void rpmsg_modify_signals(FAR struct rpmsg_s *rpmsg,
                          int setflags, int clrflags);

void rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                   FAR const char *name, uint32_t dest);
void rpmsg_ns_unbind(FAR struct rpmsg_device *rdev,
                     FAR const char *name, uint32_t dest);

void rpmsg_device_created(FAR struct rpmsg_s *rpmsg);
void rpmsg_device_destroy(FAR struct rpmsg_s *rpmsg);

int rpmsg_register(FAR const char *path, FAR struct rpmsg_s *rpmsg,
                   FAR const struct rpmsg_ops_s *ops);
void rpmsg_unregister(FAR const char *path, FAR struct rpmsg_s *rpmsg);

int rpmsg_foreach(rpmsg_foreach_t handler, FAR void *args);

#endif /* CONFIG_RPMSG */
#endif /* __DRIVERS_RPMSG_RPMSG_H */
