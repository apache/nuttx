/****************************************************************************
 * drivers/rptun/rptun.h
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

#ifndef __DRIVERS_RPTUN_RPTUN_H
#define __DRIVERS_RPTUN_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/rptun/rptun.h>
#include <openamp/open_amp.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int rptun_buffer_nused(FAR struct rpmsg_virtio_device *rvdev, bool rx);
void rptun_dump(FAR struct rpmsg_virtio_device *rvdev);

int rptun_ping_init(FAR struct rpmsg_virtio_device *rvdev,
                    FAR struct rpmsg_endpoint *ept);
void rptun_ping_deinit(FAR struct rpmsg_endpoint *ept);
int rptun_ping(FAR struct rpmsg_endpoint *ept,
               FAR const struct rptun_ping_s *ping);

#endif /* __DRIVERS_RPTUN_RPTUN_H */
