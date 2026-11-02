/****************************************************************************
 * drivers/rpmsg/rpmsg_wakelock.h
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

#ifndef __DRIVERS_RPMSG_WAKELOCK_H
#define __DRIVERS_RPMSG_WAKELOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Internal function prototypes
 ****************************************************************************/

/****************************************************************************
 * Internal data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int rpmsg_wakelock(FAR struct rpmsg_endpoint *ept,
                   int cmd, unsigned long arg);

int rpmsg_wakelock_init(FAR struct rpmsg_device *rdev,
                        FAR struct rpmsg_endpoint *ept);

void rpmsg_wakelock_deinit(FAR struct rpmsg_endpoint *ept);

#endif /* __DRIVERS_RPMSG_WAKELOCK_H */
