/****************************************************************************
 * drivers/rpmsg/rpmsg_char.h
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

#ifndef __DRIVERS_RPMSG_RPMSG_CHAR_H
#define __DRIVERS_RPMSG_RPMSG_CHAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/rpmsg/rpmsg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_RPMSG_CHAR
#  define rpmsg_char_init(chr)
#  define rpmsg_char_deinit(chr)
#else

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rpmsg_char_s
{
  struct list_node eptdevs;
  rmutex_t         lock;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void rpmsg_char_init(FAR struct rpmsg_char_s *chr);
void rpmsg_char_deinit(FAR struct rpmsg_char_s *chr);
int rpmsg_char_create_eptdev(FAR struct rpmsg_char_s *chr,
                             FAR const struct rpmsg_endpoint_info *info);
void rpmsg_char_destroy_eptdev(FAR struct rpmsg_char_s *chr,
                               FAR const struct rpmsg_endpoint_info *info);

#endif /* CONFIG_RPMSG_CHAR */
#endif /* __DRIVERS_RPMSG_RPMSG_CHAR_H */
