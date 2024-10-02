/****************************************************************************
 * include/nuttx/rpmsg/rpmsg_router.h
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

#ifndef __INCLUDE_NUTTX_RPMSG_RPMSG_ROUTER_H
#define __INCLUDE_NUTTX_RPMSG_RPMSG_ROUTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RPMSG_ROUTER

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rpmsg_router_hub_init
 *
 * Description:
 *   This function is used to initialize the rpmsg router hub.
 *
 * Parameters:
 *   edge0 - edge cpu name
 *   edge1 - edge cpu name
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsg_router_hub_init(FAR const char *edge0,
                          FAR const char *edge1);

/****************************************************************************
 * Name: rpmsg_router_edge_init
 *
 * Description:
 *   This function is used to initialize the edge core.
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsg_router_edge_init(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPMSG_ROUTER */

#endif /* __INCLUDE_NUTTX_RPMSG_RPMSG_ROUTER_H */
