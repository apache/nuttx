/****************************************************************************
 * include/nuttx/net/rpmsgdrv.h
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

#ifndef __INCLUDE_NUTTX_NET_RPMSGDRV_H
#define __INCLUDE_NUTTX_NET_RPMSGDRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/net.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NET_RPMSG_DRV
int net_rpmsg_drv_init(FAR const char *cpuname,
                       FAR const char *devname,
                       enum net_lltype_e lltype);
#else
#define net_rpmsg_drv_init(cpuname, devname, lltye)
#endif

#endif /* __INCLUDE_NUTTX_NET_RPMSGDRV_H */
