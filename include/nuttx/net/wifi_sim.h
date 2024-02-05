/****************************************************************************
 * include/nuttx/net/wifi_sim.h
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

#ifndef __INCLUDE_NUTTX_NET_WIFI_SIM_H
#define __INCLUDE_NUTTX_NET_WIFI_SIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_DRIVERS_WIFI_SIM

#include <nuttx/net/netdev_lowerhalf.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct wifi_sim_lowerhalf_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s lower;     /* The netdev lowerhalf */

  FAR void *wifi;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  wifi_sim_init(FAR struct wifi_sim_lowerhalf_s *netdev);
void wifi_sim_remove(FAR struct wifi_sim_lowerhalf_s *netdev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_WIFI_SIM */
#endif /* __INCLUDE_NUTTX_NET_WIFI_SIM_H */
