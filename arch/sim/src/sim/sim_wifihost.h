/****************************************************************************
 * arch/sim/src/sim/sim_wifihost.h
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

#ifndef __ARCH_SIM_SRC_SIM_WIFIHOST_H
#define __ARCH_SIM_SRC_SIM_WIFIHOST_H

#if CONFIG_SIM_WIFIDEV_NUMBER != 0

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
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

struct sim_wifihost_lowerhalf_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s lower;     /* The netdev lowerhalf */

  void *wifi;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int sim_wifihost_init(struct sim_wifihost_lowerhalf_s *dev, int devidx);
bool sim_wifihost_connected(struct sim_wifihost_lowerhalf_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_WIFI_SIM */
#endif /* __ARCH_SIM_SRC_SIM_WIFIHOST_H */
