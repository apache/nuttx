/****************************************************************************
 * include/nuttx/net/vlan.h
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

#ifndef __INCLUDE_NUTTX_NET_VLAN_H
#define __INCLUDE_NUTTX_NET_VLAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/netdev_lowerhalf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VLAN tag definitions */

#define VLAN_PRIO_MASK  0xe000 /* Priority Code Point */
#define VLAN_PRIO_SHIFT 13
#define VLAN_CFI_MASK   0x1000 /* Canonical Format Indicator / Drop Eligible Indicator */
#define VLAN_VID_MASK   0x0fff /* VLAN Identifier */
#define VLAN_N_VID      4096

#ifdef CONFIG_NET_VLAN

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Passed in vlan_ioctl_args structure to determine behaviour. */

enum vlan_ioctl_cmds
{
  ADD_VLAN_CMD,
  DEL_VLAN_CMD,
  SET_VLAN_INGRESS_PRIORITY_CMD,
  SET_VLAN_EGRESS_PRIORITY_CMD,
  GET_VLAN_INGRESS_PRIORITY_CMD,
  GET_VLAN_EGRESS_PRIORITY_CMD,
  SET_VLAN_NAME_TYPE_CMD,
  SET_VLAN_FLAG_CMD,
  GET_VLAN_REALDEV_NAME_CMD, /* If this works, you know it's a VLAN device, btw */
  GET_VLAN_VID_CMD           /* Get the VID of this VLAN (specified by name) */
};

struct vlan_ioctl_args
{
  int cmd; /* Should be one of the vlan_ioctl_cmds enum above. */
  char device1[IFNAMSIZ];

  union
    {
      int16_t VID;
    } u;

  int16_t vlan_qos;
};

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: vlan_register
 *
 * Description:
 *   Create a new VLAN device and register it.
 *
 * Input Parameters:
 *   real - The real device to which the VLAN is attached
 *   vid  - VLAN ID
 *   prio - Default VLAN priority (PCP)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int vlan_register(FAR struct netdev_lowerhalf_s *real, uint16_t vid,
                  uint16_t prio);

/****************************************************************************
 * Name: vlan_unregister
 *
 * Description:
 *   Unregister a VLAN device.
 *
 * Input Parameters:
 *   dev - The VLAN device to be unregistered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void vlan_unregister(FAR struct netdev_lowerhalf_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_VLAN */
#endif /* __INCLUDE_NUTTX_NET_VLAN_H */
