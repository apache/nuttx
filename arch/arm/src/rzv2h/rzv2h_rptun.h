/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_rptun.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_RPTUN_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "r_mhu_api.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct resource_table;

struct rzv2h_rptun_config_s
{
  FAR const mhu_instance_t *mhu;

  /* Status bit for the MHU register used to notify the remote processor. */

  FAR volatile const uint32_t *tx_status;
  FAR struct resource_table *resource;
  FAR const char *cpuname;
  uint32_t notifyid;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_rptun_initialize
 *
 * Description:
 *   Register the RZ/V2H RPTUN lower half using the board-provided resource
 *   table and dedicated FSP MHU_B_NS instance.  The instance must not
 *   already be open.
 *
 ****************************************************************************/

int rzv2h_rptun_initialize(
  FAR const struct rzv2h_rptun_config_s *config);

#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_RPTUN_H */
