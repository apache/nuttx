/****************************************************************************
 * arch/sim/src/sim/sim_hostcan.h
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

#ifndef __ARCH_SIM_SRC_SIM_CAN_H
#define __ARCH_SIM_SRC_SIM_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __SIM__
#  include "config.h"
#endif

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reserve data for maximum CANFD frame */

#define SIM_CAN_MAX_DLEN 64

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sim_can_s
{
  int fd;
  bool ifup;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Host CAN interface */

int host_can_init(struct sim_can_s *can, int devidx);
int host_can_read(struct sim_can_s *can, void *frame);
int host_can_send(struct sim_can_s *can, void *frame, size_t len);
int host_can_ifup(struct sim_can_s *can);
int host_can_ifdown(struct sim_can_s *can);
bool host_can_avail(struct sim_can_s *can);

#endif /* __ARCH_SIM_SRC_SIM_CAN_H */
