/****************************************************************************
 * arch/sim/src/sim/sim_rpmsg_port_uart.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/rpmsg/rpmsg_port.h>

#include "sim_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_rpmsg_port_uart_init(const char *localcpu, const char *remotecpu,
                             const char *uartpath)
{
  struct rpmsg_port_config_s config =
    {
      .remotecpu = remotecpu,
      .txnum = 10,
      .rxnum = 10,
      .txlen = 2048,
      .rxlen = 2048
    };

  return rpmsg_port_uart_initialize(&config, uartpath, localcpu);
}

