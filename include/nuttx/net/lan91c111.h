/****************************************************************************
 * include/nuttx/net/lan91c111.h
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

#ifndef __INCLUDE_NUTTX_NET_LAN91C111_H
#define __INCLUDE_NUTTX_NET_LAN91C111_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lan91c111_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   base - The controller base address
 *   irq  - The controller irq number
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LAN91C111
int lan91c111_initialize(uintptr_t base, int irq);
#else
#  define lan91c111_initialize(base, irq) 0
#endif /* CONFIG_NET_LAN91C111 */

#endif /* __INCLUDE_NUTTX_NET_LAN91C111_H */
