/****************************************************************************
 * arch/sim/src/sim/up_hcisocket_host.h
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

#ifndef _ARCH_SIM_SRC_SIM_HCISOCKET_HOST_H_
#define _ARCH_SIM_SRC_SIM_HCISOCKET_HOST_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int bthcisock_host_send(int fd, uint8_t pkt_type, uint8_t *data, size_t len);
int bthcisock_host_read(int fd, uint8_t *type, void *buf, size_t len);
int bthcisock_host_open(int dev_idx);

#endif /* _ARCH_SIM_SRC_SIM_HCISOCKET_HOST_H_ */
