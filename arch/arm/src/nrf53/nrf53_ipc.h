/****************************************************************************
 * arch/arm/src/nrf53/nrf53_ipc.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_IPC_H
#define __ARCH_ARM_SRC_NRF53_NRF53_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/nrf53_ipc.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*ipc_callback_t)(int id, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void nrf53_ipc_subscribe(int id, ipc_callback_t callback, void *args);
void nrf53_ipc_send_cfg(int id);
void nrf53_ipc_signal(int id);
void nrf53_ipc_init(void);

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_IPC_H */
