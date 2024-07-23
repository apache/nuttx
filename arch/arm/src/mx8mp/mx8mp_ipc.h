/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_ipc.h
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

#ifndef __ARCH_ARM_SRC_MX8MP_MX8MP_IPC_H
#define __ARCH_ARM_SRC_MX8MP_MX8MP_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*ipc_callback_t)(int id, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void mx8mp_ipc_subscribe(int id, ipc_callback_t callback, void *args);
void mx8mp_ipc_signal(int id);
void mx8mp_ipc_init(void);
void mx8mp_ipc_enable(void);

#endif /* __ARCH_ARM_SRC_MX8MP_MX8MP_IPC_H */
