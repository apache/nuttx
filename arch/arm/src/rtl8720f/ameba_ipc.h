/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_ipc.h
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

#ifndef __ARCH_ARM_SRC_RTL8720F_AMEBA_IPC_H
#define __ARCH_ARM_SRC_RTL8720F_AMEBA_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_ipc_initialize
 *
 * Description:
 *   Bring up the km4tz<->km4ns IPC transport once (idempotent).  Required by
 *   the SDK flash erase/program path (inter-core pause) and by WiFi.  Call
 *   after the scheduler is running and before the flash filesystem mounts.
 *
 ****************************************************************************/

void ameba_ipc_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RTL8720F_AMEBA_IPC_H */
