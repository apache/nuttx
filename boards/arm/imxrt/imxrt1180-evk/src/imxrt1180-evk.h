/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt1180-evk.h
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

#ifndef __BOARDS_ARM_IMXRT_IMXRT1180_EVK_SRC_IMXRT1180_EVK_H
#define __BOARDS_ARM_IMXRT_IMXRT1180_EVK_SRC_IMXRT1180_EVK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int imxrt_bringup(void);

#ifdef CONFIG_USBDEV_DMAMEMORY
int imxrt_dma_alloc_init(void);
#endif

#endif /* __BOARDS_ARM_IMXRT_IMXRT1180_EVK_SRC_IMXRT1180_EVK_H */
