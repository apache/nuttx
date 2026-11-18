/****************************************************************************
 * drivers/video/vl53l5cx/platform.h
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

/* Replaces the ST ULD platform.h included by the downloaded
 * vl53l5cx_buffers.h.
 */

#ifndef __DRIVERS_VIDEO_VL53L5CX_PLATFORM_H
#define __DRIVERS_VIDEO_VL53L5CX_PLATFORM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* One target per zone */

#define VL53L5CX_NB_TARGET_PER_ZONE 1

#endif /* __DRIVERS_VIDEO_VL53L5CX_PLATFORM_H */
