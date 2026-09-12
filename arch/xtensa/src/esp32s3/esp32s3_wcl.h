/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_wcl.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WCL_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WCL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The ESP32-S3 World Controller provides a binary privileged (WORLD_0,
 * kernel) / non-privileged (WORLD_1, user) split.  The world identifier is
 * shared vocabulary between the World Controller and the PMS permission
 * subsystem, so it is defined here at the lowest layer.
 */

enum esp32s3_pms_world_e
{
  PMS_WORLD_0 = 0,
  PMS_WORLD_1
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_wcl_set_vecbase
 *
 * Description:
 *   Override the Vector Table base address for a given world via the World
 *   Controller.
 *
 * Input Parameters:
 *   world   - World to which the vector table base address will apply.
 *   vecbase - Vector table base address to set.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_wcl_set_vecbase(enum esp32s3_pms_world_e world,
                             uintptr_t vecbase);

/****************************************************************************
 * Name: esp32s3_wcl_set_world0_entry
 *
 * Description:
 *   Configure the World Controller to switch to World 0 whenever the CPU
 *   performs an instruction fetch from a given address.
 *
 * Input Parameters:
 *   entry - Entry number.  Up to 13 entry addresses are supported.  Entry 0
 *           is reserved and must be skipped.
 *   addr  - Vector fetch address that triggers the switch to World 0.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_wcl_set_world0_entry(uint32_t entry, uintptr_t addr);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WCL_H */
