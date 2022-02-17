/****************************************************************************
 * drivers/segger/config/Global.h
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

#ifndef __DRIVERS_SEGGER_CONFIG_GLOBAL_H
#define __DRIVERS_SEGGER_CONFIG_GLOBAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define U8          uint8_t
#define I8           int8_t
#define U16        uint16_t
#define I16         int16_t
#define U32        uint32_t
#define I32         int32_t
#define U64        uint64_t
#define I64         int64_t
#define PTR_ADDR  uintptr_t

#endif /* __DRIVERS_SEGGER_CONFIG_GLOBAL_H */
