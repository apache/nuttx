/****************************************************************************
 * arch/risc-v/src/common/espressif/platform_include/stdatomic.h
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

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include_next <stdatomic.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ATOMIC_VAR_INIT was deprecated in C17 and removed in C23 / GCC 15.
 * Provide compatibility definition for external HAL components.
 */

#ifndef ATOMIC_VAR_INIT
#  define ATOMIC_VAR_INIT(value) (value)
#endif
