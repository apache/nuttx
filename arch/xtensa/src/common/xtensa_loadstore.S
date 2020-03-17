/****************************************************************************
 * arch/xtensa/src/common/xtensa_loadstore.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "xtensa_abi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: l32i
 *
 * Description:
 *   Execute a L32I instruction
 *
 * Entry Conditions:
 *   A2     - a pointer
 *
 ****************************************************************************/

    .global l32i
    .type   l32i, @function
    .align  4

l32i:
    ENTRY0

    l32i	a2, a2, 0

    RET0

/****************************************************************************
 * Name: s32i
 *
 * Description:
 *   Execute a S32I instruction
 *
 * Entry Conditions:
 *   A2     - a pointer
 *   A3     - a value to store
 *
 ****************************************************************************/

    .global s32i
    .type   s32i, @function
    .align  4

s32i:
    ENTRY0

    s32i	a3, a2, 0

    RET0
