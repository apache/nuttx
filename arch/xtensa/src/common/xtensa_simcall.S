/****************************************************************************
 * arch/xtensa/src/common/xtensa_simcall.S
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

#include <arch/xtensa/xtensa_abi.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: simcall
 *
 * Description:
 *   Execute a SIMCALL instruction
 *
 * Entry Conditions:
 *   A2     - SIMCALL number
 *   A3..A5 - SIMCALL arguments
 *   A6     - the pointer to store errno
 *
 ****************************************************************************/

    .global simcall
    .type   simcall, @function

simcall:
    ENTRY0

    simcall

    /* A2 - return value
     * A3 - errno
     */

    s32i	a3, a6, 0 /* store errno */

    RET0
