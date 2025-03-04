/****************************************************************************
 * arch/arm/src/common/ghs/lib_dummy.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* The following functions is needed by libmath that introduced by default
 * in the Greenhills toolchain, in nuttx we do not really need these
 * function, so add dummy implementation here to avoid the link error
 */

void __gh_long_long_printf(void)
{
}

void __gh_float_printf(void)
{
}

void __gh_fputs_stdout(void)
{
}

void __gh_set_errno(int errno)
{
}
