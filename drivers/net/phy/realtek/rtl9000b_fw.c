/****************************************************************************
 * drivers/net/phy/realtek/rtl9000b_fw.c
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

#include "rtl9000b_fw.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Parameter verification tables: exact copies from vendor reference
 * (RTL9000Bx_Initial_Configuration_Check).
 * Each triplet is {page, register, expected_value}.
 */

const uint16_t g_rtl9000b_param_check[27] =
{
  0,      0x8017, 0xfb03, 0,      0x8092, 0x6000,
  0,      0x80af, 0x6000, 0,      0x807d, 0x4443,
  0,      0x809a, 0x4443, 0,      0x81a3, 0x0f00,
  0x0a81, 0x12,   0x0004, 0,      0x83c8, 0x0005,
  0x0a5c, 0x12,   0x0003
};

const uint16_t g_rtl9000b_param_check2[5] =
{
  0x401c, 0xa610, 0x8520, 0xa510, 0x21f4
};

const uint16_t g_rtl9000b_param_check3[3] =
{
  0, 0xa000, 0x11ef
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* This file only provides the RTL9000B config verification parameter
 * tables; there are no functions defined here.
 */
