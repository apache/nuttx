/****************************************************************************
 * arch/arm/src/at32/at32_rtc.c
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

#include "chip.h"

/* This file is only a thin shell that includes the correct RTC
 * implementation for the selected AT32 family.  The correct file cannot be
 * selected by the make system because it needs the intelligence that only
 * exists in chip.h that can associate an AT32 part number with an AT32
 * family.
 */

#if defined(CONFIG_AT32_AT32F43XX)
#  include "at32f43xxx_rtcc.c"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
