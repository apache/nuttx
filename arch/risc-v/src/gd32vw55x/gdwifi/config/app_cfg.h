/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/config/app_cfg.h
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

/* app_cfg.h - application feature knobs for the GD32VW55x SDK sources
 * compiled into NuttX (replaces MSDK/app/app_cfg.h).  STA bring-up:
 * debug prints + fast reconnect + SNTP; AT/base command shells off.
 */

#ifndef _APP_CFG_H_
#define _APP_CFG_H_

#include "platform_def.h"

/* Same knobs as the vendor demo (app/inc/app_cfg.h) */
#define CONFIG_DEBUG_PRINT_ENABLE
#define CONFIG_SNTP

#endif /* _APP_CFG_H_ */
