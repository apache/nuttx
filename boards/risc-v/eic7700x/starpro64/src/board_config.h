/****************************************************************************
 * boards/risc-v/eic7700x/starpro64/src/board_config.h
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

#ifndef __BOARDS_RISCV_EIC7700X_STARPRO64_SRC_BOARD_CONFIG_H
#define __BOARDS_RISCV_EIC7700X_STARPRO64_SRC_BOARD_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_bringup
 *
 * Description:
 *   Bring up this board's devices, in this board's order.  Called from the
 *   common layer's board_late_initialize().
 *
 ****************************************************************************/

int eic7700x_bringup(void);

#endif /* __BOARDS_RISCV_EIC7700X_STARPRO64_SRC_BOARD_CONFIG_H */
