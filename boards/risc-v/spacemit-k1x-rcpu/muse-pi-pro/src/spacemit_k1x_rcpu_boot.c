/****************************************************************************
 * boards/risc-v/spacemit-k1x-rcpu/muse-pi-pro/src/spacemit_k1x_rcpu_boot.c
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

#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "muse_pi_pro.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spacemit_k1x_rcpu_boardinitialize
 *
 * Description:
 *   Board level initialisation performed before nx_start().
 *
 * Deliberately empty for the initial configuration.  On this board the RCPU
 * has no pin, clock or power resource of its own to claim: the console UART
 * is configured by spacemit_k1x_rcpu_lowsetup() and everything else belongs
 * to the application processor.  When a driver is added that needs the
 * board to configure it first, that belongs here.
 *
 ****************************************************************************/

void spacemit_k1x_rcpu_boardinitialize(void)
{
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   Board level initialisation performed after the OS is up, from the
 *   bring-up thread, so that it may use the driver framework.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  spacemit_k1x_rcpu_bringup();
}
#endif
