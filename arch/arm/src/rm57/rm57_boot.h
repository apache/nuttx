/****************************************************************************
 * arch/arm/src/rm57/rm57_boot.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_BOOT_H
#define __ARCH_ARM_SRC_RM57_RM57_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_boot_c
 *
 * Description:
 *   The C part of the boot sequence.  Entered by a branch from arm_boot
 *   (rm57_coreinit.S) once every CPU register holds a defined value, and
 *   does not return.
 *
 ****************************************************************************/

void rm57_boot_c(void);

/****************************************************************************
 * Name: rm57_board_initialize
 *
 * Description:
 *   All RM57 boards must provide the following entry point.  This
 *   function is called from arm_boot() after clocking, memory, and MPU
 *   have been configured but before any devices have been initialized.
 *
 ****************************************************************************/

void rm57_board_initialize(void);

#endif /* __ARCH_ARM_SRC_RM57_RM57_BOOT_H */
