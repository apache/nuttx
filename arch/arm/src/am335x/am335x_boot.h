/****************************************************************************
 * arch/arm/src/am335x/am335x_boot.h
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

#ifndef __ARCH_ARM_SRC_AM335X_AM335X_BOOT_H
#define __ARCH_ARM_SRC_AM335X_AM335X_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_memory_initialize
 *
 * Description:
 *   All AM335X architectures must provide the following entry point.  This
 *   entry point is called early in the initialization before memory has
 *   been configured.  This board-specific function is responsible for
 *   configuring any on-board memories.
 *
 *   Logic in imx_memory_initialize must be careful to avoid using any
 *   global variables because those will be uninitialized at the time this
 *   function is called.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void am335x_memory_initialize(void);

/****************************************************************************
 * Name: imx_board_initialize
 *
 * Description:
 *   All AM335X architectures must provide the following entry point.  This
 *   entry point is called in the initialization phase -- after
 *   imx_memory_initialize and after all memory has been configured and
 *   mapped but before any devices have been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void am335x_board_initialize(void);

#endif /* __ARCH_ARM_SRC_AM335X_AM335X_BOOT_H */
