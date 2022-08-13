/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_STM32WB_FLASH_H
#define __ARCH_ARM_SRC_STM32WB_STM32WB_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32wb_flash.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int stm32wb_flash_lock(void);
int stm32wb_flash_unlock(void);

/****************************************************************************
 * Name: stm32wb_flash_user_optbytes
 *
 * Description:
 *   Modify the contents of the user option bytes (USR OPT) on the flash.
 *   This does not set OBL_LAUNCH so new options take effect only after
 *   next power reset.
 *
 * Input Parameters:
 *   clrbits - Bits in the option bytes to be cleared
 *   setbits - Bits in the option bytes to be set
 *
 * Returned Value:
 *   Option bytes after operation is completed
 *
 ****************************************************************************/

uint32_t stm32wb_flash_user_optbytes(uint32_t clrbits, uint32_t setbits);

#endif /* __ARCH_ARM_SRC_STM32WB_STM32WB_FLASH_H */
