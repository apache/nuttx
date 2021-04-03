/****************************************************************************
 * arch/arm/src/stm32/stm32_flash.c
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

/* Provides standard flash access functions, to be used by the  flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the correct FLASH implementation for the selection STM32 part */

#if defined(CONFIG_STM32_STM32L15XX)
#  include "stm32l15xx_flash.c"
#elif defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#  include "stm32f10xxf30xx_flash.c"
#elif defined(CONFIG_STM32_STM32F20XX) || defined (CONFIG_STM32_STM32F4XXX)
#  include "stm32f20xxf40xx_flash.c"
#else
#  warning "No FLASH support for the selected part"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
