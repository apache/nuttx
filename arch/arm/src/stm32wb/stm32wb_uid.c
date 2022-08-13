/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_uid.c
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

#include "hardware/stm32wb_memorymap.h"
#include "stm32wb_uid.h"

#ifdef STM32WB_SYSMEM_UID

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32wb_get_uniqueid(uint8_t uniqueid[12])
{
  int i;

  for (i = 0; i < 12; i++)
    {
      uniqueid[i] = *((uint8_t *)(STM32WB_SYSMEM_UID) + i);
    }
}

#endif /* STM32WB_SYSMEM_UID */
