/****************************************************************************
 * arch/arm/src/kinetis/kinetis_uid.c
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

#include "hardware/kinetis_k64memorymap.h"
#include "hardware/kinetis_sim.h"
#include "kinetis_uid.h"

#ifdef CONFIG_BOARDCTL_UNIQUEID

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_get_uniqueid
 ****************************************************************************/

void kinetis_get_uniqueid(uint8_t *uniqueid)
{
  uint32_t *unique_u32;
  int i;

  unique_u32 = (uint32_t *)uniqueid;

  /* Copy into buffer LS first, which in the Kinetis is the highest memory */

  for (i = 0; i < (KINETIS_UID_SIZE / sizeof(uint32_t)); i++)
    {
      unique_u32[i] = *((uint32_t *)(KINETIS_SIM_UIDL) - i);
    }
}

#endif /* CONFIG_BOARDCTL_UNIQUEID */
