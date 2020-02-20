/****************************************************************************
 * arch/arm/src/samv7/sam_uid.c
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

#include "sam_eefc.h"
#include "sam_uid.h"

#ifdef CONFIG_BOARDCTL_UNIQUEID

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_get_uniqueid
 *
 * Description:
 *   Get unique id of device.
 *
 * Input Parameters:
 *   uniqueid  - The buffer to store unique id.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/

void sam_get_uniqueid(uint8_t uniqueid[16])
{
  uint32_t buffer[4];
  uint8_t  index;

  /* Set flash access mode to 128bit and wait status to 4 */

  sam_eefc_initaccess(SAM_EFC_ACCESS_MODE_128, 4);

  /* Get device unique id */

  sam_eefc_readsequence(FCMD_STUI, FCMD_SPUI, buffer, 4);

  for (index = 0; index < 4; index++)
    {
      uniqueid[4*index + 3] = (uint8_t)buffer[index];
      uniqueid[4*index + 2] = (uint8_t)(buffer[index] >> 8);
      uniqueid[4*index + 1] = (uint8_t)(buffer[index] >> 16);
      uniqueid[4*index + 0] = (uint8_t)(buffer[index] >> 24);
    }
}

#endif /* CONFIG_BOARDCTL_UNIQUEID */
