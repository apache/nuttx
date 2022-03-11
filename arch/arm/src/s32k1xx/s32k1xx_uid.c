/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_uid.c
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

#include <stddef.h>
#include <stdint.h>
#include <assert.h>

#include "arm_internal.h"
#include "hardware/s32k1xx_sim.h"
#include "s32k1xx_uid.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_get_uniqueid
 ****************************************************************************/

void s32k1xx_get_uniqueid(uint8_t *uniqueid)
{
  uint32_t regval;

  DEBUGASSERT(uniqueid != NULL);

  /* Retrieve all 16 bytes (128 bits) of the unique identifier and store the
   * individual bytes in order from lowest byte first to highest byte last.
   */

  regval = getreg32(S32K1XX_SIM_UIDL);

  uniqueid[0]  = (uint8_t)((regval & 0x000000ff) >>  0);
  uniqueid[1]  = (uint8_t)((regval & 0x0000ff00) >>  8);
  uniqueid[2]  = (uint8_t)((regval & 0x00ff0000) >> 16);
  uniqueid[3]  = (uint8_t)((regval & 0xff000000) >> 24);

  regval = getreg32(S32K1XX_SIM_UIDML);

  uniqueid[4]  = (uint8_t)((regval & 0x000000ff) >>  0);
  uniqueid[5]  = (uint8_t)((regval & 0x0000ff00) >>  8);
  uniqueid[6]  = (uint8_t)((regval & 0x00ff0000) >> 16);
  uniqueid[7]  = (uint8_t)((regval & 0xff000000) >> 24);

  regval = getreg32(S32K1XX_SIM_UIDMH);

  uniqueid[8]  = (uint8_t)((regval & 0x000000ff) >>  0);
  uniqueid[9]  = (uint8_t)((regval & 0x0000ff00) >>  8);
  uniqueid[10] = (uint8_t)((regval & 0x00ff0000) >> 16);
  uniqueid[11] = (uint8_t)((regval & 0xff000000) >> 24);

  regval = getreg32(S32K1XX_SIM_UIDH);

  uniqueid[12] = (uint8_t)((regval & 0x000000ff) >>  0);
  uniqueid[13] = (uint8_t)((regval & 0x0000ff00) >>  8);
  uniqueid[14] = (uint8_t)((regval & 0x00ff0000) >> 16);
  uniqueid[15] = (uint8_t)((regval & 0xff000000) >> 24);
}
