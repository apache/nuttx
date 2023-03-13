/****************************************************************************
 * arch/arm/src/nrf53/nrf53_uid.c
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

#include "arm_internal.h"
#include "nrf53_uid.h"

#include "hardware/nrf53_ficr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nrf53_get_uniqueid(uint8_t uniqueid[])
{
  uint32_t uid0 = getreg32(NRF53_FICR_BASE +
                           NRF53_FICR_INFO_DEVICEID0_OFFSET);
  uint32_t uid1 = getreg32(NRF53_FICR_BASE +
                           NRF53_FICR_INFO_DEVICEID1_OFFSET);

  uniqueid[0] = (uint8_t)((uid0 >> 0) & 0xff);
  uniqueid[1] = (uint8_t)((uid0 >> 8) & 0xff);
  uniqueid[2] = (uint8_t)((uid0 >> 16) & 0xff);
  uniqueid[3] = (uint8_t)((uid0 >> 24) & 0xff);
  uniqueid[4] = (uint8_t)((uid1 >> 0) & 0xff);
  uniqueid[5] = (uint8_t)((uid1 >> 8) & 0xff);
  uniqueid[6] = (uint8_t)((uid1 >> 16) & 0xff);
  uniqueid[7] = (uint8_t)((uid1 >> 24) & 0xff);
}
