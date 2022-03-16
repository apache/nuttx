/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_uid.c
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

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_uid.h"
#include "hardware/cxd5602_topreg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cxd56_get_uniqueid(uint8_t uniqueid[])
{
  uint32_t cuid0 = getreg32(CXD56_TOPREG_CUID0);
  uint32_t cuid1 = getreg32(CXD56_TOPREG_CUID1);

  uniqueid[0] = (uint8_t)((cuid1 >>  0) & 0xff);
  uniqueid[1] = (uint8_t)((cuid0 >> 24) & 0xff);
  uniqueid[2] = (uint8_t)((cuid0 >> 16) & 0xff);
  uniqueid[3] = (uint8_t)((cuid0 >>  8) & 0xff);
  uniqueid[4] = (uint8_t)((cuid0 >>  0) & 0xff);
}
