/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_uid.c
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "hardware/esp32c3_efuse.h"
#include "esp32c3.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_get_uniqueid
 *
 * Description:
 *   Get CPU unique ID.
 *
 * Parameters:
 *   uniqueid - unique ID buffer
 *
 ****************************************************************************/

void esp32c3_get_uniqueid(uint8_t *uniqueid)
{
  int i;

  for (i = 0; i < 16; i++)
    {
      uniqueid[i] = getreg8(EFUSE_RD_SYS_DATA_PART1_0_REG + i);
    }
}
