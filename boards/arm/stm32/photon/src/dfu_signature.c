/****************************************************************************
 * boards/arm/stm32/photon/src/dfu_signature.c
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

#include <stdint.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct dfu_signature
{
  uint32_t linker_start_address;
  uint32_t linker_end_address;
  uint8_t  reserved[4];
  uint16_t board_id;
  uint8_t  firmware_type1;
  uint8_t  firmware_type2;
  uint8_t  reserved2[8];
} end_packed_struct;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t _firmware_start[];
extern uint8_t _firmware_end[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

__attribute__((externally_visible)) locate_data(".dfu_signature")
const struct dfu_signature dfu_sign =
{
  (uint32_t)_firmware_start, /* Flash image start address */
  (uint32_t)_firmware_end,   /* Flash image end address */
  {0, 0, 0, 0},              /* reserved */
  6,                         /* Current board is photon */
  4, 1,                      /* Firmware is "system-part1" */
  {0, 0, 0, 0, 0, 0, 0, 0},  /* reserved */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
