/****************************************************************************
 * boards/renesas/m16c/skp16c26/src/m16c_buttons.c
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

#include <nuttx/board.h>

#include "chip.h"
#include "renesas_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SKP62C26 has 3 buttons control by bits 1, 2, and 3 in port 8. */

#define SW1_BIT             (1 << 3)   /* Bit 3, port 8 */
#define SW2_BIT             (1 << 2)   /* Bit 2, port 8 */
#define SW3_BIT             (1 << 1)   /* Bit 1, port 8 */

#define SW_PRESSED(p,b)     (((p) & (b)) == 0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
uint32_t board_button_initialize(void)
{
  uint8_t regval;

  regval  = getreg8(M16C_PD8);
  regval |= (SW1_BIT | SW2_BIT | SW3_BIT);
  putreg8(regval, M16C_PD8);

  return 3;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t swset  = 0;
  uint8_t regval = getreg8(M16C_P8);

  if (SW_PRESSED(regval, SW1_BIT))
    {
      swset |= SW1_PRESSED;
    }

  if (SW_PRESSED(regval, SW2_BIT))
    {
      swset |= SW2_PRESSED;
    }

  if (SW_PRESSED(regval, SW3_BIT))
    {
      swset |= SW3_PRESSED;
    }

  return swset;
}
#endif /* CONFIG_ARCH_BUTTONS */
