/****************************************************************************
 * boards/arm/str71x/olimex-strp711/src/str71_buttons.c
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
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Olimex board has two buttons, one labeled "BUT" and the other "WAKEUP"
 *
 * P0.15: WAKEUP button
 * P1.13: BUT button
 */

#define STR71X_BUTBUTTON_GPIO1    (0x2000)
#define STR71X_WAKEUPBUTTON_GPIO0 (0x8000)

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
  uint16_t reg16;

  /* Configure the GPIO0 & 1 pins as inputs */

  reg16  = getreg16(STR71X_GPIO0_PC0);
  reg16 |= STR71X_WAKEUPBUTTON_GPIO0;
  putreg16(reg16, STR71X_GPIO0_PC0);

  reg16  = getreg16(STR71X_GPIO0_PC1);
  reg16 &= ~STR71X_WAKEUPBUTTON_GPIO0;
  putreg16(reg16, STR71X_GPIO0_PC1);

  reg16  = getreg16(STR71X_GPIO0_PC2);
  reg16 &= ~STR71X_WAKEUPBUTTON_GPIO0;
  putreg16(reg16, STR71X_GPIO0_PC2);

  reg16  = getreg16(STR71X_GPIO1_PC0);
  reg16 |= STR71X_BUTBUTTON_GPIO1;
  putreg16(reg16, STR71X_GPIO1_PC0);

  reg16  = getreg16(STR71X_GPIO1_PC1);
  reg16 &= ~STR71X_BUTBUTTON_GPIO1;
  putreg16(reg16, STR71X_GPIO1_PC1);

  reg16  = getreg16(STR71X_GPIO1_PC2);
  reg16 &= ~STR71X_BUTBUTTON_GPIO1;
  putreg16(reg16, STR71X_GPIO1_PC2);

  return 2;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  if ((getreg16(STR71X_GPIO0_PD) & STR71X_WAKEUPBUTTON_GPIO0) != 0)
    {
      ret |= WAKEUP_BUTTON;
    }

  if ((getreg16(STR71X_GPIO1_PD) & STR71X_BUTBUTTON_GPIO1) != 0)
    {
      ret |= BUT_BUTTON;
    }

  return ret;
}
#endif /* CONFIG_ARCH_BUTTONS */
