/****************************************************************************
 * boards/arm/stm32l0/nucleo-l073rz/src/stm32_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/usb/usbdev.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/stm32_usbdev.h"
#include "stm32_usbdev.h"
#include "nucleo-l073rz.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#ifdef CONFIG_STM32_SPI
  /* Configure SPI chip selects */

  stm32_spidev_initialize();
#endif
}

#if defined(CONFIG_STM32_USB) && defined(CONFIG_USBDEV)
int stm32_usb_setpullup(bool enable)
{
  irqstate_t flags;
  uint16_t regval;

  flags = enter_critical_section();
  regval = getreg16(STM32_USB_BCDR);

  if (enable)
    {
      regval |= USB_BCDR_DPPU;
    }
  else
    {
      regval &= ~USB_BCDR_DPPU;
    }

  putreg16(regval, STM32_USB_BCDR);
  leave_critical_section(flags);

  return OK;
}

bool stm32_usb_pullup_enabled(void)
{
  return (getreg16(STM32_USB_BCDR) & USB_BCDR_DPPU) != 0;
}

int stm32_usbpullup(FAR struct usbdev_s *dev, bool enable)
{
  return stm32_usb_setpullup(enable);
}

void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
}
#endif

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  stm32_bringup();
}
#endif
