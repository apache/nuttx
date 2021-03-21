/****************************************************************************
 * boards/mips/pic32mx/mirtoo/src/pic32_boot.c
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

#include <debug.h>

#include <arch/board/board.h>

#include "mips_arch.h"
#include "mips_internal.h"

#include "pic32mx.h"
#include "pic32mx_pps.h"
#include "mirtoo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_U1TX  (GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN5)
#define GPIO_U1RX  (GPIO_INPUT|GPIO_PORTC|GPIO_PIN6)

#define GPIO_U2TX  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN10)
#define GPIO_U2RX  (GPIO_INPUT|GPIO_PORTB|GPIO_PIN11)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_uartinitialize
 *
 * Description:
 *   When mounted on the DTX1-4000L EV-kit1 board, serial output is available
 *   through an FT230X device via the FUNC0 and FUNC1 module outputs.
 *   If CONFIG_PIC32MX_UART2 is enabled, the src/pic32_boot will configure
 *   the UART2 pins as follows.
 *
 *   ---------- ------ ----- ------ -------------------------
 *      BOARD   OUTPUT  PIN  SIGNAL NOTES
 *   ---------- ------ ----- ------ -------------------------
 *   FT230X RXD  FUNC0 RPB11  U2RX  UART2 RX (Also PGEC2)
 *   FT230X TXD  FUNC1 RPB10  U2TX  UART2 TX (Also PGED2)
 *
 *   If CONFIG_PIC32MX_UART1 is enabled,
 *   the src/pic32_boot will configure the UART pins as follows.
 *   This will support communictions (via an external RS-232
 *   driver) through X3 pins 4 and 5:
 *
 *   ---------- ------ ----- ------ -------------------------
 *      BOARD   MODULE  PIN  SIGNAL NOTES
 *   ---------- ------ ----- ------ -------------------------
 *   X3, pin 4   FUNC4 RPBC5  U1TX  UART1 TX
 *   X3, pin 5   FUNC5 RPBC6  U1RX  UART1 RX
 *
 ****************************************************************************/

static inline void pic32mx_uartinitialize(void)
{
#ifdef CONFIG_PIC32MX_UART2

  /* Make sure that TRIS pins are set correctly.
   * Configure the UART pins as digital inputs and outputs first.
   */

  pic32mx_configgpio(GPIO_U2TX);
  pic32mx_configgpio(GPIO_U2RX);

  /* Configure UART TX and RX pins to RPB10 and 11, respectively */

  putreg32(PPS_INSEL_RPB11,  PIC32MX_PPS_U2RXR);
  putreg32(PPS_OUTSEL_U2TX, PIC32MX_PPS_RPB10R);
#endif

#ifdef CONFIG_PIC32MX_UART1

  /* Make sure that TRIS pins are set correctly.
   * Configure the UART pins as digital inputs and outputs first.
   */

  pic32mx_configgpio(GPIO_U1TX);
  pic32mx_configgpio(GPIO_U1RX);

  /* Configure UART TX and RX pins to RPB10 and 11, respectively */

  putreg32(PPS_INSEL_RPC6,  PIC32MX_PPS_U1RXR);
  putreg32(PPS_OUTSEL_U1TX, PIC32MX_PPS_RPC5R);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_boardinitialize
 *
 * Description:
 *   All PIC32MX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void pic32mx_boardinitialize(void)
{
  /* Configure the console UART */

  pic32mx_uartinitialize();

  /* Configure SPI chip selects if 1) at least one SPI is enabled, and 2)
   * the weak function pic32mx_spi2initialize() has been brought into the
   * link.
   */

#ifdef CONFIG_PIC32MX_SPI2
  if (pic32mx_spi2initialize)
    {
      pic32mx_spi2initialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  pic32mx_led_initialize();
#endif
}
