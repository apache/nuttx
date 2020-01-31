/****************************************************************************
 * boards/arm/tiva/launchxl-cc1310/src/cc1310_userleds.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/tiva_ioc.h"
#include "tiva_gpio.h"
#include "launchxl-cc1310.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_TIVA_UART0
/* UART0:
 *
 * The on-board XDS110 Debugger provide a USB virtual serial console using
 * UART0 (PA0/U0RX and PA1/U0TX).
 */

const struct cc13xx_pinconfig_s g_gpio_uart0_rx =
{
  .gpio = GPIO_DIO(0),
  .ioc  = IOC_IOCFG_PORTID(IOC_IOCFG_PORTID_UART0_RX) | IOC_STD_INPUT
};

const struct cc13xx_pinconfig_s g_gpio_uart0_tx =
{
  .gpio = GPIO_DIO(1),
  .ioc  = IOC_IOCFG_PORTID(IOC_IOCFG_PORTID_UART0_TX) | IOC_STD_OUTPUT
};
#endif
/* The LaunchXL-cc1310 and two LEDs controlled by software: DIO7_GLED (CR1)
 * and DIO6_RLED (CR2).  A high output value illuminates an LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

const struct cc13xx_pinconfig_s g_gpio_gled =
{
  .gpio = GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_DIO(7),
  .ioc  = IOC_IOCFG_PORTID(IOC_IOCFG_PORTID_GPIO) | IOC_STD_INPUT
};

const struct cc13xx_pinconfig_s g_gpio_rled =
{
  .gpio = GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_DIO(6),
  .ioc  = IOC_IOCFG_PORTID(IOC_IOCFG_PORTID_GPIO) | IOC_STD_INPUT
};

#ifdef CONFIG_ARCH_BUTTONS
/* The LaunchXL-CC1310 has two push-puttons:
 *
 *   DIO13_BTN1  SW1  Low input sensed when depressed
 *   DIO14_BTN2  SW2  Low input sensed when depressed
 */

#ifdef CONFIG_ARCH_IRQBUTTONS
/* Like IOC_STD_OUTPUT but with MCU wake-up enable and interrupt edge
 * detection enabled on both edges.
 */

#define IOC_CC1310_BUTTON_OUTPUT  (IOC_IOCFG_IOEV_MCU_WUEN | \
                                   IOC_IOCFG_IOSTR_AUTO | \
                                   IOC_IOCFG_IOCURR_2MA | \
                                   IOC_IOCFG_PULLCTL_DIS | \
                                   IOC_IOCFG_EDGEDET_BOTH | \
                                   IOC_IOCFG_EDGE_IRQEN | \
                                   IOC_IOCFG_IOMODE_NORMAL | \
                                   IOC_IOCFG_WUCFG_WAKEUPL | \
                                   IOC_IOCFG_IE)

#else
/* Like IOC_STD_OUTPUT but with MCU wake-up enable. */

#define  IOC_CC1310_BUTTON_OUTPUT (IOC_IOCFG_IOEV_MCU_WUEN | \
                                   IOC_IOCFG_IOSTR_AUTO | \
                                   IOC_IOCFG_IOCURR_2MA | \
                                   IOC_IOCFG_PULLCTL_DIS | \
                                   IOC_IOCFG_EDGEDET_NONE | \
                                   IOC_IOCFG_IOMODE_NORMAL | \
                                   IOC_IOCFG_WUCFG_WAKEUPL | \
                                   IOC_IOCFG_IE)
#endif /* CONFIG_ARCH_IRQBUTTONS */

const struct cc13xx_pinconfig_s g_gpio_sw1 =
{
  .gpio = GPIO_DIO(14),
  .ioc  = IOC_IOCFG_PORTID(IOC_IOCFG_PORTID_GPIO) | IOC_CC1310_BUTTON_OUTPUT
};

const struct cc13xx_pinconfig_s g_gpio_sw2 =
{
  .gpio = GPIO_DIO(15),
  .ioc  = IOC_IOCFG_PORTID(IOC_IOCFG_PORTID_GPIO) | IOC_CC1310_BUTTON_OUTPUT
};
#endif /* CONFIG_ARCH_BUTTONS */
