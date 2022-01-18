/****************************************************************************
 * boards/arm/tiva/ekk-lm3s9b96/src/ekk-lm3s9b96.h
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

#ifndef __BOARDS_ARM_TIVA_EKK_LM3S9B96_SRC_EKK_LM3S9B96_H
#define __BOARDS_ARM_TIVA_EKK_LM3S9B96_SRC_EKK_LM3S9B96_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "chip.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many SSI modules does this chip support? The LM3S9B96 supports 2 SSI
 * modules (others may support more than 2 -- in such case, the following
 * must be expanded).
 */

#if TIVA_NSSI == 0
#  undef CONFIG_TIVA_SSI0
#  undef CONFIG_TIVA_SSI1
#elif TIVA_NSSI == 1
#  undef CONFIG_TIVA_SSI1
#endif

/* EKK-LM3S9B96 Eval Kit ****************************************************/

/* GPIO Usage
 *
 * PIN SIGNAL      EVB Function
 * --- ----------- ---------------------------------------
 *  26 PA0/U0RX      Virtual COM port receive
 *  27 PA1/U0TX      Virtual COM port transmit
 *  66 PB0/USB0ID    USBID signal from the USB-On-the-Go
 *  67 PB1/USB0VBUS  USB VBUS input signal from USB-OTG
 *  92 PB4/GPIO      User pushbutton SW2.
 *  80 PC0/TCK/SWCLK JTAG or SWD clock input
 *  79 PC1/TMS/SWDIO JTAG TMS input or SWD bidirectional signal SWDIO
 *  78 PC2/TDI       JTAG TDI signal input
 *  77 PC3/TDO/SWO   JTAG TDO output or SWD trace signal SWO output.
 *  10 PD0/GPIO      User LED
 *  60 PF2/LED1      Ethernet LED1 (yellow)
 *  59 PF3/LED0      Ethernet LED0 (green)
 *  83 PH3/USB0EPEN  USB-OTG power switch
 *  76 PH4/USB0PFLT  Overcurrent input status from USB-OTG power switch
 */

/* GPIO for LED's:
 * - PD0: User LED
 */

#define LED_GPIO          (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTD | 0)

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: lm_ssidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LM3S6965 Eval Kit.
 *
 ****************************************************************************/

extern void weak_function lm_ssidev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_TIVA_EKK_LM3S9B96_SRC_EKK_LM3S9B96_H */
