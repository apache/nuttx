/****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570ls31x_usb_kit.h
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

#ifndef __BOARDS_ARM_TMS570_TMS570LS31X_USB_KIT_SRC_TMS570LS31X_USB_KIT_H
#define __BOARDS_ARM_TMS570_TMS570LS31X_USB_KIT_SRC_TMS570LS31X_USB_KIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs
 *
 * The launchpad has several LEDs:
 *
 *   - LEd D1 (white) that connects to the USB +5V supply,
 *   - LED D10 (red) that connects to the TMS570's NERROR pin,
 *   - D5 (blue), D6 (blue), and D8 (blue) connect to the XDS100 FT2322,
 *   - D7 (blue) connects to the XSD100 CPLD, and
 *   - Two white, user LEDs labeled D12 that connects to the NHET08
 *     pin and D11 that connects to GIOA2.
 *
 * NHET08 is one of 32 N2HET pins than can be available to the user if not
 * used by N2HET.  This implementation, however, uses only the single LED
 * driven by GIOA2.  That LED is tied to ground and illuminated with a high
 * level output value.
 */

#define GIO_LED_D11   (GIO_OUTPUT | GIO_CFG_DEFAULT | GIO_OUTPUT_CLEAR | \
                       GIO_PORT_GIOA | GIO_PIN1)

/* Buttons
 *
 * The launchpad has three mechanical buttons. Two of these are reset
 * buttons:  One button is labeled PORRST performs a power-on reset and one
 * labeled RST performs an MCU reset.  Only one button is available for
 * general software usage.  That button is labeled GIOA7 and is, obviously,
 * sensed on GIOA7.
 *
 * GIOA7 is tied to ground, but will be pulled high if the GIOA7 button is
 * depressed.
 */

#define GIO_BUTTON    (GIO_INPUT | GIO_CFG_PULLUP | GIO_INT_BOTHEDGES | \
                       GIO_PORT_GIOA | GIO_PIN7)
#define IRQ_BUTTON     TMS570_IRQ_GIOA7

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int tms570_bringup(void);

/****************************************************************************
 * Name: tms570_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int tms570_mmcsd_initialize(int minor);
#endif

#endif /* __BOARDS_ARM_TMS570_TMS570LS31X_USB_KIT_SRC_TMS570LS31X_USB_KIT_H */
