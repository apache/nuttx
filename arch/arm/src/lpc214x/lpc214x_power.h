/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_power.h
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

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_POWER_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register address definitions *********************************************/

#define LPC214X_PCON_PCON     (0xe01fc0c0)  /* Power control register */
#define LPC214X_PCON_PCONP    (0xe01fc0c4)  /* Power controls for peripherals register */

/* Register bit definitions *************************************************/

/* Power control register */

#define LPC214X_PCON_IDL      (0x01)        /* Bit 0=1: Idle mode ON */
#define LPC214X_PCON_PD       (0x02)        /* Bit 1=1: Power down mode ON */
#define LPC214X_PCON_BODPDM   (0x04)        /* Bit 2=1: Brown out power down mode ON */
#define LPC214X_PCON_BOGD     (0x08)        /* Bit 3=1: Brown out global disable */
#define LPC214X_PCON_BORD     (0x10)        /* Bit 4=1: Brown out reset disable */

/* Peripheral power control register */

#define LPC214X_PCONP_PCTIM0  (0x00000002)  /* Bit 1=1: Timer/counter0 control */
#define LPC214X_PCONP_PCTIM1  (0x00000004)  /* Bit 2=1: Timer/counter1 control */
#define LPC214X_PCONP_PCUART0 (0x00000008)  /* Bit 3=1: UART0 control */
#define LPC214X_PCONP_PCUART1 (0x00000010)  /* Bit 4=1: UART1 control */
#define LPC214X_PCONP_PCWM0   (0x00000020)  /* Bit 5=1: PWM0 control */
#define LPC214X_PCONP_PCI2C0  (0x00000080)  /* Bit 7=1: I2C0 control */
#define LPC214X_PCONP_PCSPI0  (0x00000100)  /* Bit 8=1: SPI0 control */
#define LPC214X_PCONP_PCRTC   (0x00000200)  /* Bit 9=1: RTCcontrol */
#define LPC214X_PCONP_PCSPI1  (0x00000400)  /* Bit 10=1: SPI1 control */
#define LPC214X_PCONP_PCAD0   (0x00001000)  /* Bit 12=1: A/C converter 0 control */
#define LPC214X_PCONP_PCI2C1  (0x00080000)  /* Bit 19=1: I2C1 control */
#define LPC214X_PCONP_PCAD1   (0x00100000)  /* Bit 20=1: A/C converter 1 control */
#define LPC214X_PCONP_PCUSB   (0x80000000)  /* Bit 31=1: USB power/clock control */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_POWER_H */
