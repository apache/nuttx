/********************************************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_usbhs.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Ramtin Amin <ramtin@lambdaconcept.com>
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
 ******************************************************************************************************************* */

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_USBHS_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_USBHS_H

/********************************************************************************************************************
 * Included Files
 ******************************************************************************************************************* */

#include <nuttx/config.h>
#include "chip.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ******************************************************************************************************************* */

/* Register Offsets ************************************************************************************************ */

#define KINETIS_USBHS_ID_OFFSET                           0x0000  /* Identification Register */
#define KINETIS_USBHS_HWGENERAL_OFFSET                    0x0004  /* General Hardware Parameters */
#define KINETIS_USBHS_HWHOST_OFFSET                       0x0008  /* Host Hardware Parameters */
#define KINETIS_USBHS_HWDEVICE_OFFSET                     0x000c  /* Device Hardware Parameters */
#define KINETIS_USBHS_HWTXBUF_OFFSET                      0x0010  /* TX Buffer Hardware Parameters */
#define KINETIS_USBHS_HWRXBUF_OFFSET                      0x0014  /* RX Buffer Hardware Parameters */
#define KINETIS_USBHS_GPTIMER0LD_OFFSET                   0x0080  /* General Purpose Timer 0 Load */
#define KINETIS_USBHS_GPTIMER0CTL_OFFSET                  0x0084  /* General Purpose Timer 0 Control */
#define KINETIS_USBHS_GPTIMER1LD_OFFSET                   0x0088  /* General Purpose Timer 1 Load */
#define KINETIS_USBHS_GPTIMER1CTL_OFFSET                  0x008c  /* General Purpose Timer 1 Control */
#define KINETIS_USBHS_HCIVERSION_OFFSET                   0x0100  /* Host Interface Version Number */
#define KINETIS_USBHS_CAPLENGTH_OFFSET                    0x0103  /* Capability Register Length */
#define KINETIS_USBHS_HCSPARAMS_OFFSET                    0x0104  /* Host Structural Parameters */
#define KINETIS_USBHS_HCCPARAMS_OFFSET                    0x0108  /* Host Capability Parameters */
#define KINETIS_USBHS_DCIVERSION_OFFSET                   0x0122  /* Device Interface Version Number */
#define KINETIS_USBHS_DCCPARAMS_OFFSET                    0x0124  /* Device Capability Parameters */
#define KINETIS_USBHS_USBCMD_OFFSET                       0x0140  /* USB Command */
#define KINETIS_USBHS_USBSTS_OFFSET                       0x0144  /* USB Status */
#define KINETIS_USBHS_USBINTR_OFFSET                      0x0148  /* USB Interrupt Enable */
#define KINETIS_USBHS_FRINDEX_OFFSET                      0x014c  /* USB Frame Index */
#define KINETIS_USBHS_PERIODICLISTBASE_OFFSET             0x0154  /* Periodic Frame List Base Address */
#define KINETIS_USBHS_DEVICEADDR_OFFSET                   0x0154  /* Device Address */
#define KINETIS_USBHS_ASYNCLISTADDR_OFFSET                0x0158  /* Current Asynchronous List Address */
#define KINETIS_USBHS_EPLISTADDR_OFFSET                   0x0158  /* Address at Endpoint List */
#define KINETIS_USBHS_TTCTRL_OFFSET                       0x015c  /* Host TT Asynchronous Buffer Control */
#define KINETIS_USBHS_BURSTSIZE_OFFSET                    0x0160  /* Master Interface Data Burst Size */
#define KINETIS_USBHS_TXFILLTUNING_OFFSET                 0x0164  /* Host Transmit FIFO Tuning Control */
#define KINETIS_USBHS_CONFIGFLAG_OFFSET                   0x0180  /* Configure Flag Register */
#define KINETIS_USBHS_PORTSC1_OFFSET                      0x0184  /* Port Status/Control */
#define KINETIS_USBHS_OTGSC_OFFSET                        0x01a4  /* On-The-Go Status and Control */
#define KINETIS_USBHS_MODE_OFFSET                         0x01a8  /* USB Mode Register */
#define KINETIS_USBHS_EPSETUPSR_OFFSET                    0x01ac  /* Endpoint Setup Status Register */
#define KINETIS_USBHS_EPPRIME_OFFSET                      0x01b0  /* Endpoint Initialization */
#define KINETIS_USBHS_EPFLUSH_OFFSET                      0x01b4  /* Endpoint De-initialize */
#define KINETIS_USBHS_EPSR_OFFSET                         0x01b8  /* Endpoint Status Register */
#define KINETIS_USBHS_EPCOMPLETE_OFFSET                   0x01bc  /* Endpoint Complete */
#define KINETIS_USBHS_EPCR0_OFFSET                        0x01c0  /* Endpoint Control Register 0 */
#define KINETIS_USBHS_EPCR1_OFFSET                        0x01c4  /* Endpoint Control Register 1 */
#define KINETIS_USBHS_EPCR2_OFFSET                        0x01c8  /* Endpoint Control Register 2 */
#define KINETIS_USBHS_EPCR3_OFFSET                        0x01cc  /* Endpoint Control Register 3 */
#define KINETIS_USBHS_EPCR4_OFFSET                        0x01d0  /* Endpoint Control Register 4 */
#define KINETIS_USBHS_EPCR5_OFFSET                        0x01d4  /* Endpoint Control Register 5 */
#define KINETIS_USBHS_EPCR6_OFFSET                        0x01d8  /* Endpoint Control Register 6 */
#define KINETIS_USBHS_EPCR7_OFFSET                        0x01dc  /* Endpoint Control Register 7 */
#define KINETIS_USBHS_USBGENCTRL_OFFSET                   0x0200  /* USB General Control Register */

#define KINETIS_USBHSPHY_PWD_OFFSET                       0x0000  /* USB PHY Power-Down Register */
#define KINETIS_USBHSPHY_PWD_SET_OFFSET                   0x0004  /* USB PHY Power-Down Register */
#define KINETIS_USBHSPHY_PWD_CLR_OFFSET                   0x0008  /* USB PHY Power-Down Register */
#define KINETIS_USBHSPHY_PWD_TOG_OFFSET                   0x000c  /* USB PHY Power-Down Register */
#define KINETIS_USBHSPHY_TX_OFFSET                        0x0010  /* USB PHY Transmitter Control Register */
#define KINETIS_USBHSPHY_TX_SET_OFFSET                    0x0014  /* USB PHY Transmitter Control Register */
#define KINETIS_USBHSPHY_TX_CLR_OFFSET                    0x0018  /* USB PHY Transmitter Control Register */
#define KINETIS_USBHSPHY_TX_TOG_OFFSET                    0x001c  /* USB PHY Transmitter Control Register */
#define KINETIS_USBHSPHY_RX_OFFSET                        0x0020  /* USB PHY Receiver Control Register */
#define KINETIS_USBHSPHY_RX_SET_OFFSET                    0x0024  /* USB PHY Receiver Control Register */
#define KINETIS_USBHSPHY_RX_CLR_OFFSET                    0x0028  /* USB PHY Receiver Control Register */
#define KINETIS_USBHSPHY_RX_TOG_OFFSET                    0x002c  /* USB PHY Receiver Control Register */
#define KINETIS_USBHSPHY_CTRL_OFFSET                      0x0030  /* USB PHY General Control Register */
#define KINETIS_USBHSPHY_CTRL_SET_OFFSET                  0x0034  /* USB PHY General Control Register */
#define KINETIS_USBHSPHY_CTRL_CLR_OFFSET                  0x0038  /* USB PHY General Control Register */
#define KINETIS_USBHSPHY_CTRL_TOG_OFFSET                  0x003c  /* USB PHY General Control Register */
#define KINETIS_USBHSPHY_STATUS_OFFSET                    0x0040  /* USB PHY Status Register */
#define KINETIS_USBHSPHY_DEBUG_SET_OFFSET                 0x0054  /* USB PHY Debug Register */
#define KINETIS_USBHSPHY_DEBUG_CLR_OFFSET                 0x0058  /* USB PHY Debug Register */
#define KINETIS_USBHSPHY_DEBUG_TOG_OFFSET                 0x005c  /* USB PHY Debug Register */
#define KINETIS_USBHSPHY_DEBUG0_STATUS_OFFSET             0x0060  /* UTMI Debug Status Register 0 */
#define KINETIS_USBHSPHY_DEBUG1_OFFSET                    0x0070  /* UTMI Debug Status Register 1 */
#define KINETIS_USBHSPHY_DEBUG1_SET_OFFSET                0x0074  /* UTMI Debug Status Register 1 */
#define KINETIS_USBHSPHY_DEBUG1_CLR_OFFSET                0x0078  /* UTMI Debug Status Register 1 */
#define KINETIS_USBHSPHY_DEBUG1_TOG_OFFSET                0x007c  /* UTMI Debug Status Register 1 */
#define KINETIS_USBHSPHY_VERSION_OFFSET                   0x0080  /* UTMI RTL Version */
#define KINETIS_USBHSPHY_PLL_SIC_OFFSET                   0x00a0  /* USB PHY PLL Control/Status Register */
#define KINETIS_USBHSPHY_PLL_SIC_SET_OFFSET               0x00a4  /* USB PHY PLL Control/Status Register */
#define KINETIS_USBHSPHY_PLL_SIC_CLR_OFFSET               0x00a8  /* USB PHY PLL Control/Status Register */
#define KINETIS_USBHSPHY_PLL_SIC_TOG_OFFSET               0x00ac  /* USB PHY PLL Control/Status Register */
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT_OFFSET          0x00c0  /* USB PHY VBUS Detect Control Register */
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT_SET_OFFSET      0x00c4  /* USB PHY VBUS Detect Control Register */
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT_CLR_OFFSET      0x00c8  /* USB PHY VBUS Detect Control Register */
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT_TOG_OFFSET      0x00cc  /* USB PHY VBUS Detect Control Register */
#define KINETIS_USBHSPHY_USB1_VBUS_DET_STAT_OFFSET        0x00d0  /* USB PHY VBUS Detector Status Register */
#define KINETIS_USBHSPHY_USB1_CHRG_DET_STAT_OFFSET        0x00f0  /* USB PHY Charger Detect Status Register */
#define KINETIS_USBHSPHY_ANACTRL_OFFSET                   0x0100  /* USB PHY Analog Control Register */
#define KINETIS_USBHSPHY_ANACTRL_SET_OFFSET               0x0104  /* USB PHY Analog Control Register */
#define KINETIS_USBHSPHY_ANACTRL_CLR_OFFSET               0x0108  /* USB PHY Analog Control Register */
#define KINETIS_USBHSPHY_ANACTRL_TOG_OFFSET               0x010c  /* USB PHY Analog Control Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_OFFSET             0x0110  /* USB PHY Loopback Control/Status Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_SET_OFFSET         0x0114  /* USB PHY Loopback Control/Status Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_CLR_OFFSET         0x0118  /* USB PHY Loopback Control/Status Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_TOG_OFFSET         0x011c  /* USB PHY Loopback Control/Status Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_OFFSET     0x0120  /* USB PHY Loopback Packet Number Select Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_SET_OFFSET 0x0124  /* USB PHY Loopback Packet Number Select Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_CLR_OFFSET 0x0128  /* USB PHY Loopback Packet Number Select Register */
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_TOG_OFFSET 0x012c  /* USB PHY Loopback Packet Number Select Register */
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_OFFSET          0x0130  /* USB PHY Trim Override Enable Register */
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_SET_OFFSET      0x0134  /* USB PHY Trim Override Enable Register */
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_CLR_OFFSET      0x0138  /* USB PHY Trim Override Enable Register */
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_TOG_OFFSET      0x013c  /* USB PHY Trim Override Enable Register */

#define KINETIS_USBHSDCD_CONTROL_OFFSET                   0x0000  /* Control register */
#define KINETIS_USBHSDCD_CLOCK_OFFSET                     0x0004  /* Clock register */
#define KINETIS_USBHSDCD_STATUS_OFFSET                    0x0008  /* Status register */
#define KINETIS_USBHSDCD_SIGNAL_OVERRIDE_OFFSET           0x000c  /* Signal Override Register */
#define KINETIS_USBHSDCD_TIMER0_OFFSET                    0x0010  /* TIMER0 register */
#define KINETIS_USBHSDCD_TIMER1_OFFSET                    0x0014  /* TIMER1 register */
#define KINETIS_USBHSDCD_TIMER2_BC11_OFFSET               0x0018  /*  TIMER2_BC11 register */
#define KINETIS_USBHSDCD_TIMER2_BC12_OFFSET               0x001c  /* TIMER2_BC12 register */

/* Register Addresses ********************************************************************************************** */

#define KINETIS_USBHS_ID                                  (KINETIS_USBHS_BASE + KINETIS_USBHS_ID_OFFSET)
#define KINETIS_USBHS_HWGENERAL                           (KINETIS_USBHS_BASE + KINETIS_USBHS_HWGENERAL_OFFSET)
#define KINETIS_USBHS_HWHOST                              (KINETIS_USBHS_BASE + KINETIS_USBHS_HWHOST_OFFSET)
#define KINETIS_USBHS_HWDEVICE                            (KINETIS_USBHS_BASE + KINETIS_USBHS_HWDEVICE_OFFSET)
#define KINETIS_USBHS_HWTXBUF                             (KINETIS_USBHS_BASE + KINETIS_USBHS_HWTXBUF_OFFSET)
#define KINETIS_USBHS_HWRXBUF                             (KINETIS_USBHS_BASE + KINETIS_USBHS_HWRXBUF_OFFSET)
#define KINETIS_USBHS_GPTIMER0LD                          (KINETIS_USBHS_BASE + KINETIS_USBHS_GPTIMER0LD_OFFSET)
#define KINETIS_USBHS_GPTIMER0CTL                         (KINETIS_USBHS_BASE + KINETIS_USBHS_GPTIMER0CTL_OFFSET)
#define KINETIS_USBHS_GPTIMER1LD                          (KINETIS_USBHS_BASE + KINETIS_USBHS_GPTIMER1LD_OFFSET)
#define KINETIS_USBHS_GPTIMER1CTL                         (KINETIS_USBHS_BASE + KINETIS_USBHS_GPTIMER1CTL_OFFSET)
#define KINETIS_USBHS_HCIVERSION                          (KINETIS_USBHS_BASE + KINETIS_USBHS_HCIVERSION_OFFSET)
#define KINETIS_USBHS_CAPLENGTH                           (KINETIS_USBHS_BASE + KINETIS_USBHS_CAPLENGTH_OFFSET)
#define KINETIS_USBHS_HCSPARAMS                           (KINETIS_USBHS_BASE + KINETIS_USBHS_HCSPARAMS_OFFSET)
#define KINETIS_USBHS_HCCPARAMS                           (KINETIS_USBHS_BASE + KINETIS_USBHS_HCCPARAMS_OFFSET)
#define KINETIS_USBHS_DCIVERSION                          (KINETIS_USBHS_BASE + KINETIS_USBHS_DCIVERSION_OFFSET)
#define KINETIS_USBHS_DCCPARAMS                           (KINETIS_USBHS_BASE + KINETIS_USBHS_DCCPARAMS_OFFSET)
#define KINETIS_USBHS_USBCMD                              (KINETIS_USBHS_BASE + KINETIS_USBHS_USBCMD_OFFSET)
#define KINETIS_USBHS_USBSTS                              (KINETIS_USBHS_BASE + KINETIS_USBHS_USBSTS_OFFSET)
#define KINETIS_USBHS_USBINTR                             (KINETIS_USBHS_BASE + KINETIS_USBHS_USBINTR_OFFSET)
#define KINETIS_USBHS_FRINDEX                             (KINETIS_USBHS_BASE + KINETIS_USBHS_FRINDEX_OFFSET)
#define KINETIS_USBHS_PERIODICLISTBASE                    (KINETIS_USBHS_BASE + KINETIS_USBHS_PERIODICLISTBASE_OFFSET)
#define KINETIS_USBHS_DEVICEADDR                          (KINETIS_USBHS_BASE + KINETIS_USBHS_DEVICEADDR_OFFSET)
#define KINETIS_USBHS_ASYNCLISTADDR                       (KINETIS_USBHS_BASE + KINETIS_USBHS_ASYNCLISTADDR_OFFSET)
#define KINETIS_USBHS_EPLISTADDR                          (KINETIS_USBHS_BASE + KINETIS_USBHS_EPLISTADDR_OFFSET)
#define KINETIS_USBHS_TTCTRL                              (KINETIS_USBHS_BASE + KINETIS_USBHS_TTCTRL_OFFSET)
#define KINETIS_USBHS_BURSTSIZE                           (KINETIS_USBHS_BASE + KINETIS_USBHS_BURSTSIZE_OFFSET)
#define KINETIS_USBHS_TXFILLTUNING                        (KINETIS_USBHS_BASE + KINETIS_USBHS_TXFILLTUNING_OFFSET)
#define KINETIS_USBHS_CONFIGFLAG                          (KINETIS_USBHS_BASE + KINETIS_USBHS_CONFIGFLAG_OFFSET)
#define KINETIS_USBHS_PORTSC1                             (KINETIS_USBHS_BASE + KINETIS_USBHS_PORTSC1_OFFSET)
#define KINETIS_USBHS_OTGSC                               (KINETIS_USBHS_BASE + KINETIS_USBHS_OTGSC_OFFSET)
#define KINETIS_USBHS_MODE                                (KINETIS_USBHS_BASE + KINETIS_USBHS_MODE_OFFSET)
#define KINETIS_USBHS_EPSETUPSR                           (KINETIS_USBHS_BASE + KINETIS_USBHS_EPSETUPSR_OFFSET)
#define KINETIS_USBHS_EPPRIME                             (KINETIS_USBHS_BASE + KINETIS_USBHS_EPPRIME_OFFSET)
#define KINETIS_USBHS_EPFLUSH                             (KINETIS_USBHS_BASE + KINETIS_USBHS_EPFLUSH_OFFSET)
#define KINETIS_USBHS_EPSR                                (KINETIS_USBHS_BASE + KINETIS_USBHS_EPSR_OFFSET)
#define KINETIS_USBHS_EPCOMPLETE                          (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCOMPLETE_OFFSET)
#define KINETIS_USBHS_EPCR0                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR0_OFFSET)
#define KINETIS_USBHS_EPCR1                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR1_OFFSET)
#define KINETIS_USBHS_EPCR2                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR2_OFFSET)
#define KINETIS_USBHS_EPCR3                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR3_OFFSET)
#define KINETIS_USBHS_EPCR4                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR4_OFFSET)
#define KINETIS_USBHS_EPCR5                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR5_OFFSET)
#define KINETIS_USBHS_EPCR6                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR6_OFFSET)
#define KINETIS_USBHS_EPCR7                               (KINETIS_USBHS_BASE + KINETIS_USBHS_EPCR7_OFFSET)
#define KINETIS_USBHS_USBGENCTRL                          (KINETIS_USBHS_BASE + KINETIS_USBHS_USBGENCTRL_OFFSET)

#define KINETIS_USBHSPHY_PWD                              (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PWD_OFFSET)
#define KINETIS_USBHSPHY_PWD_SET                          (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PWD_SET_OFFSET)
#define KINETIS_USBHSPHY_PWD_CLR                          (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PWD_CLR_OFFSET)
#define KINETIS_USBHSPHY_PWD_TOG                          (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PWD_TOG_OFFSET)
#define KINETIS_USBHSPHY_TX                               (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TX_OFFSET)
#define KINETIS_USBHSPHY_TX_SET                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TX_SET_OFFSET)
#define KINETIS_USBHSPHY_TX_CLR                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TX_CLR_OFFSET)
#define KINETIS_USBHSPHY_TX_TOG                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TX_TOG_OFFSET)
#define KINETIS_USBHSPHY_RX                               (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_RX_OFFSET)
#define KINETIS_USBHSPHY_RX_SET                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_RX_SET_OFFSET)
#define KINETIS_USBHSPHY_RX_CLR                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_RX_CLR_OFFSET)
#define KINETIS_USBHSPHY_RX_TOG                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_RX_TOG_OFFSET)
#define KINETIS_USBHSPHY_CTRL                             (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_CTRL_OFFSET)
#define KINETIS_USBHSPHY_CTRL_SET                         (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_CTRL_SET_OFFSET)
#define KINETIS_USBHSPHY_CTRL_CLR                         (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_CTRL_CLR_OFFSET)
#define KINETIS_USBHSPHY_CTRL_TOG                         (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_CTRL_TOG_OFFSET)
#define KINETIS_USBHSPHY_STATUS                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_STATUS_OFFSET)
#define KINETIS_USBHSPHY_DEBUG_SET                        (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG_SET_OFFSET)
#define KINETIS_USBHSPHY_DEBUG_CLR                        (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG_CLR_OFFSET)
#define KINETIS_USBHSPHY_DEBUG_TOG                        (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG_TOG_OFFSET)
#define KINETIS_USBHSPHY_DEBUG0_STATUS                    (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG0_STATUS_OFFSET)
#define KINETIS_USBHSPHY_DEBUG1                           (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG1_OFFSET)
#define KINETIS_USBHSPHY_DEBUG1_SET                       (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG1_SET_OFFSET)
#define KINETIS_USBHSPHY_DEBUG1_CLR                       (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG1_CLR_OFFSET)
#define KINETIS_USBHSPHY_DEBUG1_TOG                       (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_DEBUG1_TOG_OFFSET)
#define KINETIS_USBHSPHY_VERSION                          (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_VERSION_OFFSET)
#define KINETIS_USBHSPHY_PLL_SIC                          (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PLL_SIC_OFFSET)
#define KINETIS_USBHSPHY_PLL_SIC_SET                      (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PLL_SIC_SET_OFFSET)
#define KINETIS_USBHSPHY_PLL_SIC_CLR                      (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PLL_SIC_CLR_OFFSET)
#define KINETIS_USBHSPHY_PLL_SIC_TOG                      (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_PLL_SIC_TOG_OFFSET)
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT                 (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_VBUS_DETECT_OFFSET)
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT_SET             (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_VBUS_DETECT_SET_OFFSET)
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT_CLR             (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_VBUS_DETECT_CLR_OFFSET)
#define KINETIS_USBHSPHY_USB1_VBUS_DETECT_TOG             (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_VBUS_DETECT_TOG_OFFSET)
#define KINETIS_USBHSPHY_USB1_VBUS_DET_STAT               (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_VBUS_DET_STAT_OFFSET)
#define KINETIS_USBHSPHY_USB1_CHRG_DET_STAT               (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_CHRG_DET_STAT_OFFSET)
#define KINETIS_USBHSPHY_ANACTRL                          (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_ANACTRL_OFFSET)
#define KINETIS_USBHSPHY_ANACTRL_SET                      (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_ANACTRL_SET_OFFSET)
#define KINETIS_USBHSPHY_ANACTRL_CLR                      (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_ANACTRL_CLR_OFFSET)
#define KINETIS_USBHSPHY_ANACTRL_TOG                      (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_ANACTRL_TOG_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK                    (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK_SET                (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_SET_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK_CLR                (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_CLR_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK_TOG                (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_TOG_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT            (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_SET        (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_SET_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_CLR        (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_CLR_OFFSET)
#define KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_TOG        (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_USB1_LOOPBACK_HSFSCNT_TOG_OFFSET)
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN                 (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_OFFSET)
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_SET             (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_SET_OFFSET)
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_CLR             (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_CLR_OFFSET)
#define KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_TOG             (KINETIS_USBHSPHY_BASE + KINETIS_USBHSPHY_TRIM_OVERRIDE_EN_TOG_OFFSET)

#define KINETIS_USBHSDCD_CONTROL                          (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_CONTROL_OFFSET)
#define KINETIS_USBHSDCD_CLOCK                            (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_CLOCK_OFFSET)
#define KINETIS_USBHSDCD_STATUS                           (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_STATUS_OFFSET)
#define KINETIS_USBHSDCD_SIGNAL_OVERRIDE                  (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_SIGNAL_OVERRIDE_OFFSET)
#define KINETIS_USBHSDCD_TIMER0                           (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_TIMER0_OFFSET)
#define KINETIS_USBHSDCD_TIMER1                           (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_TIMER1_OFFSET)
#define KINETIS_USBHSDCD_TIMER2_BC11                      (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_TIMER2_BC11_OFFSET)
#define KINETIS_USBHSDCD_TIMER2_BC12                      (KINETIS_USBHSDCD_BASE + KINETIS_USBHSDCD_TIMER2_BC12_OFFSET)

/* Register Bit-Field Definitions ********************************************************************************** */
/* Identification Register */

#define USBHS_ID_VERSIONID_SHIFT                          (29)      /* Bits 29-31: Version ID */
#define USBHS_ID_VERSIONID_MASK                           (0x7 << USBHS_ID_VERSIONID_SHIFT)
#define USBHS_ID_VERSION_SHIFT                            (25)      /* Bits 25-28: Version */
#define USBHS_ID_VERSION_MASK                             (0xf << USBHS_ID_VERSION_SHIFT)
#define USBHS_ID_REVISION_SHIFT                           (21)      /* Bits 21-24: Revision */
#define USBHS_ID_REVISION_MASK                            (0xf << USBHS_ID_REVISION_SHIFT)
#define USBHS_ID_TAG_SHIFT                                (16)      /* Bits 16-20: Tag */
#define USBHS_ID_TAG_MASK                                 (0x1f << USBHS_ID_TAG_SHIFT)
                                                                    /* Bits 14-15: Reserved */
#define USBHS_ID_NID_SHIFT                                (8)       /* Bits 8-13: Ones complement version of ID */
#define USBHS_ID_NID_MASK                                 (0x3f << USBHS_ID_NID_SHIFT)
                                                                    /* Bits 6-7: Reserved */
#define USBHS_ID_ID_SHIFT                                 (0)       /* Bits 0-5: ID Configuration number */
#define USBHS_ID_ID_MASK                                  (0x3f << USBHS_ID_ID_SHIFT)

/* General Hardware Parameters Register */

                                                                    /* Bits 11-31: Reserved */
#define USBHS_HWGENERAL_SM_SHIFT                          (9)       /* Bits 9-10: Serial mode */
#define USBHS_HWGENERAL_SM_MASK                           (0x3 << USBHS_HWGENERAL_SM_SHIFT)
#define USBHS_HWGENERAL_PHYM_SHIFT                        (6)       /* Bits 6-8: PHY Mode */
#define USBHS_HWGENERAL_PHYM_MASK                         (0x7 << USBHS_HWGENERAL_PHYM_SHIFT)
#define USBHS_HWGENERAL_PHYW_SHIFT                        (4)       /* Bits 4-5: PHY Width */
#define USBHS_HWGENERAL_PHYW_MASK                         (0x3 << USBHS_HWGENERAL_PHYW_SHIFT)
                                                                    /* Bits 0-3: Reserved */

/* Host Hardware Parameters Register */

#define USBHS_HWHOST_TTPER_SHIFT                          (24)      /* Bits 24-31: Transaction translator periodic contexts */
#define USBHS_HWHOST_TTPER_MASK                           (0xff << USBHS_HWHOST_TTPER_SHIFT)
#define USBHS_HWHOST_TTASY_SHIFT                          (16)      /* Bits 16-23: Transaction translator contexts */
#define USBHS_HWHOST_TTASY_MASK                           (0xff << USBHS_HWHOST_TTASY_SHIFT)
                                                                    /* Bits 5-14: Reserved */
#define USBHS_HWHOST_NPORT_SHIFT                          (1)       /* Bits 1-3: Number of Ports */
#define USBHS_HWHOST_NPORT_MASK                           (0x7 << USBHS_HWHOST_NPORT_SHIFT)
#define USBHS_HWHOST_HC                                   (1 << 0)  /* Bit 0: Host Capable */

/* Device Hardware Parameters Register */

                                                                    /* Bits 6-31: Reserved */
#define USBHS_HWDEVICE_DEVEP_SHIFT                        (1)       /* Bits 1-5: Device endpoints */
#define USBHS_HWDEVICE_DEVEP_MASK                         (0x1f << USBHS_HWDEVICE_DEVEP_SHIFT)
#define USBHS_HWDEVICE_DC                                 (1 << 0)  /* Bit 0: Device Capable */

/* Transmit Buffer Hardware Parameters Register */

#define USBHS_HWTXBUF_TXLC                                (1 << 31) /* Bit 31: Transmit local Context Registers */
                                                                    /* Bits 24-30: Reserved */
#define USBHS_HWTXBUF_TXCHANADD_SHIFT                     (16)      /* Bits 16-23: Transmit Channel Address */
#define USBHS_HWTXBUF_TXCHANADD_MASK                      (0xff << USBHS_HWTXBUF_TXCHANADD_SHIFT)
#define USBHS_HWTXBUF_TXADD_SHIFT                         (8)       /* Bits 8-15: Transmit Address */
#define USBHS_HWTXBUF_TXADD_MASK                          (0xff << USBHS_HWTXBUF_TXADD_SHIFT)
#define USBHS_HWTXBUF_TXBURST_SHIFT                       (0)       /* Bits 0-7: Transmit Burst */
#define USBHS_HWTXBUF_TXBURST_MASK                        (0xff << USBHS_HWTXBUF_TXBURST_SHIFT)

/* Receive Buffer Hardware Parameters Register */

                                                                    /* Bits 16-31: Reserved */
#define USBHS_HWRXBUF_RXADD_SHIFT                         (8)       /* Bits 8-15: Receive Address */
#define USBHS_HWRXBUF_RXADD_MASK                          (0xff << USBHS_HWRXBUF_RXADD_SHIFT)
#define USBHS_HWRXBUF_RXBURST_SHIFT                       (0)       /* Bits 0-7: Receive Burst */
#define USBHS_HWRXBUF_RXBURST_MASK                        (0xff << USBHS_HWRXBUF_RXBURST_SHIFT)

/* General Purpose Timer n Load Register */

                                                                     /* Bits 24-31: Reserved */
#define USBHS_GPTIMERnLD_GPTLD_SHIFT                      (0)        /* Bits 0-23: Value to be loaded into the countdown timer on reset */
#define USBHS_GPTIMERnLD_GPTLD_MASK                       (0xffffff << USBHS_GPTIMERnLD_GPTLD_SHIFT)

/* General Purpose Timer n Control Register */

#define USBHS_GPTIMERnCTL_RUN                             (1 << 31) /* Bit 31: Timer Run */
#define USBHS_GPTIMERnCTL_RST                             (1 << 30) /* Bit 30: Timer Reset */
                                                                    /* Bits 25-29: Reserved */
#define USBHS_GPTIMERnCTL_MODE                            (1 << 24) /* Bit 24: Timer Mode */
#define USBHS_GPTIMERnCTL_GPTCNT_SHIFT                    (0)       /* Bits 0-23: Timer Count */
#define USBHS_GPTIMERnCTL_GPTCNT_MASK                     (0xffffff << USBHS_GPTIMERnCTL_GPTCNT_SHIFT)

/* System Bus Interface Configuration Register */

                                                                    /* Bits 3-31: Reserved */
#define USBHS_USB_SBUSCFG_BURSTMODE_SHIFT                 (0)       /* Bits 0-2: Burst mode */
#define USBHS_USB_SBUSCFG_BURSTMODE_MASK                  (0x7 << USBHS_USB_SBUSCFG_BURSTMODE_SHIFT)

/* Host Controller Interface Version and Capability Registers Length Register */

#define USBHS_HCIVERSION_HCIVERSION_SHIFT                 (16)      /* Bits 16-31: EHCI revision number */
#define USBHS_HCIVERSION_HCIVERSION_MASK                  (0xffff << USBHS_HCIVERSION_HCIVERSION_SHIFT)
                                                                    /* Bits 8-15: Reserved */
#define USBHS_HCIVERSION_CAPLENGTH_SHIFT                  (0)       /* Bits 0-7: Capability registers length */
#define USBHS_HCIVERSION_CAPLENGTH_MASK                   (0xff << USBHS_HCIVERSION_CAPLENGTH_SHIFT)

/* Host Controller Structural Parameters Register */

                                                                    /* Bits 28-31: Reserved */
#define USBHS_HCSPARAMS_N_TT_SHIFT                        (24)      /* Bits 24-27: Number of Transaction Translators */
#define USBHS_HCSPARAMS_N_TT_MASK                         (0xf << USBHS_HCSPARAMS_N_TT_SHIFT)
#define USBHS_HCSPARAMS_N_PTT_SHIFT                       (20)      /* Bits 22-30: Ports per Transaction Translator */
#define USBHS_HCSPARAMS_N_PTT_MASK                        (0xf << USBHS_HCSPARAMS_N_PTT_SHIFT)
                                                                    /* Bits 17-19: Reserved */
#define USBHS_HCSPARAMS_PI                                (1 << 16) /* Bit 16: Port Indicators */
#define USBHS_HCSPARAMS_N_CC_SHIFT                        (12)      /* Bits 12-15: Number of Companion Controllers */
#define USBHS_HCSPARAMS_N_CC_MASK                         (0xf << USBHS_HCSPARAMS_N_CC_SHIFT)
#define USBHS_HCSPARAMS_N_PCC_SHIFT                       (8)       /* Bits 8-11: Number Ports per CC */
#define USBHS_HCSPARAMS_N_PCC_MASK                        (0xf << USBHS_HCSPARAMS_N_PCC_SHIFT)
                                                                    /* Bits 5-7: Reserved */
#define USBHS_HCSPARAMS_PPC                               (1 << 4)  /* Bit 4: Power Port Control */
#define USBHS_HCSPARAMS_N_PORTS_SHIFT                     (0)       /* Bits 0-3: Number of Ports */
#define USBHS_HCSPARAMS_N_PORTS_MASK                      (0xf << USBHS_HCSPARAMS_N_PORTS_SHIFT)

/* Host Controller Capability Parameters Register */

                                                                    /* Bits 16-31: Reserved */
#define USBHS_HCCPARAMS_EECP_SHIFT                        (8)       /* Bits 8-15: EHCI Extended Capabilities Pointer */
#define USBHS_HCCPARAMS_EECP_MASK                         (0xff << USBHS_HCCPARAMS_EECP_SHIFT)
#define USBHS_HCCPARAMS_IST_SHIFT                         (4)       /* Bits 4-7: Isochronous Scheduling Threshold */
#define USBHS_HCCPARAMS_IST_MASK                          (0xf << USBHS_HCCPARAMS_IST_SHIFT)
                                                                    /* Bit 3: Reserved */
#define USBHS_HCCPARAMS_ASP                               (1 << 2)  /* Bit 2: Asynchronous Schedule Park capability */
#define USBHS_HCCPARAMS_PFL                               (1 << 1)  /* Bit 1: Programmable Frame List flag */
#define USBHS_HCCPARAMS_ADC                               (1 << 0)  /* Bit 0: 64-bit addressing capability */

/* Device Controller Capability Parameters */

                                                                    /* Bits 9-31: Reserved */
#define USBHS_DCCPARAMS_HC                                (1 << 8)  /* Bit 8: Host Capable */
#define USBHS_DCCPARAMS_DC                                (1 << 7)  /* Bit 7: Device Capable */
                                                                    /* Bits 5-6: Reserved */
#define USBHS_DCCPARAMS_DEN_SHIFT                         (0)       /* Bits 0-4: Device Endpoint Number */
#define USBHS_DCCPARAMS_DEN_MASK                          (0x1f << USBHS_DCCPARAMS_DEN_SHIFT)

/* USB Command Register */

                                                                    /* Bits 24-31: Reserved */
#define USBHS_USBCMD_ITC_SHIFT                            (16)      /* Bits 16-23: Interrupt Threshold Control */
#define USBHS_USBCMD_ITC_MASK                             (0xff << USBHS_USBCMD_ITC_SHIFT)
#define USBHS_USBCMD_FS2                                  (1 << 15) /* Bit 15: Frame list Size 2 */
#define USBHS_USBCMD_ATDTW                                (1 << 14) /* Bit 14: Add dTD TripWire */
#define USBHS_USBCMD_SUTW                                 (1 << 13) /* Bit 13: Setup TripWire */
                                                                    /* Bit 12: Reserved */
#define USBHS_USBCMD_ASPE                                 (1 << 11) /* Bit 11: Asynchronous Schedule Park mode Enable */
                                                                    /* Bit 10: Reserved */
#define USBHS_USBCMD_ASP_SHIFT                            (8)       /* Bits 8-9: Asynchronous Schedule Park mode count */
#define USBHS_USBCMD_ASP_MASK                             (0x3 << USBHS_USBCMD_ASP_SHIFT)
                                                                    /* Bit 7: Reserved */
#define USBHS_USBCMD_IAA                                  (1 << 6)  /* Bit 6: Interrupt on Async Advance doorbell */
#define USBHS_USBCMD_ASE                                  (1 << 5)  /* Bit 5: Asynchronous Schedule Enable */
#define USBHS_USBCMD_PSE                                  (1 << 4)  /* Bit 4: Periodic Schedule Enable */
#define USBHS_USBCMD_FS_SHIFT                             (2)       /* Bits 2-3: Frame list Size */
#define USBHS_USBCMD_FS_MASK                              (0x3 << USBHS_USBCMD_FS_SHIFT)
#define USBHS_USBCMD_RST                                  (1 << 1)  /* Bit 1: Controller Reset */
#define USBHS_USBCMD_RS                                   (1 << 0)  /* Bit 0: Run/Stop */

/* USB Status Register */

                                                                    /* Bits 26-31: Reserved */
#define USBHS_USBSTS_TI1                                  (1 << 25) /* Bit 25: General purpose Timer 1 Interrupt */
#define USBHS_USBSTS_TI0                                  (1 << 24) /* Bit 24: General purpose Timer 0 Interrupt */
                                                                    /* Bits 22-30: Reserved */
#define USBHS_USBSTS_UPIUSB                               (1 << 19) /* Bit 19:  host Periodic Interrupt */
#define USBHS_USBSTS_UAI                                  (1 << 18) /* Bit 18: USB host Asynchronous Interrupt */
                                                                    /* Bit 17: Reserved */
#define USBHS_USBSTS_NAKI                                 (1 << 16) /* Bit 16: NAK Interrupt */
#define USBHS_USBSTS_AS                                   (1 << 15) /* Bit 15: Asynchronous schedule Status */
#define USBHS_USBSTS_PS                                   (1 << 14) /* Bit 14: Periodic schedule Status */
#define USBHS_USBSTS_RCL                                  (1 << 13) /* Bit 13: Reclamation */
#define USBHS_USBSTS_HCH                                  (1 << 12) /* Bit 12: Host Controller Halted */
                                                                    /* Bits 9-11: Reserved */
#define USBHS_USBSTS_SLI                                  (1 << 8)  /* Bit 8: Device-controller suspend */
#define USBHS_USBSTS_SRI                                  (1 << 7)  /* Bit 7: SOF Received */
#define USBHS_USBSTS_URI                                  (1 << 6)  /* Bit 6: USB Reset received */
#define USBHS_USBSTS_AAI                                  (1 << 5)  /* Bit 5: Interrupt on Async Advance */
#define USBHS_USBSTS_SEI                                  (1 << 4)  /* Bit 4: System Error */
#define USBHS_USBSTS_FRI                                  (1 << 3)  /* Bit 3: Port Change detect */
#define USBHS_USBSTS_PCI                                  (1 << 2)  /* Bit 2: Port Change detect */
#define USBHS_USBSTS_UEI                                  (1 << 1)  /* Bit 1: USB Error Interrupt */
#define USBHS_USBSTS_UI                                   (1 << 0)  /* Bit 0: USB Interrupt (USBINT) */

/* USB Interrupt Enable Register */

                                                                    /* Bits 26-31: Reserved */
#define USBHS_USBINTR_TIE1                                (1 << 25) /* Bit 25: General purpose Timer 1 Interrupt Enable */
#define USBHS_USBINTR_TIE0                                (1 << 24) /* Bit 24: General purpose Timer 0 Interrupt Enable */
                                                                    /* Bits 22-30: Reserved */
#define USBHS_USBINTR_UPIE                                (1 << 19) /* Bit 19: USB host Periodic Interrupt Enable */
#define USBHS_USBINTR_UAIE                                (1 << 18) /* Bit 18: USB host Asynchronous Interrupt Enable */
                                                                    /* Bit 17: Reserved */
#define USBHS_USBINTR_NAKE                                (1 << 16) /* Bit 16: NAK Interrupt Enable */
                                                                    /* Bits 9-15: Reserved */
#define USBHS_USBINTR_SLE                                 (1 << 8)  /* Bit 8: Sleep (DC suspend) Enable */
#define USBHS_USBINTR_SRE                                 (1 << 7)  /* Bit 7: SOF-Received Enable */
#define USBHS_USBINTR_URE                                 (1 << 6)  /* Bit 6: USB-Reset Enable */
#define USBHS_USBINTR_AAE                                 (1 << 5)  /* Bit 5: Interrupt on Async advance Enable */
#define USBHS_USBINTR_SEE                                 (1 << 4)  /* Bit 4: System Error Enable */
#define USBHS_USBINTR_FRE                                 (1 << 3)  /* Bit 3: Frame list Rollover Enable */
#define USBHS_USBINTR_PCE                                 (1 << 2)  /* Bit 2: Port Change detect Enable */
#define USBHS_USBINTR_UEE                                 (1 << 1)  /* Bit 1: USB Error interrupt Enable */
#define USBHS_USBINTR_UE                                  (1 << 0)  /* Bit 0: USB interrupt Enable */

/* Frame Index Register */

                                                                    /* Bits 14_31: Reserved */
#define USBHS_FRINDEX_FRINDEX_SHIFT                       (0)       /* Bits 0-13: Frame Index */
#define USBHS_FRINDEX_FRINDEX_MASK                        (0x3fff << USBHS_FRINDEX_FRINDEX_SHIFT)

/* Periodic Frame List Base Address Register */

#define USBHS_PERIODICLISTBASE_PERBASE_SHIFT              (12)      /* Bits 12-31: Base address */
#define USBHS_PERIODICLISTBASE_PERBASE_MASK               (0xfffff << USBHS_PERIODICLISTBASE_PERBASE_SHIFT)
                                                                    /* Bits 0-12: Reserved */

/* Device Address Register */

#define USBHS_DEVICEADDR_USBADR_SHIFT                     (25)      /* Bits 25-31: Device Address */
#define USBHS_DEVICEADDR_USBADR_MASK                      (0x7f << USBHS_DEVICEADDR_USBADR_SHIFT)
#define USBHS_DEVICEADDR_USBADRA                          (1 << 24) /* Bit 24: Device Address Advance */
                                                                    /* Bits 0-23: Reserved */

/* Current Asynchronous List Address Register */

#define USBHS_ASYNCLISTADDR_ASYBASE_SHIFT                 (5)       /* Bits 5-31: Link pointer low (LPL) */
#define USBHS_ASYNCLISTADDR_ASYBASE_MASK                  (0x7ffffff << USBHS_ASYNCLISTADDR_ASYBASE_SHIFT)
                                                                    /* Bits 0-4: Reserved */

/* Endpoint List Address Register */

#define USBHS_EPLISTADDR_EPBASE_SHIFT                     (11)      /* Bits 11-31: Endpoint list address */
#define USBHS_EPLISTADDR_EPBASE_MASK                      (0x1fffff << USBHS_EPLISTADDR_EPBASE_SHIFT)
                                                                    /* Bits 0-10: Reserved */

/* Host TT Asynchronous Buffer Control */

                                                                    /* Bit 31: Reserved */
#define USBHS_TTCTRL_TTHA_SHIFT                           (24)      /* Bits 24-30: TT Hub Address */
#define USBHS_TTCTRL_TTHA_MASK                            (0x7f << USBHS_TTCTRL_TTHA_SHIFT)
                                                                    /* Bits 0-23: Reserved */

/* Master Interface Data Burst Size Register */

                                                                    /* Bits 16-31: Reserved */
#define USBHS_BURSTSIZE_TXPBURST_SHIFT                    (8)       /* Bits 8-15: Programable TX Burst length */
#define USBHS_BURSTSIZE_TXPBURST_MASK                     (0xff << USBHS_BURSTSIZE_TXPBURST_SHIFT)
#define USBHS_BURSTSIZE_RXPBURST_SHIFT                    (0)       /* Bits 0-7: Programable RX Burst length */
#define USBHS_BURSTSIZE_RXPBURST_MASK                     (0xff << USBHS_BURSTSIZE_RXPBURST_SHIFT)

/* Transmit FIFO Tuning Control Register */

                                                                    /* Bits 31-22: Reserved */
#define USBHS_TXFILLTUNING_TXFIFOTHRES_SHIFT              (16)      /* Bits 16-21: FIFO burst Threshold */
#define USBHS_TXFILLTUNING_TXFIFOTHRES_MASK               (0x3f << USBHS_TXFILLTUNING_TXFIFOTHRES_SHIFT)
                                                                    /* Bits 15-13: Reserved */
#define USBHS_TXFILLTUNING_TXSCHHEALTH_SHIFT              (8)       /* Bits 8-12: Scheduler Health counter */
#define USBHS_TXFILLTUNING_TXSCHHEALTH_MASK               (0x1f << USBHS_TXFILLTUNING_TXSCHHEALTH_SHIFT)
                                                                    /* Bit 7: Reserved */
#define USBHS_TXFILLTUNING_TXSCHOH_SHIFT                  (0)       /* Bits 0-6: Scheduler Overhead */
#define USBHS_TXFILLTUNING_TXSCHOH_MASK                   (0x7f << USBHS_TXFILLTUNING_TXSCHOH_SHIFT)

/* Endpoint NAK Register */

                                                                    /* Bits 20-31: Reserved */
#define USBHS_ENDPTNAK_EPTN_SHIFT                         (16)      /* Bits 16-19: TX Endpoint NAK */
#define USBHS_ENDPTNAK_EPTN_MASK                          (0xf << USBHS_ENDPTNAK_EPTN_SHIFT)
                                                                    /* Bits 4-15: Reserved */
#define USBHS_ENDPTNAK_EPRN_SHIFT                         (0)       /* Bits 0-3: RX Endpoint NAK */
#define USBHS_ENDPTNAK_EPRN_MASK                          (0xf << USBHS_ENDPTNAK_EPRN_SHIFT)

/* Endpoint NAK Enable Register */

                                                                    /* Bits 20-31: Reserved */
#define USBHS_ENDPTNAKEN_EPTNE_SHIFT                      (16)      /* Bits 16-19: TX Endpoint NAK */
#define USBHS_ENDPTNAKEN_EPTNE_MASK                       (0xf << USBHS_ENDPTNAKEN_EPTNE_SHIFT)
                                                                    /* Bits 4-15: Reserved */
#define USBHS_ENDPTNAKEN_EPRNE_SHIFT                      (0)       /* Bits 0-3: RX Endpoint NAK */
#define USBHS_ENDPTNAKEN_EPRNE_MASK                       (0xf << USBHS_ENDPTNAKEN_EPRNE_SHIFT)

/* Configure Flag Register (reserved) */

/* Port Status and Control Registers */

#define USBHS_PORTSC1_PTS_SHIFT                           (30)      /* Bits 30-31: Port Transceiver Select [1:0] */
#define USBHS_PORTSC1_PTS_MASK                            (0x3 << USBHS_PORTSC1_PTS_SHIFT)
                                                                    /* Bit 28-29: Reserved */
#define USBHS_PORTSC1_PSPD_SHIFT                          (26)      /* Bits 26-27: Port Speed */
#define USBHS_PORTSC1_PSPD_MASK                           (0x3 << USBHS_PORTSC1_PSPD_SHIFT)
#  define USBHS_PORTSC1_PTS2                              (1 << 25) /* Bit 25: Port Transceiver Select [2] */
#  define USBHS_PORTSC1_PFSC                              (1 << 24) /* Bit 24: Port force Full-Speed Connect */
#  define USBHS_PORTSC1_PHCD                              (1 << 23) /* Bit 23: PHY low power suspend */
#  define USBHS_PORTSC1_WKOC                              (1 << 22) /* Bit 22: Wake on Over-Current enable */
#  define USBHS_PORTSC1_WKDS                              (1 << 21) /* Bit 21: Wake on Disconnect enable */
#  define USBHS_PORTSC1_WKCN                              (1 << 20) /* Bit 20: Wake on Connect enable */
#define USBHS_PORTSC1_PTC_SHIFT                           (16)      /* Bits 16-19: Port Test Control */
#define USBHS_PORTSC1_PTC_MASK                            (0xf << USBHS_PORTSC1_PTC_SHIFT)
#define USBHS_PORTSC1_PIC_SHIFT                           (14)      /* Bits 14-15: Port Indicator Control */
#define USBHS_PORTSC1_PIC_MASK                            (0x3 << USBHS_PORTSC1_PIC_SHIFT)
#define USBHS_PORTSC1_PO                                  (1 << 13) /* Bit 13: Port Owner */
#define USBHS_PORTSC1_PP                                  (1 << 12) /* Bit 12: Port Power */
#define USBHS_PORTSC1_LS_SHIFT                            (10)      /* Bits 10-11: Line Status */
#define USBHS_PORTSC1_LS_MASK                             (0x3 << USBHS_PORTSC1_LS_SHIFT)
#define USBHS_PORTSC1_HSP                                 (1 << 9)  /* Bit 9:  High Speed Port */
#define USBHS_PORTSC1_PR                                  (1 << 8)  /* Bit 8:  Port Reset */
#define USBHS_PORTSC1_SUSP                                (1 << 7)  /* Bit 7:  Suspend */
#define USBHS_PORTSC1_FPR                                 (1 << 6)  /* Bit 6:  Force Port Resume */
#define USBHS_PORTSC1_OCC                                 (1 << 5)  /* Bit 5:  Over-Current Change */
#define USBHS_PORTSC1_OCA                                 (1 << 4)  /* Bit 4:  Over-current active */
#define USBHS_PORTSC1_PEC                                 (1 << 3)  /* Bit 3:  Port Enable/disable Change */
#define USBHS_PORTSC1_PE                                  (1 << 2)  /* Bit 2:  Port Enabled/disabled */
#define USBHS_PORTSC1_CSC                                 (1 << 1)  /* Bit 1:  Connect Change Status */
#define USBHS_PORTSC1_CCS                                 (1 << 0)  /* Bit 0:  Current Connect Status */

/* On-the-Go Status and Control Register */

                                                                    /* Bit 31: Reserved */
#define USBHS_OTGSC_DPIE                                  (1 << 30) /* Bit 30: Data Pulse Interrupt Enable */
#define USBHS_OTGSC_MSE                                   (1 << 29) /* Bit 29: 1 Milli-Second timer interrupt Enable */
#define USBHS_OTGSC_BSEIE                                 (1 << 28) /* Bit 28: B Session End Interrupt Enable */
#define USBHS_OTGSC_BSVIE                                 (1 << 27) /* Bit 27: B Session Valid Interrupt Enable */
#define USBHS_OTGSC_ASVIE                                 (1 << 26) /* Bit 26: A Session Valid Interrupt Enable */
#define USBHS_OTGSC_AVVIE                                 (1 << 25) /* Bit 25: A VBUS Valid Interrupt Enable */
#define USBHS_OTGSC_IDIE                                  (1 << 24) /* Bit 24: USB ID Interrupt Enable */
                                                                    /* Bit 23: Reserved */
#define USBHS_OTGSC_DPIS                                  (1 << 22) /* Bit 22: Data Pulse interrupt Status */
#define USBHS_OTGSC_MSS                                   (1 << 21) /* Bit 21: 1 Milli-Second timer interrupt Status */
#define USBHS_OTGSC_BSEIS                                 (1 << 20) /* Bit 20: B Session End Interrupt Status */
#define USBHS_OTGSC_BSVIS                                 (1 << 19) /* Bit 19: B Session Valid Interrupt Status */
#define USBHS_OTGSC_ASVIS                                 (1 << 18) /* Bit 18: A Session Valid Interrupt Status */
#define USBHS_OTGSC_AVVIS                                 (1 << 17) /* Bit 17: A VBUS Valid Interrupt Status */
#define USBHS_OTGSC_IDIS                                  (1 << 16) /* Bit 16: USB ID Interrupt Status */
                                                                    /* Bit 15: Reserved */
#define USBHS_OTGSC_DPS                                   (1 << 14) /* Bit 14: Data bus Pulsing Status */
#define USBHS_OTGSC_MST                                   (1 << 13) /* Bit 13: 1 Milli-Second timer Toggle */
#define USBHS_OTGSC_BSE                                   (1 << 12) /* Bit 12: B Session End */
#define USBHS_OTGSC_BSV                                   (1 << 11) /* Bit 11: B Session Valid */
#define USBHS_OTGSC_ASV                                   (1 << 10) /* Bit 10: A Session Valid */
#define USBHS_OTGSC_AVV                                   (1 << 9)  /* Bit 9:  A VBus Valid */
#define USBHS_OTGSC_ID                                    (1 << 8)  /* Bit 8:  USB ID */
#define USBHS_OTGSC_HABA                                  (1 << 7)  /* Bit 7:  Hardware Assist B-Disconnect to A-connect */
                                                                    /* Bit 6:  Reserved */
#define USBHS_OTGSC_IDPU                                  (1 << 5)  /* Bit 5:  ID Pull-Up */
#define USBHS_OTGSC_DP                                    (1 << 4)  /* Bit 4:  Data Pulsing */
#define USBHS_OTGSC_OT                                    (1 << 3)  /* Bit 3:  OTG Termination */
#define USBHS_OTGSC_HAAR                                  (1 << 2)  /* Bit 2:  Hardware Assist Auto-Reset */
#define USBHS_OTGSC_VC                                    (1 << 1)  /* Bit 1:  VBUS Charge */
#define USBHS_OTGSC_VD                                    (1 << 0)  /* Bit 0:  VBUS Discharge */

/* USB Mode Register */

                                                                    /* Bits 15-31: Reserved */
#define USBHS_USBMODE_TXHSD_SHIFT                         (12)      /* Bits 12-14: Tx to Tx HS Delay */
#define USBHS_USBMODE_TXHSD_MASK                          (0x7 << USBHS_USBMODE_TXHSD_SHIFT)
                                                                    /* Bits 5-11: Reserved */
#define USBHS_USBMODE_SDIS                                (1 << 4)  /* Bit 4:  Stream DISable */
#define USBHS_USBMODE_SLOM                                (1 << 3)  /* Bit 3:  Setup Lock-Out Mode */
#define USBHS_USBMODE_ES                                  (1 << 2)  /* Bit 2:  Endian Select */
#define USBHS_USBMODE_CM_SHIFT                            (0)       /* Bits 0-1: Controller Mode */
#define USBHS_USBMODE_CM_MASK                             (0x3 << USBHS_USBMODE_CM_SHIFT)

/* Endpoint Setup Status Register */

                                                                    /* Bits 4_31: Reserved */
#define USBHS_EPSETUPSR_EPSETUPSTAT_SHIFT                 (0)       /* Bits 0-3: Setup Endpoint Status */
#define USBHS_EPSETUPSR_EPSETUPSTAT_MASK                  (0xf << USBHS_EPSETUPSR_EPSETUPSTAT_SHIFT)

/* Endpoint Initialization Register */

                                                                    /* Bits 20-31: Reserved */
#define USBHS_EPPRIME_PETB_SHIFT                          (16)      /* Bits 16-19: Prime Endpoint Tansmit Buffer */
#define USBHS_EPPRIME_PETB_MASK                           (0xf << USBHS_EPPRIME_PETB_SHIFT)
                                                                    /* Bits 4-15: Reserved */
#define USBHS_EPPRIME_PERB_SHIFT                          (0)       /* Bits 0-3: Prime Endpoint Receive Buffer */
#define USBHS_EPPRIME_PERB_MASK                           (0xf << USBHS_EPPRIME_PERB_SHIFT)

/* Endpoint Flush Register */

                                                                    /* Bits 20-31: Reserved */
#define USBHS_EPFLUSH_FETB_SHIFT                           (16)     /* Bits 16-19: Flush Endpoint Transmit Buffer */
#define USBHS_EPFLUSH_FETB_MASK                            (0xf << USBHS_EPFLUSH_FETB_SHIFT)
                                                                    /* Bits 4-15: Reserved */
#define USBHS_EPFLUSH_FERB_SHIFT                           (0)      /* Bits 0-3: Flush Endpoint Receive Buffer */
#define USBHS_EPFLUSH_FERB_MASK                            (0xf << USBHS_EPFLUSH_FERB_SHIFT)

/* Endpoint Status Register */

                                                                    /* Bits 31-20: Reserved */
#define USBHS_EPSR_ETBR_SHIFT                             (16)      /* Bits 16-19: Endpoint Transmit Buffer Ready */
#define USBHS_EPSR_ETBR_MASK                              (0xf << USBHS_EPSR_ETBR_SHIFT)
                                                                    /* Bits 5-14: Reserved */
#define USBHS_EPSR_ERBR_SHIFT                             (0)       /* Bits 0-3: Endpoint Receive Buffer Ready */
#define USBHS_EPSR_ERBR_MASK                              (0xf << USBHS_EPSR_ERBR_SHIFT)

/* Endpoint Complete Register */

                                                                    /* Bits 20-31: Reserved */
#define USBHS_EPCOMPLETE_ETCE_SHIFT                       (16)      /* Bits 16-19: Endpoint Transmit Complete Event */
#define USBHS_EPCOMPLETE_ETCE_MASK                        (0xf << USBHS_EPCOMPLETE_ETCE_SHIFT)
                                                                    /* Bits 4-15: Reserved */
#define USBHS_EPCOMPLETE_ERCE_SHIFT                       (0)       /* Bits 0-3: Endpoint Receive Complete Event */
#define USBHS_EPCOMPLETE_ERCE_MASK                        (0xf << USBHS_EPCOMPLETE_ERCE_SHIFT)

/* Endpoint Control Register 0 */

                                                                    /* Bits 24-31: Undefined */
#define USBHS_EPCR0_TXE                                   (1 << 23) /* Bit 23: TX Endpoint Enable */
                                                                    /* Bits 20-22: Reserved */
#define USBHS_EPCR0_TXT_SHIFT                             (18)      /* Bits 18-19: TX Endpoint Type */
#define USBHS_EPCR0_TXT_MASK                              (0x3 << USBHS_EPCR0_TXT_SHIFT)
                                                                    /* Bit 17: Reserved */
#define USBHS_EPCR0_TXS                                   (1 << 16) /* Bit 16: TX Endpoint Stall */
                                                                    /* Bits 8-15: Reserved */
#define USBHS_EPCR0_RXE                                   (1 << 7)  /* Bit 7:  RX endpoint Enable */
                                                                    /* Bits 4-6: Reserved */
#define USBHS_EPCR0_RXT_SHIFT                             (2)       /* Bits 2-3: RX endpoint Type */
#define USBHS_EPCR0_RXT_MASK                              (0x3 << USBHS_EPCR0_RXT_SHIFT)
                                                                    /* Bit 1:  Reserved */
#define USBHS_EPCR0_RXS                                   (1 << 0)  /* Bit 0:  RX endpoint Stall */

/* Endpoint Control Register n */

                                                                    /* Bits 24-31: Reserved */
#define USBHS_EPCRn_TXE                                   (1 << 23) /* Bit 23: TX endpoint Enable */
#define USBHS_EPCRn_TXR                                   (1 << 22) /* Bit 22: TX data toggle Reset */
#define USBHS_EPCRn_TXI                                   (1 << 21) /* Bit 21: TX data toggle Inhibit */
                                                                    /* Bit 20: Reserved */
#define USBHS_EPCRn_TXT_SHIFT                             (18)      /* Bits 18-19: TX endpoint Type */
#define USBHS_EPCRn_TXT_MASK                              (0x3 << USBHS_EPCRn_TXT_SHIFT)
#define USBHS_EPCRn_TXD                                   (1 << 17) /* Bit 17: TX endpoint Data source */
#define USBHS_EPCRn_TXS                                   (1 << 16) /* Bit 16: TX endpoint Stall */
                                                                    /* Bits 8-15: Reserved */
#define USBHS_EPCRn_RXE                                   (1 << 7)  /* Bit 7:  RX endpoint Enable */
#define USBHS_EPCRn_RXR                                   (1 << 6)  /* Bit 6:  RX data toggle Reset */
#define USBHS_EPCRn_RXI                                   (1 << 5)  /* Bit 5:  RX data toggle Inhibit */
                                                                    /* Bit 4:  Reserved */
#define USBHS_EPCRn_RXT_SHIFT                             (2)       /* Bits 2-3: RX endpoint Type */
#define USBHS_EPCRn_RXT_MASK                              (0x3 << USBHS_EPCRn_RXT_SHIFT)
#define USBHS_EPCRn_RXD                                   (1 << 1)  /* Bit 1:  RX endpoint Data sink */
#define USBHS_EPCRn_RXS                                   (1 << 0)  /* Bit 0:  RX endpoint Stall */

/* USB General Control Register */

                                                                    /* Bits 6-31: Reserved */
#define USBHS_USBGENCTRL_WU_INT_CLR                       (1 << 5)  /* Bit 5:  Wakeup Interrupt Clear */
                                                                    /* Bits 1-4: Reserved */
#define USBHS_USBGENCTRL_WU_IE                            (1 << 0)  /* Bit 0:  Wakeup Interrupt Enable */

/* USB PHY Power-Down Register */

                                                                    /* Bits 21-31: Reserved */
#define USBPHY_PWDn_RXPWDRX                               (1 << 20) /* Bit 20: Auto cleared if USB wakeup while ENAUTOCLR_PHY_PWD */
#define USBPHY_PWDn_RXPWDDIFF                             (1 << 19) /* Bit 19: Auto cleared if USB wakeup while ENAUTOCLR_PHY_PWD */
#define USBPHY_PWDn_RXPWD1PT1                             (1 << 18) /* Bit 18: Auto cleared if USB wakeup while ENAUTOCLR_PHY_PWD */
#define USBPHY_PWDn_RXPWDENV                              (1 << 17) /* Bit 17: Auto cleared if USB wakeup while ENAUTOCLR_PHY_PWD */
                                                                    /* Bits 13-16: Reserved */
#define USBPHY_PWDn_TXPWDV2I                              (1 << 12) /* Bit 12: Auto cleared if USB wakeup while ENAUTOCLR_PHY_PWD */
#define USBPHY_PWDn_TXPWDIBIAS                            (1 << 11) /* Bit 11: Auto cleared if USB wakeup while ENAUTOCLR_PHY_PWD */
#define USBPHY_PWDn_TXPWDFS                               (1 << 10) /* Bit 10: Auto cleared if USB wakeup while ENAUTOCLR_PHY_PWD */
                                                                    /* Bits 0-9: Reserved */

/* USB PHY Transmitter Control Register */

#define USBPHY_TXn_USBPHY_TX_EDGECTRL_SHIFT               (29)      /* Bits 28-26: Edge-rate of the current sensing in HS transmit */
#define USBPHY_TXn_USBPHY_TX_EDGECTRL_MASK                (0x7 << USBPHY_TXn_USBPHY_TX_EDGECTRL_SHIFT)
                                                                    /* Bit 20-25: Reserved */
#define USBPHY_TXn_TXCAL45DP_SHIFT                        (19)      /* Bits 16-19: Trim termination resistance to the USB_DP output */
#define USBPHY_TXn_TXCAL45DP_MASK                         (0xf << USBPHY_TXn_TXCAL45DP_SHIFT)
                                                                    /* Bits 12-15: Reserved */
#define USBPHY_TXn_TXCAL45DM_SHIFT                        (11)      /* Bits 8-11: Trim termination resistance to the USB_DM output */
#define USBPHY_TXn_TXCAL45DM_MASK                         (0xf << USBPHY_TXn_TXCAL45DM_SHIFT)
                                                                    /* Bits 4-7: Reserved */
#define USBPHY_TXn_D_CAL_SHIFT                            (0)       /* Bits 7-0: Trim current source for the High Speed TX drivers */
#define USBPHY_TXn_D_CAL_MASK                             (0x7f << USBPHY_TXn_D_CAL_SHIFT)

/* USB PHY Receiver Control Register */

                                                                    /* Bits 23-31: Reserved */
#define USBPHY_RXn_RXDBYPASS                              (1 << 22) /* Bit 22: Test mode, replace FS differential receiver with DP single ended receiver */
                                                                    /* Bits 7-21: Reserved */
#define USBPHY_RXn_DISCONADJ_SHIFT                        (4)       /* Bits 6-4: Adjusts the trip point for the disconnect detector */
#define USBPHY_RXn_DISCONADJ_MASK                         (0x7 << USBPHY_RXn_DISCONADJ_SHIFT)
                                                                    /* Bit 3: Reserved */
#define USBPHY_RXn_ENVADJ_SHIFT                           (0)       /* Bits 0-3:  Adjusts the trip point for the envelope detector */
#define USBPHY_RXn_ENVADJ_MASK                            (0x7 << USBPHY_RXn_ENVADJ_SHIFT)

/* USB PHY General Control Register */

#define USBPHY_CTRLn_SFTRST                               (1 << 31) /* Bit 31: Soft-reset the USBPHY_PWD, USBPHY_TX, USBPHY_RX, and USBPHY_CTRL */
#define USBPHY_CTRLn_CLKGATE                              (1 << 30) /* Bit 30: Gate UTMI Clocks */
#define USBPHY_CTRLn_UTMI_SUSPENDM                        (1 << 29) /* Bit 29: Indicats powered-down state */
#define USBPHY_CTRLn_HOST_FORCE_LS_SE0                    (1 << 28) /* Bit 28: Forces next FS packet tohave a EOP with low-speed timing */
#define USBPHY_CTRLn_OTG_ID_VALUE                         (1 << 27) /* Bit 27: Indicates the results of USB_ID pin  */
                                                                    /* Bit 25-26: Reserved */
#define USBPHY_CTRLn_FSDLL_RST_EN                         (1 << 24) /* Bit 24: Reset ofthe FSDLL lock detection logic at the end of each TX packet */
                                                                    /* Bit 21-23: Reserved */
#define USBPHY_CTRLn_ENAUTOCLR_PHY_PWD                    (1 << 20) /* Bit 20: Auto-clear the PWD register bits in USBPHY_PWD if wakeup event while suspended */
#define USBPHY_CTRLn_ENAUTOCLR_CLKGATE                    (1 << 19) /* Bit 19: Auto-clear the CLKGATE bit if wakeup event while suspended */
#define USBPHY_CTRLn_AUTORESUME_EN                        (1 << 18) /* Bit 18: Auto resume, HW will send Resume to respond to the device remote wakeup */
                                                                    /* Bit 16-17: Reserved */
#define USBPHY_CTRLn_ENUTMILEVEL3                         (1 << 15) /* Bit 15: Enables UTMI+ Level 3 operation for the USB HS PHY */
#define USBPHY_CTRLn_ENUTMILEVEL2                         (1 << 14) /* Bit 14: Enables UTMI+ Level 2 operation for the USB HS PHY */
                                                                    /* Bit 13: Reserved */
#define USBPHY_CTRLn_DEVPLUGIN_IRQ                        (1 << 12) /* Bit 12: Indicates device is connected */
                                                                    /* Bits 5-11: Reserved */
#define USBPHY_CTRLn_ENDEVPLUGINDET                       (1 << 4)  /* Bit 4:  Enables non-standard resistive plugged-in detection */
#define USBPHY_CTRLn_HOSTDISCONDETECT_IRQ                 (1 << 3)  /* Bit 3:  Indicates that the device has disconnected in High-Speed mode */
                                                                    /* Bit 2:  Reserved */
#define USBPHY_CTRLn_ENHOSTDISCONDETECT                   (1 << 1)  /* Bit 1:  For host mode, enables high-speed disconnect detector */
                                                                    /* Bit 0:  Reserved */

/* USB PHY Status Register */

                                                                    /* Bits 11-31: Reserved */
#define USBPHY_STATUS_RESUME_STATUS                       (1 << 10) /* Bit 10: Indicates that the host is sending a wake-up after Suspend */
                                                                    /* Bit 9:  Reserved */
#define USBPHY_STATUS_OTGID_STATUS                        (1 << 8)  /* Bit 8:  Indicates the results of USB_ID pin on the USB cable */
                                                                    /* Bit 7: Reserved */
#define USBPHY_STATUS_DEVPLUGIN_STATUS                    (1 << 6)  /* Bit 6:  Status indicator for non-standard resistive plugged-in detection */
                                                                    /* Bits 4-5: Reserved */
#define USBPHY_STATUS_HOSTDISCONDETECT_STATUS             (1 << 3)  /* Bit 3:  Indicates at that the remote device has disconnected while in High-Speed mode */
                                                                    /* Bits 0-2: Reserved */

/* USB PHY Debug Register */

                                                                    /* Bit 31: Reserved */
#define USBPHY_DEBUGn_CLKGATE                             (1 << 30) /* Bit 30: Gate Test Clocks */
#define USBPHY_DEBUGn_HOST_RESUME_DEBUG                   (1 << 29) /* Bit 29: Trigger the host resume SE0 with HOST_FORCE_LS_SE0 = 0 or UTMI_SUSPEND=1 */
#define USBPHY_DEBUGn_SQUELCHRESETLENGTH_SHIFT            (25)      /* Bits 25-28: Duration of RESET in terms of the number of 480-MHz cycles */
#define USBPHY_DEBUGn_SQUELCHRESETLENGTH_MASK             (0xf << USBPHY_DEBUGn_SQUELCHRESETLENGTH_SHIFT)
#define USBPHY_DEBUGn_ENSQUELCHRESET                      (1 << 24) /* Bit 24: Set bit to allow squelch to reset high-speed receive */
                                                                    /* Bits 21-23: Reserved */
#define USBPHY_DEBUGn_SQUELCHRESETCOUNT_SHIFT             (16)       /* Bits 16-20: Delay in between the detection of squelch to the reset of high-speed RX */
#define USBPHY_DEBUGn_SQUELCHRESETCOUNT_MASK              (0x1f << USBPHY_DEBUGn_SQUELCHRESETCOUNT_SHIFT)
                                                                    /* Bits 13-15: Reserved */
#define USBPHY_DEBUGn_ENTX2RXCOUNT                        (1 << 12) /* Bit 12: Allow a countdown to transition in between TX and RX */
#define USBPHY_DEBUGn_TX2RXCOUNT_SHIFT                    (8)       /* Bits 8-11: Delay in between the end of transmit to the beginning of receive */
#define USBPHY_DEBUGn_TX2RXCOUNT_MASK                     (0xf << USBPHY_DEBUGn_TX2RXCOUNT_SHIFT)
                                                                    /* Bits 6-7: Reserved */
#define USBPHY_DEBUGn_ENHSTPULLDOWN_SHIFT                 (4)       /* Bits 4-5: Host pulldown overdrive mode */
#define USBPHY_DEBUGn_ENHSTPULLDOWN_MASK                  (0x3 << USBPHY_DEBUGn_ENHSTPULLDOWN_SHIFT)
#define USBPHY_DEBUGn_HSTPULLDOWN_SHIFT                   (2)       /* Bits 2-3: Connect pulldown resistors on USB_DP/USB_DM pins if pulldown overdrive mode enabled */
#define USBPHY_DEBUGn_HSTPULLDOWN_MASK                    (0x3 << USBPHY_DEBUGn_HSTPULLDOWN_SHIFT)
#define USBPHY_DEBUGn_DEBUG_INTERFACE_HOLD                (1 << 1)  /* Bit 1:  Use holding registers to assist in timing for external UTMI interface */
#define USBPHY_DEBUGn_OTGIDPIOLOCK                        (1 << 0)  /* Bit 0:  Hold sampled OTG ID from USBPHY_STATUS_OTGID_STATUS */

/* UTMI Debug Status Register 0 */

#define USBPHY_DEBUG0_STATUS_SQUELCH_COUNT_SHIFT            (26)    /* Bits 26-31: Running count of the squelch reset instead of normal end for HS RX */
#define USBPHY_DEBUG0_STATUS_SQUELCH_COUNT_MASK             (0x3f << USBPHY_DEBUG0_STATUS_SQUELCH_COUNT_SHIFT)
#define USBPHY_DEBUG0_STATUS_UTMI_RXERROR_FAIL_COUNT_SHIFT  (16)    /* Bits 25-16: Running count of the UTMI_RXERROR */
#define USBPHY_DEBUG0_STATUS_UTMI_RXERROR_FAIL_COUNT_MASK   (0x3ff << USBPHY_DEBUG0_STATUS_UTMI_RXERROR_FAIL_COUNT_SHIFT)
#define USBPHY_DEBUG0_STATUS_LOOP_BACK_FAIL_COUNT_SHIFT     (0)     /* Bits 0-15: Running count of the failed pseudo-random generator loopback */
#define USBPHY_DEBUG0_STATUS_LOOP_BACK_FAIL_COUNT_MASK      (0xffff << USBPHY_DEBUG0_STATUS_LOOP_BACK_FAIL_COUNT_SHIFT)

/* UTMI Debug Status Register 1 */

                                                                    /* Bits 15-31: Reserved */
#define USBPHY_DEBUG1n_ENTAILADJVD_SHIFT                  (13)      /* Bits 14-13: Delay increment of the rise of squelch */
#define USBPHY_DEBUG1n_ENTAILADJVD_MASK                   (0x3 << USBPHY_DEBUG1n_ENTAILADJVD_SHIFT)
                                                                    /* Bits 0-12: Reserved */

/* UTMI RTL Version */

#define USBPHY_VERSION_MAJOR_SHIFT                        (24)       /* Bits 24-31: Fixed read-only value reflecting the MAJOR field of the RTL version */
#define USBPHY_VERSION_MAJOR_MASK                         (0xff << USBPHY_VERSION_MAJOR_SHIFT)
#define USBPHY_VERSION_MINOR_SHIFT                        (16)       /* Bits 16-23: Fixed read-only value reflecting the MINOR field of the RTL version */
#define USBPHY_VERSION_MINOR_MASK                         (0xff << USBPHY_VERSION_MINOR_SHIFT)
#define USBPHY_VERSION_STEP_SHIFT                         (0)       /* Bits 0-15: Fixed read-only value reflecting the stepping of the RTL version */
#define USBPHY_VERSION_STEP_MASK                          (0xffff << USBPHY_VERSION_STEP_SHIFT)

/* USB PHY PLL Control/Status Register */

#define USBPHY_PLL_SICn_PLL_LOCK                          (1 << 31) /* Bit 31: USB PLL lock status indicator */
                                                                    /* Bits 17-30: Reserved */
#define USBPHY_PLL_SICn_PLL_BYPASS                        (1 << 16) /* Bit 16: Bypass the USB PLL */
                                                                    /* Bits 14-15: Reserved */
#define USBPHY_PLL_SICn_PLL_ENABLE                        (1 << 13) /* Bit 13: Enable the clock output from the USB PLL */
#define USBPHY_PLL_SICn_PLL_POWER                         (1 << 12) /* Bit 12: Power up the USB PLL */
#define USBPHY_PLL_SICn_PLL_HOLD_RING_OFF                 (1 << 11) /* Bit 11: Analog debug bit */
                                                                    /* Bit 10: Reserved */
#define USBPHY_PLL_SICn_PLL_EN_USB_CLKS                   (1 << 9)  /* Bit 6:  Enable the USB clock output from the USB PHY PLL */
                                                                    /* Bits 2-5: Reserved */
#define USBPHY_PLL_SICn_PLL_DIV_SEL_SHIFT                 (0)       /* Bits 0-4: Controls the USB PLL feedback loop divider */
#define USBPHY_PLL_SICn_PLL_DIV_SEL_MASK                  (0x1f << USBPHY_PLL_SICn_PLL_DIV_SEL_SHIFT)

/* USB PHY VBUS Detect Control Register */

#define USBPHY_USB1_VBUS_DETECTn_EN_CHARGER_RESISTOR      (1 << 31) /* Bit 31: Enables resistors used for an older method of resistive battery charger detection */
                                                                    /* Bits 27-30: Reserved */
#define USBPHY_USB1_VBUS_DETECTn_DISCHARGE_VBUS           (1 << 26) /* Bit 26: Controls VBUS discharge resistor */
                                                                    /* Bits 21-25: Reserved */
#define USBPHY_USB1_VBUS_DETECTn_PWRUP_CMPS               (1 << 20) /* Bit 20: Enables the VBUS_VALID comparator */
                                                                    /* Bit 19: Reserved */
#define USBPHY_USB1_VBUS_DETECTn_VBUSVALID_TO_SESSVALID   (1 << 18) /* Bit 18: Selects the comparator used for VBUS_VALID */
                                                                    /* Bits 11-17: Reserved */
#define USBPHY_USB1_VBUS_DETECTn_VBUS_SOURCE_SEL_SHIFT    (9)       /* Bits 9-10: Selects the source of the VBUS_VALID signal reported to the USB controller */
#define USBPHY_USB1_VBUS_DETECTn_VBUS_SOURCE_SEL_MASK     (0x3 << USBPHY_USB1_VBUS_DETECTn_VBUS_SOURCE_SEL_SHIFT)
#define USBPHY_USB1_VBUS_DETECTn_VBUSVALID_SEL            (1 << 8)  /* Bit 8:  Selects the source of the VBUS_VALID signal reported to the USB controller */
#define USBPHY_USB1_VBUS_DETECTn_VBUSVALID_OVERRIDE       (1 << 7)  /* Bit 7:  Override value for VBUS_VALID signal sent to USB controller */
#define USBPHY_USB1_VBUS_DETECTn_AVALID_OVERRIDE          (1 << 6)  /* Bit 6:  Override value for A-Device Session Valid */
#define USBPHY_USB1_VBUS_DETECTn_BVALID_OVERRIDE          (1 << 5)  /* Bit 5:  Override value for B-Device Session Valid */
#define USBPHY_USB1_VBUS_DETECTn_SESSEND_OVERRIDE         (1 << 4)  /* Bit 4:  Override value for SESSEND */
#define USBPHY_USB1_VBUS_DETECTn_VBUS_OVERRIDE_EN         (1 << 3)  /* Bit 3:  VBUS detect signal override enable */
#define USBPHY_USB1_VBUS_DETECTn_VBUSVALID_THRESH_SHIFT   (0)       /* Bits 0-2: Sets the threshold for the VBUSVALID comparator */
#define USBPHY_USB1_VBUS_DETECTn_VBUSVALID_THRESH_MASK    (0x7 << USBPHY_USB1_VBUS_DETECTn_VBUSVALID_THRESH_SHIFT)

/* USB PHY VBUS Detector Status Register */

                                                                    /* Bits 5-31: Reserved */
#define USBPHY_USB1_VBUS_DET_STAT_VBUS_VALID_3V           (1 << 4)  /* Bit 4:  VBUS_VALID_3V detector status */
#define USBPHY_USB1_VBUS_DET_STAT_VBUS_VALID              (1 << 3)  /* Bit 3:  VBUS voltage status */
#define USBPHY_USB1_VBUS_DET_STAT_AVALID                  (1 << 2)  /* Bit 2:  A-Device Session Valid status */
#define USBPHY_USB1_VBUS_DET_STAT_BVALID                  (1 << 1)  /* Bit 1:  B-Device Session Valid status */
#define USBPHY_USB1_VBUS_DET_STAT_SESSEND                 (1 << 0)  /* Bit 0:  Session End indicator */

/* USB PHY Charger Detect Status Register */

                                                                    /* Bits 5-31: Reserved */
#define USBPHY_USB1_CHRG_DET_STAT_SECDET_DCP              (1 << 4)  /* Bit 4:  Battery Charging Secondary Detection phase output */
#define USBPHY_USB1_CHRG_DET_STAT_DP_STATE                (1 << 3)  /* Bit 3:  Single ended receiver output for the USB_DP pin, from charger detection circuits */
#define USBPHY_USB1_CHRG_DET_STAT_DM_STATE                (1 << 2)  /* Bit 2:  Single ended receiver output for the USB_DM pin, from charger detection circuits */
#define USBPHY_USB1_CHRG_DET_STAT_CHRG_DETECTED           (1 << 1)  /* Bit 1:  Battery Charging Primary Detection phase output */
#define USBPHY_USB1_CHRG_DET_STAT_PLUG_CONTACT            (1 << 0)  /* Bit 0:  Battery Charging Data Contact Detection phase output */

/* USB PHY Analog Control Register */

#define USBPHY_ANACTRLn_PFD_STABLE                        (1 << 31) /* Bit 31: PFD stable signal from the Phase Fractional Divider */
                                                                    /* Bits 16-30: Reserved */
#define USBPHY_ANACTRLn_EMPH_CUR_CTRL_SHIFT               (14)      /* Bits 14-15: Pre-emphasis current added for the High-Speed TX drivers */
#define USBPHY_ANACTRLn_EMPH_CUR_CTRL_MASK                (0x3 << USBPHY_ANACTRLn_EMPH_CUR_CTRL_SHIFT)
#define USBPHY_ANACTRLn_EMPH_EN                           (1 << 13) /* Bit 13: Enables pre-emphasis for the High-Speed TX drivers */
#define USBPHY_ANACTRLn_EMPH_PULSE_CTRL_SHIFT             (11)      /* Bits 11-12: Controls pre-emphasis time duration for High Speed TX drivers */
#define USBPHY_ANACTRLn_EMPH_PULSE_CTRL_MASK              (0x3 << USBPHY_ANACTRLn_EMPH_PULSE_CTRL_SHIFT)
#define USBPHY_ANACTRLn_DEV_PULLDOWN                      (1 << 10) /* Bit 10: Enable the pulldown resistors on both USB_DP and USB_DM pins */
#define USBPHY_ANACTRLn_PFD_FRAC_SHIFT                    (4)       /* Bits 4-9: PFD fractional divider used to select the pfd_clk output frequency */
#define USBPHY_ANACTRLn_PFD_FRAC_MASK                     (0x3f << USBPHY_ANACTRLn_PFD_FRAC_SHIFT)
#define USBPHY_ANACTRLn_PFD_CLK_SEL_SHIFT                 (2)       /* Bits 2-3: Selects the frequency relationship between pfd_clk output and exported USB1PFDCLK */
#define USBPHY_ANACTRLn_PFD_CLK_SEL_MASK                  (0x3 << USBPHY_ANACTRLn_PFD_CLK_SEL_SHIFT)
#define USBPHY_ANACTRLn_PFD_CLKGATE                       (1 << 1)  /* Bit 1:  Clock gating (disabling) for the PFD */
#define USBPHY_ANACTRLn_TESTCLK_SEL                       (1 << 0)  /* Bit 0:  Test clock selection to analog test */

/* USB PHY Loopback Control/Status Register */

                                                                    /* Bits 24-31: Reserved */
#define USBPHY_USB1_LOOPBACKn_TSTPKT_SHIFT                (16)      /* Bits 16-23: Selects the packet data byte used for USB loopback testing in Pulse mode */
#define USBPHY_USB1_LOOPBACKn_TSTPKT_MASK                 (0xff << USBPHY_USB1_LOOPBACKn_TSTPKT_SHIFT)
#define USBPHY_USB1_LOOPBACKn_TSTI_HSFS_MODE_EN           (1 << 15) /* Bit 15: Enable the loopback test to dynamically change the packet speed */
                                                                    /* Bits 9-14: Reserved */
#define USBPHY_USB1_LOOPBACKn_UTMO_DIG_TST1               (1 << 8)  /* Bit 8:  Status bit for USB loopback test */
#define USBPHY_USB1_LOOPBACKn_UTMO_DIG_TST0               (1 << 7)  /* Bit 7:  Status bit for USB loopback test */
#define USBPHY_USB1_LOOPBACKn_TSTI_TX_HIZ                 (1 << 6)  /* Bit 6:  Sets TX Hi-Z for USB loopback test */
#define USBPHY_USB1_LOOPBACKn_TSTI_TX_EN                  (1 << 5)  /* Bit 5:  Enable TX for USB loopback test */
#define USBPHY_USB1_LOOPBACKn_TSTI_TX_LS_MODE             (1 << 4)  /* Bit 4:  Choose LS/FS/HS for USB loopback testing */
#define USBPHY_USB1_LOOPBACKn_TSTI_TX_HS_MODE             (1 << 3)  /* Bit 3:  Select HS or FS mode for USB loopback testing */
#define USBPHY_USB1_LOOPBACKn_UTMI_DIG_TST1               (1 << 2)  /* Bit 2:  Mode control for USB loopback test */
#define USBPHY_USB1_LOOPBACKn_UTMI_DIG_TST0               (1 << 1)  /* Bit 1:  Mode control for USB loopback test */
#define USBPHY_USB1_LOOPBACKn_UTMI_TESTSTART              (1 << 0)  /* Bit 0:  This bit enables the USB loopback test */

/* USB PHY Loopback Packet Number Select Register */

#define USBPHY_USB1_LOOPBACK_HSFSCNTn_TSTI_FS_NUMBER_SHIFT  (16)       /* Bits 16-31: Full speed packet number */
#define USBPHY_USB1_LOOPBACK_HSFSCNTn_TSTI_FS_NUMBER_MASK   (0xffff << USBPHY_USB1_LOOPBACK_HSFSCNTn_TSTI_FS_NUMBER_SHIFT)
#define USBPHY_USB1_LOOPBACK_HSFSCNTn_TSTI_HS_NUMBER_SHIFT  (0)       /* Bits 0-15: High speed packet number */
#define USBPHY_USB1_LOOPBACK_HSFSCNTn_TSTI_HS_NUMBER_MASK   (0xffff << USBPHY_USB1_LOOPBACK_HSFSCNTn_TSTI_HS_NUMBER_SHIFT)

/* USB PHY Trim Override Enable Register */

#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_CAL45DM_SHIFT       (28)      /* Bits 28-31: IFR value of TX_CAL45DM */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_CAL45DM_MASK        (0xf << USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_CAL45DM_SHIFT)
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_CAL45DP_SHIFT       (24)      /* Bits 24-27: IFR value of TX_CAL45DP */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_CAL45DP_MASK        (0xf << USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_CAL45DP_SHIFT)
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_D_CAL_SHIFT         (20)      /* Bits 22-30: IFR value of TX_D_CAL */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_D_CAL_MASK          (0xf << USBPHY_TRIM_OVERRIDE_ENn_TRIM_USBPHY_TX_D_CAL_SHIFT)
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USB_REG_ENV_TAIL_ADJ_VD_SHIFT (18)      /* Bits 18-19: IFR value of ENV_TAIL_ADJ */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_USB_REG_ENV_TAIL_ADJ_VD_MASK  (0x3 << USBPHY_TRIM_OVERRIDE_ENn_TRIM_USB_REG_ENV_TAIL_ADJ_VD_SHIFT)
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_PLL_CTRL0_DIV_SEL_SHIFT       (16)      /* Bits 16-17: IFR value of PLL_DIV_SEL */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_PLL_CTRL0_DIV_SEL_MASK        (0x3 << USBPHY_TRIM_OVERRIDE_ENn_TRIM_PLL_CTRL0_DIV_SEL_SHIFT)
                                                                              /* Bits 5-15: Reserved */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_TX_CAL45DM_OVERRIDE           (1 << 4)  /* Bit 4:  Override enable for TX_CAL45DM */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_TX_CAL45DP_OVERRIDE           (1 << 3)  /* Bit 3:  Override enable for TX_CAL45DP */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_TX_D_CAL_OVERRIDE             (1 << 2)  /* Bit 2:  Override enable for TX_D_CAL */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_ENV_TAIL_ADJ_VD_OVERRIDE      (1 << 1)  /* Bit 1:  Override enable for ENV_TAIL_ADJ */
#define USBPHY_TRIM_OVERRIDE_ENn_TRIM_DIV_SEL_OVERRIDE              (1 << 0)  /* Bit 0:  Override enable for PLL_DIV_SEL */

#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_USBHS_H */
