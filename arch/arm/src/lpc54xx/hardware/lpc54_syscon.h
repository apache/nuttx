/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_syscon.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SYSCON_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SYSCON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Main system configuration */

#define LPC54_SYSCON_AHBMATPRIO_OFFSET         0x0010  /* AHB multilayer matrix priority control */
#define LPC54_SYSCON_SYSTCKCAL_OFFSET          0x0040  /* System tick counter calibration */
#define LPC54_SYSCON_NMISRC_OFFSET             0x0048  /* NMI source select */
#define LPC54_SYSCON_ASYNCAPBCTRL_OFFSET       0x004c  /* Asynchronous APB control */
#define LPC54_SYSCON_PIOPORCAP0_OFFSET         0x00c0  /* POR captured value of port 0 */
#define LPC54_SYSCON_PIOPORCAP1_OFFSET         0x00c4  /* POR captured value of port 1 */
#define LPC54_SYSCON_PIORESCAP0_OFFSET         0x00d0  /* Reset captured value of port 0 */
#define LPC54_SYSCON_PIORESCAP1_OFFSET         0x00d4  /* Reset captured value of port 1 */
#define LPC54_SYSCON_PRESETCTRL0_OFFSET        0x0100  /* Peripheral reset control 0 */
#define LPC54_SYSCON_PRESETCTRL1_OFFSET        0x0104  /* Peripheral reset control 1 */
#define LPC54_SYSCON_PRESETCTRL2_OFFSET        0x0108  /* Peripheral reset control 2 */
#define LPC54_SYSCON_PRESETCTRLSET0_OFFSET     0x0120  /* Set bits in PRESETCTRL0 */
#define LPC54_SYSCON_PRESETCTRLSET1_OFFSET     0x0124  /* Set bits in PRESETCTRL1 */
#define LPC54_SYSCON_PRESETCTRLSET2_OFFSET     0x0128  /* Set bits in PRESETCTRL2 */
#define LPC54_SYSCON_PRESETCTRLCLR0_OFFSET     0x0140  /* Clear bits in PRESETCTRL0 */
#define LPC54_SYSCON_PRESETCTRLCLR1_OFFSET     0x0144  /* Clear bits in PRESETCTRL1 */
#define LPC54_SYSCON_PRESETCTRLCLR2_OFFSET     0x0148  /* Clear bits in PRESETCTRL2 */
#define LPC54_SYSCON_SYSRSTSTAT_OFFSET         0x01f0  /* System reset status register */
#define LPC54_SYSCON_AHBCLKCTRL0_OFFSET        0x0200  /* AHB Clock control 0 */
#define LPC54_SYSCON_AHBCLKCTRL1_OFFSET        0x0204  /* AHB Clock control 1 */
#define LPC54_SYSCON_AHBCLKCTRL2_OFFSET        0x0208  /* AHB Clock control 2 */
#define LPC54_SYSCON_AHBCLKCTRLSET0_OFFSET     0x0220  /* Set bits in AHBCLKCTRL0 */
#define LPC54_SYSCON_AHBCLKCTRLSET1_OFFSET     0x0224  /* Set bits in AHBCLKCTRL1 */
#define LPC54_SYSCON_AHBCLKCTRLSET2_OFFSET     0x0228  /* Set bits in AHBCLKCTRL2 */
#define LPC54_SYSCON_AHBCLKCTRLCLR0_OFFSET     0x0240  /* Clear bits in AHBCLKCTRL0 */
#define LPC54_SYSCON_AHBCLKCTRLCLR1_OFFSET     0x0244  /* Clear bits in AHBCLKCTRL1 */
#define LPC54_SYSCON_AHBCLKCTRLCLR2_OFFSET     0x0248  /* Clear bits in AHBCLKCTRL2 */
#define LPC54_SYSCON_MAINCLKSELA_OFFSET        0x0280  /* Main clock source select A */
#define LPC54_SYSCON_MAINCLKSELB_OFFSET        0x0284  /* Main clock source select B */
#define LPC54_SYSCON_CLKOUTSELA_OFFSET         0x0288  /* CLKOUT clock source select */
#define LPC54_SYSCON_SYSPLLCLKSEL_OFFSET       0x0290  /* PLL clock source select */
#define LPC54_SYSCON_AUDPLLCLKSEL_OFFSET       0x0298  /* Audio PLL clock source select */
#define LPC54_SYSCON_SPIFICLKSEL_OFFSET        0x02a0  /* SPIFI clock source select */
#define LPC54_SYSCON_ADCCLKSEL_OFFSET          0x02a4  /* ADC clock source select */
#define LPC54_SYSCON_USB0CLKSEL_OFFSET         0x02a8  /* USB0 clock source select */
#define LPC54_SYSCON_USB1CLKSEL_OFFSET         0x02ac  /* USB1 clock source select */
#define LPC54_SYSCON_FCLKSEL0_OFFSET           0x02b0  /* Flexcomm Interface 0 clock source select */
#define LPC54_SYSCON_FCLKSEL1_OFFSET           0x02b4  /* Flexcomm Interface 1 clock source select */
#define LPC54_SYSCON_FCLKSEL2_OFFSET           0x02b8  /* Flexcomm Interface 2 clock source select */
#define LPC54_SYSCON_FCLKSEL3_OFFSET           0x02bc  /* Flexcomm Interface 3 clock source select */
#define LPC54_SYSCON_FCLKSEL4_OFFSET           0x02c0  /* Flexcomm Interface 4 clock source select */
#define LPC54_SYSCON_FCLKSEL5_OFFSET           0x02c4  /* Flexcomm Interface 5 clock source select */
#define LPC54_SYSCON_FCLKSEL6_OFFSET           0x02c8  /* Flexcomm Interface 6 clock source select */
#define LPC54_SYSCON_FCLKSEL7_OFFSET           0x02cc  /* Flexcomm Interface 7 clock source select */
#define LPC54_SYSCON_FCLKSEL8_OFFSET           0x02d0  /* Flexcomm Interface 8 clock source select */
#define LPC54_SYSCON_FCLKSEL9_OFFSET           0x02d4  /* Flexcomm Interface 9 clock source select */
#define LPC54_SYSCON_MCLKCLKSEL_OFFSET         0x02e0  /* MCLK clock source select */
#define LPC54_SYSCON_FRGCLKSEL_OFFSET          0x02e8  /* Fractional Rate Generator clock source select */
#define LPC54_SYSCON_DMICCLKSEL_OFFSET         0x02ec  /* Digital microphone (DMIC) subsystem clock select */
#define LPC54_SYSCON_SCTCLKSEL_OFFSET          0x02f0  /* SCTimer/PWM clock source select */
#define LPC54_SYSCON_LCDCLKSEL_OFFSET          0x02f4  /* LCD clock source select */
#define LPC54_SYSCON_SDIOCLKSEL_OFFSET         0x02f8  /* SDIO clock source select */
#define LPC54_SYSCON_SYSTICKCLKDIV_OFFSET      0x0300  /* SYSTICK clock divider */
#define LPC54_SYSCON_ARMTRCLKDIV_OFFSET        0x0304  /* ARM Trace clock divider */
#define LPC54_SYSCON_CAN0CLKDIV_OFFSET         0x0308  /* MCAN0 clock divider */
#define LPC54_SYSCON_CAN1CLKDIV_OFFSET         0x030c  /* MCAN1 clock divider */
#define LPC54_SYSCON_SC0CLKDIV_OFFSET          0x0310  /* Smartcard0 clock divider */
#define LPC54_SYSCON_SC1CLKDIV_OFFSET          0x0314  /* Smartcard1 clock divider */
#define LPC54_SYSCON_AHBCLKDIV_OFFSET          0x0380  /* System clock divider */
#define LPC54_SYSCON_CLKOUTDIV_OFFSET          0x0384  /* CLKOUT clock divider */
#define LPC54_SYSCON_FROHFDIV_OFFSET           0x0388  /* FROHF clock divider */
#define LPC54_SYSCON_SPIFICLKDIV_OFFSET        0x0390  /* SPIFI clock divider */
#define LPC54_SYSCON_ADCCLKDIV_OFFSET          0x0394  /* ADC clock divider */
#define LPC54_SYSCON_USB0CLKDIV_OFFSET         0x0398  /* USB0 clock divider */
#define LPC54_SYSCON_USB1CLKDIV_OFFSET         0x039c  /* USB1 clock divider */
#define LPC54_SYSCON_FRGCTRL_OFFSET            0x03a0  /* Fractional rate divider */
#define LPC54_SYSCON_DMICCLKDIV_OFFSET         0x03a8  /* DMIC clock divider */
#define LPC54_SYSCON_MCLKDIV_OFFSET            0x03ac  /* I2S MCLK clock divider */
#define LPC54_SYSCON_LCDCLKDIV_OFFSET          0x03b0  /* LCD clock divider */
#define LPC54_SYSCON_SCTCLKDIV_OFFSET          0x03b4  /* SCT/PWM clock divider */
#define LPC54_SYSCON_EMCCLKDIV_OFFSET          0x03b8  /* EMC clock divider */
#define LPC54_SYSCON_SDIOCLKDIV_OFFSET         0x03bc  /* SDIO clock divider */
#define LPC54_SYSCON_FLASHCFG_OFFSET           0x0400  /* Flash wait states configuration */
#define LPC54_SYSCON_USB0CLKCTRL_OFFSET        0x040c  /* USB0 clock control */
#define LPC54_SYSCON_USB0CLKSTAT_OFFSET        0x0410  /* USB0 clock status */
#define LPC54_SYSCON_FREQMECTRL_OFFSET         0x0418  /* Frequency measure register */
#define LPC54_SYSCON_MCLKIO_OFFSET             0x0420  /* MCLK input/output control */
#define LPC54_SYSCON_USB1CLKCTRL_OFFSET        0x0424  /* USB1 clock control */
#define LPC54_SYSCON_USB1CLKSTAT_OFFSET        0x0428  /* USB1 clock status */
#define LPC54_SYSCON_EMCSYSCTRL_OFFSET         0x0444  /* EMC system control */
#define LPC54_SYSCON_EMCDLYCTRL_OFFSET         0x0448  /* EMC clock delay control */
#define LPC54_SYSCON_EMCDLYCAL_OFFSET          0x044c  /* EMC delay chain calibration control */
#define LPC54_SYSCON_ETHPHYSEL_OFFSET          0x0450  /* Ethernet PHY selection */
#define LPC54_SYSCON_ETHSBDCTRL_OFFSET         0x0454  /* Ethernet SBD flow control */
#define LPC54_SYSCON_SDIOCLKCTRL_OFFSET        0x0460  /* SDIO CCLKIN phase and delay control */
#define LPC54_SYSCON_FROCTRL_OFFSET            0x0500  /* FRO oscillator control */
#define LPC54_SYSCON_SYSOSCCTRL_OFFSET         0x0504  /* System oscillator control */
#define LPC54_SYSCON_WDTOSCCTRL_OFFSET         0x0508  /* Watchdog oscillator control */
#define LPC54_SYSCON_RTCOSCCTRL_OFFSET         0x050c  /* RTC oscillator 32 kHz output control */
#define LPC54_SYSCON_USBPLLCTRL_OFFSET         0x051c  /* USB PLL control */
#define LPC54_SYSCON_USBPLLSTAT_OFFSET         0x0520  /* USB PLL status */
#define LPC54_SYSCON_SYSPLLCTRL_OFFSET         0x0580  /* System PLL control */
#define LPC54_SYSCON_SYSPLLSTAT_OFFSET         0x0584  /* PLL status */
#define LPC54_SYSCON_SYSPLLNDEC_OFFSET         0x0588  /* PLL N divider */
#define LPC54_SYSCON_SYSPLLPDEC_OFFSET         0x058c  /* PLL P divider */
#define LPC54_SYSCON_SYSPLLMDEC_OFFSET         0x0590  /* System PLL M divider */
#define LPC54_SYSCON_AUDPLLCTRL_OFFSET         0x05a0  /* Audio PLL control */
#define LPC54_SYSCON_AUDPLLSTAT_OFFSET         0x05a4  /* Audio PLL status */
#define LPC54_SYSCON_AUDPLLNDEC_OFFSET         0x05a8  /* Audio PLL N divider */
#define LPC54_SYSCON_AUDPLLPDEC_OFFSET         0x05ac  /* Audio PLL P divider */
#define LPC54_SYSCON_AUDPLLMDEC_OFFSET         0x05b0  /* Audio PLL M divider */
#define LPC54_SYSCON_AUDPLLFRAC_OFFSET         0x05b4  /* Audio PLL fractional divider control */
#define LPC54_SYSCON_PDSLEEPCFG0_OFFSET        0x0600  /* Sleep configuration register 0 */
#define LPC54_SYSCON_PDSLEEPCFG1_OFFSET        0x0604  /* Sleep configuration register 1 */
#define LPC54_SYSCON_PDRUNCFG0_OFFSET          0x0610  /* Power configuration register 0 */
#define LPC54_SYSCON_PDRUNCFG1_OFFSET          0x0614  /* Power configuration register 1 */
#define LPC54_SYSCON_PDRUNCFGSET0_OFFSET       0x0620  /* Set bits in PDRUNCFG0 */
#define LPC54_SYSCON_PDRUNCFGSET1_OFFSET       0x0624  /* Set bits in PDRUNCFG1 */
#define LPC54_SYSCON_PDRUNCFGCLR0_OFFSET       0x0630  /* Clear bits in PDRUNCFG0 */
#define LPC54_SYSCON_PDRUNCFGCLR1_OFFSET       0x0634  /* Clear bits in PDRUNCFG1 */
#define LPC54_SYSCON_STARTER0_OFFSET           0x0680  /* Start logic 0 wake-up enable register */
#define LPC54_SYSCON_STARTER1_OFFSET           0x0684  /* Start logic 1 wake-up enable register */
#define LPC54_SYSCON_STARTERSET0_OFFSET        0x06a0  /* Set bits in STARTER0 */
#define LPC54_SYSCON_STARTERSET1_OFFSET        0x06a4  /* Set bits in STARTER1 */
#define LPC54_SYSCON_STARTERCLR0_OFFSET        0x06c0  /* Clear bits in STARTER0 */
#define LPC54_SYSCON_STARTERCLR1_OFFSET        0x06c4  /* Clear bits in STARTER1 */
#define LPC54_SYSCON_HWWAKE_OFFSET             0x0780  /* Configures special cases of hardware wake-up */
#define LPC54_SYSCON_AUTOCGOR_OFFSET           0x0e04  /* Auto clock-gate override */
#define LPC54_SYSCON_JTAGIDCODE_OFFSET         0x0ff4  /* JTAG ID code */
#define LPC54_SYSCON_DEVICE_ID0_OFFSET         0x0ff8  /* Part ID */
#define LPC54_SYSCON_DEVICE_ID1_OFFSET         0x0ffc  /* Boot ROM and die revision */

/* Asynchronous system configuration */

#define LPC54_SYSCON_ASYNCPRESETCTRL_OFFSET    0x0000  /* Async peripheral reset control */
#define LPC54_SYSCON_ASYNCPRESETCTRLSET_OFFSET 0x0004  /* Set bits in ASYNCPRESETCTRL */
#define LPC54_SYSCON_ASYNCPRESETCTRLCLR_OFFSET 0x0008  /* Clear bits in ASYNCPRESETCTRL */
#define LPC54_SYSCON_ASYNCAPBCLKCTRL_OFFSET    0x0010  /* Async peripheral clock control */
#define LPC54_SYSCON_ASYNCAPBCLKCTRLSET_OFFSET 0x0014  /* Set bits in ASYNCAPBCLKCTRL  */
#define LPC54_SYSCON_ASYNCAPBCLKCTRLCLR_OFFSET 0x0018  /* Clear bits in ASYNCAPBCLKCTRL */
#define LPC54_SYSCON_ASYNCAPBCLKSELA_OFFSET    0x0020  /* Async APB clock source select A */

/* Other system configuration */

#define LPC54_SYSCON_BODCTRL_OFFSET            0x0044  /* Brown-Out Detect control */

/* Register addresses *******************************************************/

/* Main system configuration */

#define LPC54_SYSCON_AHBMATPRIO                (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBMATPRIO_OFFSET)
#define LPC54_SYSCON_SYSTCKCAL                 (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSTCKCAL_OFFSET)
#define LPC54_SYSCON_NMISRC                    (LPC54_SYSCON_BASE + LPC54_SYSCON_NMISRC_OFFSET)
#define LPC54_SYSCON_ASYNCAPBCTRL              (LPC54_SYSCON_BASE + LPC54_SYSCON_ASYNCAPBCTRL_OFFSET)
#define LPC54_SYSCON_PIOPORCAP0                (LPC54_SYSCON_BASE + LPC54_SYSCON_PIOPORCAP0_OFFSET)
#define LPC54_SYSCON_PIOPORCAP1                (LPC54_SYSCON_BASE + LPC54_SYSCON_PIOPORCAP1_OFFSET)
#define LPC54_SYSCON_PIORESCAP0                (LPC54_SYSCON_BASE + LPC54_SYSCON_PIORESCAP0_OFFSET)
#define LPC54_SYSCON_PIORESCAP1                (LPC54_SYSCON_BASE + LPC54_SYSCON_PIORESCAP1_OFFSET)
#define LPC54_SYSCON_PRESETCTRL0               (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRL0_OFFSET)
#define LPC54_SYSCON_PRESETCTRL1               (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRL1_OFFSET)
#define LPC54_SYSCON_PRESETCTRL2               (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRL2_OFFSET)
#define LPC54_SYSCON_PRESETCTRLSET0            (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRLSET0_OFFSET)
#define LPC54_SYSCON_PRESETCTRLSET1            (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRLSET1_OFFSET)
#define LPC54_SYSCON_PRESETCTRLSET2            (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRLSET2_OFFSET)
#define LPC54_SYSCON_PRESETCTRLCLR0            (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRLCLR0_OFFSET)
#define LPC54_SYSCON_PRESETCTRLCLR1            (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRLCLR1_OFFSET)
#define LPC54_SYSCON_PRESETCTRLCLR2            (LPC54_SYSCON_BASE + LPC54_SYSCON_PRESETCTRLCLR2_OFFSET)
#define LPC54_SYSCON_SYSRSTSTAT                (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSRSTSTAT_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRL0               (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRL0_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRL1               (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRL1_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRL2               (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRL2_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRLSET0            (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRLSET0_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRLSET1            (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRLSET1_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRLSET2            (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRLSET2_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRLCLR0            (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRLCLR0_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRLCLR1            (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRLCLR1_OFFSET)
#define LPC54_SYSCON_AHBCLKCTRLCLR2            (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKCTRLCLR2_OFFSET)
#define LPC54_SYSCON_MAINCLKSELA               (LPC54_SYSCON_BASE + LPC54_SYSCON_MAINCLKSELA_OFFSET)
#define LPC54_SYSCON_MAINCLKSELB               (LPC54_SYSCON_BASE + LPC54_SYSCON_MAINCLKSELB_OFFSET)
#define LPC54_SYSCON_CLKOUTSELA                (LPC54_SYSCON_BASE + LPC54_SYSCON_CLKOUTSELA_OFFSET)
#define LPC54_SYSCON_SYSPLLCLKSEL              (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSPLLCLKSEL_OFFSET)
#define LPC54_SYSCON_AUDPLLCLKSEL              (LPC54_SYSCON_BASE + LPC54_SYSCON_AUDPLLCLKSEL_OFFSET)
#define LPC54_SYSCON_SPIFICLKSEL               (LPC54_SYSCON_BASE + LPC54_SYSCON_SPIFICLKSEL_OFFSET)
#define LPC54_SYSCON_ADCCLKSEL                 (LPC54_SYSCON_BASE + LPC54_SYSCON_ADCCLKSEL_OFFSET)
#define LPC54_SYSCON_USB0CLKSEL                (LPC54_SYSCON_BASE + LPC54_SYSCON_USB0CLKSEL_OFFSET)
#define LPC54_SYSCON_USB1CLKSEL                (LPC54_SYSCON_BASE + LPC54_SYSCON_USB1CLKSEL_OFFSET)
#define LPC54_SYSCON_FCLKSEL0                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL0_OFFSET)
#define LPC54_SYSCON_FCLKSEL1                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL1_OFFSET)
#define LPC54_SYSCON_FCLKSEL2                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL2_OFFSET)
#define LPC54_SYSCON_FCLKSEL3                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL3_OFFSET)
#define LPC54_SYSCON_FCLKSEL4                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL4_OFFSET)
#define LPC54_SYSCON_FCLKSEL5                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL5_OFFSET)
#define LPC54_SYSCON_FCLKSEL6                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL6_OFFSET)
#define LPC54_SYSCON_FCLKSEL7                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL7_OFFSET)
#define LPC54_SYSCON_FCLKSEL8                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL8_OFFSET)
#define LPC54_SYSCON_FCLKSEL9                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FCLKSEL9_OFFSET)
#define LPC54_SYSCON_MCLKCLKSEL                (LPC54_SYSCON_BASE + LPC54_SYSCON_MCLKCLKSEL_OFFSET)
#define LPC54_SYSCON_FRGCLKSEL                 (LPC54_SYSCON_BASE + LPC54_SYSCON_FRGCLKSEL_OFFSET)
#define LPC54_SYSCON_DMICCLKSEL                (LPC54_SYSCON_BASE + LPC54_SYSCON_DMICCLKSEL_OFFSET)
#define LPC54_SYSCON_SCTCLKSEL                 (LPC54_SYSCON_BASE + LPC54_SYSCON_SCTCLKSEL_OFFSET)
#define LPC54_SYSCON_LCDCLKSEL                 (LPC54_SYSCON_BASE + LPC54_SYSCON_LCDCLKSEL_OFFSET)
#define LPC54_SYSCON_SDIOCLKSEL                (LPC54_SYSCON_BASE + LPC54_SYSCON_SDIOCLKSEL_OFFSET)
#define LPC54_SYSCON_SYSTICKCLKDIV             (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSTICKCLKDIV_OFFSET)
#define LPC54_SYSCON_ARMTRCLKDIV               (LPC54_SYSCON_BASE + LPC54_SYSCON_ARMTRCLKDIV_OFFSET)
#define LPC54_SYSCON_CAN0CLKDIV                (LPC54_SYSCON_BASE + LPC54_SYSCON_CAN0CLKDIV_OFFSET)
#define LPC54_SYSCON_CAN1CLKDIV                (LPC54_SYSCON_BASE + LPC54_SYSCON_CAN1CLKDIV_OFFSET)
#define LPC54_SYSCON_SC0CLKDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_SC0CLKDIV_OFFSET)
#define LPC54_SYSCON_SC1CLKDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_SC1CLKDIV_OFFSET)
#define LPC54_SYSCON_AHBCLKDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_AHBCLKDIV_OFFSET)
#define LPC54_SYSCON_CLKOUTDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_CLKOUTDIV_OFFSET)
#define LPC54_SYSCON_FROHFDIV                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FROHFDIV_OFFSET)
#define LPC54_SYSCON_SPIFICLKDIV               (LPC54_SYSCON_BASE + LPC54_SYSCON_SPIFICLKDIV_OFFSET)
#define LPC54_SYSCON_ADCCLKDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_ADCCLKDIV_OFFSET)
#define LPC54_SYSCON_USB0CLKDIV                (LPC54_SYSCON_BASE + LPC54_SYSCON_USB0CLKDIV_OFFSET)
#define LPC54_SYSCON_USB1CLKDIV                (LPC54_SYSCON_BASE + LPC54_SYSCON_USB1CLKDIV_OFFSET)
#define LPC54_SYSCON_FRGCTRL                   (LPC54_SYSCON_BASE + LPC54_SYSCON_FRGCTRL_OFFSET)
#define LPC54_SYSCON_DMICCLKDIV                (LPC54_SYSCON_BASE + LPC54_SYSCON_DMICCLKDIV_OFFSET)
#define LPC54_SYSCON_MCLKDIV                   (LPC54_SYSCON_BASE + LPC54_SYSCON_MCLKDIV_OFFSET)
#define LPC54_SYSCON_LCDCLKDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_LCDCLKDIV_OFFSET)
#define LPC54_SYSCON_SCTCLKDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_SCTCLKDIV_OFFSET)
#define LPC54_SYSCON_EMCCLKDIV                 (LPC54_SYSCON_BASE + LPC54_SYSCON_EMCCLKDIV_OFFSET)
#define LPC54_SYSCON_SDIOCLKDIV                (LPC54_SYSCON_BASE + LPC54_SYSCON_SDIOCLKDIV_OFFSET)
#define LPC54_SYSCON_FLASHCFG                  (LPC54_SYSCON_BASE + LPC54_SYSCON_FLASHCFG_OFFSET)
#define LPC54_SYSCON_USB0CLKCTRL               (LPC54_SYSCON_BASE + LPC54_SYSCON_USB0CLKCTRL_OFFSET)
#define LPC54_SYSCON_USB0CLKSTAT               (LPC54_SYSCON_BASE + LPC54_SYSCON_USB0CLKSTAT_OFFSET)
#define LPC54_SYSCON_FREQMECTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_FREQMECTRL_OFFSET)
#define LPC54_SYSCON_MCLKIO                    (LPC54_SYSCON_BASE + LPC54_SYSCON_MCLKIO_OFFSET)
#define LPC54_SYSCON_USB1CLKCTRL               (LPC54_SYSCON_BASE + LPC54_SYSCON_USB1CLKCTRL_OFFSET)
#define LPC54_SYSCON_USB1CLKSTAT               (LPC54_SYSCON_BASE + LPC54_SYSCON_USB1CLKSTAT_OFFSET)
#define LPC54_SYSCON_EMCSYSCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_EMCSYSCTRL_OFFSET)
#define LPC54_SYSCON_EMCDLYCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_EMCDLYCTRL_OFFSET)
#define LPC54_SYSCON_EMCDLYCAL                 (LPC54_SYSCON_BASE + LPC54_SYSCON_EMCDLYCAL_OFFSET)
#define LPC54_SYSCON_ETHPHYSEL                 (LPC54_SYSCON_BASE + LPC54_SYSCON_ETHPHYSEL_OFFSET)
#define LPC54_SYSCON_ETHSBDCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_ETHSBDCTRL_OFFSET)
#define LPC54_SYSCON_SDIOCLKCTRL               (LPC54_SYSCON_BASE + LPC54_SYSCON_SDIOCLKCTRL_OFFSET)
#define LPC54_SYSCON_FROCTRL                   (LPC54_SYSCON_BASE + LPC54_SYSCON_FROCTRL_OFFSET)
#define LPC54_SYSCON_SYSOSCCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSOSCCTRL_OFFSET)
#define LPC54_SYSCON_WDTOSCCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_WDTOSCCTRL_OFFSET)
#define LPC54_SYSCON_RTCOSCCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_RTCOSCCTRL_OFFSET)
#define LPC54_SYSCON_USBPLLCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_USBPLLCTRL_OFFSET)
#define LPC54_SYSCON_USBPLLSTAT                (LPC54_SYSCON_BASE + LPC54_SYSCON_USBPLLSTAT_OFFSET)
#define LPC54_SYSCON_SYSPLLCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSPLLCTRL_OFFSET)
#define LPC54_SYSCON_SYSPLLSTAT                (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSPLLSTAT_OFFSET)
#define LPC54_SYSCON_SYSPLLNDEC                (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSPLLNDEC_OFFSET)
#define LPC54_SYSCON_SYSPLLPDEC                (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSPLLPDEC_OFFSET)
#define LPC54_SYSCON_SYSPLLMDEC                (LPC54_SYSCON_BASE + LPC54_SYSCON_SYSPLLMDEC_OFFSET)
#define LPC54_SYSCON_AUDPLLCTRL                (LPC54_SYSCON_BASE + LPC54_SYSCON_AUDPLLCTRL_OFFSET)
#define LPC54_SYSCON_AUDPLLSTAT                (LPC54_SYSCON_BASE + LPC54_SYSCON_AUDPLLSTAT_OFFSET)
#define LPC54_SYSCON_AUDPLLNDEC                (LPC54_SYSCON_BASE + LPC54_SYSCON_AUDPLLNDEC_OFFSET)
#define LPC54_SYSCON_AUDPLLPDEC                (LPC54_SYSCON_BASE + LPC54_SYSCON_AUDPLLPDEC_OFFSET)
#define LPC54_SYSCON_AUDPLLMDEC                (LPC54_SYSCON_BASE + LPC54_SYSCON_AUDPLLMDEC_OFFSET)
#define LPC54_SYSCON_AUDPLLFRAC                (LPC54_SYSCON_BASE + LPC54_SYSCON_AUDPLLFRAC_OFFSET)
#define LPC54_SYSCON_PDSLEEPCFG0               (LPC54_SYSCON_BASE + LPC54_SYSCON_PDSLEEPCFG0_OFFSET)
#define LPC54_SYSCON_PDSLEEPCFG1               (LPC54_SYSCON_BASE + LPC54_SYSCON_PDSLEEPCFG1_OFFSET)
#define LPC54_SYSCON_PDRUNCFG0                 (LPC54_SYSCON_BASE + LPC54_SYSCON_PDRUNCFG0_OFFSET)
#define LPC54_SYSCON_PDRUNCFG1                 (LPC54_SYSCON_BASE + LPC54_SYSCON_PDRUNCFG1_OFFSET)
#define LPC54_SYSCON_PDRUNCFGSET0              (LPC54_SYSCON_BASE + LPC54_SYSCON_PDRUNCFGSET0_OFFSET)
#define LPC54_SYSCON_PDRUNCFGSET1              (LPC54_SYSCON_BASE + LPC54_SYSCON_PDRUNCFGSET1_OFFSET)
#define LPC54_SYSCON_PDRUNCFGCLR0              (LPC54_SYSCON_BASE + LPC54_SYSCON_PDRUNCFGCLR0_OFFSET)
#define LPC54_SYSCON_PDRUNCFGCLR1              (LPC54_SYSCON_BASE + LPC54_SYSCON_PDRUNCFGCLR1_OFFSET)
#define LPC54_SYSCON_STARTER0                  (LPC54_SYSCON_BASE + LPC54_SYSCON_STARTER0_OFFSET)
#define LPC54_SYSCON_STARTER1                  (LPC54_SYSCON_BASE + LPC54_SYSCON_STARTER1_OFFSET)
#define LPC54_SYSCON_STARTERSET0               (LPC54_SYSCON_BASE + LPC54_SYSCON_STARTERSET0_OFFSET)
#define LPC54_SYSCON_STARTERSET1               (LPC54_SYSCON_BASE + LPC54_SYSCON_STARTERSET1_OFFSET)
#define LPC54_SYSCON_STARTERCLR0               (LPC54_SYSCON_BASE + LPC54_SYSCON_STARTERCLR0_OFFSET)
#define LPC54_SYSCON_STARTERCLR1               (LPC54_SYSCON_BASE + LPC54_SYSCON_STARTERCLR1_OFFSET)
#define LPC54_SYSCON_HWWAKE                    (LPC54_SYSCON_BASE + LPC54_SYSCON_HWWAKE_OFFSET)
#define LPC54_SYSCON_AUTOCGOR                  (LPC54_SYSCON_BASE + LPC54_SYSCON_AUTOCGOR_OFFSET)
#define LPC54_SYSCON_JTAGIDCODE                (LPC54_SYSCON_BASE + LPC54_SYSCON_JTAGIDCODE_OFFSET)
#define LPC54_SYSCON_DEVICE_ID0                (LPC54_SYSCON_BASE + LPC54_SYSCON_DEVICE_ID0_OFFSET)
#define LPC54_SYSCON_DEVICE_ID1                (LPC54_SYSCON_BASE + LPC54_SYSCON_DEVICE_ID1_OFFSET)

/* Asynchronous system configuration */

#define LPC54_SYSCON_ASYNCPRESETCTRL           (LPC54_ASYSCON_BASE + LPC54_SYSCON_ASYNCPRESETCTRL_OFFSET)
#define LPC54_SYSCON_ASYNCPRESETCTRLSET        (LPC54_ASYSCON_BASE + LPC54_SYSCON_ASYNCPRESETCTRLSET_OFFSET)
#define LPC54_SYSCON_ASYNCPRESETCTRLCLR        (LPC54_ASYSCON_BASE + LPC54_SYSCON_ASYNCPRESETCTRLCLR_OFFSET)
#define LPC54_SYSCON_ASYNCAPBCLKCTRL           (LPC54_ASYSCON_BASE + LPC54_SYSCON_ASYNCAPBCLKCTRL_OFFSET)
#define LPC54_SYSCON_ASYNCAPBCLKCTRLSET        (LPC54_ASYSCON_BASE + LPC54_SYSCON_ASYNCAPBCLKCTRLSET_OFFSET)
#define LPC54_SYSCON_ASYNCAPBCLKCTRLCLR        (LPC54_ASYSCON_BASE + LPC54_SYSCON_ASYNCAPBCLKCTRLCLR_OFFSET)
#define LPC54_SYSCON_ASYNCAPBCLKSELA           (LPC54_ASYSCON_BASE + LPC54_SYSCON_ASYNCAPBCLKSELA_OFFSET)

/* Other system configuration */

#define LPC54_SYSCON_BODCTRL                   (LPC54_OSYSCON_BASE + LPC54_SYSCON_BODCTRL_OFFSET)

/* Register bit definitions *************************************************/

/* Main system configuration */

/* AHB multilayer matrix priority control */
#define SYSCON_AHBMATPRIO_

/* System tick counter calibration */
#define SYSCON_SYSTCKCAL_

/* NMI source select */
#define SYSCON_NMISRC_

/* Asynchronous APB control */
#define SYSCON_ASYNCAPBCTRL_

/* POR captured value of port 0 */
#define SYSCON_PIOPORCAP0_

/* POR captured value of port 1 */
#define SYSCON_PIOPORCAP1_

/* Reset captured value of port 0 */
#define SYSCON_PIORESCAP0_

/* Reset captured value of port 1 */
#define SYSCON_PIORESCAP1_

/* Peripheral reset control 0:
 * PRESETCTRL0, PRESETCTRLSET0, and PRESETCTRLCLR0
 */

#define SYSCON_PRESETCTRL0_FLASH               (1 << 7)  /* Bit 7:  Reset the flash controller */
#define SYSCON_PRESETCTRL0_FMC                 (1 << 8)  /* Bit 8:  Reset the Flash accelerator */
#define SYSCON_PRESETCTRL0_EEPROM              (1 << 9)  /* Bit 9:  Reset EEPROM */
#define SYSCON_PRESETCTRL0_SPIFI               (1 << 10) /* Bit 10: Reset the SPIFI */
#define SYSCON_PRESETCTRL0_INPUTMUX            (1 << 11) /* Bit 11: Reset the input muxes */
#define SYSCON_PRESETCTRL0_IOCON               (1 << 13) /* Bit 13: Reset the IOCON block */
#define SYSCON_PRESETCTRL0_GPIO0               (1 << 14) /* Bit 14: Reset the GPIO0 port registers */
#define SYSCON_PRESETCTRL0_GPIO1               (1 << 15) /* Bit 15: Reset the GPIO1 port registers */
#define SYSCON_PRESETCTRL0_GPIO2               (1 << 16) /* Bit 16: Reset the GPIO2 port registers */
#define SYSCON_PRESETCTRL0_GPIO3               (1 << 17) /* Bit 17: Reset the GPIO3 port registers */
#define SYSCON_PRESETCTRL0_PINT                (1 << 18) /* Bit 18: Reset the pin interrupt block */
#define SYSCON_PRESETCTRL0_GINT                (1 << 19) /* Bit 19: Reset the grouped pin interrupt block */
#define SYSCON_PRESETCTRL0_DMA                 (1 << 20) /* Bit 20: Reset the DMA controller */
#define SYSCON_PRESETCTRL0_CRC                 (1 << 21) /* Bit 21: Reset the CRC engine */
#define SYSCON_PRESETCTRL0_WWDT                (1 << 22) /* Bit 22: Reset the Watchdog Timer */
#define SYSCON_PRESETCTRL0_RTC                 (1 << 23) /* Bit 23: Enables the bus clock for the RTC */
#define SYSCON_PRESETCTRL0_ADC0                (1 << 27) /* Bit 27: Reset the ADC0 register interface */

/* Peripheral reset control 2:
 * PRESETCTRL1, PRESETCTRLSET1, and PRESETCTRLCLR1
 */

#define SYSCON_PRESETCTRL1_MRT                 (1 << 0)  /* Bit 0:  Reset the Multi-Rate Timer */
#define SYSCON_PRESETCTRL1_SCT0                (1 << 2)  /* Bit 2:  Reset SCT0 */
#define SYSCON_PRESETCTRL1_MCAN0               (1 << 7)  /* Bit 7:  Reset MCAN0 */
#define SYSCON_PRESETCTRL1_MCAN1               (1 << 8)  /* Bit 8:  Reset MCAN1 */
#define SYSCON_PRESETCTRL1_UTICK               (1 << 10) /* Bit 10: Reset the Micro-tick Timer */
#define SYSCON_PRESETCTRL1_FLEXCOMM0           (1 << 11) /* Bit 11: Reset Flexcomm Interface 0 */
#define SYSCON_PRESETCTRL1_FLEXCOMM1           (1 << 12) /* Bit 12: Reset Flexcomm Interface 1 */
#define SYSCON_PRESETCTRL1_FLEXCOMM2           (1 << 13) /* Bit 13: Reset Flexcomm Interface 2 */
#define SYSCON_PRESETCTRL1_FLEXCOMM3           (1 << 14) /* Bit 14: Reset Flexcomm Interface 3 */
#define SYSCON_PRESETCTRL1_FLEXCOMM4           (1 << 15) /* Bit 15: Reset Flexcomm Interface 4 */
#define SYSCON_PRESETCTRL1_FLEXCOMM5           (1 << 16) /* Bit 16: Reset Flexcomm Interface 5 */
#define SYSCON_PRESETCTRL1_FLEXCOMM6           (1 << 17) /* Bit 17: Reset Flexcomm Interface 6 */
#define SYSCON_PRESETCTRL1_FLEXCOMM7           (1 << 18) /* Bit 18: Reset Flexcomm Interface 7 */
#define SYSCON_PRESETCTRL1_DMIC                (1 << 19) /* Bit 19: Reset the digital microphone interface */
#define SYSCON_PRESETCTRL1_CTIMER2             (1 << 22) /* Bit 22: Reset CTIMER 2 */
#define SYSCON_PRESETCTRL1_USB0D               (1 << 25) /* Bit 25: Reset the USB0 device interface */
#define SYSCON_PRESETCTRL1_CTIMER0             (1 << 26) /* Bit 26: Reset timer CTIMER0 */
#define SYSCON_PRESETCTRL1_CTIMER1             (1 << 27) /* Bit 27: Reset timer CTIMER1 */

/* Peripheral reset control 2:
 * PRESETCTRL2, PRESETCTRL2, and PRESETCTRLCLR2
 */

#define SYSCON_PRESETCTRL2_LCD                 (1 << 2)  /* Bit 2:  Reset the LCD interface */
#define SYSCON_PRESETCTRL2_SDIO                (1 << 3)  /* Bit 3:  Reset the SDIO interface */
#define SYSCON_PRESETCTRL2_USB1H               (1 << 4)  /* Bit 4:  Reset the USB1 host interface */
#define SYSCON_PRESETCTRL2_USB1D               (1 << 5)  /* Bit 5:  Reset the USB1 device interface */
#define SYSCON_PRESETCTRL2_USB1RAM             (1 << 6)  /* Bit 6:  Reset the USB1 RAM interface */
#define SYSCON_PRESETCTRL2_EMC                 (1 << 7)  /* Bit 7:  Reset the EMC interface */
#define SYSCON_PRESETCTRL2_ETH                 (1 << 8)  /* Bit 8:  Reset the ethernet interface */
#define SYSCON_PRESETCTRL2_GPIO4               (1 << 9)  /* Bit 9:  Reset the GPIO4 interface */
#define SYSCON_PRESETCTRL2_GPIO5               (1 << 10) /* Bit 10: Reset the GPIO5 interface */
#define SYSCON_PRESETCTRL2_OTP                 (1 << 12) /* Bit 12: Reset the OTP interface */
#define SYSCON_PRESETCTRL2_RNG                 (1 << 13) /* Bit 13: Reset the RNG interface */
#define SYSCON_PRESETCTRL2_FLEXCOMM8           (1 << 14) /* Bit 14: Reset the Flexcomm Interface 8 */
#define SYSCON_PRESETCTRL2_FLEXCOMM9           (1 << 15) /* Bit 15: Reset the Flexcomm Interface 9 */
#define SYSCON_PRESETCTRL2_USB0HMR             (1 << 16) /* Bit 16: Reset the USB host master interface */
#define SYSCON_PRESETCTRL2_USB0HSL             (1 << 17) /* Bit 17: Reset the USB host slave interface */
#define SYSCON_PRESETCTRL2_SHA                 (1 << 18) /* Bit 18: Reset the SHA interface */
#define SYSCON_PRESETCTRL2_SC0                 (1 << 19) /* Bit 19: Reset the Smart card0 interface */
#define SYSCON_PRESETCTRL2_SC1                 (1 << 20) /* Bit 20: Reset the Smart card1 interface */

/* System reset status register */
#define SYSCON_SYSRSTSTAT_

/* AHB Clock control 0:
 * AHBCLKCTRL0, AHBCLKCTRLCLR0, and AHBCLKCTRLSET0
 */

#define SYSCON_AHBCLKCTRL0_ROM                 (1 << 1)  /* Bit 1:  Enables the clock for the Boot ROM */
#define SYSCON_AHBCLKCTRL0_SRAM1               (1 << 3)  /* Bit 3:  Enables the clock for SRAM1 */
#define SYSCON_AHBCLKCTRL0_SRAM2               (1 << 4)  /* Bit 4:  Enables the clock for SRAM2 */
#define SYSCON_AHBCLKCTRL0_SRAM3               (1 << 5)  /* Bit 5:  Enables the clock for SRAM3 */
#define SYSCON_AHBCLKCTRL0_FLASH               (1 << 7)  /* Bit 7:  Enables the clock for the flash controller */
#define SYSCON_AHBCLKCTRL0_FMC                 (1 << 8)  /* Bit 8:  Enables the clock for the Flash accelerator */
#define SYSCON_AHBCLKCTRL0_EEPROM              (1 << 9)  /* Bit 9:  Enables the clock for EEPROM */
#define SYSCON_AHBCLKCTRL0_SPIFI               (1 << 10) /* Bit 10: Enables the clock for the SPIFI */
#define SYSCON_AHBCLKCTRL0_INPUTMUX            (1 << 11) /* Bit 11: Enables the clock for the input muxes */
#define SYSCON_AHBCLKCTRL0_IOCON               (1 << 13) /* Bit 13: Enables the clock for the IOCON block */
#define SYSCON_AHBCLKCTRL0_GPIO0               (1 << 14) /* Bit 14: Enables the clock for the GPIO0 port registers */
#define SYSCON_AHBCLKCTRL0_GPIO1               (1 << 15) /* Bit 15: Enables the clock for the GPIO1 port registers */
#define SYSCON_AHBCLKCTRL0_GPIO2               (1 << 16) /* Bit 16: Enables the clock for the GPIO2 port registers */
#define SYSCON_AHBCLKCTRL0_GPIO3               (1 << 17) /* Bit 17: Enables the clock for the GPIO3 port registers */
#define SYSCON_AHBCLKCTRL0_PINT                (1 << 18) /* Bit 18: Enables the clock for the pin interrupt block */
#define SYSCON_AHBCLKCTRL0_GINT                (1 << 19) /* Bit 19: Enables the clock for the grouped pin interrupt block */
#define SYSCON_AHBCLKCTRL0_DMA                 (1 << 20) /* Bit 20: Enables the clock for the DMA controller */
#define SYSCON_AHBCLKCTRL0_CRC                 (1 << 21) /* Bit 21: Enables the clock for the CRC engine */
#define SYSCON_AHBCLKCTRL0_WWDT                (1 << 22) /* Bit 22: Enables the clock for the Watchdog Timer */
#define SYSCON_AHBCLKCTRL0_RTC                 (1 << 23) /* Bit 23: Enables the bus clock for the RTC */
#define SYSCON_AHBCLKCTRL0_ADC0                (1 << 27) /* Bit 27: Enables the clock for the ADC0 register interface */

/* AHB Clock control 1: AHBCLKCTRL1, AHBCLKCTRLCLR1, and AHBCLKCTRLSET1 */

#define SYSCON_AHBCLKCTRL1_MRT                 (1 << 0)  /* Bit 0:  Enables the clock for the Multi-Rate Timer */
#define SYSCON_AHBCLKCTRL1_RIT                 (1 << 1)  /* Bit 1:  Enables the clock for the Repetitive Interrupt Timer */
#define SYSCON_AHBCLKCTRL1_SCT0                (1 << 2)  /* Bit 2:  Enables the clock for SCT0 */
#define SYSCON_AHBCLKCTRL1_MCAN0               (1 << 7)  /* Bit 7:  Enables the clock for MCAN0 */
#define SYSCON_AHBCLKCTRL1_MCAN1               (1 << 8)  /* Bit 8:  Enables the clock for MCAN1 */
#define SYSCON_AHBCLKCTRL1_UTICK               (1 << 10) /* Bit 10: Enables the clock for the Micro-tick Timer */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM0           (1 << 11) /* Bit 11: Enables the clock for Flexcomm Interface 0 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM1           (1 << 12) /* Bit 12: Enables the clock for Flexcomm Interface 1 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM2           (1 << 13) /* Bit 13: Enables the clock for Flexcomm Interface 2 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM3           (1 << 14) /* Bit 14: Enables the clock for Flexcomm Interface 3 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM4           (1 << 15) /* Bit 15: Enables the clock for Flexcomm Interface 4 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM5           (1 << 16) /* Bit 16: Enables the clock for Flexcomm Interface 5 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM6           (1 << 17) /* Bit 17: Enables the clock for Flexcomm Interface 6 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM7           (1 << 18) /* Bit 18: Enables the clock for Flexcomm Interface 7 */
#define SYSCON_AHBCLKCTRL1_DMIC                (1 << 19) /* Bit 19: Enables the clock for the digital microphone interface */
#define SYSCON_AHBCLKCTRL1_CTIMER2             (1 << 22) /* Bit 22: Enables the clock for CTIMER 2 */
#define SYSCON_AHBCLKCTRL1_USB0D               (1 << 25) /* Bit 25: Enables the clock for the USB0 device interface */
#define SYSCON_AHBCLKCTRL1_CTIMER0             (1 << 26) /* Bit 26: Enables the clock for timer CTIMER0 */
#define SYSCON_AHBCLKCTRL1_CTIMER1             (1 << 27) /* Bit 27: Enables the clock for timer CTIMER1 */

/* AHB Clock control 2: AHBCLKCTRL2, AHBCLKCTRLCLR2, and AHBCLKCTRLSET2 */

#define SYSCON_AHBCLKCTRL2_LCD                 (1 << 2)  /* Bit 2:  Enables the clock for the LCD interface */
#define SYSCON_AHBCLKCTRL2_SDIO                (1 << 3)  /* Bit 3:  Enables the clock for the SDIO interface */
#define SYSCON_AHBCLKCTRL2_USB1H               (1 << 4)  /* Bit 4:  Enables the clock for the USB1 host interface */
#define SYSCON_AHBCLKCTRL2_USB1D               (1 << 5)  /* Bit 5:  Enables the clock for the USB1 device interface */
#define SYSCON_AHBCLKCTRL2_USB1RAM             (1 << 6)  /* Bit 6:  Enables the clock for the USB1 RAM interface */
#define SYSCON_AHBCLKCTRL2_EMC                 (1 << 7)  /* Bit 7:  Enables the clock for the EMC interface */
#define SYSCON_AHBCLKCTRL2_ETH                 (1 << 8)  /* Bit 8:  Enables the clock for the ethernet interface */
#define SYSCON_AHBCLKCTRL2_GPIO4               (1 << 9)  /* Bit 9:  Enables the clock for the GPIO4 interface */
#define SYSCON_AHBCLKCTRL2_GPIO5               (1 << 10) /* Bit 10: Enables the clock for the GPIO5 interface */
#define SYSCON_AHBCLKCTRL2_OTP                 (1 << 12) /* Bit 12: Enables the clock for the OTP interface */
#define SYSCON_AHBCLKCTRL2_RNG                 (1 << 13) /* Bit 13: Enables the clock for the RNG interface */
#define SYSCON_AHBCLKCTRL2_FLEXCOMM8           (1 << 14) /* Bit 14: Enables the clock for the Flexcomm Interface 8 */
#define SYSCON_AHBCLKCTRL2_FLEXCOMM9           (1 << 15) /* Bit 15: Enables the clock for the Flexcomm Interface 9 */
#define SYSCON_AHBCLKCTRL2_USB0HMR             (1 << 16) /* Bit 16: Enables the clock for the USB host master interface */
#define SYSCON_AHBCLKCTRL2_USB0HSL             (1 << 17) /* Bit 17: Enables the clock for the USB host slave interface */
#define SYSCON_AHBCLKCTRL2_SHA                 (1 << 18) /* Bit 18: Enables the clock for the SHA interface */
#define SYSCON_AHBCLKCTRL2_SC0                 (1 << 19) /* Bit 19: Enables the clock for the Smart card0 interface */
#define SYSCON_AHBCLKCTRL2_SC1                 (1 << 20) /* Bit 20: Enables the clock for the Smart card1 interface */

/* Main clock source select A */

#define SYSCON_MAINCLKSELA_SHIFT               (0)       /* Bits 0-1:  Clock source for main clock */
#define SYSCON_MAINCLKSELA_MASK                (3 << SYSCON_MAINCLKSELA_SHIFT)
#  define SYSCON_MAINCLKSELA_FRO12             (0 << SYSCON_MAINCLKSELA_SHIFT) /* FRO 12 MHz (fro_12m) */
#  define SYSCON_MAINCLKSELA_CLKIN             (1 << SYSCON_MAINCLKSELA_SHIFT) /* CLKIN (clk_in) */
#  define SYSCON_MAINCLKSELA_WDTCLK            (2 << SYSCON_MAINCLKSELA_SHIFT) /* Watchdog oscillator (wdt_clk) */
#  define SYSCON_MAINCLKSELA_FROHF             (3 << SYSCON_MAINCLKSELA_SHIFT) /* FRO 96 or 48 MHz (fro_hf) */

/* Main clock source select B */

#define SYSCON_MAINCLKSELB_SHIFT               (0)       /* Bits 0-1: Clock source for main clock */
#define SYSCON_MAINCLKSELB_MASK                (3 << SYSCON_MAINCLKSELB_SHIFT)
#  define SYSCON_MAINCLKSELB_MAINCLKSELA       (0 << SYSCON_MAINCLKSELB_SHIFT) /* Use MAINCLKSELA selection */
#  define SYSCON_MAINCLKSELB_PLLCLK            (2 << SYSCON_MAINCLKSELB_SHIFT) /* System PLL output (pll_clk) */
#  define SYSCON_MAINCLKSELB_32KCLK            (3 << SYSCON_MAINCLKSELB_SHIFT) /* RTC oscillator 32 kHz output (32k_clk) */

/* CLKOUT clock source select */
#define SYSCON_CLKOUTSELA_

/* PLL clock source select */

#define SYSCON_SYSPLLCLKSEL_SHIFT              (0)       /* Bits 0-2: System PLL clock source selection */
#define SYSCON_SYSPLLCLKSEL_MASK               (7 << SYSCON_SYSPLLCLKSEL_SHIFT)
#  define SYSCON_SYSPLLCLKSEL_FFRO             (0 << SYSCON_SYSPLLCLKSEL_SHIFT) /* FRO 12 MHz (fro_12m) */
#  define SYSCON_SYSPLLCLKSEL_CLKIN            (1 << SYSCON_SYSPLLCLKSEL_SHIFT) /* CLKIN (clk_in) */
#  define SYSCON_SYSPLLCLKSEL_RTC              (3 << SYSCON_SYSPLLCLKSEL_SHIFT) /* RTC oscillator 32 KHz output */
#  define SYSCON_SYSPLLCLKSEL_NONE             (7 << SYSCON_SYSPLLCLKSEL_SHIFT) /* None */

/* Audio PLL clock source select */
#define SYSCON_AUDPLLCLKSEL_

/* SPIFI clock source select */
#define SYSCON_SPIFICLKSEL_

/* ADC clock source select */
#define SYSCON_ADCCLKSEL_

/* USB0 clock source select */
#define SYSCON_USB0CLKSEL_

/* USB1 clock source select */
#define SYSCON_USB1CLKSEL_

/* Flexcomm Interface 0..9 clock source select */

#define SYSCON_FCLKSEL_SHIFT                   (0)     /* Bits 0-2: Flexcomm Interface clock source selection */
#define SYSCON_FCLKSEL_MASK                    (7 << SYSCON_FCLKSEL_SHIFT)
#  define SYSCON_FCLKSEL_FRO12M                (0 << SYSCON_FCLKSEL_SHIFT) /* FRO 12 MHz (fro_12m) */
#  define SYSCON_FCLKSEL_FROHFDIV              (1 << SYSCON_FCLKSEL_SHIFT) /* FRO 96 or 48 MHz divided (fro_hf_div) */
#  define SYSCON_FCLKSEL_AUDPLLCLK             (2 << SYSCON_FCLKSEL_SHIFT) /* Audio PLL clock (audio_pll_clk) */
#  define SYSCON_FCLKSEL_MCLK                  (3 << SYSCON_FCLKSEL_SHIFT) /* MCK */
#  define SYSCON_FCLKSEL_FRG                   (4 << SYSCON_FCLKSEL_SHIFT) /* FRG clock */
#  define SYSCON_FCLKSEL_NONE                  (7 << SYSCON_FCLKSEL_SHIFT) /* None */

/* MCLK clock source select */

#define SYSCON_MCLKCLKSEL_SHIFT                (0)     /* Bits 0-2: MCLK source select */
#define SYSCON_MCLKCLKSEL_MASK                 (7 << SYSCON_MCLKCLKSEL_SHIFT)
#  define SYSCON_MCLKCLKSEL_FROHFDIV           (0 << SYSCON_MCLKCLKSEL_SHIFT) /* FRO 96 or 48 MHz divided (fro_hf_div) */
#  define SYSCON_MCLKCLKSEL_AUDPLLCLK          (1 << SYSCON_MCLKCLKSEL_SHIFT) /* Audio PLL clock (audio_pll_clk) */
#  define SYSCON_MCLKCLKSEL_NONE               (7 << SYSCON_MCLKCLKSEL_SHIFT) /* None */

/* Fractional Rate Generator clock source select */

#define SYSCON_FRGCLKSEL_SHIFT                 (0)       /* Bits 0-2: Fractional Rate Generator clock source */
#define SYSCON_FRGCLKSEL_MASK                  (7 << SYSCON_FRGCLKSEL_SHIFT)
#  define SYSCON_FRGCLKSEL_MAINCLK             (0 << SYSCON_FRGCLKSEL_SHIFT) /* Main clock (main_clk) */
#  define SYSCON_FRGCLKSEL_PLLCLK              (1 << SYSCON_FRGCLKSEL_SHIFT) /* System PLL output (pll_clk) */
#  define SYSCON_FRGCLKSEL_FRO12M              (2 << SYSCON_FRGCLKSEL_SHIFT) /* FRO 12 MHz (fro_12m) */
#  define SYSCON_FRGCLKSEL_FROHF               (3 << SYSCON_FRGCLKSEL_SHIFT) /* FRO 96 or 48 MHz (fro_hf) */
#  define SYSCON_FRGCLKSEL_NONE                (7 << SYSCON_FRGCLKSEL_SHIFT) /* None */

/* Digital microphone (DMIC) subsystem clock select */
#define SYSCON_DMICCLKSEL_
/* SCTimer/PWM clock source select */
#define SYSCON_SCTCLKSEL_

/* LCD clock source select */

#define SYSCON_LCDCLKSEL_SHIFT                 (0)       /* Bits 0-1: LCD clock source select */
#define SYSCON_LCDCLKSEL_MASK                  (3 << SYSCON_LCDCLKSEL_SHIFT)
#  define SYSCON_LCDCLKSEL_MAINCLK             (0 << SYSCON_LCDCLKSEL_SHIFT) /* Main clock (main_clk) */
#  define SYSCON_LCDCLKSEL_LCDCLKIN            (1 << SYSCON_LCDCLKSEL_SHIFT) /* LCD external clock input (LCD_CLKIN) */
#  define SYSCON_LCDCLKSEL_FROHF               (2 << SYSCON_LCDCLKSEL_SHIFT) /* FRO 96 or 48 MHz (fro_hf) */
#  define SYSCON_LCDCLKSEL_NONE                (3 << SYSCON_LCDCLKSEL_SHIFT) /* None */

/* SDIO clock source select */

#define SYSCON_SDIOCLKSEL_SHIFT                (0)       /* Bits 0-2: SDIO clock source select */
#define SYSCON_SDIOCLKSEL_MASK                 (7 << SYSCON_SDIOCLKSEL_SHIFT)
#  define SYSCON_SDIOCLKSEL_MAINCLK            (0 << SYSCON_SDIOCLKSEL_SHIFT) /* Main clock (main_clk) */
#  define SYSCON_SDIOCLKSEL_PLLCLK             (1 << SYSCON_SDIOCLKSEL_SHIFT) /* System PLL output (pll_clk) */
#  define SYSCON_SDIOCLKSEL_USBPLLCLK          (2 << SYSCON_SDIOCLKSEL_SHIFT) /* USB PLL clock (usb_pll_clk) */
#  define SYSCON_SDIOCLKSEL_FROHF              (3 << SYSCON_SDIOCLKSEL_SHIFT) /* FRO 96 or 48 MHz (fro_hf) */
#  define SYSCON_SDIOCLKSEL_AUDIOPLLCLK        (4 << SYSCON_SDIOCLKSEL_SHIFT) /* Audio PLL clock (audio_pll_clk) */

/* SYSTICK clock divider */

#define SYSCON_SYSTICKCLKDIV_DIV_SHIFT         (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_SYSTICKCLKDIV_DIV_MASK          (0xff << SYSCON_SYSTICKCLKDIV_DIV_SHIFT)
#  define SYSCON_SYSTICKCLKDIV_DIV(n)          ((uint32_t)((n)-1) << SYSCON_SYSTICKCLKDIV_DIV_SHIFT)
#define SYSCON_SYSTICKCLKDIV_RESET             (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_SYSTICKCLKDIV_HALT              (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_SYSTICKCLKDIV_REQFLAG           (1 << 31) /* Bit 31: Divider status flag */

/* ARM Trace clock divider */
#define SYSCON_ARMTRCLKDIV_

/* MCAN0 clock divider */
#define SYSCON_CAN0CLKDIV_

/* MCAN1 clock divider */
#define SYSCON_CAN1CLKDIV_

/* Smartcard0 clock divider */
#define SYSCON_SC0CLKDIV_

/* Smartcard1 clock divider */
#define SYSCON_SC1CLKDIV_

/* System clock divider */

#define SYSCON_AHBCLKDIV_DIV_SHIFT             (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_AHBCLKDIV_DIV_MASK              (0xff << SYSCON_AHBCLKDIV_DIV_SHIFT)
#  define SYSCON_AHBCLKDIV_DIV(n)              ((uint32_t)((n)-1) << SYSCON_AHBCLKDIV_DIV_SHIFT)
#define SYSCON_AHBCLKDIV_REQFLAG               (1 << 31) /* Bit 32: Divider status flag */

/* CLKOUT clock divider */
#define SYSCON_CLKOUTDIV_

/* FROHF clock divider */
#define SYSCON_FROHFDIV_

/* SPIFI clock divider */
#define SYSCON_SPIFICLKDIV_

/* ADC clock divider */
#define SYSCON_ADCCLKDIV_

/* USB0 clock divider */
#define SYSCON_USB0CLKDIV_

/* USB1 clock divider */
#define SYSCON_USB1CLKDIV_

/* Fractional rate divider */

#define SYSCON_FRGCTRL_DIV_SHIFT               (0)       /*  Bits 0-7: Denominator of the fractional divider */
#define SYSCON_FRGCTRL_DIV_MASK                (0xff << SYSCON_FRGCTRL_DIV_SHIFT)
#  define SYSCON_FRGCTRL_DIV(n)                ((uint32_t)((n)-1) << SYSCON_FRGCTRL_DIV_SHIFT)
#define SYSCON_FRGCTRL_MULT_SHIFT              (8)       /* Bit 8-15: Numerator of the fractional divider */
#define SYSCON_FRGCTRL_MULT_MASK               (0xff << SYSCON_FRGCTRL_MULT_SHIFT)
#  define SYSCON_FRGCTRL_MULT(n)               ((uint32_t)(n) << SYSCON_FRGCTRL_MULT_SHIFT)

/* DMIC clock divider */
#define SYSCON_DMICCLKDIV_

/* I2S MCLK clock divider */
#define SYSCON_MCLKDIV_

/* LCD clock divider */

#define SYSCON_LCDCLKDIV_DIV_SHIFT             (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_LCDCLKDIV_DIV_MASK              (0xff <<SYSCON_LCDCLKDIV_DIV_SHIFT)
#  define SYSCON_LCDCLKDIV_DIV(n)              ((uint32_t)((n)-1) << SYSCON_LCDCLKDIV_DIV_SHIFT)
#define SYSCON_LCDCLKDIV_RESET                 (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_LCDCLKDIV_HALT                  (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_LCDCLKDIV_REQFLAG               (1 << 31) /* Bit 31: Divider status flag */

/* SCT/PWM clock divider */
#define SYSCON_SCTCLKDIV_

/* EMC clock divider */

#define SYSCON_EMCCLKDIV_DIV_SHIFT             (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_EMCCLKDIV_DIV_MASK              (0xff <<SYSCON_EMCCLKDIV_DIV_SHIFT)
#  define SYSCON_EMCCLKDIV_DIV(n)              ((uint32_t)((n)-1) << SYSCON_EMCCLKDIV_DIV_SHIFT)
#define SYSCON_EMCCLKDIV_RESET                 (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_EMCCLKDIV_HALT                  (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_EMCCLKDIV_REQFLAG               (1 << 31) /* Bit 31: Divider status flag */

/* SDIO clock divider */

#define SYSCON_SDIOCLKDIV_DIV_SHIFT            (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_SDIOCLKDIV_DIV_MASK             (0xff <<SYSCON_SDIOCLKDIV_DIV_SHIFT)
#  define SYSCON_SDIOCLKDIV_DIV(n)             ((uint32_t)((n)-1) << SYSCON_SDIOCLKDIV_DIV_SHIFT)
#define SYSCON_SDIOCLKDIV_RESET                (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_SDIOCLKDIV_HALT                 (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_SDIOCLKDIV_REQFLAG              (1 << 31) /* Bit 31: Divider status flag */

/* Flash wait states configuration */

#define SYSCON_FLASHCFG_FETCHCFG_SHIFT         (0)       /* Bits 0-1: Instruction fetch configuration */
#define SYSCON_FLASHCFG_FETCHCFG_MASK          (3 << SYSCON_FLASHCFG_FETCHCFG_SHIFT)
#  define SYSCON_FLASHCFG_FETCHCFG_NONE        (0 << SYSCON_FLASHCFG_FETCHCFG_SHIFT) /* Instruction fetches not buffered */
#  define SYSCON_FLASHCFG_FETCHCFG_ONE         (1 << SYSCON_FLASHCFG_FETCHCFG_SHIFT) /* One buffer used for instruction fetches */
#  define SYSCON_FLASHCFG_FETCHCFG_ALL         (2 << SYSCON_FLASHCFG_FETCHCFG_SHIFT) /*  All buffers used for instruction fetches */

#define SYSCON_FLASHCFG_DATACFG_SHIFT          (2)       /* Bit 2-3: Data read configuration */
#define SYSCON_FLASHCFG_DATACFG_MASK           (3 << SYSCON_FLASHCFG_DATACFG_SHIFT)
#  define SYSCON_FLASHCFG_DATACFG_NONE         (0 << SYSCON_FLASHCFG_DATACFG_SHIFT) /* Data accesses from flash not buffered */
#  define SYSCON_FLASHCFG_DATACFG_ONE          (1 << SYSCON_FLASHCFG_DATACFG_SHIFT) /* One buffer used for data accesses */
#  define SYSCON_FLASHCFG_DATACFG_ALL          (2 << SYSCON_FLASHCFG_DATACFG_SHIFT) /*  All buffers used for data accesses */

#define SYSCON_FLASHCFG_ACCEL                  (1 << 4)  /* Bit 4:  Acceleration enable */
#define SYSCON_FLASHCFG_PREFEN                 (1 << 5)  /* Bit 5:  Prefetch enable */
#define SYSCON_FLASHCFG_PREFOVR                (1 << 6)  /* Bit 6:  Prefetch override */
#define SYSCON_FLASHCFG_FLASHTIM_SHIFT         (12)      /* Bits 12-15: Flash memory access time */
#define SYSCON_FLASHCFG_FLASHTIM_MASK          (15 << SYSCON_FLASHCFG_FLASHTIM_SHIFT)
#  define SYSCON_FLASHCFG_FLASHTIM(n)          ((uint32_t)((n)-1) << SYSCON_FLASHCFG_FLASHTIM_SHIFT)

/* USB0 clock control */
#define SYSCON_USB0CLKCTRL_

/* USB0 clock status */
#define SYSCON_USB0CLKSTAT_

/* Frequency measure register */
#define SYSCON_FREQMECTRL_

/* MCLK input/output control */
#define SYSCON_MCLKIO_

/* USB1 clock control */
#define SYSCON_USB1CLKCTRL_

/* USB1 clock status */
#define SYSCON_USB1CLKSTAT_

/* EMC system control */

#define SYSCON_EMCSYSCTRL_SC                   (1 << 0)  /* Bit 0:  EMC Shift Control */
#define SYSCON_EMCSYSCTRL_RD                   (1 << 1)  /* Bit 1:  EMC Reset Disable */
#define SYSCON_EMCSYSCTRL_BC                   (1 << 2)  /* Bit 2:  External Memory Controller burst control */
#define SYSCON_EMCSYSCTRL_FBCLKINSEL           (1 << 3)  /* Bit 3:  External Memory Controller clock select */

/* EMC clock delay control */
#define SYSCON_EMCDLYCTRL_

/* EMC delay chain calibration control */
#define SYSCON_EMCDLYCAL_

/* Ethernet PHY selection */

#define SYSCON_ETHPHYSEL_MASK                  (1 << 2)  /* Bit 2:  PHY_SEL PHY interface */
#  define SYSCON_ETHPHYSEL_MII                 (0)       /*         Select MII PHY Interface */
#  define SYSCON_ETHPHYSEL_RMII                (1 << 2)  /*         Select RMII PHY Interface */

/* Ethernet SBD flow control */

#define SYSCON_ETHSBDCTRL_SHIFT                (0)       /* Bits 0-1: Sideband Flow Control */
#define SYSCON_ETHSBDCTRL_MASK                 (3 << SYSCON_ETHSBDCTRL_SHIFT)
#  define SYSCON_ETHSBDCTRL_CHAN1              (0 << SYSCON_ETHSBDCTRL_SHIFT) /* Controls channel 0 */
#  define SYSCON_ETHSBDCTRL_CHAN2              (2 << SYSCON_ETHSBDCTRL_SHIFT) /* Controls channel 1 */

/* SDIO CCLKIN phase and delay control */

#define SYSCON_SDIOCLKCTRL_DRVPHASE_SHIFT      (0)       /* Bit 0-1: cclk_in_drv phase */
#define SYSCON_SDIOCLKCTRL_DRVPHASE_MASK       (3 << SYSCON_SDIOCLKCTRL_DRVPHASE_SHIFT)
#  define SYSCON_SDIOCLKCTRL_DRVPHASE_0        (0 << SYSCON_SDIOCLKCTRL_DRVPHASE_SHIFT) /* 0 degree shift */
#  define SYSCON_SDIOCLKCTRL_DRVPHASE_90       (1 << SYSCON_SDIOCLKCTRL_DRVPHASE_SHIFT) /* 90 degree shift */
#  define SYSCON_SDIOCLKCTRL_DRVPHASE_180      (2 << SYSCON_SDIOCLKCTRL_DRVPHASE_SHIFT) /* 180 degree shift */
#  define SYSCON_SDIOCLKCTRL_DRVPHASE_270      (3 << SYSCON_SDIOCLKCTRL_DRVPHASE_SHIFT) /* 270 degree shift */

#define SYSCON_SDIOCLKCTRL_SMPPHASE_SHIFT      (2)       /* Bits 2-3: cclk_in_sample delay */
#define SYSCON_SDIOCLKCTRL_SMPPHASE_MASK       (3 << SYSCON_SDIOCLKCTRL_SMPPHASE_SHIFT)
#  define SYSCON_SDIOCLKCTRL_SMPPHASE_0        (0 << SYSCON_SDIOCLKCTRL_SMPPHASE_SHIFT) /* 0 degree shift */
#  define SYSCON_SDIOCLKCTRL_SMPPHASE_90       (1 << SYSCON_SDIOCLKCTRL_SMPPHASE_SHIFT) /* 90 degree shift */
#  define SYSCON_SDIOCLKCTRL_SMPPHASE_180      (2 << SYSCON_SDIOCLKCTRL_SMPPHASE_SHIFT) /* 180 degree shift */
#  define SYSCON_SDIOCLKCTRL_SMPPHASE_270      (3 << SYSCON_SDIOCLKCTRL_SMPPHASE_SHIFT) /* 270 degree shift */

#define SYSCON_SDIOCLKCTRL_PHASEACTIVE         (1 << 7)  /* Bit 7:  Enables the delays */
#define SYSCON_SDIOCLKCTRL_DRVDLY_SHIFT        (16)      /* Bits 16-20: cclk_in_drv delay */
#define SYSCON_SDIOCLKCTRL_DRVDLY_MASK         (0x1f << SYSCON_SDIOCLKCTRL_DRVDLY_SHIFT)
#  define SYSCON_SDIOCLKCTRL_DRVDLY(n)         ((uint32_t)((n)-1) << SYSCON_SDIOCLKCTRL_DRVDLY_SHIFT)
#define SYSCON_SDIOCLKCTRL_DRVDLYACTIVE        (1 << 23) /* Bit 23: Enables drive delay */
#define SYSCON_SDIOCLKCTRL_SMPDLY_SHIFT        (24)      /* Bits 24-28: cclk_in_sample delay */
#define SYSCON_SDIOCLKCTRL_SMPDLY_MASK         (0x1f << SYSCON_SDIOCLKCTRL_SMP_DLYSHIFT)
#  define SYSCON_SDIOCLKCTRL_SMPDLY(n)         ((uint32_t)((n)-1) << SYSCON_SDIOCLKCTRL_SMPDLY_SHIFT)
#define SYSCON_SDIOCLKCTRL_CCLK_SMPDLYACTIVE   (1 << 31) /* Bit 31: Enables sample delay */

/* Default phase and delay settings. */

#if 1 /* REVISIT.  Phase settings are only needed for clocking >= 50MHz. */
#  define SYSCON_SDIOCLKCTRL_DRVPHASE_DEFAULT  0
#  define SYSCON_SDIOCLKCTRL_SMPPHASE_DEFAULT  0
#else
#  define SYSCON_SDIOCLKCTRL_DRVPHASE_DEFAULT  (SYSCON_SDIOCLKCTRL_SMPPHASE_90 | SYSCON_SDIOCLKCTRL_PHASEACTIVE)
#  define SYSCON_SDIOCLKCTRL_SMPPHASE_DEFAULT  (SYSCON_SDIOCLKCTRL_SMPPHASE_0  | SYSCON_SDIOCLKCTRL_PHASEACTIVE)
#endif

#define SYSCON_SDIOCLKCTRL_DRVDLY_DEFAULT      (SYSCON_SDIOCLKCTRL_DRVDLY(32)  | SYSCON_SDIOCLKCTRL_DRVDLYACTIVE)
#define SYSCON_SDIOCLKCTRL_SMPDLY_DEFAULT      (SYSCON_SDIOCLKCTRL_SMPDLY(1)   | SYSCON_SDIOCLKCTRL_CCLK_SMPDLYACTIVE)

/* FRO oscillator control */

#define SYSCON_FROCTRL_

/* System oscillator control */

#define SYSCON_SYSOSCCTRL_

/* Watchdog oscillator control */

#define SYSCON_WDTOSCCTRL_DIVSEL_SHIFT         (0)       /* Bits 0-4: Divider adjust oscillator value */
#define SYSCON_WDTOSCCTRL_DIVSEL_MASK          (0x1f << SYSCON_WDTOSCCTRL_DIVSEL_SHIFT)
#  define SYSCON_WDTOSCCTRL_DIVSEL(n)          ((uint32_t)(((n) >> 1) - 1) << SYSCON_WDTOSCCTRL_DIVSEL_SHIFT)
#define SYSCON_WDTOSCCTRL_FREQSEL_SHIFT        (5)       /* Bits 5-9: Frequency select */
#define SYSCON_WDTOSCCTRL_FREQSEL_MASK         (0x1f << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)
#  define SYSCON_WDTOSCCTRL_FREQSEL_04pMHZ     (1 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 0.4 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_0p6MHZ     (2 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 0.6 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_0p75MHZ    (3 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 0.75 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_0p9MHZ     (4 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 0.9 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p0MHZ     (5 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 1.0 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p2MHZ     (6 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 1.2 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p3MHZ     (7 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 1.3 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p4MHZ     (8 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 1.4 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p5MHZ     (9 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)  /* 1.5 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p6MHZ     (10 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 1.6 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p7MHZ     (11 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 1.7 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p8MHZ     (12 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 1.8 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p9MHZ     (13 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 1.9 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p0MHZ     (14 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.0 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p05MHZ    (15 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.05 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p1MHZ     (16 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.1 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p2MHZ     (17 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.2 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p25MHZ    (18 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.25 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p3MHZ     (19 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.3 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p4MHZ     (20 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.4 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p45MHZ    (21 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.45 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p5MHZ     (22 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.5 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p6MHZ     (23 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.6 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p65MHZ    (24 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.65 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p7MHZ     (25 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.7 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p8MHZ     (26 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.8 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p85MHZ    (27 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.85 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p9MHZ     (28 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.9 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p95MHZ    (29 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 2.95 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_3p0MHZ     (30 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 3.0 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_3p05MHZ    (31 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* 3.05 MHz */

/* RTC oscillator 32 kHz output control */

#define SYSCON_RTCOSCCTRL_EN                   (1 << 0)  /* Bit 0: RTC 32 kHz clock enable */

/* USB PLL control */
#define SYSCON_USBPLLCTRL_

/* USB PLL status */
#define SYSCON_USBPLLSTAT_

/* System PLL control */

#define SYSCON_SYSPLLCTRL_SELR_SHIFT           (0)       /* Bits 0-3: Bandwidth select R value */
#define SYSCON_SYSPLLCTRL_SELR_MASK            (15 << SYSCON_SYSPLLCTRL_SELR_SHIFT)
#  define SYSCON_SYSPLLCTRL_SELR(n)            ((uint32_t)(n) << SYSCON_SYSPLLCTRL_SELR_SHIFT)
#define SYSCON_SYSPLLCTRL_SELI_SHIFT           (4)       /* Bits 4-9: Bandwidth select I value */
#define SYSCON_SYSPLLCTRL_SELI_MASK            (0x3f << SYSCON_SYSPLLCTRL_SELI_SHIFT)
#  define SYSCON_SYSPLLCTRL_SELI(n)            ((uint32_t)(n) << SYSCON_SYSPLLCTRL_SELI_SHIFT)
#define SYSCON_SYSPLLCTRL_SELP_SHIFT           (10)      /* Bits 10-14: Bandwidth select P value */
#define SYSCON_SYSPLLCTRL_SELP_MASK            (0x1f << SYSCON_SYSPLLCTRL_SELP_SHIFT)
#  define SYSCON_SYSPLLCTRL_SELP(n)            ((uint32_t)(n) << SYSCON_SYSPLLCTRL_SELP_SHIFT)
#define SYSCON_SYSPLLCTRL_BYPASS               (1 << 15) /* Bit 15: PLL bypass control */
#define SYSCON_SYSPLLCTRL_UPLIMOFF             (1 << 17) /* Bit 17: Disable upper frequency limiter */
#define SYSCON_SYSPLLCTRL_DIRECTI              (1 << 19) /* Bit 19: PLL direct input enable */
#define SYSCON_SYSPLLCTRL_DIRECTO              (1 << 10) /* Bit 20: PLL direct output enable */

/* PLL status */

#define SYSCON_SYSPLLSTAT_LOCK                 (1 << 0)  /* Bit 0:   LOCK PLL lock indicator */

/* PLL N divider */

#define SYSCON_SYSPLLNDEC_NDEC_SHIFT           (0)       /* Bits 0-9: Decoded N-divider coefficient */
#define SYSCON_SYSPLLNDEC_NDEC_MASK            (0x3ff << SYSCON_SYSPLLNDEC_NDEC_SHIFT)
#  define SYSCON_SYSPLLNDEC_NDEC(n)            ((uint32_t)(n) << SYSCON_SYSPLLNDEC_NDEC_SHIFT)
#define SYSCON_SYSPLLNDEC_NREQ                 (1 << 10) /* Bit 10:  NDEC reload request */

/* PLL P divider */

#define SYSCON_SYSPLLPDEC_PDEC_SHIFT           (0)       /* Bits 0-6: Decoded P-divider coefficient */
#define SYSCON_SYSPLLPDEC_PDEC_MASK            (0x7f << SYSCON_SYSPLLPDEC_PDEC_SHIFT)
#  define SYSCON_SYSPLLPDEC_PDEC(n)            ((uint32_t)(n) << SYSCON_SYSPLLPDEC_PDEC_SHIFT)
#define SYSCON_SYSPLLPDEC_PREQ                 (1 << 7)  /* Bit 7:  PDEC reload request */

/* System PLL M divider */

#define SYSCON_SYSPLLMDEC_MDEC_SHIFT           (0)       /* Bits 0-16: Decoded M-divider coefficient */
#define SYSCON_SYSPLLMDEC_MDEC_MASK            (0xffff << SYSCON_SYSPLLMDEC_MDEC_SHIFT)
#  define SYSCON_SYSPLLMDEC_MDEC(n)            ((uint32_t)(n) << SYSCON_SYSPLLMDEC_MDEC_SHIFT)
#define SYSCON_SYSPLLMDEC_MREQ                 (1 << 17) /* Bit 17:  MDEC reload request */

/* Audio PLL control */
#define SYSCON_AUDPLLCTRL_

/* Audio PLL status */
#define SYSCON_AUDPLLSTAT_

/* Audio PLL N divider */
#define SYSCON_AUDPLLNDEC_

/* Audio PLL P divider */
#define SYSCON_AUDPLLPDEC_

/* Audio PLL M divider */
#define SYSCON_AUDPLLMDEC_

/* Audio PLL fractional divider control */
#define SYSCON_AUDPLLFRAC_

/* Sleep configuration register 0 */
#define SYSCON_PDSLEEPCFG0_

/* Sleep configuration register 1 */
#define SYSCON_PDSLEEPCFG1_

/* Power configuration register 0 (also corresponding set/clear registers) */

#define SYSCON_PDRUNCFG0_FRO                   (1 << 4)  /* Bit 4:  FRO oscillator */
#define SYSCON_PDRUNCFG0_TS                    (1 << 6)  /* Bit 6:  Temp sensor */
#define SYSCON_PDRUNCFG0_BODRST                (1 << 7)  /* Bit 7:  Brown-out Detect reset */
#define SYSCON_PDRUNCFG0_BODINTR               (1 << 8)  /* Bit 8:  Brown-out Detect interrupt */
#define SYSCON_PDRUNCFG0_VD2ANA                (1 << 9)  /* Bit 9:  Analog supply for System Oscillator */
#define SYSCON_PDRUNCFG0_ADC0                  (1 << 10) /* Bit 10: ADC power */
#define SYSCON_PDRUNCFG0_SRAMX                 (1 << 13) /* Bit 13: Controls SRAMX */
#define SYSCON_PDRUNCFG0_SRAM0                 (1 << 14) /* Bit 14: Controls SRAM0 */
#define SYSCON_PDRUNCFG0_SRAM123               (1 << 15) /* Bit 15: Controls SRAM1, SRAM2, and SRAM3 */
#define SYSCON_PDRUNCFG0_USBRAM                (1 << 16) /* Bit 16: Controls USB_RAM */
#define SYSCON_PDRUNCFG0_VDDA                  (1 << 19) /* Bit 19: VDDA to the ADC */
#define SYSCON_PDRUNCFG0_WDTOSC                (1 << 20) /* Bit 20: Watchdog oscillator */
#define SYSCON_PDRUNCFG0_USB0PHY               (1 << 21) /* Bit 21: USB0 PHY power */
#define SYSCON_PDRUNCFG0_SYSPLL                (1 << 22) /* Bit 22: System PLL power */
#define SYSCON_PDRUNCFG0_VREFP                 (1 << 23) /* Bit 23: VREFP to the ADC  */
#define SYSCON_PDRUNCFG0_VD3                   (1 << 26) /* Bit 26: Power control for all PLLs */
#define SYSCON_PDRUNCFG0_VD4                   (1 << 27) /* Bit 27: Power control for all SRAMs and ROM */
#define SYSCON_PDRUNCFG0_VD5                   (1 << 28) /* Bit 28: Power control both USB0 PHY and USB1 PHY */
#define SYSCON_PDRUNCFG0_VD6                   (1 << 29) /* Bit 29  Power control for EEPROM */

/* Power configuration register 1  (also corresponding set/clear registers) */

#define SYSCON_PDRUNCFG1_USB1PHY               (1 << 0)  /* Bit 0:  USB1 high speed PHY */
#define SYSCON_PDRUNCFG1_USB1PLL               (1 << 1)  /* Bit 1:  USB PLL power */
#define SYSCON_PDRUNCFG1_AUDPLL                (1 << 2)  /* Bit 2:  Audio PLL power and fractional divider */
#define SYSCON_PDRUNCFG1_SYSOSC                (1 << 3)  /* Bit 3:  System Oscillator Power */
#define SYSCON_PDRUNCFG1_EEPROM                (1 << 5)  /* Bit 5:  EEPROM power */
#define SYSCON_PDRUNCFG1_RNG                   (1 << 7)  /* Bit 7:  Random Number Generator Power */

/* Start logic 0 wake-up enable register */
#define SYSCON_STARTER0_

/* Start logic 1 wake-up enable register */
#define SYSCON_STARTER1_

/* Set bits in STARTER0 */
#define SYSCON_STARTERSET0_

/* Set bits in STARTER1 */
#define SYSCON_STARTERSET1_

/* Clear bits in STARTER0 */
#define SYSCON_STARTERCLR0_

/* Clear bits in STARTER1 */
#define SYSCON_STARTERCLR1_

/* Configures special cases of hardware wake-up */
#define SYSCON_HWWAKE_

/* Auto clock-gate override */
#define SYSCON_AUTOCGOR_

/* JTAG ID code */
#define SYSCON_JTAGIDCODE_

/* Part ID */
#define SYSCON_DEVICE_ID0_

/* Boot ROM and die revision */
#define SYSCON_DEVICE_ID1_

/* Asynchronous system configuration */

/* Async peripheral reset control, set, and clear registers */

#define SYSCON_ASYNCPRESET_CTIMER3             (1 << 13) /* Bit 13:  CTIMER3 reset control */
#define SYSCON_ASYNCPRESET_CTIMER4             (1 << 14) /* Bit 14:  CTIMER4 reset control */

/* Async peripheral clock control, set and clear registers */

#define SYSCON_ASYNCAPBCLKCTRL_CTIMER3         (1 << 13) /* Bit 13:  CTIMER3 clock control */
#define SYSCON_ASYNCAPBCLKCTRL_CTIMER4         (1 << 14) /* Bit 14:  CTIMER4 clock control */

/* Async APB clock source select A */
#define SYSCON_ASYNCAPBCLKSELA_

/* Other system configuration */

/* Brown-Out Detect control */
#define SYSCON_BODCTRL_

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_SYSCON_H */
