/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_pm.h
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

/* References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J–SAM–12/2013
 *   "Atmel SAM D21E / SAM D21G / SAM D21J SMART ARM-Based Microcontroller
 *   Datasheet", Atmel-42181E–SAM-D21_Datasheet–02/2015
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_PM_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PM register offsets ******************************************************/

#define SAM_PM_CTRL_OFFSET        0x0000  /* Control register */
#define SAM_PM_SLEEP_OFFSET       0x0001  /* Sleep mode register */
#define SAM_PM_CPUSEL_OFFSET      0x0008  /* CPU clock select register */
#define SAM_PM_APBASEL_OFFSET     0x0009  /* APBA clock select register */
#define SAM_PM_APBBSEL_OFFSET     0x000a  /* APBB clock select register */
#define SAM_PM_APBCSEL_OFFSET     0x000b  /* APBC clock select register */
#define SAM_PM_AHBMASK_OFFSET     0x0014  /* AHB mask register */
#define SAM_PM_APBAMASK_OFFSET    0x0018  /* APBA mask register */
#define SAM_PM_APBBMASK_OFFSET    0x001c  /* APBB mask register */
#define SAM_PM_APBCMASK_OFFSET    0x0020  /* APBC mask register */
#define SAM_PM_INTENCLR_OFFSET    0x0034  /* Interrupt enable clear register */
#define SAM_PM_INTENSET_OFFSET    0x0035  /* Interrupt enable set register */
#define SAM_PM_INTFLAG_OFFSET     0x0036  /* Interrupt flag status and clear register */
#define SAM_PM_RCAUSE_OFFSET      0x0038  /* Reset cause register */

/* PM register addresses ****************************************************/

#define SAM_PM_CTRL               (SAM_PM_BASE+SAM_PM_CTRL_OFFSET)
#define SAM_PM_SLEEP              (SAM_PM_BASE+SAM_PM_SLEEP_OFFSET)
#define SAM_PM_CPUSEL             (SAM_PM_BASE+SAM_PM_CPUSEL_OFFSET)
#define SAM_PM_APBASEL            (SAM_PM_BASE+SAM_PM_APBASEL_OFFSET)
#define SAM_PM_APBBSEL            (SAM_PM_BASE+SAM_PM_APBBSEL_OFFSET)
#define SAM_PM_APBCSEL            (SAM_PM_BASE+SAM_PM_APBCSEL_OFFSET)
#define SAM_PM_AHBMASK            (SAM_PM_BASE+SAM_PM_AHBMASK_OFFSET)
#define SAM_PM_APBAMASK           (SAM_PM_BASE+SAM_PM_APBAMASK_OFFSET)
#define SAM_PM_APBBMASK           (SAM_PM_BASE+SAM_PM_APBBMASK_OFFSET)
#define SAM_PM_APBCMASK           (SAM_PM_BASE+SAM_PM_APBCMASK_OFFSET)
#define SAM_PM_INTENCLR           (SAM_PM_BASE+SAM_PM_INTENCLR_OFFSET)
#define SAM_PM_INTENSET           (SAM_PM_BASE+SAM_PM_INTENSET_OFFSET)
#define SAM_PM_INTFLAG            (SAM_PM_BASE+SAM_PM_INTFLAG_OFFSET)
#define SAM_PM_RCAUSE             (SAM_PM_BASE+SAM_PM_RCAUSE_OFFSET)

/* PM register bit definitions **********************************************/

/* Control register */

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define PM_CTRL_CFDEN           (1 << 2)  /* Bit 2: Clock Failure Detector Enable */
#  define PM_CTRL_BKUPCLK         (1 << 4)  /* Bit 4: Backup Clock Select */
#endif

/* Sleep mode register */

#define PM_SLEEP_IDLE_SHIFT       (0)        /* Bits 0-1: Idle Mode Configuration */
#define PM_SLEEP_IDLE_MASK        (3 << PM_SLEEP_IDLE_SHIFT)
#  define PM_SLEEP_IDLE_CPU       (0 << PM_SLEEP_IDLE_SHIFT) /* CPU clock domain stopped */
#  define PM_SLEEP_IDLE_CPUAHB    (1 << PM_SLEEP_IDLE_SHIFT) /* CPU and AHB clock domains stopped */
#  define PM_SLEEP_IDLE_CPUAHBAPB (2 << PM_SLEEP_IDLE_SHIFT) /* CPU, AHB, and APB clock domains stopped */

/* CPU clock select register */

#define PM_CPUSEL_CPUDIV_SHIFT    (0)  /* Bits 0-2: CPU Prescaler Selection */
#define PM_CPUSEL_CPUDIV_MASK     (7 << PM_CPUSEL_CPUDIV_SHIFT)
#  define PM_CPUSEL_CPUDIV_1      (0 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 1 */
#  define PM_CPUSEL_CPUDIV_2      (1 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 2 */
#  define PM_CPUSEL_CPUDIV_4      (2 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 4 */
#  define PM_CPUSEL_CPUDIV_8      (3 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 8 */
#  define PM_CPUSEL_CPUDIV_16     (4 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 16 */
#  define PM_CPUSEL_CPUDIV_32     (5 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 32 */
#  define PM_CPUSEL_CPUDIV_64     (6 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 64 */
#  define PM_CPUSEL_CPUDIV_128    (7 << PM_CPUSEL_CPUDIV_SHIFT) /* Divide by 128 */

/* APBA clock select register */

#define PM_APBASEL_APBADIV_SHIFT  (0)  /* Bits 0-2: APBA Prescaler Selection */
#define PM_APBASEL_APBADIV_MASK   (7 << PM_APBASEL_APBADIV_SHIFT)
#  define PM_APBASEL_APBADIV_1    (0 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 1 */
#  define PM_APBASEL_APBADIV_2    (1 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 2 */
#  define PM_APBASEL_APBADIV_4    (2 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 4 */
#  define PM_APBASEL_APBADIV_8    (3 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 8 */
#  define PM_APBASEL_APBADIV_16   (4 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 16 */
#  define PM_APBASEL_APBADIV_32   (5 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 32 */
#  define PM_APBASEL_APBADIV_64   (6 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 64 */
#  define PM_APBASEL_APBADIV_128  (7 << PM_APBASEL_APBADIV_SHIFT) /* Divide by 128 */

/* APBB clock select register */

#define PM_APBBSEL_APBBDIV_SHIFT  (0)  /* Bits 0-2: APBB Prescaler Selection */
#define PM_APBBSEL_APBBDIV_MASK   (7 << PM_APBBSEL_APBBDIV_SHIFT)
#  define PM_APBBSEL_APBBDIV_1    (0 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 1 */
#  define PM_APBBSEL_APBBDIV_2    (1 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 2 */
#  define PM_APBBSEL_APBBDIV_4    (2 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 4 */
#  define PM_APBBSEL_APBBDIV_8    (3 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 8 */
#  define PM_APBBSEL_APBBDIV_16   (4 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 16 */
#  define PM_APBBSEL_APBBDIV_32   (5 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 32 */
#  define PM_APBBSEL_APBBDIV_64   (6 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 64 */
#  define PM_APBBSEL_APBBDIV_128  (7 << PM_APBBSEL_APBBDIV_SHIFT) /* Divide by 128 */

/* APBC clock select register */

#define PM_APBCSEL_APBCDIV_SHIFT  (0)  /* Bits 0-2: APBC Prescaler Selection */
#define PM_APBCSEL_APBCDIV_MASK   (7 << PM_APBCSEL_APBCDIV_SHIFT)
#  define PM_APBCSEL_APBCDIV_1    (0 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 1 */
#  define PM_APBCSEL_APBCDIV_2    (1 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 2 */
#  define PM_APBCSEL_APBCDIV_4    (2 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 4 */
#  define PM_APBCSEL_APBCDIV_8    (3 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 8 */
#  define PM_APBCSEL_APBCDIV_16   (4 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 16 */
#  define PM_APBCSEL_APBCDIV_32   (5 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 32 */
#  define PM_APBCSEL_APBCDIV_64   (6 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 64 */
#  define PM_APBCSEL_APBCDIV_128  (7 << PM_APBCSEL_APBCDIV_SHIFT) /* Divide by 128 */

/* AHB mask register */

#define PM_AHBMASK_HPB0           (1 << 0)  /* Bit 0:  HPB0 */
#define PM_AHBMASK_HPB1           (1 << 1)  /* Bit 1:  HPB1 */
#define PM_AHBMASK_HPB2           (1 << 2)  /* Bit 2:  HPB2 */
#define PM_AHBMASK_DSU            (1 << 3)  /* Bit 3:  DSU */
#define PM_AHBMASK_NVMCTRL        (1 << 4)  /* Bit 4:  NVMCTRL  */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define PM_AHBMASK_DMAC         (1 << 5)  /* Bit 4:  DMA controller  */
#  define PM_AHBMASK_USB          (1 << 6)  /* Bit 4:  USB  */
#endif

/* APBA mask register */

#define PM_APBAMASK_PAC0          (1 << 0)  /* Bit 0:  PAC0 */
#define PM_APBAMASK_PM            (1 << 1)  /* Bit 1:  PM */
#define PM_APBAMASK_SYSCTRL       (1 << 2)  /* Bit 2:  SYSCTRL */
#define PM_APBAMASK_GCLK          (1 << 3)  /* Bit 3:  GCLK */
#define PM_APBAMASK_WDT           (1 << 4)  /* Bit 4:  WDT */
#define PM_APBAMASK_RTC           (1 << 5)  /* Bit 5:  RTC */
#define PM_APBAMASK_EIC           (1 << 6)  /* Bit 6:  EIC */

/* APBB mask register */

#define PM_APBBMASK_PAC1          (1 << 0)  /* Bit 0:  PAC1 */
#define PM_APBBMASK_DSU           (1 << 1)  /* Bit 1:  DSU */
#define PM_APBBMASK_NVMCTRL       (1 << 2)  /* Bit 2:  NVMCTRL */
#define PM_APBBMASK_PORT          (1 << 3)  /* Bit 3:  PORT */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define PM_APBBMASK_DMAC        (1 << 4)  /* Bit 4:  DMA controller */
#  define PM_APBBMASK_USB         (1 << 5)  /* Bit 5:  USB  */
#endif

/* APBC mask register */

#define PM_APBCMASK_PAC2          (1 << 0)  /* Bit 0:  PAC2 */
#define PM_APBCMASK_EVSYS         (1 << 1)  /* Bit 1:  EVSYS */
#define PM_APBCMASK_SERCOM(n)     (1 << ((n)+2))
#  define PM_APBCMASK_SERCOM0     (1 << 2)  /* Bit 2:  SERCOM0 */
#  define PM_APBCMASK_SERCOM1     (1 << 3)  /* Bit 3:  SERCOM1 */
#  define PM_APBCMASK_SERCOM2     (1 << 4)  /* Bit 4:  SERCOM2 */
#  define PM_APBCMASK_SERCOM3     (1 << 5)  /* Bit 5:  SERCOM3 */
#  define PM_APBCMASK_SERCOM4     (1 << 6)  /* Bit 6:  SERCOM4 */
#  define PM_APBCMASK_SERCOM5     (1 << 7)  /* Bit 7:  SERCOM5 */

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define PM_APBCMASK_TC0         (1 << 8)  /* Bit 8:  TC0 */
#  define PM_APBCMASK_TC1         (1 << 9)  /* Bit 9:  TC1 */
#  define PM_APBCMASK_TC2         (1 << 10) /* Bit 10: TC2 */
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define PM_APBCMASK_TCC0        (1 << 8)  /* Bit 8:  TCC0 */
#  define PM_APBCMASK_TCC1        (1 << 9)  /* Bit 9:  TCC1 */
#  define PM_APBCMASK_TCC2        (1 << 10) /* Bit 10: TCC2 */
#endif

#define PM_APBCMASK_TC3           (1 << 11) /* Bit 11: TC3 */
#define PM_APBCMASK_TC4           (1 << 12) /* Bit 12: TC4 */
#define PM_APBCMASK_TC5           (1 << 13) /* Bit 13: TC5 */
#define PM_APBCMASK_TC6           (1 << 14) /* Bit 14: TC6 */
#define PM_APBCMASK_TC7           (1 << 15) /* Bit 15: TC7 */
#define PM_APBCMASK_ADC           (1 << 16) /* Bit 16: ADC */
#define PM_APBCMASK_AC            (1 << 17) /* Bit 17: AC */
#define PM_APBCMASK_DAC           (1 << 18) /* Bit 18: DAC */
#define PM_APBCMASK_PTC           (1 << 19) /* Bit 19: PTC */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define PM_APBCMASK_I2S         (1 << 20) /* Bit 20: Inter IC Sound */
#endif

/* Interrupt enable clear, Interrupt enable set,
 * and Interrupt flag status and clear registers
 */

#define PM_INT_CKRDY              (1 << 0)  /* Bit 0: Clock Ready Interrupt */

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define PM_INT_CFD              (1 << 1)  /* Bit 1: Clock Failure Detector Interrupt */
#endif

/* Reset cause register */

#define PM_RCAUSE_POR             (1 << 0)  /* Bit 0: Power-On Reset */
#define PM_RCAUSE_BOD12           (1 << 1)  /* Bit 1: Brown Out 12 Detector Reset */
#define PM_RCAUSE_BOD33           (1 << 2)  /* Bit 2: Brown Out 33 Detector Reset */
#define PM_RCAUSE_EXT             (1 << 4)  /* Bit 4: External Reset */
#define PM_RCAUSE_WDT             (1 << 5)  /* Bit 5: Watchdog Reset */
#define PM_RCAUSE_SYST            (1 << 6)  /* Bit 6: System Reset Request */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_PM_H */
