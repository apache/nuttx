/************************************************************************************
 * arch/arm/include/tiva/chip.h
 *
 *   Copyright (C) 2009-2010, 2013-2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jose Pablo Carballo <jcarballo@nx-engineering.com>
 *            Jim Ewing <jim@droidifi.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_TIVA_CHIP_H
#define __ARCH_ARM_INCLUDE_TIVA_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip (only the LM3S6918 and 65 right now) */

#if defined(CONFIG_ARCH_CHIP_LM3S6918)
#  define LM3S                 1  /* LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define TIVA_NTIMERS         4  /* Four 16/32-bit timers */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            1  /* One watchdog timer */
#  define TIVA_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  undef  TIVA_ETHTS              /* No timestamp register */
#  define TIVA_NSSI            2  /* Two SSI modules */
#  define TIVA_NUARTS          2  /* Two UART modules */
#  define TIVA_NI2C            2  /* Two I2C modules */
#  define TIVA_NADC            1  /* One ADC module */
#  define TIVA_NPWM            0  /* No PWM generator modules */
#  define TIVA_NQEI            0  /* No quadrature encoders */
#  define TIVA_NPORTS          8  /* 8 Ports (GPIOA-H) 5-38 GPIOs */
#  define TIVA_NCANCONTROLLER  0  /* No CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_LM3S6432)
#  define LM3S                 1  /* LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define TIVA_NTIMERS         3  /* Three 16/32-bit timers */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            1  /* One watchdog timer */
#  define TIVA_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  undef  TIVA_ETHTS              /* No timestamp register */
#  define TIVA_NSSI            1  /* One SSI module */
#  define TIVA_NUARTS          2  /* Two UART modules */
#  define TIVA_NI2C            1  /* Two I2C modules */
#  define TIVA_NADC            1  /* One ADC module */
#  define TIVA_NPWM            1  /* One PWM generator module */
#  define TIVA_NQEI            0  /* No quadrature encoders */
#  define TIVA_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#  define TIVA_NCANCONTROLLER  0  /* No CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_LM3S6965)
#  define LM3S                 1  /* LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define TIVA_NTIMERS         4  /* Four 16/32-bit timers */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            1  /* One watchdog timer */
#  define TIVA_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  undef  TIVA_ETHTS              /* No timestamp register */
#  define TIVA_NSSI            1  /* One SSI module */
#  define TIVA_NUARTS          3  /* Three UART modules */
#  define TIVA_NI2C            2  /* Two I2C modules */
#  define TIVA_NADC            1  /* One ADC module */
#  define TIVA_NPWM            3  /* Three PWM generator modules */
#  define TIVA_NQEI            2  /* Two quadrature encoders */
#  define TIVA_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#  define TIVA_NCANCONTROLLER  0  /* No CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96)
#  define LM3S                 1  /* LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define TIVA_NTIMERS         4  /* Four 16/32-bit timers */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            1  /* One watchdog timer */
#  define TIVA_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  undef  TIVA_ETHTS              /* No timestamp register */
#  define TIVA_NSSI            2  /* Two SSI modules */
#  define TIVA_NUARTS          3  /* Three UART modules */
#  define TIVA_NI2C            2  /* Two I2C modules */
#  define TIVA_NADC            2  /* Two ADC module */
#  define TIVA_CAN             2  /* Two CAN module */
#  define TIVA_NPWM            4  /* Four PWM generator modules */
#  define TIVA_NQEI            2  /* Two quadrature encoders */
#  define TIVA_NPORTS          9  /* 9 Ports (GPIOA-H,J) 0-65 GPIOs */
#  define TIVA_NCANCONTROLLER  0  /* No CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_LM3S8962)
#  define LM3S                 1  /* LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define TIVA_NTIMERS         6  /* Four 16/32-bit timers */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            1  /* One watchdog timer */
#  define TIVA_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  define TIVA_NSSI            1  /* One SSI module */
#  define TIVA_NUARTS          3  /* Two UART modules */
#  define TIVA_NI2C            2  /* One I2C module */
#  define TIVA_NADC            1  /* One ADC module */
#  define TIVA_NPWM            3  /* Three PWM generator modules */
#  define TIVA_NQEI            2  /* Two quadrature encoders */
#  define TIVA_NPORTS          7  /* 7 Ports (GPIOA-G), 5-42 GPIOs */
#  define TIVA_NCANCONTROLLER  1  /* One CAN controller */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_LM4F120)
#  undef  LM3S                    /* Not LM3S family */
#  define LM4F                 1  /* LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define TIVA_NTIMERS         6  /* Six 16/32-bit timers */
#  define TIVA_NWIDETIMERS     6  /* Six 32/64-bit timers */
#  define TIVA_NWDT            2  /* Two watchdog timer timers */
#  define TIVA_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  define TIVA_NSSI            4  /* Four SSI module */
#  define TIVA_NUARTS          8  /* Eight UART modules */
#  define TIVA_NI2C            4  /* Four I2C modules */
#  define TIVA_NADC            2  /* Two ADC modules */
#  define TIVA_NPWM            0  /* No PWM generator modules */
#  define TIVA_NQEI            0  /* No quadrature encoders */
#  define TIVA_NPORTS          6  /* 6 Ports (GPIOA-F), 0-43 GPIOs */
#  define TIVA_NCANCONTROLLER  1  /* One CAN controller */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PGE) || defined(CONFIG_ARCH_CHIP_TM4C123GH6PZ) || \
      defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  define TIVA_NTIMERS         6  /* Six 16/32-bit timers */
#  define TIVA_NWIDETIMERS     6  /* Six 32/64-bit timers */
#  define TIVA_NWDT            2  /* Two watchdog timers */
#  define TIVA_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  define TIVA_NSSI            4  /* Four SSI module */
#  define TIVA_NUARTS          8  /* Eight UART modules */
#  define TIVA_NI2C            6  /* Six I2C modules */
#  define TIVA_NADC            2  /* Two ADC modules */
#  define TIVA_NPWM            2  /* Two PWM generator modules */
#  define TIVA_NQEI            1  /* One quadrature encoders */
#  define TIVA_NPORTS          15 /* Fifteen Ports (GPIOA-H, J-N, P-Q) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       1  /* One USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PMI)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  define TIVA_NTIMERS         6  /* Six 16/32-bit timers */
#  define TIVA_NWIDETIMERS     6  /* Six 32/64-bit timers */
#  define TIVA_NWDT            2  /* Two watchdog timers */
#  define TIVA_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  define TIVA_NSSI            4  /* Four SSI module */
#  define TIVA_NUARTS          8  /* Eight UART modules */
#  define TIVA_NI2C            4  /* Four I2C modules */
#  define TIVA_NADC            2  /* Two ADC modules */
#  define TIVA_NPWM            2  /* Two PWM generator modules */
#  define TIVA_NQEI            2  /* Two quadrature encoders */
#  define TIVA_NPORTS          6  /* Six Ports (GPIOA-F) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       1  /* One USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C1294NC)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  define TIVA_NTIMERS         8  /* Eight Dual 16/32-bit timers A/B */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            2  /* Two watchdog timers */
#  define TIVA_NETHCONTROLLERS 1  /* One 10/100Mbit Ethernet controller */
#  define TIVA_NLCD            1  /* One LCD controller */
#  define TIVA_NSSI            4  /* Four SSI modules */
#  define TIVA_NUARTS          8  /* Eight UART modules */
#  define TIVA_NI2C            10 /* Ten I2C modules */
#  define TIVA_NADC            2  /* Two ADC modules */
#  define TIVA_NPWM            4  /* Four PWM generator modules */
#  define TIVA_NQEI            1  /* One quadrature encoders */
#  define TIVA_NPORTS          15 /* Fifteen Ports (GPIOA-H, J-N, P-Q) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       1  /* One USB 2.0 OTG HS */
#  define TIVA_NCRC            1  /* One CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  define TIVA_NTIMERS         8  /* Eight Dual 16/32-bit timers A/B */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            2  /* Two watchdog timers */
#  define TIVA_NETHCONTROLLERS 1  /* One 10/100Mbit Ethernet controller */
#  define TIVA_NLCD            1  /* One LCD controller */
#  define TIVA_NSSI            4  /* Four SSI modules */
#  define TIVA_NUARTS          8  /* Eight UART modules */
#  define TIVA_NI2C            10 /* Ten I2C modules */
#  define TIVA_NADC            2  /* Two ADC modules */
#  define TIVA_NPWM            4  /* Four PWM generator modules */
#  define TIVA_NQEI            1  /* One quadrature encoder module */
#  define TIVA_NPORTS          18 /* Eighteen Ports (GPIOA-H, J-N, P-T) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       1  /* One USB 2.0 OTG HS */
#  define TIVA_NCRC            1  /* One CRC module */
#  define TIVA_NAES            1  /* One AES module */
#  define TIVA_NDES            1  /* One DES module */
#  define TIVA_NHASH           1  /* One SHA1/MD5 hash module */
#else
#  error "Capabilities not specified for this TIVA/Stellaris chip"
#endif

/* The TIVA/Stellaris only supports 8 priority levels.  The hardware priority
 * mechanism will only look at the upper N bits of the 8-bit priority level
 * (where N is 3 for the Tiva/Stellaris family), so any prioritization must be
 * performed in those bits.  The default priority level is set to the middle
 * value
 */

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* Bits [7:5] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20 /* Three bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the up_irq_save() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_TIVA_CHIP_H */
