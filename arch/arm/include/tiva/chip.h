/****************************************************************************
 * arch/arm/include/tiva/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_TIVA_CHIP_H
#define __ARCH_ARM_INCLUDE_TIVA_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_LM3S6918)
#  define LM3S                 1  /* LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96) || \
      defined(CONFIG_ARCH_CHIP_LM3S9B92)
#  define LM3S                 1  /* LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  define TIVA_NQEI            1  /* One quadrature encoder */
#  define TIVA_NPORTS          15 /* Fifteen Ports (GPIOA-H, J-N, P-Q) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       1  /* One USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PM)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PZ)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  define TIVA_NQEI            2  /* Two quadrature encoders */
#  define TIVA_NPORTS         10  /* Ten Ports (GPIOA-K) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       1  /* One USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PGE)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  define TIVA_NQEI            2  /* Two quadrature encoders */
#  define TIVA_NPORTS         14  /* Fourteen Ports (GPIOA-P) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       1  /* One USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C123AH6PM)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  define TIVA_NQEI            2  /* Two quadrature encoders */
#  define TIVA_NPORTS          7  /* Seven Ports (GPIOA-G) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C1294NCPDT)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  define TIVA_NQEI            1  /* One quadrature encoder */
#  define TIVA_NPORTS          15 /* Fifteen Ports (GPIOA-H, J-N, P-Q) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       1  /* One USB 2.0 OTG HS */
#  define TIVA_NCRC            1  /* One CRC module */
#  define TIVA_NAES            0  /* No AES module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C129ENCPDT)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#  define TIVA_NQEI            1  /* One quadrature encoder */
#  define TIVA_NPORTS          15 /* Fifteen Ports (GPIOA-H, J-N, P-Q) */
#  define TIVA_NCANCONTROLLER  2  /* Two CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       1  /* One USB 2.0 OTG HS */
#  define TIVA_NCRC            1  /* One CRC module */
#  define TIVA_NAES            1  /* One AES module */
#  define TIVA_NDES            1  /* One DES module */
#  define TIVA_NHASH           1  /* One SHA1/MD5 hash module */
#elif defined(CONFIG_ARCH_CHIP_TM4C129XNCZAD) || defined(CONFIG_ARCH_CHIP_TM4C129ENCZAD)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  define TM4C                 1  /* TM4C family */
#  undef  SIMPLELINK              /* Not SimpleLink family */
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
#elif defined(CONFIG_ARCH_CHIP_CC13X0)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define SIMPLELINK           1  /* SimpleLink family */
#  define TIVA_NTIMERS         4  /* Eight 16- or four 32-bit GPTM timers */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            1  /* One watchdog timer */
                                  /* One RF timer */
#  define TIVA_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  define TIVA_NSSI            2  /* Two SSI modules */
#  define TIVA_NUARTS          1  /* One UART module */
#  define TIVA_NI2C            1  /* One I2C module */
                                  /* One I2S module */
#  define TIVA_NADC            1  /* One 12-bit, 8 channel ADC module */
                                  /* One continuous time comparator */
                                  /* Ultra low power clocked comparator */
                                  /* Programmable current source */
#  define TIVA_NPWM            0  /* No PWM generator modules */
#  define TIVA_NQEI            0  /* No quadrature encoder modules */
#  define TIVA_NPORTS          1  /* One Port */
#  define TIVA_NCANCONTROLLER  0  /* No CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            1  /* One AES-128 module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           0  /* No SHA1/MD5 hash module */
                                  /* One TRNG module */
                                  /* Capacitive sensing, up to 8 channels */
                                  /* Integrated temperatore sensor */
#elif defined(CONFIG_ARCH_CHIP_CC13X2)
#  undef  LM3S                    /* Not LM3S family */
#  undef  LM4F                    /* Not LM4F family */
#  undef  TM4C                    /* Not TM4C family */
#  define SIMPLELINK           1  /* SimpleLink family */
#  define TIVA_NTIMERS         4  /* Eight 16- or four 32-bit GPTM timers */
#  define TIVA_NWIDETIMERS     0  /* No 32/64-bit timers */
#  define TIVA_NWDT            1  /* One watchdog timer */
#  define TIVA_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define TIVA_NLCD            0  /* No LCD controller */
#  define TIVA_NSSI            2  /* Two SSI modules */
#  define TIVA_NUARTS          2  /* Two UART module */
#  define TIVA_NI2C            1  /* One I2C module */
                                  /* One I2S module */
                                  /* One RTC */
#  define TIVA_NADC            1  /* One 12-bit, 8 channel ADC module */
                                  /* Two comparators with reference DAC */
#  define TIVA_NPWM            0  /* No PWM generator modules */
#  define TIVA_NQEI            0  /* No quadrature encoder modules */
#  define TIVA_NPORTS          1  /* One Port */
#  define TIVA_NCANCONTROLLER  0  /* No CAN controllers */
#  define TIVA_NUSBOTGFS       0  /* No USB 2.0 OTG FS */
#  define TIVA_NUSBOTGHS       0  /* No USB 2.0 OTG HS */
#  define TIVA_NCRC            0  /* No CRC module */
#  define TIVA_NAES            1  /* One AES-256 module */
#  define TIVA_NDES            0  /* No DES module */
#  define TIVA_NHASH           1  /* SHA2 Accelerator (up to SHA-512) */
                                  /* ECC and RSA Public Key Hardware Accelerator */
                                  /* One TRNG module */
                                  /* Capacitive sensing, up to 8 channels */
                                  /* Integrated temperature and battery monitor */
#else
#  error "Capabilities not specified for this TIVA/Stellaris chip"
#endif

/* The TIVA/Stellaris only supports 8 priority levels.  The hardware
 * priority mechanism will only look at the upper N bits of the 8-bit
 * priority level (where N is 3 for the Tiva/Stellaris family), so any
 * prioritization must be performed in those bits.  The default priority
 * level is set to the middle value
 */

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* Bits [7:5] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20 /* Three bits of interrupt priority used */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_TIVA_CHIP_H */
