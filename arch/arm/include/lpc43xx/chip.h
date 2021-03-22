/****************************************************************************
 * arch/arm/include/lpc43xx/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_LPC43XX_CHIP_H
#define __ARCH_ARM_INCLUDE_LPC43XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Per the data sheet: LPC4350/30/20/10 Rev. 3.2 — 4 June 2012 */

/* Get customizations for each supported chip.
 *
 * SRAM Resources
 * --------------------- -------- ------- ------- -------
 * Local SRAM            LPC4310  LPC4320 LPC4330 LPC4350
 * --------------------- -------- ------- ------- -------
 * BANK 0 (0x1000 0000)     96Kb   128Kb   128Kb   128Kb
 * BANK 1 (0x1008 0000)     40Kb    40Kb    72Kb    72Kb
 * --------------------- -------- ------- ------- -------
 * SUBTOTAL                136Kb   168Kb   200Kb   200Kb
 * --------------------- -------- ------- ------- -------
 * AHB SRAM              LPC4310  LPC4320 LPC4330 LPC4350
 * --------------------- -------- ------- ------- -------
 * BANK 0 (0x2000 0000)     16Kb    48Kb   48Kb    48Kb
 * BANK 1 (0x2000 8000)             NOTE 1 NOTE 1  NOTE 1
 * BANK 2 (0x2000 c000)     16Kb    16Kb   16Kb    16Kb
 * --------------------- -------- ------- ------- -------
 * SUBTOTAL                 32Kb    32Kb   64Kb    64Kb
 * --------------------- -------- ------- ------- -------
 * TOTAL                   168Kb   200Kb  264Kb   264Kb
 * --------------------- -------- ------- ------- -------
 *
 * NOTE 1: The 64Kb of AHB of SRAM on the LPC4350/30/20 span all AHB SRAM
 * banks but are treated as two banks of 48 an 16Kb by the NuttX memory
 * manager.  This gives some symmetry to all of the members of the family.
 */

/* Per the user manual: UM10503, Rev. 1.2 — 8 June 2012 */

/* Get customizations for each supported chip.
 *
 * SRAM Resources
 * --------------- -------- ------- ------- ------- ------- ------- -------
 * Local SRAM      LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357 LPC4337
 * --------------- -------- ------- ------- ------- ------- ------- -------
 * BANK 0          96Kb    96Kb   128Kb   128Kb    32Kb    32Kb    32Kb
 * (0x1000 0000)
 * BANK 1          40Kb    40Kb    72Kb    72Kb    40Kb    40Kb    40Kb
 * (0x1008 0000)
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * SUBTOTAL        136Kb   136Kb   200Kb   200Kb    72Kb    72Kb    72Kb
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * AHB SRAM        LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357 LPC4337
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * BANK 0          16Kb    48Kb   48Kb    48Kb     48Kb    48Kb    48Kb
 * (0x2000 0000)
 * BANK 1                   NOTE 1 NOTE 1  NOTE 1  NOTE 1  NOTE 1  NOTE 1
 * (0x2000 8000)
 * BANK 2          16Kb    16Kb   16Kb    16Kb    16Kb    16Kb     16Kb
 * (0x2000 c000)
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * SUBTOTAL        32Kb    64Kb   64Kb    64Kb     64Kb    64Kb    64Kb
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * TOTAL           168Kb   200Kb  264Kb   264Kb    136Kb   136Kb   136Kb
 * -------------- -------- ------- ------- ------- ------- ------- -------
 *
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * FLASH          LPC4310  LPC4320 LPC4330 LPC4350 LPC4353 LPC4357 LPC4337
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * BANK A                                           256Kb   512Kb   512Kb
 * (0x1a00 0000)
 * BANK B                                           256Kb   512Kb   512Kb
 * (0x1b00 8000)
 * -------------- -------- ------- ------- ------- ------- ------- -------
 * TOTAL            None    None    None    None    512Kb  1024Kb   1024Kb
 * -------------- -------- ------- ------- ------- ------- ------- -------

 *
 * NOTE 1: The 64Kb of AHB of SRAM on the LPC4350/30/20 span all AHB SRAM
 * banks but are treated as two banks of 48 an 16Kb by the NuttX memory
 * manager.  This gives some symmetry to all of the members of the family.
 */

#if defined(CONFIG_ARCH_CHIP_LPC4310FBD144)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (96*1024)   /* 136Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  undef  LPC43_USB0                           /* No USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4310FET100)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (96*1024)   /* 136Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  undef  LPC43_USB0                           /* No USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  undef  LPC43_MCPWM                          /* No PWM capability */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (4)         /* Four ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4320FBD144)
#  warning "Data sheet and user manual are consistement for the LPC4320"
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 168Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4320FET100)
#  warning "Data sheet and user manual are consistement for the LPC4320"
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 168Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (16*1024)   /* 32Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  undef  LPC43_ETHERNET                       /* No Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  undef  LPC43_USB1                           /* No USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  undef  LPC43_MCPWM                          /* No PWM capability */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (4)         /* Four ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FBD144)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET100)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  undef  LPC43_MCPWM                          /* No PWM capability */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (4)         /* Four ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET180)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET256)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4337JBD144)
#  define LPC43_FLASH_BANKA_SIZE   (512*1024)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (0)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (0)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (0)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4337FET256)
#  define LPC43_FLASH_BANKA_SIZE   (512*1024)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4350FBD208)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  define LPC43_NLCD               (1)         /* One LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET180)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  define LPC43_NLCD               (1)         /* One LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET256)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  define LPC43_NLCD               (1)         /* One LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4353FBD208)
#  define LPC43_FLASH_BANKA_SIZE   (256*1024)  /* 512Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (256*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET180)
#  define LPC43_FLASH_BANKA_SIZE   (256*1024)  /* 512Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (256*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET256)
#  define LPC43_FLASH_BANKA_SIZE   (256*1024)  /* 512Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (256*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET180)
#  define LPC43_FLASH_BANKA_SIZE   (512*1024)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4357FBD208)
#  define LPC43_FLASH_BANKA_SIZE   (512*1024)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET256)
#  define LPC43_FLASH_BANKA_SIZE   (512*1024)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (1)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels */
#  undef  LPC43_NADC12                         /* No 12-bit ADC controllers */
#elif defined(CONFIG_ARCH_CHIP_LPC4370FET100)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM (plus 18Kb for Cortex-M0)*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (32*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (16*1024)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  undef  LPC43_NLCD                           /* No LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  undef  LPC43_USB1_ULPI                      /* No USB1 (Host, Device) with ULPI I/F */
#  undef  LPC43_MCPWM                          /* No PWM capability */
#  undef  LPC43_QEI                            /* No Quadrature Encoder capability */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two C-CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10             (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels (per ADC)*/
#  define LPC43_NADC12             (1)         /* ONne 12-bit ADC controllers (ADCHS)*/
#elif defined(CONFIG_ARCH_CHIP_LPC4370FET256)
#  define LPC43_FLASH_BANKA_SIZE   (0)         /* Flashless */
#  define LPC43_FLASH_BANKB_SIZE   (0)
#  define LPC43_LOCSRAM_BANK0_SIZE (128*1024)  /* 200Kb Local SRAM (plus 18Kb for Cortex-M0)*/
#  define LPC43_LOCSRAM_BANK1_SIZE (72*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (32*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (16*1024)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (0)         /* No EEPROM */
#  define LPC43_NLCD               (1)         /* One LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (1)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (1)         /* One PWM interface */
#  define LPC43_QEI                (1)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two C-CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC10_CHANNELS    (8)         /* Eight ADC channels (per ADC)*/
#  define LPC43_NADC12             (1)         /* ONne 12-bit ADC controllers (ADCHS)*/
#elif defined(CONFIG_ARCH_CHIP_LPC4337JET100)
#  define LPC43_FLASH_BANKA_SIZE   (512*1024)  /* 1024Kb FLASH */
#  define LPC43_FLASH_BANKB_SIZE   (512*1024)
#  define LPC43_LOCSRAM_BANK0_SIZE (32*1024)   /* 72Kb Local SRAM */
#  define LPC43_LOCSRAM_BANK1_SIZE (40*1024)
#  define LPC43_AHBSRAM_BANK0_SIZE (48*1024)   /* 64Kb AHB SRAM */
#  define LPC43_AHBSRAM_BANK1_SIZE (0)
#  define LPC43_AHBSRAM_BANK2_SIZE (16*1024)
#  define LPC43_EEPROM_SIZE        (16*1024)   /* 16Kb EEPROM */
#  define LPC43_NLCD               (0)         /* Has LCD controller */
#  define LPC43_ETHERNET           (1)         /* One Ethernet controller */
#  define LPC43_USB0               (1)         /* Have USB0 (Host, Device, OTG) */
#  define LPC43_USB1               (1)         /* Have USB1 (Host, Device) */
#  define LPC43_USB1_ULPI          (0)         /* Have USB1 (Host, Device) with ULPI I/F */
#  define LPC43_MCPWM              (0)         /* One PWM interface */
#  define LPC43_QEI                (0)         /* One Quadrature Encoder interface */
#  define LPC43_NUSARTS            (4)         /* Three USARTs + 1 UART */
#  define LPC43_NSSP               (2)         /* Two SSP controllers */
#  define LPC43_NTIMERS            (4)         /* Four Timers */
#  define LPC43_NI2C               (2)         /* Two I2C controllers */
#  define LPC43_NI2S               (2)         /* Two I2S controllers */
#  define LPC43_NCAN               (2)         /* Two CAN controllers */
#  define LPC43_NDAC               (1)         /* One 10-bit DAC */
#  define LPC43_NADC               (2)         /* Two 10-bit ADC controllers */
#  define LPC43_NADC_CHANNELS      (4)         /* Four ADC channels */
#else
#  error "Unsupported LPC43xx chip"
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-31. The lower the value, the
 * greater the priority of the corresponding interrupt.
 *
 * The Cortex-M4 core supports up to 53 interrupts an 8 prgrammable interrupt
 * priority levels; The Cortex-M0 core supports up to 32 interrupts with 4
 * programmable interrupt priorities.
 */

#define LPC43M4_SYSH_PRIORITY_MIN     0xe0 /* All bits[7:5] set is minimum priority */
#define LPC43M4_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define LPC43M4_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define LPC43M4_SYSH_PRIORITY_STEP    0x20 /* Steps between priorities */

#define LPC43M0_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define LPC43M0_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define LPC43M0_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define LPC43M0_SYSH_PRIORITY_STEP    0x40 /* Steps between priorities */

/* Only the Cortex-M4 is supported by NuttX */

#define NVIC_SYSH_PRIORITY_MIN        LPC43M4_SYSH_PRIORITY_MIN
#define NVIC_SYSH_PRIORITY_DEFAULT    LPC43M4_SYSH_PRIORITY_DEFAULT
#define NVIC_SYSH_PRIORITY_MAX        LPC43M4_SYSH_PRIORITY_MAX
#define NVIC_SYSH_PRIORITY_STEP       LPC43M4_SYSH_PRIORITY_STEP

#endif /* __ARCH_ARM_INCLUDE_LPC43XX_CHIP_H */
