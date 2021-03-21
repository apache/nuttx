/****************************************************************************
 * arch/arm/include/max326xx/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_MAX326XX_CHIP_H
#define __ARCH_ARM_INCLUDE_MAX326XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported MAX326xx family.  Only sizes and
 * numbers of things are provided here.  See arch/arm/src/max326xx/Kconfig
 * for other, boolean configuration settings.
 *
 * MAX326xx Families are determined by sharing a common User Guide for the
 * chip specification:
 *
 *   MAX32620/32621 Family:  MAX32620 Rev C, User Guide, AN6242, Rev 2, 2/17
 *   MAX32630/32632 Family:  MAX32630 Rev B, User Guide, AN6349, Rev 0, 10/16
 *   MAX32660 Family:        MAX32660 User Guide, AN6659, Rev0, 7/18
 */

/* MAX32620/32621 Family:
 *
 *   Part           Flash  SRAM   Trust      Pin/Package
 *                  (Mb)   (Kb)   Protection
 *   MAX32620ICQ+    2     256     No        100 TQFP
 *   MAX32620IWG+    2     256     No         81 WLP
 *   MAX32620IWG+T   2     256     No         81 WLP
 *   MAX32620IWGL+   1     256     No         81 WLP
 *   MAX32620IWGL+T  1     256     No         81 WLP
 *   MAX32621ICQ+    2     256     Yes       100 TQFP
 *   MAX32621IWG+    2     256     Yes        81 WLP
 *   MAX32621IWG+T   2     256     Yes        81 WLP
 */

#if defined(CONFIG_ARCH_CHIP_MAX32620) || defined(CONFIG_ARCH_CHIP_MAX32621)

/* Peripherals */

#  define MAX326_NWDOG       0    /* No Watchdog Timers */
#  define MAX326_NWWDOG      2    /* Two Windowed Watchdog Timers */
#  define MAX326_NRWDOG      1    /* One Recovery Watchdog Timer */
#  define MAX326_NWAKEUP     1    /* One Wakeup Timer */
#  define MAX326_NRTC        1    /* One RTC */
#  define MAX326_NCRC        1    /* One CRC16/32 */
#  define MAX326_NAES        1    /* One AES 128,192, 256 */
#  define MAX326_NUSB20      1    /* One USB 2.0 device */
#  define MAX326_NTMR32      6    /* Six 32-bit Timers */
#  define MAX326_NTMR8       0    /* No 8-bit Timers */
#  define MAX326_NPTENGINE  16    /* Sixteen pulse train engines */
#  define MAX326_NSPIM       3    /* Three SPI master */
#  define MAX326_NSPIS       1    /* One SPI slave */
#  define MAX326_NSPIXIP     1    /* One SPI XIP */
#  define MAX326_NI2SS       0    /* No I2S slave */
#  define MAX326_NI2CM       3    /* Three I2C master */
#  define MAX326_NI2CS       1    /* One I2C slave */
#  define MAX326_NUART       4    /* Four UARTs */
#  define MAX326_N1WIREM     1    /* One 1-Wire master */
#  define MAX326_NADC10      1    /* One 10-bit ADC */

/* MAX32630/32632 Family:
 *
 *   Part           Flash  SRAM   Trust      Secure     Pin/Package
 *                  (Mb)   (Kb)   Protection Bootloader
 *   MAX32630IWQ+    2     512    No         No         100 WLP
 *   MAX32630IWQ+T   2     512    No         No         100 WLP
 *   MAX32630ICQ+    2     512    No         No         100 TQFP-EP
 *   MAX32631IWQ+    2     512    Yes        No         100 WLP
 *   MAX32631IWQ+T   2     512    Yes        No         100 WLP
 *   MAX32631ICQ+    2     512    Yes        No         100 TQFP-EP
 *   MAX32632IWQ+    2     512    Yes        Yes        100 WLP
 *   MAX32632IWQ+T   2     512    Yes        Yes        100 WLP
 */

#elif defined(CONFIG_ARCH_CHIP_MAX32630) || defined(CONFIG_ARCH_CHIP_MAX32632)

/* Peripherals */

#  define MAX326_NWDOG       0    /* No Watchdog Timers */
#  define MAX326_NWWDOG      2    /* Two Windowed Watchdog Timers */
#  define MAX326_NRWDOG      0    /* No Recovery Watchdog Timer */
#  define MAX326_NWAKEUP     1    /* One Wakeup Timer */
#  define MAX326_NRTC        1    /* One RTC */
#  define MAX326_NCRC        1    /* One CRC16/32 */
#  define MAX326_NAES        1    /* One AES 128,192, 256 */
#  define MAX326_NUSB20      1    /* One USB 2.0 device */
#  define MAX326_NTMR32      6    /* Six 32-bit Timers */
#  define MAX326_NTMR8       0    /* No 8-bit Timers */
#  define MAX326_NPTENGINE  16    /* Sixteen pulse train engines */
#  define MAX326_NSPIM       3    /* Three SPI master */
#  define MAX326_NSPIS       1    /* One SPI slave */
#  define MAX326_NSPIXIP     1    /* One SPI XIP */
#  define MAX326_NI2SS       0    /* No I2S slave */
#  define MAX326_NI2CM       3    /* Three I2C master */
#  define MAX326_NI2CS       1    /* One I2C slave */
#  define MAX326_NUART       4    /* Four UARTs */
#  define MAX326_N1WIREM     1    /* One 1-Wire master */
#  define MAX326_NADC10      1    /* One 10-bit ADC */

/* MAX32660 Family:
 *
 *   Part             Flash  SRAM  Secure     Pin/Package
 *                    (Mb)   (Kb)  Bootloader
 *   MAX32660GWE+     256    96    No         16 WLP
 *   MAX32660GWE+T    256    96    No         16 WLP
 *   MAX32660GTP+     256    96    No         20 TQFN-EP
 *   MAX32660GTP+T    256    96    No         20 TQFN-EP
 *   MAX32660GTG+     256    96    No         24 TQFN-EP
 *   MAX32660GTG+T    256    96    No         24 TQFN-EP
 *   MAX32660GWEBL+*  256    96    Yes        16 WLP
 *   MAX32660GWEBL+T* 256    96    Yes        16 WLP
 *   MAX32660GTGBL+*  256    96    Yes        24 TQFN-EP
 *   MAX32660GTGBL+T* 256    96    Yes        24 TQFN-EP
 *   MAX32660GWELA+*  128    64    No         16 WLP
 *   MAX32660GWELA+T* 128    64    No         16 WLP
 *   MAX32660GTGLA+*  128    64    No         24 TQFN-EP
 *   MAX32660GTGLA+T* 128    64    No         24 TQFN-EP
 *   MAX32660GWELB+*   64    32    No         16 WLP
 *   MAX32660GWELB+T*  64    32    No         16 WLP
 *   MAX32660GTGLB+*   64    32    No         24 TQFN-EP
 *   MAX32660GTGLB+T*  64    32    No         24 TQFN-EP
 */

#elif defined(CONFIG_ARCH_CHIP_MAX32660)

/* Peripherals */

#  define MAX326_NWDOG       1    /* One Watchdog Timer */
#  define MAX326_NWWDOG      0    /* No Windowed Watchdog Timers */
#  define MAX326_NRWDOG      0    /* No Recovery Watchdog Timer */
#  define MAX326_NWAKEUP     0    /* No Wakeup Timer */
#  define MAX326_NRTC        1    /* One RTC */
#  define MAX326_NCRC        0    /* No CRC16/32 */
#  define MAX326_NAES        0    /* No AES 128,192, 256 */
#  define MAX326_NUSB20      0    /* No USB 2.0 device */
#  define MAX326_NTMR32      2    /* Two 32-bit Timers */
#  define MAX326_NTMR8       1    /* One 8-bit Timers */
#  define MAX326_NPTENGINE   0    /* No pulse train engines */
#  define MAX326_NSPIM       2    /* Three SPI master */
#  define MAX326_NSPIS       2    /* Two SPI slave */
#  define MAX326_NSPIXIP     0    /* No SPI XIP */
#  define MAX326_NI2SS       1    /* One I2S slave */
#  define MAX326_NI2CM       2    /* Two I2C master */
#  define MAX326_NI2CS       2    /* One I2C slave */
#  define MAX326_NUART       2    /* Two UARTs */
#  define MAX326_N1WIREM     0    /* No 1-Wire master */
#  define MAX326_NADC10      0    /* No 10-bit ADC */

#else
#  error Unrecognized MAX326XX chip
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0x00-0xe0. The lower the
 * value, the greater the priority of the corresponding interrupt. The
 * processor implements only bits[7:4] of each field, bits[6:0] read as zero
 * and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN      0xe0 /* All bits[7:5] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT  0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX      0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP     0x20 /* Eight priority levels in steps 0x20 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_MAX326XX_CHIP_H */
