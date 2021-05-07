/****************************************************************************
 * arch/arm/include/kl/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_KL_CHIP_H
#define __ARCH_ARM_INCLUDE_KL_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_MKL25Z64)

#  define KL_Z64             1          /* Kinetis KL25Z128 family */
#  define KL_FLASH_SIZE      (64*1024)  /* 64Kb */
#  define KL_SRAM_SIZE       (8*1024)   /* 8Kb */
#  undef  KL_MPU                        /* No memory protection unit */
#  undef  KL_EXTBUS                     /* No external bus interface */
#  define KL_NDMACH          4          /* Up to 4 DMA channels */
#  undef  KL_NENET                      /* No Ethernet controller */
#  define KL_NUSBHOST        1          /* One USB host controller */
#  define KL_NUSBOTG         1          /* With USB OTG controller */
#  define KL_NUSBDEV         1          /* One USB device controller */
#  undef  KL_NSDHC                      /* No SD host controller */
#  define KL_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KL_NI2C            2          /* Two I2C modules */
#  define KL_NUART           3          /* Three UARTs */
#  define KL_NSPI            2          /* Two SPI modules */
#  undef  KL_NCAN                       /* No CAN in 64-pin chips */
#  define KL_NI2S            1          /* One I2S module */
#  undef  KL_NSLCD                      /* One segment LCD interface (up to 25x8/29x4) */
#  define KL_NADC16          1          /* One 16-bit ADC */
#  undef  KL_NADC12                     /* No 12-channel ADC */
#  undef  KL_NADC13                     /* No 13-channel ADC */
#  undef  KL_NADC15                     /* No 15-channel ADC */
#  undef  KL_NADC18                     /* No 18-channel ADC */
#  undef  KL_NPGA                       /* No Programmable Gain Amplifiers */
#  define KL_NCMP            1          /* One analog comparator */
#  define KL_NDAC6           1          /* Three 6-bit DAC */
#  define KL_NDAC12          1          /* Two 12-bit DAC */
#  define KL_NVREF           1          /* Voltage reference */
#  define KL_NTIMERS6        1          /* One 6 channel timer */
#  define KL_NTIMERS2        2          /* Two 2 channel timers */
#  undef  KL_NRNG                       /* No random number generator */
#  define KL_NRTC            1          /* Real time clock */
#  undef  KL_NMMCAU                     /* No hardware encryption */
#  undef  KL_NTAMPER                    /* No tamper detect */
#  undef  KL_NCRC                       /* No CRC */

#elif defined(CONFIG_ARCH_CHIP_MKL25Z128)

#  define KL_Z128            1          /* Kinetis KL25Z128 family */
#  define KL_FLASH_SIZE      (128*1024) /* 64Kb */
#  define KL_SRAM_SIZE       (16*1024)  /* 16Kb */
#  undef  KL_MPU                        /* No memory protection unit */
#  undef  KL_EXTBUS                     /* No external bus interface */
#  define KL_NDMACH          4          /* Up to 4 DMA channels */
#  undef  KL_NENET                      /* No Ethernet controller */
#  define KL_NUSBHOST        1          /* One USB host controller */
#  define KL_NUSBOTG         1          /* With USB OTG controller */
#  define KL_NUSBDEV         1          /* One USB device controller */
#  undef  KL_NSDHC                      /* No SD host controller */
#  define KL_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KL_NI2C            2          /* Two I2C modules */
#  define KL_NUART           3          /* Three UARTs */
#  define KL_NSPI            2          /* Two SPI modules */
#  undef  KL_NCAN                       /* No CAN in 64-pin chips */
#  define KL_NI2S            1          /* One I2S module */
#  undef  KL_NSLCD                      /* One segment LCD interface (up to 25x8/29x4) */
#  define KL_NADC16          1          /* One 16-bit ADC */
#  undef  KL_NADC12                     /* No 12-channel ADC */
#  undef  KL_NADC13                     /* No 13-channel ADC */
#  undef  KL_NADC15                     /* No 15-channel ADC */
#  undef  KL_NADC18                     /* No 18-channel ADC */
#  undef  KL_NPGA                       /* No Programmable Gain Amplifiers */
#  define KL_NCMP            1          /* One analog comparator */
#  define KL_NDAC6           1          /* Three 6-bit DAC */
#  define KL_NDAC12          1          /* Two 12-bit DAC */
#  define KL_NVREF           1          /* Voltage reference */
#  define KL_NTIMERS8        1          /* One 8 channel timers */
#  undef  KL_NTIMERS12                  /* No 12 channel timers */
#  undef  KL_NTIMERS20                  /* No 20 channel timers */
#  undef  KL_NRNG                       /* No random number generator */
#  define KL_NRTC            1          /* Real time clock */
#  undef  KL_NMMCAU                     /* No hardware encryption */
#  undef  KL_NTAMPER                    /* No tamper detect */
#  undef  KL_NCRC                       /* No CRC */

#elif defined(CONFIG_ARCH_CHIP_MKL26Z128)

#  define KL_Z128            1          /* Kinetis KL25Z128 family */
#  define KL_FLASH_SIZE      (128*1024) /* 64Kb */
#  define KL_SRAM_SIZE       (16*1024)  /* 16Kb */
#  undef  KL_MPU                        /* No memory protection unit */
#  undef  KL_EXTBUS                     /* No external bus interface */
#  define KL_NDMACH          4          /* Up to 4 DMA channels */
#  undef  KL_NENET                      /* No Ethernet controller */
#  define KL_NUSBHOST        1          /* One USB host controller */
#  define KL_NUSBOTG         1          /* With USB OTG controller */
#  define KL_NUSBDEV         1          /* One USB device controller */
#  undef  KL_NSDHC                      /* No SD host controller */
#  define KL_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KL_NI2C            2          /* Two I2C modules */
#  define KL_NUART           3          /* Three UARTs */
#  define KL_NSPI            2          /* Two SPI modules */
#  undef  KL_NCAN                       /* No CAN in 64-pin chips */
#  define KL_NI2S            1          /* One I2S module */
#  undef  KL_NSLCD                      /* One segment LCD interface (up to 25x8/29x4) */
#  define KL_NADC16          1          /* One 16-bit ADC */
#  undef  KL_NADC12                     /* No 12-channel ADC */
#  undef  KL_NADC13                     /* No 13-channel ADC */
#  undef  KL_NADC15                     /* No 15-channel ADC */
#  undef  KL_NADC18                     /* No 18-channel ADC */
#  undef  KL_NPGA                       /* No Programmable Gain Amplifiers */
#  define KL_NCMP            1          /* One analog comparator */
#  define KL_NDAC6           1          /* Three 6-bit DAC */
#  define KL_NDAC12          1          /* Two 12-bit DAC */
#  define KL_NVREF           1          /* Voltage reference */
#  define KL_NTIMERS8        1          /* One 8 channel timers */
#  undef  KL_NTIMERS12                  /* No 12 channel timers */
#  undef  KL_NTIMERS20                  /* No 20 channel timers */
#  undef  KL_NRNG                       /* No random number generator */
#  define KL_NRTC            1          /* Real time clock */
#  undef  KL_NMMCAU                     /* No hardware encryption */
#  undef  KL_NTAMPER                    /* No tamper detect */
#  undef  KL_NCRC                       /* No CRC */

#else
#  error "Unsupported Kinetis chip"
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-15. The lower the value, the
 * greater the priority of the corresponding interrupt. The processor
 * implements only bits[7:6] of each field, bits[5:0] read as zero and ignore
 * writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Steps between supported priority values */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/
#endif /* __ARCH_ARM_INCLUDE_KL_CHIP_H */
