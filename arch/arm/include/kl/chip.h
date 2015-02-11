/************************************************************************************
 * arch/arm/include/kinetis/chip.h
 *
 *   Copyright (C) 2013, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_INCLUDE_KL_CHIP_H
#define __ARCH_ARM_INCLUDE_KL_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_MKL25Z128)

#  define KL_Z128            1          /* Kinetics KL25Z128 family */
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

#  define KL_Z128            1          /* Kinetics KL25Z128 family */
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

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-15. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:4] of each field, bits[3:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Steps between supported priority values */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
#endif /* __ARCH_ARM_INCLUDE_KL_CHIP_H */
