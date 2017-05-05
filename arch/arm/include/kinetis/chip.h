/************************************************************************************
 * arch/arm/include/kinetis/chip.h
 *
 *   Copyright (C) 2011, 2013, 2015-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_CHIP_H
#define __ARCH_ARM_INCLUDE_KINETIS_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/kinetis/kinetis_mcg.h>
#include <arch/kinetis/kinetis_sim.h>
#include <arch/kinetis/kinetis_pmc.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

/* MK20DX/DN---VLH5
 *
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  PART NUMBER   CPU    PIN PACKAGE TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                FREQ   CNT         FLASH  FLASH
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  MK20DN32VLH5  50 MHz 64  LQFP     32 KB 32 KB   —       8 KB 40
 *  MK20DX32VLH5  50 MHz 64  LQFP     64 KB 32 KB   2 KB    8 KB 40
 *  MK20DN64VLH5  50 MHz 64  LQFP     64 KB 64 KB   —      16 KB 40
 *  MK20DX64VLH5  50 MHz 64  LQFP     96 KB 64 KB   2 KB   16 KB 40
 *  MK20DN128VLH5 50 MHz 64  LQFP    128 KB 128 KB  —      16 KB 40
 *  MK20DX128VLH5 50 MHz 64  LQFP    160 KB 128 KB  2 KB   16 KB 40
 */

#if defined(CONFIG_ARCH_CHIP_MK20DN32VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX32VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DN64VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX64VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DN128VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX128VLH5)

#  define KINETIS_K20             1          /* Kinetis K20 family */
#  undef  KINETIS_K40                        /* Not Kinetis K40 family */
#  undef  KINETIS_K60                        /* Not Kinetis K60 family */
#  undef  KINETIS_K64                        /* Not Kinetis K64 family */
#  undef  KINETIS_K66                        /* Not Kinetis K66 family */

#if defined(CONFIG_ARCH_CHIP_MK20DN32VLH5)
#  define KINETIS_FLASH_SIZE      (64*1024)  /* 32Kb */
#  define KINETIS_FLEXMEM_SIZE    (0*1024)   /* No FlexMEM */
#  define KINETIS_SRAM_SIZE       (8*1024)   /* 8Kb */
#elif defined(CONFIG_ARCH_CHIP_MK20DX32VLH5)
#  define KINETIS_FLASH_SIZE      (64*1024)  /* 32Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)  /* 32Kb */
#  define KINETIS_SRAM_SIZE       (8*1024)   /* 8Kb */
#elif defined(CONFIG_ARCH_CHIP_MK20DN64VLH5)
#  define KINETIS_FLASH_SIZE      (64*1024)  /* 64Kb */
#  define KINETIS_FLEXMEM_SIZE    (0*1024)   /* No FlexMEM */
#  define KINETIS_SRAM_SIZE       (16*1024)  /* 16Kb */
#elif defined(CONFIG_ARCH_CHIP_MK20DX64VLH5)
#  define KINETIS_FLASH_SIZE      (64*1024)  /* 64Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)  /* 32Kb */
#  define KINETIS_SRAM_SIZE       (16*1024)  /* 16Kb */
#elif defined(CONFIG_ARCH_CHIP_MK20DN128VLH5)
#  define KINETIS_FLASH_SIZE      (128*1024) /* 128Kb */
#  define KINETIS_FLEXMEM_SIZE    (0*1024)   /* No FlexMEM */
#  define KINETIS_SRAM_SIZE       (16*1024)  /* 16Kb */
#elif defined(CONFIG_ARCH_CHIP_MK20DX128VLH5)
#  define KINETIS_FLASH_SIZE      (128*1024) /* 128Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)  /* 32Kb */
#  define KINETIS_SRAM_SIZE       (16*1024)  /* 16Kb */
#endif

#  undef  KINETIS_MPU                        /* No memory protection unit */
#  undef  KINETIS_EXTBUS                     /* No external bus interface */
#  define KINETIS_NDMACH          16         /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                      /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1          /* One USB host controller */
#  define KINETIS_NUSBOTG         1          /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1          /* One USB device controller */
#  undef  KINETIS_NSDHC                      /* No SD host controller */
#  define KINETIS_NTOUCHIF        1          /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            1          /* One I2C modules */
#  undef  KINETIS_NISO7816                   /* No UART with ISO-786 */
#  define KINETIS_NUART           3          /* Three UARTs */
#  define KINETIS_NSPI            2          /* Two SPI modules */
#  undef  KINETIS_NCAN                       /* No CAN controllers */
#  define KINETIS_NI2S            1          /* One I2S module */
#  undef  KINETIS_NSLCD                      /* No segment LCD interface */
#  define KINETIS_NADC16          1          /* One 16-bit ADC */
#  undef  KINETIS_NADC12                     /* No 12-channel ADC */
#  undef  KINETIS_NADC13                     /* No 13-channel ADC */
#  undef  KINETIS_NADC15                     /* No 15-channel ADC */
#  undef  KINETIS_NADC18                     /* No 18-channel ADC */
#  define KINETIS_NPGA            1          /* One Programmable Gain Amplifiers */
#  define KINETIS_NCMP            2          /* Two analog comparators */
#  define KINETIS_NDAC6           2          /* Two 6-bit DAC */
#  undef  KINETIS_NDAC12          0          /* No 12-bit DAC */
#  define KINETIS_NVREF           1          /* Voltage reference */
#  define KINETIS_NTIMERS8        2          /* Two 2-8 channel FlexTimers */
#  undef  KINETIS_NTIMERS12                  /* No 12 channel timers */
#  undef  KINETIS_NTIMERS20                  /* No 20 channel timers */
#  undef  KINETIS_NRNG                       /* No random number generator */
#  define KINETIS_NRTC            1          /* Real time clock */
#  undef  KINETIS_NMMCAU                     /* No hardware encryption */
#  undef  KINETIS_NTAMPER                    /* No tamper detect */
#  define KINETIS_NCRC            1          /* One CRC */

/* MK20DX---VLH7
 *
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  PART NUMBER   CPU    PIN PACKAGE TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                FREQ   CNT         FLASH  FLASH
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  MK20DX64VLH7  72 MHz 64  LQFP     96 KB  64 KB  2 KB   16 KB 40
 *  MK20DX128VLH7 72 MHz 64  LQFP    160 KB 128 KB  2 KB   32 KB 40
 *  MK20DX256VLH7 72 MHz 64  LQFP    288 KB 256 KB  2 KB   64 KB 40
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 */

#elif defined(CONFIG_ARCH_CHIP_MK20DX64VLH7) || \
      defined(CONFIG_ARCH_CHIP_MK20DX128VLH7) || \
      defined(CONFIG_ARCH_CHIP_MK20DX256VLH7)

#  define KINETIS_K20             1            /* Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#if defined(CONFIG_ARCH_CHIP_MK20DX64VLH7)
#  define KINETIS_FLASH_SIZE      (64*1024)    /* 64Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)    /* 32Kb */
#  define KINETIS_SRAM_SIZE       (16*1024)    /* 16Kb */
#elif defined(CONFIG_ARCH_CHIP_MK20DX128VLH7)
#  define KINETIS_FLASH_SIZE      (128*1024)   /* 128Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)    /* 32Kb */
#  define KINETIS_SRAM_SIZE       (32*1024)    /* 32Kb */
#else /* if defined(CONFIG_ARCH_CHIP_MK20DX256VLH7) */
#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)    /* 32Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#endif

#  undef  KINETIS_MPU                          /* No memory protection unit */
#  undef  KINETIS_EXTBUS                       /* No external bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                        /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  undef  KINETIS_NSDHC                        /* No SD host controller */
#  undef  KINETIS_NTOUCHIF                     /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2            /* Two I2C modules */
#  undef  KINETIS_NISO7816                     /* No UART with ISO-786 */
#  define KINETIS_NUART           3            /* Three UARTs */
#  define KINETIS_NSPI            1            /* One SPI module */
#  define KINETIS_NCAN            1            /* Two CAN controller */
#  define KINETIS_NI2S            1            /* One I2S module */
#  undef  KINETIS_NSLCD                        /* No segment LCD interface */
#  define KINETIS_NADC16          2            /* Two 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            2            /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          1            /* One 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS12       2            /* Two 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  undef  KINETIS_NCRC                         /* No CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X64VFX50) || defined(CONFIG_ARCH_CHIP_MK40X64VLH50) || \
    defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  define KINETIS_K40             1            /* Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (64*1024)    /* 64Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)    /* 32Kb */
#  define KINETIS_SRAM_SIZE       (16*1024)    /* 16Kb */
#  undef  KINETIS_MPU                          /* No memory protection unit */
#  undef  KINETIS_EXTBUS                       /* No external bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                        /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  undef  KINETIS_NSDHC                        /* No SD host controller */
#  undef  KINETIS_NTOUCHIF                     /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2            /* Two I2C modules */
#  undef  KINETIS_NISO7816                     /* No UART with ISO-786 */
#  define KINETIS_NUART           6            /* Six UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  if defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)
#    define KINETIS_NCAN          2            /* Two CAN controllers */
#  else
#    undef KINETIS_NCAN                        /* No CAN in 64-pin chips */
#  endif
#  define KINETIS_NI2S            1            /* One I2S module */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 25x8/29x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            2            /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X128VFX50) || defined(CONFIG_ARCH_CHIP_MK40X128VLH50) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLK50) || defined(CONFIG_ARCH_CHIP_MK40X128VMB50) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLL50) || defined(CONFIG_ARCH_CHIP_MK40X128VML50) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VFX72) || defined(CONFIG_ARCH_CHIP_MK40X128VLH72) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLK72) || defined(CONFIG_ARCH_CHIP_MK40X128VMB72) || \
    defined(CONFIG_ARCH_CHIP_MK40X128VLL72) || defined(CONFIG_ARCH_CHIP_MK40X128VML72)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  define KINETIS_K40             1            /* Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (128*1024)   /* 128Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)    /* 32Kb */
#  define KINETIS_SRAM_SIZE       (32*1024)    /* 32Kb */
#  undef  KINETIS_MPU                          /* No memory protection unit */
#  undef  KINETIS_EXTBUS                       /* No external bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                        /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  undef  KINETIS_NSDHC                        /* No SD host controller */
#  undef  KINETIS_NTOUCHIF                     /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2            /* Two I2C modules */
#  undef  KINETIS_NISO7816                     /* No UART with ISO-786 */
#  define KINETIS_NUART           6            /* Six UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S module */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            2            /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLK72) || defined(CONFIG_ARCH_CHIP_MK40X256VMB72) || \
    defined(CONFIG_ARCH_CHIP_MK40X256VLL72) || defined(CONFIG_ARCH_CHIP_MK40X256VML72)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  define KINETIS_K40             1            /* Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  define KINETIS_FLEXMEM_SIZE    (32*1024)    /* 32Kb */
#  define KINETIS_SRAM_SIZE       (32*1024)    /* 64Kb */
#  undef  KINETIS_MPU                          /* No memory protection unit */
#  undef  KINETIS_EXTBUS                       /* No external bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                        /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  undef  KINETIS_NSDHC                        /* No SD host controller */
#  undef  KINETIS_NTOUCHIF                     /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2            /* Two I2C modules */
#  undef  KINETIS_NISO7816                     /* No UART with ISO-786 */
#  define KINETIS_NUART           6            /* Six UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S module */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            2            /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  define KINETIS_K40             1            /* Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (128*1024)   /* 128Kb */
#  define KINETIS_FLEXMEM_SIZE    (128*1024)   /* 128Kb */
#  define KINETIS_SRAM_SIZE       (32*1024)    /* 32Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                        /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* One SD host controller */
#  undef  KINETIS_NTOUCHIF                     /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2            /* Two I2C modules */
#  undef  KINETIS_NISO7816                     /* No UART with ISO-786 */
#  define KINETIS_NUART           6            /* Six UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S module */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 40x8/44x4)*/
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            2            /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  define KINETIS_K40             1            /* Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  define KINETIS_FLEXMEM_SIZE    (256*1024)   /* 256Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 32Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                        /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* One SD host controller */
#  undef  KINETIS_NTOUCHIF                     /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2            /* Two I2C modules */
#  undef  KINETIS_NISO7816                     /* No UART with ISO-786 */
#  define KINETIS_NUART           6            /* Six UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S module */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 40x8/44x4)*/
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            2            /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK40N512VLK100) || defined(CONFIG_ARCH_CHIP_MK40N512VMB100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLL100) || defined(CONFIG_ARCH_CHIP_MK40N512VML100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  define KINETIS_K40             1            /* Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (512*1024)   /* 512Kb */
#  undef  KINETIS_FLEXMEM_SIZE                 /* No FlexMemory */
#  define KINETIS_SRAM_SIZE       (128*1024)   /* 128Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  undef  KINETIS_NENET                        /* No Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* One SD host controller */
#  undef  KINETIS_NTOUCHIF                     /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            2            /* Two I2C modules */
#  undef  KINETIS_NISO7816                     /* No UART with ISO-786 */
#  define KINETIS_NUART           6            /* Six UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S module */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 40x8/44x4)*/
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            2            /* Two Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLL100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           4            /* Four additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  define KINETIS_NADC12          1            /* One 12-channel ADC (ADC0)*/
#  define KINETIS_NADC13          1            /* No 13-channel ADC (ADC1) */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          1            /* One 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLL100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024)   /* 256Kb  */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 32Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           4            /* Four additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  define KINETIS_NADC12          1            /* One 12-channel ADC (ADC0)*/
#  define KINETIS_NADC13          1            /* No 13-channel ADC (ADC1) */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          1            /* One 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLL100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (512*1024)   /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024)   /* 128Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_ENET_HAS_DBSWAP              /* MAC-NET supports DBSWP bit */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           4            /* Four additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  define KINETIS_NADC12          1            /* One 12-channel ADC (ADC0)*/
#  define KINETIS_NADC13          1            /* No 13-channel ADC (ADC1) */
#  undef  KINETIS_NADC15                       /* No 15-channel ADC */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          1            /* One 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VML100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           4            /* Four additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  define KINETIS_NADC12          1            /* One 12-channel ADC (ADC0)*/
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC1) */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          1            /* One 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VML100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024)   /* 256Kb */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           4            /* Four additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  define KINETIS_NADC12          1            /* One 12-channel ADC (ADC0)*/
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC1) */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          1            /* One 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VML100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (512*1024)   /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024)   /* 128Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           4            /* Four additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  define KINETIS_NADC12          1            /* One 12-channel ADC (ADC0)*/
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC1) */
#  undef  KINETIS_NADC18                       /* No 18-channel ADC */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          1            /* One 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           5            /* Five additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1            /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLQ100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024)   /* 256Kb */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           5            /* Five additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1            /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLQ100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (512*1024)   /* 512Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024)   /* 128Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           5            /* Five additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1            /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VMD100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           5            /* Five additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1            /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60X256VMD100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (256*1024)   /* 256Kb */
#  define KINETIS_FLEXNVM_SIZE    (256*1024)   /* 256Kb */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (64*1024)    /* 64Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           5            /* Five additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1            /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60N512VMD100)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (512*1024)   /* 512Kb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024)   /* 128Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NISO7816        1            /* One UART with ISO-786 */
#  define KINETIS_NUART           5            /* Five additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  undef  KINETIS_NADC12                       /* No 12-channel ADC */
#  undef  KINETIS_NADC13                       /* No 13-channel ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC0) */
#  define KINETIS_NADC18          1            /* One 18-channel ADC (ADC1) */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  undef  KINETIS_NDAC6                        /* No 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  undef  KINETIS_NTIMERS12                    /* No 12 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  undef  KINETIS_NTIMERS20                    /* No 20 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  undef  KINETIS_NRNG                         /* No random number generator */
#  undef  KINETIS_NMMCAU                       /* No hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  define KINETIS_K60             1            /* Kinetis K60 family */
#  define KINETIS_NEW_MCG         1            /* Kinetis New MCG - different VDIV */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  undef  KINETIS_FLEXRAM_SIZE                 /* No FlexRAM */
#  define KINETIS_SRAM_SIZE       (128*1024)   /* 128Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          32           /* Up to 32 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBHOST        1            /* One USB host controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NTOUCHIF        1            /* Xtrinsic touch sensing interface */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Five additional UARTs */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            2            /* Two I2S modules */
#  define KINETIS_NADC16          4            /* Four 16-bit ADC */
#  define KINETIS_NADC15          1            /* One 15-channel ADC (ADC0) */
#  define KINETIS_NPGA            4            /* Four Programmable Gain Amplifiers */
#  define KINETIS_NCMP            4            /* Four analog comparators */
#  undef  KINETIS_NDAC6           4            /* Four 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Twp 12-bit DACs */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS20       4            /* Four 20 channel timers */
#  define KINETIS_NTIMERS12       3            /* Three 12 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  undef  KINETIS_NTAMPER                      /* No tamper detect */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLL12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FX512VLL12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FX512VDC12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3             Three SPI modules
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VDC12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3             Three SPI modules
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FX512VLQ12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLQ12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FX512VMD12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VMD12)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  define KINETIS_K64             1            /* Kinetis K64 family */
#  undef  KINETIS_K66                          /* Not Kinetis K66 family */

#  define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#  undef  KINETIS_FLEXNVM_SIZE                 /* No FlexNVM */
#  define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#  define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          16           /* Up to 16 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            3            /* Three I2C modules */
#  define KINETIS_NUART           6            /* Six UART modues */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            1            /* One CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  define KINETIS_NSLCD           1            /* One segment LCD interface (up to 36x8/40x4) */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            3            /* Three analog comparators */
#  define KINETIS_NDAC6           3            /* Three 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */

/* MK66F N/X 1M0/2M0 V MD/LQ 18
 *
 *  --------------- ------- --- ------- ------- ------ ------ ------ -----
 *  PART NUMBER     CPU     PIN PACKAGE  TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                  FREQ    CNT          FLASH  FLASH
 *  --------------- ------- --- ------- ------- ------ ------ ------ -----
 *  MK66FN2M0VMD18  180 MHz 144 MAPBGA   2   MB    —    — KB  260 KB 100
 *  MK66FX1M0VMD18  180 MHz 144 MAPBGA  1.25 MB  1 MB   4 KB  256 KB 100
 *  MK66FN2M0VLQ18  180 MHz 144 LQFP     2   MB    —    — KB  260 KB 100
 *  MK66FX1M0VLQ18  180 MHz 144 LQFP    1.25 MB  1 MB   4 KB  256 KB 100
 */

#elif defined(CONFIG_ARCH_CHIP_MK66FN2M0VMD18) || \
      defined(CONFIG_ARCH_CHIP_MK66FX1M0VMD18) || \
      defined(CONFIG_ARCH_CHIP_MK66FN2M0VLQ18) || \
      defined(CONFIG_ARCH_CHIP_MK66FX1M0VLQ18)
#  undef  KINETIS_K20                          /* Not Kinetis K20 family */
#  undef  KINETIS_K40                          /* Not Kinetis K40 family */
#  undef  KINETIS_K60                          /* Not Kinetis K60 family */
#  undef  KINETIS_K64                          /* Not Kinetis K64 family */
#  define KINETIS_K66             1            /* Kinetis K66 family */

#  if defined(CONFIG_ARCH_CHIP_MK66FX1M0VMD18) || \
      defined(CONFIG_ARCH_CHIP_MK66FX1M0VLQ18)
#    define KINETIS_FLASH_SIZE      (1024*1024)  /* 1Mb */
#    define KINETIS_FLEXNVM_SIZE    (256*1024)   /* 256Kb  */
#    define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#    define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  endif
#  if defined(CONFIG_ARCH_CHIP_MK66FN2M0VMD18) || \
      defined(CONFIG_ARCH_CHIP_MK66FN2M0VLQ18)
#    define KINETIS_FLASH_SIZE      (2048*1024)  /* 2Mb */
#  undef  KINETIS_FLEXNVM_SIZE                   /* No FlexNVM */
#    define KINETIS_FLEXRAM_SIZE    (4*1024)     /* 4Kb */
#    define KINETIS_SRAM_SIZE       (256*1024)   /* 256Kb */
#  endif
#  define KINETIS_MPU             1            /* Memory protection unit */
#  define KINETIS_EXTBUS          1            /* External bus interface */
#  define KINETIS_NDMACH          32           /* Up to 32 DMA channels */
#  define KINETIS_NENET           1            /* One IEEE 1588 Ethernet controller */
#  define KINETIS_NUSBOTG         1            /* With USB OTG controller */
#  define KINETIS_NUSBDEV         1            /* One USB device controller */
#  define KINETIS_NSDHC           1            /* SD host controller */
#  define KINETIS_NI2C            4            /* Four I2C modules */
#  define KINETIS_NUART           5            /* Five UART modules */
#  define KINETIS_NLPUART         1            /* One LPUART modules */
#  define KINETIS_NSPI            3            /* Three SPI modules */
#  define KINETIS_NCAN            2            /* Two CAN controllers */
#  define KINETIS_NI2S            1            /* One I2S modules */
#  undef  KINETIS_NSLCD                        /* No segment LCD interface */
#  define KINETIS_NADC16          2            /* Four 16-bit ADC */
#  define KINETIS_NCMP            4            /* Four analog comparators */
#  define KINETIS_NDAC6           4            /* Four 6-bit DAC */
#  define KINETIS_NDAC12          2            /* Two 12-bit DAC */
#  define KINETIS_NVREF           1            /* Voltage reference */
#  define KINETIS_NTIMERS8        2            /* Two 8 channel FTM timers */
#  define KINETIS_NTIMERS2        2            /* Two 2 channel FTM timers */
#  define KINETIS_NTPMTIMERS2     2            /* Two 2 channel TPM timers */
#  define KINETIS_NRTC            1            /* Real time clock */
#  define KINETIS_NRNG            1            /* Random number generator */
#  define KINETIS_NMMCAU          1            /* Hardware encryption */
#  define KINETIS_NCRC            1            /* CRC */
#else
#  error "Unsupported Kinetis chip"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-15. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:4] of each field, bits[3:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Steps between supported priority values */

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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_KINETIS_CHIP_H */
