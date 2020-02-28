/************************************************************************************
 * arch/arm/include/nuc1xx/chip.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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


#ifndef __ARCH_ARM_INCLUDE_NUC1XX_CHIP_H
#define __ARCH_ARM_INCLUDE_NUC1XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Chip capabilities ****************************************************************/
/* NUC100 Advanced Line (Low Density) */

#if defined(CONFIG_ARCH_CHIP_NUC100LC1BN) /* Flash 32K SRAM 4K, LQFP48 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (32*1024)  /* 32K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     35        /* (35) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100LD1BN) /* Flash 64K SRAM 4K, LQFP48 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     31        /* (35) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100LD2BN) /* Flash 64K SRAM 8K, LQFP48 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (8*1024)   /* 8K SRAM */
#  define NUC_NIO     31        /* (35) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100RC1BN) /* Flash 32K SRAM 4K, LQFP64 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (32*1024)  /* 32K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     49        /* (49) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  define NUC_EBI     1         /* Supports EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100RD1BN) /* Flash 64K SRAM 4K, LQFP64 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     49        /* (49) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  define NUC_EBI     1         /* Supports EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100RD2BN) /* Flash 64K SRAM 8K, LQFP64 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (8*1024)   /* 4K SRAM */
#  define NUC_NIO     49        /* (49) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  define NUC_EBI     1         /* Supports EBI */

/* NUC100 Advanced Line (Medium Density) */

#elif defined(CONFIG_ARCH_CHIP_NUC100LD3AN) /* Flash 64K SRAM 16K, LQFP48 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     35        /* (35) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    6         /* (6) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100LE3AN) /* Flash 128K SRAM 16K, LQFP48 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (128*1024) /* 64K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     35        /* (35) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    6         /* (6) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100RD3AN) /* Flash 64K SRAM 16K, LQFP64 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     49        /* (49) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparator */
#  define NUC_NPWM    6         /* (6) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100RE3AN) /* Flash 128K SRAM 16K, LQFP64 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (128*1024) /* 128K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     49        /* (49) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparator */
#  define NUC_NPWM    6         /* (6) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100VD2AN) /* Flash 64K SRAM 8K, LQFP100 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (8*1024)   /* 8K SRAM */
#  define NUC_NIO     80        /* (80) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    4         /* (4) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparator */
#  define NUC_NPWM    8         /* (8) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100VD3AN) /* Flash 64K SRAM 16K, LQFP100 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     80        /* (80) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    4         /* (4) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparator */
#  define NUC_NPWM    8         /* (8) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC100VE3AN) /* Flash 128K SRAM 8K, LQFP100 package */
#  define NUC100      1         /* NUC100 family */
#  undef  NUC120                /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (128*1024) /* 128K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     80        /* (80) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    4         /* (4) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    0         /* No USB */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparator */
#  define NUC_NPWM    8         /* (8) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */

/* NUC120 USB Line (Low Density) */

#elif defined(CONFIG_ARCH_CHIP_NUC120LC1BN) /* Flash 32K SRAM 4K, LQFP48 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (32*1024)  /* 32K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     31        /* (31) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120LD1BN) /* Flash 64K SRAM 4K, LQFP48 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     31        /* (31) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120LD2BN) /* Flash 64K SRAM 8K, LQFP48 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (8*1024)   /* 8K SRAM */
#  define NUC_NIO     31        /* (31) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120RC1BN) /* Flash 32K SRAM 4K, LQFP64 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (32*1024)  /* 32K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     45        /* (45) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  define NUC_EBI     1         /* Have EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120RD1BN) /* Flash 64K SRAM 4K, LQFP64 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (4*1024)   /* 4K SRAM */
#  define NUC_NIO     45        /* (45) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  define NUC_EBI     1         /* Have EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120RD2BN) /* Flash 64K SRAM 8K, LQFP64 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  define NUC_LOW     1         /* Low density part */
#  undef  NUC_MEDIUM            /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (8*1024)   /* 8K SRAM */
#  define NUC_NIO     45        /* (45) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   1         /* 1 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  define NUC_EBI     1         /* Have EBI */

/* NUC120 USB Line (Medium Density) */

#elif defined(CONFIG_ARCH_CHIP_NUC120LD3AN) /* Flash 64K SRAM 16K, LQFP48 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     31        /* (31) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120LE3AN) /* Flash 128K SRAM 16K, LQFP48 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (128*1024) /* 128K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     31        /* (31) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    1         /* (1) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   1         /* (1) Analog Comparator */
#  define NUC_NPWM    4         /* (4) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120RD3AN) /* Flash 64K SRAM 16K, LQFP64 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     45        /* (45) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    6         /* (6) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120RE3AN) /* Flash 128K SRAM 16K, LQFP64 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (128*1024) /* 128K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     45        /* (45) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  2         /* (2) UARTs */
#  define NUC_NSPI    2         /* (2) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    6         /* (6) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120VD2AN) /* Flash 64K SRAM 8K, LQFP100 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (8*1024)   /* 8K SRAM */
#  define NUC_NIO     76        /* (76) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    4         /* (4) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    8         /* (8) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120VD3AN) /* Flash 64K SRAM 16K, LQFP100 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (64*1024)  /* 64K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     76        /* (76) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    4         /* (4) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    8         /* (8) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */
#elif defined(CONFIG_ARCH_CHIP_NUC120VE3AN) /* Flash 128K SRAM 16K, LQFP100 package */
#  undef  NUC100                /* NUC100 family */
#  define NUC120      1         /* NUC120 family */
#  undef  NUC_LOW               /* Low density part */
#  define NUC_MEDIUM  1         /* Medium density part */
#  define NUC_FLASH  (128*1024) /* 128K FLASH */
#  define NUC_SRAM   (16*1024)  /* 16K SRAM */
#  define NUC_NIO     76        /* (76) GPIO */
#  define NUC_NTIMERS 4         /* 4x32-bit Timers */
#  define NUC_NPDMA   9         /* 9 PDMA channels */
#  define NUC_NUARTS  3         /* (3) UARTs */
#  define NUC_NSPI    4         /* (4) SPI */
#  define NUC_NI2C    2         /* (2) I2C */
#  define NUC_NUSB    1         /* (1) USB 2.0 full speed */
#  define NUC_NLIN    0         /* No LIN */
#  define NUC_NCAN    0         /* No CAN */
#  define NUC_NI2S    1         /* (1) I2S */
#  define NUC_NCOMP   2         /* (2) Analog Comparators */
#  define NUC_NPWM    8         /* (8) PWM */
#  define NUC_NADC    8         /* 8x12-bit ADC */
#  define NUC_RTC     1         /* RTC */
#  undef  NUC_EBI               /* No EBI */

#else
#  error "Unrecognized NUC1XX chip"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-3. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:6] of each field, bits[5:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Five bits of interrupt priority used */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_NUC1XX_CHIP_H */
