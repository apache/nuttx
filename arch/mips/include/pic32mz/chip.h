/****************************************************************************
 * arch/mips/include/pic32mz/chip.h
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_INCLUDE_PIC32MZ_CHIP_H
#define __ARCH_MIPS_INCLUDE_PIC32MZ_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Available in 64/100/124/144 pin packages.  Description here is specifically
 * for the 144 pin package (PIC32MZ2048ECH144) and should be reviewed for
 * other parts.
 */

#if defined(CONFIG_ARCH_CHIP_PIC32MZ2048ECH)
#  define CHIP_PIC32MZEC    1    /* PIC32MZEC family */
#  undef  CHIP_PIC32MZEF         /* Not PIC32MZEF family */
#  define CHIP_BOOTFLASH_KB 160  /* 160Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 2048 /* 2048Kb program FLASH */
#  define CHIP_DATAMEM_KB   512  /* 512Kb data memory */
#  define CHIP_NTIMERS      9    /* 5 timers */
#  define CHIP_NIC          9    /* 5 input capture */
#  define CHIP_NOC          9    /* 5 output compare */
#  define CHIP_NUARTS       6    /* 6 UARTS */
#  define CHIP_UARTFIFOD    8    /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         6    /* 6 SPI/I2S interfaces */
#  define CHIP_NCAN         2    /* 2 CAN interfaces */
#  define CHIP_NCRTYPO      0    /* No crypto support */
#  define CHIP_RNG          1    /* 1 Random number generator */
#  define CHIP_NDMACH       8    /* 8 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  16   /* 16 dedicated DMA channels */
#  define CHIP_NADC10       48   /* 48 ADC channels */
#  define CHIP_NCM          2    /* 2 Analog comparators */
#  define CHIP_USBHSOTG     1    /* 1 USB 2.0 HSOTG */
#  define CHIP_NI2C         5    /* 5 I2C interfaces */
#  define CHIP_NPMP         1    /* Have parallel master port */
#  define CHIP_NEBI         1    /* Have eternal bus interface */
#  define CHIP_NSQI         1    /* 1 Serial quad interface */
#  define CHIP_NRTCC        1    /* Has RTCC */
#  define CHIP_NETHERNET    1    /* 1 Ethernet MAC */
#  define CHIP_NPORTS       10   /* 10 ports (A-H, J-K) */
#  define CHIP_NJTAG        1    /* Has JTAG */
#  define CHIP_NTRACE       1    /* Has trace capability */

/* Available in 64/100/124/144 pin packages.  Description here is specifically
 * for the 144 pin package (PIC32MZ2048ECM144) and should be reviewed for
 * other parts.
 */

#elif defined(CONFIG_ARCH_CHIP_PIC32MZ2048ECM)
#  define CHIP_PIC32MZEC    1    /* PIC32MZEC family */
#  undef  CHIP_PIC32MZEF         /* Not PIC32MZEF family */
#  define CHIP_BOOTFLASH_KB 160  /* 160Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 2048 /* 2048Kb program FLASH */
#  define CHIP_DATAMEM_KB   512  /* 512Kb data memory */
#  define CHIP_NTIMERS      9    /* 5 timers */
#  define CHIP_NIC          9    /* 5 input capture */
#  define CHIP_NOC          9    /* 5 output compare */
#  define CHIP_NUARTS       6    /* 6 UARTS */
#  define CHIP_UARTFIFOD    8    /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         6    /* 6 SPI/I2S interfaces */
#  define CHIP_NCAN         2    /* 2 CAN interfaces */
#  define CHIP_NCRTYPO      1    /* Has crypto support */
#  define CHIP_RNG          1    /* 1 Random number generator */
#  define CHIP_NDMACH       8    /* 8 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  18   /* 18 dedicated DMA channels */
#  define CHIP_NADC10       48   /* 48 ADC channels */
#  define CHIP_NCM          2    /* 2 Analog comparators */
#  define CHIP_USBHSOTG     1    /* 1 USB 2.0 HSOTG */
#  define CHIP_NI2C         5    /* 5 I2C interfaces */
#  define CHIP_NPMP         1    /* Have parallel master port */
#  define CHIP_NEBI         1    /* Have eternal bus interface */
#  define CHIP_NSQI         1    /* 1 Serial quad interface */
#  define CHIP_NRTCC        1    /* Has RTCC */
#  define CHIP_NETHERNET    1    /* 1 Ethernet MAC */
#  define CHIP_NPORTS       10   /* 10 ports (A-H, J-K) */
#  define CHIP_NJTAG        1    /* Has JTAG */
#  define CHIP_NTRACE       1    /* Has trace capability */

/* Available in 64/100/124/144 pin packages.  Description here is specifically
 * for the 124 and 144 pin packages (PIC32MZ2048EFH1100, and
 * PIC32MZ2048EFH144).  The PIC32MZ2048EFH1100 differs in that it has only
 * 40 ADC channels.  The PIC32MZ2048EFH1064 differs in that it has only 24 ADC
 * channels, two fewer SPI/I2S, one fewer I2C, and no EBI.  There are
 * additional differences between all family members in the number of pins
 * how they may be mapped.
 */

#elif defined(CONFIG_ARCH_CHIP_PIC32MZ2048EFH)
#  undef  CHIP_PIC32MZEC         /* Not PIC32MZEC family */
#  define CHIP_PIC32MZEF    1    /* PIC32MZEF family */
#  define CHIP_BOOTFLASH_KB 160  /* 160Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 2048 /* 2048Kb program FLASH */
#  define CHIP_DATAMEM_KB   512  /* 512Kb data memory */
#  define CHIP_NTIMERS      9    /* 5 timers */
#  define CHIP_NIC          9    /* 5 input capture */
#  define CHIP_NOC          9    /* 5 output compare */
#  define CHIP_NUARTS       6    /* 6 UARTS */
#  define CHIP_UARTFIFOD    8    /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         6    /* 6 SPI/I2S interfaces */
#  define CHIP_NCAN         2    /* 2 CAN 2.0B interfaces */
#  define CHIP_NCRTYPO      0    /* No crypto support */
#  define CHIP_RNG          1    /* 1 Random number generator */
#  define CHIP_NDMACH       8    /* 8 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  16   /* 16 dedicated DMA channels */
#  define CHIP_NADC10       48   /* 48 ADC channels */
#  define CHIP_NCM          2    /* 2 Analog comparators */
#  define CHIP_USBHSOTG     1    /* 1 USB 2.0 HSOTG */
#  define CHIP_NI2C         5    /* 5 I2C interfaces */
#  define CHIP_NPMP         1    /* Have parallel master port */
#  define CHIP_NEBI         1    /* Have eternal bus interface */
#  define CHIP_NSQI         1    /* 1 Serial quad interface */
#  define CHIP_NRTCC        1    /* Has RTCC */
#  define CHIP_NETHERNET    1    /* 1 Ethernet MAC */
#  define CHIP_NPORTS       10   /* 10 ports (A-H, J-K) */
#  define CHIP_NJTAG        1    /* Has JTAG */
#  define CHIP_NTRACE       1    /* Has trace capability */

/* Available in 64/100/124/144 pin packages.  Description here is specifically
 * for the 124, and 144 pin packages (PIC32MZ2048EFM124, and
 * PIC32MZ2048EFH144).  The PIC32MZ2048EFM100 differs in that it has only 40
 * ADC channels.  The PIC32MZ2048EFM064 differs in that it has only 24 ADC
 * channels, two fewer SPI/I2S, one fewer I2C, and no EBI.  There are
 * additional differences between all family members in the number of pins
 * how they may be mapped.
 */

#elif defined(CONFIG_ARCH_CHIP_PIC32MZ2048EFM)
#  undef  CHIP_PIC32MZEC         /* Not PIC32MZEC family */
#  define CHIP_PIC32MZEF    1    /* PIC32MZEF family */
#  define CHIP_BOOTFLASH_KB 160  /* 160Kb boot FLASH */
#  define CHIP_PROGFLASH_KB 2048 /* 2048Kb program FLASH */
#  define CHIP_DATAMEM_KB   512  /* 512Kb data memory */
#  define CHIP_NTIMERS      9    /* 5 timers */
#  define CHIP_NIC          9    /* 5 input capture */
#  define CHIP_NOC          9    /* 5 output compare */
#  define CHIP_NUARTS       6    /* 6 UARTS */
#  define CHIP_UARTFIFOD    8    /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         6    /* 6 SPI/I2S interfaces */
#  define CHIP_NCAN         2    /* 2 CAN 2.0B interfaces */
#  define CHIP_NCRTYPO      1    /* Has crypto support */
#  define CHIP_RNG          1    /* 1 Random number generator */
#  define CHIP_NDMACH       8    /* 8 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  18   /* 18 dedicated DMA channels */
#  define CHIP_NADC10       48   /* 48 ADC channels */
#  define CHIP_NCM          2    /* 2 Analog comparators */
#  define CHIP_USBHSOTG     1    /* 1 USB 2.0 HSOTG */
#  define CHIP_NI2C         5    /* 5 I2C interfaces */
#  define CHIP_NPMP         1    /* Have parallel master port */
#  define CHIP_NEBI         1    /* Have eternal bus interface */
#  define CHIP_NSQI         1    /* 1 Serial quad interface */
#  define CHIP_NRTCC        1    /* Has RTCC */
#  define CHIP_NETHERNET    1    /* 1 Ethernet MAC */
#  define CHIP_NPORTS       10   /* 10 ports (A-H, J-K) */
#  define CHIP_NJTAG        1    /* Has JTAG */
#  define CHIP_NTRACE       1    /* Has trace capability */

#else
#  error "Unrecognized PIC32MZ device"
#endif

/* IPL priority levels *****************************************************/
/* These priorities will be used by the core to properly disable/mask
 * interrupts.
 */

#define CHIP_MIN_PRIORITY    1                       /* Minimum priority. */
#define CHIP_MAX_PRIORITY    7                       /* Maximum priority. */
#define CHIP_SW0_PRIORITY    (CHIP_MAX_PRIORITY - 1) /* SW0 priority. */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_PIC32MZ_CHIP_H */
