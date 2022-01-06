/****************************************************************************
 * arch/mips/include/pic32mz/chip.h
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

#ifndef __ARCH_MIPS_INCLUDE_PIC32MZ_CHIP_H
#define __ARCH_MIPS_INCLUDE_PIC32MZ_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Available in 64/100/124/144 pin packages.  Description here is
 * specifically for the 144 pin package (PIC32MZ2048ECH144) and should be
 * reviewed for other parts.
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

/* Available in 64/100/124/144 pin packages.  Description here is
 * specifically for the 144 pin package (PIC32MZ2048ECM144) and should
 * be reviewed for other parts.
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

/* Available in 64/100/124/144 pin packages.  Description here is
 * specifically for the 100 pin package (PIC32MZ2048EFG100).
 */

#elif defined(CONFIG_ARCH_CHIP_PIC32MZ2048EFG)
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
#  define CHIP_NCAN         0    /* No CAN 2.0B interfaces */
#  define CHIP_NCRTYPO      0    /* No crypto support */
#  define CHIP_RNG          1    /* 1 Random number generator */
#  define CHIP_NDMACH       8    /* 8 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  12   /* 12 dedicated DMA channels */
#  define CHIP_NADC10       40   /* 40 ADC channels */
#  define CHIP_NCM          2    /* 2 Analog comparators */
#  define CHIP_USBHSOTG     1    /* 1 USB 2.0 HSOTG */
#  define CHIP_NI2C         5    /* 5 I2C interfaces */
#  define CHIP_NPMP         1    /* Have parallel master port */
#  define CHIP_NEBI         1    /* Have eternal bus interface */
#  define CHIP_NSQI         1    /* 1 Serial quad interface */
#  define CHIP_NRTCC        1    /* Has RTCC */
#  define CHIP_NETHERNET    1    /* 1 Ethernet MAC */
#  define CHIP_NPORTS       7    /* 7 ports (A-G) */
#  define CHIP_NJTAG        1    /* Has JTAG */
#  define CHIP_NTRACE       1    /* Has trace capability */

/* Available in 64/100/124/144 pin packages.  Description here is
 * specifically for the 124 and 144 pin packages (PIC32MZ2048EFH1100, and
 * PIC32MZ2048EFH144).  The PIC32MZ2048EFH1100 differs in that it has only
 * 40 ADC channels.  The PIC32MZ2048EFH1064 differs in that it has only 24
 * ADC channels, two fewer SPI/I2S, one fewer I2C, and no EBI.  There are
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

/* Available in 64/100/124/144 pin packages.  Description here is
 * specifically for the 124, and 144 pin packages (PIC32MZ2048EFM124, and
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

/* IPL priority levels ******************************************************/

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
