/****************************************************************************
 * arch/arm/include/sama5/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMA5_CHIP_H
#define __ARCH_ARM_INCLUDE_SAMA5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* SAMA5D2 Family
 *
 *                           SAMA5D21  SAMA5D22  SAMA5D23  SAMA5D24
 * ------------------------- --------- --------- --------- ---------
 * Pin Count                 196       196       196       256
 * Max. Operating Frequency  500 MHz   500 MHz   500 MHz   500 MHz
 * CPU                       Cortex-A5 Cortex-A5 Cortex-A5 Cortex-A5
 * Max I/O Pins              72        72        72        105
 * USB Transceiver           1         1         1         1
 * USB Speed                 Hi-Speed  Hi-Speed  Hi-Speed  Hi-Speed
 * USB Interface             2         2         2         3
 * SPI                       6         6         6         7
 * QuadSPI                   2         2         2         2
 * TWIHS (I2C)               6         6         6         7
 * UART                      9         9         9         10
 * CAN                       -         1         1         -
 * SDIO/SD/MMC               1         1         1         2
 * I2SC                      2         2         2         2
 * SSC                       2         2         2         2
 * Class D                   1         1         1         2
 * PDMIC                     1         1         1         2
 * Camera Interface          1         1         1         1
 * ADC Inputs                5         5         5         1
 * AESB                      -         1         1         1
 * SRAM (Kbytes)             128       128       128       128
 * DDR Bus                   16-bit    16-bit    16-bit    16/32-bit
 * Timers                    6         6         6         6
 * Tamper pins               6         6         6         2
 * Packages                  BGA196    BGA196    BGA196    BGA256
 *
 *                           SAMA5D26  SAMA5D27  SAMA5D28
 * ------------------------- --------- --------- ---------
 * Pin Count                 289       289       289
 * Max. Operating Frequency  500 MHz   500 MHz   500 MHz
 * CPU                       Cortex-A5 Cortex-A5 Cortex-A5
 * Max I/O Pins              128       128       128
 * USB Transceiver           1         1         1
 * USB Speed                 Hi-Speed  Hi-Speed  Hi-Speed
 * USB Interface             3         3         3
 * SPI                       7         7         7
 * QuadSPI                   2         2         2
 * TWIHS (I2C)               7         7         7
 * UART                      10        10        10
 * CAN                       -         2         2
 * SDIO/SD/MMC               2         2         2
 * I2SC                      2         2         2
 * SSC                       2         2         2
 * Class D                   2         2         2
 * PDMIC                     2         2         2
 * Camera Interface          1         1         1
 * ADC Inputs                12        12        12
 * AESB                      -         1         1
 * SRAM (Kbytes)             128       128       128
 * DDR Bus                   16/32-bit 16/32-bit 16/32-bit
 * Timers                    6         6         6
 * Tamper pins               8         8         8
 * Packages                  BGA289    BGA289    BGA289
 */

#if defined(CONFIG_ARCH_CHIP_ATSAMA5D21) || defined(CONFIG_ARCH_CHIP_ATSAMA5D22) || \
    defined(CONFIG_ARCH_CHIP_ATSAMA5D22) || defined(CONFIG_ARCH_CHIP_ATSAMA5D23) || \
    defined(CONFIG_ARCH_CHIP_ATSAMA5D24) || defined(CONFIG_ARCH_CHIP_ATSAMA5D26) || \
    defined(CONFIG_ARCH_CHIP_ATSAMA5D27) || defined(CONFIG_ARCH_CHIP_ATSAMA5D28)
#  define ATSAMA5D2        1         /* SAMA5D2 family */
#  undef  ATSAMA5D3                  /* Not SAMA5D3 family */
#  undef  ATSAMA5D4                  /* Not SAMA5D4 family */
#  define SAM_ISRAM0_SIZE (128*1024) /* SRAM0: 128KB */
#ifdef CONFIG_ARMV7A_L2CC_PL310
#  define SAM_ISRAM1_SIZE (0)        /* (SRAM1 used for L2 cache )*/
#else
#  define SAM_ISRAM0_SIZE (64*1024)  /* SRAM1: 128KB */
#endif
#  define SAM_NDMAC        2         /* (2) XDMA controllers */
#  define SAM_NDMACHAN     16        /* (16) DMA channels per XDMA controller */

/* SAMA5D3 Family
 *
 *                           ATSAMA5D31    ATSAMA5D33    ATSAMA5D34
 * ------------------------- ------------- ------------- -------------
 * Pin Count                 324           324           324
 * Max. Operating Frequency  536 MHz       536 MHz       536 MHz
 * CPU                       Cortex-A5     Cortex-A5     Cortex-A5
 * Max I/O Pins              160           160           160
 * Ext Interrupts            160           160           160
 * USB Transceiver           3             3             3
 * USB Speed                 Hi-Speed      Hi-Speed      Hi-Speed
 * USB Interface             Host, Device  Host, Device  Host, Device
 * SPI                       6             6             6
 * TWI (I2C)                 3             3             3
 * UART                      7             5             5
 * CAN                       -             -             2
 * LIN                       4             4             4
 * SSC                       2             2             2
 * Ethernet                  1             1             1
 * SD / eMMC                 3             2             3
 * Graphic LCD               Yes           Yes           Yes
 * Camera Interface          Yes           Yes           Yes
 * ADC channels              12            12            12
 * ADC Resolution (bits)     12            12            12
 * ADC Speed (ksps)          440           440           440
 * Resistive Touch Screen    Yes           Yes           Yes
 * Crypto Engine             AES/DES/      AES/DES/      AES/DES/
 *                           SHA/TRNG      SHA/TRNG      SHA/TRNG
 * SRAM (Kbytes)             128           128           128
 * External Bus Interface    1             1             1
 * DRAM Memory               DDR2/LPDDR,   DDR2/LPDDR,   DDR2/LPDDR,
 *                           SDRAM/LPSDR   SDRAM/LPSDR   DDR2/LPDDR,
 * NAND Interface            Yes           Yes           Yes
 * Temp. Range (deg C)       -40 to 85     -40 to 85     -40 to 85
 * I/O Supply Class          1.8/3.3       1.8/3.3       1.8/3.3
 * Operating Voltage (Vcc)   1.08 to 1.32  1.08 to 1.32  1.08 to 1.32
 * FPU                       Yes           Yes           Yes
 * MPU / MMU                 No/Yes        No/Yes        No/Yes
 * Timers                    5             5             5
 * Output Compare channels   6             6             6
 * Input Capture Channels    6             6             6
 * PWM Channels              4             4             4
 * 32kHz RTC                 Yes           Yes           Yes
 * Packages                  LFBGA324_A    LFBGA324_A    LFBGA324_A
 *
 *                           ATSAMA5D35    ATSAMA5D36
 * ------------------------- ------------- -------------
 * Pin Count                 324           324
 * Max. Operating Frequency  536 MHz       536 MHz
 * CPU                       Cortex-A5     Cortex-A5
 * Max I/O Pins              160           160
 * Ext Interrupts            160           160
 * USB Transceiver           3             3
 * USB Speed                 Hi-Speed      Hi-Speed
 * USB Interface             Host, Device  Host, Device
 * SPI                       6             6
 * TWI (I2C)                 3             3
 * UART                      7             7
 * CAN                       2             2
 * LIN                       4             4
 * SSC                       2             2
 * Ethernet                  2             2
 * SD / eMMC                 2             3
 * Graphic LCD               -             Yes
 * Camera Interface          Yes           Yes
 * ADC channels              12            12
 * ADC Resolution (bits)     12            12
 * ADC Speed (ksps)          440           1000
 * Resistive Touch Screen    Yes           Yes
 * Crypto Engine             AES/DES/      AES/DES/
 *                           SHA/TRNG      SHA/TRNG
 * SRAM (Kbytes)             128           128
 * External Bus Interface    1             1
 * DRAM Memory               DDR2/LPDDR,   DDR2/LPDDR,
 *                           DDR2/LPDDR,   DDR2/LPDDR,
 * NAND Interface            Yes           Yes
 * Temp. Range (deg C)       -40 to 85     -40 to 105
 * I/O Supply Class          1.8/3.3       1.8/3.3
 * Operating Voltage (Vcc)   1.08 to 1.322 1.08 to 1.32
 * FPU                       Yes           Yes
 * MPU / MMU                 No/Yes        No/Yes
 * Timers                    6             6
 * Output Compare channels   6             6
 * Input Capture Channels    6             6
 * PWM Channels              4             4
 * 32kHz RTC                 Yes           Yes
 * Packages                  LFBGA324_A    LFBGA324_A
 */

#elif defined(CONFIG_ARCH_CHIP_ATSAMA5D31)
#  undef  ATSAMA5D2                  /* Not SAMA5D2 family */
#  define ATSAMA5D3        1         /* SAMA5D3 family */
#  undef  ATSAMA5D4                  /* Not SAMA5D4 family */
#  define SAM_ISRAM0_SIZE (64*1024)  /* 128KB of SRAM in two banks */
#  define SAM_ISRAM1_SIZE (64*1024)
#  define SAM_NDMAC        2         /* (2) DMA controllers */
#  define SAM_NDMACHAN     8         /* (8) DMA channels per DMA controller */
#elif defined(CONFIG_ARCH_CHIP_ATSAMA5D33)
#  undef  ATSAMA5D2                  /* Not SAMA5D2 family */
#  define ATSAMA5D3        1         /* SAMA5D3 family */
#  undef  ATSAMA5D4                  /* Not SAMA5D4 family */
#  define SAM_ISRAM0_SIZE (64*1024)  /* 128KB of SRAM in two banks */
#  define SAM_ISRAM1_SIZE (64*1024)
#  define SAM_NDMAC        2         /* (2) DMA controllers */
#  define SAM_NDMACHAN     8         /* (8) DMA channels per DMA controller */
#elif defined(CONFIG_ARCH_CHIP_ATSAMA5D34)
#  undef  ATSAMA5D2                  /* Not SAMA5D2 family */
#  define ATSAMA5D3        1         /* SAMA5D3 family */
#  undef  ATSAMA5D4                  /* Not SAMA5D4 family */
#  define SAM_ISRAM0_SIZE (64*1024)  /* 128KB of SRAM in two banks */
#  define SAM_ISRAM1_SIZE (64*1024)
#  define SAM_NDMAC        2         /* (2) DMA controllers */
#  define SAM_NDMACHAN     8         /* (8) DMA channels per DMA controller */
#elif defined(CONFIG_ARCH_CHIP_ATSAMA5D35)
#  undef  ATSAMA5D2                  /* Not SAMA5D2 family */
#  define ATSAMA5D3        1         /* SAMA5D3 family */
#  undef  ATSAMA5D4                  /* Not SAMA5D4 family */
#  define SAM_ISRAM0_SIZE (64*1024)  /* 128KB of SRAM in two banks */
#  define SAM_ISRAM1_SIZE (64*1024)
#  define SAM_NDMAC        2         /* (2) DMA controllers */
#  define SAM_NDMACHAN     8         /* (8) DMA channels per DMA controller */
#elif defined(CONFIG_ARCH_CHIP_ATSAMA5D36)
#  undef  ATSAMA5D2                  /* Not SAMA5D2 family */
#  define ATSAMA5D3        1         /* SAMA5D3 family */
#  undef  ATSAMA5D4                  /* Not SAMA5D4 family */
#  define SAM_ISRAM0_SIZE (64*1024)  /* 128KB of SRAM in two banks */
#  define SAM_ISRAM1_SIZE (64*1024)
#  define SAM_NDMAC        2         /* (2) DMA controllers */
#  define SAM_NDMACHAN     8         /* (8) DMA channels per DMA controller */

/* The SAMA5D4 series devices are similar to the SAMA5D3 family except that:
 *
 * - Some parts support a 32-bit DDR data path (SAMA5D42 and SAMA5D44)
 * - Some parts support a Video Decoder (SAMA5D43 and SAMA5D44)
 * - Includes an L2 data cache, NEON FPU, and TrustZone
 * - New XDMAC DMA controller
 * - There are few differences in the support peripherals.  As examples:
 *   Gigbit Ethernet is not supported, for example; 10/100Base-T Ethernet
 *   is different.  Additional instances of peripherals:  USART4, TWI3,
 *   and SPI2.
 */

#elif defined(CONFIG_ARCH_CHIP_ATSAMA5D41) || defined(CONFIG_ARCH_CHIP_ATSAMA5D42) || \
      defined(CONFIG_ARCH_CHIP_ATSAMA5D43) || defined(CONFIG_ARCH_CHIP_ATSAMA5D44)
#  undef  ATSAMA5D2                  /* Not SAMA5D2 family */
#  undef  ATSAMA5D3                  /* Not SAMA5D3 family */
#  define ATSAMA5D4        1         /* SAMA5D4 family */
#  define SAM_ISRAM0_SIZE (64*1024)  /* 128KB of SRAM in two banks */
#  define SAM_ISRAM1_SIZE (64*1024)
#  define SAM_NDMAC        2         /* (2) XDMA controllers */
#  define SAM_NDMACHAN     16        /* (16) DMA channels per XDMA controller */
#else
#  error Unrecognized SAMAD5 chip
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAMA5_CHIP_H */
