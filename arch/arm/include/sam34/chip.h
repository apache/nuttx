/****************************************************************************
 * arch/arm/include/sam34/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_SAM34_CHIP_H
#define __ARCH_ARM_INCLUDE_SAM34_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

/* AT91SAM3U Family *********************************************************/

/* FEATURE      SAM3U4E  SAM3U2E  SAM3U1E  SAM3U4C  SAM3U2C  SAM3U1C
 * ----------- -------- -------- -------- -------- -------- --------
 * Flash        2x128KB  128KB    64KB     2x128KB  128KB    64KB
 * SRAM         32+16KB  16+16KB  16KB     32+16KB  16+16KB  16KB
 * NFC          Yes      Yes      Yes      Yes      Yes      Yes
 * NFC SRAM     4KB      4KB      4KB      4KB      4KB      4KB
 * Package      LQFP144  LQFP144  LQFP144  LQFP100  LQFP100  LQFP100
 *              LFBGA144 LFBGA144 LFBGA144 LFBGA100 LFBGA100 LFBGA100
 * No. PIOs     96       96       96       57       57       57
 * SHDN Pin     Yes      Yes      Yes      No       No       No
 * EMAC         ---      ---      ---      ---      ---      ---
 * EBI          Yes      Yes      Yes      Yes      Yes      Yes
 * EBI data     8/16     8/16     8/16     8        8        8
 * EBI ch       4        4        4        2        2        2
 * EBI addr     24       24       24       8        8        8
 * SDRAM        ---      ---      ---      ---      ---      ---
 * DMA          4        4        4        4        4        4
 * 12-bit ADC   8ch      8ch      8ch      4ch      4ch      4ch
 * 10-bit ADC   8ch      8ch      8ch      4ch      4ch      4ch
 * 32-bit Timer 1        1        1        1        1        1
 * 16-bit Timer 3        3        3        3        3        3
 * PDC Channels 17       17       17       17       17       17
 * USART        4        4        4        3        3        3
 * UART         2        2        2        1        1        1
 * SPI          1        1        1        1        1        1
 * SSC          1        1        1        1        1        1
 * HSMCI        8 bit    8 bit    8 bit    4 bit    4 bit    4 bit
 */

#if defined(CONFIG_ARCH_CHIP_ATSAM3U4E)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            (16*1024)     /*  16KB */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3U2E)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM0_SIZE            (16*1024)     /*  16KB */
#  define SAM34_SRAM1_SIZE            (16*1024)     /*  16KB */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3U1E)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (64*1024)     /*  64KB */
#  define SAM34_SRAM0_SIZE            (16*1024)     /*  16KB */
#  define SAM34_SRAM1_SIZE            0             /*  No SRAM1 */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3U4C)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            (16*1024)     /*  16KB */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3U2C)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM0_SIZE            (16*1024)     /*  16KB */
#  define SAM34_SRAM1_SIZE            (16*1024)     /*  16KB */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3U1C)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (64*1024)     /*  64KB */
#  define SAM34_SRAM0_SIZE            (16*1024)     /*  16KB */
#  define SAM34_SRAM1_SIZE            0             /*  No SRAM1 */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

/* AT91SAM3X/3A Families ****************************************************/

/* FEATURE      SAM3X8E  SAM3X8C  SAM3X4E  SAM3X4C  SAM3A8C  SAM3A4C
 * ------------ -------- -------- -------- -------- -------- --------
 * Flash        2x256KB  2x256KB  2x128KB  2x128KB  2x256KB  2x128KB
 * SRAM         64+32KB  64+32KB  32+32KB  32+32KB  64+32KB  32+32KB
 * NFC          Yes      ---      Yes      ---      ---      ---
 * NFC SRAM     4KB      ---      4KB      ---      ---      ---
 * Package      LQFP144  LQFP100  LQFP144  LQFP100  LQFP100  LQFP100
 *              LFBGA144 LFBGA100 LFBGA144 LFBGA100 LFBGA100 LFBGA100
 * No. PIOs     103      63       103      63       63       63
 * SHDN Pin     Yes      No       Yes      No       No       No
 * EMAC         MII/RMII RMII     MII/RMII RMII     ---      ---
 * EBI          Yes      ---      Yes      ---      ---      ---
 * SDRAM        ---      ---      ---      ---      ---      ---
 * DMA          6        4        6        4        4        4
 * 12-bit ADC   16ch     16ch     16ch     16ch     16ch     16ch
 * 12-bit DAC   2ch      2ch      2ch      2ch      2ch      2ch
 * 32-bit Timer 9        9        9        9        9        9
 * PDC Channels 17       17       17       15       15       15
 * USART        3        3        3        3        3        3
 * UART         2        1        2        1        1        1
 * SPI          1/4+3    1/4+3    1/4+3    1/4+3    1/4+3    1/4+3
 * HSMCI        8 bit    4 bit    8 bit    4 bit    4 bit    4 bit
 */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3X8E)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (512*1024)    /* 512KB */
#  define SAM34_SRAM0_SIZE            (64*1024)     /*  64KB */
#  define SAM34_SRAM1_SIZE            (32*1024)     /*  32KB */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              6             /* 6 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                1             /* One USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3X8C)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (512*1024)    /* 512KB */
#  define SAM34_SRAM0_SIZE            (64*1024)     /*  64KB */
#  define SAM34_SRAM1_SIZE            (32*1024)     /*  32KB */
#  define SAM34_NFCSRAM_SIZE          0             /* No NFC SRAM */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                1             /* One USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3X4E)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            (32*1024)     /*  32KB */
#  define SAM34_NFCSRAM_SIZE          (4*1024)      /*   4KB */

/* Peripherals */

#  define SAM34_NDMACHAN              6             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              17            /* 17 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                1             /* One USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3X4C)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            (32*1024)     /*  32KB */
#  define SAM34_NFCSRAM_SIZE          0             /* No NFC SRAM */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              15            /* 15 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                1             /* One USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3A8C)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (512*1024)    /* 512KB */
#  define SAM34_SRAM0_SIZE            (64*1024)     /*  64KB */
#  define SAM34_SRAM1_SIZE            (32*1024)     /*  32KB */
#  define SAM34_NFCSRAM_SIZE          0             /* No NFC SRAM */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              15            /* 15 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                1             /* One USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM3A4C)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  64KB */
#  define SAM34_SRAM1_SIZE            (32*1024)     /*  32KB */
#  define SAM34_NFCSRAM_SIZE          0             /* No NFC SRAM */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMA Channels */
#  define SAM34_NPDCCHAN              15            /* 15 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                1             /* One USB high speed device */
#  define SAM34_NUHPHS                1             /* One USB high speed embedded host */
#  define SAM34_NUDPFS                0             /* No USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

/* AT91SAM4C Family *********************************************************/

/* FEATURE     SAM4CMP16B
 * ----------- ---------
 * Flash        1024KB
 * SRAM         128KB
 * SMC          Yes
 * GMCC         2KB
 * Package      LQFP144
 *              LFBGA144
 * No. PIOs     117
 * SHDN Pin     No
 * EMAC         MII
 * CAN          2
 * EBI          Yes
 * EBI data     8
 * EBI ch       4
 * EBI addr     24
 * SDRAM        ---
 * DMA          4
 * 16-bit ADC0  16ch
 * 16-bit ADC1  8ch
 * 12-bit DAC   2ch
 * Timer        9
 * PDC Channels 24+9
 * USART        2
 * UART         2
 * SPI          1
 * HSMCI        4 bit
 */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4CMP16B)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)  /* 1024KB */
#  define SAM34_SRAM0_SIZE            (128*1024)   /* 128KB */
#  define SAM34_SRAM1_SIZE            0            /* None */
#  define SAM34_NFCSRAM_SIZE          0            /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              21           /* 21 PDC Channels */
#  define SAM34_NMCI2                 0            /* No memory card interface */
#  define SAM32_NSLCD                 1            /* 1 segment LCD interface */
#  define SAM32_NAESA                 1            /* 1 advanced encryption standard */
#  define SAM32_NUDPHS                0            /* No USB high speed device */
#  define SAM32_NUHPHS                0            /* No USB high speed embedded host */
#  define SAM32_NUDPFS                0            /* No USB full speed device */
#  define SAM32_NUHPFS                0            /* No USB full speed embedded host */

/* AT91SAM4L Family *********************************************************/

/* Sub-family differences:
 *
 *   FEATURE                 ATSAM4LCxx    ATSAM4LSxx
 *   ----------------------- ------------- -------------
 *   SEGMENT LCD             Yes           No
 *   AESA                    Yes           No
 *   USB                     Device + Host Device Only
 *
 *   Note:  The SEGMENT LCD capability varies with packaging.
 *
 *   FEATURE                 ATSAM4Lx2x    ATSAM4Lx4x
 *   ----------------------- ------------- -------------
 *   FLASH                   256KB         128KB
 *   SRAM                    32KB          32KB
 *
 * Packaging differences:
 *
 *   FEATURE                 ATSAM4LxxC ATSAM4LxxB ATSAM4LxxA
 *   ----------------------- ---------- ---------- ----------
 *   Number of Pins          100        64         48
 *   Max Frequency           48MHz      48MHz      48MHz
 *   SEGMENT LCD             4x40       4x23       4x13
 *   GPIO                    75         43         27
 *   High-drive pins         6          3          1
 *   External Interrupts     8+NMI      8+NMI      8+NMI
 *   TWI Masters             2          2          1
 *   TWI Master/Slave        2          2          1
 *   USART                   4          4          3
 *   PICOUART                1          1          1
 *   Peripheral DMA Channels 16         16         16
 *   Peripheral Even System  1          1          1
 *   SPI                     1          1          1
 *   Asynchronous Timers     1          1          1
 *   Timer/Counter Channels  6          3          3
 *   Parallel Capture Inputs 8          8          8
 *   Frequency Meter         1          1          1
 *   Watchdog Timer          1          1          1
 *   Power Manager           1          1          1
 *   Glue Logic LUT          2          2          1
 *   ADC                     15-channel 7-channel  3-channel
 *   DAC                     1-channel  1-channel  1-channel
 *   Analog Comparators      4          2          1
 *   CATB Sensors            32         32         26
 *   Audio Bitstream DAC     1          1          1
 *   IIS Controller          1          1          1
 *   Packages                TQFP/VFBGA TQFP/QFN   TQFP/QFN
 */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4LC2C) || defined (CONFIG_ARCH_CHIP_ATSAM4LC2B) || \
      defined(CONFIG_ARCH_CHIP_ATSAM4LC2A)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              16            /* 16 PDC Channels */
#  define SAM34_NMCI2                 0             /* No memory card interface */
#  define SAM34_NSLCD                 1             /* 1 segment LCD interface */
#  define SAM34_NAESA                 1             /* 1 advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                1             /* 1 USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4LC4C) || defined (CONFIG_ARCH_CHIP_ATSAM4LC4B) || \
      defined(CONFIG_ARCH_CHIP_ATSAM4LC4A)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              16            /* 16 PDC Channels */
#  define SAM34_NMCI2                 0             /* No memory card interface */
#  define SAM34_NSLCD                 1             /* 1 segment LCD interface */
#  define SAM34_NAESA                 1             /* 1 advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                1             /* 1 USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4LS2C) || defined (CONFIG_ARCH_CHIP_ATSAM4LS2B) || \
      defined(CONFIG_ARCH_CHIP_ATSAM4LS2A)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              16            /* 16 PDC Channels */
#  define SAM34_NMCI2                 0             /* No memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4LS4C) || defined (CONFIG_ARCH_CHIP_ATSAM4LS4B) || \
      defined(CONFIG_ARCH_CHIP_ATSAM4LS4A)

/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (32*1024)     /*  32KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              16            /* 16 PDC Channels */
#  define SAM34_NMCI2                 0             /* No memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

/* AT91SAM4S Family *********************************************************/

/* FEATURE       SAM4SD32C SAM4SD32B SAM4SD16C SAM4SD16B SAM4SA16C SAM4SA16B
 * ------------- --------- --------- --------- --------- --------- ---------
 * Flash         2x1MB     2x1MB     2x512KB   1x1MB     1x1MB     1x1MB
 * SRAM          160KB     160KB     160KB     160KB     160KB     160KB
 * HCACHE        2KB       2KB       2KB       2KB       2KB       2KB
 * Pins          100       64        100       64        100       64
 * No. PIOs      79        47        79        47        79        47
 * Ext. BUS      Yes       No        Yes       No        Yes       No
 * 12-bit ADC    16 ch     11 ch     16 ch     11 ch     16 ch     11 ch
 * 12-bit DAC    2 ch      2 ch      2 ch      2 ch      2 ch      2 ch
 * Timer Counter 6 ch      3 ch      6 ch      3 ch      6 ch      3 ch
 * PDC           22 ch     22 ch     22 ch     22 ch     22 ch     22 ch
 * USART         2         2         2         2         2         2
 * UART          2         2         2         2         2         2
 * HSMCI         Yes       Yes       Yes       Yes       Yes       Yes
 *
 * FEATURE       SAM4S16C SAM4S16B SAM4S8C SAM4S8B SAM4S4C
 * ------------- -------- -------- ------- ------- -------
 * Flash         1x1MB    1x1MB    1x512KB 1x512KB 1x256KB
 * SRAM          128KB    128KB    128KB   128KB   64KB
 * HCACHE        -        -        -       -       -
 * Pins          100      64       100     64      100
 * No. PIOs      79       47       79      47      79
 * Ext. BUS      Yes      No       Yes     No      Yes
 * 12-bit ADC    16 ch    11 ch    16 ch   11 ch   16 ch
 * 12-bit DAC    2 ch     2 ch     2 ch    2 ch    2 ch
 * Timer Counter 6 ch     3 ch     6 ch    3 ch    6 ch
 * PDC           22 ch    22 ch    22 ch   22 ch   22 ch
 * USART         2        2        2       2       2
 * UART          2        2        2       2       2
 * HSMCI         Yes      Yes      Yes     Yes     Yes
 */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4SD32C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (2*1024*1024) /* 2x1MB */
#  define SAM34_SRAM0_SIZE            (160*1024)    /* 160KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4SD32B)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (2*1024*1024) /* 2x1MB */
#  define SAM34_SRAM0_SIZE            (160*1024)    /* 160KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4SD16C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 2x512KB */
#  define SAM34_SRAM0_SIZE            (160*1024)    /* 160KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4SD16B)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 2x512KB */
#  define SAM34_SRAM0_SIZE            (160*1024)    /* 160KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4SA16C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 1MB */
#  define SAM34_SRAM0_SIZE            (160*1024)    /* 160KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4SA16B)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 1MB */
#  define SAM34_SRAM0_SIZE            (160*1024)    /* 160KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4S16C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 1MB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4S16B)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 1MB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4S8C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (512*1024)    /* 512KB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4S8B)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (512*1024)    /* 512KB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4S4C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (256*1024)    /* 256KB */
#  define SAM34_SRAM0_SIZE            (64*1024)     /* 64KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              0             /* No DMAC Channels */
#  define SAM34_NPDCCHAN              22            /* 22 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

/* AT91SAM4E Family *********************************************************/

/* FEATURE     SAM4E16E  SAM4E8E  SAM4E16C SAM4E8C
 * ----------- --------- -------- -------- --------
 * Flash        1024KB   512KB    1024KB   512KB
 * SRAM         128KB    128KB    128KB    128KB
 * SMC          Yes      Yes      Yes      Yes
 * GMCC         2KB      2KB      2KB      2KB
 * Package      LQFP144  LQFP144  LQFP100  LQFP100
 *              LFBGA144 LFBGA144 TFBGA100 TFBGA100
 * No. PIOs     117      117      79       79
 * SHDN Pin     No       No       No       No
 * EMAC         MII      MII      MII      MII
 * CAN          2        2        1        1
 * EBI          Yes      Yes      No       No
 * EBI data     8        8        ---      ---
 * EBI ch       4        4        ---      ---
 * EBI addr     24       24       ---      ---
 * SDRAM        ---      ---      ---      ---
 * DMA          4        4        4        4
 * 16-bit ADC0  16ch     16ch     6ch      6ch
 * 16-bit ADC1  8ch      8ch      4ch      4ch
 * 12-bit DAC   2ch      2ch      2ch      2ch
 * Timer        9        9        3        3
 * PDC Channels 24+9     24+9     21+9     21+9
 * USART        2        2        2        2
 * UART         2        2        2        2
 * SPI          1        1        1        1
 * HSMCI        4 bit    4 bit    4 bit    4 bit
 */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4E16E)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMAC Channels */
#  define SAM34_NPDCCHAN              24            /* 24 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4E8E)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (512*1024)    /* 512KB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMAC Channels */
#  define SAM34_NPDCCHAN              24            /* 24 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4E16C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMAC Channels */
#  define SAM34_NPDCCHAN              21            /* 21 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#elif defined(CONFIG_ARCH_CHIP_ATSAM4E8C)
/* Internal memory */

#  define SAM34_FLASH_SIZE            (512*1024)    /* 512KB */
#  define SAM34_SRAM0_SIZE            (128*1024)    /* 128KB */
#  define SAM34_SRAM1_SIZE            0             /* None */
#  define SAM34_NFCSRAM_SIZE          0             /* None */

/* Peripherals */

#  define SAM34_NDMACHAN              4             /* 4 DMAC Channels */
#  define SAM34_NPDCCHAN              21            /* 21 PDC Channels */
#  define SAM34_NMCI2                 1             /* 1 memory card interface */
#  define SAM34_NSLCD                 0             /* No segment LCD interface */
#  define SAM34_NAESA                 0             /* No advanced encryption standard */
#  define SAM34_NUDPHS                0             /* No USB high speed device */
#  define SAM34_NUHPHS                0             /* No USB high speed embedded host */
#  define SAM34_NUDPFS                1             /* 1 USB full speed device */
#  define SAM34_NUHPFS                0             /* No USB full speed embedded host */

#else
#  error "Unknown SAM3/4 chip type"
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-15. The lower the value, the
 * greater the priority of the corresponding interrupt. The processor
 * implements only bits[7:4] of each field, bits[3:0] read as zero and ignore
 * writes.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x10 /* Four bits of interrupt priority used */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAM34_CHIP_H */
