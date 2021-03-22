/****************************************************************************
 * arch/arm/include/tms570/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_TMS570_CHIP_H
#define __ARCH_ARM_INCLUDE_TMS570_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/*                 TMS570LS0432PZ TMS570LS0332PZ TMS570LS0232PZ
 * Package         100 QFP        100 QFP        100 QFP
 * CPU             ARM Cortex-R4  ARM Cortex-R4  ARM Cortex-R4
 * Frequency (MHz) 80             80             80
 * Flash (KB)      384            256            128
 * RAM (KB)        32             32             32
 * Data Flash (KB) 16             16             16
 * EMAC            –              –              –
 * FlexRay         –              –              –
 * CAN             2              2              2
 * MibADC (CH)     1 (16)         1 (16)         1 (16)
 * N2HET (Ch)      1 (19)         1 (19)         1 (19)
 * ePWM Channels   –              –              –
 * eCAP Channels   0              0              0
 * eQEP Channels   1              1              1
 * MibSPI (CS)     1 (4)          1 (4)          1 (4)
 * SPI (CS)        2              2              2
 * SCI (LIN)       1 (1)          1 (1)          1 (1)
 * I2C             –              –              –
 * GPIO (INT)      45 (8)         45 (9)         45 (8)
 * EMIF            –              –              –
 * ETM (Trace)     –              –              –
 * RTP/DMM         –              –              –
 */

#if defined(CONFIG_ARCH_CHIP_TMS570LS0232PZ)
#  define TMS570_CORTEX_R4   1           /* Cortex-R4 family */
#  undef  TMS570_CORTEX_R4F              /* Not Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (128*2014)  /* 128 KB Program FLASH */
#  define TMS570_SRAM        (32*1024)   /* 32 KB SRAM */
#  define TMS570_DFLASH      (16*1024)   /* 16 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       0           /* No 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 0           /* No Flexray channels */
#  define TMS570_NCAN        2           /* Two CAN */
#  define TMS570_NMIBADC     1           /* One MiBADC */
#  define TMS570_MIBADC_NCH  16          /* 16 MibADC channels */
#  define TMS570_NN2HET      1           /* One N2HET */
#  define TMS570_N2HET_NCH   19          /* 19 N2HET channels */
#  define TMS570_EPWM_NCH    0           /* No ePWM channels */
#  define TMS570_ECAP_NCH    0           /* No eCAP channels */
#  define TMS570_EQEP_NCH    1           /* One eQEP channel */
#  define TMS570_NMIBSPI     1           /* One MibSPI */
#  define TMS570_MIBSPI1_NCS 4           /* MibSPI1: 4 chip selects */
#  define TMS570_MIBSPI2_NCS 0           /* MibSPI2: No chip selects */
#  define TMS570_MIBSPI3_NCS 0           /* MibSPI3: No chip selects */
#  define TMS570_NSPI        2           /* Two SPI */
#  define TMS570_SPI1_NCS    0           /* SPI1: No chip selects */
#  define TMS570_SPI2_NCS    0           /* SPI2: No chip selects */
#  define TMS570_NSCI        1           /* One SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        0           /* No I2C */
#  define TMS570_NGPIOINT    8           /* 8 GPIO interrupts */
#  define TMS570_NEMIF16     0           /* No EMIF 16-bit data */
#  undef  TMS570_ETM                     /* No ETM (trace) */
#  undef  TMS570_RTP                     /* No RAM trace port (RTP) */
#  undef  TMS570_DMM                     /* No DMM */

#elif defined(CONFIG_ARCH_CHIP_TMS570LS0332PZ)
#  define TMS570_CORTEX_R4   1           /* Cortex-R4 family */
#  undef  TMS570_CORTEX_R4F              /* Not Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (256*2014)  /* 256 KB Program FLASH */
#  define TMS570_SRAM        (32*1024)   /* 32 KB SRAM */
#  define TMS570_DFLASH      (16*1024)   /* 16 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       0           /* No 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 0           /* No Flexray channels */
#  define TMS570_NCAN        2           /* Two CAN */
#  define TMS570_NMIBADC     1           /* One MiBADC */
#  define TMS570_MIBADC_NCH  16          /* 16 MibADC channels */
#  define TMS570_NN2HET      1           /* One N2HET */
#  define TMS570_N2HET_NCH   19          /* 19 N2HET channels */
#  define TMS570_EPWM_NCH    0           /* No ePWM channels */
#  define TMS570_ECAP_NCH    0           /* No eCAP channels */
#  define TMS570_EQEP_NCH    1           /* One eQEP channel */
#  define TMS570_NMIBSPI     1           /* One MibSPI */
#  define TMS570_MIBSPI1_NCS 4           /* MibSPI1: 4 chip selects */
#  define TMS570_MIBSPI2_NCS 0           /* MibSPI2: No chip selects */
#  define TMS570_MIBSPI3_NCS 0           /* MibSPI3: No chip selects */
#  define TMS570_NSPI        2           /* Two SPI */
#  define TMS570_SPI1_NCS    0           /* SPI1: No chip selects */
#  define TMS570_SPI2_NCS    0           /* SPI2: No chip selects */
#  define TMS570_NSCI        1           /* One SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        0           /* No I2C */
#  define TMS570_NGPIOINT    9           /* 9 GPIO interrupts */
#  define TMS570_NEMIF16     0           /* No EMIF 16-bit data */
#  undef  TMS570_ETM                     /* No ETM (trace) */
#  undef  TMS570_RTP                     /* No RAM trace port (RTP) */
#  undef  TMS570_DMM                     /* No DMM */

#elif defined(CONFIG_ARCH_CHIP_TMS570LS0432PZ)
#  define TMS570_CORTEX_R4   1           /* Cortex-R4 family */
#  undef  TMS570_CORTEX_R4F              /* Not Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (384*2014)  /* 384 KB Program FLASH */
#  define TMS570_SRAM        (32*1024)   /* 32 KB SRAM */
#  define TMS570_DFLASH      (16*1024)   /* 16 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       0           /* No 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 0           /* No Flexray channels */
#  define TMS570_NCAN        2           /* Two CAN */
#  define TMS570_NMIBADC     1           /* One MiBADC */
#  define TMS570_MIBADC_NCH  16          /* 16 MibADC channels */
#  define TMS570_NN2HET      1           /* One N2HET */
#  define TMS570_N2HET_NCH   19          /* 19 N2HET channels */
#  define TMS570_EPWM_NCH    0           /* No ePWM channels */
#  define TMS570_ECAP_NCH    0           /* No eCAP channels */
#  define TMS570_EQEP_NCH    1           /* One eQEP channel */
#  define TMS570_NMIBSPI     1           /* One MibSPI */
#  define TMS570_MIBSPI1_NCS 4           /* MibSPI1: 4 chip selects */
#  define TMS570_MIBSPI2_NCS 0           /* MibSPI2: No chip selects */
#  define TMS570_MIBSPI3_NCS 0           /* MibSPI3: No chip selects */
#  define TMS570_NSPI        2           /* Two SPI */
#  define TMS570_SPI1_NCS    0           /* SPI1: No chip selects */
#  define TMS570_SPI2_NCS    0           /* SPI2: No chip selects */
#  define TMS570_NSCI        1           /* One SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        0           /* No I2C */
#  define TMS570_NGPIOINT    8           /* 8 GPIO interrupts */
#  define TMS570_NEMIF16     0           /* No EMIF 16-bit data */
#  undef  TMS570_ETM                     /* No ETM (trace) */
#  undef  TMS570_RTP                     /* No RAM trace port (RTP) */
#  undef  TMS570_DMM                     /* No DMM */

/*                 TMS570LS1227ZWT TMS570LS0714ZWT
 * Package         337 BGA         337 BGA
 * CPU             ARM Cortex-R4F  ARM Cortex-R4F
 * Frequency (MHz) 180             180
 * Flash (KB)      1280            768
 * RAM (KB)        192             128
 * Data Flash (KB) 64              64
 * EMAC            10/100          –
 * FlexRay         2-ch            –
 * CAN             3               3
 * MibADC (CH)     2 (24)          2 (24)
 * N2HET (Ch)      2 (44)          2 (44)
 * ePWM Channels   14              14
 * eCAP Channels   6               6
 * eQEP Channels   2               2
 * MibSPI (CS)     3 (6+6+4)       3 (6+6+4)
 * SPI (CS)        2 (2+1)         2 (2+1)
 * SCI (LIN)       2 (1)           2 (1)
 * I2C             1               1
 * GPIO (INT)      101 (16)        101 (16)
 * EMIF            16-bit data     –
 * ETM (Trace)     –               –
 * RTP/DMM         –               –
 *
 *                 TMS570LS0714PGE TMS570LS0714PZ
 * Package         144 QFP         100 QFP
 * CPU             ARM Cortex-R4F  ARM Cortex-R4F
 * Frequency (MHz) 160             100
 * Flash (KB)      768             768
 * RAM (KB)        128             128
 * Data Flash (KB) 64              64
 * EMAC            –               –
 * FlexRay         –               –
 * CAN             3               2
 * MibADC (CH)     2 (24)          2 (16)
 * N2HET (Ch)      2 (40)          2 (21)
 * ePWM Channels   14              8
 * eCAP Channels   6               4
 * eQEP Channels   2               1
 * MibSPI (CS)     3 (5+6+4)       2 (5+1)
 * SPI (CS)        1 (1)           1 (1)
 * SCI (LIN)       2 (1)           1 (1)
 * I2C             1               –
 * GPIO (INT)      64 (10)         45 (9)
 * EMIF            –               –
 * ETM (Trace)     –               –
 * RTP/DMM         –               –
 */

#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PZ)
#  undef  TMS570_CORTEX_R4               /* Not Cortex-R4 family */
#  define TMS570_CORTEX_R4F  1           /* Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (768*2014)  /* 768 KB Program FLASH */
#  define TMS570_SRAM        (128*1024)  /* 128 KB SRAM */
#  define TMS570_DFLASH      (64*1024)   /* 64 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       0           /* No 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 0           /* No Flexray channels */
#  define TMS570_NCAN        3           /* Three CAN */
#  define TMS570_NMIBADC     2           /* Two MiBADC */
#  define TMS570_MIBADC_NCH  16          /* 16 MibADC channels */
#  define TMS570_NN2HET      2           /* Two N2HET */
#  define TMS570_N2HET_NCH   21          /* 21 N2HET channels */
#  define TMS570_EPWM_NCH    8           /* 8 ePWM channels */
#  define TMS570_ECAP_NCH    4           /* 4 eCAP channels */
#  define TMS570_EQEP_NCH    1           /* 1 eQEP channels */
#  define TMS570_NMIBSPI     2           /* 2 MibSPI */
#  define TMS570_MIBSPI1_NCS 5           /* MibSPI1: 5 chip selects */
#  define TMS570_MIBSPI2_NCS 1           /* MibSPI2: 1 chip selects */
#  define TMS570_MIBSPI3_NCS 0           /* MibSPI3: No chip selects */
#  define TMS570_NSPI        1           /* One SPI */
#  define TMS570_SPI1_NCS    1           /* SPI1: One chip selects */
#  define TMS570_SPI2_NCS    0           /* SPI2: No chip selects */
#  define TMS570_NSCI        1           /* One SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        0           /* No I2C */
#  define TMS570_NGPIOINT    9           /* 9 GPIO interrupts */
#  define TMS570_NEMIF16     0           /* No EMIF 16-bit data */
#  undef  TMS570_ETM                     /* No ETM (trace) */
#  undef  TMS570_RTP                     /* No RAM trace port (RTP) */
#  undef  TMS570_DMM                     /* No DMM */

#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PGE)
#  undef  TMS570_CORTEX_R4               /* Not Cortex-R4 family */
#  define TMS570_CORTEX_R4F  1           /* Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (768*2014)  /* 768 KB Program FLASH */
#  define TMS570_SRAM        (128*1024)  /* 128 KB SRAM */
#  define TMS570_DFLASH      (64*1024)   /* 64 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       0           /* No 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 0           /* No Flexray channels */
#  define TMS570_NCAN        3           /* Three CAN */
#  define TMS570_NMIBADC     2           /* Two MiBADC */
#  define TMS570_MIBADC_NCH  24          /* 24 MibADC channels */
#  define TMS570_NN2HET      2           /* Two N2HET */
#  define TMS570_N2HET_NCH   40          /* 40 N2HET channels */
#  define TMS570_EPWM_NCH    14          /* 14 ePWM channels */
#  define TMS570_ECAP_NCH    6           /* 6 eCAP channels */
#  define TMS570_EQEP_NCH    2           /* 2 eQEP channels */
#  define TMS570_NMIBSPI     3           /* 3 MibSPI */
#  define TMS570_MIBSPI1_NCS 5           /* MibSPI1: 5 chip selects */
#  define TMS570_MIBSPI2_NCS 6           /* MibSPI2: 6 chip selects */
#  define TMS570_MIBSPI3_NCS 4           /* MibSPI3: 4 chip selects */
#  define TMS570_NSPI        1           /* One SPI */
#  define TMS570_SPI1_NCS    1           /* SPI1: One chip selects */
#  define TMS570_SPI2_NCS    0           /* SPI2: No chip selects */
#  define TMS570_NSCI        2           /* Two SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        1           /* One I2C */
#  define TMS570_NGPIOINT    10          /* 16 GPIO interrupts */
#  define TMS570_NEMIF16     0           /* No EMIF 16-bit data */
#  undef  TMS570_ETM                     /* No ETM (trace) */
#  undef  TMS570_RTP                     /* No RAM trace port (RTP) */
#  undef  TMS570_DMM                     /* No DMM */

#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714ZWT)
#  undef  TMS570_CORTEX_R4               /* Not Cortex-R4 family */
#  define TMS570_CORTEX_R4F  1           /* Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (768*2014)  /* 768 KB Program FLASH */
#  define TMS570_SRAM        (128*1024)  /* 128 KB SRAM */
#  define TMS570_DFLASH      (64*1024)   /* 64 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       0           /* No 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 0           /* No Flexray channels */
#  define TMS570_NCAN        3           /* Three CAN */
#  define TMS570_NMIBADC     2           /* Two MiBADC */
#  define TMS570_MIBADC_NCH  24          /* 24 MibADC channels */
#  define TMS570_NN2HET      2           /* Two N2HET */
#  define TMS570_N2HET_NCH   44          /* 44 N2HET channels */
#  define TMS570_EPWM_NCH    14          /* 14 ePWM channels */
#  define TMS570_ECAP_NCH    6           /* 6 eCAP channels */
#  define TMS570_EQEP_NCH    2           /* 2 eQEP channels */
#  define TMS570_NMIBSPI     3           /* 3 MibSPI */
#  define TMS570_MIBSPI1_NCS 6           /* MibSPI1: 6 chip selects */
#  define TMS570_MIBSPI2_NCS 6           /* MibSPI2: 6 chip selects */
#  define TMS570_MIBSPI3_NCS 4           /* MibSPI3: 4 chip selects */
#  define TMS570_NSPI        2           /* Two SPI */
#  define TMS570_SPI1_NCS    2           /* SPI1: Two chip selects */
#  define TMS570_SPI2_NCS    1           /* SPI2: One chip selects */
#  define TMS570_NSCI        2           /* Two SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        1           /* One I2C */
#  define TMS570_NGPIOINT    16          /* 16 GPIO interrupts */
#  define TMS570_NEMIF16     0           /* No EMIF 16-bit data */
#  undef  TMS570_ETM                     /* No ETM (trace) */
#  undef  TMS570_RTP                     /* No RAM trace port (RTP) */
#  undef  TMS570_DMM                     /* No DMM */

#elif defined(CONFIG_ARCH_CHIP_TMS570LS1227ZWT)
#  undef  TMS570_CORTEX_R4               /* Not Cortex-R4 family */
#  define TMS570_CORTEX_R4F  1           /* Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (1280*2014) /* 1,280 KB Program FLASH */
#  define TMS570_SRAM        (192*1024)  /* 192 KB SRAM */
#  define TMS570_DFLASH      (64*1024)   /* 64 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       1           /* One 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 2           /* Two Flexray channels */
#  define TMS570_NCAN        3           /* Three CAN */
#  define TMS570_NMIBADC     2           /* Two MiBADC */
#  define TMS570_MIBADC_NCH  24          /* 24 MibADC channels */
#  define TMS570_NN2HET      2           /* Two N2HET */
#  define TMS570_N2HET_NCH   44          /* 44 N2HET channels */
#  define TMS570_EPWM_NCH    14          /* 14 ePWM channels */
#  define TMS570_ECAP_NCH    6           /* 6 eCAP channels */
#  define TMS570_EQEP_NCH    2           /* 2 eQEP channels */
#  define TMS570_NMIBSPI     3           /* 3 MibSPI */
#  define TMS570_MIBSPI1_NCS 6           /* MibSPI1: 6 chip selects */
#  define TMS570_MIBSPI2_NCS 6           /* MibSPI2: 6 chip selects */
#  define TMS570_MIBSPI3_NCS 4           /* MibSPI3: 4 chip selects */
#  define TMS570_NSPI        2           /* Two SPI */
#  define TMS570_SPI1_NCS    2           /* SPI1: Two chip selects */
#  define TMS570_SPI2_NCS    1           /* SPI2: One chip selects */
#  define TMS570_NSCI        2           /* Two SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        1           /* One I2C */
#  define TMS570_NGPIOINT    16          /* 16 GPIO interrupts */
#  define TMS570_NEMIF16     1           /* One EMIF 16-bit data */
#  undef  TMS570_ETM                     /* No ETM (trace) */
#  undef  TMS570_RTP                     /* No RAM trace port (RTP) */
#  undef  TMS570_DMM                     /* No DMM */

#elif defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#  undef  TMS570_CORTEX_R4               /* Not Cortex-R4 family */
#  define TMS570_CORTEX_R4F  1           /* Cortex-R4F family */
#  undef  TMS570_CORTEX_R5               /* Not Cortex-R5 family */
#  undef  TMS570_CORTEX_R5F              /* Not Cortex-R5F family */
#  undef  TMS570_CORTEX_R7               /* Not Cortex-R7 family */
#  undef  TMS570_CORTEX_R7F              /* Not Cortex-R7F family */
#  define TMS570_PFLASH      (3000*1024) /* 3072 KB Program FLASH */
#  define TMS570_SRAM        (256*1024)  /* 256 KB SRAM */
#  define TMS570_DFLASH      (64*1024)   /* 64 KB Data FLASH (EEPROM) */
#  define TMS570_NEMAC       1           /* One 10/100 Mbit EMAC */
#  define TMS570_FLEXRAY_NCH 2           /* Two Flexray channels */
#  define TMS570_NCAN        3           /* Three CAN */
#  define TMS570_NMIBADC     2           /* Two MiBADC */
#  define TMS570_MIBADC_NCH  24          /* 24 MibADC channels */
#  define TMS570_NN2HET      2           /* Two N2HET */
#  define TMS570_N2HET_NCH   44          /* 44 N2HET channels */
#  undef TMS570_EPWM_NCH                 /* 14 ePWM channels */
#  undef TMS570_ECAP_NCH                 /* 6 eCAP channels */
#  undef TMS570_EQEP_NCH                 /* 2 eQEP channels */
#  define TMS570_NMIBSPI     3           /* 3 MibSPI */
#  define TMS570_MIBSPI1_NCS 6           /* MibSPI1: 6 chip selects */
#  define TMS570_MIBSPI2_NCS 6           /* MibSPI2: 6 chip selects */
#  define TMS570_MIBSPI3_NCS 4           /* MibSPI3: 4 chip selects */
#  define TMS570_NSPI        2           /* Two SPI */
#  define TMS570_SPI1_NCS    2           /* SPI1: Two chip selects */
#  define TMS570_SPI2_NCS    1           /* SPI2: One chip selects */
#  define TMS570_NSCI        2           /* Two SCI */
#  define TMS570_SCI1_LIN    1           /* SCI1: LIN supported */
#  undef  TMS570_SCI2_LIN                /* SCI2: LIN not supported */
#  define TMS570_NI2C        1           /* One I2C */
#  define TMS570_NGPIOINT    16          /* 16 GPIO interrupts */
#  define TMS570_NEMIF16     1           /* One EMIF 16-bit data */
#  define TMS570_ETM         1           /* 32-bit ETM (trace) */
#  define TMS570_RTP         1           /* 16-bit trace port (RTP) */
#  define TMS570_DMM         1           /* 16-bit DMM */

#else
#  error Unrecognized TMS570 chip
#endif
#endif /* __ARCH_ARM_INCLUDE_TMS570_CHIP_H */
