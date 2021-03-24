/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_cmt.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_CMT_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_CMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_CMT_CGH1_OFFSET  0x0000 /* CMT Carrier Generator High Data Register 1 */
#define KINETIS_CMT_CGL1_OFFSET  0x0001 /* CMT Carrier Generator Low Data Register 1 */
#define KINETIS_CMT_CGH2_OFFSET  0x0002 /* CMT Carrier Generator High Data Register 2 */
#define KINETIS_CMT_CGL2_OFFSET  0x0003 /* CMT Carrier Generator Low Data Register 2 */
#define KINETIS_CMT_OC_OFFSET    0x0004 /* CMT Output Control Register */
#define KINETIS_CMT_MSC_OFFSET   0x0005 /* CMT Modulator Status and Control Register */
#define KINETIS_CMT_CMD1_OFFSET  0x0006 /* CMT Modulator Data Register Mark High */
#define KINETIS_CMT_CMD2_OFFSET  0x0007 /* CMT Modulator Data Register Mark Low */
#define KINETIS_CMT_CMD3_OFFSET  0x0008 /* CMT Modulator Data Register Space High */
#define KINETIS_CMT_CMD4_OFFSET  0x0009 /* CMT Modulator Data Register Space Low */
#define KINETIS_CMT_PPS_OFFSET   0x000a /* CMT Primary Prescaler Register */
#define KINETIS_CMT_DMA_OFFSET   0x000b /* CMT Direct Memory Access */

/* Register Addresses *******************************************************/

#define KINETIS_CMT_CGH1         (KINETIS_CMT_BASE+KINETIS_CMT_CGH1_OFFSET)
#define KINETIS_CMT_CGL1         (KINETIS_CMT_BASE+KINETIS_CMT_CGL1_OFFSET)
#define KINETIS_CMT_CGH2         (KINETIS_CMT_BASE+KINETIS_CMT_CGH2_OFFSET)
#define KINETIS_CMT_CGL2         (KINETIS_CMT_BASE+KINETIS_CMT_CGL2_OFFSET)
#define KINETIS_CMT_OC           (KINETIS_CMT_BASE+KINETIS_CMT_OC_OFFSET)
#define KINETIS_CMT_MSC          (KINETIS_CMT_BASE+KINETIS_CMT_MSC_OFFSET)
#define KINETIS_CMT_CMD1         (KINETIS_CMT_BASE+KINETIS_CMT_CMD1_OFFSET)
#define KINETIS_CMT_CMD2         (KINETIS_CMT_BASE+KINETIS_CMT_CMD2_OFFSET)
#define KINETIS_CMT_CMD3         (KINETIS_CMT_BASE+KINETIS_CMT_CMD3_OFFSET)
#define KINETIS_CMT_CMD4         (KINETIS_CMT_BASE+KINETIS_CMT_CMD4_OFFSET)
#define KINETIS_CMT_PPS          (KINETIS_CMT_BASE+KINETIS_CMT_PPS_OFFSET)
#define KINETIS_CMT_DMA          (KINETIS_CMT_BASE+KINETIS_CMT_DMA_OFFSET)

/* Register Bit Definitions *************************************************/

/* CMT Carrier Generator High/Low Data Register 1
 * (8-bit Primary Carrier High Time Data Value)
 */

/* CMT Carrier Generator High/Low Data Register 2
 * (8-bit Secondary Carrier High Time Data Value)
 */

/* CMT Output Control Register (8-bit) */

                                           /* Bits 0-4: Reserved */
#define CMT_OC_IROPEN            (1 << 5)  /* Bit 5:  IRO Pin Enable */
#define CMT_OC_CMTPOL            (1 << 6)  /* Bit 6:  CMT Output Polarity */
#define CMT_OC_IROL              (1 << 7)  /* Bit 7:  IRO Latch Control */

/* CMT Modulator Status and Control Register (8-bit) */

#define CMT_MSC_MCGEN            (1 << 0)  /* Bit 0:  Modulator and Carrier Generator Enable */
#define CMT_MSC_EOCIE            (1 << 1)  /* Bit 1:  End of Cycle Interrupt Enable */
#define CMT_MSC_FSK              (1 << 2)  /* Bit 2:  FSK Mode Select */
#define CMT_MSC_BASE             (1 << 3)  /* Bit 3:  Baseband Enable */
#define CMT_MSC_EXSPC            (1 << 4)  /* Bit 4:  Extended Space Enable */
#define CMT_MSC_CMTDIV_SHIFT     (5)       /* Bits 5-6: CMT Clock Divide Prescaler */
#define CMT_MSC_CMTDIV_MASK      (3 << CMT_MSC_CMTDIV_SHIFT)
#  define CMT_MSC_CMTDIV_DIV1    (0 << CMT_MSC_CMTDIV_SHIFT) /* IF / 1 */
#  define CMT_MSC_CMTDIV_DIV2    (1 << CMT_MSC_CMTDIV_SHIFT) /* IF / 2 */
#  define CMT_MSC_CMTDIV_DIV4    (2 << CMT_MSC_CMTDIV_SHIFT) /* IF / 4 */
#  define CMT_MSC_CMTDIV_DIV8    (3 << CMT_MSC_CMTDIV_SHIFT) /* IF / 8 */

#define CMT_MSC_EOCF             (1 << 7)  /* Bit 7:  End Of Cycle Status Flag */

/* CMT Modulator Data Register Mark High/Low (8-bit command data) */

/* CMT Modulator Data Register Space High/Low (8-bit command data) */

/* CMT Primary Prescaler Register (8-bit) */

#define CMT_PPS_SHIFT            (0)       /* Bits 0-3: Primary Prescaler Divider */
#define CMT_PPS_MASK             (15 << CMT_PPS_SHIFT)
#  define CMT_PPS_DIV(n)         (((n)-1) << CMT_PPS_SHIFT) /* Bus clock / n, n=1..16 */

                                           /* Bits 4-7: Reserved */

/* CMT Direct Memory Access (8-bit) */

#define CMT_DMA_ENABLE           (1 << 0)  /* Bit 0:  DMA Enable */
                                           /* Bits 1-7: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_CMT_H */
