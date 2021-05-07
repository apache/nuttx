/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_pcm.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_PCM_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_PCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCM register base address offset into the APB2 domain ********************/

#define LPC31_PCM_VBASE                (LPC31_APB2_VSECTION+LPC31_APB2_PCM_OFFSET)
#define LPC31_PCM_PBASE                (LPC31_APB2_PSECTION+LPC31_APB2_PCM_OFFSET)

/* PCM register offsets (with respect to the PCM base) **********************/

#define LPC31_PCM_GLOBAL_OFFSET        0x000 /* Global register */
#define LPC31_PCM_CNTL0_OFFSET         0x004 /* Control register 0 */
#define LPC31_PCM_CNTL1_OFFSET         0x008 /* Control register 1 */

#define LPC31_PCM_HPOUT_OFFSET(n)      (0x00c+((n)<<2)) /* Transmit data register n */

#define LPC31_PCM_HPOUT0_OFFSET        0x00c /* Transmit data register 0 */
#define LPC31_PCM_HPOUT1_OFFSET        0x010 /* Transmit data register 1 */
#define LPC31_PCM_HPOUT2_OFFSET        0x014 /* Transmit data register 2 */
#define LPC31_PCM_HPOUT3_OFFSET        0x018 /* Transmit data register 3 */
#define LPC31_PCM_HPOUT4_OFFSET        0x01c /* Transmit data register 4 */
#define LPC31_PCM_HPOUT5_OFFSET        0x020 /* Transmit data register 5 */

#define LPC31_PCM_HPIN_OFFSET(n)       (0x024+((n)<<2)) /* Transmit data register n */

#define LPC31_PCM_HPIN0_OFFSET         0x024 /* Receive data register 0 */
#define LPC31_PCM_HPIN1_OFFSET         0x028 /* Receive data register 1 */
#define LPC31_PCM_HPIN2_OFFSET         0x02c /* Receive data register 2 */
#define LPC31_PCM_HPIN3_OFFSET         0x030 /* Receive data register 3 */
#define LPC31_PCM_HPIN4_OFFSET         0x034 /* Receive data register 4 */
#define LPC31_PCM_HPIN5_OFFSET         0x038 /* Receive data register 5 */
#define LPC31_PCM_CNTL2_OFFSET         0x03c /* Control register 2 */

/* PCM register (virtual) addresses *****************************************/

#define LPC31_PCM_GLOBAL               (LPC31_PCM_VBASE+LPC31_PCM_GLOBAL_OFFSET)
#define LPC31_PCM_CNTL0                (LPC31_PCM_VBASE+LPC31_PCM_CNTL0_OFFSET)
#define LPC31_PCM_CNTL1                (LPC31_PCM_VBASE+LPC31_PCM_CNTL1_OFFSET)
#define LPC31_PCM_HPOUT(n)             (LPC31_PCM_VBASE+LPC31_PCM_HPOUT_OFFSET(n))
#define LPC31_PCM_HPOUT0               (LPC31_PCM_VBASE+LPC31_PCM_HPOUT0_OFFSET)
#define LPC31_PCM_HPOUT1               (LPC31_PCM_VBASE+LPC31_PCM_HPOUT1_OFFSET)
#define LPC31_PCM_HPOUT2               (LPC31_PCM_VBASE+LPC31_PCM_HPOUT2_OFFSET)
#define LPC31_PCM_HPOUT3               (LPC31_PCM_VBASE+LPC31_PCM_HPOUT3_OFFSET)
#define LPC31_PCM_HPOUT4               (LPC31_PCM_VBASE+LPC31_PCM_HPOUT4_OFFSET)
#define LPC31_PCM_HPOUT5               (LPC31_PCM_VBASE+LPC31_PCM_HPOUT5_OFFSET)
#define LPC31_PCM_HPIN(n)              (LPC31_PCM_VBASE+LPC31_PCM_HPIN_OFFSET(n))
#define LPC31_PCM_HPIN0                (LPC31_PCM_VBASE+LPC31_PCM_HPIN0_OFFSET)
#define LPC31_PCM_HPIN1                (LPC31_PCM_VBASE+LPC31_PCM_HPIN1_OFFSET)
#define LPC31_PCM_HPIN2                (LPC31_PCM_VBASE+LPC31_PCM_HPIN2_OFFSET)
#define LPC31_PCM_HPIN3                (LPC31_PCM_VBASE+LPC31_PCM_HPIN3_OFFSET)
#define LPC31_PCM_HPIN4                (LPC31_PCM_VBASE+LPC31_PCM_HPIN4_OFFSET)
#define LPC31_PCM_HPIN5                (LPC31_PCM_VBASE+LPC31_PCM_HPIN5_OFFSET)
#define LPC31_PCM_CNTL2                (LPC31_PCM_VBASE+LPC31_PCM_CNTL2_OFFSET)

/* PCM register bit definitions *********************************************/

/* GLOBAL register, address 0x15000000 */

#define PCM_GLOBAL_DMARXENABLE           (1 << 4)  /* Bit 4:  Enable DMA RX */
#define PCM_GLOBAL_DMATXENABLE           (1 << 3)  /* Bit 3:  Enable DMA TX */
#define PCM_GLOBAL_NORMAL                (1 << 2)  /* Bit 2:  Slave/Normal mode */
#define PCM_GLOBAL_ONOFF                 (1 << 0)  /* Bit 0:  IPINT active */

/* CNTL0 register, address 0x15000004 */

#define PCM_CNTL0_MASTER                 (1 << 14)  /* Bit 14:  PCM/IOM master mode */
#define PCM_CNTL0_LOOPBACK               (1 << 11)  /* Bit 11:  Internal loop-back mode */
#define PCM_CNTL0_TYPOD                  (1 << 10)  /* Bit 10:  Type of PCM_FCS and PCM_DCLK output port */
#define PCM_CNTL0_TYPDOIP_SHIFT          (8)        /* Bits 8-9: Type of PCM/IOM data output ports */
#define PCM_CNTL0_TYPDOIP_MASK           (3 << PCM_CNTL0_TYPDOIP_SHIFT)
#define PCM_CNTL0_TYPFRMSYNC_SHIFT       (6)        /* Bits 6-7: Shape of frame synchronization signal */
#define PCM_CNTL0_TYPFRMSYNC_MASK        (3 << PCM_CNTL0_TYPFRMSYNC_SHIFT)
#define PCM_CNTL0_CLKSPD_SHIFT           (3)        /* Bits 3-5: Port frequency selection */
#define PCM_CNTL0_CLKSPD_MASK            (7 << PCM_CNTL0_CLKSPD_SHIFT)

/* CNTL1 register, address 0x15000008 */

#define PCM_CNTL1_ENSLT_SHIFT            (0)        /* Bits 0-11: Enable PCM/IOM Slots, one per slot */
#define PCM_CNTL1_ENSLT_MASK             (0xfff << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT0               (0x001 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT1               (0x002 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT2               (0x004 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT3               (0x008 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT4               (0x010 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT5               (0x020 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT6               (0x040 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT7               (0x080 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT8               (0x100 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT9               (0x200 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT10              (0x400 << PCM_CNTL1_ENSLT_SHIFT)
#  define PCM_CNTL1_ENSLT11              (0x800 << PCM_CNTL1_ENSLT_SHIFT)

/* HPOUTn registers, addresses 0x1500000c to 0x15000020 (two per slot) */

#define PCM_HPOUT_SHIFT                  (0)       /* Bits 0-15: Transmit data register */
#define PCM_HPOUT_MASK                   (0xffff << PCM_HPOUT_SHIFT)

/* HPINn registers, addresses 0x15000024 to 0x15000038 (two per slot) */

#define PCM_HPIN_SHIFT                   (0)       /* Bits 0-15: Receive data register */
#define PCM_HPIN_MASK                    (0xffff << PCM_HPIN_SHIFT)

/* CNTL2 register, address 0x1500003c */

#define PCM_CNTL2_SLOTDIRINV_SHIFT       (0)        /* Bits 0-11: PCM A/B port configuration, one per slot */
#define PCM_CNTL2_SLOTDIRINV_MASK        (0xfff << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV0          (0x001 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV1          (0x002 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV2          (0x004 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV3          (0x008 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV4          (0x010 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV5          (0x020 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV6          (0x040 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV7          (0x080 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV8          (0x100 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV9          (0x200 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV10         (0x400 << PCM_CNTL2_SLOTDIRINV_SHIFT)
#  define PCM_CNTL2_SLOTDIRINV11         (0x800 << PCM_CNTL2_SLOTDIRINV_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_PCM_H */
