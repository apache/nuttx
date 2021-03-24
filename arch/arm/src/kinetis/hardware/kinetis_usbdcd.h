/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_usbdcd.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_USBDCD_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_USBDCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_USBDCD_CONTROL_OFFSET       0x0000 /* Control Register */
#define KINETIS_USBDCD_CLOCK_OFFSET         0x0004 /* Clock Register */
#define KINETIS_USBDCD_STATUS_OFFSET        0x0008 /* Status Register */
#define KINETIS_USBDCD_TIMER0_OFFSET        0x0010 /* TIMER0 Register */
#define KINETIS_USBDCD_TIMER1_OFFSET        0x0014 /* TIMER1 Register */
#ifdef KINETIS_K64
#  define KINETIS_USBDCD_TIMER2_BC11_OFFSET 0x0018 /* TIMER2_BC11 Register */
#  define KINETIS_USBDCD_TIMER2_BC12_OFFSET 0x001c /* TIMER2_BC12 Register */
#else
#  define KINETIS_USBDCD_TIMER2_OFFSET      0x0018 /* TIMER2 Register */
#endif

/* Register Addresses *******************************************************/

#define KINETIS_USBDCD_CONTROL           (KINETIS_USBDCD_BASE+KINETIS_USBDCD_CONTROL_OFFSET)
#define KINETIS_USBDCD_CLOCK             (KINETIS_USBDCD_BASE+KINETIS_USBDCD_CLOCK_OFFSET)
#define KINETIS_USBDCD_STATUS            (KINETIS_USBDCD_BASE+KINETIS_USBDCD_STATUS_OFFSET)
#define KINETIS_USBDCD_TIMER0            (KINETIS_USBDCD_BASE+KINETIS_USBDCD_TIMER0_OFFSET)
#define KINETIS_USBDCD_TIMER1            (KINETIS_USBDCD_BASE+KINETIS_USBDCD_TIMER1_OFFSET)
#ifdef KINETIS_K64
#  define KINETIS_USBDCD_TIMER2_BC11     (KINETIS_USBDCD_BASE+KINETIS_USBDCD_TIMER2_BC11_OFFSET)
#  define KINETIS_USBDCD_TIMER2_BC12     (KINETIS_USBDCD_BASE+KINETIS_USBDCD_TIMER2_BC12_OFFSET)
#else
#  define KINETIS_USBDCD_TIMER2          (KINETIS_USBDCD_BASE+KINETIS_USBDCD_TIMER2_OFFSET)
#endif

/* Register Bit Definitions *************************************************/

/* Control Register */
#define USBDCD_CONTROL_IACK              (1 << 0)  /* Bit 0:  Interrupt Acknowledge */
                                                   /* Bits 1-7:  Reserved */
#define USBDCD_CONTROL_IF                (1 << 8)  /* Bit 8:  Interrupt Flag */
                                                   /* Bits 9-15:  Reserved */
#define USBDCD_CONTROL_IE                (1 << 16) /* Bit 16: Interrupt Enable */
                                                   /* Bits 17-23:  Reserved */
#define USBDCD_CONTROL_START             (1 << 24) /* Bit 24: Start Change Detection Sequence */
#define USBDCD_CONTROL_SR                (1 << 25) /* Bit 25: Software Reset */
                                                   /* Bits 26-31:  Reserved */

/* Clock Register */
#define USBDCD_CLOCK_UNIT                (1 << 0)  /* Bit 0:  Unit of measurement encoding for Clock Speed */
                                                   /* Bit 1:  Reserved */
#define USBDCD_CLOCK_SPEED_SHIFT         (2)       /* Bits 2-11: Value of Clock Speed */
#define USBDCD_CLOCK_SPEED_MASK          (0x3ff << USBDCD_CLOCK_SPEED_SHIFT)
                                                   /* Bits 12-31:  Reserved */

/* Status Register */

                                                   /* Bits 0-15:  Reserved */
#define USBDCD_STATUS_SEQ_RES_SHIFT      (16)      /* Bits 16-17: Charger Detection Sequence Results */
#define USBDCD_STATUS_SEQ_RES_MASK       (3 << USBDCD_STATUS_SEQ_RES_SHIFT)
#  define USBDCD_STATUS_SEQ_RES_NONE     (0 << USBDCD_STATUS_SEQ_RES_SHIFT) /* No results */
#  define USBDCD_STATUS_SEQ_RES_STD      (1 << USBDCD_STATUS_SEQ_RES_SHIFT) /* Standard host */
#  define USBDCD_STATUS_SEQ_RES_CHGPORT  (2 << USBDCD_STATUS_SEQ_RES_SHIFT) /* Charging port */
#  define USBDCD_STATUS_SEQ_RES_DEDCTD   (3 << USBDCD_STATUS_SEQ_RES_SHIFT) /* Dedicated charge */

#define USBDCD_STATUS_SEQ_STAT_SHIFT     (18)      /* Bits 18-19: Charger Detection Sequence Stat */
#define USBDCD_STATUS_SEQ_STAT_MASK      (3 << USBDCD_STATUS_SEQ_STAT_SHIFT)
#  define USBDCD_STATUS_SEQ_STAT_DISAB   (0 << USBDCD_STATUS_SEQ_STAT_SHIFT) /* Not enabled or data pins not detected */
#  define USBDCD_STATUS_SEQ_STAT_DATPIN  (1 << USBDCD_STATUS_SEQ_STAT_SHIFT) /* Data pin contact detection complete */
#  define USBDCD_STATUS_SEQ_STAT_CHGDET  (2 << USBDCD_STATUS_SEQ_STAT_SHIFT) /* Charger detection is complete */
#  define USBDCD_STATUS_SEQ_STAT_CHGTYPE (3 << USBDCD_STATUS_SEQ_STAT_SHIFT) /* Charger type detection complete */

#define USBDCD_STATUS_ERR                (1 << 20) /* Bit 20: Error Flag */
#define USBDCD_STATUS_TO                 (1 << 21) /* Bit 21: Timeout Flag */
#define USBDCD_STATUS_ACTIVE             (1 << 22) /* Bit 22: Active Status Indicator */
                                                   /* Bits 23-31:  Reserved */

/* TIMER0 Register */

#define USBDCD_TIMER0_TUNITCON_SHIFT     (0)       /* Bits 0-11: Unit Connection Timer Elapse (in ms) */
#define USBDCD_TIMER0_TUNITCON_MASK      (0xfff << USBDCD_TIMER0_TUNITCON_SHIFT)
                                                   /* Bits 12-15:  Reserved */
#define USBDCD_TIMER0_TSEQ_INIT_SHIFT    (16)      /* Bits 16-25: Sequence Initiation Time */
#define USBDCD_TIMER0_TSEQ_INIT_MASK     (0x3ff << USBDCD_TIMER0_TSEQ_INIT_SHIFT)
                                                   /* Bits 26-31:  Reserved */

/* TIMER1 Register */

#define USBDCD_TIMER1_TVDPSRC_ON_SHIFT   (0)       /* Bits 0-9: Time Period Comparator Enabled */
#define USBDCD_TIMER1_TVDPSRC_ON_MASK    (0x3ff << USBDCD_TIMER1_TVDPSRC_ON_SHIFT)
                                                   /* Bits 10-15:  Reserved */
#define USBDCD_TIMER1_TDCD_DBNC_SHIFT    (16)      /* Bits 16-25: Time Period to Debounce D+ Signal */
#define USBDCD_TIMER1_TDCD_DBNC__MASK    (0x3ff << USBDCD_TIMER1_TDCD_DBNC_SHIFT)
                                                   /* Bits 26-31:  Reserved */

/* TIMER2 Register */

                                                   /* Bits 26-31:  Reserved */
#define USBDCD_TIMER2_TVDPSRC_CON_SHIFT  (16)      /* Bits 16-25: Time Period Before Enabling D+ Pullup */
#define USBDCD_TIMER2_TVDPSRC_CON_MASK   (0x3ff << USBDCD_TIMER2_TVDPSRC_CON_SHIFT)
                                                   /* Bits 4-15:  Reserved */
#define USBDCD_TIMER2_CHECK_DM_SHIFT     (0)       /* Bits 0-3: Time Before Check of D- Line */
#define USBDCD_TIMER2_CHECK_DM_MASK      (15 << USBDCD_TIMER2_CHECK_DM_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_USBDCD_H */
