/****************************************************************************
 * arch/arm/src/tms570/hardware/tms570_rti.h
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

/* References:
 * TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller,
 * Technical Reference Manual, Texas Instruments,
 * Literature Number: SPNU517A, September 2013
 */

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_RTI_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_RTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tms570_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define TMS570_RTI_GCTRL_OFFSET         0x0000 /* RTI Global Control Register */
                                               /* 0x0004 RTITBCTRL Reserved. Do NOT use. */
#define TMS570_RTI_CAPCTRL_OFFSET       0x0008 /* RTI Capture Control Register */
#define TMS570_RTI_COMPCTRL_OFFSET      0x000c /* RTI Compare Control Register */
#define TMS570_RTI_FRC0_OFFSET          0x0010 /* RTI Free Running Counter 0 Register */
#define TMS570_RTI_UC0_OFFSET           0x0014 /* RTI Up Counter 0 Register */
#define TMS570_RTI_CPUC0_OFFSET         0x0018 /* RTI Compare Up Counter 0 Register */
#define TMS570_RTI_CAFRC0_OFFSET        0x0020 /* RTI Capture Free Running Counter 0 Register */
#define TMS570_RTI_CAUC0_OFFSET         0x0024 /* RTI Capture Up Counter 0 Register */
#define TMS570_RTI_FRC1_OFFSET          0x0030 /* RTI Free Running Counter 1 Register */
#define TMS570_RTI_UC1_OFFSET           0x0034 /* RTI Up Counter 1 Register */
#define TMS570_RTI_CPUC1_OFFSET         0x0038 /* RTI Compare Up Counter 1 Register */
#define TMS570_RTI_CAFRC1_OFFSET        0x0040 /* RTI Capture Free Running Counter 1 Register */
#define TMS570_RTI_CAUC1_OFFSET         0x0044 /* RTI Capture Up Counter 1 Register */
#define TMS570_RTI_COMP0_OFFSET         0x0050 /* RTI Compare 0 Register Section */
#define TMS570_RTI_UDCP0_OFFSET         0x0054 /* RTI Update Compare 0 Register */
#define TMS570_RTI_COMP1_OFFSET         0x0058 /* RTI Compare 1 Register */
#define TMS570_RTI_UDCP1_OFFSET         0x005c /* RTI Update Compare 1 Register */
#define TMS570_RTI_COMP2_OFFSET         0x0060 /* RTI Compare 2 Register */
#define TMS570_RTI_UDCP2_OFFSET         0x0064 /* RTI Update Compare 2 Register */
#define TMS570_RTI_COMP3_OFFSET         0x0068 /* RTI Compare 3 Register */
#define TMS570_RTI_UDCP3_OFFSET         0x006c /* RTI Update Compare 3 Register */
                                               /* 0x0070 RTITBLCOMP Reserved. Do NOT use. */
                                               /* 0x0074 RTITBHCOMP Reserved. Do NOT use. */
#define TMS570_RTI_SETINTENA_OFFSET     0x0080 /* RTI Set Interrupt Enable Register */
#define TMS570_RTI_CLEARINTENA_OFFSET   0x0084 /* RTI Clear Interrupt Enable Register */
#define TMS570_RTI_INTFLAG_OFFSET       0x0088 /* RTI Interrupt Flag Register */
#define TMS570_RTI_DWDCTRL_OFFSET       0x0090 /* Digital Watchdog Control Register */
#define TMS570_RTI_DWDPRLD_OFFSET       0x0094 /* Digital Watchdog Preload Register */
#define TMS570_RTI_WDSTATUS_OFFSET      0x0098 /* Watchdog Status Register */
#define TMS570_RTI_WDKEY_OFFSET         0x009c /* RTI Watchdog Key Register */
#define TMS570_RTI_DWDCNTR_OFFSET       0x00a0 /* RTI Digital Watchdog Down Counter Register */
#define TMS570_RTI_WWDRXNCTRL_OFFSET    0x00a4 /* Digital Windowed Watchdog Reaction Control Register Section */
#define TMS570_RTI_WWDSIZECTRL_OFFSET   0x00a8 /* Digital Windowed Watchdog Window Size Control Register */
#define TMS570_RTI_INTCLRENABLE_OFFSET  0x00ac /* RTI Compare Interrupt Clear Enable Register */
#define TMS570_RTI_COMP0CLR_OFFSET      0x00b0 /* RTI Compare 0 Clear Register */
#define TMS570_RTI_COMP1CLR_OFFSET      0x00b4 /* RTI Compare 1 Clear Register */
#define TMS570_RTI_COMP2CLR_OFFSET      0x00b8 /* RTI Compare 2 Clear Register */
#define TMS570_RTI_COMP3CLR_OFFSET      0x00bc /* RTI Compare 3 Clear Register */

/* Register Addresses *******************************************************/

#define TMS570_RTI_GCTRL                (TMS570_RTIDWWD_BASE+TMS570_RTI_GCTRL_OFFSET)
#define TMS570_RTI_CAPCTRL              (TMS570_RTIDWWD_BASE+TMS570_RTI_CAPCTRL_OFFSET)
#define TMS570_RTI_COMPCTRL             (TMS570_RTIDWWD_BASE+TMS570_RTI_COMPCTRL_OFFSET)
#define TMS570_RTI_FRC0                 (TMS570_RTIDWWD_BASE+TMS570_RTI_FRC0_OFFSET)
#define TMS570_RTI_UC0                  (TMS570_RTIDWWD_BASE+TMS570_RTI_UC0_OFFSET)
#define TMS570_RTI_CPUC0                (TMS570_RTIDWWD_BASE+TMS570_RTI_CPUC0_OFFSET)
#define TMS570_RTI_CAFRC0               (TMS570_RTIDWWD_BASE+TMS570_RTI_CAFRC0_OFFSET)
#define TMS570_RTI_CAUC0                (TMS570_RTIDWWD_BASE+TMS570_RTI_CAUC0_OFFSET)
#define TMS570_RTI_FRC1                 (TMS570_RTIDWWD_BASE+TMS570_RTI_FRC1_OFFSET)
#define TMS570_RTI_UC1                  (TMS570_RTIDWWD_BASE+TMS570_RTI_UC1_OFFSET)
#define TMS570_RTI_CPUC1                (TMS570_RTIDWWD_BASE+TMS570_RTI_CPUC1_OFFSET)
#define TMS570_RTI_CAFRC1               (TMS570_RTIDWWD_BASE+TMS570_RTI_CAFRC1_OFFSET)
#define TMS570_RTI_CAUC1                (TMS570_RTIDWWD_BASE+TMS570_RTI_CAUC1_OFFSET)
#define TMS570_RTI_COMP0                (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP0_OFFSET)
#define TMS570_RTI_UDCP0                (TMS570_RTIDWWD_BASE+TMS570_RTI_UDCP0_OFFSET)
#define TMS570_RTI_COMP1                (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP1_OFFSET)
#define TMS570_RTI_UDCP1                (TMS570_RTIDWWD_BASE+TMS570_RTI_UDCP1_OFFSET)
#define TMS570_RTI_COMP2                (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP2_OFFSET)
#define TMS570_RTI_UDCP2                (TMS570_RTIDWWD_BASE+TMS570_RTI_UDCP2_OFFSET)
#define TMS570_RTI_COMP3                (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP3_OFFSET)
#define TMS570_RTI_UDCP3                (TMS570_RTIDWWD_BASE+TMS570_RTI_UDCP3_OFFSET)
#define TMS570_RTI_SETINTENA            (TMS570_RTIDWWD_BASE+TMS570_RTI_SETINTENA_OFFSET)
#define TMS570_RTI_CLEARINTENA          (TMS570_RTIDWWD_BASE+TMS570_RTI_CLEARINTENA_OFFSET)
#define TMS570_RTI_INTFLAG              (TMS570_RTIDWWD_BASE+TMS570_RTI_INTFLAG_OFFSET)
#define TMS570_RTI_DWDCTRL              (TMS570_RTIDWWD_BASE+TMS570_RTI_DWDCTRL_OFFSET)
#define TMS570_RTI_DWDPRLD              (TMS570_RTIDWWD_BASE+TMS570_RTI_DWDPRLD_OFFSET)
#define TMS570_RTI_WDSTATUS             (TMS570_RTIDWWD_BASE+TMS570_RTI_WDSTATUS_OFFSET)
#define TMS570_RTI_WDKEY                (TMS570_RTIDWWD_BASE+TMS570_RTI_WDKEY_OFFSET)
#define TMS570_RTI_DWDCNTR              (TMS570_RTIDWWD_BASE+TMS570_RTI_DWDCNTR_OFFSET)
#define TMS570_RTI_WWDRXNCTRL           (TMS570_RTIDWWD_BASE+TMS570_RTI_WWDRXNCTRL_OFFSET)
#define TMS570_RTI_WWDSIZECTRL          (TMS570_RTIDWWD_BASE+TMS570_RTI_WWDSIZECTRL_OFFSET)
#define TMS570_RTI_INTCLRENABLE         (TMS570_RTIDWWD_BASE+TMS570_RTI_INTCLRENABLE_OFFSET)
#define TMS570_RTI_COMP0CLR             (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP0CLR_OFFSET)
#define TMS570_RTI_COMP1CLR             (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP1CLR_OFFSET)
#define TMS570_RTI_COMP2CLR             (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP2CLR_OFFSET)
#define TMS570_RTI_COMP3CLR             (TMS570_RTIDWWD_BASE+TMS570_RTI_COMP3CLR_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* RTI Global Control Register */

#define RTI_GCTRL_CNT0EN                (1 << 0)  /* Bit 0:  Counter 0 enable */
#define RTI_GCTRL_CNT1EN                (1 << 1)  /* Bit 1:  Counter 1 enable */
#define RTI_GCTRL_COS                   (1 << 15) /* Bit 15: Continue on suspend */

/* RTI Capture Control Register */

#define RTI_CAPCTRL_CAPCNTR0            (1 << 0)  /* Bit 0:  Capture counter 0 */
#define RTI_CAPCTRL_CAPCNTR1            (1 << 1)  /* Bit 1:  Capture counter 1 */

/* RTI Compare Control Register */

#define RTI_COMPCTRL_COMPSEL0           (1 << 0)  /* Bit 0:  Compare select 0 */
#define RTI_COMPCTRL_COMPSEL1           (1 << 4)  /* Bit 4:  Compare select 1 */
#define RTI_COMPCTRL_COMPSEL2           (1 << 8)  /* Bit 8:  Compare select 2 */
#define RTI_COMPCTRL_COMPSEL3           (1 << 12) /* Bit 12: Compare select 3 */

/* RTI Free Running Counter 0/1 Register (32-bit counter value) */

/* RTI Up Counter 0/1 Register (32-bit counter value) */

/* RTI Compare Up Counter 0/1 Register (32-bit counter value) */

/* RTI Capture Free Running Counter 0/1 Register (32-bit counter value) */

/* RTI Capture Up Counter 0/1 Register (32-bit counter value) */

/* RTI Compare 0/1/2/3 Register Section (32-bit counter value) */

/* RTI Update Compare 0/1/2/3 Register (32-bit counter value) */

/* RTI Set Interrupt Enable Register, RTI Clear Interrupt Enable Register,
 * and RTI Interrupt Flag Register
 */

#define RTI_INT0                        (1 << 0)  /* Bit 0:  Compare interrupt 0 */
#define RTI_INT1                        (1 << 1)  /* Bit 1:  Compare interrupt 1 */
#define RTI_INT2                        (1 << 2)  /* Bit 2:  Compare interrupt 2 */
#define RTI_INT3                        (1 << 3)  /* Bit 3:  Compare interrupt 3 */
#define RTI_TBINT                       (1 << 16) /* Bit 16: Timebase interrupt */
#define RTI_OVL0INT                     (1 << 17) /* Bit 17: Free running counter 0 overflow interrupt */
#define RTI_OVL1INT                     (1 << 18) /* Bit 18: Free running counter 1 overflow interrupt */
#define RTI_ALLINTS                     0x0007000f

/* Digital Watchdog Control Register */
#define RTI_DWDCTRL_

/* Digital Watchdog Preload Register */
#define RTI_DWDPRLD_

/* Watchdog Status Register */
#define RTI_WDSTATUS_

/* RTI Watchdog Key Register */
#define RTI_WDKEY_

/* RTI Digital Watchdog Down Counter Register */
#define RTI_DWDCNTR_

/* Digital Windowed Watchdog Reaction Control Register Section */
#define RTI_WWDRXNCTRL_

/* Digital Windowed Watchdog Window Size Control Register */
#define RTI_WWDSIZECTRL_

/* RTI Compare Interrupt Clear Enable Register */
#define RTI_INTCLRENABLE_

/* RTI Compare 0 Clear Register */
#define RTI_COMP0CLR_

/* RTI Compare 1 Clear Register */
#define RTI_COMP1CLR_

/* RTI Compare 2 Clear Register */
#define RTI_COMP2CLR_

/* RTI Compare 3 Clear Register */
#define RTI_COMP3CLR_

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_RTI_H */
