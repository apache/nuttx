/****************************************************************************
 * arch/avr/include/at90usb/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_AT90USB_IRQ_H
#define __ARCH_AVR_INCLUDE_AT90USB_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr/avr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The AT90USB has 38 interrupt vectors including vector 0, the reset
 * vector.  The remaining 37 are assigned IRQ numbers here:
 */

#define AT90USB_IRQ_INT0     0  /* 0x0002 External Interrupt Request 0 */
#define AT90USB_IRQ_INT1     1  /* 0x0004 External Interrupt Request 1 */
#define AT90USB_IRQ_INT2     2  /* 0x0006 External Interrupt Request 2 */
#define AT90USB_IRQ_INT3     3  /* 0x0008 External Interrupt Request 3 */
#define AT90USB_IRQ_INT4     4  /* 0x000a External Interrupt Request 4 */
#define AT90USB_IRQ_INT5     5  /* 0x000c External Interrupt Request 5 */
#define AT90USB_IRQ_INT6     6  /* 0x000e External Interrupt Request 6 */
#define AT90USB_IRQ_INT7     7  /* 0x0010 External Interrupt Request 7 */
#define AT90USB_IRQ_PCINT0   8  /* 0x0012 Pin Change Interrupt Request 0 */
#define AT90USB_IRQ_USBGEN   9  /* 0x0014 USB General USB General Interrupt request */
#define AT90USB_IRQ_USBEP   10  /* 0x0016 USB Endpoint/Pipe USB ENdpoint/Pipe Interrupt request */
#define AT90USB_IRQ_WDT     11  /* 0x0018 Watchdog Time-out Interrupt */
#define AT90USB_IRQ_T2COMPA 12  /* 0x001a TIMER2 COMPA Timer/Counter2 Compare Match A */
#define AT90USB_IRQ_T2COMPB 13  /* 0x001c TIMER2 COMPB Timer/Counter2 Compare Match B */
#define AT90USB_IRQ_T2OVF   14  /* 0x001e TIMER2 OVF Timer/Counter2 Overflow */
#define AT90USB_IRQ_T1CAPT  15  /* 0x0020 TIMER1 CAPT Timer/Counter1 Capture Event */
#define AT90USB_IRQ_T1COMPA 16  /* 0x0022 TIMER1 COMPA Timer/Counter1 Compare Match A */
#define AT90USB_IRQ_T1COMPB 17  /* 0x0024 TIMER1 COMPB Timer/Counter1 Compare Match B */
#define AT90USB_IRQ_T1COMPC 18  /* 0x0026 TIMER1 COMPC Timer/Counter1 Compare Match c */
#define AT90USB_IRQ_T1OVF   19  /* 0x0028 TIMER1 OVF Timer/Counter1 Overflow */
#define AT90USB_IRQ_T0COMPA 20  /* 0x002a TIMER0 COMPA Timer/Counter0 Compare Match A */
#define AT90USB_IRQ_T0COMPB 21  /* 0x002c TIMER0 COMPB Timer/Counter0 Compare Match B */
#define AT90USB_IRQ_T0OVF   22  /* 0x002e TIMER0 OVF Timer/Counter0 Overflow */
#define AT90USB_IRQ_SPI     23  /* 0x0030 STC SPI Serial Transfer Complete */
#define AT90USB_IRQ_U1RX    24  /* 0x0032 USART1 Rx Complete */
#define AT90USB_IRQ_U1DRE   25  /* 0x0034 USART1 Data Register Empty */
#define AT90USB_IRQ_U1TX    26  /* 0x0036 USART1 Tx Complete */
#define AT90USB_IRQ_ANACOMP 27  /* 0x0038 ANALOG COMP Analog Comparator */
#define AT90USB_IRQ_ADC     28  /* 0x003a ADC Conversion Complete */
#define AT90USB_IRQ_EE      29  /* 0x003c EEPROM Ready */
#define AT90USB_IRQ_T3CAPT  30  /* 0x003e TIMER3 CAPT Timer/Counter3 Capture Event */
#define AT90USB_IRQ_T3COMPA 31  /* 0x0034 TIMER3 COMPA Timer/Counter3 Compare Match A */
#define AT90USB_IRQ_T3COMPB 32  /* 0x0042 TIMER3 COMPB Timer/Counter3 Compare Match B */
#define AT90USB_IRQ_T3COMPC 33  /* 0x0044 TIMER3 COMPC Timer/Counter3 Compare Match C */
#define AT90USB_IRQ_T3OVF   34  /* 0x0046 TIMER3 OVF Timer/Counter3 Overflow */
#define AT90USB_IRQ_TWI     35  /* 0x0048 TWI Two-wire Serial Interface */
#define AT90USB_IRQ_SPMRDY  36  /* 0x004a Store Program Memory Ready */

#define NR_IRQS             37
#define AVR_PC_SIZE         16
#define XCPTCONTEXT_REGS    37 /* Size of the register state save array (in bytes) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_AVR_INCLUDE_AT90USB_IRQ_H */
