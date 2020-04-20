/****************************************************************************
 * arch/avr/include/atmega/irq.h
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_ATMEGA_IRQ_H
#define __ARCH_AVR_INCLUDE_ATMEGA_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr/avr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ATmega interrupt vectors other than vector 0, the reset vector, are
 * assigned here:
 */

#if defined(CONFIG_ARCH_CHIP_ATMEGA128)

#  define ATMEGA_IRQ_INT0         0  /* 0x0002 External Interrupt Request 0 */
#  define ATMEGA_IRQ_INT1         1  /* 0x0004 External Interrupt Request 1 */
#  define ATMEGA_IRQ_INT2         2  /* 0x0006 External Interrupt Request 2 */
#  define ATMEGA_IRQ_INT3         3  /* 0x0008 External Interrupt Request 3 */
#  define ATMEGA_IRQ_INT4         4  /* 0x000a External Interrupt Request 4 */
#  define ATMEGA_IRQ_INT5         5  /* 0x000c External Interrupt Request 5 */
#  define ATMEGA_IRQ_INT6         6  /* 0x000e External Interrupt Request 6 */
#  define ATMEGA_IRQ_INT7         7  /* 0x0010 External Interrupt Request 7 */
#  define ATMEGA_IRQ_T2COMP       8  /* 0x0012 TIMER2 COMP Timer/Counter2 Compare Match */
#  define ATMEGA_IRQ_T2OVF        9  /* 0x0014 TIMER2 OVF Timer/Counter2 Overflow */
#  define ATMEGA_IRQ_T1CAPT      10  /* 0x0016 TIMER1 CAPT Timer/Counter1 Capture Event */
#  define ATMEGA_IRQ_T1COMPA     11  /* 0x0018 TIMER1 COMPA Timer/Counter1 Compare Match A */
#  define ATMEGA_IRQ_T1COMPB     12  /* 0x001a TIMER1 COMPB Timer/Counter1 Compare Match B */
#  define ATMEGA_IRQ_T1OVF       13  /* 0x001c TIMER1 OVF Timer/Counter1 Overflow */
#  define ATMEGA_IRQ_T0COMP      14  /* 0x001e TIMER0 COMP Timer/Counter0 Compare Match */
#  define ATMEGA_IRQ_T0OVF       15  /* 0x0020 TIMER0 OVF Timer/Counter0 Overflow */
#  define ATMEGA_IRQ_SPI         16  /* 0x0022 STC SPI Serial Transfer Complete */
#  define ATMEGA_IRQ_U0RX        17  /* 0x0024 USART0 Rx Complete */
#  define ATMEGA_IRQ_U0DRE       18  /* 0x0026 USART0 Data Register Empty */
#  define ATMEGA_IRQ_U0TX        19  /* 0x0028 USART0 Tx Complete */
#  define ATMEGA_IRQ_ADC         20  /* 0x002a ADC Conversion Complete */
#  define ATMEGA_IRQ_EE          21  /* 0x002c EEPROM Ready */
#  define ATMEGA_IRQ_ANACOMP     22  /* 0x002e ANALOG COMP Analog Comparator */
#  define ATMEGA_IRQ_T1COMPC     23  /* 0x0030 TIMER1 COMPC Timer/Countre1 Compare Match C */
#  define ATMEGA_IRQ_T3CAPT      24  /* 0x0032 TIMER3 CAPT Timer/Counter3 Capture Event */
#  define ATMEGA_IRQ_T3COMPA     25  /* 0x0034 TIMER3 COMPA Timer/Counter3 Compare Match A */
#  define ATMEGA_IRQ_T3COMPB     26  /* 0x0036 TIMER3 COMPB Timer/Counter3 Compare Match B */
#  define ATMEGA_IRQ_T3COMPC     27  /* 0x0038 TIMER3 COMPC Timer/Counter3 Compare Match C */
#  define ATMEGA_IRQ_T3OVF       28  /* 0x003a TIMER3 OVF Timer/Counter3 Overflow */
#  define ATMEGA_IRQ_U1RX        29  /* 0x003c USART1 Rx Complete */
#  define ATMEGA_IRQ_U1DRE       30  /* 0x003e USART1 Data Register Empty */
#  define ATMEGA_IRQ_U1TX        31  /* 0x0040 USART1 Tx Complete */
#  define ATMEGA_IRQ_TWI         32  /* 0x0042 TWI Two-wire Serial Interface */
#  define ATMEGA_IRQ_SPMRDY      33  /* 0x0044 Store Program Memory Ready */

#  define NR_IRQS                34
#  define AVR_PC_SIZE            16
#  define XCPTCONTEXT_REGS       37 /* Size of the register state save array (in bytes) */

#elif defined(CONFIG_ARCH_CHIP_ATMEGA1284P)

#  define ATMEGA_IRQ_INT0         0  /* 0x0002 External Interrupt Request 0 */
#  define ATMEGA_IRQ_INT1         1  /* 0x0004 External Interrupt Request 1 */
#  define ATMEGA_IRQ_INT2         2  /* 0x0006 External Interrupt Request 2 */
#  define ATMEGA_IRQ_PCINT0       3  /* 0x0008 Pin Change Interrupt Request 0 */
#  define ATMEGA_IRQ_PCINT1       4  /* 0x000a Pin Change Interrupt Request 1 */
#  define ATMEGA_IRQ_PCINT2       5  /* 0x000c Pin Change Interrupt Request 2 */
#  define ATMEGA_IRQ_PCINT3       6  /* 0x000e Pin Change Interrupt Request 3 */
#  define ATMEGA_IRQ_WDT          7  /* 0x0010 Watchdog Time-Out Interrupt */
#  define ATMEGA_IRQ_T2COMPA      8  /* 0x0012 TIMER2 COMPA Timer/Counter2 Compare Match */
#  define ATMEGA_IRQ_T2COMPB      9  /* 0x0014 TIMER2 COMPB Timer/Counter2 Compare Match */
#  define ATMEGA_IRQ_T2OVF       10  /* 0x0016 TIMER2 OVF Timer/Counter2 Overflow */
#  define ATMEGA_IRQ_T1CAPT      11  /* 0x0018 TIMER1 CAPT Timer/Counter1 Capture Event */
#  define ATMEGA_IRQ_T1COMPA     12  /* 0x001a TIMER1 COMPA Timer/Counter1 Compare Match A */
#  define ATMEGA_IRQ_T1COMPB     13  /* 0x001c TIMER1 COMPB Timer/Counter1 Compare Match B */
#  define ATMEGA_IRQ_T1OVF       14  /* 0x001e TIMER1 OVF Timer/Counter1 Overflow */
#  define ATMEGA_IRQ_T0COMPA     15  /* 0x0020 TIMER0 COMP Timer/Counter0 Compare Match */
#  define ATMEGA_IRQ_T0COMPB     16  /* 0x0022 TIMER0 COMP Timer/Counter0 Compare Match */
#  define ATMEGA_IRQ_T0OVF       17  /* 0x0024 TIMER0 OVF Timer/Counter0 Overflow */
#  define ATMEGA_IRQ_SPI         18  /* 0x0026 STC SPI Serial Transfer Complete */
#  define ATMEGA_IRQ_U0RX        19  /* 0x0028 USART0 Rx Complete */
#  define ATMEGA_IRQ_U0DRE       20  /* 0x002a USART0 Data Register Empty */
#  define ATMEGA_IRQ_U0TX        21  /* 0x002c USART0 Tx Complete */
#  define ATMEGA_IRQ_ANACOMP     22  /* 0x002e ANALOG COMP Analog Comparator */
#  define ATMEGA_IRQ_ADC         23  /* 0x0030 ADC Conversion Complete */
#  define ATMEGA_IRQ_EE          24  /* 0x0032 EEPROM Ready */
#  define ATMEGA_IRQ_TWI         25  /* 0x0034 TWI Two-wire Serial Interface */
#  define ATMEGA_IRQ_SPMRDY      26  /* 0x0036 Store Program Memory Ready */
#  define ATMEGA_IRQ_U1RX        27  /* 0x0038 USART1 Rx Complete */
#  define ATMEGA_IRQ_U1DRE       28  /* 0x003a USART1 Data Register Empty */
#  define ATMEGA_IRQ_U1TX        29  /* 0x003c USART1 Tx Complete */
#  define ATMEGA_IRQ_T3CAPT      30  /* 0x003e TIMER3 CAPT Timer/Counter3 Capture Event */
#  define ATMEGA_IRQ_T3COMPA     31  /* 0x0040 TIMER3 COMPA Timer/Counter3 Compare Match A */
#  define ATMEGA_IRQ_T3COMPB     32  /* 0x0042 TIMER3 COMPB Timer/Counter3 Compare Match B */
#  define ATMEGA_IRQ_T3OVF       33  /* 0x0044 TIMER3 OVF Timer/Counter3 Overflow */

#  define NR_IRQS                34
#  define AVR_PC_SIZE            16
#  define XCPTCONTEXT_REGS       37 /* Size of the register state save array (in bytes) */

#elif defined(CONFIG_ARCH_CHIP_ATMEGA2560)

#  define ATMEGA_IRQ_INT0        2 /* 0x0002 INT0 External Interrupt Request 0 */
#  define ATMEGA_IRQ_INT1        3 /* 0x0004 INT1 External Interrupt Request 1 */
#  define ATMEGA_IRQ_INT2        4 /* 0x0006 INT2 External Interrupt Request 2 */
#  define ATMEGA_IRQ_INT3        5 /* 0x0008 INT3 External Interrupt Request 3 */
#  define ATMEGA_IRQ_INT4        6 /* 0x000A INT4 External Interrupt Request 4 */
#  define ATMEGA_IRQ_INT5        7 /* 0x000C INT5  External Interrupt Request 5 */
#  define ATMEGA_IRQ_INT6        8 /* 0x000E INT6 External Interrupt Request 6 */
#  define ATMEGA_IRQ_INT7        9 /* 0x0010 INT7 External Interrupt Request 7 */
#  define ATMEGA_IRQ_PCINT0      10 /* 0x0012 PCINT0 Pin Change Interrupt Req 0 */
#  define ATMEGA_IRQ_PCINT1      11 /* 0x0014 PCINT1 Pin Change Interrupt Req 1 */
#  define ATMEGA_IRQ_PCINT2      12 /* 0x0016 PCINT2 Pin Change Interrupt Req 2 */
#  define ATMEGA_IRQ_WDT         13 /* 0x0018 WDT Watchdog Time-out Interrupt */
#  define ATMEGA_IRQ_TIM2_COMPA  14 /* 0x001A TIMER2 COMPA Timer/Counter2 Compare Match A */
#  define ATMEGA_IRQ_TIM2_COMPB  15 /* 0x001C TIMER2 COMPB Timer/Counter2 Compare Match B */
#  define ATMEGA_IRQ_TIM2_OVF    16 /* 0x001E TIMER2 OVF Timer/Counter2 Overflow */
#  define ATMEGA_IRQ_TIM1_CAPT   17 /* 0x0020 TIMER1 CAPT Timer/Counter1 Capture Event */
#  define ATMEGA_IRQ_TIM1_COMPA  18 /* 0x0022 TIMER1 COMPA Timer/Counter1 Compare Match A */
#  define ATMEGA_IRQ_TIM1_COMPB  19 /* 0x0024 TIMER1 COMPB Timer/Counter1 Compare Match B */
#  define ATMEGA_IRQ_TIM1_COMPC  20 /* 0x0026 TIMER1 COMPC Timer/Counter1 Compare Match C */
#  define ATMEGA_IRQ_TIM1_OVF    21 /* 0x0028 TIMER1 OVF Timer/Counter1 Overflow */
#  define ATMEGA_IRQ_TIM0_COMPA  22 /* 0x002A TIMER0 COMPA Timer/Counter0 Compare Match A */
#  define ATMEGA_IRQ_TIM0_COMPB  23 /* 0x002C TIMER0 COMPB Timer/Counter0 Compare match B */
#  define ATMEGA_IRQ_TIM0_OVF    24 /* 0x002E TIMER0 OVF Timer/Counter0 Overflow */
#  define ATMEGA_IRQ_SPI_STC     25 /* 0x0030 SPI, STC SPI Serial Transfer Complete */
#  define ATMEGA_IRQ_U0RX        26 /* 0x0032 USART0 RX USART0 Rx Complete */
#  define ATMEGA_IRQ_U0DRE       27 /* 0x0034 USART0 UDRE USART0 Data Register Empty */
#  define ATMEGA_IRQ_U0TX        28 /* 0x0036 USART0 TX USART0 Tx Complete */
#  define ATMEGA_IRQ_ANA_COMP    29 /* 0x0038 ANALOG COMP Analog Comparator */
#  define ATMEGA_IRQ_ADC         30 /* 0x003A ADC ADC Conversion Complete */
#  define ATMEGA_IRQ_EE_RDY      31 /* 0x003C EE READY EEPROM Ready */
#  define ATMEGA_IRQ_TIM3_CAPT   32 /* 0x003E TIMER3 CAPT Timer/Counter3 Capture Event */
#  define ATMEGA_IRQ_TIM3_COMPA  33 /* 0x0040 TIMER3 COMPA Timer/Counter3 Compare Match A */
#  define ATMEGA_IRQ_TIM3_COMPB  34 /* 0x0042 TIMER3 COMPB Timer/Counter3 Compare Match B */
#  define ATMEGA_IRQ_TIM3_COMPC  35 /* 0x0044 TIMER3 COMPC Timer/Counter3 Compare Match C */
#  define ATMEGA_IRQ_TIM3_OVF    36 /* 0x0046 TIMER3 OVF Timer/Counter3 Overflow */
#  define ATMEGA_IRQ_U1RX        37 /* 0x0048 USART1 RX USART1 Rx Complete */
#  define ATMEGA_IRQ_U1DRE       38 /* 0x004A USART1 UDRE USART1 Data Register Empty */
#  define ATMEGA_IRQ_U1TX        39 /* 0x004C USART1 TX USART1 Tx Complete */
#  define ATMEGA_IRQ_TWI         40 /* 0x004E TWI 2-wire Serial Interface */
#  define ATMEGA_IRQ_SPM_RDY     41 /* 0x0050 SPM READY Store Program Memory Ready */
#  define ATMEGA_IRQ_TIM4_CAPT   42 /* 0x0052 TIMER4 CAPT Timer/Counter4 Capture Event */
#  define ATMEGA_IRQ_TIM4_COMPA  43 /* 0x0054 TIMER4 COMPA Timer/Counter4 Compare Match A */
#  define ATMEGA_IRQ_TIM4_COMPB  44 /* 0x0056 TIMER4 COMPB Timer/Counter4 Compare Match B */
#  define ATMEGA_IRQ_TIM4_COMPC  45 /* 0x0058 TIMER4 COMPC Timer/Counter4 Compare Match C */
#  define ATMEGA_IRQ_TIM4_OVF    46 /* 0x005A TIMER4 OVF Timer/Counter4 Overflow */
#  define ATMEGA_IRQ_TIM5_CAPT   47 /* 0x005C TIMER5 CAPT Timer/Counter5 Capture Event */
#  define ATMEGA_IRQ_TIM5_COMPA  48 /* 0x005E TIMER5 COMPA Timer/Counter5 Compare Match A */
#  define ATMEGA_IRQ_TIM5_COMPB  49 /* 0x0060 TIMER5 COMPB Timer/Counter5 Compare Match B */
#  define ATMEGA_IRQ_TIM5_COMPC  50 /* 0x0062 TIMER5 COMPC Timer/Counter5 Compare Match C */
#  define ATMEGA_IRQ_TIM5_OVF    51 /* 0x0064 TIMER5 OVF Timer/Counter5 Overflow */
#  define ATMEGA_IRQ_U2RX        52 /* 0x0066 USART2 RX USART2 Rx Complete */
#  define ATMEGA_IRQ_U2DRE       53 /* 0x0068 USART2 UDRE USART2 Data Register Empty */
#  define ATMEGA_IRQ_U2TX        54 /* 0x006A USART2 TX USART2 Tx Complete */
#  define ATMEGA_IRQ_U3RX        55 /* 0x006C USART3 RX USART3 Rx Complete */
#  define ATMEGA_IRQ_U3DRE       56 /* 0x006E USART3 UDRE USART3 Data Register Empty */
#  define ATMEGA_IRQ_U3TX        57 /* 0x0070 USART3 TX USART3 Tx Complete */

#  define NR_IRQS                58
#  define AVR_PC_SIZE            24
#  define XCPTCONTEXT_REGS       38 /* Size of the register state save array (in bytes) */

#else
  #error "Unrecognized chip"
#endif

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

#endif /* __ARCH_AVR_INCLUDE_ATMEGA_IRQ_H */
