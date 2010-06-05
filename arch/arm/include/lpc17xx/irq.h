/****************************************************************************
 * arch/lpc17xxx/irq.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_LPC17XX_IRQ_H
#define __ARCH_LPC17XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define LPC17_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define LPC17_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define LPC17_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define LPC17_IRQ_MPU            (4) /* Vector  4: Memory management (MPU) */
#define LPC17_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define LPC17_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define LPC17_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define LPC17_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define LPC17_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define LPC17_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define LPC17_IRQ_EXTINT        (16)                  /* Vector number of the first external interrupt */
#define LPC17_IRQ_WDT           (LPC17_IRQ_EXTINT+0)  /* WDT Watchdog Interrupt (WDINT) */
#define LPC17_IRQ_TMR0          (LPC17_IRQ_EXTINT+1)  /* Timer 0 Match 0 - 1 (MR0, MR1)
                                                       * Capture 0 - 1 (CR0, CR1) */
#define LPC17_IRQ_TMR1          (LPC17_IRQ_EXTINT+2)  /* Timer 1 Match 0 - 2 (MR0, MR1, MR2)
                                                       * Capture 0 - 1 (CR0, CR1) */
#define LPC17_IRQ_TMR2          (LPC17_IRQ_EXTINT+3)  /* Timer 2 Match 0-3
                                                       * Capture 0-1 */
#define LPC17_IRQ_TMR3          (LPC17_IRQ_EXTINT+4)  /* Timer 3 Match 0-3
                                                       * Capture 0-1 */
#define LPC17_IRQ_UART0         (LPC17_IRQ_EXTINT+5)  /* UART 0 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_UART1         (LPC17_IRQ_EXTINT+6)  /* UART 1 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * Modem Control Change
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_UART2         (LPC17_IRQ_EXTINT+7)  /* UART 2 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_UART3         (LPC17_IRQ_EXTINT+8)  /* UART 3 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_PWM1          (LPC17_IRQ_EXTINT+9)  /* PWM1 Match 0 - 6 of PWM1
                                                       * Capture 0-1 of PWM1 */
#define LPC17_IRQ_I2C0          (LPC17_IRQ_EXTINT+10) /* I2C0 SI (state change) */
#define LPC17_IRQ_I2C1          (LPC17_IRQ_EXTINT+11) /* I2C1 SI (state change) */
#define LPC17_IRQ_I2C2          (LPC17_IRQ_EXTINT+12) /* I2C2 SI (state change) */
#define LPC17_IRQ_SPIF           (LPC17_IRQ_EXTINT+13) /* SPI SPI Interrupt Flag (SPIF)
                                                       * Mode Fault (MODF) */
#define LPC17_IRQ_SSP0          (LPC17_IRQ_EXTINT+14) /* SSP0 Tx FIFO half empty of SSP0
                                                       * Rx FIFO half full of SSP0
                                                       * Rx Timeout of SSP0
                                                       * Rx Overrun of SSP0 */
#define LPC17_IRQ_SSP1          (LPC17_IRQ_EXTINT+15) /* SSP 1 Tx FIFO half empty
                                                       * Rx FIFO half full
                                                       * Rx Timeout
                                                       * Rx Overrun */
#define LPC17_IRQ_PLL0          (LPC17_IRQ_EXTINT+16) /* PLL0 (Main PLL) PLL0 Lock (PLOCK0) */
#define LPC17_IRQ_RTC           (LPC17_IRQ_EXTINT+17) /* RTC Counter Increment (RTCCIF)
                                                       * Alarm (RTCALF) */
#define LPC17_IRQ_EINT0         (LPC17_IRQ_EXTINT+18) /* External Interrupt 0 (EINT0) */
#define LPC17_IRQ_EINT1         (LPC17_IRQ_EXTINT+19) /* External Interrupt 1 (EINT1) */
#define LPC17_IRQ_EINT2         (LPC17_IRQ_EXTINT+20) /* External Interrupt 2 (EINT2) */
#define LPC17_IRQ_EINT3         (LPC17_IRQ_EXTINT+21) /* External Interrupt 3 (EINT3)
                                                       * Note: EINT3 channel is shared with GPIO interrupts */
#define LPC17_IRQ_ADC           (LPC17_IRQ_EXTINT+22) /* ADC A/D Converter end of conversion */
#define LPC17_IRQ_BOD           (LPC17_IRQ_EXTINT+23) /* BOD Brown Out detect */
#define LPC17_IRQ_USB           (LPC17_IRQ_EXTINT+24) /* USB USB_INT_REQ_LP, USB_INT_REQ_HP,
                                                       * USB_INT_REQ_DMA */
#define LPC17_IRQ_CAN           (LPC17_IRQ_EXTINT+25) /* CAN CAN Common, CAN 0 Tx, CAN 0 Rx,
                                                       *                 CAN 1 Tx, CAN 1 Rx */
#define LPC17_IRQ_GPDMA         (LPC17_IRQ_EXTINT+26) /* GPDMA IntStatus of DMA channel 0,
                                                       *       IntStatus of DMA channel 1 */
#define LPC17_IRQ_I2S           (LPC17_IRQ_EXTINT+27) /* I2S irq, dmareq1, dmareq2 */
#define LPC17_IRQ_ETH           (LPC17_IRQ_EXTINT+28) /* Ethernet WakeupInt, SoftInt, TxDoneInt,
                                                       * TxFinishedInt, TxErrorInt,* TxUnderrunInt,
                                                       * RxDoneInt, RxFinishedInt, RxErrorInt,
                                                       * RxOverrunInt */
#define LPC17_IRQ_RITINT        (LPC17_IRQ_EXTINT+29) /* Repetitive Interrupt Timer (RITINT) */
#define LPC17_IRQ_MCPWM         (LPC17_IRQ_EXTINT+30) /* Motor Control PWM IPER[2:0], IPW[2:0],
                                                       * ICAP[2:0], FES */
#define LPC17_IRQ_QEI           (LPC17_IRQ_EXTINT+31) /* Quadrature Encoder INX_Int, TIM_Int, VELC_Int,
                                                       * DIR_Int, ERR_Int, ENCLK_Int, POS0_Int, POS1_Int
                                                       * POS2_Int, REV_Int, POS0REV_Int, OS1REV_Int,
                                                       * POS2REV_Int */
#define LPC17_IRQ_PLL1          (LPC17_IRQ_EXTINT+32) /* PLL1 (USB PLL) PLL1 Lock (PLOCK1) */
#define LPC17_IRQ_USBACT        (LPC17_IRQ_EXTINT+33) /* USB Activity Interrupt USB_NEED_CLK */
#define LPC17_IRQ_CANACT        (LPC17_IRQ_EXTINT+34) /* CAN Activity Interrupt CAN1WAKE, CAN2WAKE */
#define LPC17_IRQ_NEXTINT       (35)

/* No GPIO interrupts yet */

#define LPC17_NGPIOAIRQS        0

/* Total number of IRQ numbers */

#define NR_IRQS                 (LPC17_IRQ_EXTINT+LPC17_IRQ_NEXTINT+LPC17_NGPIOAIRQS)


/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_LPC17XX_IRQ_H */

