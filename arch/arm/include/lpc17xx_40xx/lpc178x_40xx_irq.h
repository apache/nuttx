/****************************************************************************
 * arch/arm/include/lpc17xx_40xx/lpc178x_40xx_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_LPC17XX_40XX_LPC178X_40XX_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC17XX_40XX_LPC178X_40XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* External interrupts (vectors >= 16) */

#define LPC17_40_IRQ_WDT           (LPC17_40_IRQ_EXTINT+0)  /* WDT Watchdog Interrupt (WDINT) */
#define LPC17_40_IRQ_TMR0          (LPC17_40_IRQ_EXTINT+1)  /* Timer 0 Match 0 - 1 (MR0, MR1)
                                                             * Capture 0 - 1 (CR0, CR1) */
#define LPC17_40_IRQ_TMR1          (LPC17_40_IRQ_EXTINT+2)  /* Timer 1 Match 0 - 2 (MR0, MR1, MR2)
                                                             * Capture 0 - 1 (CR0, CR1) */
#define LPC17_40_IRQ_TMR2          (LPC17_40_IRQ_EXTINT+3)  /* Timer 2 Match 0-3
                                                             * Capture 0-1 */
#define LPC17_40_IRQ_TMR3          (LPC17_40_IRQ_EXTINT+4)  /* Timer 3 Match 0-3
                                                             * Capture 0-1 */
#define LPC17_40_IRQ_UART0         (LPC17_40_IRQ_EXTINT+5)  /* UART 0 Rx Line Status (RLS)
                                                             * Transmit Holding Register Empty (THRE)
                                                             * Rx Data Available (RDA)
                                                             * Character Time-out Indicator (CTI)
                                                             * End of Auto-Baud (ABEO)
                                                             * Auto-Baud Time-Out (ABTO) */
#define LPC17_40_IRQ_UART1         (LPC17_40_IRQ_EXTINT+6)  /* UART 1 Rx Line Status (RLS)
                                                             * Transmit Holding Register Empty (THRE)
                                                             * Rx Data Available (RDA)
                                                             * Character Time-out Indicator (CTI)
                                                             * Modem Control Change
                                                             * End of Auto-Baud (ABEO)
                                                             * Auto-Baud Time-Out (ABTO) */
#define LPC17_40_IRQ_UART2         (LPC17_40_IRQ_EXTINT+7)  /* UART 2 Rx Line Status (RLS)
                                                             * Transmit Holding Register Empty (THRE)
                                                             * Rx Data Available (RDA)
                                                             * Character Time-out Indicator (CTI)
                                                             * End of Auto-Baud (ABEO)
                                                             * Auto-Baud Time-Out (ABTO) */
#define LPC17_40_IRQ_UART3         (LPC17_40_IRQ_EXTINT+8)  /* UART 3 Rx Line Status (RLS)
                                                             * Transmit Holding Register Empty (THRE)
                                                             * Rx Data Available (RDA)
                                                             * Character Time-out Indicator (CTI)
                                                             * End of Auto-Baud (ABEO)
                                                             * Auto-Baud Time-Out (ABTO) */
#define LPC17_40_IRQ_PWM1          (LPC17_40_IRQ_EXTINT+9)  /* PWM1 Match 0 - 6 of PWM1
                                                             * Capture 0-1 of PWM1 */
#define LPC17_40_IRQ_I2C0          (LPC17_40_IRQ_EXTINT+10) /* I2C0 SI (state change) */
#define LPC17_40_IRQ_I2C1          (LPC17_40_IRQ_EXTINT+11) /* I2C1 SI (state change) */
#define LPC17_40_IRQ_I2C2          (LPC17_40_IRQ_EXTINT+12) /* I2C2 SI (state change) */
#define LPC17_40_IRQ_RESERVED29    (LPC17_40_IRQ_EXTINT+13) /* Unused  */
#define LPC17_40_IRQ_SSP0          (LPC17_40_IRQ_EXTINT+14) /* SSP0 Tx FIFO half empty of SSP0
                                                             * Rx FIFO half full of SSP0
                                                             * Rx Timeout of SSP0
                                                             * Rx Overrun of SSP0 */
#define LPC17_40_IRQ_SSP1          (LPC17_40_IRQ_EXTINT+15) /* SSP 1 Tx FIFO half empty
                                                             * Rx FIFO half full
                                                             * Rx Timeout
                                                             * Rx Overrun */
#define LPC17_40_IRQ_PLL0          (LPC17_40_IRQ_EXTINT+16) /* PLL0 (Main PLL) PLL0 Lock (PLOCK0) */
#define LPC17_40_IRQ_RTC           (LPC17_40_IRQ_EXTINT+17) /* RTC Counter Increment (RTCCIF)
                                                             * Alarm (RTCALF) */
#define LPC17_40_IRQ_EINT0         (LPC17_40_IRQ_EXTINT+18) /* External Interrupt 0 (EINT0) */
#define LPC17_40_IRQ_EINT1         (LPC17_40_IRQ_EXTINT+19) /* External Interrupt 1 (EINT1) */
#define LPC17_40_IRQ_EINT2         (LPC17_40_IRQ_EXTINT+20) /* External Interrupt 2 (EINT2) */
#define LPC17_40_IRQ_EINT3         (LPC17_40_IRQ_EXTINT+21) /* External Interrupt 3 (EINT3)
                                                             * Note: EINT3 channel is shared with GPIO interrupts */
#define LPC17_40_IRQ_ADC           (LPC17_40_IRQ_EXTINT+22) /* ADC A/D Converter end of conversion */
#define LPC17_40_IRQ_BOD           (LPC17_40_IRQ_EXTINT+23) /* BOD Brown Out detect */
#define LPC17_40_IRQ_USB           (LPC17_40_IRQ_EXTINT+24) /* USB USB_INT_REQ_LP, USB_INT_REQ_HP,
                                                             * USB_INT_REQ_DMA */
#define LPC17_40_IRQ_CAN           (LPC17_40_IRQ_EXTINT+25) /* CAN CAN Common, CAN 0 Tx, CAN 0 Rx,
                                                             *                 CAN 1 Tx, CAN 1 Rx */
#define LPC17_40_IRQ_GPDMA         (LPC17_40_IRQ_EXTINT+26) /* GPDMA IntStatus of DMA channel 0,
                                                             *       IntStatus of DMA channel 1 */
#define LPC17_40_IRQ_I2S           (LPC17_40_IRQ_EXTINT+27) /* I2S irq, dmareq1, dmareq2 */
#define LPC17_40_IRQ_ETH           (LPC17_40_IRQ_EXTINT+28) /* Ethernet WakeupInt, SoftInt, TxDoneInt,
                                                             * TxFinishedInt, TxErrorInt,* TxUnderrunInt,
                                                             * RxDoneInt, RxFinishedInt, RxErrorInt,
                                                             * RxOverrunInt */
#define LPC17_40_IRQ_MCI           (LPC17_40_IRQ_EXTINT+29) /* MCI SD Card Interface */
#define LPC17_40_IRQ_MCPWM         (LPC17_40_IRQ_EXTINT+30) /* Motor Control PWM IPER[2:0], IPW[2:0],
                                                             * ICAP[2:0], FES */
#define LPC17_40_IRQ_QEI           (LPC17_40_IRQ_EXTINT+31) /* Quadrature Encoder INX_Int, TIM_Int, VELC_Int,
                                                             * DIR_Int, ERR_Int, ENCLK_Int, POS0_Int, POS1_Int
                                                             * POS2_Int, REV_Int, POS0REV_Int, OS1REV_Int,
                                                             * POS2REV_Int */
#define LPC17_40_IRQ_PLL1          (LPC17_40_IRQ_EXTINT+32) /* PLL1 (USB PLL) PLL1 Lock (PLOCK1) */
#define LPC17_40_IRQ_USBACT        (LPC17_40_IRQ_EXTINT+33) /* USB Activity Interrupt USB_NEED_CLK */
#define LPC17_40_IRQ_CANACT        (LPC17_40_IRQ_EXTINT+34) /* CAN Activity Interrupt CAN1WAKE, CAN2WAKE */
#define LPC17_40_IRQ_UART4         (LPC17_40_IRQ_EXTINT+35) /* UART 4 Rx Line Status (RLS)
                                                             * Transmit Holding Register Empty (THRE)
                                                             * Rx Data Available (RDA)
                                                             * Character Time-out Indicator (CTI)
                                                             * End of Auto-Baud (ABEO)
                                                             * Auto-Baud Time-Out (ABTO) */
#define LPC17_40_IRQ_SSP2          (LPC17_40_IRQ_EXTINT+36) /* SSP2 Tx FIFO half empty of SSP2
                                                             * Rx FIFO half full of SSP2
                                                             * Rx Timeout of SSP2
                                                             * Rx Overrun of SSP2 */
#define LPC17_40_IRQ_LCD           (LPC17_40_IRQ_EXTINT+37) /* LCD interrupt
                                                             * BER, VCompI, LNBUI, FUFI, CrsrI */
#define LPC17_40_IRQ_GPIO          (LPC17_40_IRQ_EXTINT+38) /* GPIO Interrupt
                                                             * P0xREI, P2xREI, P0xFEI, P2xFEI */
#define LPC17_40_IRQ_PWM0          (LPC17_40_IRQ_EXTINT+39) /* PWM0 Match 0 - 6 of PWM0
                                                             * Capture 0-1 of PWM0 */
#define LPC17_40_IRQ_EEPROM        (LPC17_40_IRQ_EXTINT+40) /* EEPROM Interrupt
                                                             * EE_PROG_DONE, EE_RW_DONE */
#define LPC17_40_IRQ_NEXTINT       (41)
#define LPC17_40_IRQ_NIRQS         (LPC17_40_IRQ_EXTINT+LPC17_40_IRQ_NEXTINT)

/* GPIO interrupts.
 * The LPC177x_8x supports several interrupts on ports 0 and 2 (only).
 * We go through some special efforts to keep the number of IRQs
 * to a minimum in this sparse interrupt case.
 *
 * 31 interrupts on Port 0:  p0.0 - p0.30
 * 31 interrupts on Port 2:  p2.0 - p2.30
 * --
 * 42
 */

#ifdef CONFIG_LPC17_40_GPIOIRQ
#  define LPC17_40_VALID_GPIOINT0   (0xfffffffful) /* GPIO port 0 interrupt set */
#  define LPC17_40_VALID_GPIOINT2   (0xfffffffful) /* GPIO port 2 interrupt set */

/* Set 1: 16 interrupts p0.0-p0.15 */

#  define LPC17_40_VALID_SHIFT0L    (0)
#  define LPC17_40_VALID_FIRST0L    (LPC17_40_IRQ_EXTINT+LPC17_40_IRQ_NEXTINT)

#  define LPC17_40_IRQ_P0p0         (LPC17_40_VALID_FIRST0L+0)
#  define LPC17_40_IRQ_P0p1         (LPC17_40_VALID_FIRST0L+1)
#  define LPC17_40_IRQ_P0p2         (LPC17_40_VALID_FIRST0L+2)
#  define LPC17_40_IRQ_P0p3         (LPC17_40_VALID_FIRST0L+3)
#  define LPC17_40_IRQ_P0p4         (LPC17_40_VALID_FIRST0L+4)
#  define LPC17_40_IRQ_P0p5         (LPC17_40_VALID_FIRST0L+5)
#  define LPC17_40_IRQ_P0p6         (LPC17_40_VALID_FIRST0L+6)
#  define LPC17_40_IRQ_P0p7         (LPC17_40_VALID_FIRST0L+7)
#  define LPC17_40_IRQ_P0p8         (LPC17_40_VALID_FIRST0L+8)
#  define LPC17_40_IRQ_P0p9         (LPC17_40_VALID_FIRST0L+9)
#  define LPC17_40_IRQ_P0p10        (LPC17_40_VALID_FIRST0L+10)
#  define LPC17_40_IRQ_P0p11        (LPC17_40_VALID_FIRST0L+11)
#  define LPC17_40_IRQ_P0p12        (LPC17_40_VALID_FIRST0L+12)
#  define LPC17_40_IRQ_P0p13        (LPC17_40_VALID_FIRST0L+13)
#  define LPC17_40_IRQ_P0p14        (LPC17_40_VALID_FIRST0L+14)
#  define LPC17_40_IRQ_P0p15        (LPC17_40_VALID_FIRST0L+15)
#  define LPC17_40_VALID_NIRQS0L    (16)

/* Set 2: 16 interrupts p0.16-p0.31 */

#  define LPC17_40_VALID_SHIFT0H    (16)
#  define LPC17_40_VALID_FIRST0H    (LPC17_40_VALID_FIRST0L+LPC17_40_VALID_NIRQS0L)

#  define LPC17_40_IRQ_P0p16        (LPC17_40_VALID_FIRST0H+0)
#  define LPC17_40_IRQ_P0p17        (LPC17_40_VALID_FIRST0H+1)
#  define LPC17_40_IRQ_P0p18        (LPC17_40_VALID_FIRST0H+2)
#  define LPC17_40_IRQ_P0p19        (LPC17_40_VALID_FIRST0H+3)
#  define LPC17_40_IRQ_P0p20        (LPC17_40_VALID_FIRST0H+4)
#  define LPC17_40_IRQ_P0p21        (LPC17_40_VALID_FIRST0H+5)
#  define LPC17_40_IRQ_P0p22        (LPC17_40_VALID_FIRST0H+6)
#  define LPC17_40_IRQ_P0p23        (LPC17_40_VALID_FIRST0H+7)
#  define LPC17_40_IRQ_P0p24        (LPC17_40_VALID_FIRST0H+8)
#  define LPC17_40_IRQ_P0p25        (LPC17_40_VALID_FIRST0H+9)
#  define LPC17_40_IRQ_P0p26        (LPC17_40_VALID_FIRST0H+10)
#  define LPC17_40_IRQ_P0p27        (LPC17_40_VALID_FIRST0H+11)
#  define LPC17_40_IRQ_P0p28        (LPC17_40_VALID_FIRST0H+12)
#  define LPC17_40_IRQ_P0p29        (LPC17_40_VALID_FIRST0H+13)
#  define LPC17_40_IRQ_P0p30        (LPC17_40_VALID_FIRST0H+14)
#  define LPC17_40_IRQ_P0p31        (LPC17_40_VALID_FIRST0H+15)
#  define LPC17_40_VALID_NIRQS0H    (16)

/* Set 3: 16 interrupts p2.0-p2.15 */

#  define LPC17_40_VALID_SHIFT2L    (0)
#  define LPC17_40_VALID_FIRST2L    (LPC17_40_VALID_FIRST0H+LPC17_40_VALID_NIRQS0H)

#  define LPC17_40_IRQ_P2p0         (LPC17_40_VALID_FIRST2L+0)
#  define LPC17_40_IRQ_P2p1         (LPC17_40_VALID_FIRST2L+1)
#  define LPC17_40_IRQ_P2p2         (LPC17_40_VALID_FIRST2L+2)
#  define LPC17_40_IRQ_P2p3         (LPC17_40_VALID_FIRST2L+3)
#  define LPC17_40_IRQ_P2p4         (LPC17_40_VALID_FIRST2L+4)
#  define LPC17_40_IRQ_P2p5         (LPC17_40_VALID_FIRST2L+5)
#  define LPC17_40_IRQ_P2p6         (LPC17_40_VALID_FIRST2L+6)
#  define LPC17_40_IRQ_P2p7         (LPC17_40_VALID_FIRST2L+7)
#  define LPC17_40_IRQ_P2p8         (LPC17_40_VALID_FIRST2L+8)
#  define LPC17_40_IRQ_P2p9         (LPC17_40_VALID_FIRST2L+9)
#  define LPC17_40_IRQ_P2p10        (LPC17_40_VALID_FIRST2L+10)
#  define LPC17_40_IRQ_P2p11        (LPC17_40_VALID_FIRST2L+11)
#  define LPC17_40_IRQ_P2p12        (LPC17_40_VALID_FIRST2L+12)
#  define LPC17_40_IRQ_P2p13        (LPC17_40_VALID_FIRST2L+13)
#  define LPC17_40_IRQ_P2p14        (LPC17_40_VALID_FIRST2L+14)
#  define LPC17_40_IRQ_P2p15        (LPC17_40_VALID_FIRST2L+15)
#  define LPC17_40_VALID_NIRQS2L    (16)

/* Set 4: 16 interrupts p2.16 - p2.31 */

#  define LPC17_40_VALID_SHIFT2H    (16)
#  define LPC17_40_VALID_FIRST2H    (LPC17_40_VALID_FIRST2L+LPC17_40_VALID_NIRQS2L)

#  define LPC17_40_IRQ_P2p16        (LPC17_40_VALID_FIRST2H+0)
#  define LPC17_40_IRQ_P2p17        (LPC17_40_VALID_FIRST2H+1)
#  define LPC17_40_IRQ_P2p18        (LPC17_40_VALID_FIRST2H+2)
#  define LPC17_40_IRQ_P2p19        (LPC17_40_VALID_FIRST2H+3)
#  define LPC17_40_IRQ_P2p20        (LPC17_40_VALID_FIRST2H+4)
#  define LPC17_40_IRQ_P2p21        (LPC17_40_VALID_FIRST2H+5)
#  define LPC17_40_IRQ_P2p22        (LPC17_40_VALID_FIRST2H+6)
#  define LPC17_40_IRQ_P2p23        (LPC17_40_VALID_FIRST2H+7)
#  define LPC17_40_IRQ_P2p24        (LPC17_40_VALID_FIRST2H+8)
#  define LPC17_40_IRQ_P2p25        (LPC17_40_VALID_FIRST2H+9)
#  define LPC17_40_IRQ_P2p26        (LPC17_40_VALID_FIRST2H+10)
#  define LPC17_40_IRQ_P2p27        (LPC17_40_VALID_FIRST2H+11)
#  define LPC17_40_IRQ_P2p28        (LPC17_40_VALID_FIRST2H+12)
#  define LPC17_40_IRQ_P2p29        (LPC17_40_VALID_FIRST2H+13)
#  define LPC17_40_IRQ_P2p30        (LPC17_40_VALID_FIRST2H+14)
#  define LPC17_40_IRQ_P2p31        (LPC17_40_VALID_FIRST2H+15)
#  define LPC17_40_VALID_NIRQS2H    (16)

#  define LPC17_40_NGPIOAIRQS       (LPC17_40_VALID_NIRQS0L+LPC17_40_VALID_NIRQS0H+LPC17_40_VALID_NIRQS2L+LPC17_40_VALID_NIRQS2H)
#else
#  define LPC17_40_NGPIOAIRQS       (0)
#endif

/* Total number of IRQ numbers */

#define NR_IRQS                     (LPC17_40_IRQ_EXTINT+LPC17_40_IRQ_NEXTINT+LPC17_40_NGPIOAIRQS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_LPC17XX_40XX_LPC178X_40XX_IRQ_H */
