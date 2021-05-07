/****************************************************************************
 * arch/arm/include/imx1/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_IMX1_IRQ_H
#define __ARCH_ARM_INCLUDE_IMX1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* i.MX1 Interrupts */

#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_UART3PFERR         ( 0)
#  define IMX_IRQ_UART3RTS           ( 1)
#  define IMX_IRQ_UART3DTR           ( 2)
#  define IMX_IRQ_UART3UARTC         ( 3)
#  define IMX_IRQ_UART3TX            ( 4)
#  define IMX_IRQ_PENUP              ( 5)
#endif
#define IMX_IRQ_CSI                  ( 6)
#define IMX_IRQ_MMAMAC               ( 7)
#define IMX_IRQ_MMA                  ( 8)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_COMP               ( 9)
#endif
#define IMX_IRQ_MSHCXINT             (10)
#define IMX_IRQ_GPIOPORTA            (11)
#define IMX_IRQ_GPIOPORTB            (12)
#define IMX_IRQ_GPIOPORTC            (13)
#define IMX_IRQ_LCDC                 (14)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_SIM                (15)
#  define IMX_IRQ_SIMDATA            (16)
#endif
#define IMX_IRQ_RTC                  (17)
#define IMX_IRQ_RTCSAMINT            (18)
#define IMX_IRQ_UART2PFERR           (19)
#define IMX_IRQ_UART2RTS             (20)
#define IMX_IRQ_UART2DTR             (21)
#define IMX_IRQ_UART2UARTC           (22)
#define IMX_IRQ_UART2TX              (23)
#define IMX_IRQ_UART2RX              (24)
#define IMX_IRQ_UART1PFERR           (25)
#define IMX_IRQ_UART1RTS             (26)
#define IMX_IRQ_UART1DTR             (27)
#define IMX_IRQ_UART1UARTC           (28)
#define IMX_IRQ_UART1TX              (29)
#define IMX_IRQ_UART1RX              (30)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_PENDATA            (33)
#endif
#define IMX_IRQ_PWM                  (34)
#define IMX_IRQ_MMCSD                (35)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_SSI2TX             (36)
#  define IMX_IRQ_SSI2RX             (37)
#  define IMX_IRQ_SSI2ERR            (38)
#endif
#define IMX_IRQ_I2C                  (39)
#define IMX_IRQ_CSPI2                (40)
#define IMX_IRQ_CSPI1                (41)
#define IMX_IRQ_SSITX                (42)
#define IMX_IRQ_SSITXERR             (43)
#define IMX_IRQ_SSIRX                (44)
#define IMX_IRQ_SSIRXERR             (45)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_TOUCH              (46)
#endif
#define IMX_IRQ_USBD0                (47)
#define IMX_IRQ_USBD1                (48)
#define IMX_IRQ_USBD2                (49)
#define IMX_IRQ_USBD3                (50)
#define IMX_IRQ_USBD4                (51)
#define IMX_IRQ_USBD5                (52)
#define IMX_IRQ_USBD6                (53)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_UART3RX            (54)
#  define IMX_IRQ_BTSYS              (55)
#  define IMX_IRQ_BTTIM              (56)
#  define IMX_IRQ_BTWUI              (57)
#endif
#define IMX_IRQ_TIMER2               (58)
#define IMX_IRQ_TIMER1               (59)
#define IMX_IRQ_DMAERR               (60)
#define IMX_IRQ_DMA                  (61)
#define IMX_IRQ_GPIOPORTD            (62)
#define IMX_IRQ_WDT                  (63)

#define IMX_IRQ_SYSTIMER             IMX_IRQ_TIMER1
#define NR_IRQS                      (64)

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

#endif /* __ARCH_ARM_INCLUDE_IMX1_IRQ_H */
