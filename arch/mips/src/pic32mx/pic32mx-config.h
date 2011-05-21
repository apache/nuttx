/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-config.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_LPC17XX_LPC17_PIC32_H
#define __ARCH_MIPS_SRC_LPC17XX_LPC17_PIC32_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"
#include "pic32mx-memorymap.h"
#include "pic32mx-uart.h"
#include "pic32mx-int.h"
#include "pic32mx-devcfg.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Interrupt Priorities *************************************************************/

       
#ifndef CONFIG_PIC32MX_WDTPRIO
#  define CONFIG_PIC32MX_WDTPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_WDTPRIO < 4
#  error "CONFIG_PIC32MX_WDTPRIO is too small"
#endif
#if CONFIG_PIC32MX_WDTPRIO > 31
#  error "CONFIG_PIC32MX_WDTPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_RTCCPRIO
#  define CONFIG_PIC32MX_RTCCPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_RTCCPRIO < 4
#  error "CONFIG_PIC32MX_RTCCPRIO is too small"
#endif
#if CONFIG_PIC32MX_RTCCPRIO > 31
#  error "CONFIG_PIC32MX_RTCCPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T1PRIO
#  define CONFIG_PIC32MX_T1PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T1PRIO < 4
#  error "CONFIG_PIC32MX_T1PRIO is too small"
#endif
#if CONFIG_PIC32MX_T1PRIO > 31
#  error "CONFIG_PIC32MX_T1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T2PRIO
#  define CONFIG_PIC32MX_T2PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T2PRIO < 4
#  error "CONFIG_PIC32MX_T2PRIO is too small"
#endif
#if CONFIG_PIC32MX_T2PRIO > 31
#  error "CONFIG_PIC32MX_T2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T3PRIO
#  define CONFIG_PIC32MX_T3PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T3PRIO < 4
#  error "CONFIG_PIC32MX_T3PRIO is too small"
#endif
#if CONFIG_PIC32MX_T3PRIO > 31
#  error "CONFIG_PIC32MX_T3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T4PRIO
#  define CONFIG_PIC32MX_T4PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T4PRIO < 4
#  error "CONFIG_PIC32MX_T4PRIO is too small"
#endif
#if CONFIG_PIC32MX_T4PRIO > 31
#  error "CONFIG_PIC32MX_T4PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T5PRIO
#  define CONFIG_PIC32MX_T5PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T5PRIO < 4
#  error "CONFIG_PIC32MX_T5PRIO is too small"
#endif
#if CONFIG_PIC32MX_T5PRIO > 31
#  error "CONFIG_PIC32MX_T5PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC1PRIO
#  define CONFIG_PIC32MX_IC1PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC1PRIO < 4
#  error "CONFIG_PIC32MX_IC1PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC1PRIO > 31
#  error "CONFIG_PIC32MX_IC1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC2PRIO
#  define CONFIG_PIC32MX_IC2PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC2PRIO < 4
#  error "CONFIG_PIC32MX_IC2PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC2PRIO > 31
#  error "CONFIG_PIC32MX_IC2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC3PRIO
#  define CONFIG_PIC32MX_IC3PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC3PRIO < 4
#  error "CONFIG_PIC32MX_IC3PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC3PRIO > 31
#  error "CONFIG_PIC32MX_IC3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC4PRIO
#  define CONFIG_PIC32MX_IC4PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC4PRIO < 4
#  error "CONFIG_PIC32MX_IC4PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC4PRIO > 31
#  error "CONFIG_PIC32MX_IC4PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC5PRIO
#  define CONFIG_PIC32MX_IC5PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC5PRIO < 4
#  error "CONFIG_PIC32MX_IC5PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC5PRIO > 31
#  error "CONFIG_PIC32MX_IC5PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC1PRIO
#  define CONFIG_PIC32MX_OC1PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC1PRIO < 4
#  error "CONFIG_PIC32MX_OC1PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC1PRIO > 31
#  error "CONFIG_PIC32MX_OC1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC2PRIO
#  define CONFIG_PIC32MX_OC2PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC2PRIO < 4
#  error "CONFIG_PIC32MX_OC2PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC2PRIO > 31
#  error "CONFIG_PIC32MX_OC2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC3PRIO
#  define CONFIG_PIC32MX_OC3PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC3PRIO < 4
#  error "CONFIG_PIC32MX_OC3PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC3PRIO > 31
#  error "CONFIG_PIC32MX_OC3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC4PRIO
#  define CONFIG_PIC32MX_OC4PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC4PRIO < 4
#  error "CONFIG_PIC32MX_OC4PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC4PRIO > 31
#  error "CONFIG_PIC32MX_OC4PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC5PRIO
#  define CONFIG_PIC32MX_OC5PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC5PRIO < 4
#  error "CONFIG_PIC32MX_OC5PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC5PRIO > 31
#  error "CONFIG_PIC32MX_OC5PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_I2C1PRIO
#  define CONFIG_PIC32MX_I2C1PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_I2C1PRIO < 4
#  error "CONFIG_PIC32MX_I2C1PRIO is too small"
#endif
#if CONFIG_PIC32MX_I2C1PRIO > 31
#  error "CONFIG_PIC32MX_I2C1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_I2C2PRIO
#  define CONFIG_PIC32MX_I2C2PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_I2C2PRIO < 4
#  error "CONFIG_PIC32MX_I2C2PRIO is too small"
#endif
#if CONFIG_PIC32MX_I2C2PRIO > 31
#  error "CONFIG_PIC32MX_I2C2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_SPI1PRIO
#  define CONFIG_PIC32MX_SPI1PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_SPI1PRIO < 4
#  error "CONFIG_PIC32MX_SPI1PRIO is too small"
#endif
#if CONFIG_PIC32MX_SPI1PRIO > 31
#  error "CONFIG_PIC32MX_SPI1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_SPI2PRIO
#  define CONFIG_PIC32MX_SPI2PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_SPI2PRIO < 4
#  error "CONFIG_PIC32MX_SPI2PRIO is too small"
#endif
#if CONFIG_PIC32MX_SPI2PRIO > 31
#  error "CONFIG_PIC32MX_SPI2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_UART1PRIO
#  define CONFIG_PIC32MX_UART1PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_UART1PRIO < 4
#  error "CONFIG_PIC32MX_UART1PRIO is too small"
#endif
#if CONFIG_PIC32MX_UART1PRIO > 31
#  error "CONFIG_PIC32MX_UART1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_UART2PRIO
#  define CONFIG_PIC32MX_UART2PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_UART2PRIO < 4
#  error "CONFIG_PIC32MX_UART2PRIO is too small"
#endif
#if CONFIG_PIC32MX_UART2PRIO > 31
#  error "CONFIG_PIC32MX_UART2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_PMPPRIO
#  define CONFIG_PIC32MX_PMPPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_PMPPRIO < 4
#  error "CONFIG_PIC32MX_PMPPRIO is too small"
#endif
#if CONFIG_PIC32MX_PMPPRIO > 31
#  error "CONFIG_PIC32MX_PMPPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_ADCPRIO
#  define CONFIG_PIC32MX_ADCPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_ADCPRIO < 4
#  error "CONFIG_PIC32MX_ADCPRIO is too small"
#endif
#if CONFIG_PIC32MX_ADCPRIO > 31
#  error "CONFIG_PIC32MX_ADCPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CVRPRIO
#  define CONFIG_PIC32MX_CVRPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CVRPRIO < 4
#  error "CONFIG_PIC32MX_CVRPRIO is too small"
#endif
#if CONFIG_PIC32MX_CVRPRIO > 31
#  error "CONFIG_PIC32MX_CVRPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CM1PRIO
#  define CONFIG_PIC32MX_CM1PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CM1PRIO < 4
#  error "CONFIG_PIC32MX_CM1PRIO is too small"
#endif
#if CONFIG_PIC32MX_CM1PRIO > 31
#  error "CONFIG_PIC32MX_CM1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CM2PRIO
#  define CONFIG_PIC32MX_CM2PRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CM2PRIO < 4
#  error "CONFIG_PIC32MX_CM2PRIO is too small"
#endif
#if CONFIG_PIC32MX_CM2PRIO > 31
#  error "CONFIG_PIC32MX_CM2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OSCPRIO
#  define CONFIG_PIC32MX_OSCPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OSCPRIO < 4
#  error "CONFIG_PIC32MX_OSCPRIO is too small"
#endif
#if CONFIG_PIC32MX_OSCPRIO > 31
#  error "CONFIG_PIC32MX_OSCPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_DDPPRIO
#  define CONFIG_PIC32MX_DDPPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_DDPPRIO < 4
#  error "CONFIG_PIC32MX_DDPPRIO is too small"
#endif
#if CONFIG_PIC32MX_DDPPRIO > 31
#  error "CONFIG_PIC32MX_DDPPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_FLASHPRIO
#  define CONFIG_PIC32MX_FLASHPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_FLASHPRIO < 4
#  error "CONFIG_PIC32MX_FLASHPRIO is too small"
#endif
#if CONFIG_PIC32MX_FLASHPRIO > 31
#  error "CONFIG_PIC32MX_FLASHPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_BMXPRIO
#  define CONFIG_PIC32MX_BMXPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_BMXPRIO < 4
#  error "CONFIG_PIC32MX_BMXPRIO is too small"
#endif
#if CONFIG_PIC32MX_BMXPRIO > 31
#  error "CONFIG_PIC32MX_BMXPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_DMAPRIO
#  define CONFIG_PIC32MX_DMAPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_DMAPRIO < 4
#  error "CONFIG_PIC32MX_DMAPRIO is too small"
#endif
#if CONFIG_PIC32MX_DMAPRIO > 31
#  error "CONFIG_PIC32MX_DMAPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CHEPRIO
#  define CONFIG_PIC32MX_CHEPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CHEPRIO < 4
#  error "CONFIG_PIC32MX_CHEPRIO is too small"
#endif
#if CONFIG_PIC32MX_CHEPRIO > 31
#  error "CONFIG_PIC32MX_CHEPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_USBPRIO
#  define CONFIG_PIC32MX_USBPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_USBPRIO < 4
#  error "CONFIG_PIC32MX_USBPRIO is too small"
#endif
#if CONFIG_PIC32MX_USBPRIO > 31
#  error "CONFIG_PIC32MX_USBPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IOPORTAPRIO
#  define CONFIG_PIC32MX_IOPORTAPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IOPORTAPRIO < 4
#  error "CONFIG_PIC32MX_IOPORTAPRIO is too small"
#endif
#if CONFIG_PIC32MX_IOPORTAPRIO > 31
#  error "CONFIG_PIC32MX_IOPORTAPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IOPORTBPRIO
#  define CONFIG_PIC32MX_IOPORTBPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IOPORTBPRIO < 4
#  error "CONFIG_PIC32MX_IOPORTBPRIO is too small"
#endif
#if CONFIG_PIC32MX_IOPORTBPRIO > 31
#  error "CONFIG_PIC32MX_IOPORTBPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IOPORTCPRIO
#  define CONFIG_PIC32MX_IOPORTCPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IOPORTCPRIO < 4
#  error "CONFIG_PIC32MX_IOPORTCPRIO is too small"
#endif
#if CONFIG_PIC32MX_IOPORTCPRIO > 31
#  error "CONFIG_PIC32MX_IOPORTCPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IOPORTDPRIO
#  define CONFIG_PIC32MX_IOPORTDPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IOPORTDPRIO < 4
#  error "CONFIG_PIC32MX_IOPORTDPRIO is too small"
#endif
#if CONFIG_PIC32MX_IOPORTDPRIO > 31
#  error "CONFIG_PIC32MX_IOPORTDPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IOPORTEPRIO
#  define CONFIG_PIC32MX_IOPORTEPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IOPORTEPRIO < 4
#  error "CONFIG_PIC32MX_IOPORTEPRIO is too small"
#endif
#if CONFIG_PIC32MX_IOPORTEPRIO > 31
#  error "CONFIG_PIC32MX_IOPORTEPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IOPORTFPRIO
#  define CONFIG_PIC32MX_IOPORTFPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IOPORTFPRIO < 4
#  error "CONFIG_PIC32MX_IOPORTFPRIO is too small"
#endif
#if CONFIG_PIC32MX_IOPORTFPRIO > 31
#  error "CONFIG_PIC32MX_IOPORTFPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IOPORTGPRIO
#  define CONFIG_PIC32MX_IOPORTGPRIO (INT_CP0_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IOPORTGPRIO < 4
#  error "CONFIG_PIC32MX_IOPORTGPRIO is too small"
#endif
#if CONFIG_PIC32MX_IOPORTGPRIO > 31
#  error "CONFIG_PIC32MX_IOPORTGPRIO is too large"
#endif

/* UARTs ****************************************************************************/
/* Don't enable UARTs not supported by the chip. */

#if CHIP_NEUARTS < 1
#  undef CONFIG_PIC32MX_UART1
#  undef CONFIG_PIC32MX_UART2
#endif
#if CHIP_NEUARTS < 2
#  undef CONFIG_PIC32MX_UART2
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_PIC32MX_UART1) || defined(CONFIG_LPC17_UART1)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be at most one defined.  It
 * could be on any UARTn, n=0,1
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Device Configuration *************************************************************/
/* DEVCFG3 */

#ifndef CONFIG_PIC32MX_USERID
#  define CONFIG_PIC32MX_USERID   0x584e   /* "NutX" */
#endif

#ifndef CONFIG_PIC32MX_SRSSEL
#  define CONFIG_PIC32MX_SRSSEL   4         /* Middle priority */
#endif

#ifdef CONFIG_PIC32MX_USB
#  ifndef CONFIG_PIC32MX_USBIDO
#    define CONFIG_PIC32MX_USBIDO 1        /* USBID pin is controlled by the USB module */
#  endif
#  ifndef CONFIG_PIC32MX_VBUSIO
#    define CONFIG_PIC32MX_VBUSIO 1        /* VBUSON pin is controlled by the USB module */
#  endif
#else
#  ifndef CONFIG_PIC32MX_USBIDO
#    define CONFIG_PIC32MX_USBIDO 0        /* USBID pin is controlled by the Port function */
#  endif
#  ifndef CONFIG_PIC32MX_VBUSIO
#    define CONFIG_PIC32MX_VBUSIO 0        /* VBUSON pin is controlled by the Port function */
#  endif
#endif

/* DEVCFG2 */

#undef CONFIG_PIC32MX_PLLIDIV
#if BOARD_PLL_IDIV == 1
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV1
#elif BOARD_PLL_IDIV == 2
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV2
#elif BOARD_PLL_IDIV == 3
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV3
#elif BOARD_PLL_IDIV == 4
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV4
#elif BOARD_PLL_IDIV == 5
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV5
#elif BOARD_PLL_IDIV == 6
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV6
#elif BOARD_PLL_IDIV == 10
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV10
#elif BOARD_PLL_IDIV == 12
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV12
#else
#  error "Unsupported BOARD_PLL_IDIV"
#endif

#undef CONFIG_PIC32MX_PLLMULT
#if BOARD_PLL_MULT == 15
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL15
#elif BOARD_PLL_MULT == 16
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL16
#elif BOARD_PLL_MULT == 17
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL17
#elif BOARD_PLL_MULT == 18
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL18
#elif BOARD_PLL_MULT == 19
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL19
#elif BOARD_PLL_MULT == 20
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL20
#elif BOARD_PLL_MULT == 21
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL21
#elif BOARD_PLL_MULT == 24
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL24
#else
#  error "Unsupported BOARD_PLL_MULT"
#endif

#undef CONFIG_PIC32MX_UPLLIDIV
#if BOARD_UPLL_IDIV == 15
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL15
#elif BOARD_UPLL_IDIV == 16
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL16
#elif BOARD_UPLL_IDIV == 17
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL17
#elif BOARD_UPLL_IDIV == 18
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL18
#elif BOARD_UPLL_IDIV == 19
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL19
#elif BOARD_UPLL_IDIV == 20
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL20
#elif BOARD_UPLL_IDIV == 21
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL21
#elif BOARD_UPLL_IDIV == 24
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FPLLMULT_MUL24
#else
#  error "Unsupported BOARD_UPLL_IDIV"
#endif

#undef CONFIG_PIC32MX_PLLODIV
#if BOARD_PLL_ODIV == 1
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV1
#elif BOARD_PLL_ODIV == 2
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 4
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 8
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 16
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 32
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 64
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 128
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#else
#  error "Unsupported BOARD_PLL_ODIV"
#endif

/* DEVCFG1 */

#undef CONFIG_PIC32MX_PBDIV
#if BOARD_PBDIV == 1
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV1
#elif BOARD_PBDIV == 2
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV2
#elif BOARD_PBDIV == 4
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV4
#elif BOARD_PBDIV == 8
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV8
#else
#  error "Unsupported BOARD_PBDIV"
#endif

#undef CONFIG_PIC32MX_WDPS
#if BOARD_WD_PRESCALER == 1
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_1
#elif BOARD_WD_PRESCALER == 2
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_2
#elif BOARD_WD_PRESCALER == 4
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_4
#elif BOARD_WD_PRESCALER == 8
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_8
#elif BOARD_WD_PRESCALER == 16
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_16
#elif BOARD_WD_PRESCALER == 32
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_32
#elif BOARD_WD_PRESCALER == 64
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_64
#elif BOARD_WD_PRESCALER == 128
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_128
#elif BOARD_WD_PRESCALER == 256
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_256
#elif BOARD_WD_PRESCALER == 512
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_512
#elif BOARD_WD_PRESCALER == 1024
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_1024
#elif BOARD_WD_PRESCALER == 2048
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_2048
#elif BOARD_WD_PRESCALER == 4096
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_4096
#elif BOARD_WD_PRESCALER == 8192
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_8192
#elif BOARD_WD_PRESCALER == 16384
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_16384
#elif BOARD_WD_PRESCALER == 32768
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_32768
#elif BOARD_WD_PRESCALER == 65536
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_65536
#elif BOARD_WD_PRESCALER == 131072
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_131072
#elif BOARD_WD_PRESCALER == 262144
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_262144
#elif BOARD_WD_PRESCALER == 524288
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_524288
#elif BOARD_WD_PRESCALER == 1048576
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_1048576
#else
#  error "Unsupported BOARD_WD_PRESCALER"
#endif

#undef CONFIG_PIC32MX_WDENABLE
#if BOARD_WD_ENABLE
#  define CONFIG_PIC32MX_WDENABLE DEVCFG1_FWDTEN
#else
#  define CONFIG_PIC32MX_WDENABLE 0
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_MIPS_SRC_LPC17XX_LPC17_PIC32_H */
