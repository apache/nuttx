/************************************************************************************
 * arch/arm/src/stm32/iar/stm32_vectors.S
 *
 *   Copyright (C) 2009-2013, 2015-2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "chip.h"
#include "exc_return.h"

/************************************************************************************
 * Configuration
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  /* In kernel mode without an interrupt stack, this interrupt handler will set the
   * MSP to the stack pointer of the interrupted thread.  If the interrupted thread
   * was a privileged thread, that will be the MSP otherwise it will be the PSP.  If
   * the PSP is used, then the value of the MSP will be invalid when the interrupt
   * handler returns because it will be a pointer to an old position in the
   * unprivileged stack.  Then when the high priority interrupt occurs and uses this
   * stale MSP, there will most likely be a system failure.
   *
   * If the interrupt stack is selected, on the other hand, then the interrupt
   * handler will always set the MSP to the interrupt stack.  So when the high
   * priority interrupt occurs, it will either use the MSP of the last privileged
   * thread to run or, in the case of the nested interrupt, the interrupt stack if
   * no privileged task has run.
   */

#  if defined(CONFIG_BUILD_PROTECTED) && CONFIG_ARCH_INTERRUPTSTACK < 4
#    error Interrupt stack must be used with high priority interrupts in kernel mode
#  endif

  /* Use the BASEPRI to control interrupts is required if nested, high
   * priority interrupts are supported.
   */

#  ifndef CONFIG_ARMV7M_USEBASEPRI
#    error CONFIG_ARMV7M_USEBASEPRI must be used with CONFIG_ARCH_HIPRI_INTERRUPT
#  endif
#endif

/* Memory Map ***********************************************************************/
/*
 * 0x0800:0000 - Beginning of FLASH. Address of vectors (if not using bootloader)
 *               Mapped to address 0x0000:0000 at boot time.
 * 0x0800:3000 - Address of vectors if using bootloader
 * 0x0803:ffff - End of flash
 * 0x2000:0000 - Start of SRAM and start of .data (_sdata)
 *             - End of .data (_edata) abd start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack, start of heap
 * 0x2000:ffff - End of SRAM and end of heap
 */

	MODULE stm32_vectors

	/* Forward declaration of sections. */
	SECTION	CSTACK:DATA:NOROOT(3)

IDLE_STACK  EQU(sfb(CSTACK)+CONFIG_IDLETHREAD_STACKSIZE-4)
HEAP_BASE   EQU(sfb(CSTACK)+CONFIG_IDLETHREAD_STACKSIZE)

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

/* Check if common ARMv7 interrupt vectoring is used (see arch/arm/src/armv7-m/up_vectors.S) */

#ifndef CONFIG_ARMV7M_CMNVECTOR

	PUBLIC	__vector_table
	EXTERN	__start
	EXTERN	up_doirq

/************************************************************************************
 * Macros
 ************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

HANDLER	MACRO label, irqno
	THUMB
label:
	mov		r0, #irqno
	b		exception_common
	ENDM

/************************************************************************************
 * Vectors
 ************************************************************************************/

	SECTION .intvec:CODE:ROOT(2)

/* _vectors replaced on __vector_table for IAR C-SPY Simulator */
__vector_table:

/* Processor Exceptions */

	DCD	IDLE_STACK			/* Vector  0: Reset stack pointer */
	DCD	__start				/* Vector  1: Reset vector */
	DCD	stm32_nmi			/* Vector  2: Non-Maskable Interrupt (NMI) */
	DCD	stm32_hardfault		/* Vector  3: Hard fault */
	DCD	stm32_mpu			/* Vector  4: Memory management (MPU) */
	DCD	stm32_busfault		/* Vector  5: Bus fault */
	DCD	stm32_usagefault	/* Vector  6: Usage fault */
	DCD	stm32_reserved		/* Vector  7: Reserved */
	DCD	stm32_reserved		/* Vector  8: Reserved */
	DCD	stm32_reserved		/* Vector  9: Reserved */
	DCD	stm32_reserved		/* Vector 10: Reserved */
	DCD	stm32_svcall		/* Vector 11: SVC call */
	DCD	stm32_dbgmonitor	/* Vector 12: Debug monitor */
	DCD	stm32_reserved		/* Vector 13: Reserved */
	DCD	stm32_pendsv		/* Vector 14: Pendable system service request */
	DCD	stm32_systick		/* Vector 15: System tick */

/* External Interrupts */

#if !defined(CONFIG_STM32_NOEXT_VECTORS)
#undef VECTOR
#define VECTOR(l,i) .word l

/* IAR Assembler:
 * You must not mix assembler language and C-style preprocessor directives.
 * Conceptually, they are different languages and mixing them might lead to unexpected
 * behavior because an assembler directive is not necessarily accepted as a part of the C
 * preprocessor language.
 */

#if defined(CONFIG_STM32_STM32L15XX)
#  include "chip/stm32l15xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F10XX)

#  if defined(CONFIG_STM32_VALUELINE)

	DCD	stm32_wwdg        /* Vector 16+0:  Window Watchdog interrupt */
	DCD	stm32_pvd         /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	DCD	stm32_tamper      /* Vector 16+2:  Tamper interrupt */
	DCD	stm32_rtc         /* Vector 16+3:  RTC Wakeup through EXTI line interrupt */
	DCD	stm32_flash       /* Vector 16+4:  Flash global interrupt */
	DCD	stm32_rcc         /* Vector 16+5:  RCC global interrupt */
	DCD	stm32_exti0       /* Vector 16+6:  EXTI Line 0 interrupt */
	DCD	stm32_exti1       /* Vector 16+7:  EXTI Line 1 interrupt */
	DCD	stm32_exti2       /* Vector 16+8:  EXTI Line 2 interrupt */
	DCD	stm32_exti3       /* Vector 16+9:  EXTI Line 3 interrupt */
	DCD	stm32_exti4       /* Vector 16+10: EXTI Line 4 interrupt */
	DCD	stm32_dma1ch1     /* Vector 16+11: DMA1 Channel 1 global interrupt */
	DCD	stm32_dma1ch2     /* Vector 16+12: DMA1 Channel 2 global interrupt */
	DCD	stm32_dma1ch3     /* Vector 16+13: DMA1 Channel 3 global interrupt */
	DCD	stm32_dma1ch4     /* Vector 16+14: DMA1 Channel 4 global interrupt */
	DCD	stm32_dma1ch5     /* Vector 16+15: DMA1 Channel 5 global interrupt */
	DCD	stm32_dma1ch6     /* Vector 16+16: DMA1 Channel 6 global interrupt */
	DCD	stm32_dma1ch7     /* Vector 16+17: DMA1 Channel 7 global interrupt */
	DCD	stm32_adc1        /* Vector 16+18: ADC1 global interrupt */
	DCD	stm32_reserved    /* Vector 16+19: Reserved 0 */
	DCD	stm32_reserved    /* Vector 16+20: Reserved 1 */
	DCD	stm32_reserved    /* Vector 16+21: Reserved 2 */
	DCD	stm32_reserved    /* Vector 16+22: Reserved 3 */
	DCD	stm32_exti95      /* Vector 16+23: EXTI Line[9:5] interrupts */
	DCD	stm32_tim1brk     /* Vector 16+24: TIM1 Break interrupt; TIM15 global interrupt */
	DCD	stm32_tim1up      /* Vector 16+25: TIM1 Update interrupt; TIM16 global interrupt */
	DCD	stm32_tim1trgcom  /* Vector 16+26: TIM1 Trigger and Commutation interrupts; TIM17 global interrupt */
	DCD	stm32_tim1cc      /* Vector 16+27: TIM1 Capture Compare interrupt */
	DCD	stm32_tim2        /* Vector 16+28: TIM2 global interrupt */
	DCD	stm32_tim3        /* Vector 16+29: TIM3 global interrupt */
	DCD	stm32_tim4        /* Vector 16+30: TIM4 global interrupt */
	DCD	stm32_i2c1ev      /* Vector 16+31: I2C1 event interrupt */
	DCD	stm32_i2c1er      /* Vector 16+32: I2C1 error interrupt */
	DCD	stm32_i2c2ev      /* Vector 16+33: I2C2 event interrupt */
	DCD	stm32_i2c2er      /* Vector 16+34: I2C2 error interrupt */
	DCD	stm32_spi1        /* Vector 16+35: SPI1 global interrupt */
	DCD	stm32_spi2        /* Vector 16+36: SPI2 global interrupt */
	DCD	stm32_usart1      /* Vector 16+37: USART1 global interrupt */
	DCD	stm32_usart2      /* Vector 16+38: USART2 global interrupt */
	DCD	stm32_usart3      /* Vector 16+39: USART3 global interrupt */
	DCD	stm32_exti1510    /* Vector 16+40: EXTI Line[15:10] interrupts */
	DCD	stm32_rtcalr      /* Vector 16+41: RTC alarms (A and B) through EXTI line interrupt */
	DCD	stm32_cec         /* Vector 16+42: CEC global interrupt */
	DCD	stm32_tim12       /* Vector 16+43: TIM12 global interrupt */
	DCD	stm32_tim13       /* Vector 16+44: TIM13 global interrupt */
	DCD	stm32_tim14       /* Vector 16+45: TIM14 global interrupt */
	DCD	stm32_reserved    /* Vector 16+46: Reserved 4 */
	DCD	stm32_reserved    /* Vector 16+47: Reserved 5 */
	DCD	stm32_fsmc        /* Vector 16+48: FSMC global interrupt */
	DCD	stm32_reserved    /* Vector 16+49: Reserved 6 */
	DCD	stm32_tim5        /* Vector 16+50: TIM5 global interrupt */
	DCD	stm32_spi3        /* Vector 16+51: SPI3 global interrupt */
	DCD	stm32_uart4       /* Vector 16+52: USART2 global interrupt */
	DCD	stm32_uart5       /* Vector 16+53: UART5 global interrupt */
	DCD	stm32_tim6        /* Vector 16+54: TIM6 global interrupt */
	DCD	stm32_tim7        /* Vector 16+55: TIM7 global interrupt */
	DCD	stm32_dma2ch1     /* Vector 16+56: DMA2 Channel 1 global interrupt */
	DCD	stm32_dma2ch2     /* Vector 16+57: DMA2 Channel 2 global interrupt */
	DCD	stm32_dma2ch3     /* Vector 16+58: DMA2 Channel 3 global interrupt */
	DCD	stm32_dma2ch45    /* Vector 16+59: DMA2 Channel 4 and 5 global interrupt */
	DCD	stm32_dma2ch5     /* Vector 16+60: DMA2 Channel 5 global interrupt */

#  elif defined(CONFIG_STM32_CONNECTIVITYLINE)

	DCD	stm32_wwdg        /* Vector 16+0:  Window Watchdog interrupt */
	DCD	stm32_pvd         /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	DCD	stm32_tamper      /* Vector 16+2:  Tamper interrupt */
	DCD	stm32_rtc         /* Vector 16+3:  RTC global interrupt */
	DCD	stm32_flash       /* Vector 16+4:  Flash global interrupt */
	DCD	stm32_rcc         /* Vector 16+5:  RCC global interrupt */
	DCD	stm32_exti0       /* Vector 16+6:  EXTI Line 0 interrupt */
	DCD	stm32_exti1       /* Vector 16+7:  EXTI Line 1 interrupt */
	DCD	stm32_exti2       /* Vector 16+8:  EXTI Line 2 interrupt */
	DCD	stm32_exti3       /* Vector 16+9:  EXTI Line 3 interrupt */
	DCD	stm32_exti4       /* Vector 16+10: EXTI Line 4 interrupt */
	DCD	stm32_dma1ch1     /* Vector 16+11: DMA1 Channel 1 global interrupt */
	DCD	stm32_dma1ch2     /* Vector 16+12: DMA1 Channel 2 global interrupt */
	DCD	stm32_dma1ch3     /* Vector 16+13: DMA1 Channel 3 global interrupt */
	DCD	stm32_dma1ch4     /* Vector 16+14: DMA1 Channel 4 global interrupt */
	DCD	stm32_dma1ch5     /* Vector 16+15: DMA1 Channel 5 global interrupt */
	DCD	stm32_dma1ch6     /* Vector 16+16: DMA1 Channel 6 global interrupt */
	DCD	stm32_dma1ch7     /* Vector 16+17: DMA1 Channel 7 global interrupt */
	DCD	stm32_adc12       /* Vector 16+18: ADC1 and ADC2 global interrupt */
	DCD	stm32_can1tx      /* Vector 16+19: CAN1 TX interrupts */
	DCD	stm32_can1rx0     /* Vector 16+20: CAN1 RX0 interrupts */
	DCD	stm32_can1rx      /* Vector 16+21: CAN1 RX1 interrupt */
	DCD	stm32_can1sce     /* Vector 16+22: CAN1 SCE interrupt */
	DCD	stm32_exti95      /* Vector 16+23: EXTI Line[9:5] interrupts */
	DCD	stm32_tim1brk     /* Vector 16+24: TIM1 Break interrupt */
	DCD	stm32_tim1up      /* Vector 16+25: TIM1 Update interrupt */
	DCD	stm32_tim1trgcom  /* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	DCD	stm32_tim1cc      /* Vector 16+27: TIM1 Capture Compare interrupt */
	DCD	stm32_tim2        /* Vector 16+28: TIM2 global interrupt */
	DCD	stm32_tim3        /* Vector 16+29: TIM3 global interrupt */
	DCD	stm32_tim4        /* Vector 16+30: TIM4 global interrupt */
	DCD	stm32_i2c1ev      /* Vector 16+31: I2C1 event interrupt */
	DCD	stm32_i2c1er      /* Vector 16+32: I2C1 error interrupt */
	DCD	stm32_i2c2ev      /* Vector 16+33: I2C2 event interrupt */
	DCD	stm32_i2c2er      /* Vector 16+34: I2C2 error interrupt */
	DCD	stm32_spi1        /* Vector 16+35: SPI1 global interrupt */
	DCD	stm32_spi2        /* Vector 16+36: SPI2 global interrupt */
	DCD	stm32_usart1      /* Vector 16+37: USART1 global interrupt */
	DCD	stm32_usart2      /* Vector 16+38: USART2 global interrupt */
	DCD	stm32_usart3      /* Vector 16+39: USART3 global interrupt */
	DCD	stm32_exti1510    /* Vector 16+40: EXTI Line[15:10] interrupts */
	DCD	stm32_rtcalr      /* Vector 16+41: RTC alarm through EXTI line interrupt */
	DCD	stm32_otgfswkup   /* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
	DCD	stm32_reserved    /* Vector 16+43: Reserved 0 */
	DCD	stm32_reserved    /* Vector 16+44: Reserved 1 */
	DCD	stm32_reserved    /* Vector 16+45: Reserved 2 */
	DCD	stm32_reserved    /* Vector 16+46: Reserved 3 */
	DCD	stm32_reserved    /* Vector 16+47: Reserved 4 */
	DCD	stm32_reserved    /* Vector 16+48: Reserved 5 */
	DCD	stm32_reserved    /* Vector 16+49: Reserved 6 */
	DCD	stm32_tim5        /* Vector 16+50: TIM5 global interrupt */
	DCD	stm32_spi3        /* Vector 16+51: SPI3 global interrupt */
	DCD	stm32_uart4       /* Vector 16+52: UART4 global interrupt */
	DCD	stm32_uart5       /* Vector 16+53: UART5 global interrupt */
	DCD	stm32_tim6        /* Vector 16+54: TIM6 global interrupt */
	DCD	stm32_tim7        /* Vector 16+55: TIM7 global interrupt */
	DCD	stm32_dma2ch1     /* Vector 16+56: DMA2 Channel 1 global interrupt */
	DCD	stm32_dma2ch2     /* Vector 16+57: DMA2 Channel 2 global interrupt */
	DCD	stm32_dma2ch3     /* Vector 16+58: DMA2 Channel 3 global interrupt */
	DCD	stm32_dma2ch4     /* Vector 16+59: DMA2 Channel 4 global interrupt */
	DCD	stm32_dma2ch5     /* Vector 16+60: DMA2 Channel 5 global interrupt */
	DCD	stm32_eth         /* Vector 16+61: Ethernet global interrupt */
	DCD	stm32_ethwkup     /* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
	DCD	stm32_can2tx      /* Vector 16+63: CAN2 TX interrupts */
	DCD	stm32_can2rx0     /* Vector 16+64: CAN2 RX0 interrupts */
	DCD	stm32_can2rx1     /* Vector 16+65: CAN2 RX1 interrupt */
	DCD	stm32_can2sce     /* Vector 16+66: CAN2 SCE interrupt */
	DCD	stm32_otgfs       /* Vector 16+67: USB On The Go FS global interrupt */

#  else /* CONFIG_STM32_CONNECTIVITYLINE */

	DCD	stm32_wwdg        /* Vector 16+0:  Window Watchdog interrupt */
	DCD	stm32_pvd         /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	DCD	stm32_tamper      /* Vector 16+2:  Tamper interrupt */
	DCD	stm32_rtc         /* Vector 16+3:  RTC global interrupt */
	DCD	stm32_flash       /* Vector 16+4:  Flash global interrupt */
	DCD	stm32_rcc         /* Vector 16+5:  RCC global interrupt */
	DCD	stm32_exti0       /* Vector 16+6:  EXTI Line 0 interrupt */
	DCD	stm32_exti1       /* Vector 16+7:  EXTI Line 1 interrupt */
	DCD	stm32_exti2       /* Vector 16+8:  EXTI Line 2 interrupt */
	DCD	stm32_exti3       /* Vector 16+9:  EXTI Line 3 interrupt */
	DCD	stm32_exti4       /* Vector 16+10: EXTI Line 4 interrupt */
	DCD	stm32_dma1ch1     /* Vector 16+11: DMA1 Channel 1 global interrupt */
	DCD	stm32_dma1ch2     /* Vector 16+12: DMA1 Channel 2 global interrupt */
	DCD	stm32_dma1ch3     /* Vector 16+13: DMA1 Channel 3 global interrupt */
	DCD	stm32_dma1ch4     /* Vector 16+14: DMA1 Channel 4 global interrupt */
	DCD	stm32_dma1ch5     /* Vector 16+15: DMA1 Channel 5 global interrupt */
	DCD	stm32_dma1ch6     /* Vector 16+16: DMA1 Channel 6 global interrupt */
	DCD	stm32_dma1ch7     /* Vector 16+17: DMA1 Channel 7 global interrupt */
	DCD	stm32_adc12       /* Vector 16+18: ADC1 and ADC2 global interrupt */
	DCD	stm32_usbhpcantx  /* Vector 16+19: USB High Priority or CAN TX interrupts*/
	DCD	stm32_usblpcanrx0 /* Vector 16+20: USB Low Priority or CAN RX0 interrupts*/
	DCD	stm32_can1rx1     /* Vector 16+21: CAN1 RX1 interrupt */
	DCD	stm32_can1sce     /* Vector 16+22: CAN1 SCE interrupt */
	DCD	stm32_exti95      /* Vector 16+23: EXTI Line[9:5] interrupts */
	DCD	stm32_tim1brk     /* Vector 16+24: TIM1 Break interrupt */
	DCD	stm32_tim1up      /* Vector 16+25: TIM1 Update interrupt */
	DCD	stm32_tim1rtgcom  /* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	DCD	stm32_tim1cc      /* Vector 16+27: TIM1 Capture Compare interrupt */
	DCD	stm32_tim2        /* Vector 16+28: TIM2 global interrupt */
	DCD	stm32_tim3        /* Vector 16+29: TIM3 global interrupt */
	DCD	stm32_tim4        /* Vector 16+30: TIM4 global interrupt */
	DCD	stm32_i2c1ev      /* Vector 16+31: I2C1 event interrupt */
	DCD	stm32_i2c1er      /* Vector 16+32: I2C1 error interrupt */
	DCD	stm32_i2c2ev      /* Vector 16+33: I2C2 event interrupt */
	DCD	stm32_i2c2er      /* Vector 16+34: I2C2 error interrupt */
	DCD	stm32_spi1        /* Vector 16+35: SPI1 global interrupt */
	DCD	stm32_spi2        /* Vector 16+36: SPI2 global interrupt */
	DCD	stm32_usart1      /* Vector 16+37: USART1 global interrupt */
	DCD	stm32_usart2      /* Vector 16+38: USART2 global interrupt */
	DCD	stm32_usart3      /* Vector 16+39: USART3 global interrupt */
	DCD	stm32_exti1510    /* Vector 16+40: EXTI Line[15:10] interrupts */
	DCD	stm32_rtcalr      /* Vector 16+41: RTC alarm through EXTI line interrupt */
	DCD	stm32_usbwkup     /* Vector 16+42: USB wakeup from suspend through EXTI line interrupt*/
	DCD	stm32_tim8brk     /* Vector 16+43: TIM8 Break interrupt */
	DCD	stm32_tim8up      /* Vector 16+44: TIM8 Update interrupt */
	DCD	stm32_tim8trgcom  /* Vector 16+45: TIM8 Trigger and Commutation interrupts */
	DCD	stm32_tim8cc      /* Vector 16+46: TIM8 Capture Compare interrupt */
	DCD	stm32_adc3        /* Vector 16+47: ADC3 global interrupt */
	DCD	stm32_fsmc        /* Vector 16+48: FSMC global interrupt */
	DCD	stm32_sdio        /* Vector 16+49: SDIO global interrupt */
	DCD	stm32_tim5        /* Vector 16+50: TIM5 global interrupt */
	DCD	stm32_spi3        /* Vector 16+51: SPI3 global interrupt */
	DCD	stm32_uart4       /* Vector 16+52: UART4 global interrupt */
	DCD	stm32_uart5       /* Vector 16+53: UART5 global interrupt */
	DCD	stm32_tim6        /* Vector 16+54: TIM6 global interrupt */
	DCD	stm32_tim7        /* Vector 16+55: TIM7 global interrupt */
	DCD	stm32_dma2ch1     /* Vector 16+56: DMA2 Channel 1 global interrupt */
	DCD	stm32_dma2ch2     /* Vector 16+57: DMA2 Channel 2 global interrupt */
	DCD	stm32_dma2ch3     /* Vector 16+58: DMA2 Channel 3 global interrupt */
	DCD	stm32_dma2ch45    /* Vector 16+59: DMA2 Channel 4&5 global interrupt */

#  endif /* CONFIG_STM32_CONNECTIVITYLINE */

#elif defined(CONFIG_STM32_STM32F20XX)
	DCD	stm32_wwdg        /* Vector 16+0:  Window Watchdog interrupt */
	DCD	stm32_pvd         /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	DCD	stm32_tamper      /* Vector 16+2:  Tamper and time stamp interrupts */
	DCD	stm32_rtc_wkup    /* Vector 16+3:  RTC global interrupt */
	DCD	stm32_flash       /* Vector 16+4:  Flash global interrupt */
	DCD	stm32_rcc         /* Vector 16+5:  RCC global interrupt */
	DCD	stm32_exti0       /* Vector 16+6:  EXTI Line 0 interrupt */
	DCD	stm32_exti1       /* Vector 16+7:  EXTI Line 1 interrupt */
	DCD	stm32_exti2       /* Vector 16+8:  EXTI Line 2 interrupt */
	DCD	stm32_exti3       /* Vector 16+9:  EXTI Line 3 interrupt */
	DCD	stm32_exti4       /* Vector 16+10: EXTI Line 4 interrupt */
	DCD	stm32_dma1s0      /* Vector 16+11: DMA1 Stream 0 global interrupt */
	DCD	stm32_dma1s1      /* Vector 16+12: DMA1 Stream 1 global interrupt */
	DCD	stm32_dma1s2      /* Vector 16+13: DMA1 Stream 2 global interrupt */
	DCD	stm32_dma1s3      /* Vector 16+14: DMA1 Stream 3 global interrupt */
	DCD	stm32_dma1s4      /* Vector 16+15: DMA1 Stream 4 global interrupt */
	DCD	stm32_dma1s5      /* Vector 16+16: DMA1 Stream 5 global interrupt */
	DCD	stm32_dma1s6      /* Vector 16+17: DMA1 Stream 6 global interrupt */
	DCD	stm32_adc         /* Vector 16+18: ADC1, ADC2, and ADC3 global interrupt */
	DCD	stm32_can1tx      /* Vector 16+19: CAN1 TX interrupts */
	DCD	stm32_can1rx0     /* Vector 16+20: CAN1 RX0 interrupts */
	DCD	stm32_can1rx1     /* Vector 16+21: CAN1 RX1 interrupt */
	DCD	stm32_can1sce     /* Vector 16+22: CAN1 SCE interrupt */
	DCD	stm32_exti95      /* Vector 16+23: EXTI Line[9:5] interrupts */
	DCD	stm32_tim1brk     /* Vector 16+24: TIM1 Break interrupt/TIM9 global interrupt */
	DCD	stm32_tim1up      /* Vector 16+25: TIM1 Update interrupt/TIM10 global interrupt */
	DCD	stm32_tim1trgcom  /* Vector 16+26: TIM1 Trigger and Commutation interrupts/TIM11 global interrupt */
	DCD	stm32_tim1cc      /* Vector 16+27: TIM1 Capture Compare interrupt */
	DCD	stm32_tim2        /* Vector 16+28: TIM2 global interrupt */
	DCD	stm32_tim3        /* Vector 16+29: TIM3 global interrupt */
	DCD	stm32_tim4        /* Vector 16+30: TIM4 global interrupt */
	DCD	stm32_i2c1ev      /* Vector 16+31: I2C1 event interrupt */
	DCD	stm32_i2c1er      /* Vector 16+32: I2C1 error interrupt */
	DCD	stm32_i2c2ev      /* Vector 16+33: I2C2 event interrupt */
	DCD	stm32_i2c2er      /* Vector 16+34: I2C2 error interrupt */
	DCD	stm32_spi1        /* Vector 16+35: SPI1 global interrupt */
	DCD	stm32_spi2        /* Vector 16+36: SPI2 global interrupt */
	DCD	stm32_usart1      /* Vector 16+37: USART1 global interrupt */
	DCD	stm32_usart2      /* Vector 16+38: USART2 global interrupt */
	DCD	stm32_usart3      /* Vector 16+39: USART3 global interrupt */
	DCD	stm32_exti1510    /* Vector 16+40: EXTI Line[15:10] interrupts */
	DCD	stm32_rtcalrm     /* Vector 16+41: RTC alarm through EXTI line interrupt */
	DCD	stm32_otgfswkup   /* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
	DCD	stm32_tim8brk     /* Vector 16+43: TIM8 Break interrupt/TIM12 global interrupt */
	DCD	stm32_tim8up      /* Vector 16+44: TIM8 Update interrup/TIM13 global interrupt */
	DCD	stm32_tim8trgcom  /* Vector 16+45: TIM8 Trigger and Commutation interrupts/TIM14 global interrupt */
	DCD	stm32_tim8cc      /* Vector 16+46: TIM8 Capture Compare interrupt */
	DCD	stm32_dma1s7      /* Vector 16+47: DMA1 Stream 7 global interrupt */
	DCD	stm32_fsmc        /* Vector 16+48: FSMC global interrupt */
	DCD	stm32_sdio        /* Vector 16+49: SDIO global interrupt */
	DCD	stm32_tim5        /* Vector 16+50: TIM5 global interrupt */
	DCD	stm32_spi3        /* Vector 16+51: SPI3 global interrupt */
	DCD	stm32_uart4       /* Vector 16+52: UART4 global interrupt */
	DCD	stm32_uart5       /* Vector 16+53: UART5 global interrupt */
	DCD	stm32_tim6        /* Vector 16+54: TIM6 global interrupt/DAC1 and DAC2 underrun error interrupts */
	DCD	stm32_tim7        /* Vector 16+55: TIM7 global interrupt */
	DCD	stm32_dma2s0      /* Vector 16+56: DMA2 Stream 0 global interrupt */
	DCD	stm32_dma2s1      /* Vector 16+57: DMA2 Stream 1 global interrupt */
	DCD	stm32_dma2s2      /* Vector 16+58: DMA2 Stream 2 global interrupt */
	DCD	stm32_dma2s3      /* Vector 16+59: DMA2 Stream 3 global interrupt */
	DCD	stm32_dma2s4      /* Vector 16+60: DMA2 Stream 4 global interrupt */
	DCD	stm32_eth         /* Vector 16+61: Ethernet global interrupt */
	DCD	stm32_ethwkup     /* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
	DCD	stm32_can2tx      /* Vector 16+63: CAN2 TX interrupts */
	DCD	stm32_can2rx0     /* Vector 16+64: CAN2 RX0 interrupts */
	DCD	stm32_can2rx1     /* Vector 16+65: CAN2 RX1 interrupt */
	DCD	stm32_can2sce     /* Vector 16+66: CAN2 SCE interrupt */
	DCD	stm32_otgfs       /* Vector 16+67: USB On The Go FS global interrupt */
	DCD	stm32_dma2s5      /* Vector 16+68: DMA2 Stream 5 global interrupt */
	DCD	stm32_dma2s6      /* Vector 16+69: DMA2 Stream 6 global interrupt */
	DCD	stm32_dma2s7      /* Vector 16+70: DMA2 Stream 7 global interrupt */
	DCD	stm32_usart6      /* Vector 16+71: USART6 global interrupt */
	DCD	stm32_i2c3ev      /* Vector 16+72: I2C3 event interrupt */
	DCD	stm32_i2c3er      /* Vector 16+73: I2C3 error interrupt */
	DCD	stm32_otghsep1out /* Vector 16+74: USB On The Go HS End Point 1 Out global interrupt */
	DCD	stm32_otghsep1in  /* Vector 16+75: USB On The Go HS End Point 1 In global interrupt */
	DCD	stm32_otghswkup   /* Vector 16+76: USB On The Go HS Wakeup through EXTI interrupt */
	DCD	stm32_otghs       /* Vector 16+77: USB On The Go HS global interrupt */
	DCD	stm32_dcmi        /* Vector 16+78: DCMI global interrupt */
	DCD	stm32_cryp        /* Vector 16+79: CRYP crypto global interrupt */
	DCD	stm32_hash        /* Vector 16+80: Hash and Rng global interrupt */
#elif defined(CONFIG_STM32_STM32F30XX)
#  include "chip/stm32f30xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F33XX)
#  include "chip/stm32f33xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F37XX)
#  include "chip/stm32f37xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#  include "chip/stm32f42xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F446)
#  include "chip/stm32f44xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "chip/stm32f40xxx_vectors.h"
#else
#  error "No vectors for STM32 chip"
#endif
#endif /* CONFIG_STM32_NOEXT_VECTORS */

/************************************************************************************
 * .text
 ************************************************************************************/

	SECTION .text:CODE:NOROOT(8)

handlers:
	HANDLER	stm32_reserved, STM32_IRQ_RESERVED		/* Unexpected/reserved vector */
	HANDLER	stm32_nmi, STM32_IRQ_NMI				/* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER	stm32_hardfault, STM32_IRQ_HARDFAULT	/* Vector  3: Hard fault */
	HANDLER	stm32_mpu, STM32_IRQ_MEMFAULT			/* Vector  4: Memory management (MPU) */
	HANDLER	stm32_busfault, STM32_IRQ_BUSFAULT		/* Vector  5: Bus fault */
	HANDLER	stm32_usagefault, STM32_IRQ_USAGEFAULT	/* Vector  6: Usage fault */
	HANDLER	stm32_svcall, STM32_IRQ_SVCALL			/* Vector 11: SVC call */
	HANDLER	stm32_dbgmonitor, STM32_IRQ_DBGMONITOR	/* Vector 12: Debug Monitor */
	HANDLER	stm32_pendsv, STM32_IRQ_PENDSV			/* Vector 14: Penable system service request */
	HANDLER	stm32_systick, STM32_IRQ_SYSTICK		/* Vector 15: System tick */

#if !defined(CONFIG_STM32_NOEXT_VECTORS)

/* IAR Assembler:
 * You must not mix assembler language and C-style preprocessor directives.
 * Conceptually, they are different languages and mixing them might lead to unexpected
 * behavior because an assembler directive is not necessarily accepted as a part of the C
 * preprocessor language.
 */

#if defined(CONFIG_STM32_STM32L15XX)
#  include "chip/stm32l15xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F10XX)


#  if defined(CONFIG_STM32_VALUELINE)

	HANDLER	stm32_wwdg, STM32_IRQ_WWDG                 /* Vector 16+0:  Window Watchdog interrupt */
	HANDLER	stm32_pvd, STM32_IRQ_PVD                   /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	HANDLER	stm32_tamper, STM32_IRQ_TAMPER             /* Vector 16+2:  Tamper interrupt */
	HANDLER	stm32_rtc, STM32_IRQ_RTC                   /* Vector 16+3:  RTC Wakeup through EXTI line interrupt */
	HANDLER	stm32_flash, STM32_IRQ_FLASH               /* Vector 16+4:  Flash global interrupt */
	HANDLER	stm32_rcc, STM32_IRQ_RCC                   /* Vector 16+5:  RCC global interrupt */
	HANDLER	stm32_exti0, STM32_IRQ_EXTI0               /* Vector 16+6:  EXTI Line 0 interrupt */
	HANDLER	stm32_exti1, STM32_IRQ_EXTI1               /* Vector 16+7:  EXTI Line 1 interrupt */
	HANDLER	stm32_exti2, STM32_IRQ_EXTI2               /* Vector 16+8:  EXTI Line 2 interrupt */
	HANDLER	stm32_exti3, STM32_IRQ_EXTI3               /* Vector 16+9:  EXTI Line 3 interrupt */
	HANDLER	stm32_exti4, STM32_IRQ_EXTI4               /* Vector 16+10: EXTI Line 4 interrupt */
	HANDLER	stm32_dma1ch1, STM32_IRQ_DMA1CH1           /* Vector 16+11: DMA1 Channel 1 global interrupt */
	HANDLER	stm32_dma1ch2, STM32_IRQ_DMA1CH2           /* Vector 16+12: DMA1 Channel 2 global interrupt */
	HANDLER	stm32_dma1ch3, STM32_IRQ_DMA1CH3           /* Vector 16+13: DMA1 Channel 3 global interrupt */
	HANDLER	stm32_dma1ch4, STM32_IRQ_DMA1CH4           /* Vector 16+14: DMA1 Channel 4 global interrupt */
	HANDLER	stm32_dma1ch5, STM32_IRQ_DMA1CH5           /* Vector 16+15: DMA1 Channel 5 global interrupt */
	HANDLER	stm32_dma1ch6, STM32_IRQ_DMA1CH6           /* Vector 16+16: DMA1 Channel 6 global interrupt */
	HANDLER	stm32_dma1ch7, STM32_IRQ_DMA1CH7           /* Vector 16+17: DMA1 Channel 7 global interrupt */
	HANDLER	stm32_adc1, STM32_IRQ_ADC1                 /* Vector 16+18: ADC1 global interrupt */
	HANDLER	stm32_exti95, STM32_IRQ_EXTI95             /* Vector 16+23: EXTI Line[9:5] interrupts */
	HANDLER	stm32_tim1brk, STM32_IRQ_TIM1BRK           /* Vector 16+24: TIM1 Break interrupt; TIM15 global interrupt */
	HANDLER	stm32_tim1up, STM32_IRQ_TIM1UP             /* Vector 16+25: TIM1 Update interrupt; TIM16 global interrupt */
	HANDLER	stm32_tim1trgcom, STM32_IRQ_TIM1TRGCOM     /* Vector 16+26: TIM1 Trigger and Commutation interrupts; TIM17 global interrupt */
	HANDLER	stm32_tim1cc, STM32_IRQ_TIM1CC             /* Vector 16+27: TIM1 Capture Compare interrupt */
	HANDLER	stm32_tim2, STM32_IRQ_TIM2                 /* Vector 16+28: TIM2 global interrupt */
	HANDLER	stm32_tim3, STM32_IRQ_TIM3                 /* Vector 16+29: TIM3 global interrupt */
	HANDLER	stm32_tim4, STM32_IRQ_TIM4                 /* Vector 16+30: TIM4 global interrupt */
	HANDLER	stm32_i2c1ev, STM32_IRQ_I2C1EV             /* Vector 16+31: I2C1 event interrupt */
	HANDLER	stm32_i2c1er, STM32_IRQ_I2C1ER             /* Vector 16+32: I2C1 error interrupt */
	HANDLER	stm32_i2c2ev, STM32_IRQ_I2C2EV             /* Vector 16+33: I2C2 event interrupt */
	HANDLER	stm32_i2c2er, STM32_IRQ_I2C2ER             /* Vector 16+34: I2C2 error interrupt */
	HANDLER	stm32_spi1, STM32_IRQ_SPI1                 /* Vector 16+35: SPI1 global interrupt */
	HANDLER	stm32_spi2, STM32_IRQ_SPI2                 /* Vector 16+36: SPI2 global interrupt */
	HANDLER	stm32_usart1, STM32_IRQ_USART1             /* Vector 16+37: USART1 global interrupt */
	HANDLER	stm32_usart2, STM32_IRQ_USART2             /* Vector 16+38: USART2 global interrupt */
	HANDLER	stm32_usart3, STM32_IRQ_USART3             /* Vector 16+39: USART3 global interrupt */
	HANDLER	stm32_exti1510, STM32_IRQ_EXTI1510         /* Vector 16+40: EXTI Line[15:10] interrupts */
	HANDLER	stm32_rtcalr, STM32_IRQ_RTCALR             /* Vector 16+41: RTC alarms (A and B) through EXTI line interrupt */
	HANDLER	stm32_cec, STM32_IRQ_CEC                   /* Vector 16+42: CEC global interrupt */
	HANDLER	stm32_tim12, STM32_IRQ_TIM12               /* Vector 16+43: TIM12 global interrupt */
	HANDLER	stm32_tim13, STM32_IRQ_TIM13               /* Vector 16+44: TIM13 global interrupt */
	HANDLER	stm32_tim14, STM32_IRQ_TIM14               /* Vector 16+45: TIM14 global interrupt */
	HANDLER	stm32_fsmc, STM32_IRQ_FSMC                 /* Vector 16+48: FSMC global interrupt */
	HANDLER	stm32_tim5, STM32_IRQ_TIM5                 /* Vector 16+50: TIM5 global interrupt */
	HANDLER	stm32_spi3, STM32_IRQ_SPI3                 /* Vector 16+51: SPI3 global interrupt */
	HANDLER	stm32_uart4, STM32_IRQ_UART4               /* Vector 16+52: USART2 global interrupt */
	HANDLER	stm32_uart5, STM32_IRQ_UART5               /* Vector 16+53: UART5 global interrupt */
	HANDLER	stm32_tim6, STM32_IRQ_TIM6                 /* Vector 16+54: TIM6 global interrupt */
	HANDLER	stm32_tim7, STM32_IRQ_TIM7                 /* Vector 16+55: TIM7 global interrupt */
	HANDLER	stm32_dma2ch1, STM32_IRQ_DMA2CH1           /* Vector 16+56: DMA2 Channel 1 global interrupt */
	HANDLER	stm32_dma2ch2, STM32_IRQ_DMA2CH2           /* Vector 16+57: DMA2 Channel 2 global interrupt */
	HANDLER	stm32_dma2ch3, STM32_IRQ_DMA2CH3           /* Vector 16+58: DMA2 Channel 3 global interrupt */
	HANDLER	stm32_dma2ch45, STM32_IRQ_DMA2CH45         /* Vector 16+59: DMA2 Channel 4 and 5 global interrupt */
	HANDLER	stm32_dma2ch5, STM32_IRQ_DMA2CH5           /* Vector 16+60: DMA2 Channel 5 global interrupt */

#  elif defined(CONFIG_STM32_CONNECTIVITYLINE)

	HANDLER	stm32_wwdg, STM32_IRQ_WWDG                /* Vector 16+0:  Window Watchdog interrupt */
	HANDLER	stm32_pvd, STM32_IRQ_PVD                  /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	HANDLER	stm32_tamper, STM32_IRQ_TAMPER            /* Vector 16+2:  Tamper interrupt */
	HANDLER	stm32_rtc, STM32_IRQ_RTC                  /* Vector 16+3:  RTC global interrupt */
	HANDLER	stm32_flash, STM32_IRQ_FLASH              /* Vector 16+4:  Flash global interrupt */
	HANDLER	stm32_rcc, STM32_IRQ_RCC                  /* Vector 16+5:  RCC global interrupt */
	HANDLER	stm32_exti0, STM32_IRQ_EXTI0              /* Vector 16+6:  EXTI Line 0 interrupt */
	HANDLER	stm32_exti1, STM32_IRQ_EXTI1              /* Vector 16+7:  EXTI Line 1 interrupt */
	HANDLER	stm32_exti2, STM32_IRQ_EXTI2              /* Vector 16+8:  EXTI Line 2 interrupt */
	HANDLER	stm32_exti3, STM32_IRQ_EXTI3              /* Vector 16+9:  EXTI Line 3 interrupt */
	HANDLER	stm32_exti4, STM32_IRQ_EXTI4              /* Vector 16+10: EXTI Line 4 interrupt */
	HANDLER	stm32_dma1ch1, STM32_IRQ_DMA1CH1          /* Vector 16+11: DMA1 Channel 1 global interrupt */
	HANDLER	stm32_dma1ch2, STM32_IRQ_DMA1CH2          /* Vector 16+12: DMA1 Channel 2 global interrupt */
	HANDLER	stm32_dma1ch3, STM32_IRQ_DMA1CH3          /* Vector 16+13: DMA1 Channel 3 global interrupt */
	HANDLER	stm32_dma1ch4, STM32_IRQ_DMA1CH4          /* Vector 16+14: DMA1 Channel 4 global interrupt */
	HANDLER	stm32_dma1ch5, STM32_IRQ_DMA1CH5          /* Vector 16+15: DMA1 Channel 5 global interrupt */
	HANDLER	stm32_dma1ch6, STM32_IRQ_DMA1CH6          /* Vector 16+16: DMA1 Channel 6 global interrupt */
	HANDLER	stm32_dma1ch7, STM32_IRQ_DMA1CH7          /* Vector 16+17: DMA1 Channel 7 global interrupt */
	HANDLER	stm32_adc12, STM32_IRQ_ADC12              /* Vector 16+18: ADC1 and ADC2 global interrupt */
	HANDLER	stm32_can1tx, STM32_IRQ_CAN1TX            /* Vector 16+19: CAN1 TX interrupts */
	HANDLER	stm32_can1rx0, STM32_IRQ_CAN1RX0          /* Vector 16+20: CAN1 RX0 interrupts */
	HANDLER	stm32_can1rx, STM32_IRQ_CAN1RX1           /* Vector 16+21: CAN1 RX1 interrupt */
	HANDLER	stm32_can1sce, STM32_IRQ_CAN1SCE          /* Vector 16+22: CAN1 SCE interrupt */
	HANDLER	stm32_exti95, STM32_IRQ_EXTI95            /* Vector 16+23: EXTI Line[9:5] interrupts */
	HANDLER	stm32_tim1brk, STM32_IRQ_TIM1BRK          /* Vector 16+24: TIM1 Break interrupt */
	HANDLER	stm32_tim1up, STM32_IRQ_TIM1UP            /* Vector 16+25: TIM1 Update interrupt */
	HANDLER	stm32_tim1trgcom, STM32_IRQ_TIM1TRGCOM    /* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	HANDLER	stm32_tim1cc, STM32_IRQ_TIM1CC            /* Vector 16+27: TIM1 Capture Compare interrupt */
	HANDLER	stm32_tim2, STM32_IRQ_TIM2                /* Vector 16+28: TIM2 global interrupt */
	HANDLER	stm32_tim3, STM32_IRQ_TIM3                /* Vector 16+29: TIM3 global interrupt */
	HANDLER	stm32_tim4, STM32_IRQ_TIM4                /* Vector 16+30: TIM4 global interrupt */
	HANDLER	stm32_i2c1ev, STM32_IRQ_I2C1EV            /* Vector 16+31: I2C1 event interrupt */
	HANDLER	stm32_i2c1er, STM32_IRQ_I2C1ER            /* Vector 16+32: I2C1 error interrupt */
	HANDLER	stm32_i2c2ev, STM32_IRQ_I2C2EV            /* Vector 16+33: I2C2 event interrupt */
	HANDLER	stm32_i2c2er, STM32_IRQ_I2C2ER            /* Vector 16+34: I2C2 error interrupt */
	HANDLER	stm32_spi1, STM32_IRQ_SPI1                /* Vector 16+35: SPI1 global interrupt */
	HANDLER	stm32_spi2, STM32_IRQ_SPI2                /* Vector 16+36: SPI2 global interrupt */
	HANDLER	stm32_usart1, STM32_IRQ_USART1            /* Vector 16+37: USART1 global interrupt */
	HANDLER	stm32_usart2, STM32_IRQ_USART2            /* Vector 16+38: USART2 global interrupt */
	HANDLER	stm32_usart3, STM32_IRQ_USART3            /* Vector 16+39: USART3 global interrupt */
	HANDLER	stm32_exti1510, STM32_IRQ_EXTI1510        /* Vector 16+40: EXTI Line[15:10] interrupts */
	HANDLER	stm32_rtcalr, STM32_IRQ_RTCALRM           /* Vector 16+41: RTC alarm through EXTI line interrupt */
	HANDLER	stm32_otgfswkup, STM32_IRQ_OTGFSWKUP      /* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
	HANDLER	stm32_tim5, STM32_IRQ_TIM5                /* Vector 16+50: TIM5 global interrupt */
	HANDLER	stm32_spi3, STM32_IRQ_SPI3                /* Vector 16+51: SPI3 global interrupt */
	HANDLER	stm32_uart4 , STM32_IRQ_UART4             /* Vector 16+52: UART4 global interrupt */
	HANDLER	stm32_uart5, STM32_IRQ_UART5              /* Vector 16+53: UART5 global interrupt */
	HANDLER	stm32_tim6, STM32_IRQ_TIM6                /* Vector 16+54: TIM6 global interrupt */
	HANDLER	stm32_tim7, STM32_IRQ_TIM7                /* Vector 16+55: TIM7 global interrupt */
	HANDLER	stm32_dma2ch1, STM32_IRQ_DMA2CH1          /* Vector 16+56: DMA2 Channel 1 global interrupt */
	HANDLER	stm32_dma2ch2, STM32_IRQ_DMA2CH2          /* Vector 16+57: DMA2 Channel 2 global interrupt */
	HANDLER	stm32_dma2ch3, STM32_IRQ_DMA2CH3          /* Vector 16+58: DMA2 Channel 3 global interrupt */
	HANDLER	stm32_dma2ch4, STM32_IRQ_DMA2CH4          /* Vector 16+59: DMA2 Channel 4 global interrupt */
	HANDLER	stm32_dma2ch5, STM32_IRQ_DMA2CH5          /* Vector 16+60: DMA2 Channel 5 global interrupt */
	HANDLER	stm32_eth, STM32_IRQ_ETH                  /* Vector 16+61: Ethernet global interrupt */
	HANDLER	stm32_ethwkup, STM32_IRQ_ETHWKUP          /* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
	HANDLER	stm32_can2tx, STM32_IRQ_CAN2TX            /* Vector 16+63: CAN2 TX interrupts */
	HANDLER	stm32_can2rx0, STM32_IRQ_CAN2RX0          /* Vector 16+64: CAN2 RX0 interrupts */
	HANDLER	stm32_can2rx1, STM32_IRQ_CAN2RX1          /* Vector 16+65: CAN2 RX1 interrupt */
	HANDLER	stm32_can2sce, STM32_IRQ_CAN2SCE          /* Vector 16+66: CAN2 SCE interrupt */
	HANDLER	stm32_otgfs, STM32_IRQ_OTGFS              /* Vector 16+67: USB On The Go FS global interrupt */

#  else /* CONFIG_STM32_CONNECTIVITYLINE */

	HANDLER	stm32_wwdg, STM32_IRQ_WWDG                /* Vector 16+0:  Window Watchdog interrupt */
	HANDLER	stm32_pvd, STM32_IRQ_PVD                  /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	HANDLER	stm32_tamper, STM32_IRQ_TAMPER            /* Vector 16+2:  Tamper interrupt */
	HANDLER	stm32_rtc, STM32_IRQ_RTC                  /* Vector 16+3:  RTC global interrupt */
	HANDLER	stm32_flash, STM32_IRQ_FLASH              /* Vector 16+4:  Flash global interrupt */
	HANDLER	stm32_rcc, STM32_IRQ_RCC                  /* Vector 16+5:  RCC global interrupt */
	HANDLER	stm32_exti0, STM32_IRQ_EXTI0              /* Vector 16+6:  EXTI Line 0 interrupt */
	HANDLER	stm32_exti1, STM32_IRQ_EXTI1              /* Vector 16+7:  EXTI Line 1 interrupt */
	HANDLER	stm32_exti2, STM32_IRQ_EXTI2              /* Vector 16+8:  EXTI Line 2 interrupt */
	HANDLER	stm32_exti3, STM32_IRQ_EXTI3              /* Vector 16+9:  EXTI Line 3 interrupt */
	HANDLER	stm32_exti4, STM32_IRQ_EXTI4              /* Vector 16+10: EXTI Line 4 interrupt */
	HANDLER	stm32_dma1ch1, STM32_IRQ_DMA1CH1          /* Vector 16+11: DMA1 Channel 1 global interrupt */
	HANDLER	stm32_dma1ch2, STM32_IRQ_DMA1CH2          /* Vector 16+12: DMA1 Channel 2 global interrupt */
	HANDLER	stm32_dma1ch3, STM32_IRQ_DMA1CH3          /* Vector 16+13: DMA1 Channel 3 global interrupt */
	HANDLER	stm32_dma1ch4, STM32_IRQ_DMA1CH4          /* Vector 16+14: DMA1 Channel 4 global interrupt */
	HANDLER	stm32_dma1ch5, STM32_IRQ_DMA1CH5          /* Vector 16+15: DMA1 Channel 5 global interrupt */
	HANDLER	stm32_dma1ch6, STM32_IRQ_DMA1CH6          /* Vector 16+16: DMA1 Channel 6 global interrupt */
	HANDLER	stm32_dma1ch7, STM32_IRQ_DMA1CH7          /* Vector 16+17: DMA1 Channel 7 global interrupt */
	HANDLER	stm32_adc12, STM32_IRQ_ADC12              /* Vector 16+18: ADC1 and ADC2 global interrupt */
	HANDLER	stm32_usbhpcantx, STM32_IRQ_USBHPCANTX    /* Vector 16+19: USB High Priority or CAN TX interrupts*/
	HANDLER	stm32_usblpcanrx0, STM32_IRQ_USBLPCANRX0  /* Vector 16+20: USB Low Priority or CAN RX0 interrupts*/
	HANDLER	stm32_can1rx1, STM32_IRQ_CAN1RX1          /* Vector 16+21: CAN1 RX1 interrupt */
	HANDLER	stm32_can1sce, STM32_IRQ_CAN1SCE          /* Vector 16+22: CAN1 SCE interrupt */
	HANDLER	stm32_exti95, STM32_IRQ_EXTI95            /* Vector 16+23: EXTI Line[9:5] interrupts */
	HANDLER	stm32_tim1brk, STM32_IRQ_TIM1BRK          /* Vector 16+24: TIM1 Break interrupt */
	HANDLER	stm32_tim1up, STM32_IRQ_TIM1UP            /* Vector 16+25: TIM1 Update interrupt */
	HANDLER	stm32_tim1rtgcom, STM32_IRQ_TIM1TRGCOM    /* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	HANDLER	stm32_tim1cc, STM32_IRQ_TIM1CC            /* Vector 16+27: TIM1 Capture Compare interrupt */
	HANDLER	stm32_tim2, STM32_IRQ_TIM2                /* Vector 16+28: TIM2 global interrupt */
	HANDLER	stm32_tim3, STM32_IRQ_TIM3                /* Vector 16+29: TIM3 global interrupt */
	HANDLER	stm32_tim4, STM32_IRQ_TIM4                /* Vector 16+30: TIM4 global interrupt */
	HANDLER	stm32_i2c1ev, STM32_IRQ_I2C1EV            /* Vector 16+31: I2C1 event interrupt */
	HANDLER	stm32_i2c1er, STM32_IRQ_I2C1ER            /* Vector 16+32: I2C1 error interrupt */
	HANDLER	stm32_i2c2ev, STM32_IRQ_I2C2EV            /* Vector 16+33: I2C2 event interrupt */
	HANDLER	stm32_i2c2er, STM32_IRQ_I2C2ER            /* Vector 16+34: I2C2 error interrupt */
	HANDLER	stm32_spi1, STM32_IRQ_SPI1                /* Vector 16+35: SPI1 global interrupt */
	HANDLER	stm32_spi2, STM32_IRQ_SPI2                /* Vector 16+36: SPI2 global interrupt */
	HANDLER	stm32_usart1, STM32_IRQ_USART1            /* Vector 16+37: USART1 global interrupt */
	HANDLER	stm32_usart2, STM32_IRQ_USART2            /* Vector 16+38: USART2 global interrupt */
	HANDLER	stm32_usart3, STM32_IRQ_USART3            /* Vector 16+39: USART3 global interrupt */
	HANDLER	stm32_exti1510, STM32_IRQ_EXTI1510        /* Vector 16+40: EXTI Line[15:10] interrupts */
	HANDLER	stm32_rtcalr, STM32_IRQ_RTCALRM           /* Vector 16+41: RTC alarm through EXTI line interrupt */
	HANDLER	stm32_usbwkup, STM32_IRQ_USBWKUP          /* Vector 16+42: USB wakeup from suspend through EXTI line interrupt*/
	HANDLER	stm32_tim8brk, STM32_IRQ_TIM8BRK          /* Vector 16+43: TIM8 Break interrupt */
	HANDLER	stm32_tim8up, STM32_IRQ_TIM8UP            /* Vector 16+44: TIM8 Update interrupt */
	HANDLER	stm32_tim8trgcom, STM32_IRQ_TIM8TRGCOM    /* Vector 16+45: TIM8 Trigger and Commutation interrupts */
	HANDLER	stm32_tim8cc, STM32_IRQ_TIM8CC            /* Vector 16+46: TIM8 Capture Compare interrupt */
	HANDLER	stm32_adc3, STM32_IRQ_ADC3                /* Vector 16+47: ADC3 global interrupt */
	HANDLER	stm32_fsmc, STM32_IRQ_FSMC                /* Vector 16+48: FSMC global interrupt */
	HANDLER	stm32_sdio, STM32_IRQ_SDIO                /* Vector 16+49: SDIO global interrupt */
	HANDLER	stm32_tim5, STM32_IRQ_TIM5                /* Vector 16+50: TIM5 global interrupt */
	HANDLER	stm32_spi3, STM32_IRQ_SPI3                /* Vector 16+51: SPI3 global interrupt */
	HANDLER	stm32_uart4, STM32_IRQ_UART4              /* Vector 16+52: UART4 global interrupt */
	HANDLER	stm32_uart5, STM32_IRQ_UART5              /* Vector 16+53: UART5 global interrupt */
	HANDLER	stm32_tim6, STM32_IRQ_TIM6                /* Vector 16+54: TIM6 global interrupt */
	HANDLER	stm32_tim7, STM32_IRQ_TIM7                /* Vector 16+55: TIM7 global interrupt */
	HANDLER	stm32_dma2ch1, STM32_IRQ_DMA2CH1          /* Vector 16+56: DMA2 Channel 1 global interrupt */
	HANDLER	stm32_dma2ch2, STM32_IRQ_DMA2CH2          /* Vector 16+57: DMA2 Channel 2 global interrupt */
	HANDLER	stm32_dma2ch3, STM32_IRQ_DMA2CH3          /* Vector 16+58: DMA2 Channel 3 global interrupt */
	HANDLER	stm32_dma2ch45, STM32_IRQ_DMA2CH45        /* Vector 16+59: DMA2 Channel 4&5 global interrupt */

#  endif /* CONFIG_STM32_CONNECTIVITYLINE */

#elif defined(CONFIG_STM32_STM32F20XX)

	HANDLER	stm32_wwdg, STM32_IRQ_WWDG                /* Vector 16+0:  Window Watchdog interrupt */
	HANDLER	stm32_pvd, STM32_IRQ_PVD                  /* Vector 16+1:  PVD through EXTI Line detection interrupt */
	HANDLER	stm32_tamper, STM32_IRQ_TAMPER            /* Vector 16+2:  Tamper and time stamp interrupts */
	HANDLER	stm32_rtc_wkup, STM32_IRQ_RTC_WKUP        /* Vector 16+3:  RTC global interrupt */
	HANDLER	stm32_flash, STM32_IRQ_FLASH              /* Vector 16+4:  Flash global interrupt */
	HANDLER	stm32_rcc, STM32_IRQ_RCC                  /* Vector 16+5:  RCC global interrupt */
	HANDLER	stm32_exti0, STM32_IRQ_EXTI0              /* Vector 16+6:  EXTI Line 0 interrupt */
	HANDLER	stm32_exti1, STM32_IRQ_EXTI1              /* Vector 16+7:  EXTI Line 1 interrupt */
	HANDLER	stm32_exti2, STM32_IRQ_EXTI2              /* Vector 16+8:  EXTI Line 2 interrupt */
	HANDLER	stm32_exti3, STM32_IRQ_EXTI3              /* Vector 16+9:  EXTI Line 3 interrupt */
	HANDLER	stm32_exti4, STM32_IRQ_EXTI4              /* Vector 16+10: EXTI Line 4 interrupt */
	HANDLER	stm32_dma1s0, STM32_IRQ_DMA1S0            /* Vector 16+11: DMA1 Stream 0 global interrupt */
	HANDLER	stm32_dma1s1, STM32_IRQ_DMA1S1            /* Vector 16+12: DMA1 Stream 1 global interrupt */
	HANDLER	stm32_dma1s2, STM32_IRQ_DMA1S2            /* Vector 16+13: DMA1 Stream 2 global interrupt */
	HANDLER	stm32_dma1s3, STM32_IRQ_DMA1S3            /* Vector 16+14: DMA1 Stream 3 global interrupt */
	HANDLER	stm32_dma1s4, STM32_IRQ_DMA1S4            /* Vector 16+15: DMA1 Stream 4 global interrupt */
	HANDLER	stm32_dma1s5, STM32_IRQ_DMA1S5            /* Vector 16+16: DMA1 Stream 5 global interrupt */
	HANDLER	stm32_dma1s6, STM32_IRQ_DMA1S6            /* Vector 16+17: DMA1 Stream 6 global interrupt */
	HANDLER	stm32_adc, STM32_IRQ_ADC                  /* Vector 16+18: ADC1, ADC2, and ADC3 global interrupt */
	HANDLER	stm32_can1tx, STM32_IRQ_CAN1TX            /* Vector 16+19: CAN1 TX interrupts */
	HANDLER	stm32_can1rx0, STM32_IRQ_CAN1RX0          /* Vector 16+20: CAN1 RX0 interrupts */
	HANDLER	stm32_can1rx1, STM32_IRQ_CAN1RX1          /* Vector 16+21: CAN1 RX1 interrupt */
	HANDLER	stm32_can1sce, STM32_IRQ_CAN1SCE          /* Vector 16+22: CAN1 SCE interrupt */
	HANDLER	stm32_exti95, STM32_IRQ_EXTI95            /* Vector 16+23: EXTI Line[9:5] interrupts */
	HANDLER	stm32_tim1brk, STM32_IRQ_TIM1BRK          /* Vector 16+24: TIM1 Break interrupt/TIM9 global interrupt */
	HANDLER	stm32_tim1up, STM32_IRQ_TIM1UP            /* Vector 16+25: TIM1 Update interrupt/TIM10 global interrupt */
	HANDLER	stm32_tim1trgcom, STM32_IRQ_TIM1TRGCOM    /* Vector 16+26: TIM1 Trigger and Commutation interrupts/TIM11 global interrupt */
	HANDLER	stm32_tim1cc, STM32_IRQ_TIM1CC            /* Vector 16+27: TIM1 Capture Compare interrupt */
	HANDLER	stm32_tim2, STM32_IRQ_TIM2                /* Vector 16+28: TIM2 global interrupt */
	HANDLER	stm32_tim3, STM32_IRQ_TIM3                /* Vector 16+29: TIM3 global interrupt */
	HANDLER	stm32_tim4, STM32_IRQ_TIM4                /* Vector 16+30: TIM4 global interrupt */
	HANDLER	stm32_i2c1ev, STM32_IRQ_I2C1EV            /* Vector 16+31: I2C1 event interrupt */
	HANDLER	stm32_i2c1er, STM32_IRQ_I2C1ER            /* Vector 16+32: I2C1 error interrupt */
	HANDLER	stm32_i2c2ev, STM32_IRQ_I2C2EV            /* Vector 16+33: I2C2 event interrupt */
	HANDLER	stm32_i2c2er, STM32_IRQ_I2C2ER            /* Vector 16+34: I2C2 error interrupt */
	HANDLER	stm32_spi1, STM32_IRQ_SPI1                /* Vector 16+35: SPI1 global interrupt */
	HANDLER	stm32_spi2, STM32_IRQ_SPI2                /* Vector 16+36: SPI2 global interrupt */
	HANDLER	stm32_usart1, STM32_IRQ_USART1            /* Vector 16+37: USART1 global interrupt */
	HANDLER	stm32_usart2, STM32_IRQ_USART2            /* Vector 16+38: USART2 global interrupt */
	HANDLER	stm32_usart3, STM32_IRQ_USART3            /* Vector 16+39: USART3 global interrupt */
	HANDLER	stm32_exti1510, STM32_IRQ_EXTI1510        /* Vector 16+40: EXTI Line[15:10] interrupts */
	HANDLER	stm32_rtcalrm, STM32_IRQ_RTCALRM          /* Vector 16+41: RTC alarm through EXTI line interrupt */
	HANDLER	stm32_otgfswkup, STM32_IRQ_OTGFSWKUP      /* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
	HANDLER	stm32_tim8brk, STM32_IRQ_TIM8BRK          /* Vector 16+43: TIM8 Break interrupt/TIM12 global interrupt */
	HANDLER	stm32_tim8up, STM32_IRQ_TIM8UP            /* Vector 16+44: TIM8 Update interrup/TIM13 global interrupt */
	HANDLER	stm32_tim8trgcom, STM32_IRQ_TIM8TRGCOM    /* Vector 16+45: TIM8 Trigger and Commutation interrupts/TIM14 global interrupt */
	HANDLER	stm32_tim8cc, STM32_IRQ_TIM8CC            /* Vector 16+46: TIM8 Capture Compare interrupt */
	HANDLER	stm32_dma1s7, STM32_IRQ_DMA1S7            /* Vector 16+47: DMA1 Stream 7 global interrupt */
	HANDLER	stm32_fsmc, STM32_IRQ_FSMC                /* Vector 16+48: FSMC global interrupt */
	HANDLER	stm32_sdio, STM32_IRQ_SDIO                /* Vector 16+49: SDIO global interrupt */
	HANDLER	stm32_tim5, STM32_IRQ_TIM5                /* Vector 16+50: TIM5 global interrupt */
	HANDLER	stm32_spi3, STM32_IRQ_SPI3                /* Vector 16+51: SPI3 global interrupt */
	HANDLER	stm32_uart4, STM32_IRQ_UART4              /* Vector 16+52: UART4 global interrupt */
	HANDLER	stm32_uart5, STM32_IRQ_UART5              /* Vector 16+53: UART5 global interrupt */
	HANDLER	stm32_tim6, STM32_IRQ_TIM6                /* Vector 16+54: TIM6 global interrupt/DAC1 and DAC2 underrun error interrupts */
	HANDLER	stm32_tim7, STM32_IRQ_TIM7                /* Vector 16+55: TIM7 global interrupt */
	HANDLER	stm32_dma2s0, STM32_IRQ_DMA2S0            /* Vector 16+56: DMA2 Stream 0 global interrupt */
	HANDLER	stm32_dma2s1, STM32_IRQ_DMA2S1            /* Vector 16+57: DMA2 Stream 1 global interrupt */
	HANDLER	stm32_dma2s2, STM32_IRQ_DMA2S2            /* Vector 16+58: DMA2 Stream 2 global interrupt */
	HANDLER	stm32_dma2s3, STM32_IRQ_DMA2S3            /* Vector 16+59: DMA2 Stream 3 global interrupt */
	HANDLER	stm32_dma2s4, STM32_IRQ_DMA2S4            /* Vector 16+60: DMA2 Stream 4 global interrupt */
	HANDLER	stm32_eth, STM32_IRQ_ETH                  /* Vector 16+61: Ethernet global interrupt */
	HANDLER	stm32_ethwkup, STM32_IRQ_ETHWKUP          /* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
	HANDLER	stm32_can2tx, STM32_IRQ_CAN2TX            /* Vector 16+63: CAN2 TX interrupts */
	HANDLER	stm32_can2rx0, STM32_IRQ_CAN2RX0          /* Vector 16+64: CAN2 RX0 interrupts */
	HANDLER	stm32_can2rx1, STM32_IRQ_CAN2RX1          /* Vector 16+65: CAN2 RX1 interrupt */
	HANDLER	stm32_can2sce, STM32_IRQ_CAN2SCE          /* Vector 16+66: CAN2 SCE interrupt */
	HANDLER	stm32_otgfs, STM32_IRQ_OTGFS              /* Vector 16+67: USB On The Go FS global interrupt */
	HANDLER	stm32_dma2s5, STM32_IRQ_DMA2S5            /* Vector 16+68: DMA2 Stream 5 global interrupt */
	HANDLER	stm32_dma2s6, STM32_IRQ_DMA2S6            /* Vector 16+69: DMA2 Stream 6 global interrupt */
	HANDLER	stm32_dma2s7, STM32_IRQ_DMA2S7            /* Vector 16+70: DMA2 Stream 7 global interrupt */
	HANDLER	stm32_usart6, STM32_IRQ_USART6            /* Vector 16+71: USART6 global interrupt */
	HANDLER	stm32_i2c3ev, STM32_IRQ_I2C3EV            /* Vector 16+72: I2C3 event interrupt */
	HANDLER	stm32_i2c3er, STM32_IRQ_I2C3ER            /* Vector 16+73: I2C3 error interrupt */
	HANDLER	stm32_otghsep1out, STM32_IRQ_OTGHSEP1OUT  /* Vector 16+74: USB On The Go HS End Point 1 Out global interrupt */
	HANDLER	stm32_otghsep1in, STM32_IRQ_OTGHSEP1IN    /* Vector 16+75: USB On The Go HS End Point 1 In global interrupt */
	HANDLER	stm32_otghswkup, STM32_IRQ_OTGHSWKUP      /* Vector 16+76: USB On The Go HS Wakeup through EXTI interrupt */
	HANDLER	stm32_otghs, STM32_IRQ_OTGHS	            /* Vector 16+77: USB On The Go HS global interrupt */
	HANDLER	stm32_dcmi, STM32_IRQ_DCMI                /* Vector 16+78: DCMI global interrupt */
	HANDLER	stm32_cryp, STM32_IRQ_CRYP                /* Vector 16+79: CRYP crypto global interrupt */
	HANDLER	stm32_hash, STM32_IRQ_HASH                /* Vector 16+80: Hash and Rng global interrupt */

#elif defined(CONFIG_STM32_STM32F30XX)
#  include "chip/stm32f30xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F33XX)
#  include "chip/stm32f33xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F37XX)
#  include "chip/stm32f37xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#  include "chip/stm32f42xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F446)
#  include "chip/stm32f44xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "chip/stm32f40xxx_vectors.h"
#else
#  error "No handlers for STM32 chip"
#endif

#endif /* CONFIG_STM32_NOEXT_VECTORS */

/* Common IRQ handling logic.  On entry here, the return stack is on either
 * the PSP or the MSP and looks like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * MSP->REG_R0
 *
 * And
 *      R0 contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

	PUBLIC exception_common

exception_common:

	/* Complete the context save */

#ifdef CONFIG_BUILD_PROTECTED
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the stack is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	beq		l1						/* Branch if context already on the MSP */
	mrs		r1, psp					/* R1=The process stack pointer (PSP) */
	mov     sp, r1					/* Set the MSP to the PSP */

l1:
#endif

	/* r1 holds the value of the stack pointer AFTER the exception handling logic
	 * pushed the various registers onto the stack.  Get r2 = the value of the
	 * stack pointer BEFORE the interrupt modified it.
	 */

	mov		r2, sp					/* R2=Copy of the main/process stack pointer */
	add		r2, r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */
#ifdef CONFIG_ARMV7M_USEBASEPRI
	mrs		r3, basepri				/* R3=Current BASEPRI setting */
#else
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#endif

#ifdef CONFIG_ARCH_FPU
	/* Skip over the block of memory reserved for floating pointer register save.
	 * Lazy FPU register saving is used.  FPU registers will be saved in this
	 * block only if a context switch occurs (this means, of course, that the FPU
	 * cannot be used in interrupt processing).
	 */

	sub		sp, #(4*SW_FPU_REGS)
#endif

	/* Save the remaining registers on the stack after the registers pushed
	 * by the exception handling logic. r2=SP and r3=primask or basepri, r4-r11,
	 * r14=register values.
	 */

#ifdef CONFIG_BUILD_PROTECTED
	stmdb	sp!, {r2-r11,r14}		/* Save the remaining registers plus the SP value */
#else
	stmdb	sp!, {r2-r11}			/* Save the remaining registers plus the SP value */
#endif

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

#else
	/* Set the BASEPRI register so that further normal interrupts will be
	 * masked.  Nested, high priority may still occur, however.
	 */

	mov		r2, #NVIC_SYSH_DISABLE_PRIORITY
	msr		basepri, r2				/* Set the BASEPRI */
#endif

	/* There are two arguments to up_doirq:
	 *
	 *   R0 = The IRQ number
	 *   R1 = The top of the stack points to the saved state
	 */

	mov		r1, sp

	/* Also save the top of the stack in a preserved register */

	mov		r4, sp

#if CONFIG_ARCH_INTERRUPTSTACK > 7
	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will set the MSP to use
	 * a special special interrupt stack pointer.  The way that this is done
	 * here prohibits nested interrupts without some additional logic!
	 */

	ldr		sp, =g_intstackbase

#else
	/* Otherwise, we will re-use the interrupted thread's stack.  That may
	 * mean using either MSP or PSP stack for interrupt level processing (in
	 * kernel mode).
	 */

	bic		r2, r4, #7				/* Get the stack pointer with 8-byte alignment */
	mov		sp, r2					/* Instantiate the aligned stack */

#endif

	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, r4					/* Recover R1=main stack pointer */

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		l2						/* Branch if no context switch */

	/* We are returning with a pending context switch.
	 *
	 * If the FPU is enabled, then we will need to restore FPU registers.
	 * This is not done in normal interrupt save/restore because the cost
	 * is prohibitive.  This is only done when switching contexts.  A
	 * consequence of this is that floating point operations may not be
	 * performed in interrupt handling logic.
	 *
	 * Here:
	 *   r0 = Address of the register save area
	 *
	 * NOTE: It is a requirement that up_restorefpu() preserve the value of
	 * r0!
	 */

#ifdef CONFIG_ARCH_FPU
	bl		up_restorefpu			/* Restore the FPU registers */
#endif

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie in the
	 * stack but, rather, within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	add		r1, r0, #SW_XCPT_SIZE	/* R1=Address of HW save area in reg array */
	ldmia	r1, {r4-r11}			/* Fetch eight registers in HW save area */
	ldr		r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
	stmdb	r1!, {r4-r11}			/* Store eight registers in HW save area */
#ifdef CONFIG_BUILD_PROTECTED
	ldmia	r0, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r0, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif
	b		l3						/* Re-join common logic */

	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created
	 *
	 * Here:
	 *   r1 = Address of the return stack (same as r0)
	 */

l2:
#ifdef CONFIG_BUILD_PROTECTED
	ldmia	r1!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r1!, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif

#ifdef CONFIG_ARCH_FPU
	/* Skip over the block of memory reserved for floating pointer register
	 * save. Then R1 is the address of the HW save area
	 */

	add		r1, #(4*SW_FPU_REGS)
#endif

	/* Set up to return from the exception
	 *
	 * Here:
	 *   r1 = Address on the target thread's stack position at the start of
	 *        the registers saved by hardware
	 *   r3 = primask or basepri
	 *   r4-r11 = restored register values
	 */

l3:

#ifdef CONFIG_BUILD_PROTECTED
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the stack is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	mrs		r2, control				/* R2=Contents of the control register */
	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	beq		l4						/* Branch if privileged */

	orr		r2, r2, #1				/* Unprivileged mode */
	msr		psp, r1					/* R1=The process stack pointer */
	b		l5
l4:
	bic		r2, r2, #1				/* Privileged mode */
	msr		msp, r1					/* R1=The main stack pointer */
l5:
	msr		control, r2				/* Save the updated control register */
#else
	msr		msp, r1					/* Recover the return MSP value */

	/* Preload r14 with the special return value first (so that the return
	 * actually occurs with interrupts still disabled).
	 */

	ldr		r14, =EXC_RETURN_PRIVTHR	/* Load the special value */
#endif

	/* Restore the interrupt state */

#ifdef CONFIG_ARMV7M_USEBASEPRI
	msr		basepri, r3				/* Restore interrupts priority masking */
#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
	cpsie	i						/* Re-enable interrupts */
#endif

#else
	msr		primask, r3				/* Restore interrupts */
#endif

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) continue to use the MSP
	 */

	bx		r14						/* And return */

/************************************************************************************
 *  Name: g_intstackalloc/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 7
	.bss
	.global	g_intstackalloc
	.global	g_intstackbase
	.align	8
g_intstackalloc:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK + 4) & ~7)
g_intstackbase:
	.size	g_intstackalloc, .-g_intstackalloc
#endif

#endif /* CONFIG_ARMV7M_CMNVECTOR */

/************************************************************************************
 * .rodata
 ************************************************************************************/

	SECTION .rodata:CONST:NOROOT(2)

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_idle_topstack below.
 */

	PUBLIC g_idle_topstack

g_idle_topstack:
	DCD	HEAP_BASE

	END
