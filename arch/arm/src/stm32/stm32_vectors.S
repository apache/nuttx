/************************************************************************************
 * arch/arm/src/stm32/stm32_vectors.S
 * arch/arm/src/chip/stm32_vectors.S
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

/* Memory Map:
 *
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

#define IDLE_STACK      (_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE       (_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)

/* The Cortex-M3 return from interrupt is unusual.  We provide the following special
 * address to the BX instruction.  The particular value also forces a return to
 * thread mode and covers state from the main stack point, the MSP (vs. the MSP).
 */

#define EXC_RETURN      0xfffffff9

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		__start

	.syntax		unified
	.thumb
	.file		"stm32_vectors.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

	.macro	HANDLER, label, irqno
	.thumb_func
\label:
	mov	r0, #\irqno
	b	stm32_common
	.endm

/************************************************************************************
 * Vectors
 ************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		2
	.globl		stm32_vectors
	.type		stm32_vectors, function

stm32_vectors:

/* Processor Exceptions */

	.word	IDLE_STACK			/* Vector  0: Reset stack pointer */
	.word	__start				/* Vector  1: Reset vector */
	.word	stm32_nmi			/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	stm32_hardfault		/* Vector  3: Hard fault */
	.word	stm32_mpu			/* Vector  4: Memory management (MPU) */
	.word	stm32_busfault		/* Vector  5: Bus fault */
	.word	stm32_usagefault	/* Vector  6: Usage fault */
	.word	stm32_reserved		/* Vector  7: Reserved */
	.word	stm32_reserved		/* Vector  8: Reserved */
	.word	stm32_reserved		/* Vector  9: Reserved */
	.word	stm32_reserved		/* Vector 10: Reserved */
	.word	stm32_svcall		/* Vector 11: SVC call */
	.word	stm32_dbgmonitor	/* Vector 12: Debug monitor */
	.word	stm32_reserved		/* Vector 13: Reserved */
	.word	stm32_pendsv		/* Vector 14: Pendable system service request */
	.word	stm32_systick		/* Vector 15: System tick */

/* External Interrupts */

#ifdef CONFIG_STM32_CONNECTIVITY_LINE
	.word	stm32_wwdg 			/* Vector 16+0:  Window Watchdog interrupt */
	.word	stm32_pvd			/* Vector 16+1:  PVD through EXTI Line detection interrupt */
	.word	stm32_tamper		/* Vector 16+2:  Tamper interrupt */
	.word	stm32_rtc			/* Vector 16+3:  RTC global interrupt */
	.word	stm32_flash			/* Vector 16+4:  Flash global interrupt */
	.word	stm32_rcc			/* Vector 16+5:  RCC global interrupt */
	.word	stm32_exti0			/* Vector 16+6:  EXTI Line 0 interrupt */
	.word	stm32_exti1			/* Vector 16+7:  EXTI Line 1 interrupt */
	.word	stm32_exti2			/* Vector 16+8:  EXTI Line 2 interrupt */
	.word	stm32_exti3			/* Vector 16+9:  EXTI Line 3 interrupt */
	.word	stm32_exti4			/* Vector 16+10: EXTI Line 4 interrupt */
	.word	stm32_dma1ch1		/* Vector 16+11: DMA1 Channel 1 global interrupt */
	.word	stm32_dma1ch2		/* Vector 16+12: DMA1 Channel 2 global interrupt */
	.word	stm32_dma1ch3		/* Vector 16+13: DMA1 Channel 3 global interrupt */
	.word	stm32_dma1ch4		/* Vector 16+14: DMA1 Channel 4 global interrupt */
	.word	stm32_dma1ch5		/* Vector 16+15: DMA1 Channel 5 global interrupt */
	.word	stm32_dma1ch6		/* Vector 16+16: DMA1 Channel 7 global interrupt */
	.word	stm32_adc12			/* Vector 16+18: ADC1 and ADC2 global interrupt */
	.word	stm32_can1tx		/* Vector 16+19: CAN1 TX interrupts */
	.word	stm32_can1rx0		/* Vector 16+20: CAN1 RX0 interrupts */
	.word	stm32_can1rx		/* Vector 16+21: CAN1 RX1 interrupt */
	.word	stm32_can1sce		/* Vector 16+22: CAN1 SCE interrupt */
	.word	stm32_exti95		/* Vector 16+23: EXTI Line[9:5] interrupts */
	.word	stm32_tim1brk		/* Vector 16+24: TIM1 Break interrupt */
	.word	stm32_tim1up		/* Vector 16+25: TIM1 Update interrupt */
	.word	stm32_tim1trgcom	/* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	.word	stm32_tim1cc		/* Vector 16+27: TIM1 Capture Compare interrupt */
	.word	stm32_tim2			/* Vector 16+28: TIM2 global interrupt */
	.word	stm32_tim3			/* Vector 16+29: TIM3 global interrupt */
	.word	stm32_tim4			/* Vector 16+30: TIM4 global interrupt */
	.word	stm32_i2c1ev		/* Vector 16+31: I2C1 event interrupt */
	.word	stm32_i2c1er		/* Vector 16+32: I2C1 error interrupt */
	.word	stm32_i2c2ev		/* Vector 16+33: I2C2 event interrupt */
	.word	stm32_i2c2er		/* Vector 16+34: I2C2 error interrupt */
	.word	stm32_spi1			/* Vector 16+35: SPI1 global interrupt */
	.word	stm32_spi2			/* Vector 16+36: SPI2 global interrupt */
	.word	stm32_usart1		/* Vector 16+37: USART1 global interrupt */
	.word	stm32_usart2		/* Vector 16+38: USART2 global interrupt */
	.word	stm32_usart3		/* Vector 16+39: USART3 global interrupt */
	.word	stm32_exti1510		/* Vector 16+40: EXTI Line[15:10] interrupts */
	.word	stm32_rtcalr		/* Vector 16+41: RTC alarm through EXTI line interrupt */
	.word	stm32_otgfswkup		/* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
	.word	stm32_reserved		/* Vector 16+43: Reserved */
	.word	stm32_reserved		/* Vector 16+44: Reserved */
	.word	stm32_reserved		/* Vector 16+45: Reserved */
	.word	stm32_reserved		/* Vector 16+46: Reserved */
	.word	stm32_reserved		/* Vector 16+47: Reserved */
	.word	stm32_reserved		/* Vector 16+48: Reserved */
	.word	stm32_reserved		/* Vector 16+49: Reserved */
	.word	stm32_tim5			/* Vector 16+50: TIM5 global interrupt */
	.word	stm32_spi3			/* Vector 16+51: SPI3 global interrupt */
	.word	stm32_uart4 		/* Vector 16+52: UART4 global interrupt */
	.word	stm32_uart5			/* Vector 16+53: UART5 global interrupt */
	.word	stm32_tim6			/* Vector 16+54: TIM6 global interrupt */
	.word	stm32_tim7			/* Vector 16+55: TIM7 global interrupt */
	.word	stm32_dma2ch1		/* Vector 16+56: DMA2 Channel 1 global interrupt */
	.word	stm32_dma2ch2		/* Vector 16+57: DMA2 Channel 2 global interrupt */
	.word	stm32_dma2ch3		/* Vector 16+58: DMA2 Channel 3 global interrupt */
	.word	stm32_dma2ch4		/* Vector 16+59: DMA2 Channel 4 global interrupt */
	.word	stm32_dma2ch5		/* Vector 16+60: DMA2 Channel 5 global interrupt */
	.word	stm32_eth			/* Vector 16+61: Ethernet global interrupt */
	.word	stm32_ethwkup		/* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
	.word	stm32_can2tx		/* Vector 16+63: CAN2 TX interrupts */
	.word	stm32_can2rx0		/* Vector 16+64: CAN2 RX0 interrupts */
	.word	stm32_can2rx1		/* Vector 16+65: CAN2 RX1 interrupt */
	.word	stm32_can2sce		/* Vector 16+66: CAN2 SCE interrupt */
	.word	stm32_otgfs			/* Vector 16+67: USB On The Go FS global interrupt */
#else
	.word	stm32_wwdg			/* Vector 16+0:  Window Watchdog interrupt */
	.word	stm32_pvd			/* Vector 16+1:  PVD through EXTI Line detection interrupt */
	.word	stm32_tamper		/* Vector 16+2:  Tamper interrupt */
	.word	stm32_rtc			/* Vector 16+3:  RTC global interrupt */
	.word	stm32_flash			/* Vector 16+4:  Flash global interrupt */
	.word	stm32_rcc			/* Vector 16+5:  RCC global interrupt */
	.word	stm32_exti0			/* Vector 16+6:  EXTI Line 0 interrupt */
	.word	stm32_exti1			/* Vector 16+7:  EXTI Line 1 interrupt */
	.word	stm32_exti2			/* Vector 16+8:  EXTI Line 2 interrupt */
	.word	stm32_exti3			/* Vector 16+9:  EXTI Line 3 interrupt */
	.word	stm32_exti4			/* Vector 16+10: EXTI Line 4 interrupt */
	.word	stm32_dma1ch1		/* Vector 16+11: DMA1 Channel 1 global interrupt */
	.word	stm32_dma1ch2		/* Vector 16+12: DMA1 Channel 2 global interrupt */
	.word	stm32_dma1ch3		/* Vector 16+13: DMA1 Channel 3 global interrupt */
	.word	stm32_dma1ch4		/* Vector 16+14: DMA1 Channel 4 global interrupt */
	.word	stm32_dma1ch5		/* Vector 16+15: DMA1 Channel 5 global interrupt */
	.word	stm32_dma1ch6		/* Vector 16+16: DMA1 Channel 6 global interrupt */
	.word	stm32_dma1ch7		/* Vector 16+17: DMA1 Channel 7 global interrupt */
	.word	stm32_adc12			/* Vector 16+18: ADC1 and ADC2 global interrupt */
	.word	stm32_usbhpcantx	/* Vector 16+19: USB High Priority or CAN TX interrupts*/
	.word	stm32_usblpcanrx0	/* Vector 16+20: USB Low Priority or CAN RX0 interrupts*/
	.word	stm32_can1rx1		/* Vector 16+21: CAN1 RX1 interrupt */
	.word	stm32_can1sce		/* Vector 16+22: CAN1 SCE interrupt */
	.word	stm32_exti95		/* Vector 16+23: EXTI Line[9:5] interrupts */
	.word	stm32_tim1brk		/* Vector 16+24: TIM1 Break interrupt */
	.word	stm32_tim1up		/* Vector 16+25: TIM1 Update interrupt */
	.word	stm32_tim1rtgcom	/* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	.word	stm32_tim1cc		/* Vector 16+27: TIM1 Capture Compare interrupt */
	.word	stm32_tim2			/* Vector 16+28: TIM2 global interrupt */
	.word	stm32_tim3			/* Vector 16+29: TIM3 global interrupt */
	.word	stm32_tim4			/* Vector 16+30: TIM4 global interrupt */
	.word	stm32_i2c1ev		/* Vector 16+31: I2C1 event interrupt */
	.word	stm32_i2c1er		/* Vector 16+32: I2C1 error interrupt */
	.word	stm32_i2c2ev		/* Vector 16+33: I2C2 event interrupt */
	.word	stm32_i2c2er		/* Vector 16+34: I2C2 error interrupt */
	.word	stm32_spi1			/* Vector 16+35: SPI1 global interrupt */
	.word	stm32_spi2			/* Vector 16+36: SPI2 global interrupt */
	.word	stm32_usart1		/* Vector 16+37: USART1 global interrupt */
	.word	stm32_usart2		/* Vector 16+38: USART2 global interrupt */
	.word	stm32_usart3		/* Vector 16+39: USART3 global interrupt */
	.word	stm32_exti1510		/* Vector 16+40: EXTI Line[15:10] interrupts */
	.word	stm32_rtcalr		/* Vector 16+41: RTC alarm through EXTI line interrupt */
	.word	stm32_usbwkup		/* Vector 16+42: USB wakeup from suspend through EXTI line interrupt*/
	.word	stm32_tim8brk		/* Vector 16+43: TIM8 Break interrupt */
	.word	stm32_tim8up		/* Vector 16+44: TIM8 Update interrupt */
	.word	stm32_tim8trgcom	/* Vector 16+45: TIM8 Trigger and Commutation interrupts */
	.word	stm32_tim8cc		/* Vector 16+46: TIM8 Capture Compare interrupt */
	.word	stm32_adc3			/* Vector 16+47: ADC3 global interrupt */
	.word	stm32_fsmc			/* Vector 16+48: FSMC global interrupt */
	.word	stm32_sdio			/* Vector 16+49: SDIO global interrupt */
	.word	stm32_tim5			/* Vector 16+50: TIM5 global interrupt */
	.word	stm32_spi3			/* Vector 16+51: SPI3 global interrupt */
	.word	stm32_uart4			/* Vector 16+52: UART4 global interrupt */
	.word	stm32_uart5			/* Vector 16+53: UART5 global interrupt */
	.word	stm32_tim6			/* Vector 16+54: TIM6 global interrupt */
	.word	stm32_tim7			/* Vector 16+55: TIM7 global interrupt */
	.word	stm32_dma2ch1		/* Vector 16+56: DMA2 Channel 1 global interrupt */
	.word	stm32_dma2ch2		/* Vector 16+57: DMA2 Channel 2 global interrupt */
	.word	stm32_dma2ch3		/* Vector 16+58: DMA2 Channel 3 global interrupt */
	.word	stm32_dma2ch45		/* Vector 16+59: DMA2 Channel 4&5 global interrupt */
#endif
	.size	stm32_vectors, .-stm32_vectors

/************************************************************************************
 * .text
 ************************************************************************************/

	.text
        .type   handlers, function
        .thumb_func
handlers:
	HANDLER	stm32_reserved, STM32_IRQ_RESERVED		/* Unexpected/reserved vector */
	HANDLER stm32_nmi, STM32_IRQ_NMI				/* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER stm32_hardfault, STM32_IRQ_HARDFAULT	/* Vector  3: Hard fault */
	HANDLER stm32_mpu, STM32_IRQ_MPU				/* Vector  4: Memory management (MPU) */
	HANDLER stm32_busfault, STM32_IRQ_BUSFAULT		/* Vector  5: Bus fault */
	HANDLER stm32_usagefault, STM32_IRQ_USAGEFAULT	/* Vector  6: Usage fault */
	HANDLER stm32_svcall, STM32_IRQ_SVCALL			/* Vector 11: SVC call */
	HANDLER stm32_dbgmonitor, STM32_IRQ_DBGMONITOR	/* Vector 12: Debug Monitor */
	HANDLER stm32_pendsv, STM32_IRQ_PENDSV			/* Vector 14: Penable system service request */
	HANDLER stm32_systick, STM32_IRQ_SYSTICK		/* Vector 15: System tick */

#ifdef CONFIG_STM32_CONNECTIVITY_LINE
	HANDLER	stm32_wwdg, STM32_IRQ_WWDG 				/* Vector 16+0:  Window Watchdog interrupt */
	HANDLER	stm32_pvd, STM32_IRQ_PVD 				/* Vector 16+1:  PVD through EXTI Line detection interrupt */
	HANDLER	stm32_tamper, STM32_IRQ_TAMPER			/* Vector 16+2:  Tamper interrupt */
	HANDLER	stm32_rtc, STM32_IRQ_RTC 				/* Vector 16+3:  RTC global interrupt */
	HANDLER	stm32_flash, STM32_IRQ_FLASH 			/* Vector 16+4:  Flash global interrupt */
	HANDLER	stm32_rcc, STM32_IRQ_RCC				/* Vector 16+5:  RCC global interrupt */
	HANDLER	stm32_exti0, STM32_IRQ_EXTI0 			/* Vector 16+6:  EXTI Line 0 interrupt */
	HANDLER	stm32_exti1, STM32_IRQ_EXTI1			/* Vector 16+7:  EXTI Line 1 interrupt */
	HANDLER	stm32_exti2, STM32_IRQ_EXTI2			/* Vector 16+8:  EXTI Line 2 interrupt */
	HANDLER	stm32_exti3, STM32_IRQ_EXTI3			/* Vector 16+9:  EXTI Line 3 interrupt */
	HANDLER	stm32_exti4, STM32_IRQ_EXTI4			/* Vector 16+10: EXTI Line 4 interrupt */
	HANDLER	stm32_dma1ch1, STM32_IRQ_DMA1CH1		/* Vector 16+11: DMA1 Channel 1 global interrupt */
	HANDLER	stm32_dma1ch2, STM32_IRQ_DMA1CH2		/* Vector 16+12: DMA1 Channel 2 global interrupt */
	HANDLER	stm32_dma1ch3, STM32_IRQ_DMA1CH3 		/* Vector 16+13: DMA1 Channel 3 global interrupt */
	HANDLER	stm32_dma1ch4, STM32_IRQ_DMA1CH4 		/* Vector 16+14: DMA1 Channel 4 global interrupt */
	HANDLER	stm32_dma1ch5, STM32_IRQ_DMA1CH5		/* Vector 16+15: DMA1 Channel 5 global interrupt */
	HANDLER	stm32_dma1ch6, STM32_IRQ_DMA1CH6 		/* Vector 16+16: DMA1 Channel 7 global interrupt */
	HANDLER	stm32_adc12, STM32_IRQ_ADC12 			/* Vector 16+18: ADC1 and ADC2 global interrupt */
	HANDLER	stm32_can1tx, STM32_IRQ_CAN1TX			/* Vector 16+19: CAN1 TX interrupts */
	HANDLER	stm32_can1rx0, STM32_IRQ_CAN1RX0		/* Vector 16+20: CAN1 RX0 interrupts */
	HANDLER	stm32_can1rx, STM32_IRQ_CAN1RX1			/* Vector 16+21: CAN1 RX1 interrupt */
	HANDLER	stm32_can1sce, STM32_IRQ_CAN1SCE		/* Vector 16+22: CAN1 SCE interrupt */
	HANDLER	stm32_exti95, STM32_IRQ_EXTI95			/* Vector 16+23: EXTI Line[9:5] interrupts */
	HANDLER	stm32_tim1brk, STM32_IRQ_TIM1BRK		/* Vector 16+24: TIM1 Break interrupt */
	HANDLER	stm32_tim1up, STM32_IRQ_TIM1UP			/* Vector 16+25: TIM1 Update interrupt */
	HANDLER	stm32_tim1trgcom, STM32_IRQ_TIM1TRGCOM	/* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	HANDLER	stm32_tim1cc, STM32_IRQ_TIM1CC			/* Vector 16+27: TIM1 Capture Compare interrupt */
	HANDLER	stm32_tim2, STM32_IRQ_TIM2				/* Vector 16+28: TIM2 global interrupt */
	HANDLER	stm32_tim3, STM32_IRQ_TIM3 				/* Vector 16+29: TIM3 global interrupt */
	HANDLER	stm32_tim4, STM32_IRQ_TIM4				/* Vector 16+30: TIM4 global interrupt */
	HANDLER	stm32_i2c1ev, STM32_IRQ_I2C1EV			/* Vector 16+31: I2C1 event interrupt */
	HANDLER	stm32_i2c1er, STM32_IRQ_I2C1ER			/* Vector 16+32: I2C1 error interrupt */
	HANDLER	stm32_i2c2ev, STM32_IRQ_I2C2EV 			/* Vector 16+33: I2C2 event interrupt */
	HANDLER	stm32_i2c2er, STM32_IRQ_I2C2ER			/* Vector 16+34: I2C2 error interrupt */
	HANDLER	stm32_spi1, STM32_IRQ_SPI1 				/* Vector 16+35: SPI1 global interrupt */
	HANDLER	stm32_spi2, STM32_IRQ_SPI2 				/* Vector 16+36: SPI2 global interrupt */
	HANDLER	stm32_usart1, STM32_IRQ_USART1			/* Vector 16+37: USART1 global interrupt */
	HANDLER	stm32_usart2, STM32_IRQ_USART2			/* Vector 16+38: USART2 global interrupt */
	HANDLER	stm32_usart3, STM32_IRQ_USART3			/* Vector 16+39: USART3 global interrupt */
	HANDLER	stm32_exti1510, STM32_IRQ_EXTI1510 		/* Vector 16+40: EXTI Line[15:10] interrupts */
	HANDLER	stm32_rtcalr, STM32_IRQ_RTCALR			/* Vector 16+41: RTC alarm through EXTI line interrupt */
	HANDLER	stm32_otgfswkup, STM32_IRQ_OTGFSWKUP	/* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
	HANDLER	stm32_tim5, STM32_IRQ_TIM5				/* Vector 16+50: TIM5 global interrupt */
	HANDLER	stm32_spi3, STM32_IRQ_SPI3  			/* Vector 16+51: SPI3 global interrupt */
	HANDLER	stm32_uart4 , STM32_IRQ_UART4			/* Vector 16+52: UART4 global interrupt */
	HANDLER	stm32_uart5, STM32_IRQ_UART5 			/* Vector 16+53: UART5 global interrupt */
	HANDLER	stm32_tim6, STM32_IRQ_TIM6 				/* Vector 16+54: TIM6 global interrupt */
	HANDLER	stm32_tim7, STM32_IRQ_TIM7 				/* Vector 16+55: TIM7 global interrupt */
	HANDLER	stm32_dma2ch1, STM32_IRQ_DMA2CH1 		/* Vector 16+56: DMA2 Channel 1 global interrupt */
	HANDLER	stm32_dma2ch2, STM32_IRQ_DMA2CH2 		/* Vector 16+57: DMA2 Channel 2 global interrupt */
	HANDLER	stm32_dma2ch3, STM32_IRQ_DMA2CH3 		/* Vector 16+58: DMA2 Channel 3 global interrupt */
	HANDLER	stm32_dma2ch4, STM32_IRQ_DMA2CH4		/* Vector 16+59: DMA2 Channel 4 global interrupt */
	HANDLER	stm32_dma2ch5, STM32_IRQ_DMA2CH5		/* Vector 16+60: DMA2 Channel 5 global interrupt */
	HANDLER	stm32_eth, STM32_IRQ_ETH				/* Vector 16+61: Ethernet global interrupt */
	HANDLER	stm32_ethwkup, STM32_IRQ_ETHWKUP		/* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
	HANDLER	stm32_can2tx, STM32_IRQ_CAN2TX			/* Vector 16+63: CAN2 TX interrupts */
	HANDLER	stm32_can2rx0, STM32_IRQ_CAN2RX0		/* Vector 16+64: CAN2 RX0 interrupts */
	HANDLER	stm32_can2rx1, STM32_IRQ_CAN2RX1		/* Vector 16+65: CAN2 RX1 interrupt */
	HANDLER	stm32_can2sce, STM32_IRQ_CAN2SCE		/* Vector 16+66: CAN2 SCE interrupt */
	HANDLER	stm32_otgfs, STM32_IRQ_OTGFS			/* Vector 16+67: USB On The Go FS global interrupt */
#else
	HANDLER	stm32_wwdg, STM32_IRQ_WWDG				/* Vector 16+0:  Window Watchdog interrupt */
	HANDLER	stm32_pvd, STM32_IRQ_PVD 				/* Vector 16+1:  PVD through EXTI Line detection interrupt */
	HANDLER	stm32_tamper, STM32_IRQ_TAMPER 			/* Vector 16+2:  Tamper interrupt */
	HANDLER	stm32_rtc, STM32_IRQ_RTC 				/* Vector 16+3:  RTC global interrupt */
	HANDLER	stm32_flash, STM32_IRQ_FLASH 			/* Vector 16+4:  Flash global interrupt */
	HANDLER	stm32_rcc, STM32_IRQ_RCC 				/* Vector 16+5:  RCC global interrupt */
	HANDLER	stm32_exti0, STM32_IRQ_EXTI0 			/* Vector 16+6:  EXTI Line 0 interrupt */
	HANDLER	stm32_exti1, STM32_IRQ_EXTI1			/* Vector 16+7:  EXTI Line 1 interrupt */
	HANDLER	stm32_exti2, STM32_IRQ_EXTI2			/* Vector 16+8:  EXTI Line 2 interrupt */
	HANDLER	stm32_exti3, STM32_IRQ_EXTI3 			/* Vector 16+9:  EXTI Line 3 interrupt */
	HANDLER	stm32_exti4, STM32_IRQ_EXTI4 			/* Vector 16+10: EXTI Line 4 interrupt */
	HANDLER	stm32_dma1ch1, STM32_IRQ_DMA1CH1 		/* Vector 16+11: DMA1 Channel 1 global interrupt */
	HANDLER	stm32_dma1ch2, STM32_IRQ_DMA1CH2		/* Vector 16+12: DMA1 Channel 2 global interrupt */
	HANDLER	stm32_dma1ch3, STM32_IRQ_DMA1CH3		/* Vector 16+13: DMA1 Channel 3 global interrupt */
	HANDLER	stm32_dma1ch4, STM32_IRQ_DMA1CH4 		/* Vector 16+14: DMA1 Channel 4 global interrupt */
	HANDLER	stm32_dma1ch5, STM32_IRQ_DMA1CH5 		/* Vector 16+15: DMA1 Channel 5 global interrupt */
	HANDLER	stm32_dma1ch6, STM32_IRQ_DMA1CH6 		/* Vector 16+16: DMA1 Channel 6 global interrupt */
	HANDLER	stm32_dma1ch7, STM32_IRQ_DMA1CH7 		/* Vector 16+17: DMA1 Channel 7 global interrupt */
	HANDLER	stm32_adc12, STM32_IRQ_ADC12 			/* Vector 16+18: ADC1 and ADC2 global interrupt */
	HANDLER	stm32_usbhpcantx, STM32_IRQ_USBHPCANTX 	/* Vector 16+19: USB High Priority or CAN TX interrupts*/
	HANDLER	stm32_usblpcanrx0, STM32_IRQ_USBLPCANRX0 	/* Vector 16+20: USB Low Priority or CAN RX0 interrupts*/
	HANDLER	stm32_can1rx1, STM32_IRQ_CAN1RX1 		/* Vector 16+21: CAN1 RX1 interrupt */
	HANDLER	stm32_can1sce, STM32_IRQ_CAN1SCE 		/* Vector 16+22: CAN1 SCE interrupt */
	HANDLER	stm32_exti95, STM32_IRQ_EXTI95 			/* Vector 16+23: EXTI Line[9:5] interrupts */
	HANDLER	stm32_tim1brk, STM32_IRQ_TIM1BRK 		/* Vector 16+24: TIM1 Break interrupt */
	HANDLER	stm32_tim1up, STM32_IRQ_TIM1UP 			/* Vector 16+25: TIM1 Update interrupt */
	HANDLER	stm32_tim1rtgcom, STM32_IRQ_TIM1TRGCOM 	/* Vector 16+26: TIM1 Trigger and Commutation interrupts */
	HANDLER	stm32_tim1cc, STM32_IRQ_TIM1CC 			/* Vector 16+27: TIM1 Capture Compare interrupt */
	HANDLER	stm32_tim2, STM32_IRQ_TIM2 				/* Vector 16+28: TIM2 global interrupt */
	HANDLER	stm32_tim3, STM32_IRQ_TIM3				/* Vector 16+29: TIM3 global interrupt */
	HANDLER	stm32_tim4, STM32_IRQ_TIM4 				/* Vector 16+30: TIM4 global interrupt */
	HANDLER	stm32_i2c1ev, STM32_IRQ_I2C1EV 			/* Vector 16+31: I2C1 event interrupt */
	HANDLER	stm32_i2c1er, STM32_IRQ_I2C1ER 			/* Vector 16+32: I2C1 error interrupt */
	HANDLER	stm32_i2c2ev, STM32_IRQ_I2C2EV			/* Vector 16+33: I2C2 event interrupt */
	HANDLER	stm32_i2c2er, STM32_IRQ_I2C2ER			/* Vector 16+34: I2C2 error interrupt */
	HANDLER	stm32_spi1, STM32_IRQ_SPI1				/* Vector 16+35: SPI1 global interrupt */
	HANDLER	stm32_spi2, STM32_IRQ_SPI2				/* Vector 16+36: SPI2 global interrupt */
	HANDLER	stm32_usart1, STM32_IRQ_USART1 			/* Vector 16+37: USART1 global interrupt */
	HANDLER	stm32_usart2, STM32_IRQ_USART2 			/* Vector 16+38: USART2 global interrupt */
	HANDLER	stm32_usart3, STM32_IRQ_USART3 			/* Vector 16+39: USART3 global interrupt */
	HANDLER	stm32_exti1510, STM32_IRQ_EXTI1510		/* Vector 16+40: EXTI Line[15:10] interrupts */
	HANDLER	stm32_rtcalr, STM32_IRQ_RTCALR			/* Vector 16+41: RTC alarm through EXTI line interrupt */
	HANDLER	stm32_usbwkup, STM32_IRQ_USBWKUP 		/* Vector 16+42: USB wakeup from suspend through EXTI line interrupt*/
	HANDLER	stm32_tim8brk, STM32_IRQ_TIM8BRK 		/* Vector 16+43: TIM8 Break interrupt */
	HANDLER	stm32_tim8up, STM32_IRQ_TIM8UP			/* Vector 16+44: TIM8 Update interrupt */
	HANDLER	stm32_tim8trgcom, STM32_IRQ_TIM8TRGCOM 	/* Vector 16+45: TIM8 Trigger and Commutation interrupts */
	HANDLER	stm32_tim8cc, STM32_IRQ_TIM8CC 			/* Vector 16+46: TIM8 Capture Compare interrupt */
	HANDLER	stm32_adc3, STM32_IRQ_ADC3				/* Vector 16+47: ADC3 global interrupt */
	HANDLER	stm32_fsmc, STM32_IRQ_FSMC				/* Vector 16+48: FSMC global interrupt */
	HANDLER	stm32_sdio, STM32_IRQ_SDIO				/* Vector 16+49: SDIO global interrupt */
	HANDLER	stm32_tim5, STM32_IRQ_TIM5				/* Vector 16+50: TIM5 global interrupt */
	HANDLER	stm32_spi3, STM32_IRQ_SPI3 				/* Vector 16+51: SPI3 global interrupt */
	HANDLER	stm32_uart4, STM32_IRQ_UART4 			/* Vector 16+52: UART4 global interrupt */
	HANDLER	stm32_uart5, STM32_IRQ_UART5			/* Vector 16+53: UART5 global interrupt */
	HANDLER	stm32_tim6, STM32_IRQ_TIM6 				/* Vector 16+54: TIM6 global interrupt */
	HANDLER	stm32_tim7, STM32_IRQ_TIM7				/* Vector 16+55: TIM7 global interrupt */
	HANDLER	stm32_dma2ch1, STM32_IRQ_DMA2CH1		/* Vector 16+56: DMA2 Channel 1 global interrupt */
	HANDLER	stm32_dma2ch2, STM32_IRQ_DMA2CH2		/* Vector 16+57: DMA2 Channel 2 global interrupt */
	HANDLER	stm32_dma2ch3, STM32_IRQ_DMA2CH3 		/* Vector 16+58: DMA2 Channel 3 global interrupt */
	HANDLER	stm32_dma2ch45, STM32_IRQ_DMA2CH45		/* Vector 16+59: DMA2 Channel 4&5 global interrupt */
#endif

/* Common IRQ handling logic.  On entry here, the stack is like the following:
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
 * and R0 contains the IRQ number
 */

stm32_common:

	/* Complete the context save */

	mrs		r1, msp					/* R1=The main stack pointer */
	mov		r2, r1					/* R2=Copy of the main stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP before the interrupt was taken */
	mrs		r3, primask				/* R3=Current PRIMASK setting */
	stmdb	r1!, {r2-r11}			/* Save the remaining registers plus the SP value */

	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will re-use the main stack for interrupt level processing.
	 */

#ifdef CONFIG_ARCH_INTERRUPTSTACK
	ld		sp, #up_interruptstack_base
	str		r1, [sp, #-4]!			/* Save the MSP on the interrupt stack */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	ldr		r1, [sp, #+4]!			/* Recover R1=main stack pointer */
#else
	mov		sp, r1					/* We are using the main stack pointer */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, sp					/* Recover R1=main stack pointer */
#endif

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		1f						/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather, are within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	add	r1, r0, #SW_XCPT_SIZE		/* R1=Address of HW save area in reg array */
	ldmia	r1, {r4-r11}			/* Fetch eight registers in HW save area */
	ldr	r1, [r0, #(4*REG_SP)]		/* R1=Value of SP before interrupt */
	stmdb	r1!, {r4-r11}			/* Store eight registers in HW save area */
	ldmia	r0, {r2-r11}			/* Recover R4-R11 + 2 temp values */
	b		2f						/* Re-join common logic */

	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created
	 */
1:
	ldmia	r1!, {r2-r11}			/* Recover R4-R11 + 2 temp values */
2:
	msr		msp, r1					/* Recover the return MSP value */

	/* Do we need to restore interrupts? */

	tst		r3, #1					/* PRIMASK bit 1=1 means that interrupts are masked */
	bne		3f
	cpsie	i						/* Restore interrupts */

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) continue to use the MSP
	 */
3:
	ldr		r14, =EXC_RETURN		/* Load the special value */
	bx		r14						/* And return */
	.size	handlers, .-handlers

/************************************************************************************
 *  Name: up_interruptstack/g_userstack
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
        .bss
        .align  4
up_interruptstack:
        .skip   (CONFIG_ARCH_INTERRUPTSTACK & ~3)
up_interruptstack_base:
        .size   up_interruptstack, .-up_interruptstack
#endif

/************************************************************************************
 * .rodata
 ************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_heapbase below.
 */

        .globl  g_heapbase
        .type   g_heapbase, object
g_heapbase:
        .long   _ebss+CONFIG_IDLETHREAD_STACKSIZE
        .size   g_heapbase, .-g_heapbase

        .end
