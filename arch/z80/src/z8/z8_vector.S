/**************************************************************************
 * arch/z80/src/z8/z8_xdef.S
 * Interrupt Handling
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS or IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER or CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, or CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS or SERVICES; LOSS
 * OF USE, DATA, or PROFITS; or BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, or TORT (INCLUDING NEGLIGENCE or OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <ez8.inc>
#include <configl.inc>
#include <vect.inc>

/**************************************************************************
 * Definitions
 **************************************************************************/

/**************************************************************************
 * External References / External Definitions
 **************************************************************************/

#if defined(ENCORE_VECTORS)
	xdef _z8_wdt_handler
	xdef _z8_trap_handler
# if defined(EZ8_TIMER3)
	xdef _z8_timer2_handler
# endif
	xdef _z8_timer1_handler
	xdef _z8_timer0_handler
# if defined(EZ8_UART0)
	xdef _z8_uart0rx_handler
	xdef _z8_uart0tx_handler
# endif
# if defined(EZ8_I2C)
	xdef _z8_i2c_handler
# endif
# if defined(EZ8_SPI)
	xdef _z8_spi_handler
# endif
# if defined(EZ8_ADC)
	xdef _z8_adc_handler
# endif
	xdef _z8_p7ad_handler
	xdef _z8_p6ad_handler
 	xdef _z8_p5ad_handler
	xdef _z8_p4ad_handler
	xdef _z8_p3ad_handler
	xdef _z8_p2ad_handler
	xdef _z8_p1ad_handler
	xdef _z8_p0ad_handler
# if defined(EZ8_TIMER4)
	xdef _z8_timer3_handler
# endif
# if defined(EZ8_UART1)
	xdef _z8_uart1rx_handler
	xdef _z8_uart1tx_handler
# endif
# if defined(EZ8_DMA)
	xdef _z8_dma_handler
# endif
# if !defined(EZ8_PORT1)
	xdef _z8_c3_handler
	xdef _z8_c2_handler
	xdef _z8_c1_handler
	xdef _z8_c0_handler
# endif

/**************************************************************************/

#elif defined(ENCORE_XP_VECTORS)

	xdef _z8_wdt_handler
	xdef _z8_trap_handler
# if defined(EZ8_TIMER3)
	xdef _z8_timer2_handler
# endif
	xdef _z8_timer1_handler
	xdef _z8_timer0_handler
# if defined(EZ8_UART0)
	xdef _z8_uart0rx_handler
	xdef _z8_uart0tx_handler
# endif
# if defined(EZ8_I2C)
	xdef _z8_i2c_handler
# endif
# if defined(EZ8_SPI)
	xdef _z8_spi_handler
#   endif
# if  defined(EZ8_ADC) || defined(EZ8_ADC_NEW)
	xdef _z8_adc_handler
# endif
	xdef _z8_p7ad_handler
	xdef _z8_p6ad_handler
 	xdef _z8_p5ad_handler
	xdef _z8_p4ad_handler
	xdef _z8_p3ad_handler
	xdef _z8_p2ad_handler
	xdef _z8_p1ad_handler
	xdef _z8_p0ad_handler
# if defined(EZ8_TIMER4)
	xdef _z8_timer3_handler
# endif
# if defined(EZ8_UART1)
	xdef _z8_uart1rx_handler
	xdef _z8_uart1tx_handler
# endif
# if defined(EZ8_DMA)
	xdef _z8_dma_handler
# endif
# if !defined(EZ8_PORT1)
	xdef _z8_c3_handler
	xdef _z8_c2_handler
	xdef _z8_c1_handler
	xdef _z8_c0_handler
# endif
	xdef _z8_potrap_handler
	xdef _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_XP16K_VECTORS)

	xdef _z8_wdt_handler
	xdef _z8_trap_handler
# if defined(EZ8_TIMER3)
	xdef _z8_timer2_handler
# endif
	xdef _z8_timer1_handler
	xdef _z8_timer0_handler
# if defined(EZ8_UART0)
	xdef _z8_uart0rx_handler
	xdef _z8_uart0tx_handler
# endif
# if defined(EZ8_I2C)
	xdef _z8_i2c_handler
# endif
# if defined(EZ8_ESPI)
	xdef _z8_spi_handler
# endif
# if  defined(EZ8_ADC_NEW)
	xdef _z8_adc_handler
# endif
	xdef _z8_p7ad_handler
	xdef _z8_p6ad_handler
 	xdef _z8_p5ad_handler
	xdef _z8_p4ad_handler
	xdef _z8_p3ad_handler
	xdef _z8_p2ad_handler
	xdef _z8_p1ad_handler
	xdef _z8_p0ad_handler
# if defined(EZ8_MCT)
	xdef _z8_mct_handler
# endif
# if defined(EZ8_UART1)
	xdef _z8_uart1rx_handler
	xdef _z8_uart1tx_handler
# endif
	xdef _z8_c3_handler
	xdef _z8_c2_handler
	xdef _z8_c1_handler
	xdef _z8_c0_handler
	xdef _z8_potrap_handler
	xdef _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_MC_VECTORS)

	xdef _z8_wdt_handler
	xdef _z8_trap_handler
	xdef _z8_pwmtimer_handler
	xdef _z8_pwmfault_handler
# if defined(EZ8_ADC_NEW)
	xdef _z8_adc_handler
# endif
	xdef _z8_cmp_handler
	xdef _z8_timer0_handler
# if defined(EZ8_UART0
	xdef _z8_uart0rx_handler
	xdef _z8_uart0tx_handler
# endif
# if defined(EZ8_SPI)
	xdef _z8_spi_handler
# endif
# if defined(EZ8_I2C)
	xdef _z8_i2c_handler
# endif
	xdef _z8_c0_handler
	xdef _z8_c0_handler
	xdef _z8_p7p3a_handler
	xdef _z8_p6ap2a_handler
	xdef _z8_p5ap1a_handler
	xdef _z8_p4ap0a_handler
	xdef _z8_potrap_handler
	xdef _z8_wotrap_handler
#endif 

 /**************************************************************************
 * Code
 **************************************************************************/

#if defined(ENCORE_VECTORS)
	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
# if defined(EZ8_TIMER3)
	vector TIMER2	= _z8_timer2_handler
# endif
	vector TIMER1	= _z8_timer1_handler
	vector TIMER0	= _z8_timer0_handler
# if defined(EZ8_UART0)
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
# endif
# if defined(EZ8_I2C)
	vector I2C	= _z8_i2c_handler
# endif
# if defined(EZ8_SPI)
	vector SPI	= _z8_spi_handler
# endif
# if defined(EZ8_ADC)
	vector ADC	= _z8_adc_handler
# endif
	vector P7AD	= _z8_p7ad_handler
	vector P6AD	= _z8_p6ad_handler
 	vector P5AD	= _z8_p5ad_handler
	vector P4AD	= _z8_p4ad_handler
	vector P3AD	= _z8_p3ad_handler
	vector P2AD	= _z8_p2ad_handler
	vector P1AD	= _z8_p1ad_handler
	vector P0AD	= _z8_p0ad_handler
# if defined(EZ8_TIMER4)
	vector TIMER3	= _z8_timer3_handler
# endif
# if defined(EZ8_UART1)
	vector UART1_RX	= _z8_uart1rx_handler
	vector UART1_TX	= _z8_uart1tx_handler
# endif
# if defined(EZ8_DMA)
	vector DMA	= _z8_dma_handler
# endif
# if !defined(EZ8_PORT1)
	vector C3	= _z8_c3_handler
	vector C2	= _z8_c2_handler
	vector C1	= _z8_c1_handler
	vector C0	= _z8_c0_handler
# endif

/**************************************************************************/

#elif defined(ENCORE_XP_VECTORS)

	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
# if defined(EZ8_TIMER3)
	vector TIMER2	= _z8_timer2_handler
# endif
	vector TIMER1	= _z8_timer1_handler
	vector TIMER0	= _z8_timer0_handler
# if defined(EZ8_UART0)
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
# endif
# if defined(EZ8_I2C)
	vector I2C	= _z8_i2c_handler
# endif
# if defined(EZ8_SPI)
	vector SPI	= _z8_spi_handler
#   endif
# if  defined(EZ8_ADC) || defined(EZ8_ADC_NEW)
	vector ADC	= _z8_adc_handler
# endif
	vector P7AD	= _z8_p7ad_handler
	vector P6AD	= _z8_p6ad_handler
 	vector P5AD	= _z8_p5ad_handler
	vector P4AD	= _z8_p4ad_handler
	vector P3AD	= _z8_p3ad_handler
	vector P2AD	= _z8_p2ad_handler
	vector P1AD	= _z8_p1ad_handler
	vector P0AD	= _z8_p0ad_handler
# if defined(EZ8_TIMER4)
	vector TIMER3	= _z8_timer3_handler
# endif
# if defined(EZ8_UART1)
	vector UART1_RX	= _z8_uart1rx_handler
	vector UART1_TX	= _z8_uart1tx_handler
# endif
# if defined(EZ8_DMA)
	vector DMA	= _z8_dma_handler
# endif
# if !defined(EZ8_PORT1)
	vector C3	= _z8_c3_handler
	vector C2	= _z8_c2_handler
	vector C1	= _z8_c1_handler
	vector C0	= _z8_c0_handler
# endif
	vector POTRAP	= _z8_potrap_handler
	vector WOTRAP	= _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_XP16K_VECTORS)

	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
# if defined(EZ8_TIMER3)
	vector TIMER2	= _z8_timer2_handler
# endif
	vector TIMER1	= _z8_timer1_handler
	vector TIMER0	= _z8_timer0_handler
# if defined(EZ8_UART0)
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
# endif
# if defined(EZ8_I2C)
	vector I2C	= _z8_i2c_handler
# endif
# if defined(EZ8_ESPI)
	vector SPI	= _z8_spi_handler
# endif
# if  defined(EZ8_ADC_NEW)
	vector ADC	= _z8_adc_handler
# endif
	vector P7AD	= _z8_p7ad_handler
	vector P6AD	= _z8_p6ad_handler
 	vector P5AD	= _z8_p5ad_handler
	vector P4AD	= _z8_p4ad_handler
	vector P3AD	= _z8_p3ad_handler
	vector P2AD	= _z8_p2ad_handler
	vector P1AD	= _z8_p1ad_handler
	vector P0AD	= _z8_p0ad_handler
# if defined(EZ8_MCT)
	vector MCT	= _z8_mct_handler
# endif
# if defined(EZ8_UART1)
	vector UART1_RX	= _z8_uart1rx_handler
	vector UART1_TX	= _z8_uart1tx_handler
# endif
	vector C3	= _z8_c3_handler
	vector C2	= _z8_c2_handler
	vector C1	= _z8_c1_handler
	vector C0	= _z8_c0_handler
	vector POTRAP	= _z8_potrap_handler
	vector WOTRAP	= _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_MC_VECTORS)

	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
	vector PWMTIMER	= _z8_pwmtimer_handler
	vector PWMFAULT	= _z8_pwmfault_handler
# if defined(EZ8_ADC_NEW)
	vector ADC	= _z8_adc_handler
# endif
	vector CMP	= _z8_cmp_handler
	vector TIMER0	= _z8_timer0_handler
# if defined(EZ8_UART0
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
# endif
# if defined(EZ8_SPI)
	vector SPI	= _z8_spi_handler
# endif
# if defined(EZ8_I2C)
	vector I2C	= _z8_i2c_handler
# endif
	vector C0	= _z8_c0_handler
	vector PB	= _z8_c0_handler
	vector P7A	= _z8_p7p3a_handler
	vector P6A	= _z8_p6ap2a_handler
	vector P5A	= _z8_p5ap1a_handler
	vector P4A	= _z8_p4ap0a_handler
	vector POTRAP	= _z8_potrap_handler
	vector WOTRAP	= _z8_wotrap_handler
#endif 

/**************************************************************************
 * Name: _z16f_*_handler
 *
 * Description:
 *   Map individual interrupts into interrupt number and branch to common
 *   interrupt handling logic.  If higher interrupt handling performance
 *   for particular interrupts is required, then those interrupts should
 *   be picked off here and handled outside of the common logic.
 *
 **************************************************************************/

	define startup, space=rom
	segment startup
#if defined(ENCORE_VECTORS)
_z8_wdt_handler:
_z8_trap_handler:
# if defined(EZ8_TIMER3)
_z8_timer2_handler:
# endif
_z8_timer1_handler:
_z8_timer0_handler:
# if defined(EZ8_UART0)
_z8_uart0rx_handler:
_z8_uart0tx_handler:
# endif
# if defined(EZ8_I2C)
_z8_i2c_handler:
# endif
# if defined(EZ8_SPI)
_z8_spi_handler:
# endif
# if defined(EZ8_ADC)
_z8_adc_handler:
# endif
_z8_p7ad_handler:
_z8_p6ad_handler:
 _z8_p5ad_handler:
_z8_p4ad_handler:
_z8_p3ad_handler:
_z8_p2ad_handler:
_z8_p1ad_handler:
_z8_p0ad_handler:
# if defined(EZ8_TIMER4)
_z8_timer3_handler:
# endif
# if defined(EZ8_UART1)
_z8_uart1rx_handler:
_z8_uart1tx_handler:
# endif
# if defined(EZ8_DMA)
_z8_dma_handler:
# endif
# if !defined(EZ8_PORT1)
_z8_c3_handler:
_z8_c2_handler:
_z8_c1_handler:
_z8_c0_handler:
# endif

/**************************************************************************/

#elif defined(ENCORE_XP_VECTORS)

_z8_wdt_handler:
_z8_trap_handler:
# if defined(EZ8_TIMER3)
_z8_timer2_handler:
# endif
_z8_timer1_handler:
_z8_timer0_handler:
# if defined(EZ8_UART0)
_z8_uart0rx_handler:
_z8_uart0tx_handler:
# endif
# if defined(EZ8_I2C)
_z8_i2c_handler:
# endif
# if defined(EZ8_SPI)
_z8_spi_handler:
#   endif
# if  defined(EZ8_ADC) || defined(EZ8_ADC_NEW)
_z8_adc_handler:
# endif
_z8_p7ad_handler:
_z8_p6ad_handler:
 _z8_p5ad_handler:
_z8_p4ad_handler:
_z8_p3ad_handler:
_z8_p2ad_handler:
_z8_p1ad_handler:
_z8_p0ad_handler:
# if defined(EZ8_TIMER4)
_z8_timer3_handler:
# endif
# if defined(EZ8_UART1)
_z8_uart1rx_handler:
_z8_uart1tx_handler:
# endif
# if defined(EZ8_DMA)
_z8_dma_handler:
# endif
# if !defined(EZ8_PORT1)
_z8_c3_handler:
_z8_c2_handler:
_z8_c1_handler:
_z8_c0_handler:
# endif
_z8_potrap_handler:
_z8_wotrap_handler:

/**************************************************************************/

#elif defined(ENCORE_XP16K_VECTORS)

_z8_wdt_handler:
_z8_trap_handler:
# if defined(EZ8_TIMER3)
_z8_timer2_handler:
# endif
_z8_timer1_handler:
_z8_timer0_handler:
# if defined(EZ8_UART0)
_z8_uart0rx_handler:
_z8_uart0tx_handler:
# endif
# if defined(EZ8_I2C)
_z8_i2c_handler:
# endif
# if defined(EZ8_ESPI)
_z8_spi_handler:
# endif
# if  defined(EZ8_ADC_NEW)
_z8_adc_handler:
# endif
_z8_p7ad_handler:
_z8_p6ad_handler:
 _z8_p5ad_handler:
_z8_p4ad_handler:
_z8_p3ad_handler:
_z8_p2ad_handler:
_z8_p1ad_handler:
_z8_p0ad_handler:
# if defined(EZ8_MCT)
_z8_mct_handler:
# endif
# if defined(EZ8_UART1)
_z8_uart1rx_handler:
_z8_uart1tx_handler:
# endif
_z8_c3_handler:
_z8_c2_handler:
_z8_c1_handler:
_z8_c0_handler:
_z8_potrap_handler:
_z8_wotrap_handler:

/**************************************************************************/

#elif defined(ENCORE_MC_VECTORS)

_z8_wdt_handler:
_z8_trap_handler:
_z8_pwmtimer_handler:
_z8_pwmfault_handler:
# if defined(EZ8_ADC_NEW)
_z8_adc_handler:
# endif
_z8_cmp_handler:
_z8_timer0_handler:
# if defined(EZ8_UART0
_z8_uart0rx_handler:
_z8_uart0tx_handler:
# endif
# if defined(EZ8_SPI)
_z8_spi_handler:
# endif
# if defined(EZ8_I2C)
_z8_i2c_handler:
# endif
_z8_c0_handler:
_z8_c0_handler:
_z8_p7p3a_handler:
_z8_p6ap2a_handler:
_z8_p5ap1a_handler:
_z8_p4ap0a_handler:
_z8_potrap_handler:
_z8_wotrap_handler:
#endif 

/**************************************************************************
 * Name: _z16f_common_handler
 *
 * Description:
 *   Common IRQ handling logic
 *
 **************************************************************************/

_z8_common_handler:
	iret

/**************************************************************************
 * Data
 **************************************************************************/

	end	_z8_common_handler
