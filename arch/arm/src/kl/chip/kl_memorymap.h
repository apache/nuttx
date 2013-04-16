/************************************************************************************
 * arch/arm/src/kl/kl_memorymap.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KL_KL_MEMORYMAP_H
#define __ARCH_ARM_SRC_KL_KL_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* from NUC memmap */
#define KL_GPIO_BASEOLD    0x50004000 /* -0x50007fff: GPIO control registers */
#define KL_GCR_BASE        0x50000000 /* -0x500001ff: System global control registers */

/* Memory Map ***********************************************************************/
/* K40 Family
 *
 * The memory map for the following parts is defined in Freescale document
 * K40P144M100SF2RM
 */

# define KL_FLASH_BASE     0x00000000 /* –0x0fffffff Program flash and read-
                                       *             only data (Includes exception
                                       *             vectors in first 1024 bytes) */
# define KL_SRAML_BASE     0x18000000 /* –0x1fffffff SRAM_L: Lower SRAM
                                       *             (ICODE/DCODE) */
# define KL_SRAMU_BASE     0x20000000 /* –0x200fffff SRAM_U: Upper SRAM bitband
                                       *             region */
                        /* 0x20100000  * –0x21ffffff Reserved */
# define KL_SALIAS_BASE    0x22000000 /* –0x23ffffff Aliased to SRAM_U bitband */
                        /* 0x24000000  * –0x3fffffff Reserved */
# define KL_BRIDGE0_BASE   0x40000000 /* –0x4007ffff Bitband region for peripheral
                                       *             bridge 0 (AIPS-Lite0) */
# define KL_BRIDGE1_BASE   0x40080000 /* –0x400fffff Bitband region for peripheral
                                       *             bridge 1 (AIPS-Lite1) */
# define KL_GPIOBB_BASE    0x400ff000 /* –0x400fffff Bitband region for general
                                       *             purpose input/output (GPIO) */
                        /* 0x40100000  * –0x41ffffff Reserved */
# define KL_PALIAS_BASE    0x42000000 /* –0x43ffffff Aliased to peripheral bridge
                                       *             (AIPS-Lite) and general purpose
                                       *             input/output (GPIO) bitband */
                        /* 0x44000000  * –0x5fffffff Reserved */
# define KL_FLEXBUS_WBBASE 0x60000000 /* –0x7fffffff FlexBus (External Memory -
                                       *             Write-back) */
# define KL_FLEXBUS_WTBASE 0x80000000 /* –0x9fffffff FlexBus (External Memory -
                                       *             Write-through) */
# define KL_FLEXBUS_NXBASE 0xa0000000 /* –0xdfffffff FlexBus (External Memory -
                                       *             Non-executable) */
# define KL_PERIPH_BASE    0xe0000000 /* –0xe00fffff Private peripherals */
                        /* 0xe0100000  * –0xffffffff Reserved */

/* Peripheral Bridge 0 Memory Map ***************************************************/

# define KL_AIPS0_BASE     0x40000000 /* Peripheral bridge 0 (AIPS-Lite 0) */
# define KL_XBAR_BASE      0x40004000 /* Crossbar switch */
# define KL_DMAC_BASE      0x40008000 /* DMA controller */
# define KL_DMADESC_BASE   0x40009000 /* DMA controller transfer control descriptors */
# define KL_FLEXBUSC_BASE  0x4000c000 /* FlexBus controller */
# define KL_MPU_BASE       0x4000d000 /* MPU */
# define KL_FMC_BASE       0x4001f000 /* Flash memory controller */
# define KL_FTFL_BASE      0x40020000 /* Flash memory */
# define KL_DMAMUX0_BASE   0x40021000 /* DMA channel mutiplexer 0 */
# define KL_CAN0_BASE      0x40024000 /* FlexCAN 0 */
# define KL_SPI0_BASE      0x4002c000 /* SPI 0 */
# define KL_SPI1_BASE      0x4002d000 /* SPI 1 */
# define KL_I2S0_BASE      0x4002f000 /* I2S 0 */
# define KL_CRC_BASE       0x40032000 /* CRC */
# define KL_USBDCD_BASE    0x40035000 /* USB DCD */
# define KL_PDB0_BASE      0x40036000 /* Programmable delay block */
# define KL_PIT_BASE       0x40037000 /* Periodic interrupt timers (PIT) */
# define KL_FTM0_BASE      0x40038000 /* FlexTimer 0 */
# define KL_FTM1_BASE      0x40039000 /* FlexTimer 1 */
# define KL_ADC0_BASE      0x4003b000 /* Analog-to-digital converter (ADC) 0 */
# define KL_RTC_BASE       0x4003d000 /* Real time clock */
# define KL_VBATR_BASE     0x4003e000 /* VBAT register file */
# define KL_LPTMR_BASE     0x40040000 /* Low power timer */
# define KL_SYSR_BASE      0x40041000 /* System register file */
# define KL_DRYICE_BASE    0x40042000 /* DryIce */
# define KL_DRYICESS_BASE  0x40043000 /* DryIce secure storage */
# define KL_TSI0_BASE      0x40045000 /* Touch sense interface */
# define KL_SIMLP_BASE     0x40047000 /* SIM low-power logic */
# define KL_SIM_BASE       0x40048000 /* System integration module (SIM) */
# define KL_PORT_BASE(n)   (0x40049000 + ((n) << 12))
# define KL_PORTA_BASE     0x40049000 /* Port A multiplexing control */
# define KL_PORTB_BASE     0x4004a000 /* Port B multiplexing control */
# define KL_PORTC_BASE     0x4004b000 /* Port C multiplexing control */
# define KL_PORTD_BASE     0x4004c000 /* Port D multiplexing control */
# define KL_PORTE_BASE     0x4004d000 /* Port E multiplexing control */
# define KL_WDOG_BASE      0x40052000 /* Software watchdog */
# define KL_EWM_BASE       0x40061000 /* External watchdog */
# define KL_CMT_BASE       0x40062000 /* Carrier modulator timer (CMT) */
# define KL_MCG_BASE       0x40064000 /* Multi-purpose Clock Generator (MCG) */
# define KL_OSC_BASE       0x40065000 /* System oscillator (OSC) */
# define KL_I2C0_BASE      0x40066000 /* I2C 0 */
# define KL_I2C1_BASE      0x40067000 /* I2C 1 */
# define KL_UART0_BASE     0x4006a000 /* UART0 */
# define KL_UART1_BASE     0x4006b000 /* UART1 */
# define KL_UART2_BASE     0x4006c000 /* UART2 */
# define KL_UART3_BASE     0x4006d000 /* UART3 */
# define KL_USB0_BASE      0x40072000 /* USB OTG FS/LS */
# define KL_CMP_BASE       0x40073000 /* Analog comparator (CMP) / 6-bit digital-to-analog converter (DAC) */
# define KL_VREF_BASE      0x40074000 /* Voltage reference (VREF) */
# define KL_LLWU_BASE      0x4007c000 /* Low-leakage wakeup unit (LLWU) */
# define KL_PMC_BASE       0x4007d000 /* Power management controller (PMC) */
# define KL_SMC_BASE       0x4007e000 /* System Mode controller (SMC) */

/* Peripheral Bridge 1 Memory Map ***************************************************/

# define KL_AIPS1_BASE     0x40080000 /* Peripheral bridge 1 (AIPS-Lite 1) */
# define KL_CAN1_BASE      0x400a4000 /* FlexCAN 1 */
# define KL_SPI2_BASE      0x400ac000 /* SPI 2 */
# define KL_SDHC_BASE      0x400b1000 /* SDHC */
# define KL_FTM2_BASE      0x400b8000 /* FlexTimer 2 */
# define KL_ADC1_BASE      0x400bb000 /* Analog-to-digital converter (ADC) 1 */
# define KL_SLCD_BASE      0x400be000 /* Segment LCD */
# define KL_DAC0_BASE      0x400cc000 /* 12-bit digital-to-analog converter (DAC) 0 */
# define KL_DAC1_BASE      0x400cd000 /* 12-bit digital-to-analog converter (DAC) 1 */
# define KL_UART4_BASE     0x400ea000 /* UART4 */
# define KL_UART5_BASE     0x400eb000 /* UART5 */
# define KL_XBARSS_BASE    0x400ff000 /* Not an AIPS-Lite slot. The 32-bit general
                                       * purpose input/output module that shares the
                                       * crossbar switch slave port with the AIPS-Lite
                                       * is accessed at this address. */
# define KL_GPIO_BASE(n)   (0x400ff000 + ((n) << 6))
# define KL_GPIOA_BASE     0x400ff000 /* GPIO PORTA registers */
# define KL_GPIOB_BASE     0x400ff040 /* GPIO PORTB registers */
# define KL_GPIOC_BASE     0x400ff080 /* GPIO PORTC registers */
# define KL_GPIOD_BASE     0x400ff0c0 /* GPIO PORTD registers */
# define KL_GPIOE_BASE     0x400ff100 /* GPIO PORTE registers */

/* Private Peripheral Bus (PPB) Memory Map ******************************************/

# define KL_ITM_BASE       0xe0000000 /* Instrumentation Trace Macrocell (ITM) */
# define KL_DWT_BASE       0xe0001000 /* Data Watchpoint and Trace (DWT) */
# define KL_FPB_BASE       0xe0002000 /* Flash Patch and Breakpoint (FPB) */
# define KL_SCS_BASE       0xe000e000 /* System Control Space (SCS) (for NVIC) */
# define KL_TPIU_BASE      0xe0040000 /* Trace Port Interface Unit (TPIU) */
# define KL_ETM_BASE       0xe0041000 /* Embedded Trace Macrocell (ETM) */
# define KL_ETB_BASE       0xe0042000 /* Embedded Trace Buffer (ETB) */
# define KL_TFUN_BASE      0xe0043000 /* Embedded Trace Funnel */
# define KL_MCM_BASE       0xe0080000 /* Miscellaneous Control Module (including ETB Almost Full) */
# define KL_ROMTAB_BASE    0xe00ff000 /* ROM Table - allows auto-detection of debug components */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_KL_MEMORYMAP_H */
