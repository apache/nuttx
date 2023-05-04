/****************************************************************************
 * arch/z80/src/ez80/ez80f92.h
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

#ifndef __ARCH_Z80_SRC_EZ80_EZ80F92_H
#define __ARCH_Z80_SRC_EZ80_EZ80F92_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nuttx/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory map ***************************************************************/

#define EZ80_ONCHIPFLASH       0x000000 /* CS0: 64-128Kb of on-chip flash */
#define EZ80_OFFCHIPCS0        0x400000 /* CS0: Off chip use (usually flash) */
#define EZ80_OFFCHIPCS2        0x800000 /* CS2: Off chip use (e.g. memory mapped I/O) */
#define EZ80_OFFCHIPCS1        0xc00000 /* CS1: Off chip use (usually SRAM) */
#define EZ80_ONCHIPSRAM        0xffe000 /* On-chip SRAM (4-8Kb) on reset */

#if defined(CONFIG_ARCH_CHIP_EZ80L92)
#  define EZ80_FLASH_SIZE      0x020000 /* 128Kb on-chip flash */
#  define EZ80_SRAM_SIZE       0x002000 /* 8Kb on-chip sram */
#elif defined(CONFIG_ARCH_CHIP_EZ80L93)
#  define EZ80_FLASH_SIZE      0x010000 /* 64Kb on-chip flash */
#  define EZ80_SRAM_SIZE       0x001000 /* 4Kb on-chip sram */
#endif

/* Product ID Registers  ****************************************************/

#define EZ80_ZDI_ID_L          0x00
#define EZ80_ZDI_ID_H          0x01
#define EZ80_ZDI_ID_REV        0x02

/* Timer Registers  *********************************************************/

#define EZ80_TMR0_CTL          0x80        /* RW: Timer 0 control register */
#define EZ80_TMR0_DRL          0x81        /* R : Timer 0 data register (low) */
#define EZ80_TMR0_DRH          0x82        /* R : Timer 0 data register (high) */
#define EZ80_TMR0_RRL          0x81        /*  W: Timer 0 reload register (low) */
#define EZ80_TMR0_RRH          0x82        /*  W: Timer 0 reload register (high) */

#define EZ80_TMR1_CTL          0x83        /* RW: Timer 1 control register */
#define EZ80_TMR1_DRL          0x84        /* R : Timer 1 data register (low) */
#define EZ80_TMR1_DRH          0x85        /* R : Timer 1 data register (high) */
#define EZ80_TMR1_RRL          0x84        /*  W: Timer 1 reload register (low) */
#define EZ80_TMR1_RRH          0x85        /*  W: Timer 1 reload register (high) */

#define EZ80_TMR2_CTL          0x86        /* RW: Timer 2 control register */
#define EZ80_TMR2_DRL          0x87        /* R : Timer 2 data register (low) */
#define EZ80_TMR2_DRH          0x88        /* R : Timer 2 data register (high) */
#define EZ80_TMR2_RRL          0x87        /*  W: Timer 2 reload register (low) */
#define EZ80_TMR2_RRH          0x88        /*  W: Timer 2 reload register (high) */

#define EZ80_TMR3_CTL          0x89        /* RW: Timer 3 control register */
#define EZ80_TMR3_DRL          0x8a        /* R : Timer 3 data register (low) */
#define EZ80_TMR3_DRH          0x8b        /* R : Timer 3 data register (high) */
#define EZ80_TMR3_RRL          0x8a        /*  W: Timer 3 reload register (low) */
#define EZ80_TMR3_RRH          0x8b        /*  W: Timer 3 reload register (high) */

#define EZ80_TMR4_CTL          0x8c        /* RW: Timer 4 control register */
#define EZ80_TMR4_DRL          0x8d        /* R : Timer 4 data register (low) */
#define EZ80_TMR4_DRH          0x8e        /* R : Timer 4 data register (high) */
#define EZ80_TMR4_RRL          0x8d        /*  W: Timer 4 reload register (low) */
#define EZ80_TMR4_RRH          0x8e        /*  W: Timer 4 reload register (high) */

#define EZ80_TMR5_CTL          0x8f        /* RW: Timer 5 control register */
#define EZ80_TMR5_DRL          0x90        /* R : Timer 5 data register (low) */
#define EZ80_TMR5_DRH          0x91        /* R : Timer 5 data register (high) */
#define EZ80_TMR5_RRL          0x90        /*  W: Timer 5 reload register (low) */
#define EZ80_TMR5_RRH          0x91        /*  W: Timer 5 reload register (high) */

#define EZ80_TMR_ISS           0x92        /* Timer input source selection register */

/* TMR0/1/2/3 CTL Register Bit Definitions **********************************/

#define EZ80_TMRCTL_IRQ        0x80        /* Bit 7: Generate interrupt request */
#define EZ80_TMRCTL_EN         0x40        /* Bit 6: Enable timer interrupt requests */
                                           /* Bit 5: Reserved */
#define EZ80_TMRCTL_TIMCONT    0x10        /* Bit 4: Continuous mode */
#define EZ80_TMRCTL_CLKDIV     0x18        /* Bits 2-3: Timer input clock divider */
#  define EZ80_TMRCLKDIV_4     0x00        /*   00:   4 */
#  define EZ80_TMRCLKDIV_16    0x04        /*   01:  16 */
#  define EZ80_TMRCLKDIV_64    0x08        /*   10:  64 */
#  define EZ80_TMRCLKDIV_256   0x0c        /*   11: 256 */
#define EZ80_TMRCTL_RSTEN      0x02        /* Bit 1: Reload and start function enabled */
#define EZ80_TMRCTL_TIMEN      0x01        /* Bit 0: Programmable reload timer enabled */

/* WDT Registers ************************************************************/

#define EZ80_WDT_CTL           0x93
#define EZ80_WDT_RR            0x94

/* GPIO Registers ***********************************************************/

#define EZ80_PB_DR             0x9a
#define EZ80_PB_DDR            0x9b
#define EZ80_PB_ALT1           0x9c
#define EZ80_PB_ALT2           0x9d
#define EZ80_PC_DR             0x9e
#define EZ80_PC_DDR            0x9f
#define EZ80_PC_ALT1           0xa0
#define EZ80_PC_ALT2           0xa1
#define EZ80_PD_DR             0xa2
#define EZ80_PD_DDR            0xa3
#define EZ80_PD_ALT1           0xa4
#define EZ80_PD_ALT2           0xa5

/* CS Registers *************************************************************/

#define EZ80_CS0_LBR           0xa8
#define EZ80_CS0_UBR           0xa9
#define EZ80_CS0_CTL           0xaa
#define EZ80_CS1_LBR           0xab
#define EZ80_CS1_UBR           0xac
#define EZ80_CS1_CTL           0xad
#define EZ80_CS2_LBR           0xae
#define EZ80_CS2_UBR           0xaf
#define EZ80_CS2_CTL           0xb0
#define EZ80_CS3_LBR           0xb1
#define EZ80_CS3_UBR           0xb2
#define EZ80_CS3_CTL           0xb3

/* RAMCTL registers *********************************************************/

#define EZ80_RAM_CTL           0xb4
#define EZ80_RAM_ADDR_U        0xb5

/* RAMCTL bit definitions ***************************************************/

#define RAMCTL_ERAMEN          (1 << 6)    /* Bit 7: 1=On chip EMAC SRAM is enabled */
#define RAMCTL_GPRAMEN         (1 << 7)    /* Bit 7: 1=On chip GP SRAM is enabled */

/* SPI Registers ************************************************************/

#define EZ80_SPI_BRG_L         0xb8
#define EZ80_SPI_BRG_H         0xb9
#define EZ80_SPI_CTL           0xba
#define EZ80_SPI_SR            0xbb
#define EZ80_SPI_RBR           0xbc
#define EZ80_SPI_TSR           0xbc

/* Infrared Encoder/Decoder Block *******************************************/

#define EZ80_IR_CTL            0xbf        /* Infrared Encoder/Decoder Control */

/* UART Register Offsets ****************************************************/

                                           /* DLAB=0: */
#define EZ80_UART_THR          0x00        /*    W: UART Transmit holding register */
#define EZ80_UART_RBR          0x00        /*   R : UART Receive buffer register */
#define EZ80_UART_IER          0x01        /*   RW: UART Interrupt enable register */
                                           /* DLAB=1: */
#define EZ80_UART_BRG          0x00        /*   RW: UART Baud rate generator register */
#define EZ80_UART_BRGL         0x00        /*   RW: UART Baud rate generator register (low) */
#define EZ80_UART_BRGH         0x01        /*   RW: UART Baud rate generator register (high) */
                                           /* DLAB=N/A: */
#define EZ80_UART_IIR          0x02        /*   R : UART Interrupt identification register */
#define EZ80_UART_FCTL         0x02        /*    W: UART FIFO control register */
#define EZ80_UART_LCTL         0x03        /*   RW: UART Line control register */
#define EZ80_UART_MCTL         0x04        /*   RW: UART Modem control register */
#define EZ80_UART_LSR          0x05        /*   R : UART Line status register */
#define EZ80_UART_MSR          0x06        /*   R : UART Modem status register */
#define EZ80_UART_SPR          0x07        /*   RW: UART Scratchpad register */

/* UART0/1 Base Register Addresses ******************************************/

#define EZ80_UART0_BASE        0xc0
#define EZ80_UART1_BASE        0xd0

/* UART0/1 IER register bits ************************************************/

#define EZ80_UARTEIR_INTMASK   0x1f         /* Bits 5-7: Reserved */
#define EZ80_UARTEIR_TCIE      0x10         /* Bit 4: Transmission complete interrupt */
#define EZ80_UARTEIR_MIIE      0x08         /* Bit 3: Modem status input interrupt */
#define EZ80_UARTEIR_LSIE      0x04         /* Bit 2: Line status interrupt */
#define EZ80_UARTEIR_TIE       0x02         /* Bit 1: Transmit interrupt */
#define EZ80_UARTEIR_RIE       0x01         /* Bit 0: Receive interrupt */

/* UART0/1 IIR register bits ************************************************/

#define EZ80_UARTIIR_FSTS      0xc0         /* Bits 6-7: FIFO enable */
#define EZ80_UARTIIR_FDIS      0x00         /*   00: FIFO disabled */
#define EZ80_UARTIIR_FRXDIS    0x80         /*   10: Rx FIFO disabled */
#define EZ80_UARTIIR_FEN       0xc0         /*   11: FIFO enabled */
                                            /* Bits 4-5: Reserved */
#define EZ80_UARTIIR_INSTS     0x0e         /* Bits 1-3: Interrupt status code */
#  define EZ80_UARTINSTS_CTO   0x0c         /*   110: Character timeout */
#  define EZ80_UARTINSTS_TC    0x0a         /*   101: Transmission complete */
#  define EZ80_UARTINSTS_RLS   0x06         /*   011: Receiver line status */
#  define EZ80_UARTINSTS_RDR   0x04         /*   010: Receive data ready or trigger level */
#  define EZ80_UARTINSTS_TBE   0x02         /*   001: Transmisson buffer empty */
#  define EZ80_UARTINSTS_MS    0x00         /*   000: Modem status */
#define EZ80_UARTIIR_INTBIT    0x01         /* Bit 0: (NOT) Active interrupt source */
#define EZ80_UARTIIR_CAUSEMASK 0x0f

/* UART0/1 FCTL register bits ***********************************************/

#define EZ80_UARTFCTL_TRIG     0xc0         /* Bits 6-7: UART receive FIFO trigger level */
#  define EZ80_UARTTRIG_1      0x00         /*   00: Receive FIFO trigger level=1 */
#  define EZ80_UARTTRIG_4      0x40         /*   01: Receive FIFO trigger level=4 */
#  define EZ80_UARTTRIG_8      0x80         /*   10: Receive FIFO trigger level=8 */
#  define EZ80_UARTTRIG_14     0xc0         /*   11: Receive FIFO trigger level=14 */
                                            /* Bit 3-5: Reserved */
#define EZ80_UARTFCTL_CLRTXF   0x04         /* Bit 2: Transmit enable */
#define EZ80_UARTFCTL_CLRRXF   0x02         /* Bit 1: Receive enable */
#define EZ80_UARTFCTL_FIFOEN   0x01         /* Bit 0: Enable receive/transmit FIFOs */

/* UART0/1 LCTL register bits ***********************************************/

#define EZ80_UARTLCTL_DLAB     0x80         /* Bit 7: Enable access to baud rate generator */
#define EZ80_UARTLCTL_SB       0x40         /* Bit 6: Send break */
#define EZ80_UARTLCTL_FPE      0x20         /* Bit 5: Force parity error */
#define EZ80_UARTLCTL_EPS      0x10         /* Bit 4: Even parity select */
#define EZ80_UARTLCTL_PEN      0x08         /* Bit 3: Parity enable */
#define EZ80_UARTLCTL_2STOP    0x04         /* Bit 2: 2 stop bits */
#define EZ80_UARTLCTL_CHAR     0x03         /* Bits 0-2: Number of data bits */
#  define EZ80_UARTCHAR_5BITS  0x00         /*   00: 5 data bits */
#  define EZ80_UARTCHAR_6BITS  0x01         /*   01: 6 data bits */
#  define EZ80_UARTCHAR_7BITS  0x02         /*   10: 7 data bits */
#  define EZ80_UARTCHAR_8BITS  0x03         /*   11: 8 data bits */

#define EZ80_UARTLCTL_MASK     0x3f

/* UART0/1 MCTL register bits ***********************************************/

                                            /* Bits 6-7: Reserved */
#define EZ80_UARTMCTL_MDM      0x20         /* Bit 5: Multi-drop mode enable */
#define EZ80_UARTMCTL_LOOP     0x10         /* Bit 4: Loopback mode enable */
#define EZ80_UARTMCTL_OUT2     0x08         /* Bit 3: (loopback mode only) */
#define EZ80_UARTMCTL_OUT1     0x04         /* Bit 2: (loopback mode only) */
#define EZ80_UARTMCTL_RTS      0x02         /* Bit 1: Request to send */
#define EZ80_UARTMCTL_DTR      0x01         /* Bit 0: Data termnal read */

/* UART0/1 LSR register bits ************************************************/

#define EZ80_UARTLSR_ERR       0x80         /* Bit 7: Error detected in FIFO */
#define EZ80_UARTLSR_TEMT      0x40         /* Bit 6: Transmit FIFO empty and idle */
#define EZ80_UARTLSR_THRE      0x20         /* Bit 5: Transmit FIFO empty */
#define EZ80_UARTLSR_BI        0x10         /* Bit 4: Break on input */
#define EZ80_UARTLSR_FE        0x08         /* Bit 3: Framing error */
#define EZ80_UARTLSR_PE        0x04         /* Bit 2: Parity error */
#define EZ80_UARTLSR_OE        0x02         /* Bit 1: Overrun error */
#define EZ80_UARTLSR_DR        0x01         /* Bit 0: Data ready */

/* UART0/1 MSR register bits ************************************************/

#define EZ80_UARTMSR_DCD       0x80         /* Bit 7: Data carrier detect */
#define EZ80_UARTMSR_RI        0x40         /* Bit 6: Ring indicator */
#define EZ80_UARTMSR_DSR       0x20         /* Bit 5: Data set ready */
#define EZ80_UARTMSR_CTS       0x10         /* Bit 4: Clear to send */
#define EZ80_UARTMSR_DDCD      0x08         /* Bit 3: Delta on DCD input */
#define EZ80_UARTMSR_TERI      0x04         /* Bit 2: Trailing edge change on RI */
#define EZ80_UARTMSR_DDSR      0x02         /* Bit 1: Delta on DSR input */
#define EZ80_UARTMSR_DCTS      0x01         /* Bit 0: Delta on CTS input */

/* I2C Registers  ***********************************************************/

#define EZ80_I2C_SAR           0xc8
#define EZ80_I2C_XSAR          0xc9
#define EZ80_I2C_DR            0xca
#define EZ80_I2C_CTL           0xcb
#define EZ80_I2C_SR            0xcc
#define EZ80_I2C_CCR           0xcc
#define EZ80_I2C_SRR           0xcd

/* CLK Registers  ***********************************************************/

#define EZ80_CLK_PPD1          0xdb
#define EZ80_CLK_PPD2          0xdc

/* RTC Registers  ***********************************************************/

#define EZ80_RTC_SEC           0xe0
#define EZ80_RTC_MIN           0xe1
#define EZ80_RTC_HRS           0xe2
#define EZ80_RTC_DOW           0xe3
#define EZ80_RTC_DOM           0xe4
#define EZ80_RTC_MON           0xe5
#define EZ80_RTC_YR            0xe6
#define EZ80_RTC_CEN           0xe7
#define EZ80_RTC_ASEC          0xe8
#define EZ80_RTC_AMIN          0xe9
#define EZ80_RTC_AHRS          0xea
#define EZ80_RTC_ADOW          0xeb
#define EZ80_RTC_ACTRL         0xec
#define EZ80_RTC_CTRL          0xed

/* CSBMC Registers  *********************************************************/

#define EZ80_CS0_BMC           0xf0
#define EZ80_CS1_BMC           0xf1
#define EZ80_CS2_BMC           0xf2
#define EZ80_CS3_BMC           0xf3

/* FLASH Registers  *********************************************************/

#define EZ80_FLASH_KEY         0xf5
#define EZ80_FLASH_DATA        0xf6
#define EZ80_FLASH_ADDR_U      0xf7
#define EZ80_FLASH_CTRL        0xf8
#define EZ80_FLASH_FDIV        0xf9
#define EZ80_FLASH_PROT        0xfa
#define EZ80_FLASH_INTC        0xfb
#define EZ80_FLASH_PAGE        0xfc
#define EZ80_FLASH_ROW         0xfd
#define EZ80_FLASH_COL         0xfe
#define EZ80_FLASH_PGCTL       0xff

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

#endif /* __ARCH_Z80_SRC_EZ80_EZ80F92_H */
