/************************************************************************************
 * arch/z80/src/ez80/chip.h
 * arch/z80/src/chip/chip.h
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

#ifndef __EZ80_CHIP_H
#define __EZ80_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Bits in the Z80 FLAGS register ***************************************************/

#define EZ80_C_FLAG            0x01        /* Bit 0: Carry flag */
#define EZ80_N_FLAG            0x02        /* Bit 1: Add/Subtract flag  */
#define EZ80_PV_FLAG           0x04        /* Bit 2: Parity/Overflow flag */
#define EZ80_H_FLAG            0x10        /* Bit 4: Half carry flag */
#define EZ80_Z_FLAG            0x40        /* Bit 5: Zero flag */
#define EZ80_S_FLAG            0x80        /* Bit 7: Sign flag */

/* Memory Map */
/* Special Function Registers *******************************************************
 */

/* Timer Register Bit Definitions ***************************************************/

/* UART Register Offsets *************************************************************/

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

/* UART0/1 Base Register Addresses **************************************************/

#define EZ80_UART0_BASE        0xc0
#define EZ80_UART1_BASE        0xd0

/* UART0/1 IER register bits ********************************************************/

#define EZ80_UARTEIR_INTMASK   0x1f         /* Bits 5-7: Reserved */
#define EZ80_UARTEIR_TCIE      0x10         /* Bit 4: Transmission complete interrupt */
#define EZ80_UARTEIR_MIIE      0x08         /* Bit 3: Modem status input interrupt */
#define EZ80_UARTEIR_LSIE      0x04         /* Bit 2: Line status interrupt */
#define EZ80_UARTEIR_TIE       0x02         /* Bit 1: Transmit interrupt */
#define EZ80_UARTEIR_RIE       0x01         /* Bit 0: Receive interrupt */

/* UART0/1 IIR register bits ********************************************************/

#define EZ80_UARTIIR_FSTS      0x80         /* Bit 7: FIFO enable */
                                            /* Bits 4-6: Reserved */
#define EZ80_UARTIIR_INSTS     0x0e         /* Bits 1-3: Interrupt status code */
#  define EZ80_UARTINSTS_CTO   0x0c         /*   110: Character timeout */
#  define EZ80_UARTINSTS_TC    0x0a         /*   101: Transmission complete */
#  define EZ80_UARTINSTS_RLS   0x06         /*   011: Receiver line status */
#  define EZ80_UARTINSTS_RDR   0x04         /*   010: Receive data ready or trigger level */
#  define EZ80_UARTINSTS_TBE   0x02         /*   001: Transmisson buffer empty */
#  define EZ80_UARTINSTS_MS    0x00         /*   000: Modem status */
#define EZ80_UARTIIR_INTBIT    0x01         /* Bit 0: Active interrupt source */
#define EZ80_UARTIIR_CAUSEMASK 0x0f

/* UART0/1 FCTL register bits *******************************************************/

#define EZ80_UARTFCTL_TRIG     0xc0         /* Bits 6-7: UART recieve FIFO trigger level */
#  define EZ80_UARTTRIG_1      0x00         /*   00: Receive FIFO trigger level=1 */
#  define EZ80_UARTTRIG_4      0x40         /*   01: Receive FIFO trigger level=4 */
#  define EZ80_UARTTRIG_8      0x80         /*   10: Receive FIFO trigger level=8 */
#  define EZ80_UARTTRIG_14     0xc0         /*   11: Receive FIFO trigger level=14 */
                                            /* Bit 3-5: Reserved */
#define EZ80_UARTFCTL_CLRTxF   0x04         /* Bit 2: Transmit enable */
#define EZ80_UARTFCTL_CLRRxF   0x02         /* Bit 1: Receive enable */
#define EZ80_UARTFCTL_FIFOEN   0x01         /* Bit 0: Enable receive/transmit FIFOs */

/* UART0/1 LCTL register bits *******************************************************/

#define EZ80_UARTLCTL_DLAB     0x80         /* Bit 7: Enable access to baud rate generator */
#define EZ80_UARTLCTL_SB       0x40         /* Bit 6: Send break */
#define EZ80_UARTLCTL_FPE      0x20         /* Bit 5: Force parity error */
#define EZ80_UARTLCTL_EPS      0x10         /* Bit 4: Even parity select */
#define EZ80_UARTLCTL_PEN      0x08         /* Bit 3: Parity enable */
#define EZ80_UARTLCTl_2STOP    0x04         /* Bit 2: 2 stop bits */
#define EZ80_UARTLCTL_CHAR     0x03         /* Bits 0-2: Number of data bits */
#  define EZ80_UARTCHAR_5BITS  0x00         /*   00: 5 data bits */
#  define EZ80_UARTCHAR_6BITS  0x01         /*   01: 6 data bits */
#  define EZ80_UARTCHAR_7BITS  0x02         /*   10: 7 data bits */
#  define EZ80_UARTCHAR_8BITS  0x03         /*   11: 8 data bits */

/* UART0/1 MCTL register bits *******************************************************/

                                            /* Bit 7: Reserved */
#define EZ80_UARTMCTL_POLARITY 0x40         /* Bit 6: Invert polarity of RxD and TxD */
#define EZ80_UARTMCTL_MDM      0x20         /* Bit 5: Multi-drop mode enable */
#define EZ80_UARTMCTL_LOOP     0x10         /* Bit 4: Loopback mode enable */
#define EZ80_UARTMCTL_OUT2     0x08         /* Bit 3: (loopback mode only) */
#define EZ80_UARTMCTL_OUT1     0x04         /* Bit 2: (loopback mode only) */
#define EZ80_UARTMCTL_RTS      0x02         /* Bit 1: Request to send */
#define EZ80_UARTMCTL_DTR      0x01         /* Bit 0: Data termnal read */

/* UART0/1 LSR register bits ********************************************************/

#define EZ80_UARTLSR_ERR       0x80         /* Bit 7: Error detected in FIFO */
#define EZ80_UARTLSR_TEMT      0x40         /* Bit 6: Transmit FIFO empty and idle */
#define EZ80_UARTLSR_THRE      0x20         /* Bit 5: Transmit FIFO empty */
#define EZ80_UARTLSR_BI        0x10         /* Bit 4: Break on input */
#define EZ80_UARTLSR_FE        0x08         /* Bit 3: Framing error */
#define EZ80_UARTLSR_PE        0x04         /* Bit 2: Parity error */
#define EZ80_UARTLSR_OE        0x02         /* Bit 1: Overrun error */
#define EZ80_UARTLSR_DR        0x01         /* Bit 0: Data ready */

/* UART0/1 MSR register bits ********************************************************/

#define EZ80_UARTMSR_DCD       0x80         /* Bit 7: Data carrier detect */
#define EZ80_UARTMSR_RI        0x40         /* Bit 6: Ring indicator */
#define EZ80_UARTMSR_DSR       0x20         /* Bit 5: Data set ready */
#define EZ80_UARTMSR_CTS       0x10         /* Bit 4: Clear to send */
#define EZ80_UARTMSR_DDCD      0x08         /* Bit 3: Delta on DCD input */
#define EZ80_UARTMSR_TERI      0x04         /* Bit 2: Trailing edge change on RI */
#define EZ80_UARTMSR_DDSR      0x02         /* Bit 1: Delta on DSR input */
#define EZ80_UARTMSR_DCTS      0x01         /* Bit 0: Delta on CTS input */

/* Register access macros ***********************************************************/

#ifndef __ASSEMBLY__

# define getreg8(a)           (*(volatile ubyte *)(a))
# define putreg8(v,a)         (*(volatile ubyte *)(a) = (v))
# define getreg16(a)          (*(volatile uint16 *)(a))
# define putreg16(v,a)        (*(volatile uint16 *)(a) = (v))
# define getreg32(a)          (*(volatile uint32 *)(a))
# define putreg32(v,a)        (*(volatile uint32 *)(a) = (v))

#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

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

#endif  /* __EZ80_CHIP_H */
