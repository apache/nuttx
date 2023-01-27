/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_uart01.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_UART01_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_UART01_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_UART_UARTDR_OFFSET         0x000000  /* Data Register, UARTDR */
#define RP2040_UART_UARTRSR_OFFSET        0x000004  /* Receive Status Register/Error Clear Register, UARTRSR/UARTECR */
#define RP2040_UART_UARTFR_OFFSET         0x000018  /* Flag Register, UARTFR */
#define RP2040_UART_UARTILPR_OFFSET       0x000020  /* IrDA Low-Power Counter Register, UARTILPR */
#define RP2040_UART_UARTIBRD_OFFSET       0x000024  /* Integer Baud Rate Register, UARTIBRD */
#define RP2040_UART_UARTFBRD_OFFSET       0x000028  /* Fractional Baud Rate Register, UARTFBRD */
#define RP2040_UART_UARTLCR_H_OFFSET      0x00002c  /* Line Control Register, UARTLCR_H */
#define RP2040_UART_UARTCR_OFFSET         0x000030  /* Control Register, UARTCR */
#define RP2040_UART_UARTIFLS_OFFSET       0x000034  /* Interrupt FIFO Level Select Register, UARTIFLS */
#define RP2040_UART_UARTIMSC_OFFSET       0x000038  /* Interrupt Mask Set/Clear Register, UARTIMSC */
#define RP2040_UART_UARTRIS_OFFSET        0x00003c  /* Raw Interrupt Status Register, UARTRIS */
#define RP2040_UART_UARTMIS_OFFSET        0x000040  /* Masked Interrupt Status Register, UARTMIS */
#define RP2040_UART_UARTICR_OFFSET        0x000044  /* Interrupt Clear Register, UARTICR */
#define RP2040_UART_UARTDMACR_OFFSET      0x000048  /* DMA Control Register, UARTDMACR */
#define RP2040_UART_UARTPERIPHID0_OFFSET  0x000fe0  /* UARTPeriphID0 Register */
#define RP2040_UART_UARTPERIPHID1_OFFSET  0x000fe4  /* UARTPeriphID1 Register */
#define RP2040_UART_UARTPERIPHID2_OFFSET  0x000fe8  /* UARTPeriphID2 Register */
#define RP2040_UART_UARTPERIPHID3_OFFSET  0x000fec  /* UARTPeriphID3 Register */
#define RP2040_UART_UARTPCELLID0_OFFSET   0x000ff0  /* UARTPCellID0 Register */
#define RP2040_UART_UARTPCELLID1_OFFSET   0x000ff4  /* UARTPCellID1 Register */
#define RP2040_UART_UARTPCELLID2_OFFSET   0x000ff8  /* UARTPCellID2 Register */
#define RP2040_UART_UARTPCELLID3_OFFSET   0x000ffc  /* UARTPCellID3 Register */

/* Register definitions (UART) **********************************************/

#define RP2040_UART0_UARTDR         (RP2040_UART0_BASE + RP2040_UART_UARTDR_OFFSET)
#define RP2040_UART0_UARTRSR        (RP2040_UART0_BASE + RP2040_UART_UARTRSR_OFFSET)
#define RP2040_UART0_UARTFR         (RP2040_UART0_BASE + RP2040_UART_UARTFR_OFFSET)
#define RP2040_UART0_UARTILPR       (RP2040_UART0_BASE + RP2040_UART_UARTILPR_OFFSET)
#define RP2040_UART0_UARTIBRD       (RP2040_UART0_BASE + RP2040_UART_UARTIBRD_OFFSET)
#define RP2040_UART0_UARTFBRD       (RP2040_UART0_BASE + RP2040_UART_UARTFBRD_OFFSET)
#define RP2040_UART0_UARTLCR_H      (RP2040_UART0_BASE + RP2040_UART_UARTLCR_H_OFFSET)
#define RP2040_UART0_UARTCR         (RP2040_UART0_BASE + RP2040_UART_UARTCR_OFFSET)
#define RP2040_UART0_UARTIFLS       (RP2040_UART0_BASE + RP2040_UART_UARTIFLS_OFFSET)
#define RP2040_UART0_UARTIMSC       (RP2040_UART0_BASE + RP2040_UART_UARTIMSC_OFFSET)
#define RP2040_UART0_UARTRIS        (RP2040_UART0_BASE + RP2040_UART_UARTRIS_OFFSET)
#define RP2040_UART0_UARTMIS        (RP2040_UART0_BASE + RP2040_UART_UARTMIS_OFFSET)
#define RP2040_UART0_UARTICR        (RP2040_UART0_BASE + RP2040_UART_UARTICR_OFFSET)
#define RP2040_UART0_UARTDMACR      (RP2040_UART0_BASE + RP2040_UART_UARTDMACR_OFFSET)
#define RP2040_UART0_UARTPERIPHID0  (RP2040_UART0_BASE + RP2040_UART_UARTPERIPHID0_OFFSET)
#define RP2040_UART0_UARTPERIPHID1  (RP2040_UART0_BASE + RP2040_UART_UARTPERIPHID1_OFFSET)
#define RP2040_UART0_UARTPERIPHID2  (RP2040_UART0_BASE + RP2040_UART_UARTPERIPHID2_OFFSET)
#define RP2040_UART0_UARTPERIPHID3  (RP2040_UART0_BASE + RP2040_UART_UARTPERIPHID3_OFFSET)
#define RP2040_UART0_UARTPCELLID0   (RP2040_UART0_BASE + RP2040_UART_UARTPCELLID0_OFFSET)
#define RP2040_UART0_UARTPCELLID1   (RP2040_UART0_BASE + RP2040_UART_UARTPCELLID1_OFFSET)
#define RP2040_UART0_UARTPCELLID2   (RP2040_UART0_BASE + RP2040_UART_UARTPCELLID2_OFFSET)
#define RP2040_UART0_UARTPCELLID3   (RP2040_UART0_BASE + RP2040_UART_UARTPCELLID3_OFFSET)

/* Register definitions (UART1) *********************************************/

#define RP2040_UART1_UARTDR         (RP2040_UART1_BASE + RP2040_UART_UARTDR_OFFSET)
#define RP2040_UART1_UARTRSR        (RP2040_UART1_BASE + RP2040_UART_UARTRSR_OFFSET)
#define RP2040_UART1_UARTFR         (RP2040_UART1_BASE + RP2040_UART_UARTFR_OFFSET)
#define RP2040_UART1_UARTILPR       (RP2040_UART1_BASE + RP2040_UART_UARTILPR_OFFSET)
#define RP2040_UART1_UARTIBRD       (RP2040_UART1_BASE + RP2040_UART_UARTIBRD_OFFSET)
#define RP2040_UART1_UARTFBRD       (RP2040_UART1_BASE + RP2040_UART_UARTFBRD_OFFSET)
#define RP2040_UART1_UARTLCR_H      (RP2040_UART1_BASE + RP2040_UART_UARTLCR_H_OFFSET)
#define RP2040_UART1_UARTCR         (RP2040_UART1_BASE + RP2040_UART_UARTCR_OFFSET)
#define RP2040_UART1_UARTIFLS       (RP2040_UART1_BASE + RP2040_UART_UARTIFLS_OFFSET)
#define RP2040_UART1_UARTIMSC       (RP2040_UART1_BASE + RP2040_UART_UARTIMSC_OFFSET)
#define RP2040_UART1_UARTRIS        (RP2040_UART1_BASE + RP2040_UART_UARTRIS_OFFSET)
#define RP2040_UART1_UARTMIS        (RP2040_UART1_BASE + RP2040_UART_UARTMIS_OFFSET)
#define RP2040_UART1_UARTICR        (RP2040_UART1_BASE + RP2040_UART_UARTICR_OFFSET)
#define RP2040_UART1_UARTDMACR      (RP2040_UART1_BASE + RP2040_UART_UARTDMACR_OFFSET)
#define RP2040_UART1_UARTPERIPHID0  (RP2040_UART1_BASE + RP2040_UART_UARTPERIPHID0_OFFSET)
#define RP2040_UART1_UARTPERIPHID1  (RP2040_UART1_BASE + RP2040_UART_UARTPERIPHID1_OFFSET)
#define RP2040_UART1_UARTPERIPHID2  (RP2040_UART1_BASE + RP2040_UART_UARTPERIPHID2_OFFSET)
#define RP2040_UART1_UARTPERIPHID3  (RP2040_UART1_BASE + RP2040_UART_UARTPERIPHID3_OFFSET)
#define RP2040_UART1_UARTPCELLID0   (RP2040_UART1_BASE + RP2040_UART_UARTPCELLID0_OFFSET)
#define RP2040_UART1_UARTPCELLID1   (RP2040_UART1_BASE + RP2040_UART_UARTPCELLID1_OFFSET)
#define RP2040_UART1_UARTPCELLID2   (RP2040_UART1_BASE + RP2040_UART_UARTPCELLID2_OFFSET)
#define RP2040_UART1_UARTPCELLID3   (RP2040_UART1_BASE + RP2040_UART_UARTPCELLID3_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_UART_UARTDR_OE                         (1 << 11) /* Overrun error. This bit is set to 1 if data is received and the receive FIFO is already full. This is cleared to 0 once there is an empty space in the FIFO and a new character can be written to it. */
#define RP2040_UART_UARTDR_BE                         (1 << 10) /* Break error. This bit is set to 1 if a break condition was detected, indicating that the received data input was held LOW for longer than a full-word transmission time (defined as start, data, parity and stop bits). In FIFO mode, this error is associated with the character at the top of the FIFO. When a break occurs, only one 0 character is loaded into the FIFO. The next character is only enabled after the receive data input goes to a 1 (marking state), and the next valid start bit is received. */
#define RP2040_UART_UARTDR_PE                         (1 << 9)  /* Parity error. When set to 1, it indicates that the parity of the received data character does not match the parity that the EPS and SPS bits in the Line Control Register, UARTLCR_H. In FIFO mode, this error is associated with the character at the top of the FIFO. */
#define RP2040_UART_UARTDR_FE                         (1 << 8)  /* Framing error. When set to 1, it indicates that the received character did not have a valid stop bit (a valid stop bit is 1). In FIFO mode, this error is associated with the character at the top of the FIFO. */
#define RP2040_UART_UARTDR_DATA_MASK                  (0xff)    /* Receive (read) data character. Transmit (write) data character. */

#define RP2040_UART_UARTRSR_OE                        (1 << 3)  /* Overrun error. This bit is set to 1 if data is received and the FIFO is already full. This bit is cleared to 0 by a write to UARTECR. The FIFO contents remain valid because no more data is written when the FIFO is full, only the contents of the shift register are overwritten. The CPU must now read the data, to empty the FIFO. */
#define RP2040_UART_UARTRSR_BE                        (1 << 2)  /* Break error. This bit is set to 1 if a break condition was detected, indicating that the received data input was held LOW for longer than a full-word transmission time (defined as start, data, parity, and stop bits). This bit is cleared to 0 after a write to UARTECR. In FIFO mode, this error is associated with the character at the top of the FIFO. When a break occurs, only one 0 character is loaded into the FIFO. The next character is only enabled after the receive data input goes to a 1 (marking state) and the next valid start bit is received. */
#define RP2040_UART_UARTRSR_PE                        (1 << 1)  /* Parity error. When set to 1, it indicates that the parity of the received data character does not match the parity that the EPS and SPS bits in the Line Control Register, UARTLCR_H. This bit is cleared to 0 by a write to UARTECR. In FIFO mode, this error is associated with the character at the top of the FIFO. */
#define RP2040_UART_UARTRSR_FE                        (1 << 0)  /* Framing error. When set to 1, it indicates that the received character did not have a valid stop bit (a valid stop bit is 1). This bit is cleared to 0 by a write to UARTECR. In FIFO mode, this error is associated with the character at the top of the FIFO. */

#define RP2040_UART_UARTFR_RI                         (1 << 8)  /* Ring indicator. This bit is the complement of the UART ring indicator, nUARTRI, modem status input. That is, the bit is 1 when nUARTRI is LOW. */
#define RP2040_UART_UARTFR_TXFE                       (1 << 7)  /* Transmit FIFO empty. The meaning of this bit depends on the state of the FEN bit in the Line Control Register, UARTLCR_H. If the FIFO is disabled, this bit is set when the transmit holding register is empty. If the FIFO is enabled, the TXFE bit is set when the transmit FIFO is empty. This bit does not indicate if there is data in the transmit shift register. */
#define RP2040_UART_UARTFR_RXFF                       (1 << 6)  /* Receive FIFO full. The meaning of this bit depends on the state of the FEN bit in the UARTLCR_H Register. If the FIFO is disabled, this bit is set when the receive holding register is full. If the FIFO is enabled, the RXFF bit is set when the receive FIFO is full. */
#define RP2040_UART_UARTFR_TXFF                       (1 << 5)  /* Transmit FIFO full. The meaning of this bit depends on the state of the FEN bit in the UARTLCR_H Register. If the FIFO is disabled, this bit is set when the transmit holding register is full. If the FIFO is enabled, the TXFF bit is set when the transmit FIFO is full. */
#define RP2040_UART_UARTFR_RXFE                       (1 << 4)  /* Receive FIFO empty. The meaning of this bit depends on the state of the FEN bit in the UARTLCR_H Register. If the FIFO is disabled, this bit is set when the receive holding register is empty. If the FIFO is enabled, the RXFE bit is set when the receive FIFO is empty. */
#define RP2040_UART_UARTFR_BUSY                       (1 << 3)  /* UART busy. If this bit is set to 1, the UART is busy transmitting data. This bit remains set until the complete byte, including all the stop bits, has been sent from the shift register. This bit is set as soon as the transmit FIFO becomes non-empty, regardless of whether the UART is enabled or not. */
#define RP2040_UART_UARTFR_DCD                        (1 << 2)  /* Data carrier detect. This bit is the complement of the UART data carrier detect, nUARTDCD, modem status input. That is, the bit is 1 when nUARTDCD is LOW. */
#define RP2040_UART_UARTFR_DSR                        (1 << 1)  /* Data set ready. This bit is the complement of the UART data set ready, nUARTDSR, modem status input. That is, the bit is 1 when nUARTDSR is LOW. */
#define RP2040_UART_UARTFR_CTS                        (1 << 0)  /* Clear to send. This bit is the complement of the UART clear to send, nUARTCTS, modem status input. That is, the bit is 1 when nUARTCTS is LOW. */

#define RP2040_UART_UARTILPR_ILPDVSR_MASK             (0xff)    /* 8-bit low-power divisor value. These bits are cleared to 0 at reset. */

#define RP2040_UART_UARTIBRD_BAUD_DIVINT_MASK         (0xffff)  /* The integer baud rate divisor. These bits are cleared to 0 on reset. */

#define RP2040_UART_UARTFBRD_BAUD_DIVFRAC_MASK        (0x3f)    /* The fractional baud rate divisor. These bits are cleared to 0 on reset. */

#define RP2040_UART_UARTLCR_H_SPS                     (1 << 7)  /* Stick parity select. 0 = stick parity is disabled 1 = either: * if the EPS bit is 0 then the parity bit is transmitted and checked as a 1 * if the EPS bit is 1 then the parity bit is transmitted and checked as a 0. This bit has no effect when the PEN bit disables parity checking and generation. */
#define RP2040_UART_UARTLCR_H_WLEN_SHIFT              (5)       /* Word length. These bits indicate the number of data bits transmitted or received in a frame as follows: b11 = 8 bits b10 = 7 bits b01 = 6 bits b00 = 5 bits. */
#define RP2040_UART_UARTLCR_H_WLEN_MASK               (0x03 << RP2040_UART_UARTLCR_H_WLEN_SHIFT)
#define RP2040_UART_UARTLCR_H_FEN                     (1 << 4)  /* Enable FIFOs: 0 = FIFOs are disabled (character mode) that is, the FIFOs become 1-byte-deep holding registers 1 = transmit and receive FIFO buffers are enabled (FIFO mode). */
#define RP2040_UART_UARTLCR_H_STP2                    (1 << 3)  /* Two stop bits select. If this bit is set to 1, two stop bits are transmitted at the end of the frame. The receive logic does not check for two stop bits being received. */
#define RP2040_UART_UARTLCR_H_EPS                     (1 << 2)  /* Even parity select. Controls the type of parity the UART uses during transmission and reception: 0 = odd parity. The UART generates or checks for an odd number of 1s in the data and parity bits. 1 = even parity. The UART generates or checks for an even number of 1s in the data and parity bits. This bit has no effect when the PEN bit disables parity checking and generation. */
#define RP2040_UART_UARTLCR_H_PEN                     (1 << 1)  /* Parity enable: 0 = parity is disabled and no parity bit added to the data frame 1 = parity checking and generation is enabled. */
#define RP2040_UART_UARTLCR_H_BRK                     (1 << 0)  /* Send break. If this bit is set to 1, a low-level is continually output on the UARTTXD output, after completing transmission of the current character. For the proper execution of the break command, the software must set this bit for at least two complete frames. For normal use, this bit must be cleared to 0. */

#define RP2040_UART_LCR_H_WLEN(x)                     ((((x) - 5) << RP2040_UART_UARTLCR_H_WLEN_SHIFT) & RP2040_UART_UARTLCR_H_WLEN_MASK)

#define RP2040_UART_UARTCR_CTSEN                      (1 << 15) /* CTS hardware flow control enable. If this bit is set to 1, CTS hardware flow control is enabled. Data is only transmitted when the nUARTCTS signal is asserted. */
#define RP2040_UART_UARTCR_RTSEN                      (1 << 14) /* RTS hardware flow control enable. If this bit is set to 1, RTS hardware flow control is enabled. Data is only requested when there is space in the receive FIFO for it to be received. */
#define RP2040_UART_UARTCR_OUT2                       (1 << 13) /* This bit is the complement of the UART Out2 (nUARTOut2) modem status output. That is, when the bit is programmed to a 1, the output is 0. For DTE this can be used as Ring Indicator (RI). */
#define RP2040_UART_UARTCR_OUT1                       (1 << 12) /* This bit is the complement of the UART Out1 (nUARTOut1) modem status output. That is, when the bit is programmed to a 1 the output is 0. For DTE this can be used as Data Carrier Detect (DCD). */
#define RP2040_UART_UARTCR_RTS                        (1 << 11) /* Request to send. This bit is the complement of the UART request to send, nUARTRTS, modem status output. That is, when the bit is programmed to a 1 then nUARTRTS is LOW. */
#define RP2040_UART_UARTCR_DTR                        (1 << 10) /* Data transmit ready. This bit is the complement of the UART data transmit ready, nUARTDTR, modem status output. That is, when the bit is programmed to a 1 then nUARTDTR is LOW. */
#define RP2040_UART_UARTCR_RXE                        (1 << 9)  /* Receive enable. If this bit is set to 1, the receive section of the UART is enabled. Data reception occurs for either UART signals or SIR signals depending on the setting of the SIREN bit. When the UART is disabled in the middle of reception, it completes the current character before stopping. */
#define RP2040_UART_UARTCR_TXE                        (1 << 8)  /* Transmit enable. If this bit is set to 1, the transmit section of the UART is enabled. Data transmission occurs for either UART signals, or SIR signals depending on the setting of the SIREN bit. When the UART is disabled in the middle of transmission, it completes the current character before stopping. */
#define RP2040_UART_UARTCR_LBE                        (1 << 7)  /* Loopback enable. If this bit is set to 1 and the SIREN bit is set to 1 and the SIRTEST bit in the Test Control Register, UARTTCR is set to 1, then the nSIROUT path is inverted, and fed through to the SIRIN path. The SIRTEST bit in the test register must be set to 1 to override the normal half-duplex SIR operation. This must be the requirement for accessing the test registers during normal operation, and SIRTEST must be cleared to 0 when loopback testing is finished. This feature reduces the amount of external coupling required during system test. If this bit is set to 1, and the SIRTEST bit is set to 0, the UARTTXD path is fed through to the UARTRXD path. In either SIR mode or UART mode, when this bit is set, the modem outputs are also fed through to the modem inputs. This bit is cleared to 0 on reset, to disable loopback. */
#define RP2040_UART_UARTCR_SIRLP                      (1 << 2)  /* SIR low-power IrDA mode. This bit selects the IrDA encoding mode. If this bit is cleared to 0, low-level bits are transmitted as an active high pulse with a width of 3 / 16th of the bit period. If this bit is set to 1, low-level bits are transmitted with a pulse width that is 3 times the period of the IrLPBaud16 input signal, regardless of the selected bit rate. Setting this bit uses less power, but might reduce transmission distances. */
#define RP2040_UART_UARTCR_SIREN                      (1 << 1)  /* SIR enable: 0 = IrDA SIR ENDEC is disabled. nSIROUT remains LOW (no light pulse generated), and signal transitions on SIRIN have no effect. 1 = IrDA SIR ENDEC is enabled. Data is transmitted and received on nSIROUT and SIRIN. UARTTXD remains HIGH, in the marking state. Signal transitions on UARTRXD or modem status inputs have no effect. This bit has no effect if the UARTEN bit disables the UART. */
#define RP2040_UART_UARTCR_UARTEN                     (1 << 0)  /* UART enable: 0 = UART is disabled. If the UART is disabled in the middle of transmission or reception, it completes the current character before stopping. 1 = the UART is enabled. Data transmission and reception occurs for either UART signals or SIR signals depending on the setting of the SIREN bit. */

#define RP2040_UART_UARTIFLS_RXIFLSEL_SHIFT           (3)       /* Receive interrupt FIFO level select. The trigger points for the receive interrupt are as follows: b000 = Receive FIFO becomes >= 1 / 8 full b001 = Receive FIFO becomes >= 1 / 4 full b010 = Receive FIFO becomes >= 1 / 2 full b011 = Receive FIFO becomes >= 3 / 4 full b100 = Receive FIFO becomes >= 7 / 8 full b101-b111 = reserved. */
#define RP2040_UART_UARTIFLS_RXIFLSEL_MASK            (0x07 << RP2040_UART_UARTIFLS_RXIFLSEL_SHIFT)
#define RP2040_UART_UARTIFLS_TXIFLSEL_MASK            (0x07)    /* Transmit interrupt FIFO level select. The trigger points for the transmit interrupt are as follows: b000 = Transmit FIFO becomes <= 1 / 8 full b001 = Transmit FIFO becomes <= 1 / 4 full b010 = Transmit FIFO becomes <= 1 / 2 full b011 = Transmit FIFO becomes <= 3 / 4 full b100 = Transmit FIFO becomes <= 7 / 8 full b101-b111 = reserved. */

#define RP2040_UART_INTR_ALL                          (0x7ff)   /* All of interrupts */

#define RP2040_UART_UARTIMSC_OEIM                     (1 << 10) /* Overrun error interrupt mask. A read returns the current mask for the UARTOEINTR interrupt. On a write of 1, the mask of the UARTOEINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_BEIM                     (1 << 9)  /* Break error interrupt mask. A read returns the current mask for the UARTBEINTR interrupt. On a write of 1, the mask of the UARTBEINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_PEIM                     (1 << 8)  /* Parity error interrupt mask. A read returns the current mask for the UARTPEINTR interrupt. On a write of 1, the mask of the UARTPEINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_FEIM                     (1 << 7)  /* Framing error interrupt mask. A read returns the current mask for the UARTFEINTR interrupt. On a write of 1, the mask of the UARTFEINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_RTIM                     (1 << 6)  /* Receive timeout interrupt mask. A read returns the current mask for the UARTRTINTR interrupt. On a write of 1, the mask of the UARTRTINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_TXIM                     (1 << 5)  /* Transmit interrupt mask. A read returns the current mask for the UARTTXINTR interrupt. On a write of 1, the mask of the UARTTXINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_RXIM                     (1 << 4)  /* Receive interrupt mask. A read returns the current mask for the UARTRXINTR interrupt. On a write of 1, the mask of the UARTRXINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_DSRMIM                   (1 << 3)  /* nUARTDSR modem interrupt mask. A read returns the current mask for the UARTDSRINTR interrupt. On a write of 1, the mask of the UARTDSRINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_DCDMIM                   (1 << 2)  /* nUARTDCD modem interrupt mask. A read returns the current mask for the UARTDCDINTR interrupt. On a write of 1, the mask of the UARTDCDINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_CTSMIM                   (1 << 1)  /* nUARTCTS modem interrupt mask. A read returns the current mask for the UARTCTSINTR interrupt. On a write of 1, the mask of the UARTCTSINTR interrupt is set. A write of 0 clears the mask. */
#define RP2040_UART_UARTIMSC_RIMIM                    (1 << 0)  /* nUARTRI modem interrupt mask. A read returns the current mask for the UARTRIINTR interrupt. On a write of 1, the mask of the UARTRIINTR interrupt is set. A write of 0 clears the mask. */

#define RP2040_UART_UARTRIS_OERIS                     (1 << 10) /* Overrun error interrupt status. Returns the raw interrupt state of the UARTOEINTR interrupt. */
#define RP2040_UART_UARTRIS_BERIS                     (1 << 9)  /* Break error interrupt status. Returns the raw interrupt state of the UARTBEINTR interrupt. */
#define RP2040_UART_UARTRIS_PERIS                     (1 << 8)  /* Parity error interrupt status. Returns the raw interrupt state of the UARTPEINTR interrupt. */
#define RP2040_UART_UARTRIS_FERIS                     (1 << 7)  /* Framing error interrupt status. Returns the raw interrupt state of the UARTFEINTR interrupt. */
#define RP2040_UART_UARTRIS_RTRIS                     (1 << 6)  /* Receive timeout interrupt status. Returns the raw interrupt state of the UARTRTINTR interrupt. a */
#define RP2040_UART_UARTRIS_TXRIS                     (1 << 5)  /* Transmit interrupt status. Returns the raw interrupt state of the UARTTXINTR interrupt. */
#define RP2040_UART_UARTRIS_RXRIS                     (1 << 4)  /* Receive interrupt status. Returns the raw interrupt state of the UARTRXINTR interrupt. */
#define RP2040_UART_UARTRIS_DSRRMIS                   (1 << 3)  /* nUARTDSR modem interrupt status. Returns the raw interrupt state of the UARTDSRINTR interrupt. */
#define RP2040_UART_UARTRIS_DCDRMIS                   (1 << 2)  /* nUARTDCD modem interrupt status. Returns the raw interrupt state of the UARTDCDINTR interrupt. */
#define RP2040_UART_UARTRIS_CTSRMIS                   (1 << 1)  /* nUARTCTS modem interrupt status. Returns the raw interrupt state of the UARTCTSINTR interrupt. */
#define RP2040_UART_UARTRIS_RIRMIS                    (1 << 0)  /* nUARTRI modem interrupt status. Returns the raw interrupt state of the UARTRIINTR interrupt. */

#define RP2040_UART_UARTMIS_OEMIS                     (1 << 10) /* Overrun error masked interrupt status. Returns the masked interrupt state of the UARTOEINTR interrupt. */
#define RP2040_UART_UARTMIS_BEMIS                     (1 << 9)  /* Break error masked interrupt status. Returns the masked interrupt state of the UARTBEINTR interrupt. */
#define RP2040_UART_UARTMIS_PEMIS                     (1 << 8)  /* Parity error masked interrupt status. Returns the masked interrupt state of the UARTPEINTR interrupt. */
#define RP2040_UART_UARTMIS_FEMIS                     (1 << 7)  /* Framing error masked interrupt status. Returns the masked interrupt state of the UARTFEINTR interrupt. */
#define RP2040_UART_UARTMIS_RTMIS                     (1 << 6)  /* Receive timeout masked interrupt status. Returns the masked interrupt state of the UARTRTINTR interrupt. */
#define RP2040_UART_UARTMIS_TXMIS                     (1 << 5)  /* Transmit masked interrupt status. Returns the masked interrupt state of the UARTTXINTR interrupt. */
#define RP2040_UART_UARTMIS_RXMIS                     (1 << 4)  /* Receive masked interrupt status. Returns the masked interrupt state of the UARTRXINTR interrupt. */
#define RP2040_UART_UARTMIS_DSRMMIS                   (1 << 3)  /* nUARTDSR modem masked interrupt status. Returns the masked interrupt state of the UARTDSRINTR interrupt. */
#define RP2040_UART_UARTMIS_DCDMMIS                   (1 << 2)  /* nUARTDCD modem masked interrupt status. Returns the masked interrupt state of the UARTDCDINTR interrupt. */
#define RP2040_UART_UARTMIS_CTSMMIS                   (1 << 1)  /* nUARTCTS modem masked interrupt status. Returns the masked interrupt state of the UARTCTSINTR interrupt. */
#define RP2040_UART_UARTMIS_RIMMIS                    (1 << 0)  /* nUARTRI modem masked interrupt status. Returns the masked interrupt state of the UARTRIINTR interrupt. */

#define RP2040_UART_UARTICR_OEIC                      (1 << 10) /* Overrun error interrupt clear. Clears the UARTOEINTR interrupt. */
#define RP2040_UART_UARTICR_BEIC                      (1 << 9)  /* Break error interrupt clear. Clears the UARTBEINTR interrupt. */
#define RP2040_UART_UARTICR_PEIC                      (1 << 8)  /* Parity error interrupt clear. Clears the UARTPEINTR interrupt. */
#define RP2040_UART_UARTICR_FEIC                      (1 << 7)  /* Framing error interrupt clear. Clears the UARTFEINTR interrupt. */
#define RP2040_UART_UARTICR_RTIC                      (1 << 6)  /* Receive timeout interrupt clear. Clears the UARTRTINTR interrupt. */
#define RP2040_UART_UARTICR_TXIC                      (1 << 5)  /* Transmit interrupt clear. Clears the UARTTXINTR interrupt. */
#define RP2040_UART_UARTICR_RXIC                      (1 << 4)  /* Receive interrupt clear. Clears the UARTRXINTR interrupt. */
#define RP2040_UART_UARTICR_DSRMIC                    (1 << 3)  /* nUARTDSR modem interrupt clear. Clears the UARTDSRINTR interrupt. */
#define RP2040_UART_UARTICR_DCDMIC                    (1 << 2)  /* nUARTDCD modem interrupt clear. Clears the UARTDCDINTR interrupt. */
#define RP2040_UART_UARTICR_CTSMIC                    (1 << 1)  /* nUARTCTS modem interrupt clear. Clears the UARTCTSINTR interrupt. */
#define RP2040_UART_UARTICR_RIMIC                     (1 << 0)  /* nUARTRI modem interrupt clear. Clears the UARTRIINTR interrupt. */

#define RP2040_UART_UARTDMACR_DMAONERR                (1 << 2)  /* DMA on error. If this bit is set to 1, the DMA receive request outputs, UARTRXDMASREQ or UARTRXDMABREQ, are disabled when the UART error interrupt is asserted. */
#define RP2040_UART_UARTDMACR_TXDMAE                  (1 << 1)  /* Transmit DMA enable. If this bit is set to 1, DMA for the transmit FIFO is enabled. */
#define RP2040_UART_UARTDMACR_RXDMAE                  (1 << 0)  /* Receive DMA enable. If this bit is set to 1, DMA for the receive FIFO is enabled. */

#define RP2040_UART_UARTPERIPHID0_PARTNUMBER0_MASK    (0xff)    /* These bits read back as 0x11 */

#define RP2040_UART_UARTPERIPHID1_DESIGNER0_SHIFT     (4)       /* These bits read back as 0x1 */
#define RP2040_UART_UARTPERIPHID1_DESIGNER0_MASK      (0x0f << RP2040_UART_UARTPERIPHID1_DESIGNER0_SHIFT)
#define RP2040_UART_UARTPERIPHID1_PARTNUMBER1_MASK    (0x0f)    /* These bits read back as 0x0 */

#define RP2040_UART_UARTPERIPHID2_REVISION_SHIFT      (4)     /* This field depends on the revision of the UART: r1p0 0x0 r1p1 0x1 r1p3 0x2 r1p4 0x2 r1p5 0x3 */
#define RP2040_UART_UARTPERIPHID2_REVISION_MASK       (0x0f << RP2040_UART_UARTPERIPHID2_REVISION_SHIFT)
#define RP2040_UART_UARTPERIPHID2_DESIGNER1_MASK      (0x0f)  /* These bits read back as 0x4 */

#define RP2040_UART_UARTPERIPHID3_CONFIGURATION_MASK  (0xff)  /* These bits read back as 0x00 */

#define RP2040_UART_UARTPCELLID0_MASK                 (0xff)  /* These bits read back as 0x0D */

#define RP2040_UART_UARTPCELLID1_MASK                 (0xff)  /* These bits read back as 0xF0 */

#define RP2040_UART_UARTPCELLID2_MASK                 (0xff)  /* These bits read back as 0x05 */

#define RP2040_UART_UARTPCELLID3_MASK                 (0xff)  /* These bits read back as 0xB1 */

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_UART01_H */
