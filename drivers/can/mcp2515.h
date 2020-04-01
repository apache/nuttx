/****************************************************************************
 * drivers/can/mcp2515.c
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 * 3. Neither the name NuttX, Atmel, nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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
 ****************************************************************************/

#ifndef __DRIVERS_CAN_MCP2514_H
#define __DRIVERS_CAN_MCP2514_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses */

#define MCP2515_RXF0SIDH     0x00
#define MCP2515_RXF0SIDL     0x01
#define MCP2515_RXF0EID8     0x02
#define MCP2515_RXF0EID0     0x03
#define MCP2515_RXF1SIDH     0x04
#define MCP2515_RXF1SIDL     0x05
#define MCP2515_RXF1EID8     0x06
#define MCP2515_RXF1EID0     0x07
#define MCP2515_RXF2SIDH     0x08
#define MCP2515_RXF2SIDL     0x09
#define MCP2515_RXF2EID8     0x0a
#define MCP2515_RXF2EID0     0x0b
#define MCP2515_BFPCTRL      0x0c
#define MCP2515_TXRTSCTRL    0x0d
#define MCP2515_CANSTAT      0x0e
#define MCP2515_CANCTRL      0x0f
#define MCP2515_RXF3SIDH     0x10
#define MCP2515_RXF3SIDL     0x11
#define MCP2515_RXF3EID8     0x12
#define MCP2515_RXF3EID0     0x13
#define MCP2515_RXF4SIDH     0x14
#define MCP2515_RXF4SIDL     0x15
#define MCP2515_RXF4EID8     0x16
#define MCP2515_RXF4EID0     0x17
#define MCP2515_RXF5SIDH     0x18
#define MCP2515_RXF5SIDL     0x19
#define MCP2515_RXF5EID8     0x1a
#define MCP2515_RXF5EID0     0x1b
#define MCP2515_TEC          0x1c
#define MCP2515_REC          0x1d
#define MCP2515_RXM0SIDH     0x20
#define MCP2515_RXM0SIDL     0x21
#define MCP2515_RXM0EID8     0x22
#define MCP2515_RXM0EID0     0x23
#define MCP2515_RXM1SIDH     0x24
#define MCP2515_RXM1SIDL     0x25
#define MCP2515_RXM1EID8     0x26
#define MCP2515_RXM1EID0     0x27
#define MCP2515_CNF3         0x28
#define MCP2515_CNF2         0x29
#define MCP2515_CNF1         0x2a
#define MCP2515_CANINTE      0x2b
#define MCP2515_CANINTF      0x2c
#define MCP2515_EFLG         0x2d
#define MCP2515_TXB0CTRL     0x30
#define MCP2515_TXB0SIDH     0x31
#define MCP2515_TXB0SIDL     0x32
#define MCP2515_TXB0EID8     0x33
#define MCP2515_TXB0EID0     0x34
#define MCP2515_TXB0DLC      0x35
#define MCP2515_TXB0D0       0x36
#define MCP2515_TXB0D1       0x37
#define MCP2515_TXB0D2       0x38
#define MCP2515_TXB0D3       0x39
#define MCP2515_TXB0D4       0x3a
#define MCP2515_TXB0D5       0x3b
#define MCP2515_TXB0D6       0x3c
#define MCP2515_TXB0D7       0x3d
#define MCP2515_TXB1CTRL     0x40
#define MCP2515_TXB1SIDH     0x41
#define MCP2515_TXB1SIDL     0x42
#define MCP2515_TXB1EID8     0x43
#define MCP2515_TXB1EID0     0x44
#define MCP2515_TXB1DLC      0x45
#define MCP2515_TXB1D0       0x46
#define MCP2515_TXB1D1       0x47
#define MCP2515_TXB1D2       0x48
#define MCP2515_TXB1D3       0x49
#define MCP2515_TXB1D4       0x4a
#define MCP2515_TXB1D5       0x4b
#define MCP2515_TXB1D6       0x4c
#define MCP2515_TXB1D7       0x4d
#define MCP2515_TXB2CTRL     0x50
#define MCP2515_TXB2SIDH     0x51
#define MCP2515_TXB2SIDL     0x52
#define MCP2515_TXB2EID8     0x53
#define MCP2515_TXB2EID0     0x54
#define MCP2515_TXB2DLC      0x55
#define MCP2515_TXB2D0       0x56
#define MCP2515_TXB2D1       0x57
#define MCP2515_TXB2D2       0x58
#define MCP2515_TXB2D3       0x59
#define MCP2515_TXB2D4       0x5a
#define MCP2515_TXB2D5       0x5b
#define MCP2515_TXB2D6       0x5c
#define MCP2515_TXB2D7       0x5d
#define MCP2515_RXB0CTRL     0x60
#define MCP2515_RXB0SIDH     0x61
#define MCP2515_RXB0SIDL     0x62
#define MCP2515_RXB0EID8     0x63
#define MCP2515_RXB0EID0     0x64
#define MCP2515_RXB0DLC      0x65
#define MCP2515_RXB0D0       0x66
#define MCP2515_RXB0D1       0x67
#define MCP2515_RXB0D2       0x68
#define MCP2515_RXB0D3       0x69
#define MCP2515_RXB0D4       0x6a
#define MCP2515_RXB0D5       0x6b
#define MCP2515_RXB0D6       0x6c
#define MCP2515_RXB0D7       0x6d
#define MCP2515_RXB1CTRL     0x70
#define MCP2515_RXB1SIDH     0x71
#define MCP2515_RXB1SIDL     0x72
#define MCP2515_RXB1EID8     0x73
#define MCP2515_RXB1EID0     0x74
#define MCP2515_RXB1DLC      0x75
#define MCP2515_RXB1D0       0x76
#define MCP2515_RXB1D1       0x77
#define MCP2515_RXB1D2       0x78
#define MCP2515_RXB1D3       0x79
#define MCP2515_RXB1D4       0x7a
#define MCP2515_RXB1D5       0x7b
#define MCP2515_RXB1D6       0x7c
#define MCP2515_RXB1D7       0x7d

/* Offset to simplify mcp2515_receive() function */

#define MCP2515_RX0_OFFSET   0x00
#define MCP2515_RX1_OFFSET   0x10

/* Offset to simplify mcp2515_send() function */

#define MCP2515_TX0_OFFSET   0x00
#define MCP2515_TX1_OFFSET   0x10
#define MCP2515_TX2_OFFSET   0x20

/* CANCTRL: CAN CONTROL REGISTER */

#define CANCTRL_CLKPRE_SHIFT (0)      /* Bits 0-1: CLKOUT Pin Prescaler bits */
#define CANCTRL_CLKPRE_MASK  (3 << CANCTRL_CLKPRE_SHIFT)
#define CANCTRL_CLKEN        (1 << 2) /* Bit 2: CLKOUT Pin Enable bit */
#define CANCTRL_OSM          (1 << 3) /* Bit 3: One-Shot Mode bit */
#define CANCTRL_ABAT         (1 << 4) /* Bit 4: Abort All Pending Transmissions bit */
#define CANCTRL_REQOP_SHIFT  (5)      /* Bits 5-7: Request Operation Mode bits */
#define CANCTRL_REQOP_MASK   (7 << CANCTRL_REQOP_SHIFT)
#define CANCTRL_REQOP_NORMAL (0 << CANCTRL_REQOP_SHIFT)
#define CANCTRL_REQOP_SLEEP  (1 << CANCTRL_REQOP_SHIFT)
#define CANCTRL_REQOP_LOOPBK (2 << CANCTRL_REQOP_SHIFT)
#define CANCTRL_REQOP_LISTEN (3 << CANCTRL_REQOP_SHIFT)
#define CANCTRL_REQOP_CONFIG (4 << CANCTRL_REQOP_SHIFT)

/* TXBnCTRL – TRANSMIT BUFFER n CONTROL REGISTER */

#define TXBCTRL_TXP_SHIFT    (0)      /* Bits 0-1: Transmit Buffer Priority */
#define TXBCTRL_TXP_MASK     (3 << MCP2515_TXBCTRL_TXP_SHIFT)
                                      /* Bit 2: Not used */
#define TXBCTRL_TXREQ        (1 << 3) /* Bit 3: Message Transmit Request bit */
#define TXBCTRL_TXERR        (1 << 4) /* Bit 4: Transmission Error Detected bit */
#define TXBCTRL_MLOA         (1 << 5) /* Bit 5: Message Lost Arbitration bit */
#define TXBCTRL_ABTF         (1 << 6) /* Bit 6: Message Aborted Flag bit */
                                      /* Bit 7: Not used */

/* TXRTSCTRL – TXnRTS PIN CONTROL AND STATUS REGISTER */

#define TXRTSCTRL_B0RTSM     (1 << 0) /* Bit 0: TX0RTS Pin mode bit */
#define TXRTSCTRL_B1RTSM     (1 << 1) /* Bit 1: TX1RTS Pin mode bit */
#define TXRTSCTRL_B2RTSM     (1 << 2) /* Bit 2: TX2RTS Pin mode bit */
#define TXRTSCTRL_B0RTS      (1 << 3) /* Bit 3: TX0RTS Pin State bit */
#define TXRTSCTRL_B1RTS      (1 << 4) /* Bit 4: TX1RTS Pin State bit */
#define TXRTSCTRL_B2RTS      (1 << 5) /* Bit 5: TX2RTS Pin State bit */
                                      /* Bit 6-7: Not used */

/* TXBnSIDH – TRANSMIT BUFFER n STANDARD IDENTIFIER HIGH */

#define TXBSIDH_SID_MASK     0xff     /* Standard Identifier bits <10:3> */

/* TXBnSIDL – TRANSMIT BUFFER n STANDARD IDENTIFIER LOW */

#define TXBSIDL_SID_SHIFT    (5)      /* Bits 5-7: Standard Identifier bits <2:0> */
#define TXBSIDL_SID_MASK     (0x7 << TXBSIDL_SID_SHIFT)
#define TXBSIDL_EXIDE        (1 << 3) /* Bit    3: Extended Identifier Enable bit */
#define TXBSIDL_EID_SHIFT    (0)      /* Bits 0-1: Extended Identifier bits <17:16> */
#define TXBSIDL_EID_MASK     (0x03 << TXBSIDL_EID_MASK)

/* TXBnEID8 – TRANSMIT BUFFER n EXTENDED IDENTIFIER HIGH */

#define TXBEID8_EID_MASK     0xff     /* Bits 0-7: Extended Identifier bits <15:8> */

/* TXBnEID0 – TRANSMIT BUFFER n EXTENDED IDENTIFIER LOW */

#define TXBEID0_EID_MASK     0xff     /* Bits 0-7: Extended Identifier bits <7:0> */

/* TXBnDLC - TRANSMIT BUFFER n DATA LENGTH CODE */

#define TXBDLC_DLC_SHIFT     (0)      /* Bits 0-3: Data Length Code <3:0> bits */
#define TXBDLC_DLC_MASK      (0xf << TXBDLC_DLC_SHIFT)
#define TXBDLC_RTR           (1 << 6) /* Bit 6: Remote Transmission Request bit */

/* TXBnDm – TRANSMIT BUFFER n DATA BYTE m */

#define TXBD_D0              (1 << 0) /* Bit 0: Transmit Buffer n Data Field Bytes 0 */
#define TXBD_D1              (1 << 1) /* Bit 1: Transmit Buffer n Data Field Bytes 1 */
#define TXBD_D2              (1 << 2) /* Bit 2: Transmit Buffer n Data Field Bytes 2 */
#define TXBD_D3              (1 << 3) /* Bit 3: Transmit Buffer n Data Field Bytes 3 */
#define TXBD_D4              (1 << 4) /* Bit 4: Transmit Buffer n Data Field Bytes 4 */
#define TXBD_D5              (1 << 5) /* Bit 5: Transmit Buffer n Data Field Bytes 5 */
#define TXBD_D6              (1 << 6) /* Bit 6: Transmit Buffer n Data Field Bytes 6 */
#define TXBD_D7              (1 << 7) /* Bit 7: Transmit Buffer n Data Field Bytes 7 */

/* RXB0CTRL – RECEIVE BUFFER 0 CONTROL */

#define RXB0CTRL_FILHIT      (1 << 0) /* Bit 0: Filter Hit bit - 1 = Msg was accepted by Filter 1; 0 = Filter 0 */
#define RXB0CTRL_BUKT1       (1 << 1) /* Bit 1: Read-only Copy of BUKT bit (used internally by the MCP2515) */
#define RXB0CTRL_BUKT        (1 << 2) /* Bit 2: Rollover Enable bit */

/* These bits are common to RXB0 and RXB1: */

#define RXBCTRL_RXRTR        (1 << 3)                   /* Bit 3: Received Remote Transfer Request bit */
                                                        /* Bit 4: Not used */
#define RXBCTRL_RXM_SHIFT    (5)                        /* Bits 5-6: Receive Buffer Operating Mode bits */
#define RXBCTRL_RXM_MASK     (0x3 << RXBCTRL_RXM_SHIFT)
#define RXBCTRL_RXM_ALLMSG   (3 << RXBCTRL_RXM_SHIFT)   /* 11: Turn mask/filters off; receive any message */
#define RXBCTRL_RXM_ALLVALID (0 << RXBCTRL_RXM_SHIFT)   /* 00: Receive all valid msgs using (STD or EXT) that meet filter criteria */
                                                        /* Bit 7: Not used */

/* N.B.: In the datasheet DS21801D the file RXM of RXBnCTRL could to assume
 *       the value 01 and 10 to receive only STD or EXT msgs respectively.
 *       But in a more recent datasheet DS20001801H it was removed.
 */

/* RXB1CTRL – RECEIVE BUFFER 1 CONTROL */

#define RXB1CTRL_FILHIT_SHIFT (0)                           /* Filter Hit bits - indicates which acceptance filter enabled reception of message */
#define RXB1CTRL_FILHIT_MASK  (0x7 << RXB0CTRL_FILHIT_SHIFT)
#define RXB1CTRL_FILHIT_F5    (5 << RXB1CTRL_FILHIT_SHIFT)  /* Acceptance Filter 5 (RXF5) */
#define RXB1CTRL_FILHIT_F4    (4 << RXB1CTRL_FILHIT_SHIFT)  /* Acceptance Filter 4 (RXF4) */
#define RXB1CTRL_FILHIT_F3    (3 << RXB1CTRL_FILHIT_SHIFT)  /* Acceptance Filter 3 (RXF3) */
#define RXB1CTRL_FILHIT_F2    (2 << RXB1CTRL_FILHIT_SHIFT)  /* Acceptance Filter 2 (RXF2) */
#define RXB1CTRL_FILHIT_F1    (1 << RXB1CTRL_FILHIT_SHIFT)  /* Acceptance Filter 1 (RXF1)  (Only if BUKT bit set in RXB0CTRL) */
#define RXB1CTRL_FILHIT_F0    (0 << RXB1CTRL_FILHIT_SHIFT)  /* Acceptance Filter 0 (RXF0)  (Only if BUKT bit set in RXB0CTRL) */

/* BFPCTRL – RXnBF PIN CONTROL AND STATUS */

#define BFPCTRL_B0BFM        (1 << 0) /* Bit 0: RX0BF Pin Operation Mode bit */
#define BFPCTRL_B1BFM        (1 << 1) /* Bit 1: RX1BF Pin Operation Mode bit */
#define BFPCTRL_B0BFE        (1 << 2) /* Bit 2: RX0BF Pin Function Enable bit */
#define BFPCTRL_B1BFE        (1 << 3) /* Bit 3: RX1BF Pin Function Enable bit */
#define BFPCTRL_B0BFS        (1 << 4) /* Bit 4: RX0BF Pin State bit (Digital Output mode only) */
#define BFPCTRL_B1BFS        (1 << 5) /* Bit 5: RX1BF Pin State bit (Digital Output mode only) */
                                      /* Bits 6-7: Not used */

/* RXBnSIDH – RECEIVE BUFFER n STANDARD IDENTIFIER HIGH */

#define RXBSIDH_SID_MASK     0xff     /* Standard Identifier bits <10:3> */

/* RXBnSIDL – RECEIVE BUFFER n STANDARD IDENTIFIER LOW */

#define RXBSIDL_SID_SHIFT    (5)      /* Bits 5-7: Standard Identifier bits <2:0> */
#define RXBSIDL_SID_MASK     (0x7 << RXBSIDL_SID_SHIFT)
#define RXBSIDL_SRR          (1 << 4) /* Bit 4: Standard Frame Remote Transmit Request bit (valid only if IDE bit = '0')*/
#define RXBSIDL_IDE          (1 << 3) /* Bit 3: Extended Identifier Message received */
                                      /* Bit 2: Not used */
#define RXBSIDL_EID_SHIFT    (0)      /* Bits 0-1: Extended Identifier bits <17:16> */
#define RXBSIDL_EID_MASK     (0x03 << RXBSIDL_EID_SHIFT)

/* RXBnEID8 – RECEIVE BUFFER n EXTENDED IDENTIFIER HIGH */

#define RXBEID8_EID_MASK     0xff     /* Bits 0-7: Extended Identifier bits <15:8> */

/* RXBnEID0 – RECEIVE BUFFER n EXTENDED IDENTIFIER LOW */

#define RXBEID0_EID_MASK     0xff     /* Bits 0-7: Extended Identifier bits <7:0> */

/* RXBnDLC – RECEIVE BUFFER n DATA LENGTH CODE */

#define RXBDLC_DLC_SHIFT     (0)      /* Bits 0-3: Data Length Code <3:0> bits */
#define RXBDLC_DLC_MASK      (0xf << RXBDLC_DLC_SHIFT)
#define RXBDLC_RB0           (1 << 4) /* Bit 4: Reserved bit 0 */
#define RXBDLC_RB1           (1 << 5) /* Bit 5: Reserved bit 1 */
#define RXBDLC_RTR           (1 << 6) /* Bit 6: Remote Transmission Request bit */
                                      /* Bit 7: Not used */

/* RXFnSIDH – FILTER n STANDARD IDENTIFIER HIGH */

#define RXFSIDH_SID_MASK     0xff     /* Standard Identifier Filter bits <10:3> */

/* RXFnSIDL – FILTER n STANDARD IDENTIFIER LOW */

#define RXFSIDL_EID_SHIFT    (0)      /* Bit 0-1: Extended Identifier Filter bits <17:16> */
#define RXFSIDL_EID_MASK     (3 << RXFSIDL_EID_SHIFT)
#define RXFSIDL_EXIDE        (1 << 3) /* Bit 3: Extended Identifier Enable bit */
#define RXFSIDL_SID_SHIFT    (5)      /* Bits 5-7: Standard Identifier Filter bits <2:0> */
#define RXFSIDL_SID_MASK     (0x7 << RXFSIDL_SID_SHIFT)

/* RXFnEID8 – FILTER n EXTENDED IDENTIFIER HIGH */

#define RXFEID8_EID_MASK     0xff     /* Extended Identifier bits <15:8> */

/* RXFnEID0 – FILTER n EXTENDED IDENTIFIER LOW */

#define RXFEID0_EID_MASK     0xff     /* Extended Identifier bits <7:0> */

/* RXMnSIDH – MASK n STANDARD IDENTIFIER HIGH */

#define RXMSIDH_SID_MASK     0xff     /* Standard Identifier Mask bits <10:3> */

/* RXMnSIDL – MASK n STANDARD IDENTIFIER LOW */

#define RXMSIDL_EID_SHIFT    (0)      /* Bits 0-1: Extended Identifier Mask bits <17:16> */
#define RXMSIDL_EID_MASK     (3 << RXMSIDH_EID_SHIFT)
#define RXMSIDL_SID_SHIFT    (5)      /* Bits 5-7: Standard Identifier Mask bits <2:0> */
#define RXMSIDL_MASK         (7 << RXMSIDH_SID_SHIFT)

/* RXMnEID8 – MASK n EXTENDED IDENTIFIER HIGH */

#define RXMEID8_EID_MASK     0xff     /* Extended Identifier bits <15:8> */

/* RXMnEID0 – MASK n EXTENDED IDENTIFIER LOW */

#define RXMEID0_EID_MASK     0xff     /* Extended Identifier Mask bits <7:0> */

/* CNF1 – CONFIGURATION 1 */

#define CNF1_BRP_SHIFT       (0)                    /* Bits 0-5: Baud Rate Prescaler bits <5:0>, TQ = 2 x (BRP + 1)/Fosc */
#define CNF1_BRP_MASK        (0x3f << CNF1_BRP_SHIFT)
#define CNF1_SJW_SHIFT       (6)                    /* Bit 6-7: Synchronization Jump Width Length bits <1:0> */
#define CNF1_SJW_MASK        (3 << CNF1_SJW_SHIFT)
#  define CNF1_SJW_4xTQ      (3 << CNF1_SJW_SHIFT)  /* Length = 4 x TQ */
#  define CNF1_SJW_3xTQ      (2 << CNF1_SJW_SHIFT)  /* Length = 3 x TQ */
#  define CNF1_SJW_2xTQ      (1 << CNF1_SJW_SHIFT)  /* Length = 2 x TQ */
#  define CNF1_SJW_1xTQ      (0 << CNF1_SJW_SHIFT)  /* Length = 1 x TQ */

/* CNF2 – CONFIGURATION 2 */

#define CNF2_PRSEG_SHIFT     (0)      /* Bits 0-2: Propagation Segment Length bits <2:0>, (PRSEG + 1) x TQ */
#define CNF2_PRSEG_MASK      (7 << CNF2_PRSEG_SHIFT)
#define CNF2_PHSEG1_SHIFT    (3)      /* Bits 3-5: PS1 Length bits <2:0>, (PHSEG1 + 1) x TQ */
#define CNF2_PHSEG1_MASK     (7 << CNF2_PHSEG1_SHIFT)
#define CNF2_SAM             (1 << 6) /* Bit 6: Sample Point Configuration bit */
#define CNF2_BTLMODE         (1 << 7) /* Bit 7: PS2 Bit Time Length bit */

/* CNF3 - CONFIGURATION 3 */

#define CNF3_PHSEG2_SHIFT    (0)      /* Bits 0-2: PS2 Length bits<2:0>, (PHSEG2 + 1) x TQ */
#define CNF3_PHSEG2_MASK     (7 << CNF3_PHSEG2_SHIFT)
#define CNF3_WAKFIL          (1 << 6) /* Bit 3: Wake-up Filter bit */
#define CNF3_SOF             (1 << 7) /* Bit 7: Start-of-Frame signal bit */

/* TEC – TRANSMIT ERROR COUNTER */

#define TEC_MASK             0xff     /* Transmit Error Count bits <7:0> */

/* REC – RECEIVER ERROR COUNTER */

#define REC_MASK             0xff     /* Receive Error Count bits <7:0> */

/* EFLG – ERROR FLAG */

#define EFLG_EWARN           (1 << 0) /* Bit 0: Error Warning Flag bit */
#define EFLG_RXWAR           (1 << 1) /* Bit 1: Receive Error Warning Flag bit */
#define EFLG_TXWAR           (1 << 2) /* Bit 2: Transmit Error Warning Flag bit */
#define EFLG_RXEP            (1 << 3) /* Bit 3: Receive Error-Passive Flag bit */
#define EFLG_TXEP            (1 << 4) /* Bit 4: Transmit Error-Passive Flag bit */
#define EFLG_TXBO            (1 << 5) /* Bit 5: Bus-Off Error Flag bit */
#define EFLG_RX0OVR          (1 << 6) /* Bit 6: Receive Buffer 0 Overflow Flag bit */
#define EFLG_RX1OVR          (1 << 7) /* Bit 7: Receive Buffer 1 Overflow Flag bit */

/* CANINTE/CANINTF – INTERRUPT ENABLE/FLAG */

#define MCP2515_INT_RX0      (1 << 0) /* Bit 0: Receive Buffer 0 Full Interrupt Enable bit */
#define MCP2515_INT_RX1      (1 << 1) /* Bit 1: Receive Buffer 1 Full Interrupt Enable bit */
#define MCP2515_INT_TX0      (1 << 2) /* Bit 2: Transmit Buffer 0 Empty Interrupt Enable bit */
#define MCP2515_INT_TX1      (1 << 3) /* Bit 3: Transmit Buffer 1 Empty Interrupt Enable bit */
#define MCP2515_INT_TX2      (1 << 4) /* Bit 4: Transmit Buffer 2 Empty Interrupt Enable bit */
#define MCP2515_INT_ERR      (1 << 5) /* Bit 5: Error Interrupt Enable bit (multiple sources in EFLG register) */
#define MCP2515_INT_WAK      (1 << 6) /* Bit 6: Wakeup Interrupt Enable bit */
#define MCP2515_INT_MERR     (1 << 7) /* Bit 7: Message Error Interrupt Enable bit */

/* MCP2515 SPI Instruction/Command byte */

#define MCP2515_RESET        0xC0
#define MCP2515_READ         0x03
#define MCP2515_READ_RX0     0x90
#define MCP2515_READ_RX1     0x94
#define MCP2515_WRITE        0x02
#define MCP2515_LOAD_TX0     0x40
#define MCP2515_LOAD_TX1     0x42
#define MCP2515_LOAD_TX2     0x44
#define MCP2515_LOAD_TXB(n)  (0x40 + 2 * (n))
#define MCP2515_READ_RXB(n)  (((n) == 0) ? 0x90 : 0x94)
#define MCP2515_RTS_TX0      0x81
#define MCP2515_RTS_TX1      0x82
#define MCP2515_RTS_TX2      0x84
#define MCP2515_RTS(x)       (0x81+x)
#define MCP2515_RTS_ALL      0x87
#define MCP2515_READ_STATUS  0xA0
#define MCP2515_RX_STATUS    0xB0
#define MCP2515_BITMOD       0x05

/* CANCTRL register will be 0x87 after reset and in Conf. Mode */

#define DEFAULT_CANCTRL_CONFMODE 0x87

/* Crystal Frequency used on MCP2515 board */

#define MCP2515_CANCLK_FREQUENCY CONFIG_MCP2515_CLK_FREQUENCY

#endif /* __DRIVERS_CAN_MCP2514_H */
