/**************************************************************************//**
 * @file      rtl8710c_uart_type.h
 * @brief
 * @version   V1.00
 * @date      2018-1-4 15:41:32
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 * 
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 ******************************************************************************/

#ifndef _RTL8710C_UART_TYPE_H_
#define _RTL8710C_UART_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_UART_REG_TYPE

/**
 * @addtogroup hs_hal_uart_reg UART Registers.
 * @ingroup hs_hal_uart
 * @{
 */

/**
  \brief Union type to access uart_dll (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) Divisor Latch (LS) Register                                */
  
  struct {
    __IOM uint32_t dll        : 8;            /*!< [7..0] Baud rate divisor-low register, accessable only when
                                                   DLAB = 1                                                                  */
  } b;                                        /*!< bit fields for uart_dll */
} uart_dll_t, *puart_dll_t;

/**
  \brief Union type to access uart_dlm (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x00000004) Divisor Latch (MS) Register                                */
  
  struct {
    __IOM uint32_t dlm      : 8;            /*!< [7..0] Baud rate divisor-high register, accessable only when
                                                 DLAB = 1                                                                  */
  } b;                                        /*!< bit fields for uart_dlm */
} uart_dlm_t, *puart_dlm_t;

/**
  \brief Union type to access uart_ier (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x00000004) Interrupt Enable Register                                  */
  
  struct {
    __IOM uint32_t erbi     : 1;            /*!< [0..0] Enable Received Data Available Interrupt (ERBFI) (rx
                                                 trigger)                                                                  */
    __IOM uint32_t etbei    : 1;            /*!< [1..1] Enable Transmitter FIFO Empty interrupt (ETBEI) (tx fifo
                                                 empty)                                                                    */
    __IOM uint32_t elsi     : 1;            /*!< [2..2] Enable Receiver Line Status Interrupt (ELSI) (receiver
                                                 line status)                                                              */
    __IOM uint32_t edssi    : 1;            /*!< [3..3] Enable Modem Status Interrupt (EDSSI) (modem status transition)    */
  } b;                                        /*!< bit fields for uart_ier */
} uart_ier_t, *puart_ier_t;

/**
  \brief Union type to access uart_iir (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x00000008) Interrupt Identification Register                          */
  
  struct {
    __IM  uint32_t int_pend : 1;            /*!< [0..0] Indicates that an interrupt is pending when it's logic
                                                 0. When it is 1, no interrupt is pending                                  */
    __IM  uint32_t int_id   : 3;            /*!< [3..1] Bit1 and Bit2 the two bits of the IIR are used to identify
                                                 the highest priority interrupt pending as indicated in
                                                 the following table Bit3: In the FIFO mode this bit is
                                                 set along with bit 2 when a timeout interrupt is pending.                 */
  } b;                                        /*!< bit fields for uart_iir */
} uart_iir_t, *puart_iir_t;

/**
  \brief Union type to access uart_fcr (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x00000008) FIFO Control Register                                      */
  
  struct {
    __OM  uint32_t en_rxfifo_err : 1;       /*!< [0..0] Set as 1 to enable the report of Error in RCVR FIFO field
                                                 in LSR bit [7]                                                            */
    __OM  uint32_t clear_rxfifo : 1;        /*!< [1..1] Writing a Logic 1 to Bit 1 clears the Receiver FIFO and
                                                 resets its logic. But it doesn't clear the shift register.
                                                 The 1 that is written to this bit position is self-clearing.              */
    __OM  uint32_t clear_txfifo : 1;        /*!< [2..2] Write 1 to this bit clears the Transmitter FIFO and resets
                                                 its logic. But the shift register is not cleared, The 1
                                                 that is written to this bit position is self-clearing.                    */
    __OM  uint32_t dma_mode : 1;            /*!< [3..3] Support DMA mode. (cooperate with DW DDMA in the data
                                                 path)                                                                     */
    __IM  uint32_t          : 1;
    __IOM uint32_t txfifo_low_level : 1;    /*!< [5..5] Define the Transmission FIFO Low Water Level Interrupt
                                                 trigger. 0: 4 byte 1: 8 bytes                                             */
    __IOM uint32_t rxfifo_trigger_level : 2;/*!< [7..6] Define the 32-entries Receiver FIFO Interrupt trigger
                                                 level 0~31 bytes 00: 1 byte 01: 8 bytes 10: 16 bytes 11:
                                                 30 bytes (for some device detect RTS de-assertion slower,
                                                 reserve more RX FIFO space to prevent RX FIFO overflow
                                                 )                                                                         */
  } b;                                        /*!< bit fields for uart_fcr */
} uart_fcr_t, *puart_fcr_t;

/**
  \brief Union type to access uart_lcr (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) Line Control Register                                      */
  
  struct {
    __IOM uint32_t wls0       : 1;            /*!< [0..0] Word length selection,                                             */
    __IM  uint32_t            : 1;
    __IOM uint32_t stb        : 1;            /*!< [2..2] This bit specifies the number of Stop bits transmitted
                                                   and received in each serial character. 0: 1 stop bit. 1:
                                                   2 stop bits. Note that the receiver always checks the first
                                                   stop bit only.                                                            */
    __IOM uint32_t parity_en  : 1;            /*!< [3..3] Parity Enable                                                      */
    __IOM uint32_t even_parity_sel : 1;       /*!< [4..4] Even Parity select                                                 */
    __IOM uint32_t stick_parity_en : 1;       /*!< [5..5] Stick Parity enable control. LCR[5:4]: 00: Odd Parity
                                                   01: Even Parity 10: Stick Parity as 0 11: Stick Parity
                                                   as 1                                                                      */
    __IOM uint32_t break_ctrl : 1;            /*!< [6..6] Break Control bit. Break control bit causes a break condition
                                                   to be transmitted to the receiving UART.                                  */
    __IOM uint32_t dlab       : 1;            /*!< [7..7] Divisor Latch Access bit Note: DLL/DLM only can be access
                                                   when dlab bit = 1, IER only can be access when dlab bit
                                                   = 0. THR/RBR don't care about dlab bit value                              */
  } b;                                        /*!< bit fields for uart_lcr */
} uart_lcr_t, *puart_lcr_t;

/**
  \brief Union type to access uart_mcr (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) Modem Control Register                                     */
  
  struct {
    __IOM uint32_t dtr        : 1;            /*!< [0..0] Data Terminal Ready (DTR) signal control 0::DTR is logic
                                                   1 1::DTR is logic 0                                                       */
    __IOM uint32_t rts        : 1;            /*!< [1..1] Request to Send (RTS) signal control 0: RTS is logic
                                                   1 1: RTS is logic 0 The RTS output is controlled as following
                                                   equation: RTS_output = rts_en ? (~rts|FIFO_FlowCtrl):~rts                 */
    __IOM uint32_t out1       : 1;            /*!< [2..2] This bit controls the Output 1 (OUT1_) signal, which
                                                   is an auxiliary user-designated output.                                   */
    __IOM uint32_t out2       : 1;            /*!< [3..3] This bit controls the output 2 (OUT2_) signal, which
                                                   is an auxiliary user-designated output.                                   */
    __IOM uint32_t loopback_en : 1;           /*!< [4..4] LoopBack mode. This bit provides a local loopback feature
                                                   for diagnostic testing of the UART.                                       */
    __IOM uint32_t cts_en     : 1;            /*!< [5..5] CTS flow control enable (CTSE) This Bit (CTSE) is the
                                                   auto CTS flow control enable. When set (1), the auto CTS
                                                   flow control as described in the detailed description is
                                                   enabled.                                                                  */
    __IOM uint32_t rts_en     : 1;            /*!< [6..6] RTS flow control enable (RTSE) This Bit (RTSE) is the
                                                   auto RTS flow control enables. When set (1), the auto RTS
                                                   flow control as described in the detailed description is
                                                   enabled.                                                                  */
    __IOM uint32_t sw_cts     : 1;            /*!< [7..7] Software controlled CTS. The software can use this bit
                                                   to pause the UART transmission, just like the HW flow control.This
                                                   bit setting will effects the CTS flow-control: CTS = cts_en
                                                   ? (sw_cts | CTS_input) : sw_cts                                           */
  } b;                                        /*!< bit fields for uart_mcr */
} uart_mcr_t, *puart_mcr_t;

/**
  \brief Union type to access uart_lsr (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) Line Status Register                                       */
  
  struct {
    __IM  uint32_t rxfifo_datardy : 1;        /*!< [0..0] Data Ready (DR) indicator This bit is 1 when at least
                                                   one character has been received and transferred into the
                                                   FIFO. It be reset to logic 0 by reading all of the data
                                                   in the Receiver Buffer Register or the RX FIFO.                           */
    __IM  uint32_t overrun_err : 1;           /*!< [1..1] Overrun Error (OE) indicator. This bit is set (1) to
                                                   indicates that data in the RX FIFO was not read by the
                                                   CPU before the next character was transferred into the
                                                   RX FIFO,                                                                  */
    __IM  uint32_t parity_err : 1;            /*!< [2..2] Parity Error (PE) indicator. This bit is set to indicates
                                                   that the received data character does not have the correct
                                                   even or odd parity, as selected by the even-parity-select
                                                   bit.                                                                      */
    __IM  uint32_t framing_err : 1;           /*!< [3..3] Framing Error (FE) indicator. This bit is set when the
                                                   received character at the top of the FIFO did not have
                                                   a valid stop bit.                                                         */
    __IM  uint32_t break_err_int : 1;         /*!< [4..4] Break Interrupt (BI) indicator. This bit is set to logic
                                                   1 whenever the received data input is held in the Spacing
                                                   (logic 0) state for a longer than a full word transmission
                                                   time.                                                                     */
    __IM  uint32_t txfifo_empty : 1;          /*!< [5..5] TXFIFO empty indicator. It indicates that the Transmitter
                                                   FIFO is empty. This bit is set when the Transmitter FIFO
                                                   is empty; it is cleared when at least 1 byte is written
                                                   to the Transmitter FIFO.                                                  */
    __IM  uint32_t            : 1;
    __IM  uint32_t rxfifo_err : 1;            /*!< [7..7] Uart_rx_error. This bit is set when there is at least
                                                   on parity error, framing error or break indication in the
                                                   FIFO. It is clear when the CPU reads the LSR, if there
                                                   are no subsequent errors in the FIFO                                      */
  } b;                                        /*!< bit fields for uart_lsr */
} uart_lsr_t, *puart_lsr_t;

/**
  \brief Union type to access uart_msr (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) Modem Status Register                                      */
  
  struct {
    __IM  uint32_t d_cts      : 1;            /*!< [0..0] Delta Clear to Send (DCTS) indicator                               */
    __IM  uint32_t d_dsr      : 1;            /*!< [1..1] Delta Data Set Ready (DDSR) indicator                              */
    __IM  uint32_t teri       : 1;            /*!< [2..2] Trailing Edge of Ring Indicator (TERI) detector. The
                                                   RI line has changed its state from low to high state.                     */
    __IM  uint32_t d_dcd      : 1;            /*!< [3..3] Delta Data Carrier Detect (DDCD) indicator.                        */
    __IM  uint32_t r_cts      : 1;            /*!< [4..4] Complement of the CTS input or equals to RTS in loopback
                                                   mode.                                                                     */
    __IM  uint32_t r_dsr      : 1;            /*!< [5..5] Complement of the DSR input or equals to DTR in loopback
                                                   mode.                                                                     */
    __IM  uint32_t r_ri       : 1;            /*!< [6..6] Complement of the RI input or equals to Out1 in loopback
                                                   mode.                                                                     */
    __IM  uint32_t r_dcd      : 1;            /*!< [7..7] Complement of the DCD input or equals to Out2 in loopback
                                                   mode.                                                                     */
  } b;                                        /*!< bit fields for uart_msr */
} uart_msr_t, *puart_msr_t;

/**
  \brief Union type to access uart_scr (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) Scratch Pad Register                                       */
  
  struct {
    __IM  uint32_t            : 3;
    __IOM uint32_t pin_lb_test : 1;           /*!< [3..3] For uart IP txd/rxd/rts/cts pin loopback test                      */
    __IM  uint32_t fl_frame_err : 1;          /*!< [4..4] Frame error flag                                                   */
    __IM  uint32_t fl_set_bi_err : 1;         /*!< [5..5] set_bi_err flag                                                    */
    __IOM uint32_t rx_break_int_en : 1;       /*!< [6..6] Rx break signal interrupt enable                                   */
    __IOM uint32_t rx_break_int_sts : 1;      /*!< [7..7] Rx break signal interrupt status, write 1 to this bit
                                                   will clear the interrupt pending status                                   */
    __IOM uint32_t dbg_sel    : 4;            /*!< [11..8] Debug port selection                                              */
    __IM  uint32_t            : 4;
    __IOM uint32_t xfactor_adj : 11;          /*!< [26..16] The ovsr_adj, one of factors of baud rate calculation            */
  } b;                                        /*!< bit fields for uart_scr */
} uart_scr_t, *puart_scr_t;

/**
  \brief Union type to access uart_stsr (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) STS Register                                               */
  
  struct {
    __IM  uint32_t            : 3;
    __IOM uint32_t reset_rcv  : 1;            /*!< [3..3] Reset Uart Receiver                                                */
    __IOM uint32_t xfactor    : 4;            /*!< [7..4] Factor of Baud rate calculation, the ovsr[3:0]                     */
    __IM  uint32_t            : 16;
    __IM  uint32_t dma_mode   : 1;            /*!< [24..24] dma_mode field of FCR bit[3]                                     */
    __IM  uint32_t fifo_en    : 1;            /*!< [25..25] fifo_en field of FCR bit[0]                                      */
    __IM  uint32_t txfifo_low_level : 1;      /*!< [26..26] txfifo_low_level in FCR bit[5], Define the Transmission
                                                   FIFO Low Water Level Interrupt trigger. 0: 4 byte 1: 8
                                                   bytes                                                                     */
    __IM  uint32_t rxfifo_trigger_level : 2;  /*!< [28..27] rxfifo_trigger_level in FCR bit[7:6], Define the 32-entries
                                                   Receiver FIFO Interrupt trigger level 0~31 bytes 00: 1
                                                   byte 01: 8 bytes 10: 16 bytes 11: 28 bytes (for some device
                                                   detect RTS de-assertion slower, reserve more RX FIFO space
                                                   to prevent RX FIFO overflow )                                             */
  } b;                                        /*!< bit fields for uart_stsr */
} uart_stsr_t, *puart_stsr_t;

/**
  \brief Union type to access uart_rbr (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x00000024) Receiver Buffer Register                                   */
  
  struct {
    __IM  uint32_t rxdata   : 8;            /*!< [7..0] Rx data. Bit 0 is the least significant bit. It is the
                                                 first bit serially received.                                              */
  } b;                                        /*!< bit fields for uart_rbr */
} uart_rbr_t, *puart_rbr_t;

/**
  \brief Union type to access uart_thr (@ 0x00000024).
*/
typedef union {
  __OM  uint32_t w;                         /*!< (@ 0x00000024) Transmitter Holder Register                                */
  
  struct {
    __OM  uint32_t txdata   : 8;            /*!< [7..0] Tx data. Bit 0 is the least significant bit. It is the
                                                 first bit serially transmitted.                                           */
  } b;                                        /*!< bit fields for uart_thr */
} uart_thr_t, *puart_thr_t;

/**
  \brief Union type to access uart_miscr (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) MISC Control Register                                      */
  
  struct {
    __IOM uint32_t irda_enable : 1;           /*!< [0..0] IRDA mode enable control. When set this bit as 1, the
                                                   UART is co-work with IRDA SIR mode. i.e., txd/rxd are irda
                                                   signals.                                                                  */
    __IOM uint32_t txdma_en   : 1;            /*!< [1..1] TX DMA enable control. (valid when dma_mode in FCR is
                                                   1)                                                                        */
    __IOM uint32_t rxdma_en   : 1;            /*!< [2..2] RX DMA enable control. (valid when dma_mode in FCR is
                                                   1)                                                                        */
    __IOM uint32_t txdma_burstsize : 5;       /*!< [7..3] Txdma burstsize                                                    */
    __IOM uint32_t rxdma_burstsize : 6;       /*!< [13..8] Rxdma burstsize                                                   */
    __IOM uint32_t irda_tx_inv : 1;           /*!< [14..14] Invert irda_tx_o when this bit is 1.                             */
    __IOM uint32_t irda_rx_inv : 1;           /*!< [15..15] Invert irda_rx_i when this bit is 1.                             */
    __IOM uint32_t tx_en      : 1;            /*!< [16..16] The UART TX function.enable control                              */
  } b;                                        /*!< bit fields for uart_miscr */
} uart_miscr_t, *puart_miscr_t;

/**
  \brief Union type to access uart_txplsr (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) IRDA SIR TX Pulse Width Control 0 Register                 */
  
  struct {
    __IOM uint32_t txpulse_lowbound_shiftval : 15;/*!< [14..0] The shift value of SIR tx pulse's left edge position.         */
    __IOM uint32_t lowbound_shiftright : 1;   /*!< [15..15] SIR TX pulse lower bond shift control.                           */
    __IOM uint32_t txpulse_upperbound_shiftval : 15;/*!< [30..16] The shift value of SIR tx pulse's right edge position.     */
    __IOM uint32_t Upperbound_shiftright : 1; /*!< [31..31] SIR TX pulse upper bond shift control.                           */
  } b;                                        /*!< bit fields for uart_txplsr */
} uart_txplsr_t, *puart_txplsr_t;

/**
  \brief Union type to access uart_baudmonr (@ 0x00000034).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000034) Baud Monitor Register                                      */
  
  struct {
    __IM  uint32_t min_fall_space : 12;       /*!< [11..0] Min_fall_space                                                    */
    __IM  uint32_t min_low_period : 12;       /*!< [23..12] Min_low_period                                                   */
    __IOM uint32_t falling_thresh : 6;        /*!< [29..24] Falling_thresh                                                   */
    __IM  uint32_t mon_data_vld : 1;          /*!< [30..30] The monitor data valid indication                                */
    __IOM uint32_t toggle_mon_en : 1;         /*!< [31..31] Baud monitor toggle bit enable.                                  */
  } b;                                        /*!< bit fields for uart_baudmonr */
} uart_baudmonr_t, *puart_baudmonr_t;

/**
  \brief Union type to access uart_dbg2 (@ 0x0000003C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000003C) Debug Register                                             */
  
  struct {
    __IM  uint32_t dbg_uart   : 32;           /*!< [31..0] The debug port output value, depend on dbg_sel value
                                                   in SCR[2:0].                                                              */
  } b;                                        /*!< bit fields for uart_dbg2 */
} uart_dbg2_t, *puart_dbg2_t;

/**
  \brief Union type to access uart_rfcr (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) RX Filter Control Register                                 */
  
  struct {
    __IOM uint32_t rf_len     : 1;            /*!< [0..0] Set the length of received data to be check. 0/1 = 1bytes/2bytes   */
    __IOM uint32_t rf_mask_en : 1;            /*!< [1..1] Enable the mask operation for received data.                       */
    __IOM uint32_t rf_cmp_op  : 2;            /*!< [3..2] Set the RX filter comparing rule. The matched condition
                                                   is listed as following table: RF_LEN = 0: RF_CMP_OP = 0
                                                   (AND) 1st byte = Magic Pattern1 1 (OR) (1st byte = Magic
                                                   Pattern1) OR (1st byte = Magic Pattern2) 2 (XOR) (1st byte
                                                   != Magic Pattern1) AND (1st byte != Magic Pattern2) RF_LEN
                                                   = 1: RF_CMP_OP = 0 (AND) (1st byte = Magic Pattern1) AND
                                                   (2nd byte = Magic Pattern2) 1 (OR) (2nd byte = Magic Pattern1)
                                                   OR (2nd byte = Magic Pattern2) 2 (XOR) (1st byte != Magic
                                                   Pattern1) AND (2nd byte != Magic Patte                                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t rf_en      : 1;            /*!< [7..7] RX Filter enable control                                           */
  } b;                                        /*!< bit fields for uart_rfcr */
} uart_rfcr_t, *puart_rfcr_t;

/**
  \brief Union type to access uart_rfmpr (@ 0x00000044).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000044) RX Filter Magic Pattern Register                           */
  
  struct {
    __IOM uint32_t rf_mp1     : 8;            /*!< [7..0] The magic pattern1 for the 1st received byte checking.             */
    __IOM uint32_t rf_mp2     : 8;            /*!< [15..8] The magic pattern2 for the 2nd received byte checking.            */
  } b;                                        /*!< bit fields for uart_rfmpr */
} uart_rfmpr_t, *puart_rfmpr_t;

/**
  \brief Union type to access uart_rfmvr (@ 0x00000048).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000048) RX Filter Mask Value Register                              */
  
  struct {
    __IOM uint32_t rf_mv1     : 8;            /*!< [7..0] The mask value for the 1st received byte.                          */
    __IOM uint32_t rf_mv2     : 8;            /*!< [15..8] The mask value for the 2nd received byte.                         */
  } b;                                        /*!< bit fields for uart_rfmvr */
} uart_rfmvr_t, *puart_rfmvr_t;

/**
  \brief Union type to access uart_rftor (@ 0x0000004C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000004C) RX Filter Timeout Register                                 */
  
  struct {
    __IOM uint32_t rf_timeout : 20;           /*!< [19..0] Set the timeout value of the RX filter idle detection.
                                                   This value is number of ticks. A tick time is equal to
                                                   a UART bit time.                                                          */
  } b;                                        /*!< bit fields for uart_rftor */
} uart_rftor_t, *puart_rftor_t;

/**
  \brief Union type to access uart_rflvr (@ 0x00000050).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000050) RX FIFO Level Register                                     */
  
  struct {
    __IM  uint32_t rx_fifo_lv : 6;            /*!< [5..0] The level of the RX FIFO. This value indicates the number
                                                   of data bytes in the RX FIFO.                                             */
  } b;                                        /*!< bit fields for uart_rflvr */
} uart_rflvr_t, *puart_rflvr_t;

/**
  \brief Union type to access uart_tflvr (@ 0x00000054).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000054) TX FIFO Level Register                                     */
  
  struct {
    __IM  uint32_t tx_fifo_lv : 5;            /*!< [4..0] The level of the TX FIFO. This value indicates the number
                                                   of data bytes in the TX FIFO.                                             */
  } b;                                        /*!< bit fields for uart_tflvr */
} uart_tflvr_t, *puart_tflvr_t;

/**
  \brief Union type to access uart_visr (@ 0x00000058).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000058) Vendor Interrupt Status Register                           */
  
  struct {
    __IOM uint32_t rf_match_patt : 1;         /*!< [0..0] The interrupt status of RX filter pattern checking matched.
                                                   Write 1 to clear this interrupt status. 0: Interrupt is
                                                   not pending. 1: Interrupt is pending.                                     */
    __IOM uint32_t rf_timeout : 1;            /*!< [1..1] This bit indicates the interrupt pending status of the
                                                   RX filter timeout occurred. The RX idle detection mechanism
                                                   is used to check whether the receiving is in idle state
                                                   for re-aquire pattern matching. This mechanism uses a timer
                                                   to monitor new data receiving. This timer will reload the
                                                   initial value whenever a new byte is received. If the timeout
                                                   occurred (timer value countdown to 0), the RX filter will
                                                   be reset to the initialed state and restart the first received
                                                   1 or 2 bytes checking.                                                    */
    __IOM uint32_t tx_fifo_lv : 1;            /*!< [2..2] This bit indicates the interrupt pending status of the
                                                   TX FIFO water level equal to the level setting. The software
                                                   can use this interrupt to know the TX FIFO water level
                                                   is low and refill the TX FIFO. To enable this interrupt,
                                                   IER bit[5] should be written as 1. Write 1 to this bit
                                                   will clear the pending status.                                            */
    __IOM uint32_t rx_idle_timeout : 1;       /*!< [3..3] This bit indicate the receiver idle timeout interrupt
                                                   pending status. Write 1 to this bit will clear the pending
                                                   status.                                                                   */
  } b;                                        /*!< bit fields for uart_visr */
} uart_visr_t, *puart_visr_t;

/**
  \brief Union type to access uart_vier (@ 0x0000005C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000005C) Vendor Interrupt Enabling Control Register                 */
  
  struct {
    __IOM uint32_t rf_match_int_en : 1;       /*!< [0..0] The Rx filter pattern matched interrupt enabling control:
                                                   1: enable 0: disable                                                      */
    __IOM uint32_t rf_timeout_int_en : 1;     /*!< [1..1] The Rx filter idle timeout interrupt enabling control:
                                                   1: enable 0: disable                                                      */
    __IOM uint32_t tx_fifo_lv_int_en : 1;     /*!< [2..2] The TX FIFO water level interrupt enabling control: 1:
                                                   enable 0: disable                                                         */
    __IOM uint32_t rx_idle_timeout_en : 1;    /*!< [3..3] The RX idle timeout interrupt enabling control: 1: enable
                                                   0: disable                                                                */
  } b;                                        /*!< bit fields for uart_vier */
} uart_vier_t, *puart_vier_t;

/**
  \brief Union type to access uart_ritor (@ 0x00000060).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000060) RX Idle Timeout Register                                   */
  
  struct {
    __IOM uint32_t rxidle_timeout_value : 4;  /*!< [3..0] Time unit is the duration of a UART bit, depends on the
                                                   baud-rate setting. Default 0. 0: 8 bit time. (1*8) 1: 16
                                                   bit time. (2*8) 2: 32 bit time. (2^2*8) 3: 64 bit time.
                                                   (2^3*8) 4: 128 bit time. (2^4*8) 5: 256 bit time. (2^5*8)
                                                   6: 512 bit time. (2^6*8) 7: 1024 bit time. (2^7*8) 8: 2048
                                                   bit time. (2^8*8) 9: 4096 bit time. (2^9*8) 10: 8192 bit
                                                   time. (2^10*8) 11: 16384 bit time. (2^11*8) 12: 32768 bit
                                                   time. (2^12*8) 13: 65535 bit time. (2^13*8) 14: 131072
                                                   bit time. (2^14*8) 15: 262144 bit time. (2^15*8)                          */
    __IM  uint32_t            : 27;
    __IOM uint32_t rx_idle_timeout_en : 1;    /*!< [31..31] RX idle timeout enable, default 0.                               */
  } b;                                        /*!< bit fields for uart_ritor */
} uart_ritor_t, *puart_ritor_t;

/** @} */ /* End of group ls_hal_uart_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_UART_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_UART_TYPE_H_

