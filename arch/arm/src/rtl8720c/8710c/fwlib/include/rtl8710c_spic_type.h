/**************************************************************************//**
 * @file      rtl8710c_spic_type.h
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

#ifndef _RTL8710C_SPIC_TYPE_H_
#define _RTL8710C_SPIC_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SPIC_REG_TYPE

/**
 * @addtogroup hs_hal_spic_reg SPIC Registers.
 * @ingroup hs_hal_spic
 * @{
 */

/**
  \brief Union type to access spic_ctrlr0 (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) SPIC Control Register 0                                    */
  
  struct {
    __IM  uint32_t            : 6;
    __IOM uint32_t scph       : 1;            /*!< [6..6] Serial Clock Phase.                                                */
    __IOM uint32_t scpol      : 1;            /*!< [7..7] Serial Clock Polarity.                                             */
    __IOM uint32_t tmod       : 2;            /*!< [9..8] Transfer mode.                                                     */
    __IM  uint32_t            : 6;
    __IOM uint32_t addr_ch    : 2;            /*!< [17..16] Indicate channel number of address phase in transmitting
                                                   or receiving data. Data phase is used to send data after
                                                   address phase. 0 : single channel, 1 : dual channels, 2
                                                   : quad channels, 3 : octal channel.                                       */
    __IOM uint32_t data_ch    : 2;            /*!< [19..18] Indicate channel number of data phase in transmitting
                                                   or receiving data. Data phase is used to send data after
                                                   address phase. 0 : single channel, 1 : dual channels, 2
                                                   : quad channels, 3 : octal channel.                                       */
    __IOM uint32_t cmd_ch     : 2;            /*!< [21..20] Indicate channel number of command phase in transmitting
                                                   or receiving data. Data phase is used to send data after
                                                   address phase. 0 : single channel, 1 : dual channels, 2
                                                   : quad channels, 3 : octal channel.                                       */
    __IOM uint32_t fast_rd    : 1;            /*!< [22..22] Indicate to use fast read command in user mode. If
                                                   setting to 1, SPIC would use FBAUDR to derive spi_sclk.                   */
    __IOM uint32_t ck_mtimes  : 5;            /*!< [27..23] Indicate the check time.                                         */
    __IOM uint32_t addr_ddr_en : 1;           /*!< [28..28] Enable address phase ddr mode.                                   */
    __IOM uint32_t data_ddr_en : 1;           /*!< [29..29] Enable data phase ddr mode.                                      */
    __IOM uint32_t cmd_ddr_en : 1;            /*!< [30..30] Enable command phase ddr mode. Always 2-byte command
                                                   type.                                                                     */
  } b;                                        /*!< bit fields for spic_ctrlr0 */
} spic_ctrlr0_t, *pspic_ctrlr0_t;

/**
  \brief Union type to access spic_ctrlr1 (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) SPIC Control Register 1                                    */
  
  struct {
    __IOM uint32_t ndf        : 12;           /*!< [11..0] ndf                                                               */
  } b;                                        /*!< bit fields for spic_ctrlr1 */
} spic_ctrlr1_t, *pspic_ctrlr1_t;

/**
  \brief Union type to access spic_ssienr (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) SPIC Enable Register                                       */
  
  struct {
    __IOM uint32_t spic_en    : 1;            /*!< [0..0] Enable or disable SPIC.                                            */
    __OM  uint32_t atck_cmd   : 1;            /*!< [1..1] Set to enable ATCK_CMD implementation. After this bit
                                                   is set, SPIC would not accept any command until checking
                                                   flash is not busy or timeout.                                             */
  } b;                                        /*!< bit fields for spic_ssienr */
} spic_ssienr_t, *pspic_ssienr_t;

/**
  \brief Union type to access spic_ser (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) Slave Enable Register                                      */
  
  struct {
    __IOM uint32_t ser        : 1;            /*!< [0..0] SPIC only has one slave select line. This bit should
                                                   be always set.                                                            */
  } b;                                        /*!< bit fields for spic_ser */
} spic_ser_t, *pspic_ser_t;

/**
  \brief Union type to access spic_baudr (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) Baud Rate Select Register                                  */
  
  struct {
    __IOM uint32_t sckdv      : 12;           /*!< [11..0] This register controls the frequency of spi_sclk. spi_sclk
                                                   = frequency of bus_clk / (2*sckdv)                                        */
  } b;                                        /*!< bit fields for spic_baudr */
} spic_baudr_t, *pspic_baudr_t;

/**
  \brief Union type to access spic_txftlr (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) Transmit FIFO Threshold Level                              */
  
  struct {
    __IOM uint32_t tft        : 8;            /*!< [7..0] Transmit FIFO Threshold. Controls the level of entries
                                                   (or below) at which the transmit FIFO controller triggers
                                                   an interrupt.                                                             */
  } b;                                        /*!< bit fields for spic_txftlr */
} spic_txftlr_t, *pspic_txftlr_t;

/**
  \brief Union type to access spic_rxftlr (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) Receive FIFO Threshold Level                               */
  
  struct {
    __IOM uint32_t rft        : 8;            /*!< [7..0] Receive FIFO Threshold. Controls the level of entries
                                                   (or above) at which the receive FIFO controller triggers
                                                   an interrupt.                                                             */
  } b;                                        /*!< bit fields for spic_rxftlr */
} spic_rxftlr_t, *pspic_rxftlr_t;

/**
  \brief Union type to access spic_txflr (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) Transmit FIFO Level Register                               */
  
  struct {
    __IM  uint32_t txtfl      : 8;            /*!< [7..0] Transmit FIFO Level. Contains the number of valid data
                                                   entries in the transmit FIFO.                                             */
  } b;                                        /*!< bit fields for spic_txflr */
} spic_txflr_t, *pspic_txflr_t;

/**
  \brief Union type to access spic_rxflr (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000024) Receive FIFO Level Register                                */
  
  struct {
    __IM  uint32_t rxtfl      : 8;            /*!< [7..0] Receive FIFO Level. Contains the number of valid data
                                                   entries in the receive FIFO.                                              */
  } b;                                        /*!< bit fields for spic_rxflr */
} spic_rxflr_t, *pspic_rxflr_t;

/**
  \brief Union type to access spic_sr (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) Status Register                                            */
  
  struct {
    __IM  uint32_t busy       : 1;            /*!< [0..0] SPIC Busy Flag. When set, indicates that a serial transfer
                                                   is in progress; when cleared indicates that the SPIC is
                                                   idle or disabled.                                                         */
    __IM  uint32_t tfnf       : 1;            /*!< [1..1] Transmit FIFO Not Full. Set when the transmit FIFO contains
                                                   one or more empty locations, and is cleared when the FIFO
                                                   is full. 0 : Transmit FIFO is full 1 : Transmit FIFO is
                                                   not full                                                                  */
    __IM  uint32_t tfe        : 1;            /*!< [2..2] Transmit FIFO Empty. When the transmit FIFO is completely
                                                   empty, this bit is set. When the transmit FIFO contains
                                                   one or more valid entries, this bit is cleared. This bit
                                                   field does not request an interrupt. 0 : Transmit FIFO
                                                   is not empty 1 : Transmit FIFO is empty                                   */
    __IM  uint32_t rfne       : 1;            /*!< [3..3] Receive FIFO Not Empty. Set when the receive FIFO contains
                                                   one or more entries and is cleared when the receive FIFO
                                                   is empty. This bit can be polled by software to completely
                                                   empty the receive FIFO. 0 : Receive FIFO is empty 1 : Receive
                                                   FIFO is not empty                                                         */
    __IM  uint32_t rff        : 1;            /*!< [4..4] Receive FIFO Full. When the receive FIFO is completely
                                                   full, this bit is set. When the receive FIFO contains one
                                                   or more empty location, this bit is cleared. 0 : Receive
                                                   FIFO is not full 1 : Receive FIFO is full                                 */
    __IM  uint32_t txe        : 1;            /*!< [5..5] Transmission Error. Set if the transmit FIFO is empty
                                                   when a transfer is started. This bit can be set only when
                                                   the DW_apb_ssi is configured as a slave device. Data from
                                                   the previous transmission is resent on the txd line. This
                                                   bit is cleared when read. 0 : No error 1 : Transmission
                                                   error                                                                     */
    __IM  uint32_t dcol       : 1;            /*!< [6..6] Transmitting Status. This bit is set when SPIC is transmitting
                                                   command, address or data to the data register. By observing
                                                   this bit, users can avoid reading wrong data at the wrong
                                                   time if data is not ready to be read.                                     */
  } b;                                        /*!< bit fields for spic_sr */
} spic_sr_t, *pspic_sr_t;

/**
  \brief Union type to access spic_imr (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) Interrupt Mask Register                                    */
  
  struct {
    __IOM uint32_t txeim      : 1;            /*!< [0..0] Transmit FIFO Empty Interrupt Mask 0 : spi_txeir_r interrupt
                                                   is masked 1 : spi_txeir_r interrupt is not masked                         */
    __IOM uint32_t txoim      : 1;            /*!< [1..1] Transmit FIFO Overflow Interrupt Mask 0 : spi_txoir_r
                                                   interrupt is masked 1 : spi_txoir_r interrupt is not masked               */
    __IOM uint32_t rxuim      : 1;            /*!< [2..2] Receive FIFO Underflow Interrupt Mask 0 : spi_rxuir_r
                                                   interrupt is masked 1 : spi_rxuir_r interrupt is not masked               */
    __IOM uint32_t rxoim      : 1;            /*!< [3..3] Receive FIFO Overflow Interrupt Mask 0 : spi_rxoir_r
                                                   interrupt is masked 1 : spi_rxoir_r interrupt is not masked               */
    __IOM uint32_t rxfim      : 1;            /*!< [4..4] Receive FIFO Full Interrupt Mask 0 : spi_rxfir_r interrupt
                                                   is masked 1 : spi_rxfir_r interrupt is not masked                         */
    __IOM uint32_t fseim      : 1;            /*!< [5..5] FIFO Size Error Interrupt Mask. 0 : spi_fseir_r interrupt
                                                   is masked 1 : spi_fseir_r interrupt is not masked                         */
    __IOM uint32_t wbeim      : 1;            /*!< [6..6] Write Burst Error Interrupt Mask. 0 : spi_wbier_r interrupt
                                                   is masked 1 : spi_wbier_r interrupt is not masked                         */
    __IOM uint32_t byeim      : 1;            /*!< [7..7] Byte Enable Error Interrupt Mask 0 : spi_byeir_r interrupt
                                                   is masked 1 : spi_byeir_r interrupt is not masked                         */
    __IOM uint32_t aceim      : 1;            /*!< [8..8] Auto-check Timeout Error Interrupt Mask. 0 : spi_aceir_r
                                                   interrupt is masked 1 : spi_aceir_r interrupt is not masked               */
    __IOM uint32_t txsim      : 1;            /*!< [9..9] Transmit Split Interrupt Mask. 0 : spi_tx_sir_r interrupt
                                                   is masked 1 : spi_tx_sir_r interrupt is not masked                        */
  } b;                                        /*!< bit fields for spic_imr */
} spic_imr_t, *pspic_imr_t;

/**
  \brief Union type to access spic_isr (@ 0x00000030).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000030) Interrupt Status Register                                  */
  
  struct {
    __IM  uint32_t txeis      : 1;            /*!< [0..0] Transmit FIFO Empty Interrupt Status after Masking 0
                                                   : spi_txeir_r interrupt is not active after masking 1 :
                                                   spi_txeir_r interrupt is active after masking                             */
    __IM  uint32_t txois      : 1;            /*!< [1..1] Transmit FIFO Overflow Interrupt Status after Masking
                                                   0 : spi_txoir_r interrupt is not active after masking 1
                                                   : spi_txoir_r interrupt is active after masking                           */
    __IM  uint32_t rxuis      : 1;            /*!< [2..2] Receive FIFO Underflow Interrupt Status after Masking
                                                   0 : spi_rxuir_r interrupt is not active after masking 1
                                                   : spi_rxuir_r interrupt is active after masking                           */
    __IM  uint32_t rxois      : 1;            /*!< [3..3] Receive FIFO Overflow Interrupt Status after Masking
                                                   0 : spi_rxoir_r interrupt is not active after masking 1
                                                   : spi_rxoir_r interrupt is active after masking                           */
    __IM  uint32_t rxfis      : 1;            /*!< [4..4] Receive FIFO Full Interrupt Status after Masking 0 :
                                                   spi_rxfir_r interrupt is not active after masking 1 : spi_rxfir_r
                                                   interrupt is active after masking                                         */
    __IM  uint32_t fseis      : 1;            /*!< [5..5] FIFO Size Error Interrupt Status after Masking. 0 : spi_fseir_r
                                                   interrupt is not active after masking 1 : spi_fseir_r interrupt
                                                   is active after masking                                                   */
    __IM  uint32_t wbeis      : 1;            /*!< [6..6] Write Burst Error Interrupt Status after Masking. 0 :
                                                   spi_wbier_r interrupt is not active after masking 1 : spi_wbier_r
                                                   interrupt is active after masking                                         */
    __IM  uint32_t byeis      : 1;            /*!< [7..7] Byte Enable Error Interrupt Status after Masking 0 :
                                                   spi_byeir_r interrupt is not active after masking 1 : spi_byeir_r
                                                   interrupt is active after masking                                         */
    __IM  uint32_t aceis      : 1;            /*!< [8..8] Auto-check Timeout Error Interrupt Status after Masking.
                                                   0 : spi_aceir_r interrupt is not active after masking 1
                                                   : spi_aceir_r interrupt is active after masking                           */
    __IM  uint32_t txsis      : 1;            /*!< [9..9] Transmit Split Interrupt Status after Masking. 0 : spi_tx_sir_r
                                                   interrupt is not active after masking 1 : spi_tx_sir_r
                                                   interrupt is active after masking                                         */
    __IM  uint32_t rxsis      : 1;            /*!< [10..10] Receive Split Interrupt Status after Masking. 0 : spi_rx_sir_r
                                                   interrupt is not active after masking 1 : spi_rx_sir_r
                                                   interrupt is active after masking                                         */
  } b;                                        /*!< bit fields for spic_isr */
} spic_isr_t, *pspic_isr_t;

/**
  \brief Union type to access spic_risr (@ 0x00000034).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000034) Raw Interrupt Status Register                              */
  
  struct {
    __IM  uint32_t txeir      : 1;            /*!< [0..0] Transmit FIFO Empty Interrupt Raw Status before Masking
                                                   0 : spi_txeir_r interrupt is not acitve before masking
                                                   1 : spi_txeir_r interrupt is acitve before masking                        */
    __IM  uint32_t txoir      : 1;            /*!< [1..1] Transmit FIFO Overflow Interrupt Raw Status before Masking
                                                   0 : spi_txoir_r interrupt is not acitve before masking
                                                   1 : spi_txoir_r interrupt is acitve before masking                        */
    __IM  uint32_t rxuir      : 1;            /*!< [2..2] Receive FIFO Underflow Interrupt Raw Status before Masking
                                                   0 : spi_rxuir_r interrupt is not acitve before masking
                                                   1 : spi_rxuir_r interrupt is acitve before masking                        */
    __IM  uint32_t rxoir      : 1;            /*!< [3..3] Receive FIFO Overflow Interrupt Raw Status before Masking
                                                   0 : spi_rxoir_r interrupt is not acitve before masking
                                                   1 : spi_rxoir_r interrupt is acitve before masking                        */
    __IM  uint32_t rxfir      : 1;            /*!< [4..4] Receive FIFO Full Interrupt Raw Status before Masking
                                                   0 : spi_rxfir_r interrupt is not acitve before masking
                                                   1 : spi_rxfir_r interrupt is acitve before masking                        */
    __IM  uint32_t fseir      : 1;            /*!< [5..5] FIFO Size Error Interrupt Raw Status before Masking.
                                                   0 : spi_fseir_r interrupt is not acitve before masking
                                                   1 : spi_fseir_r interrupt is acitve before masking                        */
    __IM  uint32_t wbeir      : 1;            /*!< [6..6] Write Burst Error Interrupt Raw Status before Masking.
                                                   0 : spi_wbier_r interrupt is not acitve before masking
                                                   1 : spi_wbier_r interrupt is acitve before masking                        */
    __IM  uint32_t byeir      : 1;            /*!< [7..7] Byte Enable Error Interrupt Raw Status before Masking
                                                   0 : spi_byeir_r interrupt is not acitve before masking
                                                   1 : spi_byeir_r interrupt is acitve before masking                        */
    __IM  uint32_t aceir      : 1;            /*!< [8..8] Auto-check Timeout Error Interrupt Raw Status before
                                                   Masking. 0 : spi_aceir_r interrupt is not acitve before
                                                   masking 1 : spi_aceir_r interrupt is acitve before masking                */
  } b;                                        /*!< bit fields for spic_risr */
} spic_risr_t, *pspic_risr_t;

/**
  \brief Union type to access spic_txoicr (@ 0x00000038).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000038) Transmit FIFO Overflow Interrupt Clear Register            */
  
  struct {
    __IM  uint32_t txoicr     : 1;            /*!< [0..0] Clear Transmit FIFO Overflow Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_txo_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for spic_txoicr */
} spic_txoicr_t, *pspic_txoicr_t;

/**
  \brief Union type to access spic_rxoicr (@ 0x0000003C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000003C) Receive FIFO Overflow Interrupt Clear Register             */
  
  struct {
    __IM  uint32_t rxoicr     : 1;            /*!< [0..0] Clear Receive FIFO Overflow Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_rxo_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for spic_rxoicr */
} spic_rxoicr_t, *pspic_rxoicr_t;

/**
  \brief Union type to access spic_rxuicr (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) Receive FIFO Underflow Interrupt Clear Register            */
  
  struct {
    __IM  uint32_t rxuicr     : 1;            /*!< [0..0] Clear Receive FIFO Underflow Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_rxu_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for spic_rxuicr */
} spic_rxuicr_t, *pspic_rxuicr_t;

/**
  \brief Union type to access spic_faeicr (@ 0x00000044).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000044) Frame Alignment Interrupt Clear Register                   */
  
  struct {
    __IM  uint32_t faeicr     : 1;            /*!< [0..0] Clear Frame Alignment Interrupt. This register reflects
                                                   the status of the interrupt. A read from this register
                                                   clears the ssi_fae_intr interrupt; writing has no effect.                 */
  } b;                                        /*!< bit fields for spic_faeicr */
} spic_faeicr_t, *pspic_faeicr_t;

/**
  \brief Union type to access spic_icr (@ 0x00000048).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000048) Interrupt Clear Register                                   */
  
  struct {
    __IM  uint32_t icr        : 1;            /*!< [0..0] Clear Interrupts. This register is set if any of the
                                                   interrupts below are active. A read clears the ssi_txu_intr,
                                                   ssi_txo_intr, ssi_rxu_intr, ssi_rxo_intr, and the ssi_fae_intr
                                                   interrupts. Writing to this register has no effect.                       */
  } b;                                        /*!< bit fields for spic_icr */
} spic_icr_t, *pspic_icr_t;

/**
  \brief Union type to access spic_dmacr (@ 0x0000004C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000004C) DMA Control Register                                       */
  
  struct {
    __IOM uint32_t rx_dmac_en : 1;            /*!< [0..0] Receive DMA Enable. This bit enables/disables the receive
                                                   FIFO DMA channel 0 : Receive DMA disabled 1 : Receive DMA
                                                   enabled                                                                   */
    __IOM uint32_t tx_dmac_en : 1;            /*!< [1..1] Transmit DMA Enable. This bit enables/disables the transmit
                                                   FIFO DMA channel. 0 : Transmit DMA disabled 1 : Transmit
                                                   DMA enabled                                                               */
  } b;                                        /*!< bit fields for spic_dmacr */
} spic_dmacr_t, *pspic_dmacr_t;

/**
  \brief Union type to access spic_dmatdlr (@ 0x00000050).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000050) DMA Transmit Data Level Register                           */
  
  struct {
    __IOM uint32_t dmatdl     : 8;            /*!< [7..0] Transmit Data Level. This bit field controls the level
                                                   at which a DMA request is made by the transmit logic. It
                                                   is equal to the watermark level; that is, the dma_tx_req
                                                   signal is generated when the number of valid data entries
                                                   in the transmit FIFO is equal to or below this field value,
                                                   and tx_dmac_en = 1.                                                       */
  } b;                                        /*!< bit fields for spic_dmatdlr */
} spic_dmatdlr_t, *pspic_dmatdlr_t;

/**
  \brief Union type to access spic_dmardlr (@ 0x00000054).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000054) DMA Receive Data Level Register                            */
  
  struct {
    __IOM uint32_t dmardl     : 8;            /*!< [7..0] Receive Data Level. This bit field controls the level
                                                   at which a DMA request is made by the receive logic. The
                                                   watermark level = DMARDL+1; that is, dma_rx_req is generated
                                                   when the number of valid data entries in the receive FIFO
                                                   is equal to or above this field value + 1, and rx_dmac_en
                                                   = 1.                                                                      */
  } b;                                        /*!< bit fields for spic_dmardlr */
} spic_dmardlr_t, *pspic_dmardlr_t;

/**
  \brief Union type to access spic_dr_word (@ 0x00000060).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x00000060) Data Register                                              */
  
  struct {
    __IOM uint32_t word     : 32;           /*!< [31..0] Access FIFO as if its width is 4 byte per data item               */
  } b;                                        /*!< bit fields for spic_dr_word */
} spic_dr_word_t, *pspic_dr_word_t;

/**
  \brief Union type to access spic_dr_half_word (@ 0x00000060).
*/
typedef union {
  __IOM uint16_t w;                         /*!< (@ 0x00000060) Data Register                                              */
  
  struct {
    __IOM uint16_t half_word : 16;          /*!< [15..0] Access FIFO as if its width is 2 byte per data item               */
  } b;                                        /*!< bit fields for spic_dr_half_word */
} spic_dr_half_word_t, *pspic_dr_half_word_t;

/**
  \brief Union type to access spic_dr_byte (@ 0x00000060).
*/
typedef union {
  __IOM uint8_t w;                          /*!< (@ 0x00000060) Data Register                                              */
  
  struct {
    __IOM uint8_t byte      : 8;            /*!< [7..0] Access FIFO as if its width is 1 byte per data item                */
  } b;                                        /*!< bit fields for spic_dr_byte */
} spic_dr_byte_t, *pspic_dr_byte_t;

/**
  \brief Union type to access spic_read_fast_single (@ 0x000000E0).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x000000E0) Fast Read Command Register                                 */
  
  struct {
    __IOM uint32_t frd_cmd  : 8;            /*!< [7..0] Fast read command.                                                 */
  } b;                                        /*!< bit fields for spic_read_fast_single */
} spic_read_fast_single_t, *pspic_read_fast_single_t;

/**
  \brief Union type to access spic_rd_octal_io (@ 0x000000E0).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x000000E0) Fast Read Command Register                                 */
  
  struct {
    __IOM uint32_t frd_octal_cmd : 16;      /*!< [15..0] Fast read command for Octal IO mode.                              */
  } b;                                        /*!< bit fields for spic_rd_octal_io */
} spic_rd_octal_io_t, *pspic_rd_octal_io_t;

/**
  \brief Union type to access spic_read_dual_data (@ 0x000000E4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000E4) Dual Read Command Register                                 */
  
  struct {
    __IOM uint32_t rd_dual_o_cmd : 8;         /*!< [7..0] Dual data read command, 1-1-2 mode.                                */
  } b;                                        /*!< bit fields for spic_read_dual_data */
} spic_read_dual_data_t, *pspic_read_dual_data_t;

/**
  \brief Union type to access spic_read_dual_addr_data (@ 0x000000E8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000E8) Dual IO Read Command Register                              */
  
  struct {
    __IOM uint32_t rd_dual_io_cmd : 8;        /*!< [7..0] Dual address and data read command, 1-2-2 mode.                    */
  } b;                                        /*!< bit fields for spic_read_dual_addr_data */
} spic_read_dual_addr_data_t, *pspic_read_dual_addr_data_t;

/**
  \brief Union type to access spic_read_quad_data (@ 0x000000EC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000EC) Quad Read Command Register                                 */
  
  struct {
    __IOM uint32_t rd_quad_o_cmd : 8;         /*!< [7..0] Quad data read command, 1-1-4 mode.                                */
  } b;                                        /*!< bit fields for spic_read_quad_data */
} spic_read_quad_data_t, *pspic_read_quad_data_t;

/**
  \brief Union type to access spic_read_quad_addr_data (@ 0x000000F0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F0) Quad IO Read Command Register                              */
  
  struct {
    __IOM uint32_t rd_quad_io_cmd : 8;        /*!< [7..0] Quad address and data read command, 1-4-4 mode.                    */
    __IM  uint32_t            : 8;
    __IOM uint32_t prm_value  : 8;            /*!< [23..16] High Performance Read Mode Control Value.                        */
  } b;                                        /*!< bit fields for spic_read_quad_addr_data */
} spic_read_quad_addr_data_t, *pspic_read_quad_addr_data_t;

/**
  \brief Union type to access spic_write_single (@ 0x000000F4).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x000000F4) Single IO Page Program Command Register                    */
  
  struct {
    __IOM uint32_t wr_cmd   : 8;            /*!< [7..0] One bit mode page program command.                                 */
  } b;                                        /*!< bit fields for spic_write_single */
} spic_write_single_t, *pspic_write_single_t;

/**
  \brief Union type to access spic_write_octal_io (@ 0x000000F4).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x000000F4) Octal IO Page Program Command Register                     */
  
  struct {
    __IOM uint32_t wr_octal_cmd : 16;       /*!< [15..0] Octal IO page program command.                                    */
  } b;                                        /*!< bit fields for spic_write_octal_io */
} spic_write_octal_io_t, *pspic_write_octal_io_t;

/**
  \brief Union type to access spic_write_dual_data (@ 0x000000F8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F8) Dual Page Program Command Register                         */
  
  struct {
    __IOM uint32_t rd_dual_io_cmd : 8;        /*!< [7..0] Dual page program command, 1-1-2 mode.                             */
  } b;                                        /*!< bit fields for spic_write_dual_data */
} spic_write_dual_data_t, *pspic_write_dual_data_t;

/**
  \brief Union type to access spic_write_dual_addr_data (@ 0x000000FC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000FC) Dual IO Page Program Command Register                      */
  
  struct {
    __IOM uint32_t wr_dual_ii_cmd : 8;        /*!< [7..0] Dual IO page program command, 1-2-2 mode.                          */
  } b;                                        /*!< bit fields for spic_write_dual_addr_data */
} spic_write_dual_addr_data_t, *pspic_write_dual_addr_data_t;

/**
  \brief Union type to access spic_write_quad_data (@ 0x00000100).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000100) Quad Page Program Command Register                         */
  
  struct {
    __IOM uint32_t wr_quad_i_cmd : 8;         /*!< [7..0] Quad page program command, 1-1-4 mode.                             */
  } b;                                        /*!< bit fields for spic_write_quad_data */
} spic_write_quad_data_t, *pspic_write_quad_data_t;

/**
  \brief Union type to access spic_write_quad_addr_data (@ 0x00000104).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000104) Quad IO Page Program Command Register                      */
  
  struct {
    __IOM uint32_t wr_quad_ii_cmd : 8;        /*!< [7..0] Quad IO page program command, 1-4-4 mode.                          */
  } b;                                        /*!< bit fields for spic_write_quad_addr_data */
} spic_write_quad_addr_data_t, *pspic_write_quad_addr_data_t;

/**
  \brief Union type to access spic_write_enable (@ 0x00000108).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000108) Write Enable Command Register                              */
  
  struct {
    __IOM uint32_t wr_en_cmd  : 16;           /*!< [15..0] Write Enable Command. The second command byte is for
                                                   DTR mode.                                                                 */
  } b;                                        /*!< bit fields for spic_write_enable */
} spic_write_enable_t, *pspic_write_enable_t;

/**
  \brief Union type to access spic_read_status (@ 0x0000010C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000010C) Read Status Command Register                               */
  
  struct {
    __IOM uint32_t rd_st_cmd  : 16;           /*!< [15..0] Read flash status register command.                               */
  } b;                                        /*!< bit fields for spic_read_status */
} spic_read_status_t, *pspic_read_status_t;

/**
  \brief Union type to access spic_ctrlr2 (@ 0x00000110).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000110) SPIC Control Register 2                                    */
  
  struct {
    __IOM uint32_t so_dnum    : 1;            /*!< [0..0] Indicate SO input pin of SPI Flash is connected to spi_sout[0]
                                                   or spi_sout[1]. Set 1 to support multi-channels connection
                                                   in default.                                                               */
    __IM  uint32_t            : 2;
    __IOM uint32_t seq_en     : 1;            /*!< [3..3] Set to enable data-split program / read.                           */
    __IOM uint32_t tx_fifo_entry : 4;         /*!< [7..4] Indicate the valid entry of transmit FIFO.                         */
    __IOM uint32_t rx_fifo_entry : 4;         /*!< [11..8] Indicate the valid entry of receive FIFO.                         */
    __IOM uint32_t cs_active_hold : 2;        /*!< [13..12] For flash chip select active hold time after SCLK resing
                                                   edge.                                                                     */
  } b;                                        /*!< bit fields for spic_ctrlr2 */
} spic_ctrlr2_t, *pspic_ctrlr2_t;

/**
  \brief Union type to access spic_fbaudr (@ 0x00000114).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000114) Fast Read Baud Rate Select Register                        */
  
  struct {
    __IOM uint32_t fsckdv     : 12;           /*!< [11..0] This register controls the frequency of spi_sclk for
                                                   fast read command. spi_sclk = frequency of bus_clk / (2*fsckdv)           */
  } b;                                        /*!< bit fields for spic_fbaudr */
} spic_fbaudr_t, *pspic_fbaudr_t;

/**
  \brief Union type to access spic_addr_length (@ 0x00000118).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000118) Address Byte Length Register                               */
  
  struct {
    __IOM uint32_t addr_phase_length : 3;     /*!< [2..0] Indicate the number of bytes in address phase. addr_phase_length
                                                   [2] is an extended bit for 4-Byte Address with PRM mode.
                                                   Set to 5, SPIC can send 4 byte address and 1 byte PRM mode
                                                   value. 3-Byte Address with PRM mode need to set this bit
                                                   to 0, SPIC can send 3byte address and 1byte PRM mode value.
                                                   1 : One byte address, 2 : Two byte address, 3 : Three byte
                                                   address, 4: Four byte address with PRM value, 0 : Four
                                                   byte address / Three bytee addree with PRM value.                         */
  } b;                                        /*!< bit fields for spic_addr_length */
} spic_addr_length_t, *pspic_addr_length_t;

/**
  \brief Union type to access spic_auto_length (@ 0x0000011C).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x0000011C) Auto Mode Address Byte Length Register                     */
  
  struct {
    __IOM uint32_t rd_dummy_length : 12;    /*!< [11..0] Indicate dummy cycles for receiving data. It is referenced
                                                 by bus_clk.                                                               */
    __IM  uint32_t          : 4;
    __IOM uint32_t auto_addr_length : 2;    /*!< [17..16] Number of address bytes in read/write command in auto
                                                 mode                                                                      */
    __IOM uint32_t auto_dum_len : 8;        /*!< [25..18] Dummy cycle is used to check flash status in auto_write
                                                 operation if delay time of read data 1 cycle.                             */
    __IOM uint32_t cs_h_rd_dum_len : 2;     /*!< [27..26] Dummy cycle between sending read command to SPI flash.
                                                 Using the dummy cycles can avoid the timing violation of
                                                 CS high time.                                                             */
    __IOM uint32_t cs_h_wr_dum_len : 4;     /*!< [31..28] Dummy cycle between sending write command to SPI flash.
                                                 Using the dummy cycles can avoid the timing violation of
                                                 CS high time.                                                             */
  } b;                                        /*!< bit fields for spic_auto_length */
} spic_auto_length_t, *pspic_auto_length_t;

/**
  \brief Union type to access spic_auto_length_seq (@ 0x0000011C).
*/
typedef union {
  __IOM uint32_t w;                         /*!< (@ 0x0000011C) Sequential Auto Mode Address Byte Length Register          */
  
  struct {
    __IOM uint32_t rd_dummy_length : 12;    /*!< [11..0] Indicate dummy cycles for receiving data. It is referenced
                                                 by bus_clk.                                                               */
    __IOM uint32_t in_physical_cyc : 4;     /*!< [15..12] Indicate how many SPIC clk cycles from pad to internal
                                                 SPIC.                                                                     */
    __IOM uint32_t auto_addr_length : 2;    /*!< [17..16] Number of address bytes in read/write command in auto
                                                 mode                                                                      */
    __IOM uint32_t spic_cyc_per_byte : 8;   /*!< [25..18] Indicate how many SPIC clk cycles for one byte. Formula
                                                 = BAUD*2*Byte / CH, the maximum value is 256                              */
    __IOM uint32_t cs_h_rd_dum_len : 2;     /*!< [27..26] Dummy cycle between sending read command to SPI flash.
                                                 Using the dummy cycles can avoid the timing violation of
                                                 CS high time.                                                             */
    __IOM uint32_t cs_h_wr_dum_len : 4;     /*!< [31..28] Dummy cycle between sending write command to SPI flash.
                                                 Using the dummy cycles can avoid the timing violation of
                                                 CS high time.                                                             */
  } b;                                        /*!< bit fields for spic_auto_length_seq */
} spic_auto_length_seq_t, *pspic_auto_length_seq_t;

/**
  \brief Union type to access spic_valid_cmd (@ 0x00000120).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000120) Valid Command Register                                     */
  
  struct {
    __IOM uint32_t frd_single : 1;            /*!< [0..0] Execute fast read for auto read mode.                              */
    __IOM uint32_t rd_dual_i  : 1;            /*!< [1..1] Execute dual data write for auto read mode. (1-1-2)                */
    __IOM uint32_t rd_dual_io : 1;            /*!< [2..2] Execute dual address data read for auto read mode. (1-2-2)         */
    __IOM uint32_t rd_quad_o  : 1;            /*!< [3..3] Execute quad data write for auto read mode. (1-1-4)                */
    __IOM uint32_t rd_quad_io : 1;            /*!< [4..4] Execute quad address data read for auto read mode. (1-4-4)         */
    __IOM uint32_t wr_dual_i  : 1;            /*!< [5..5] Execute dual data write for auto write mode. (1-1-2)               */
    __IOM uint32_t wr_dual_ii : 1;            /*!< [6..6] Execute dual address data write for auto write mode.
                                                   (1-2-2)                                                                   */
    __IOM uint32_t wr_quad_i  : 1;            /*!< [7..7] Execute quad data write for auto write mode. (1-1-4)               */
    __IOM uint32_t wr_quad_ii : 1;            /*!< [8..8] Execute quad address data write for auto write mode.
                                                   (1-4-4)                                                                   */
    __IOM uint32_t wr_blocking : 1;           /*!< [9..9] Accept next operation after the write data push to FIFO
                                                   and FIFO is pop data to empty by SPIC. Should always be
                                                   1.                                                                        */
    __IM  uint32_t            : 1;
    __IOM uint32_t prm_en     : 1;            /*!< [11..11] Enable SPIC performance read mode in auto mode.                  */
    __IOM uint32_t ctrlr0_ch  : 1;            /*!< [12..12] Set this bit, then SPIC refers cmd_ch / data_ch / addr_en
                                                   / cmd_ddr_en / data_ddr_en / addr_ddr_en bit fields in
                                                   Control Register 0 in auto mode.                                          */
    __IM  uint32_t            : 1;
    __IOM uint32_t seq_trans_en : 1;          /*!< [14..14] Set 1 to enable read sequential transaction determination
                                                   function. If the auto read address is sequenctial, users
                                                   can save command phase and address phase under this mode.                 */
  } b;                                        /*!< bit fields for spic_valid_cmd */
} spic_valid_cmd_t, *pspic_valid_cmd_t;

/**
  \brief Union type to access spic_flash_size (@ 0x00000124).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000124) Write Enable Command Register                              */
  
  struct {
    __IOM uint32_t flash_size : 4;            /*!< [3..0] The size of flash size to select the target SPI flash
                                                   in auto mode.                                                             */
  } b;                                        /*!< bit fields for spic_flash_size */
} spic_flash_size_t, *pspic_flash_size_t;

/**
  \brief Union type to access spic_flush_fifo (@ 0x00000128).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000128) Read Status Command Register                               */
  
  struct {
    __OM  uint32_t flush_fifio : 1;           /*!< [0..0] Write to flush SPIC FIFO.                                          */
  } b;                                        /*!< bit fields for spic_flush_fifo */
} spic_flush_fifo_t, *pspic_flush_fifo_t;

/** @} */ /* End of group ls_hal_spic_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SPIC_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_SPIC_TYPE_H_

