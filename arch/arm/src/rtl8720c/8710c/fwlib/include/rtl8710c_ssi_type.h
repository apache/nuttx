/**************************************************************************//**
 * @file      rtl8710c_ssi_type.h
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

#ifndef _RTL8710C_SSI_TYPE_H_
#define _RTL8710C_SSI_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SSI_REG_TYPE

/**
 * @addtogroup hs_hal_ssi_reg SSI Registers.
 * @ingroup hs_hal_ssi
 * @{
 */

/**
  \brief Union type to access ssi_ctrlr0 (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) SSI Control Register 0                                     */
  
  struct {
    __IOM uint32_t dfs        : 4;            /*!< [3..0] Data frame size                                                    */
    __IOM uint32_t frf        : 2;            /*!< [5..4] Frame format                                                       */
    __IOM uint32_t scph       : 1;            /*!< [6..6] Serial Clock Phase. Valid when the frame format (FRF)
                                                   is set to Motorola SPI. The serial clock phase selects
                                                   the relationship of the serial clock with the slave select
                                                   signal. When SCPH = 0, data are captured on the first edge
                                                   of the serial clock. When SCPH = 1, the serial clock starts
                                                   toggling one cycle after the slave select line is activated,
                                                   and data are captured on the second edge of the serial
                                                   clock. 0: Serial clock toggles in middle of first data
                                                   bit 1: Serial clock toggles at start of first data bi                     */
    __IOM uint32_t scpol      : 1;            /*!< [7..7] Serial Clock Polarity. Valid when the frame format (FRF)
                                                   is set to Motorola SPI. Used to select the polarity of
                                                   the inactive serial clock, which is held inactive when
                                                   the DW_apb_ssi master is not actively transferring data
                                                   on the serial bus. 0 : Inactive state of serial clock is
                                                   low 1 : Inactive state of serial clock is high                            */
    __IOM uint32_t tmod       : 2;            /*!< [9..8] Transfer mode                                                      */
    __IOM uint32_t slv_oe     : 1;            /*!< [10..10] Slave ouput enable. Relevant only when the device is
                                                   a slave                                                                   */
    __IM  uint32_t            : 1;
    __IOM uint32_t cfs        : 4;            /*!< [15..12] Control frame size for Microwire frame format                    */
    __IM  uint32_t            : 5;
    __IOM uint32_t tx_byte_swap : 1;          /*!< [21..21] Reverse every transmit byte                                      */
    __IOM uint32_t tx_bit_swap : 1;           /*!< [22..22] Reverse every transmit bit                                       */
    __IOM uint32_t rx_byte_swap : 1;          /*!< [23..23] Reverse every received byte                                      */
    __IOM uint32_t rx_bit_swap : 1;           /*!< [24..24] Reverse every received bit                                       */
    __IM  uint32_t            : 6;
    __IOM uint32_t ss_t       : 1;            /*!< [31..31] SSI master chooses to toggle between successive frames
                                                   or not.                                                                   */
  } b;                                        /*!< bit fields for ssi_ctrlr0 */
} ssi_ctrlr0_t, *pssi_ctrlr0_t;

/**
  \brief Union type to access ssi_ctrlr1 (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) SSI Control Register 1                                     */
  
  struct {
    __IOM uint32_t ndf        : 16;           /*!< [15..0] Number of data frame. Only valid when SSI device is
                                                   a master                                                                  */
  } b;                                        /*!< bit fields for ssi_ctrlr1 */
} ssi_ctrlr1_t, *pssi_ctrlr1_t;

/**
  \brief Union type to access ssi_ssienr (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) SSI Enable Register                                        */
  
  struct {
    __IOM uint32_t ssi_en     : 1;            /*!< [0..0] SSI Enable. Enables and disables all DW_apb_ssi operations.
                                                   When disabled, all serial transfers are halted immediately.
                                                   Transmit and receive FIFO buffers are cleared when the
                                                   device is disabled. It is impossible to program some of
                                                   the DW_apb_ssi control registers when enabled. When disabled,
                                                   the ssi_sleep output is set (after delay) to inform the
                                                   system that it is safe to remove the ssi_clk, thus saving
                                                   power consumption in the system                                           */
  } b;                                        /*!< bit fields for ssi_ssienr */
} ssi_ssienr_t, *pssi_ssienr_t;

/**
  \brief Union type to access ssi_mwcr (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) Microwire Control Register                                 */
  
  struct {
    __IOM uint32_t mwmod      : 1;            /*!< [0..0] Microwire Transfer Mode. Defines the transfer is sequential
                                                   or not.                                                                   */
    __IOM uint32_t mdd        : 1;            /*!< [1..1] Microwire Control. Defines the direction of the data
                                                   word when the Microwire serial protocol is used.                          */
    __IOM uint32_t mhs        : 1;            /*!< [2..2] Microwire Handshaking. Only valide when SSI device is
                                                   a slave.                                                                  */
  } b;                                        /*!< bit fields for ssi_mwcr */
} ssi_mwcr_t, *pssi_mwcr_t;

/**
  \brief Union type to access ssi_ser (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) Slave Enable Register                                      */
  
  struct {
    __IOM uint32_t ser        : 8;            /*!< [7..0] This register is only valid when the device is configured
                                                   as a master device. It is used to select which slave devices
                                                   it would like to communicate with.                                        */
  } b;                                        /*!< bit fields for ssi_ser */
} ssi_ser_t, *pssi_ser_t;

/**
  \brief Union type to access ssi_baudr (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) Baud Rate Select Register                                  */
  
  struct {
    __IOM uint32_t sckdv      : 16;           /*!< [15..0] This register is only valid when the device is configured
                                                   as a master device. It is used to change the operating
                                                   frequency of SPI device.                                                  */
  } b;                                        /*!< bit fields for ssi_baudr */
} ssi_baudr_t, *pssi_baudr_t;

/**
  \brief Union type to access ssi_txftlr (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) Transmit FIFO Threshold Level                              */
  
  struct {
    __IOM uint32_t tft        : 8;            /*!< [7..0] Transmit FIFO Threshold. Controls the level of entries
                                                   (or below) at which the transmit FIFO controller triggers
                                                   an interrupt.                                                             */
  } b;                                        /*!< bit fields for ssi_txftlr */
} ssi_txftlr_t, *pssi_txftlr_t;

/**
  \brief Union type to access ssi_rxftlr (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) Receive FIFO Threshold Level                               */
  
  struct {
    __IOM uint32_t rft        : 8;            /*!< [7..0] Receive FIFO Threshold. Controls the level of entries
                                                   (or above) at which the receive FIFO controller triggers
                                                   an interrupt.                                                             */
  } b;                                        /*!< bit fields for ssi_rxftlr */
} ssi_rxftlr_t, *pssi_rxftlr_t;

/**
  \brief Union type to access ssi_txflr (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) Transmit FIFO Level Register                               */
  
  struct {
    __IM  uint32_t txtfl      : 8;            /*!< [7..0] Transmit FIFO Level. Contains the number of valid data
                                                   entries in the transmit FIFO.                                             */
  } b;                                        /*!< bit fields for ssi_txflr */
} ssi_txflr_t, *pssi_txflr_t;

/**
  \brief Union type to access ssi_rxflr (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000024) Receive FIFO Level Register                                */
  
  struct {
    __IM  uint32_t rxtfl      : 8;            /*!< [7..0] Receive FIFO Level. Contains the number of valid data
                                                   entries in the receive FIFO.                                              */
  } b;                                        /*!< bit fields for ssi_rxflr */
} ssi_rxflr_t, *pssi_rxflr_t;

/**
  \brief Union type to access ssi_sr (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) Status Register                                            */
  
  struct {
    __IM  uint32_t busy       : 1;            /*!< [0..0] SSI Busy Flag. When set, indicates that a serial transfer
                                                   is in progress; when cleared indicates that the DW_apb_ssi
                                                   is idle or disabled.                                                      */
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
  } b;                                        /*!< bit fields for ssi_sr */
} ssi_sr_t, *pssi_sr_t;

/**
  \brief Union type to access ssi_imr (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) Interrupt Mask Register                                    */
  
  struct {
    __IOM uint32_t txeim      : 1;            /*!< [0..0] Transmit FIFO Empty Interrupt Mask 0 : ssi_txe_intr interrupt
                                                   is masked 1 : ssi_txe_intr interrupt is not masked                        */
    __IOM uint32_t txoim      : 1;            /*!< [1..1] Transmit FIFO Overflow Interrupt Mask 0 : ssi_txo_intr
                                                   interrupt is masked 1 : ssi_txo_intr interrupt is not masked              */
    __IOM uint32_t rxuim      : 1;            /*!< [2..2] Receive FIFO Underflow Interrupt Mask 0 : ssi_rxu_intr
                                                   interrupt is masked 1 : ssi_rxu_intr interrupt is not masked              */
    __IOM uint32_t rxoim      : 1;            /*!< [3..3] Receive FIFO Overflow Interrupt Mask 0 : ssi_rxo_intr
                                                   interrupt is masked 1 : ssi_rxo_intr interrupt is not masked              */
    __IOM uint32_t rxfim      : 1;            /*!< [4..4] Receive FIFO Full Interrupt Mask 0 : ssi_rxf_intr interrupt
                                                   is masked 1 : ssi_rxf_intr interrupt is not masked                        */
    __IOM uint32_t mstim      : 1;            /*!< [5..5] Multi-Master Contention Interrupt Mask. This bit field
                                                   is not present if the deice is a slave. 0 : ssi_mst_intr
                                                   interrupt is masked 1 : ssi_mst_intr interrupt is not masked              */
    __IOM uint32_t txuim      : 1;            /*!< [6..6] Transmit FIFO Under Flow Interrupt Status. Should not
                                                   be set when the device is a master and not under SPI mode.
                                                   0 : ssi_txu_intr interrupt is masked 1 : ssi_txu_intr interrupt
                                                   is not masked                                                             */
    __IOM uint32_t ssrim      : 1;            /*!< [7..7] SS_N Rising Edge Detect Interrupt Mask 0 : ssi_ssr_intr
                                                   interrupt is masked 1 : ssi_ssr_intr interrupt is not masked              */
  } b;                                        /*!< bit fields for ssi_imr */
} ssi_imr_t, *pssi_imr_t;

/**
  \brief Union type to access ssi_isr (@ 0x00000030).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000030) Interrupt Status Register                                  */
  
  struct {
    __IM  uint32_t txeis      : 1;            /*!< [0..0] Transmit FIFO Empty Interrupt Status 0 = ssi_txe_intr
                                                   interrupt is not active after masking 1 = ssi_txe_intr
                                                   interrupt is active after masking                                         */
    __IM  uint32_t txois      : 1;            /*!< [1..1] Transmit FIFO Overflow Interrupt Status 0 = ssi_txo_intr
                                                   interrupt is not active after masking 1 = ssi_txo_intr
                                                   interrupt is active after masking                                         */
    __IM  uint32_t rxuis      : 1;            /*!< [2..2] Receive FIFO Underflow Interrupt Status 0 = ssi_rxu_intr
                                                   interrupt is not active after masking 1 = ssi_rxu_intr
                                                   interrupt is active after masking                                         */
    __IM  uint32_t rxois      : 1;            /*!< [3..3] Receive FIFO Overflow Interrupt Status 0 = ssi_rxo_intr
                                                   interrupt is not active after masking 1 = ssi_rxo_intr
                                                   interrupt is active after masking                                         */
    __IM  uint32_t rxfis      : 1;            /*!< [4..4] Receive FIFO Full Interrupt Status 0 = ssi_rxf_intr interrupt
                                                   is not active after masking 1 = ssi_rxf_intr interrupt
                                                   is full after masking                                                     */
    __IM  uint32_t mstis      : 1;            /*!< [5..5] Multi-Master Contention Interrupt. This bit field is
                                                   not present if the deice is a slave. 0 = ssi_mst_intr interrupt
                                                   not active after masking 1 = ssi_mst_intr interrupt is
                                                   active after masking                                                      */
    __IM  uint32_t txuis      : 1;            /*!< [6..6] Transmit FIFO Under Flow Interrupt Status 0 = ssi_txu_intr
                                                   interrupt is not active after masking 1 = ssi_txu_intr
                                                   interrupt is active after masking                                         */
    __IM  uint32_t ssris      : 1;            /*!< [7..7] SS_N Rising Edge Detect Interrupt Status 0 = ssi_ssr_intr
                                                   interrupt is not active after masking 1 = ssi_ssr_intr
                                                   interrupt is active after masking                                         */
  } b;                                        /*!< bit fields for ssi_isr */
} ssi_isr_t, *pssi_isr_t;

/**
  \brief Union type to access ssi_risr (@ 0x00000034).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000034) Raw Interrupt Status Register                              */
  
  struct {
    __IM  uint32_t txeir      : 1;            /*!< [0..0] Transmit FIFO Empty Raw Interrupt Status 0 = ssi_txe_intr
                                                   interrupt is not active prior to masking 1 = ssi_txe_intr
                                                   interrupt is active prior to masking                                      */
    __IM  uint32_t txoir      : 1;            /*!< [1..1] Transmit FIFO Overflow Raw Interrupt Status 0 = ssi_txo_intr
                                                   interrupt is not active prior to masking 1 = ssi_txo_intr
                                                   interrupt is active prior to masking                                      */
    __IM  uint32_t rxuir      : 1;            /*!< [2..2] Receive FIFO Underflow Raw Interrupt Status 0 = ssi_rxu_intr
                                                   interrupt is not active prior to masking 1 = ssi_rxu_intr
                                                   interrupt is active prior to masking                                      */
    __IM  uint32_t rxoir      : 1;            /*!< [3..3] Receive FIFO Overflow Raw Interrupt Status 0 = ssi_rxo_intr
                                                   interrupt is not active prior to masking 1 = ssi_rxo_intr
                                                   interrupt is active prior to masking                                      */
    __IM  uint32_t rxfir      : 1;            /*!< [4..4] Receive FIFO Full Raw Interrupt Status 0 = ssi_rxf_intr
                                                   interrupt is not active prior to masking 1 = ssi_rxf_intr
                                                   interrupt is full prior to masking                                        */
    __IM  uint32_t mstir      : 1;            /*!< [5..5] Multi-Master Contention Raw Interrupt Status. This bit
                                                   field is not present if the deice is a slave. 0 = ssi_mst_intr
                                                   interrupt not active prior to masking 1 = ssi_mst_intr
                                                   interrupt is active prior to masking                                      */
    __IM  uint32_t txuir      : 1;            /*!< [6..6] Transmit FIFO Under Flow Raw Interrupt Status 0 = ssi_txu_intr
                                                   interrupt is not active prior to masking 1 = ssi_txu_intr
                                                   interrupt is active prior to masking                                      */
    __IM  uint32_t ssrir      : 1;            /*!< [7..7] SS_N Rising Edge Detect Raw Interrupt Status 0 = ssi_ssr_intr
                                                   interrupt is not active prior to masking 1 = ssi_ssr_intr
                                                   interrupt is active prior to masking                                      */
  } b;                                        /*!< bit fields for ssi_risr */
} ssi_risr_t, *pssi_risr_t;

/**
  \brief Union type to access ssi_txoicr (@ 0x00000038).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000038) Transmit FIFO Overflow Interrupt Clear Register            */
  
  struct {
    __IM  uint32_t txoicr     : 1;            /*!< [0..0] Clear Transmit FIFO Overflow Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_txo_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for ssi_txoicr */
} ssi_txoicr_t, *pssi_txoicr_t;

/**
  \brief Union type to access ssi_rxoicr (@ 0x0000003C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000003C) Receive FIFO Overflow Interrupt Clear Register             */
  
  struct {
    __IM  uint32_t rxoicr     : 1;            /*!< [0..0] Clear Receive FIFO Overflow Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_rxo_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for ssi_rxoicr */
} ssi_rxoicr_t, *pssi_rxoicr_t;

/**
  \brief Union type to access ssi_rxuicr (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) Receive FIFO Underflow Interrupt Clear Register            */
  
  struct {
    __IM  uint32_t rxuicr     : 1;            /*!< [0..0] Clear Receive FIFO Underflow Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_rxu_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for ssi_rxuicr */
} ssi_rxuicr_t, *pssi_rxuicr_t;

/**
  \brief Union type to access ssi_msticr (@ 0x00000044).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000044) Multi-Master Interrupt Clear Register                      */
  
  struct {
    __IM  uint32_t msticr     : 1;            /*!< [0..0] Clear Multi-Master Contention Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_mst_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for ssi_msticr */
} ssi_msticr_t, *pssi_msticr_t;

/**
  \brief Union type to access ssi_icr (@ 0x00000048).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000048) Interrupt Clear Register                                   */
  
  struct {
    __IM  uint32_t icr        : 1;            /*!< [0..0] Clear Interrupts. This register is set if any of the
                                                   interrupts below are active. A read clears the ssi_txu_intr,
                                                   ssi_txo_intr, ssi_rxu_intr, ssi_rxo_intr, and ssi_ssr_intr
                                                   interrupts. Writing to this register has no effect.                       */
  } b;                                        /*!< bit fields for ssi_icr */
} ssi_icr_t, *pssi_icr_t;

/**
  \brief Union type to access ssi_dmacr (@ 0x0000004C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000004C) DMA Control Register                                       */
  
  struct {
    __IOM uint32_t rdmae      : 1;            /*!< [0..0] Receive DMA Enable. This bit enables/disables the receive
                                                   FIFO DMA channel 0 : Receive DMA disabled 1 : Receive DMA
                                                   enabled                                                                   */
    __IOM uint32_t tdmae      : 1;            /*!< [1..1] Transmit DMA Enable. This bit enables/disables the transmit
                                                   FIFO DMA channel. 0 : Transmit DMA disabled 1 : Transmit
                                                   DMA enabled                                                               */
  } b;                                        /*!< bit fields for ssi_dmacr */
} ssi_dmacr_t, *pssi_dmacr_t;

/**
  \brief Union type to access ssi_dmatdlr (@ 0x00000050).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000050) DMA Transmit Data Level Register                           */
  
  struct {
    __IOM uint32_t dmatdl     : 8;            /*!< [7..0] Transmit Data Level. This bit field controls the level
                                                   at which a DMA request is made by the transmit logic. It
                                                   is equal to the watermark level; that is, the dma_tx_req
                                                   signal is generated when the number of valid data entries
                                                   in the transmit FIFO is equal to or below this field value,
                                                   and TDMAE = 1.                                                            */
  } b;                                        /*!< bit fields for ssi_dmatdlr */
} ssi_dmatdlr_t, *pssi_dmatdlr_t;

/**
  \brief Union type to access ssi_dmardlr (@ 0x00000054).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000054) DMA Receive Data Level Register                            */
  
  struct {
    __IM  uint32_t            : 7;
    __IOM uint32_t dmardl     : 1;            /*!< [7..7] Receive Data Level. This bit field controls the level
                                                   at which a DMA request is made by the receive logic. The
                                                   watermark level = DMARDL+1; that is, dma_rx_req is generated
                                                   when the number of valid data entries in the receive FIFO
                                                   is equal to or above this field value + 1, and RDMAE=1.                   */
  } b;                                        /*!< bit fields for ssi_dmardlr */
} ssi_dmardlr_t, *pssi_dmardlr_t;

/**
  \brief Union type to access ssi_txuicr (@ 0x00000058).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000058) Transmit FIFO Underflow Interrupt Clear Register           */
  
  struct {
    __IM  uint32_t txuicr     : 1;            /*!< [0..0] Clear Transmit FIFO Underflow Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_txu_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for ssi_txuicr */
} ssi_txuicr_t, *pssi_txuicr_t;

/**
  \brief Union type to access ssi_ssricr (@ 0x0000005C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000005C) SS_N Rising Edge Detect Interrupt Clear Register           */
  
  struct {
    __IM  uint32_t ssiicr     : 1;            /*!< [0..0] Clear SS_N Rinsing Edge Detect Interrupt. This register
                                                   reflects the status of the interrupt. A read from this
                                                   register clears the ssi_ssr_intr interrupt; writing has
                                                   no effect.                                                                */
  } b;                                        /*!< bit fields for ssi_ssricr */
} ssi_ssricr_t, *pssi_ssricr_t;

/**
  \brief Union type to access ssi_dr (@ 0x00000060).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x00000060) Data Register                                              */
  
  struct {
    __IOM uint16_t dr         : 16;           /*!< [15..0] Data register is a 16-bit read/write buffer for the
                                                   transmit/receive FIFOs. When the register is read, data
                                                   in the receive FIFO buffer is accessed. When it is written
                                                   to, data are moved into the transmit FIFO buffer; a write
                                                   can occur only when SSI_EN = 1. FIFOs are reset when SSI_EN
                                                   = 0.When writing to this register, you must right-justify
                                                   the data. Read data are automatically right-justified.
                                                   Read : Receive FIFO buffer Write : Transmit FIFO buffer                   */
  } b;                                        /*!< bit fields for ssi_dr */
} ssi_dr_t, *pssi_dr_t;

/**
  \brief Union type to access ssi_rx_sample_dly (@ 0x000000F0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F0) Rx Sample Delay Register                                   */
  
  struct {
    __IOM uint32_t rsd        : 8;            /*!< [7..0] Receive Data Sample Delay. This register is used to delay
                                                   the sample of the rxd input signal. Each value represents
                                                   a single ssi_clk delay on the sample of the rxd signal.
                                                   If this register is programmed with a value that exceed
                                                   the depth of the internal shift register, a zero delay
                                                   will be applied to the rxd sample.                                        */
  } b;                                        /*!< bit fields for ssi_rx_sample_dly */
} ssi_rx_sample_dly_t, *pssi_rx_sample_dly_t;

/** @} */ /* End of group ls_hal_ssi_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SSI_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_SSI_TYPE_H_

