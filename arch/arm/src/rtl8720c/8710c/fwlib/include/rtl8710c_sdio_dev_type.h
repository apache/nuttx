/**************************************************************************//**
 * @file      rtl8710c_sdio_dev_type.h
 * @brief
 * @version   V1.00
 * @date      2018-6-4 21:34:55
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

#ifndef _RTL8710C_SDIO_DEV_TYPE_H_
#define _RTL8710C_SDIO_DEV_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SDIO_DEV_REG_TYPE

/**
 * @addtogroup hal_sdio_dev_reg SDIO_DEV Registers.
 * @ingroup hs_hal_sdio_dev
 * @{
 */

/**
  \brief Union type to access sdio_dev_txbd_addr (@ 0x000000A0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000A0) TX BD(Buffer Descriptor) Address Register                  */
  
  struct {
    __IOM uint32_t tx_bd_addr : 32;           /*!< [31..0] The base address of TXBD The value of this register
                                                   should be configure by CPU It should be 4 byte alignment
                                                   (Because TXBD Size is 8 Byte).                                            */
  } b;                                        /*!< bit fields for sdio_dev_txbd_addr */
} sdio_dev_txbd_addr_t, *psdio_dev_txbd_addr_t;

/**
  \brief Union type to access sdio_dev_txbd_num (@ 0x000000A4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000A4) TX BD Number Register                                      */
  
  struct {
    __IOM uint32_t tx_bd_num  : 16;           /*!< [15..0] The Number of TXBD. Unit: Number The value of this register
                                                   should be configured at SDIO device HAL initialization                    */
  } b;                                        /*!< bit fields for sdio_dev_txbd_num */
} sdio_dev_txbd_num_t, *psdio_dev_txbd_num_t;

/**
  \brief Union type to access sdio_dev_txbd_wptr (@ 0x000000A8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000A8) TX BD Write Pointer Register                               */
  
  struct {
    __IOM uint32_t tx_bd_h2c_wptr : 16;       /*!< [15..0] When the packet has been sent from Host to TX Buffer,
                                                   this index should be updated by SDIO Device HAL driver                    */
  } b;                                        /*!< bit fields for sdio_dev_txbd_wptr */
} sdio_dev_txbd_wptr_t, *psdio_dev_txbd_wptr_t;

/**
  \brief Union type to access sdio_dev_txbd_rptr (@ 0x000000AC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000AC) TX BD Read Pointer Register                                */
  
  struct {
    __IOM uint32_t tx_bd_h2c_rptr : 16;       /*!< [15..0] When the TX packet has been processed and moved to MAC
                                                   TX FIFO, this index should be updated by HAL driver.                      */
  } b;                                        /*!< bit fields for sdio_dev_txbd_rptr */
} sdio_dev_txbd_rptr_t, *psdio_dev_txbd_rptr_t;

/**
  \brief Union type to access sdio_dev_rxbd_addr (@ 0x000000B0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000B0) RX BD Address Register                                     */
  
  struct {
    __IOM uint32_t rx_bd_addr : 32;           /*!< [31..0] The base address of RX BD. The HAL driver should configure
                                                   this register at initial. It should be a 8-Byte aligned
                                                   address.                                                                  */
  } b;                                        /*!< bit fields for sdio_dev_rxbd_addr */
} sdio_dev_rxbd_addr_t, *psdio_dev_rxbd_addr_t;

/**
  \brief Union type to access sdio_dev_rxbd_num (@ 0x000000B4).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000B4) RX BD Number Register                                      */
  
  struct {
    __IOM uint16_t rx_bd_num  : 16;           /*!< [15..0] The total number of RX BD This register should be configured
                                                   by HAL driver at initialization.                                          */
  } b;                                        /*!< bit fields for sdio_dev_rxbd_num */
} sdio_dev_rxbd_num_t, *psdio_dev_rxbd_num_t;

/**
  \brief Union type to access sdio_dev_rxbd_wptr (@ 0x000000B6).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000B6) RX BD Write Pointer Register                               */
  
  struct {
    __IOM uint16_t rx_bd_c2h_wptr : 16;       /*!< [15..0] When the packet has been processed and has been moved
                                                   to host TX FIFO, this index should be updated by the HAL
                                                   driver.                                                                   */
  } b;                                        /*!< bit fields for sdio_dev_rxbd_wptr */
} sdio_dev_rxbd_wptr_t, *psdio_dev_rxbd_wptr_t;

/**
  \brief Union type to access sdio_dev_rxbd_rptr (@ 0x000000B8).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000B8) RX BD Read Pointer Register                                */
  
  struct {
    __IOM uint16_t rx_bd_c2h_rptr : 16;       /*!< [15..0] When the packet has been processed and has been transfered
                                                   to host, the HW will update this RX BD index.                             */
  } b;                                        /*!< bit fields for sdio_dev_rxbd_rptr */
} sdio_dev_rxbd_rptr_t, *psdio_dev_rxbd_rptr_t;

/**
  \brief Union type to access sdio_dev_hci_rx_ctrl (@ 0x000000BA).
*/
typedef union {
  __IOM uint8_t w;                            /*!< (@ 0x000000BA) HCI RX Control Register                                    */
  
  struct {
    __IOM uint8_t rx_req      : 1;            /*!< [0..0] HAL driver triggers this bit to enable SDIO IP HW to
                                                   fetch RX BD info. SDIO HW fetch RX BD to get the RX length
                                                   and address and then start to transfer RX packet to SDIO
                                                   Host. This bit will be cleared when all RX BD transfer
                                                   is done.                                                                  */
  } b;                                        /*!< bit fields for sdio_dev_hci_rx_ctrl */
} sdio_dev_hci_rx_ctrl_t, *psdio_dev_hci_rx_ctrl_t;

/**
  \brief Union type to access sdio_dev_sdio_ctrl (@ 0x000000BB).
*/
typedef union {
  __IOM uint8_t w;                            /*!< (@ 0x000000BB) SDIO HW Control Register                                   */
  
  struct {
    __IOM uint8_t sdio_dat_edge_inv : 1;      /*!< [0..0] Set this bit to invert the SDIO data latch clock.                  */
    __IM  uint8_t             : 6;
    __IOM uint8_t sdio_dma_rst : 1;           /*!< [7..7] The HAL driver can set this bit to reset SDIO DMA HW.
                                                   This bit will be cleared automatically by the HW.                         */
  } b;                                        /*!< bit fields for sdio_dev_sdio_ctrl */
} sdio_dev_sdio_ctrl_t, *psdio_dev_sdio_ctrl_t;

/**
  \brief Union type to access sdio_dev_rx_req_addr (@ 0x000000BC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000BC) RX Request Address Register                                */
  
  struct {
    __IOM uint32_t rx_req_addr : 32;          /*!< [31..0] The address of the requested RX packet. This register
                                                   is discarded for Ameba.                                                   */
  } b;                                        /*!< bit fields for sdio_dev_rx_req_addr */
} sdio_dev_rx_req_addr_t, *psdio_dev_rx_req_addr_t;

/**
  \brief Union type to access sdio_dev_int_mask (@ 0x000000C0).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000C0) Interrupt Mask Register                                    */
  
  struct {
    __IOM uint16_t txbd_h2c_ovf : 1;          /*!< [0..0] Set this bit to enable the H2C TX BD overflow interrupt.           */
    __IOM uint16_t h2c_bus_res_fail : 1;      /*!< [1..1] Set this bit to enable the H2C bus resource failed interrupt.      */
    __IOM uint16_t h2c_dma_ok : 1;            /*!< [2..2] Set this bit to enable the H2C DMA done interrupt.                 */
    __IOM uint16_t c2h_dma_ok : 1;            /*!< [3..3] Set this bit to enable the C2H DMA done interrupt.                 */
    __IOM uint16_t h2c_msg    : 1;            /*!< [4..4] Set this bit to enable the H2C message interrupt.                  */
    __IOM uint16_t rpwm1      : 1;            /*!< [5..5] Set this bit to enable the RPWM interrupt.                         */
    __IOM uint16_t rpwm2      : 1;            /*!< [6..6] Set this bit to enable the RPWM2 interrupt.                        */
    __IOM uint16_t sdio_rst_cmd : 1;          /*!< [7..7] Set this bit to enable the SDIO reset command interrupt            */
    __IOM uint16_t rx_bd_err  : 1;            /*!< [8..8] Set this bit to enable the RX BD error interrupt.                  */
    __IOM uint16_t rx_bd_avai : 1;            /*!< [9..9] Set this bit to enable the RX BD available interrupt.              */
    __IOM uint16_t host_wake  : 1;            /*!< [10..10] Set this bit to enable the SDIO host wake event interrupt.       */
    __IOM uint16_t host_cmd11 : 1;            /*!< [11..11] Set this bit to enable the SDIO host CMD11(3.3v ->
                                                   1.8v) interrupt.                                                          */
  } b;                                        /*!< bit fields for sdio_dev_int_mask */
} sdio_dev_int_mask_t, *psdio_dev_int_mask_t;

/**
  \brief Union type to access sdio_dev_int_sts (@ 0x000000C2).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000C2) Interrupt Status Register                                  */
  
  struct {
    __IOM uint16_t txbd_h2c_ovf : 1;          /*!< [0..0] If there is not enough TX_BD for TX Packet, this interrupt
                                                   will be raised. The HAL driver write 1 to this bit to clear
                                                   this interript status.                                                    */
    __IOM uint16_t h2c_bus_res_fail : 1;      /*!< [1..1] If SDIO is going to move packet to TX FIFO and fails
                                                   to get bus resource, this interrupt raise. HAL driver write
                                                   1 to this bit to clear this interrupt status.                             */
    __IOM uint16_t h2c_dma_ok : 1;            /*!< [2..2] Trigger by SDIO HW for the event of packet is sent from
                                                   Host to TXFIFO                                                            */
    __IOM uint16_t c2h_dma_ok : 1;            /*!< [3..3] Trigger by SDIO HW for the event of packet is sent from
                                                   RXFIFO to Host                                                            */
    __IOM uint16_t h2c_msg    : 1;            /*!< [4..4] Trigger by SDIO HW for the event of H2C_MSG_INT occurs(REG_SDIO_H2C_MSG
                                                   is written)                                                               */
    __IOM uint16_t rpwm1      : 1;            /*!< [5..5] Trigger by SDIO HW for the event of the RPWM1 occurs(HRPWM2
                                                   is written) CPU should be able to receive this interrupt
                                                   even CPU clock is gated; therefore, this interrupt should
                                                   be connected to System On circuit.                                        */
    __IOM uint16_t rpwm2      : 1;            /*!< [6..6] Trigger by SDIO HW for the event of RPWM2 occurs (HRPWM
                                                   is toggled) CPU should be able to receive this interrupt
                                                   even CPU clock is gated; therefore, this interrupt should
                                                   be connected to System On circuit.                                        */
    __IOM uint16_t sdio_rst_cmd : 1;          /*!< [7..7] Trigger by SDIO HW when SDIO is reset. CPU should be
                                                   able to receive this interrupt even CPU clock is gated;
                                                   therefore, this interrupt should be connected to System
                                                   On circuit.                                                               */
    __IOM uint16_t rx_bd_err  : 1;            /*!< [8..8] Trigger by SDIO to CPU when detect RX_BD error                     */
    __IOM uint16_t rx_bd_avai : 1;            /*!< [9..9] If the free RXBD Number become larger than FREE_RXBD_COUNT(0xD8),
                                                   trigger this interrupt. This interrupt trigger only once
                                                   when free RXBD number cross FREE_RXBD_COUNT                               */
    __IOM uint16_t host_wake  : 1;            /*!< [10..10] When Host Send TRX CMD53 while CPU is not ready (SYSTEM_CPU_RDY_IND=0
                                                   or CPU_RDY_IND=0), trigger this interrupt to wake CPU,
                                                   and then indicate BUSY status to host. CPU should be able
                                                   to receive this interrupt even CPU clock is gated; therefore,
                                                   this interrupt should be connected to System On circuit.                  */
    __IOM uint16_t host_cmd11 : 1;            /*!< [11..11] When Host sends CMD11 to notify that the signal voltage
                                                   level is going to switch to 1.8v, the HW should issue this
                                                   interrupt to notify the local CPU to do the LDO power voltage
                                                   level switch.                                                             */
  } b;                                        /*!< bit fields for sdio_dev_int_sts */
} sdio_dev_int_sts_t, *psdio_dev_int_sts_t;

/**
  \brief Union type to access sdio_dev_ccpwm (@ 0x000000C4).
*/
typedef union {
  __IOM uint8_t w;                            /*!< (@ 0x000000C4) CCPWM Register (CPU domain Sync to HCPWM register)         */
  
  struct {
    __IM  uint8_t             : 1;
    __IOM uint8_t wlan_trx    : 1;            /*!< [1..1] 1: WLAN On; 0: WLAN Off.                                           */
    __IOM uint8_t rps_st      : 1;            /*!< [2..2] 1: AP Register Active State; 0: AP Register Sleep State;           */
    __IOM uint8_t wwlan       : 1;            /*!< [3..3] 1: Wake On WLAN State; 0: Normal State;                            */
    __IM  uint8_t             : 3;
    __IOM uint8_t toggle      : 1;            /*!< [7..7] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_ccpwm */
} sdio_dev_ccpwm_t, *psdio_dev_ccpwm_t;

/**
  \brief Union type to access sdio_dev_sys_ind (@ 0x000000C5).
*/
typedef union {
  __IOM uint8_t w;                            /*!< (@ 0x000000C5) System Indication Register                                 */
  
  struct {
    __IOM uint8_t sys_cpu_rdy_ind : 1;        /*!< [0..0] Used to indicate the SDIO NIC driver that local CPU is
                                                   ready for TRX. Default: 0 This bit is synchronized to CPU_RDY_IND(Offset
                                                   0x87)                                                                     */
  } b;                                        /*!< bit fields for sdio_dev_sys_ind */
} sdio_dev_sys_ind_t, *psdio_dev_sys_ind_t;

/**
  \brief Union type to access sdio_dev_ccpwm2 (@ 0x000000C6).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000C6) CCPWM2 Register (CPU domain Sync to HCPWM2 register)       */
  
  struct {
    __IOM uint16_t active     : 1;            /*!< [0..0] 1: indicate that the SDIO device is in the activate state.         */
    __IOM uint16_t dstandby   : 1;            /*!< [1..1] 1: indicates that the SDIO device is in the deep-standby
                                                   state.                                                                    */
    __IOM uint16_t fboot      : 1;            /*!< [2..2] 1: indicates that the SDIO device boot from the fast
                                                   boot                                                                      */
    __IOM uint16_t inic_fw_rdy : 1;           /*!< [3..3] Indicates the FW mode of the SDIO device: 1: iNIC firmware;
                                                   0: SDIO boot firmware.                                                    */
    __IM  uint16_t            : 11;
    __IOM uint16_t toggle     : 1;            /*!< [15..15] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_ccpwm2 */
} sdio_dev_ccpwm2_t, *psdio_dev_ccpwm2_t;

/**
  \brief Union type to access sdio_dev_h2c_msg (@ 0x000000C8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000C8) CPU H2C Message Register (CPU domain Sync from
                                                                  SDIO_H2C_MSG register)                                     */
  
  struct {
    __IM  uint32_t h2c_msg    : 31;           /*!< [30..0] The message from SDIO host side driver.                           */
    __IM  uint32_t toggle     : 1;            /*!< [31..31] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_h2c_msg */
} sdio_dev_h2c_msg_t, *psdio_dev_h2c_msg_t;

/**
  \brief Union type to access sdio_dev_c2h_msg (@ 0x000000CC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000CC) CPU C2H Message Register (CPU domain Sync to
                                                                  SDIO_C2H_MSG register)                                     */
  
  struct {
    __IOM uint32_t c2h_msg    : 31;           /*!< [30..0] The message from local HAL driver to SDIO host side
                                                   driver.                                                                   */
    __IOM uint32_t toggle     : 1;            /*!< [31..31] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_c2h_msg */
} sdio_dev_c2h_msg_t, *psdio_dev_c2h_msg_t;

/**
  \brief Union type to access sdio_dev_crpwm (@ 0x000000D0).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000D0) CRPWM Register (SDIO host side driver to local
                                                                  HAL driver, Sync from HRPWM register)                      */
  
  struct {
    __IM  uint16_t            : 1;
    __IM  uint16_t wlan_trx   : 1;            /*!< [1..1] 1: WLAN On; 0: WLAN Off.                                           */
    __IM  uint16_t rps_st     : 1;            /*!< [2..2] 1: AP Register Active State; 0: AP Register Sleep State;           */
    __IM  uint16_t wwlan      : 1;            /*!< [3..3] 1: Wake On WLAN State; 0: Normal State;                            */
    __IM  uint16_t            : 3;
    __IM  uint16_t toggle     : 1;            /*!< [7..7] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_crpwm */
} sdio_dev_crpwm_t, *psdio_dev_crpwm_t;

/**
  \brief Union type to access sdio_dev_crpwm2 (@ 0x000000D2).
*/
typedef union {
  __IOM uint16_t w;                           /*!< (@ 0x000000D2) CRPWM2 Register (SDIO host side driver to local
                                                                  HAL driver, Sync from HRPWM register)                      */
  
  struct {
    __IM  uint16_t resv       : 15;           /*!< [14..0] Reserved                                                          */
    __IM  uint16_t toggle     : 1;            /*!< [15..15] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_crpwm2 */
} sdio_dev_crpwm2_t, *psdio_dev_crpwm2_t;

/**
  \brief Union type to access sdio_dev_ahb_dma_ctrl (@ 0x000000D4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000D4) AHB DMA Control Register                                   */
  
  struct {
    __IOM uint32_t spdio_txff_wlevel : 7;     /*!< [6..0] SPDIO FIFO TX water level Range 1~126, it can be modified
                                                   only when AHB_DMA_CS = 000                                                */
    __IM  uint32_t            : 1;
    __IOM uint32_t spdio_rxff_wlevel : 7;     /*!< [14..8] SPDIO FIFO RX water level Range 1~126, it can be modified
                                                   only when AHB_DMA_CS = 000                                                */
    __IM  uint32_t            : 1;
    __IM  uint32_t ahb_dma_state : 4;         /*!< [19..16] AHB DMA state                                                    */
    __IM  uint32_t            : 1;
    __IM  uint32_t ahbm_spdio_ready : 1;      /*!< [21..21] SPDIO AHB Master HREADY signal                                   */
    __IM  uint32_t ahbm_spdio_trans : 2;      /*!< [23..22] SPDIO AHB Master HTRANS signal                                   */
    __IOM uint32_t rx_ahb_busy_wait_cnt : 4;  /*!< [27..24] When SPDIO RX transfer, AHB controller will wait BUSY
                                                   counter for AHB access target not response READY signal.
                                                   If timeout, AHB controller will issue AHB_BUS_RES_FAIL
                                                   INT to CPU.                                                               */
    __IOM uint32_t ahb_burst_type : 3;        /*!< [30..28] This field is used to control to SPDIO AHB support
                                                   Burst type. 3'b100: Support Burst 16 DW 3'b010: Support
                                                   Burst 8 DW 3'b001: Support Burst 4 DW Default is 3'b111,
                                                   all support.                                                              */
    __IOM uint32_t dispatch_txagg_pkt : 1;    /*!< [31..31] Enable SPDIO to dispatch the Aggregated TX packet.               */
  } b;                                        /*!< bit fields for sdio_dev_ahb_dma_ctrl */
} sdio_dev_ahb_dma_ctrl_t, *psdio_dev_ahb_dma_ctrl_t;

/**
  \brief Union type to access sdio_dev_free_rxbd_cnt (@ 0x000000D8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000D8) Free RX BD Count Register                                  */
  
  struct {
    __IOM uint32_t fifo_cnt   : 7;            /*!< [6..0] The SPDIO Local FIFO counter, for debug usage.                     */
    __IM  uint32_t            : 1;
    __IOM uint32_t tx_buf_unit_size : 8;      /*!< [15..8] The Size of each single TX Buffer which is addressed
                                                   by TX_BD Unit: 64Byte Ex: 0x01=>64Byte 0x10=>1024Byte                     */
    __IOM uint32_t free_rx_bd_cnt : 16;       /*!< [31..16] If SPDIO_RXBD_C2H_RPTR is updated and the free RXBD
                                                   Number is larger than FREE_RXBD_COUNT, trigger RX_BD_AVAI_INT
                                                   interrupt.                                                                */
  } b;                                        /*!< bit fields for sdio_dev_free_rxbd_cnt */
} sdio_dev_free_rxbd_cnt_t, *psdio_dev_free_rxbd_cnt_t;

/**
  \brief Union type to access sdio_dev_h2c_msg_ext (@ 0x000000DC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000DC) CPU H2C Extension Message Register (CPU domain
                                                                  Sync from SDIO_H2C_MSG_EXT register)                       */
  
  struct {
    __IM  uint32_t h2c_msg_ext : 31;          /*!< [30..0] The message from SDIO host side driver to the local
                                                   HAL driver.                                                               */
    __IM  uint32_t toggle     : 1;            /*!< [31..31] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_h2c_msg_ext */
} sdio_dev_h2c_msg_ext_t, *psdio_dev_h2c_msg_ext_t;

/**
  \brief Union type to access sdio_dev_c2h_msg_ext (@ 0x000000E0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000E0) CPU C2H Extension Message Register (CPU domain
                                                                  Sync to SDIO_C2H_MSG_EXT register)                         */
  
  struct {
    __IOM uint32_t c2h_msg_ext : 31;          /*!< [30..0] The message from local HAL driver to SDIO host side
                                                   driver.                                                                   */
    __IOM uint32_t toggle     : 1;            /*!< [31..31] Toggling Bit: this is the one bit sequence number field.
                                                   Interrupt is issued when this bit is changed from 1 to
                                                   0 or from 0 to 1.                                                         */
  } b;                                        /*!< bit fields for sdio_dev_c2h_msg_ext */
} sdio_dev_c2h_msg_ext_t, *psdio_dev_c2h_msg_ext_t;

/** @} */ /* End of group ls_hal_sdio_dev_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SDIO_DEV_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_SDIO_DEV_TYPE_H_

