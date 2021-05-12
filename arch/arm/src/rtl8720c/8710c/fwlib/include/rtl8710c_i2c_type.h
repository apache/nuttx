/**************************************************************************//**
 * @file      rtl8710c_i2c_type.h
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

#ifndef _RTL8710C_I2C_TYPE_H_
#define _RTL8710C_I2C_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_I2C_REG_TYPE

/**
 * @addtogroup hs_hal_i2c_reg I2C Registers.
 * @ingroup hs_hal_i2c
 * @{
 */

/**
  \brief Union type to access i2c_con (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) I2C Control Register                                       */
  
  struct {
    __IOM uint32_t mst_mod    : 1;            /*!< [0..0] This bit controls whether the I2C Module master is enabled.        */
    __IOM uint32_t spd        : 2;            /*!< [2..1] These bits control at which speed the I2C Module operates.         */
    __IOM uint32_t slv_10bit_addr : 1;        /*!< [3..3] When acting as a slave, this bit controls whether the
                                                   I2C Module responds to 7- or 10-bit addresses.                            */
    __IOM uint32_t mst_10bit_addr : 1;        /*!< [4..4] This bit controls whether the I2C Module starts its transfers
                                                   in 7- or 10-bit addressing mode when acting as a master.                  */
    __IOM uint32_t rstrt_en   : 1;            /*!< [5..5] This bit determines whether RESTART conditions may be
                                                   sent when acting as a master.                                             */
    __IOM uint32_t slv_dis    : 1;            /*!< [6..6] This bit controls whether I2C has its slave 0 disabled.            */
    __IOM uint32_t slv_dis_1  : 1;            /*!< [7..7] This bit controls whether I2C has its slave 1 disabled.            */
  } b;                                        /*!< bit fields for i2c_con */
} i2c_con_t, *pi2c_con_t;

/**
  \brief Union type to access i2c_tar (@ 0x00000004).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000004) I2C Target Address Register                                */
  
  struct {
    __IOM uint32_t tar        : 10;           /*!< [9..0] This is the target address for any master transaction.             */
    __IOM uint32_t gc_start_byte : 1;         /*!< [10..10] If bit 11 (SPECIAL) is set to 1, then this bit indicates
                                                   whether a General Call or START byte command is to be performed
                                                   by the I2C Module. 0: General Call 1: START BYTE                          */
    __IOM uint32_t spec       : 1;            /*!< [11..11] This bit indicates whether software performs a General
                                                   Call or START BYTE command.                                               */
    __IOM uint32_t tar_10bit_addr : 1;        /*!< [12..12] This bit controls whether the I2C Module starts its
                                                   transfers in 7-or 10-bit addressing mode when acting as
                                                   a master.                                                                 */
  } b;                                        /*!< bit fields for i2c_tar */
} i2c_tar_t, *pi2c_tar_t;

/**
  \brief Union type to access i2c_sar (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) I2C Slave Address Register                                 */
  
  struct {
    __IOM uint32_t sar        : 10;           /*!< [9..0] The sar holds the slave address when the I2C is operating
                                                   as a slave.                                                               */
  } b;                                        /*!< bit fields for i2c_sar */
} i2c_sar_t, *pi2c_sar_t;

/**
  \brief Union type to access i2c_hs_maddr (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) I2C High Speed Master Code Address Register                */
  
  struct {
    __IOM uint32_t hs_maddr   : 3;            /*!< [2..0] This bit field holds the value of the I2C HS mode master
                                                   code                                                                      */
  } b;                                        /*!< bit fields for i2c_hs_maddr */
} i2c_hs_maddr_t, *pi2c_hs_maddr_t;

/**
  \brief Union type to access i2c_dat_cmd (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) I2C Data Buffer and Command Register                       */
  
  struct {
    __IOM uint32_t dat        : 8;            /*!< [7..0] This register contains the data to be transmitted or
                                                   received on the I2C bus.                                                  */
    __OM  uint32_t cmd        : 1;            /*!< [8..8] This bit controls whether a read or a write is performed.          */
    __OM  uint32_t stp        : 1;            /*!< [9..9] This bit controls whether a STOP is issued after the
                                                   byte is sent or received.                                                 */
    __OM  uint32_t rstrt      : 1;            /*!< [10..10] This bit controls whether a RESTART is issued before
                                                   the byte is sent or received.                                             */
    __OM  uint32_t null_dat   : 1;            /*!< [11..11] This bit controls whether to transfer slave address
                                                   only.                                                                     */
  } b;                                        /*!< bit fields for i2c_dat_cmd */
} i2c_dat_cmd_t, *pi2c_dat_cmd_t;

/**
  \brief Union type to access i2c_ss_scl_hcnt (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) Standard Speed I2C Clock SCL High Count Register           */
  
  struct {
    __IOM uint32_t hcnt       : 16;           /*!< [15..0] This register sets the SCL clock high-period count for
                                                   standard speed.                                                           */
  } b;                                        /*!< bit fields for i2c_ss_scl_hcnt */
} i2c_ss_scl_hcnt_t, *pi2c_ss_scl_hcnt_t;

/**
  \brief Union type to access i2c_ss_scl_lcnt (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) Standard Speed I2C Clock SCL Low Count Register            */
  
  struct {
    __IOM uint32_t lcnt       : 16;           /*!< [15..0] This register sets the SCL clock low period count for
                                                   standard speed.                                                           */
  } b;                                        /*!< bit fields for i2c_ss_scl_lcnt */
} i2c_ss_scl_lcnt_t, *pi2c_ss_scl_lcnt_t;

/**
  \brief Union type to access i2c_fs_scl_hcnt (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) Fast Speed I2C Clock SCL High Count Register               */
  
  struct {
    __IOM uint32_t hcnt       : 16;           /*!< [15..0] This register sets the SCL clock high-period count for
                                                   fast speed.                                                               */
  } b;                                        /*!< bit fields for i2c_fs_scl_hcnt */
} i2c_fs_scl_hcnt_t, *pi2c_fs_scl_hcnt_t;

/**
  \brief Union type to access i2c_fs_scl_lcnt (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) Fast Speed I2C Clock SCL Low Count Register                */
  
  struct {
    __IOM uint32_t lcnt       : 16;           /*!< [15..0] This register sets the SCL clock low period count for
                                                   fast speed.                                                               */
  } b;                                        /*!< bit fields for i2c_fs_scl_lcnt */
} i2c_fs_scl_lcnt_t, *pi2c_fs_scl_lcnt_t;

/**
  \brief Union type to access i2c_hs_scl_hcnt (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000024) High Speed I2C Clock SCL High Count Register               */
  
  struct {
    __IOM uint32_t hcnt       : 16;           /*!< [15..0] This register sets the SCL clock high period count for
                                                   high speed.                                                               */
  } b;                                        /*!< bit fields for i2c_hs_scl_hcnt */
} i2c_hs_scl_hcnt_t, *pi2c_hs_scl_hcnt_t;

/**
  \brief Union type to access i2c_hs_scl_lcnt (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) High Speed I2C Clock SCL Low Count Register                */
  
  struct {
    __IOM uint32_t lcnt       : 16;           /*!< [15..0] This register sets the SCL clock low period count for
                                                   high speed.                                                               */
  } b;                                        /*!< bit fields for i2c_hs_scl_lcnt */
} i2c_hs_scl_lcnt_t, *pi2c_hs_scl_lcnt_t;

/**
  \brief Union type to access i2c_intr_stat (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) I2C Interrupt Status Register                              */
  
  struct {
    __IM  uint32_t rx_under   : 1;            /*!< [0..0] Set if the processor attempts to read the receive buffer
                                                   when it is empty by reading from the data_cmd register.                   */
    __IM  uint32_t rx_over    : 1;            /*!< [1..1] Set if the receive buffer is completely filled to RX_BUFFER_DEPTH
                                                   and an additional byte is received from an external I2C
                                                   device.                                                                   */
    __IM  uint32_t rx_full    : 1;            /*!< [2..2] Set when the receive buffer reaches or goes above the
                                                   rx_tl threshold in the rx_tl register                                     */
    __IM  uint32_t tx_over    : 1;            /*!< [3..3] Set during transmit if the transmit buffer is filled
                                                   to TX_BUFFER_DEPTH and the processor attempts to issue
                                                   another I2C command by writing to the DATA_CMD register.                  */
    __IM  uint32_t tx_empty   : 1;            /*!< [4..4] This bit is set to 1 when the transmit buffer is at or
                                                   below the threshold value set in the TX_TL register.                      */
    __IM  uint32_t rd_req     : 1;            /*!< [5..5] This bit is set to 1 when I2C Module is acting as a slave
                                                   and another I2C master is attempting to read data from
                                                   I2C Module.                                                               */
    __IM  uint32_t tx_abrt    : 1;            /*!< [6..6] This bit indicates if I2C Module, as an I2C transmitter,
                                                   is unable to complete the intended actions on the contents
                                                   of the transmit FIFO.                                                     */
    __IM  uint32_t rx_done    : 1;            /*!< [7..7] When the I2C Module is acting as a slave-transmitter,
                                                   this bit is set to 1 if the master does not acknowledge
                                                   a transmitted byte. This occurs on the last byte of the
                                                   transmission, indicating that the transmission is done.                   */
    __IM  uint32_t act        : 1;            /*!< [8..8] This bit captures I2C Module activity and stays set until
                                                   it is cleared. There are four ways to clear it: - Disabling
                                                   the I2C Module - Reading the CLR_ACTIVITY register - Reading
                                                   the CLR_INTR register - System reset Once this bit is set,
                                                   it stays set unless one of the four methods is used to
                                                   clear it. Even if the I2C Module module is idle, this bit
                                                   remains set until cleared, indicating that there was activity
                                                   on the bus.                                                               */
    __IM  uint32_t stp_det    : 1;            /*!< [9..9] Indicates whether a STOP condition has occurred on the
                                                   I2C interface regardless of whether I2C Module is operating
                                                   in slave or master mode.                                                  */
    __IM  uint32_t strt_det   : 1;            /*!< [10..10] Indicates whether a START or RESTART condition has
                                                   occurred on the I2C interface regardless of whether I2C
                                                   Module is operating in slave or master mode.                              */
    __IM  uint32_t gen_call   : 1;            /*!< [11..11] Set only when a General Call address is received and
                                                   it is acknowledged.                                                       */
    __IM  uint32_t addr0_match : 1;           /*!< [12..12] Indicates whether an address matches with slave address
                                                   0 when acting as a slave.                                                 */
    __IM  uint32_t addr1_match : 1;           /*!< [13..13] Indicates whether an address matches with slave address
                                                   1 when acting as a slave.                                                 */
    __IM  uint32_t ms_code_det : 1;           /*!< [14..14] Indicates whether master code is detected.                       */
    __IM  uint32_t dma_i2c_done : 1;          /*!< [15..15] Indicates whether I2C DMA operation is done.                     */
  } b;                                        /*!< bit fields for i2c_intr_stat */
} i2c_intr_stat_t, *pi2c_intr_stat_t;

/**
  \brief Union type to access i2c_intr_msk (@ 0x00000030).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000030) I2C Interrupt Mask Register                                */
  
  struct {
    __IOM uint32_t rx_under   : 1;            /*!< [0..0] Correspond to intr_sts                                             */
    __IOM uint32_t rx_over    : 1;            /*!< [1..1] Correspond to intr_sts                                             */
    __IOM uint32_t rx_full    : 1;            /*!< [2..2] Correspond to intr_sts                                             */
    __IOM uint32_t tx_over    : 1;            /*!< [3..3] Correspond to intr_sts                                             */
    __IOM uint32_t tx_empty   : 1;            /*!< [4..4] Correspond to intr_sts                                             */
    __IOM uint32_t rd_req     : 1;            /*!< [5..5] Correspond to intr_sts                                             */
    __IOM uint32_t tx_abrt    : 1;            /*!< [6..6] Correspond to intr_sts                                             */
    __IOM uint32_t rx_done    : 1;            /*!< [7..7] Correspond to intr_sts                                             */
    __IOM uint32_t act        : 1;            /*!< [8..8] Correspond to intr_sts                                             */
    __IOM uint32_t stp_det    : 1;            /*!< [9..9] Correspond to intr_sts                                             */
    __IOM uint32_t strt_det   : 1;            /*!< [10..10] Correspond to intr_sts                                           */
    __IOM uint32_t gen_call   : 1;            /*!< [11..11] Correspond to intr_sts                                           */
    __IOM uint32_t addr0_match : 1;           /*!< [12..12] Correspond to intr_sts                                           */
    __IOM uint32_t addr1_match : 1;           /*!< [13..13] Correspond to intr_sts                                           */
    __IOM uint32_t ms_code_det : 1;           /*!< [14..14] Correspond to intr_sts                                           */
    __IOM uint32_t dma_i2c_done : 1;          /*!< [15..15] Correspond to intr_sts                                           */
  } b;                                        /*!< bit fields for i2c_intr_msk */
} i2c_intr_msk_t, *pi2c_intr_msk_t;

/**
  \brief Union type to access i2c_raw_intr_stat (@ 0x00000034).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000034) I2C Raw Interrupt Status Register                          */
  
  struct {
    __IM  uint32_t rx_under   : 1;            /*!< [0..0] Set if the processor attempts to read the receive buffer
                                                   when it is empty by reading from the data_cmd register.                   */
    __IM  uint32_t rx_over    : 1;            /*!< [1..1] Set if the receive buffer is completely filled to RX_BUFFER_DEPTH
                                                   and an additional byte is received from an external I2C
                                                   device.                                                                   */
    __IM  uint32_t rx_full    : 1;            /*!< [2..2] Set when the receive buffer reaches or goes above the
                                                   rx_tl threshold in the rx_tl register                                     */
    __IM  uint32_t tx_over    : 1;            /*!< [3..3] Set during transmit if the transmit buffer is filled
                                                   to TX_BUFFER_DEPTH and the processor attempts to issue
                                                   another I2C command by writing to the DATA_CMD register.                  */
    __IM  uint32_t tx_empty   : 1;            /*!< [4..4] This bit is set to 1 when the transmit buffer is at or
                                                   below the threshold value set in the TX_TL register.                      */
    __IM  uint32_t rd_req     : 1;            /*!< [5..5] This bit is set to 1 when I2C Module is acting as a slave
                                                   and another I2C master is attempting to read data from
                                                   I2C Module.                                                               */
    __IM  uint32_t tx_abrt    : 1;            /*!< [6..6] This bit indicates if I2C Module, as an I2C transmitter,
                                                   is unable to complete the intended actions on the contents
                                                   of the transmit FIFO.                                                     */
    __IM  uint32_t rx_done    : 1;            /*!< [7..7] When the I2C Module is acting as a slave-transmitter,
                                                   this bit is set to 1 if the master does not acknowledge
                                                   a transmitted byte. This occurs on the last byte of the
                                                   transmission, indicating that the transmission is done.                   */
    __IM  uint32_t act        : 1;            /*!< [8..8] This bit captures I2C Module activity and stays set until
                                                   it is cleared. There are four ways to clear it: - Disabling
                                                   the I2C Module - Reading the CLR_ACTIVITY register - Reading
                                                   the CLR_INTR register - System reset Once this bit is set,
                                                   it stays set unless one of the four methods is used to
                                                   clear it. Even if the I2C Module module is idle, this bit
                                                   remains set until cleared, indicating that there was activity
                                                   on the bus.                                                               */
    __IM  uint32_t stp_det    : 1;            /*!< [9..9] Indicates whether a STOP condition has occurred on the
                                                   I2C interface regardless of whether I2C Module is operating
                                                   in slave or master mode.                                                  */
    __IM  uint32_t strt_det   : 1;            /*!< [10..10] Indicates whether a START or RESTART condition has
                                                   occurred on the I2C interface regardless of whether I2C
                                                   Module is operating in slave or master mode.                              */
    __IM  uint32_t gen_call   : 1;            /*!< [11..11] Set only when a General Call address is received and
                                                   it is acknowledged.                                                       */
    __IM  uint32_t addr0_match : 1;           /*!< [12..12] Indicates whether an address matches with slave address
                                                   0 when acting as a slave.                                                 */
    __IM  uint32_t addr1_match : 1;           /*!< [13..13] Indicates whether an address matches with slave address
                                                   1 when acting as a slave.                                                 */
    __IM  uint32_t ms_code_det : 1;           /*!< [14..14] Indicates whether master code is detected.                       */
    __IM  uint32_t dma_i2c_done : 1;          /*!< [15..15] Indicates whether I2C DMA operation is done.                     */
  } b;                                        /*!< bit fields for i2c_raw_intr_stat */
} i2c_raw_intr_stat_t, *pi2c_raw_intr_stat_t;

/**
  \brief Union type to access i2c_rx_tl (@ 0x00000038).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000038) I2C Receive FIFO Threshold Register                        */
  
  struct {
    __IOM uint32_t rx_tl      : 8;            /*!< [7..0] Receive FIFO Threshold Level                                       */
  } b;                                        /*!< bit fields for i2c_rx_tl */
} i2c_rx_tl_t, *pi2c_rx_tl_t;

/**
  \brief Union type to access i2c_tx_tl (@ 0x0000003C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000003C) I2C Transmit FIFO Threshold Register                       */
  
  struct {
    __IOM uint32_t tx_tl      : 8;            /*!< [7..0] Transmit FIFO Threshold Level                                      */
  } b;                                        /*!< bit fields for i2c_tx_tl */
} i2c_tx_tl_t, *pi2c_tx_tl_t;

/**
  \brief Union type to access i2c_clr_intr (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) Clear Combined and Individual Interrupt Register           */
  
  struct {
    __IM  uint32_t clr_intr   : 1;            /*!< [0..0] Read this register to clear the combined interrupt, all
                                                   individual interrupts, and the TX_ABRT_SOURCE register.                   */
  } b;                                        /*!< bit fields for i2c_clr_intr */
} i2c_clr_intr_t, *pi2c_clr_intr_t;

/**
  \brief Union type to access i2c_clr_rx_under (@ 0x00000044).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000044) Clear RX_UNDER Interrupt Register                          */
  
  struct {
    __IM  uint32_t clr_rx_under : 1;          /*!< [0..0] Read this register to clear the rx_under interrupt (bit
                                                   0) of the raw_intr_stat register                                          */
  } b;                                        /*!< bit fields for i2c_clr_rx_under */
} i2c_clr_rx_under_t, *pi2c_clr_rx_under_t;

/**
  \brief Union type to access i2c_clr_rx_over (@ 0x00000048).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000048) Clear RX_OVER Interrupt Register                           */
  
  struct {
    __IM  uint32_t clr_rx_over : 1;           /*!< [0..0] Read this register to clear the rx_over interrupt (bit
                                                   1) of the raw_intr_stat register                                          */
  } b;                                        /*!< bit fields for i2c_clr_rx_over */
} i2c_clr_rx_over_t, *pi2c_clr_rx_over_t;

/**
  \brief Union type to access i2c_clr_tx_over (@ 0x0000004C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000004C) Clear TX_OVER Interrupt Register                           */
  
  struct {
    __IM  uint32_t clr_tx_over : 1;           /*!< [0..0] Read this register to clear the tx_over interrupt (bit
                                                   3) of the raw_intr_stat register.                                         */
  } b;                                        /*!< bit fields for i2c_clr_tx_over */
} i2c_clr_tx_over_t, *pi2c_clr_tx_over_t;

/**
  \brief Union type to access i2c_clr_rd_req (@ 0x00000050).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000050) Clear RD_REQ Interrupt Register                            */
  
  struct {
    __IM  uint32_t clr_rd_req : 1;            /*!< [0..0] Read this register to clear the rd_req interrupt (bit
                                                   5) of the raw_intr_stat register                                          */
  } b;                                        /*!< bit fields for i2c_clr_rd_req */
} i2c_clr_rd_req_t, *pi2c_clr_rd_req_t;

/**
  \brief Union type to access i2c_clr_tx_abrt (@ 0x00000054).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000054) Clear TX_ABRT Interrupt Register                           */
  
  struct {
    __IM  uint32_t clr_tx_abrt : 1;           /*!< [0..0] Read this register to clear the tx_abrt interrupt (bit
                                                   6) of the raw_intr_stat register, and the tx_abrt_src register.This
                                                   also releases the tx fifo from the flushed/reset state,
                                                   allowing more writes to the TX FIFO. Refer to Bit 9 of
                                                   the tx_abrt_src register for an exception to clearing tx_abrt_src.        */
  } b;                                        /*!< bit fields for i2c_clr_tx_abrt */
} i2c_clr_tx_abrt_t, *pi2c_clr_tx_abrt_t;

/**
  \brief Union type to access i2c_clr_rx_done (@ 0x00000058).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000058) Clear RX_DONE Interrupt Register                           */
  
  struct {
    __IM  uint32_t clr_rx_done : 1;           /*!< [0..0] Read this register to clear the rx_done interrupt (bit
                                                   7) of the raw_intr_stat register                                          */
  } b;                                        /*!< bit fields for i2c_clr_rx_done */
} i2c_clr_rx_done_t, *pi2c_clr_rx_done_t;

/**
  \brief Union type to access i2c_clr_act (@ 0x0000005C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000005C) Clear ACTIVITY Interrupt Register                          */
  
  struct {
    __IM  uint32_t clr_act    : 1;            /*!< [0..0] Reading this register clears the activity interrupt if
                                                   the I2C is not active anymore. If the I2C module is still
                                                   active on the bus, the activity interrupt bit continues
                                                   to be set. It is automatically cleared by hardware if the
                                                   module is disabled and if there is no further activity
                                                   on the bus. The value read from this register to get status
                                                   of the activity interrupt (bit 8) of the raw_intr_stat
                                                   register.                                                                 */
  } b;                                        /*!< bit fields for i2c_clr_act */
} i2c_clr_act_t, *pi2c_clr_act_t;

/**
  \brief Union type to access i2c_clr_stp_det (@ 0x00000060).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000060) Clear STOP_DET Interrupt Register                          */
  
  struct {
    __IM  uint32_t clr_stp_det : 1;           /*!< [0..0] Read this register to clear the stop_det interrupt (bit
                                                   9) of the raw_intr_stat register.                                         */
  } b;                                        /*!< bit fields for i2c_clr_stp_det */
} i2c_clr_stp_det_t, *pi2c_clr_stp_det_t;

/**
  \brief Union type to access i2c_clr_strt_det (@ 0x00000064).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000064) Clear START_DET Interrupt Register                         */
  
  struct {
    __IM  uint32_t clr_strt_det : 1;          /*!< [0..0] Read this register to clear the start_det interrupt (bit
                                                   10) of the raw_intr_stat register.                                        */
  } b;                                        /*!< bit fields for i2c_clr_strt_det */
} i2c_clr_strt_det_t, *pi2c_clr_strt_det_t;

/**
  \brief Union type to access i2c_clr_gen_call (@ 0x00000068).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000068) Clear GEN_CALL Interrupt Register                          */
  
  struct {
    __IM  uint32_t clr_gen_call : 1;          /*!< [0..0] Read this register to clear the gen_call interrupt (bit
                                                   11) of raw_intr_stat register                                             */
  } b;                                        /*!< bit fields for i2c_clr_gen_call */
} i2c_clr_gen_call_t, *pi2c_clr_gen_call_t;

/**
  \brief Union type to access i2c_enable (@ 0x0000006C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000006C) I2C Enable Register                                        */
  
  struct {
    __IOM uint32_t en         : 1;            /*!< [0..0] Controls whether the I2C Module is enabled.                        */
    __IOM uint32_t abrt       : 1;            /*!< [1..1] Abort I2C current transfer is done w/o flush Tx/Rx FIFO            */
  } b;                                        /*!< bit fields for i2c_enable */
} i2c_enable_t, *pi2c_enable_t;

/**
  \brief Union type to access i2c_sts (@ 0x00000070).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000070) I2C Status Reigster                                        */
  
  struct {
    __IM  uint32_t act        : 1;            /*!< [0..0] I2C Activity Status.                                               */
    __IM  uint32_t tfnf       : 1;            /*!< [1..1] Transmit FIFO Not Full. Set when the transmit FIFO contains
                                                   one or more empty locations, and is cleared when the FIFO
                                                   is full.                                                                  */
    __IM  uint32_t tfe        : 1;            /*!< [2..2] Transmit FIFO Completely Empty. When the transmit FIFO
                                                   is completely empty, this bit is set. When it contains
                                                   one or morevalid entries, this bit is cleared. This bit
                                                   field does not request an interrupt.                                      */
    __IM  uint32_t rfne       : 1;            /*!< [3..3] Receive FIFO Not Empty. This bit is set when the receive
                                                   FIFO contains one or more entries; it is cleared when the
                                                   receive FIFO is empty.                                                    */
    __IM  uint32_t rff        : 1;            /*!< [4..4] Receive FIFO CompletelyFull. When the receive FIFO iscompletely
                                                   full, this bit is set. When the receive FIFO contains one
                                                   or more empty location, this bit is cleared.                              */
    __IM  uint32_t mst_act    : 1;            /*!< [5..5] Master FSM Activity Status                                         */
    __IM  uint32_t slv_act    : 1;            /*!< [6..6] Slave FSM Activity Status                                          */
    __IM  uint32_t mst_hold_tx_empty : 1;     /*!< [7..7] I2C module is holding I2C bus low (clock stretch) because
                                                   of TX FIFO empty in master mode.                                          */
    __IM  uint32_t mst_hold_rx_full : 1;      /*!< [8..8] I2C module is holding I2C bus low (clock stretch) because
                                                   of RX FIFO full in master mode.                                           */
    __IM  uint32_t slv_hold_tx_empty : 1;     /*!< [9..9] I2C module is holding I2C bus low (clock stretch) because
                                                   of TX FIFO empty in slave mode.                                           */
    __IM  uint32_t slv_hold_rx_full : 1;      /*!< [10..10] I2C module is holding I2C bus low (clock stretch) because
                                                   of RX FIFO full in slave mode.                                            */
    __IOM uint32_t bus_sts    : 2;            /*!< [12..11] Show current I2C bus status                                      */
  } b;                                        /*!< bit fields for i2c_sts */
} i2c_sts_t, *pi2c_sts_t;

/**
  \brief Union type to access i2c_txflr (@ 0x00000074).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000074) I2C Transmit FIFO Level Register                           */
  
  struct {
    __IM  uint32_t txflr      : 5;            /*!< [4..0] Transmit FIFO Level. Contains the number of valid data
                                                   entries in the transmit FIFO.                                             */
  } b;                                        /*!< bit fields for i2c_txflr */
} i2c_txflr_t, *pi2c_txflr_t;

/**
  \brief Union type to access i2c_rxflr (@ 0x00000078).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000078) I2C Receive FIFO Level Register                            */
  
  struct {
    __IM  uint32_t rxflr      : 5;            /*!< [4..0] Receive FIFO Level. Contains the number of valid data
                                                   entries in the receive FIFO.                                              */
  } b;                                        /*!< bit fields for i2c_rxflr */
} i2c_rxflr_t, *pi2c_rxflr_t;

/**
  \brief Union type to access i2c_sda_hold (@ 0x0000007C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000007C) I2C SDA Hold Time Length Register                          */
  
  struct {
    __IOM uint32_t sda_hold   : 16;           /*!< [15..0] Sets the required SDA hold timein units of clk period.            */
  } b;                                        /*!< bit fields for i2c_sda_hold */
} i2c_sda_hold_t, *pi2c_sda_hold_t;

/**
  \brief Union type to access i2c_tx_abrt_src (@ 0x00000080).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000080) I2C Transmit Abort Source Register                         */
  
  struct {
    __IM  uint32_t addr_7bit_nack : 1;        /*!< [0..0] 1: Master is in 7-bit addressing mode and the address
                                                   sent was not acknowledged by any slave.                                   */
    __IM  uint32_t addr1_10bit_nack : 1;      /*!< [1..1] 1: Master is in 10-bit address mode and the first 10-bit
                                                   address byte was not acknowledged by any slave.                           */
    __IM  uint32_t addr2_10bit_nack : 1;      /*!< [2..2] 1: Master is in 10-bit address mode and the second address
                                                   byte of the 10-bit address was not acknowledged by any
                                                   slave.                                                                    */
    __IM  uint32_t txdat_nack : 1;            /*!< [3..3] 1: This is a master-mode only bit. Master has received
                                                   an acknowledgement for the address, but when it sent data
                                                   byte(s) following the address, it did not receive an acknowledge
                                                   from the remote slave(s).                                                 */
    __IM  uint32_t gcall_nack : 1;            /*!< [4..4] 1: I2C Module in master mode sent a General Call and
                                                   no slave on the bus acknowledged the General Call.                        */
    __IM  uint32_t gcall_rd   : 1;            /*!< [5..5] 1: I2C Module in master mode sent a General Call but
                                                   the user programmed the byte following the General Call
                                                   to be a read from the bus (dat_cmd[9] is set to 1).                       */
    __IM  uint32_t hs_ackdet  : 1;            /*!< [6..6] 1: Master is in High Speed mode and the High Speed Master
                                                   code was acknowledged (wrong behavior).                                   */
    __IM  uint32_t sbyte_ackdet : 1;          /*!< [7..7] 1: Master has sent a START Byte and the START Byte was
                                                   acknowledged (wrong behavior).                                            */
    __IM  uint32_t norstrt_hs : 1;            /*!< [8..8] 1: The restart is disabled (rstrt_en bit (con[5]) = 0)
                                                   and the user is trying to use the master to transfer data
                                                   in High Speed mode.                                                       */
    __IM  uint32_t norstrt_sbyte : 1;         /*!< [9..9] 1: The restart is disabled (rstrt bit con[5]) = 0) and
                                                   the user is trying to send a START Byte.                                  */
    __IM  uint32_t norstrt_10bit_rd : 1;      /*!< [10..10] 1: The restart is disabled (rstrt_en bit (con[5]) =
                                                   0) and the master sends a read command in 10-bit addressing
                                                   mode.                                                                     */
    __IM  uint32_t mst_dis    : 1;            /*!< [11..11] 1: User tries to initiate a Master operation with the
                                                   Master mode disabled.                                                     */
    __IM  uint32_t arb_lost   : 1;            /*!< [12..12] 1: Master has lost arbitration, or if tx_abrt_src[14]
                                                   is also set, then the slave transmitter has lost arbitration.             */
    __IM  uint32_t slvflush_txfifo : 1;       /*!< [13..13] 1: Slave has received a read command and some data
                                                   exists in the TX FIFO so the slave issues a tx_abrt interrupt
                                                   to flush old data in TX FIFO.                                             */
    __IM  uint32_t slv_arblost : 1;           /*!< [14..14] 1: Slave lost the bus while transmitting data to a
                                                   remote master. Tx_abrt_src[12] is set at the same time.                   */
    __IM  uint32_t slvrd_intx : 1;            /*!< [15..15] 1: When the processor side responds to a slave mode
                                                   request for data to be transmitted to a remote master and
                                                   user writes a 1 in cmd(bit 8) of dat_cmd register.                        */
  } b;                                        /*!< bit fields for i2c_tx_abrt_src */
} i2c_tx_abrt_src_t, *pi2c_tx_abrt_src_t;

/**
  \brief Union type to access i2c_slv_dat_nack (@ 0x00000084).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000084) I2C Generate Slave Data NACK Register                      */
  
  struct {
    __IOM uint32_t slv_dat_nack : 1;          /*!< [0..0] Generate NACK. This NACK generation only occurs when
                                                   I2C Module is a slavereceiver. If this register is set
                                                   to a value of 1, it can only generate a NACK after a data
                                                   byte is received; hence, the data transfer is aborted and
                                                   the data received is not pushed to the receive buffer.
                                                   When the register is set to a value of 0, it generates
                                                   NACK/ACK, depending on normal criteria.                                   */
  } b;                                        /*!< bit fields for i2c_slv_dat_nack */
} i2c_slv_dat_nack_t, *pi2c_slv_dat_nack_t;

/**
  \brief Union type to access i2c_dma_cr (@ 0x00000088).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000088) I2C DMA Control Reigster                                   */
  
  struct {
    __IOM uint32_t rdmae      : 1;            /*!< [0..0] Receive DMA Enable. This bit enables/disables the receive
                                                   FIFO DMA channel.                                                         */
    __IOM uint32_t tdmae      : 1;            /*!< [1..1] Transmit DMA Enable. This bit enables/disables the transmit
                                                   FIFO DMA channel.                                                         */
  } b;                                        /*!< bit fields for i2c_dma_cr */
} i2c_dma_cr_t, *pi2c_dma_cr_t;

/**
  \brief Union type to access i2c_dma_tdlr (@ 0x0000008C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000008C) I2C DMA Transmit Data Level Register                       */
  
  struct {
    __IOM uint32_t tdlr       : 4;            /*!< [3..0] Transmit Data Level. This bit field controls the level
                                                   at which a DMA request is made by the transmit logic. It
                                                   is equal to the watermark level; that is, the dma_tx_req
                                                   signal is generated when the number of valid data entries
                                                   in the transmit FIFO is equal to or below this field value,
                                                   and tdmae= 1.                                                             */
  } b;                                        /*!< bit fields for i2c_dma_tdlr */
} i2c_dma_tdlr_t, *pi2c_dma_tdlr_t;

/**
  \brief Union type to access i2c_dma_rdlr (@ 0x00000090).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000090) I2C DMA Receive Data Level Register                        */
  
  struct {
    __IOM uint32_t rdlr       : 4;            /*!< [3..0] Receive Data Level.                                                */
  } b;                                        /*!< bit fields for i2c_dma_rdlr */
} i2c_dma_rdlr_t, *pi2c_dma_rdlr_t;

/**
  \brief Union type to access i2c_sda_setup (@ 0x00000094).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000094) I2C SDA Setup Register                                     */
  
  struct {
    __IOM uint32_t sda_setup  : 8;            /*!< [7..0] SDA Setup. It is recommended that if the required delay
                                                   is 1000ns, then for an clk frequency of 10 MHz, sda_setup
                                                   should be programmed to a value of 11. sda_setup must be
                                                   programmed with a minimum value of 2.                                     */
  } b;                                        /*!< bit fields for i2c_sda_setup */
} i2c_sda_setup_t, *pi2c_sda_setup_t;

/**
  \brief Union type to access i2c_ack_gen_call (@ 0x00000098).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000098) I2C ACK General Call Register                              */
  
  struct {
    __IOM uint32_t ack_gen_call : 1;          /*!< [0..0] ACK General Call. When set to 1, I2C Module responds
                                                   with a ACK (by asserting data_oe) when it receives a General
                                                   Call. When set to 0, the I2C Module does not generate General
                                                   Call interrupts.                                                          */
  } b;                                        /*!< bit fields for i2c_ack_gen_call */
} i2c_ack_gen_call_t, *pi2c_ack_gen_call_t;

/**
  \brief Union type to access i2c_en_sts (@ 0x0000009C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000009C) I2C Enable Status Register                                 */
  
  struct {
    __IM  uint32_t en         : 1;            /*!< [0..0] Enable Status.                                                     */
    __IM  uint32_t slv_dis_in_busy : 1;       /*!< [1..1] Slave Disabled While Busy (Transmit, Receive). This bit
                                                   indicates if a potential or active Slave operation has
                                                   been aborted due to the setting of the ENABLE register
                                                   from 1 to 0.                                                              */
    __IM  uint32_t slv_rx_dat_lost : 1;       /*!< [2..2] Slave Received Data Lost. This bit indicates if a Slave-Receiver
                                                   operation has been aborted with at least one data byte
                                                   received from an I2C transfer due to the setting of enable
                                                   from 1 to 0.                                                              */
    __IM  uint32_t dma_dis_sts : 2;           /*!< [4..3] DMA_DISABLE_WHILE_BUSY. 00: No ill disable event is active
                                                   01: I2C is disable while busy in legacy mode 10: I2C is
                                                   disable while busy in DMA mode 11: I2C is disable while
                                                   busy in Descriptor mode                                                   */
  } b;                                        /*!< bit fields for i2c_en_sts */
} i2c_en_sts_t, *pi2c_en_sts_t;

/**
  \brief Union type to access i2c_dma_cmd (@ 0x000000A0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000A0) I2C DMA Command Register                                   */
  
  struct {
    __IOM uint32_t en         : 1;            /*!< [0..0] Set to enable DMA mode.                                            */
    __IM  uint32_t            : 4;
    __IOM uint32_t cmd        : 1;            /*!< [5..5] This bit controls whether a read or a write is performed.
                                                   This bit does not control the direction when the I2C Module
                                                   acts as a slave. It controls only the direction when it
                                                   acts as a master. 1 = Read 0 = Write                                      */
    __IOM uint32_t stp        : 1;            /*!< [6..6] This bit controls whether a STOP is issued after the
                                                   byte is sent or received.                                                 */
    __IOM uint32_t rstrt      : 1;            /*!< [7..7] This bit controls whether a RESTART is issued before
                                                   the byte is sent or received when DMA mode is enabled.                    */
  } b;                                        /*!< bit fields for i2c_dma_cmd */
} i2c_dma_cmd_t, *pi2c_dma_cmd_t;

/**
  \brief Union type to access i2c_dma_len (@ 0x000000A4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000A4) I2C DMA Transfer Data Length Register                      */
  
  struct {
    __IOM uint32_t len        : 16;           /*!< [15..0] DMA transfer data length(R/W)                                     */
    __IM  uint32_t tr_len     : 16;           /*!< [31..16] DMA mode transfer bytes(Read only)                               */
  } b;                                        /*!< bit fields for i2c_dma_len */
} i2c_dma_len_t, *pi2c_dma_len_t;

/**
  \brief Union type to access i2c_dma_mod (@ 0x000000A8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000A8) I2C DMA Mode Register                                      */
  
  struct {
    __IOM uint32_t dma_mod    : 2;            /*!< [1..0] DMA operation mode                                                 */
  } b;                                        /*!< bit fields for i2c_dma_mod */
} i2c_dma_mod_t, *pi2c_dma_mod_t;

/**
  \brief Union type to access i2c_slp (@ 0x000000AC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000AC) I2C Sleep Control Register                                 */
  
  struct {
    __IOM uint32_t clk_ctrl   : 1;            /*!< [0..0] I2C clock control, write 1 controller would gate I2C
                                                   clock until I2C slave is enable and reset synchronized
                                                   register procedure is done                                                */
    __IM  uint32_t slp_clk_gated : 1;         /*!< [1..1] I2C clock has been gated (Read Only)                               */
  } b;                                        /*!< bit fields for i2c_slp */
} i2c_slp_t, *pi2c_slp_t;

/**
  \brief Union type to access i2c_dat_fltr_rsts_l (@ 0x000000B8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000B8) Data Filter Resistor Low Register                          */
  
  struct {
    __IOM uint32_t dat_fltr_rsts_l : 20;      /*!< [19..0] Data Filter Resistor Low                                          */
  } b;                                        /*!< bit fields for i2c_dat_fltr_rsts_l */
} i2c_dat_fltr_rsts_l_t, *pi2c_dat_fltr_rsts_l_t;

/**
  \brief Union type to access i2c_dat_fltr_rsts_m (@ 0x000000BC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000BC) Data Filter Resistor Main Register                         */
  
  struct {
    __IOM uint32_t dat_fltr_rsts_m : 20;      /*!< [19..0] Data Filter Resistor Main                                         */
  } b;                                        /*!< bit fields for i2c_dat_fltr_rsts_m */
} i2c_dat_fltr_rsts_m_t, *pi2c_dat_fltr_rsts_m_t;

/**
  \brief Union type to access i2c_clk_fltr_rsts_l (@ 0x000000C0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000C0) Clock Filter Resistor Low Register                         */
  
  struct {
    __IOM uint32_t clk_fltr_rsts_l : 20;      /*!< [19..0] Clock Filter Resistor Low                                         */
  } b;                                        /*!< bit fields for i2c_clk_fltr_rsts_l */
} i2c_clk_fltr_rsts_l_t, *pi2c_clk_fltr_rsts_l_t;

/**
  \brief Union type to access i2c_clk_fltr_rsts_m (@ 0x000000C4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000C4) Clock Filter Resistor Main Register                        */
  
  struct {
    __IOM uint32_t clk_fltr_rsts_m : 20;      /*!< [19..0] Clock Filter Resistor Main                                        */
  } b;                                        /*!< bit fields for i2c_clk_fltr_rsts_m */
} i2c_clk_fltr_rsts_m_t, *pi2c_clk_fltr_rsts_m_t;

/**
  \brief Union type to access i2c_dat_fltr_cap_l (@ 0x000000C8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000C8) Data Filter Capacitor Low Register                         */
  
  struct {
    __IOM uint32_t dat_fltr_cap_l : 20;       /*!< [19..0] Data Filter Capacitor Low                                         */
  } b;                                        /*!< bit fields for i2c_dat_fltr_cap_l */
} i2c_dat_fltr_cap_l_t, *pi2c_dat_fltr_cap_l_t;

/**
  \brief Union type to access i2c_dat_fltr_cap_m (@ 0x000000CC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000CC) Data Filter Capacitor Main Register                        */
  
  struct {
    __IOM uint32_t dat_fltr_cap_m : 5;        /*!< [4..0] Data Filter Capacitor Main                                         */
  } b;                                        /*!< bit fields for i2c_dat_fltr_cap_m */
} i2c_dat_fltr_cap_m_t, *pi2c_dat_fltr_cap_m_t;

/**
  \brief Union type to access i2c_clk_fltr_cap_l (@ 0x000000D0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000D0) Clock Filter Capacitor Low Register                        */
  
  struct {
    __IOM uint32_t clk_fltr_cap_l : 20;       /*!< [19..0] Clock Filter Capacitor Low                                        */
  } b;                                        /*!< bit fields for i2c_clk_fltr_cap_l */
} i2c_clk_fltr_cap_l_t, *pi2c_clk_fltr_cap_l_t;

/**
  \brief Union type to access i2c_clk_fltr_cap_m (@ 0x000000D4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000D4) Clock Filter Capacitor Main Register                       */
  
  struct {
    __IOM uint32_t clk_fltr_cap_m : 5;        /*!< [4..0] Clock Filter Capacitor Main                                        */
  } b;                                        /*!< bit fields for i2c_clk_fltr_cap_m */
} i2c_clk_fltr_cap_m_t, *pi2c_clk_fltr_cap_m_t;

/**
  \brief Union type to access i2c_clr_addr_match (@ 0x000000E4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000E4) Clear I2C ADDR_MATCH Interrupt Register                    */
  
  struct {
    __IOM uint32_t clr_addr_match : 1;        /*!< [0..0] Clear address match interrupts raw status, and it's read
                                                   clear.                                                                    */
  } b;                                        /*!< bit fields for i2c_clr_addr_match */
} i2c_clr_addr_match_t, *pi2c_clr_addr_match_t;

/**
  \brief Union type to access i2c_clr_dma_done (@ 0x000000E8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000E8) Clear I2C DMA_DONE Interrupt Register                      */
  
  struct {
    __IOM uint32_t clr_dma_done : 1;          /*!< [0..0] Clear dma_i2c_done_intr interrupts raw status, and it's
                                                   read clear.                                                               */
  } b;                                        /*!< bit fields for i2c_clr_dma_done */
} i2c_clr_dma_done_t, *pi2c_clr_dma_done_t;

/**
  \brief Union type to access i2c_fltr (@ 0x000000EC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000EC) I2C Bus Filter Register                                    */
  
  struct {
    __IOM uint32_t dig_fltr_deg : 4;          /*!< [3..0] I2C module digital filter degree                                   */
    __IM  uint32_t            : 4;
    __IOM uint32_t dig_fltr_en : 1;           /*!< [8..8] I2C module digital filter slection (enable)                        */
  } b;                                        /*!< bit fields for i2c_fltr */
} i2c_fltr_t, *pi2c_fltr_t;

/**
  \brief Union type to access i2c_sar1 (@ 0x000000F4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F4) I2C Slave Address 1 Register                               */
  
  struct {
    __IOM uint32_t sar1       : 7;            /*!< [6..0] I2C Second Slave Address                                           */
  } b;                                        /*!< bit fields for i2c_sar1 */
} i2c_sar1_t, *pi2c_sar1_t;

/**
  \brief Union type to access i2c_ver (@ 0x000000FC).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000FC) I2C Component Version Register                             */
  
  struct {
    __IM  uint32_t ver        : 32;           /*!< [31..0] I2C module version number                                         */
  } b;                                        /*!< bit fields for i2c_ver */
} i2c_ver_t, *pi2c_ver_t;

/** @} */ /* End of group ls_hal_i2c_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_I2C_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_I2C_TYPE_H_

