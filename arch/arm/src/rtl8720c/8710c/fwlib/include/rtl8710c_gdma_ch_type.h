/**************************************************************************//**
 * @file      rtl8710c_gdma_ch_type.h
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

#ifndef _RTL8710C_GDMA_CH_TYPE_H_
#define _RTL8710C_GDMA_CH_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_GDMA_CH_REG_TYPE

/**
 * @addtogroup hs_hal_gdma_ch_reg GDMA_CH Registers.
 * @ingroup hs_hal_gdma
 * @{
 */

/**
  \brief Union type to access gdma_ch_sar (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) Source Address Register for Channel x                      */
  
  struct {
    __IOM uint32_t sar        : 32;           /*!< [31..0] Source Address of DMA transfer. Updated after each source
                                                   transfer. The SINC field in the CTLx register determines
                                                   whether the address increments, decrements, or is left
                                                   unchanged on every source transfer throughout the block
                                                   transfer.                                                                 */
  } b;                                        /*!< bit fields for gdma_ch_sar */
} gdma_ch_sar_t, *pgdma_ch_sar_t;

/**
  \brief Union type to access gdma_ch_dar (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) Destination Address Register for Channel x                 */
  
  struct {
    __IOM uint32_t dar        : 32;           /*!< [31..0] Destination address of DMA transfer. Updated after each
                                                   destination transfer. The DINC field in the CTLx register
                                                   determines whether the address increments, decrements,
                                                   or is left unchanged on every destination transfer throughout
                                                   the block transfer                                                        */
  } b;                                        /*!< bit fields for gdma_ch_dar */
} gdma_ch_dar_t, *pgdma_ch_dar_t;

/**
  \brief Union type to access gdma_ch_llp (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) Linked List Pointer Register for Channel x                 */
  
  struct {
    __IOM uint32_t lms        : 2;            /*!< [1..0] List Master Select. Identifies the AHB layer/interface
                                                   where the memory device that stores the next linked list
                                                   item resides.                                                             */
    __IOM uint32_t loc        : 30;           /*!< [31..2] Starting address in memory of next LLI if block chaining
                                                   is enabled.                                                               */
  } b;                                        /*!< bit fields for gdma_ch_llp */
} gdma_ch_llp_t, *pgdma_ch_llp_t;

/**
  \brief Union type to access gdma_ch_ctl_low (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) Lower Word of Control Register for Channel x               */
  
  struct {
    __IOM uint32_t int_en     : 1;            /*!< [0..0] If set, then all interrupt-generating sources are enabled.
                                                   Functions as a global mask bit for all interrupts for the
                                                   channel. Raw interrupt registers still assert if this field
                                                   is 0.                                                                     */
    __IOM uint32_t dst_tr_width : 3;          /*!< [3..1] Destination Transfer Width. 0x000 : 8 bit, 0x001 : 16
                                                   bit, 0x010 : 32 bit                                                       */
    __IOM uint32_t src_tr_width : 3;          /*!< [6..4] Source Transfer Width. The block_ts field should be modified
                                                   in accordance with this field. 0x000 : 8 bit, 0x001 : 16
                                                   bit, 0x010 : 32 bit                                                       */
    __IOM uint32_t dinc       : 2;            /*!< [8..7] Destination Address Increment. Indicate whether to increment
                                                   to destination address on every destination transfer. If
                                                   the device is fetching data from a destination peripheral
                                                   FIFO with a fixed adress, then this field should be set
                                                   to No Change. 0x00 : Increment, 0x1x : NoChange.                          */
    __IOM uint32_t sinc       : 2;            /*!< [10..9] Source Address Increment. Indicate whether to increment
                                                   to source address on every source transfer. If the device
                                                   is fetching data from a source peripheral FIFO with a fixed
                                                   adress, then this field should be set to No Change. 0x00
                                                   : Increment, 0x1x : NoChange.                                             */
    __IOM uint32_t dest_msize : 3;            /*!< [13..11] Destination Burst Transaction Length. Number of data
                                                   items, each of width ctl.dst_tr_width, to be read from
                                                   the destination every time a destination burst transaction
                                                   request is mode.                                                          */
    __IOM uint32_t src_msize  : 3;            /*!< [16..14] Source Burst Transaction Length. Number of data items,
                                                   each of width ctl.src_tr_width, to be read from the source
                                                   every time a source burst transaction request is mode.                    */
    __IM  uint32_t            : 3;
    __IOM uint32_t tt_fc      : 3;            /*!< [22..20] Transfer Type and Flow Control. The following transfer
                                                   types are supported. Memory to Memory, Memory to Peripheral,
                                                   Peripheral to Memory.                                                     */
    __IM  uint32_t            : 4;
    __IOM uint32_t llp_dst_en : 1;            /*!< [27..27] Block chaining is enabled on the destination side if
                                                   this field is high and llp.loc is non-zero.                               */
    __IOM uint32_t llp_src_en : 1;            /*!< [28..28] Block chaining is enabled on the source side if this
                                                   field is high and llp.loc is non-zero.                                    */
  } b;                                        /*!< bit fields for gdma_ch_ctl_low */
} gdma_ch_ctl_low_t, *pgdma_ch_ctl_low_t;

/**
  \brief Union type to access gdma_ch_ctl_up (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) Upper Word of Control Register for Channel x               */
  
  struct {
    __IOM uint32_t block_ts   : 12;           /*!< [11..0] Block Transfer Size. When the RTK_DMAC is the flow controller,
                                                   the user writes this field before the channel is enabled
                                                   in order to indicate the block size. The block_ts should
                                                   be modified according to the src_tr_width field. Ex: If
                                                   src_tr_width = 4 byte, block_ts = 2.                                      */
  } b;                                        /*!< bit fields for gdma_ch_ctl_up */
} gdma_ch_ctl_up_t, *pgdma_ch_ctl_up_t;

/**
  \brief Union type to access gdma_ch_cfg_low (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) Lower Word of Configuration Register for Channel
                                                                  x                                                          */
  
  struct {
    __IOM uint32_t inactive   : 1;            /*!< [0..0] Indicate the channel is inactive or not. 1: Inactive
                                                   state, 0: Active state.                                                   */
    __IM  uint32_t            : 7;
    __IOM uint32_t ch_susp    : 1;            /*!< [8..8] Channel Suspend. Suspends all DMA data transfers from
                                                   the source until this bit is cleared. There is no guarantee
                                                   that the current transaction will complete. Can also be
                                                   used in conjunction with CFGx.FIFO_EMPTY to cleanly disable
                                                   a channel without losing any data. 0 : Not suspended, 1
                                                   : Suspend DMA transfer from the source                                    */
    __IM  uint32_t fifo_empty : 1;            /*!< [9..9] Indicates if there is data left in the channel FIFO.
                                                   Can be used in conjunction with CFGx.CH_SUSP to cleanly
                                                   disable a channel. 1 : Channel FIFO empty, 0 : Channel
                                                   FIFO not empty.                                                           */
    __IM  uint32_t            : 8;
    __IOM uint32_t dst_hs_pol : 1;            /*!< [18..18] Destination Handshaking Interface Polarity. 0 : Active
                                                   high, 1: Active low                                                       */
    __IOM uint32_t src_hs_pol : 1;            /*!< [19..19] Source Handshaking Interface Polarity. 0 : Active high,
                                                   1: Active low                                                             */
    __IOM uint32_t max_abrst  : 10;           /*!< [29..20] Maximum OCP Burst Length. Maximum OCP burst length
                                                   that is used for DMA transfers on this channel.                           */
    __IOM uint32_t reload_src : 1;            /*!< [30..30] Automatic Source Reload. The SAR register can be automatically
                                                   reloaded from its initial value at the end of every block
                                                   for multi-block transfers. Not support in LP-GDMA.                        */
    __IOM uint32_t reload_dst : 1;            /*!< [31..31] Automatic Destination Reload. The DAR register can
                                                   be automatically reloaded from its initial value at the
                                                   end of every block for multi-block transfers. Not support
                                                   in LP-GDMA.                                                               */
  } b;                                        /*!< bit fields for gdma_ch_cfg_low */
} gdma_ch_cfg_low_t, *pgdma_ch_cfg_low_t;

/**
  \brief Union type to access gdma_ch_cfg_up (@ 0x00000044).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000044) Upper Word of Configuration Register for Channel
                                                                  x                                                          */
  
  struct {
    __IM  uint32_t            : 1;
    __IOM uint32_t fifo_mode  : 1;
    __IM  uint32_t            : 1;
    __IOM uint32_t secure_en  : 1;            /*!< [3..3] Enable secure mode DMA transfer. This bit can only be
                                                   accessed by CPU under secure mode. Non-secure GDMA does
                                                   not have this bit.                                                        */
    __IM  uint32_t            : 3;
    __IOM uint32_t src_per    : 4;            /*!< [10..7] Assigns a hardware handshaking interface to the source
                                                   of channels. The channel can then communicate with the
                                                   source peripheral connected to that interface through the
                                                   assigned hardware handshaking interface.                                  */
    __IOM uint32_t dest_per   : 4;            /*!< [14..11] Assigns a hardware handshaking interface to the destination
                                                   of channels. The channel can then communicate with the
                                                   destination peripheral connected to that interface through
                                                   the assigned hardware handshaking interface.                              */
    __IOM uint32_t extended_src_per : 1;      /*!< [15..15] The extened 4th bit of src_per when hardware handshake
                                                   interfaces number is configured to 32. Not supported in
                                                   LP-GDMA.                                                                  */
    __IOM uint32_t extended_dest_per : 1;     /*!< [16..16] The extened 4th bit of dest_per when hardware handshake
                                                   interfaces number is configured to 32. Not supported in
                                                   LP-GDMA.                                                                  */
  } b;                                        /*!< bit fields for gdma_ch_cfg_up */
} gdma_ch_cfg_up_t, *pgdma_ch_cfg_up_t;

/** @} */ /* End of group ls_hal_gdma_ch_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_GDMA_CH_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_GDMA_CH_TYPE_H_

