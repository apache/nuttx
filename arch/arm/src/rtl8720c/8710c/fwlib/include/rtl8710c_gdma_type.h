/**************************************************************************//**
 * @file      rtl8710c_gdma_type.h
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

#ifndef _RTL8710C_GDMA_TYPE_H_
#define _RTL8710C_GDMA_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_GDMA_REG_TYPE

/**
 * @addtogroup hs_hal_gdma_reg GDMA Registers.
 * @ingroup hs_hal_gdma
 * @{
 */

/**
  \brief Union type to access gdma_raw_tfr (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) IntTfr Raw Interrupt Status Register                       */
  
  struct {
    __IM  uint32_t raw_tfr    : 6;            /*!< [5..0] Raw interrupt status of transfer complete                          */
  } b;                                        /*!< bit fields for gdma_raw_tfr */
} gdma_raw_tfr_t, *pgdma_raw_tfr_t;

/**
  \brief Union type to access gdma_raw_block (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) IntBlock Raw Interrupt Status Register                     */
  
  struct {
    __IM  uint32_t raw_block  : 6;            /*!< [5..0] Raw interrupt status of one block transfer complete                */
  } b;                                        /*!< bit fields for gdma_raw_block */
} gdma_raw_block_t, *pgdma_raw_block_t;

/**
  \brief Union type to access gdma_raw_src_tran (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) IntSrcTran Raw Interrupt Status Register                   */
  
  struct {
    __IM  uint32_t raw_src_tran : 6;          /*!< [5..0] Raw interrupt status of source transfer complete                   */
  } b;                                        /*!< bit fields for gdma_raw_src_tran */
} gdma_raw_src_tran_t, *pgdma_raw_src_tran_t;

/**
  \brief Union type to access gdma_raw_dst_tran (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) IntDstTran Raw Interrupt Status Register                   */
  
  struct {
    __IM  uint32_t raw_dst_tran : 6;          /*!< [5..0] Raw interrupt status of destination transfer complete              */
  } b;                                        /*!< bit fields for gdma_raw_dst_tran */
} gdma_raw_dst_tran_t, *pgdma_raw_dst_tran_t;

/**
  \brief Union type to access gdma_raw_err (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) IntErr Raw Interrupt Status Register                       */
  
  struct {
    __IM  uint32_t raw_err    : 6;            /*!< [5..0] Raw interrupt status of transfer error                             */
  } b;                                        /*!< bit fields for gdma_raw_err */
} gdma_raw_err_t, *pgdma_raw_err_t;

/**
  \brief Union type to access gdma_status_tfr (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) IntTfr Interrupt Status Register                           */
  
  struct {
    __IM  uint32_t status_tfr : 6;            /*!< [5..0] Interrupt status of transfer complete                              */
  } b;                                        /*!< bit fields for gdma_status_tfr */
} gdma_status_tfr_t, *pgdma_status_tfr_t;

/**
  \brief Union type to access gdma_status_block (@ 0x00000030).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000030) IntBlock Interrupt Status Register                         */
  
  struct {
    __IM  uint32_t status_block : 6;          /*!< [5..0] Interrupt status of one block transfer complete                    */
  } b;                                        /*!< bit fields for gdma_status_block */
} gdma_status_block_t, *pgdma_status_block_t;

/**
  \brief Union type to access gdma_status_src_tran (@ 0x00000038).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000038) IntSrcTran Interrupt Status Register                       */
  
  struct {
    __IM  uint32_t status_src_tran : 6;       /*!< [5..0] Interrupt status of source transfer complete                       */
  } b;                                        /*!< bit fields for gdma_status_src_tran */
} gdma_status_src_tran_t, *pgdma_status_src_tran_t;

/**
  \brief Union type to access gdma_status_dst_tran (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) IntDstTran Interrupt Status Register                       */
  
  struct {
    __IM  uint32_t status_dst_tran : 6;       /*!< [5..0] Interrupt status of destination transfer complete                  */
  } b;                                        /*!< bit fields for gdma_status_dst_tran */
} gdma_status_dst_tran_t, *pgdma_status_dst_tran_t;

/**
  \brief Union type to access gdma_status_err (@ 0x00000048).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000048) IntErr Interrupt Status Register                           */
  
  struct {
    __IM  uint32_t status_err : 6;            /*!< [5..0] Interrupt status of transfer error                                 */
  } b;                                        /*!< bit fields for gdma_status_err */
} gdma_status_err_t, *pgdma_status_err_t;

/**
  \brief Union type to access gdma_mask_tfr (@ 0x00000050).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000050) IntTfr Interrupt Mask Register                             */
  
  struct {
    __IOM uint32_t mask_tfr   : 6;            /*!< [5..0] Interrupt mask of transfer complete                                */
  } b;                                        /*!< bit fields for gdma_mask_tfr */
} gdma_mask_tfr_t, *pgdma_mask_tfr_t;

/**
  \brief Union type to access gdma_mask_block (@ 0x00000058).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000058) IntBlock Interrupt Mask Register                           */
  
  struct {
    __IOM uint32_t mask_block : 6;            /*!< [5..0] Interrupt mask of one block transfer complete                      */
  } b;                                        /*!< bit fields for gdma_mask_block */
} gdma_mask_block_t, *pgdma_mask_block_t;

/**
  \brief Union type to access gdma_mask_src_tran (@ 0x00000060).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000060) IntSrcTran Interrupt Mask Register                         */
  
  struct {
    __IOM uint32_t mask_src_tran : 6;         /*!< [5..0] Interrupt mask of source transfer complete                         */
  } b;                                        /*!< bit fields for gdma_mask_src_tran */
} gdma_mask_src_tran_t, *pgdma_mask_src_tran_t;

/**
  \brief Union type to access gdma_mask_dst_tran (@ 0x00000068).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000068) IntDstTran Interrupt Mask Register                         */
  
  struct {
    __IOM uint32_t mask_dst_tran : 6;         /*!< [5..0] Interrupt mask of destination transfer complete                    */
  } b;                                        /*!< bit fields for gdma_mask_dst_tran */
} gdma_mask_dst_tran_t, *pgdma_mask_dst_tran_t;

/**
  \brief Union type to access gdma_mask_err (@ 0x00000070).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000070) IntErr Interrupt Mask Register                             */
  
  struct {
    __IOM uint32_t mask_err   : 6;            /*!< [5..0] Interrupt mask of transfer error                                   */
  } b;                                        /*!< bit fields for gdma_mask_err */
} gdma_mask_err_t, *pgdma_mask_err_t;

/**
  \brief Union type to access gdma_clear_tfr (@ 0x00000078).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000078) IntTfr Interrupt Clear Register                            */
  
  struct {
    __OM  uint32_t status_tfr : 6;            /*!< [5..0] Clear interrupt status of transfer complete                        */
  } b;                                        /*!< bit fields for gdma_clear_tfr */
} gdma_clear_tfr_t, *pgdma_clear_tfr_t;

/**
  \brief Union type to access gdma_clear_block (@ 0x00000080).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000080) IntBlock Interrupt Clear Register                          */
  
  struct {
    __OM  uint32_t clear_block : 6;           /*!< [5..0] Clear interrupt status of one block transfer complete              */
  } b;                                        /*!< bit fields for gdma_clear_block */
} gdma_clear_block_t, *pgdma_clear_block_t;

/**
  \brief Union type to access gdma_clear_src_tran (@ 0x00000088).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000088) IntSrcTran Interrupt Clear Register                        */
  
  struct {
    __OM  uint32_t clear_src_tran : 6;        /*!< [5..0] Clear interrupt status of source transfer complete                 */
  } b;                                        /*!< bit fields for gdma_clear_src_tran */
} gdma_clear_src_tran_t, *pgdma_clear_src_tran_t;

/**
  \brief Union type to access gdma_clear_dst_tran (@ 0x00000090).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000090) IntDstTran Interrupt Clear Register                        */
  
  struct {
    __OM  uint32_t clear_dst_tran : 6;        /*!< [5..0] Clear interrupt status of destination transfer complete            */
  } b;                                        /*!< bit fields for gdma_clear_dst_tran */
} gdma_clear_dst_tran_t, *pgdma_clear_dst_tran_t;

/**
  \brief Union type to access gdma_clear_err (@ 0x00000098).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000098) IntErr Interrupt Clear Register                            */
  
  struct {
    __OM  uint32_t clear_err  : 6;            /*!< [5..0] Clear interrupt status of transfer error                           */
  } b;                                        /*!< bit fields for gdma_clear_err */
} gdma_clear_err_t, *pgdma_clear_err_t;

/**
  \brief Union type to access gdma_status_int (@ 0x000000A0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000A0) Combined Interrupt Status Register                         */
  
  struct {
    __IM  uint32_t tfr        : 1;            /*!< [0..0] OR of the contents of StatusTfr register                           */
    __IM  uint32_t block      : 1;            /*!< [1..1] OR of the contents of StatusBlock register                         */
    __IM  uint32_t srct       : 1;            /*!< [2..2] OR of the contents of StatusSrcTran register                       */
    __IM  uint32_t dstt       : 1;            /*!< [3..3] OR of the contents of StatusDstTran register                       */
    __IM  uint32_t err        : 1;            /*!< [4..4] OR of the contents of StatusErr register                           */
  } b;                                        /*!< bit fields for gdma_status_int */
} gdma_status_int_t, *pgdma_status_int_t;

/**
  \brief Union type to access gdma_dma_cfg_reg (@ 0x000000D8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000D8) Configuration Register                                     */
  
  struct {
    __IOM uint32_t dma_en     : 1;            /*!< [0..0] RTK_DMAC Enable bit. 0 : Disable, 1 : Enable                       */
  } b;                                        /*!< bit fields for gdma_dma_cfg_reg */
} gdma_dma_cfg_reg_t, *pgdma_dma_cfg_reg_t;

/**
  \brief Union type to access gdma_ch_en_reg (@ 0x000000E0).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000E0) Channel Enable Register                                    */
  
  struct {
    __IOM uint32_t ch_en      : 6;            /*!< [5..0] Enables/Disables the channel. Setting this bit enables
                                                   a channel while clearing this bit disables the channel.
                                                   0 : Disable the Channel 1 : Enable the Channel The ChEnReg.CH_EN
                                                   bit is automatically cleared by hardware to disable the
                                                   channel after the last AXI transfer of the DMA transfer
                                                   to the destination has completed. Software can therefore
                                                   poll this bit to determine when this channel is free for
                                                   a new DMA transfer.                                                       */
    __IM  uint32_t            : 2;
    __OM  uint32_t ch_en_we   : 6;            /*!< [13..8] Channel enable write enable                                       */
  } b;                                        /*!< bit fields for gdma_ch_en_reg */
} gdma_ch_en_reg_t, *pgdma_ch_en_reg_t;

/**
  \brief Union type to access gdma_ch_reset_reg (@ 0x000000F8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F8) Channel Reset Register                                     */
  
  struct {
    __IOM uint32_t ch_reset_en : 6;           /*!< [5..0] Reset channel enable. Set this bit to reset a channel;
                                                   clear this bit to finish the reset flow and release the
                                                   channel. 0 : reset the channel finish 1 : reset the channel
                                                   start The reset bit is not automatically cleared by hardware.
                                                   Software must clear it to finish software reset.                          */
    __IM  uint32_t            : 2;
    __IM  uint32_t ch_reset_en_we : 6;        /*!< [13..8] Channel reset write enable                                        */
  } b;                                        /*!< bit fields for gdma_ch_reset_reg */
} gdma_ch_reset_reg_t, *pgdma_ch_reset_reg_t;

/** @} */ /* End of group ls_hal_gdma_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_GDMA_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_GDMA_TYPE_H_

