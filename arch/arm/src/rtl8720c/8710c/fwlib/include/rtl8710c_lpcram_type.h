/**************************************************************************//**
 * @file      rtl8710c_lpcram_type.h
 * @brief
 * @version   V1.00
 * @date      2018-7-2 11:12:4
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

#ifndef _RTL8710C_LPCRAM_TYPE_H_
#define _RTL8710C_LPCRAM_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_LPCRAM_REG_TYPE

/**
 * @addtogroup hs_hal_lpcram_reg LPCRAM Registers.
 * @ingroup hs_hal_psram
 * @{
 */

/**
  \brief Union type to access lpcram_ccr (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) Configuration Control Register                             */
  
  struct {
    __IOM uint32_t init       : 1;            /*!< [0..0] Start to issue PSRAM initialization sequence. Write 0
                                                   : Disable PSRAM initialization function. Write 1 : Enable
                                                   PSRAM initialization function. Read 0 : PSRAM initialization
                                                   function is still active. Read 1 : PSRAM initialization
                                                   function is done (INIT_DONE).                                             */
    __IM  uint32_t            : 2;
    __IOM uint32_t dpin       : 1;            /*!< [3..3] start to set PSRAM command function Wrtie 0 : Not to
                                                   start PSRAM DPIN function. Wrtie 1 : Start to execute DPIN
                                                   function. Read 0 : DPIN function is still active. Read
                                                   1 : DPIN function to PSRAM is done (DPIN_DONE)                            */
    __IM  uint32_t            : 4;
    __IOM uint32_t flush_fifo : 1;            /*!< [8..8] Flush all FIFO in PSRAM_LPC_CTRL Write 0 : Not to flush
                                                   FIFO Write 1 : Start to flush FIFO Read 0 : Flushing FIFO
                                                   operation is done. Read 1 : It is sitll active to flush
                                                   FIFO.                                                                     */
    __IM  uint32_t            : 22;
    __IOM uint32_t cr_update  : 1;            /*!< [31..31] Update the internal timing control registers or Quick
                                                   INIT done.                                                                */
  } b;                                        /*!< bit fields for lpcram_ccr */
} lpcram_ccr_t, *plpcram_ccr_t;

/**
  \brief Union type to access lpcram_iocr0 (@ 0x00000008).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000008) IO Configuration Control Register 0                        */
  
  struct {
    __IOM uint32_t dfi_cs_wr_dly : 4;         /*!< [3..0] Setting the latency contrast between PHY read data enable
                                                   path latency and PHY command path latency. The smallest
                                                   value is 0                                                                */
    __IOM uint32_t dfi_cs_rd_dly : 4;         /*!< [7..4] Setting the latency contrast between PHY write data path
                                                   latency and PHY command path latency. The smallest value
                                                   is 0.                                                                     */
    __IM  uint32_t            : 4;
    __IOM uint32_t tphy_wrdata : 5;           /*!< [16..12] Setting the delay latency from DFI write command to
                                                   DFI write data. The smallest value is 1.                                  */
    __IM  uint32_t            : 2;
    __IOM uint32_t fix_tphy_lat : 1;          /*!< [19..19] PSRAM_LPC_CTRL uses TPHY_WRDATA or TPHY_RDDATA only.
                                                   0 : Disable 1 : Enable                                                    */
    __IOM uint32_t tphy_rddata_en : 5;        /*!< [24..20] Setting the delay latency from DFI read command to
                                                   dri_rddata_en signal. The smallest value is 1.                            */
    __IOM uint32_t dfi_path_dly : 5;          /*!< [29..25] Select which TPHY_WRDATA/TPHY RDDATA cycle to sample
                                                   dfi_latency_en (dfi_latency_en_mux), the value should be
                                                   no less than 2.                                                           */
  } b;                                        /*!< bit fields for lpcram_iocr0 */
} lpcram_iocr0_t, *plpcram_iocr0_t;

/**
  \brief Union type to access lpcram_csr (@ 0x0000000C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000000C) Controller Status Register                                 */
  
  struct {
    __IM  uint32_t            : 8;
    __IOM uint32_t mem_idle   : 1;            /*!< [8..8] Disable memory access state. Write to set memory access
                                                   idle state. 0 : Enable memory access state. 1 : Set memory
                                                   access idel state, disable memory access. Read to check
                                                   if memory access state is in idel. 0 : Access PSRAM is
                                                   allowed. 1 : Memory state is idle and user cannot access
                                                   PSRAM through data bus.                                                   */
    __IM  uint32_t            : 8;
    __IOM uint32_t dpin_mode  : 2;            /*!< [18..17] DPIN mode decode : 00 : DPIN read data mode 01 : DPIN
                                                   write data mode 01 DPIN write data mode with iocr0[19]
                                                   for register write                                                        */
  } b;                                        /*!< bit fields for lpcram_csr */
} lpcram_csr_t, *plpcram_csr_t;

/**
  \brief Union type to access lpcram_drr (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) Device Refresh Power Up Register                           */
  
  struct {
    __IOM uint32_t tcph       : 4;            /*!< [3..0] PSRAM CS# pin HIGH cycles between subsequent command.              */
    __IOM uint32_t tcem       : 11;           /*!< [14..4] Maximum average refresh commands delay cycles.                    */
    __IOM uint32_t tpu        : 7;            /*!< [21..15] For PSRAM initialization flow, it has to wait this
                                                   period until the first access.                                            */
  } b;                                        /*!< bit fields for lpcram_drr */
} lpcram_drr_t, *plpcram_drr_t;

/**
  \brief Union type to access lpcram_cmd_dpin_ndge (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000024) Device Cmd Addr Pin Register (Negative Edge)               */
  
  struct {
    __IOM uint32_t addr       : 24;           /*!< [23..0] ADDR = {CA[39:32], CA[23:16], CA[7:0]}                            */
  } b;                                        /*!< bit fields for lpcram_cmd_dpin_ndge */
} lpcram_cmd_dpin_ndge_t, *plpcram_cmd_dpin_ndge_t;

/**
  \brief Union type to access lpcram_cmd_dpin (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) Device Cmd Addr Pin Register (Positive Edge)               */
  
  struct {
    __IOM uint32_t addr       : 24;           /*!< [23..0] ADDR = {CA[47:40], CA[31:24], CA[15:8]}                           */
  } b;                                        /*!< bit fields for lpcram_cmd_dpin */
} lpcram_cmd_dpin_t, *plpcram_cmd_dpin_t;

/**
  \brief Union type to access lpcram_cr_tdpin (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) Tie DPIN Register                                          */
  
  struct {
    __IOM uint32_t dfi_reset_n : 1;           /*!< [0..0] This is tie-dpin function. Software can use this bit
                                                   to control dfi_reset_n to active or inactive. 1 : dfi_reset_n
                                                   inactive. 0 : dfi_reset_n active.                                         */
  } b;                                        /*!< bit fields for lpcram_cr_tdpin */
} lpcram_cr_tdpin_t, *plpcram_cr_tdpin_t;

/**
  \brief Union type to access lpcram_mr_info (@ 0x00000030).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000030) Mode Latency Information Register                          */
  
  struct {
    __IOM uint32_t wl         : 5;            /*!< [4..0] Indicate PSRAM write latency counter. Refer to PSRAM
                                                   spec CR0 register altency counter field.                                  */
    __IOM uint32_t rl         : 5;            /*!< [9..5] Indicate PSRAM read latency counter. Refer to PSRAM spec
                                                   CR0 register altency counter field.                                       */
  } b;                                        /*!< bit fields for lpcram_mr_info */
} lpcram_mr_info_t, *plpcram_mr_info_t;

/**
  \brief Union type to access lpcram_mr0 (@ 0x00000034).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000034) Device CR0 Register                                        */
  
  struct {
    __IOM uint32_t burst_length : 2;          /*!< [1..0] Burst length per transaction. 00 : 128 bytes 01 : 64
                                                   bytes 10 : 16 bytes 11 : 32 bytes                                         */
    __IOM uint32_t burst_type : 1;            /*!< [2..2] Wrapped burst sequences in legacy wrapped burst manner             */
    __IOM uint32_t fixed_latency_en : 1;      /*!< [3..3] Latency mode control. 0 : variable initial latency, 1
                                                   or 2 times initial latency depending on RWDS during CA
                                                   cycles. 1 : fixed 2 times initial latency.                                */
    __IOM uint32_t ini_latency : 4;           /*!< [7..4] Initial Latency Count. 0000 : 5 clocks latency 0001 :
                                                   6 clocks latency 1110 : 3 clocks latency 1111 : 4 clocks
                                                   latency                                                                   */
    __IM  uint32_t            : 4;
    __IOM uint32_t drive_strength : 3;        /*!< [14..12] 000 : 50 ohms 001 : 35 ohms 010 : 100 ohms 011 : 200
                                                   ohms                                                                      */
    __IOM uint32_t deep_power_down_en : 1;    /*!< [15..15] Enable Deep Power Down mode. 0 : Enter Deep Power Down
                                                   mode. 1 : Normal operation.                                               */
  } b;                                        /*!< bit fields for lpcram_mr0 */
} lpcram_mr0_t, *plpcram_mr0_t;

/**
  \brief Union type to access lpcram_mr1 (@ 0x00000038).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000038) Device CR1 Register                                        */
  
  struct {
    __IOM uint32_t pasr       : 3;            /*!< [2..0] Partial array self refresh. 000 : full array 001 : bottom
                                                   1/2 array 010 : bottom 1/4 array 011 : bottom 1/8 array
                                                   100 : none 101 : top 1/2 array 110 : top 1/4 array 111
                                                   : top 1/8 array                                                           */
    __IM  uint32_t            : 2;
    __IOM uint32_t hybrid_sleep_en : 1;       /*!< [5..5] Enter Hybird Sleep mode control. 0 : not in hybrid sleep
                                                   mode. 1 : entering hybrid sleep mode.                                     */
    __IOM uint32_t refresh_rate : 1;          /*!< [6..6] Refresh Multiplier Indicator. 0 : refresh the internal
                                                   cell array using faster rate. 1 : refresh the internal
                                                   cell array using normal rate.                                             */
  } b;                                        /*!< bit fields for lpcram_mr1 */
} lpcram_mr1_t, *plpcram_mr1_t;

/**
  \brief Union type to access lpcram_dpdri (@ 0x00000060).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000060) DPIN Data Index Register                                   */
  
  struct {
    __IOM uint32_t dpin_data_index : 4;       /*!< [3..0] Indicate select which DPIN DATA register. Index 0~3 :
                                                   dpin read data. Index 4~7 : dpin write data. Index 8 :
                                                   dpin write data byteen.                                                   */
  } b;                                        /*!< bit fields for lpcram_dpdri */
} lpcram_dpdri_t, *plpcram_dpdri_t;

/**
  \brief Union type to access lpcram_dpdr (@ 0x00000064).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000064) DPIN Data Register                                         */
  
  struct {
    __IOM uint32_t dpin_data  : 32;           /*!< [31..0] Indicate the data which will be written to PSRAM or
                                                   the data read from PSRAM.                                                 */
  } b;                                        /*!< bit fields for lpcram_dpdr */
} lpcram_dpdr_t, *plpcram_dpdr_t;

/**
  \brief Union type to access lpcram_pctl_svn_id (@ 0x000000F4).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F4) PSRAM_LPC_CTRL Version Numbers                             */
  
  struct {
    __IM  uint32_t release_date : 16;         /*!< [15..0] Indicate PSRAM Controller released date.                          */
    __IM  uint32_t git_cnt    : 16;           /*!< [31..16] Indicate the Git counter of the released date.                   */
  } b;                                        /*!< bit fields for lpcram_pctl_svn_id */
} lpcram_pctl_svn_id_t, *plpcram_pctl_svn_id_t;

/**
  \brief Union type to access lpcram_pctl_idr (@ 0x000000F8).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x000000F8) PSRAM_LPC_CTRL Identification Register                     */
  
  struct {
    __IM  uint32_t cr_pctl_def : 16;          /*!< [15..0] Main support PSRAM type.                                          */
    __IM  uint32_t cr_ver     : 16;           /*!< [31..16] The control register version number.                             */
  } b;                                        /*!< bit fields for lpcram_pctl_idr */
} lpcram_pctl_idr_t, *plpcram_pctl_idr_t;

/**
  \brief Union type to access lpcram_user0_index (@ 0x00000400).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000400) User Extended Index                                        */
  
  struct {
    __IOM uint32_t index_map_addr : 32;       /*!< [31..0] The address index of the indirect access register.                */
  } b;                                        /*!< bit fields for lpcram_user0_index */
} lpcram_user0_index_t, *plpcram_user0_index_t;

/**
  \brief Union type to access lpcram_user0_data (@ 0x00000404).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000404) User Extended Data                                         */
  
  struct {
    __IOM uint32_t data_map_addr : 32;        /*!< [31..0] The data to be written to the indirect access register.           */
  } b;                                        /*!< bit fields for lpcram_user0_data */
} lpcram_user0_data_t, *plpcram_user0_data_t;

/** @} */ /* End of group ls_hal_lpcram_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_LPCRAM_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_LPCRAM_TYPE_H_

