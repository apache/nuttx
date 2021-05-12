/**************************************************************************//**
* @file     hal_lpcram.h
* @brief    The header file of hal_lpcram.c.
* @version  1.00
* @date     2017-08-22
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

#ifndef _HAL_LPCRAM_H_
#define _HAL_LPCRAM_H_
#include "cmsis.h"
#include "rtl8710c_lpcram_type.h"

#ifdef  __cplusplus
extern "C"
{
#endif

typedef struct command_address_s {
    u8 ca[6];
} command_address_t, *pcommand_address_t;

typedef struct lpcram_timing_s {
    u32 tpu_ns;
    u32 tcem_ns;
    u32 tcph_ns;
    u32 speed_MHz;
} lpcram_timing_t, *plpcram_timing_t;

typedef struct lpcramc_latency_s {
    u8 dfi_cs_wr_dly;
    u8 dfi_cs_rd_dly;
    u8 tphy_wrdata;
    u8 fix_tphy_lat;
    u8 tphy_rddata_en;
    u8 dfi_path_dly;                                                              
} lpcramc_latency_t, *plpcramc_latency_t;

enum lpcram_burst_length_e {
    Burst128Bytes = 0,
    Busrt64Bytes = 1,
    Burst16Bytes = 2,
    Burst32Bytes = 3
};
typedef uint8_t lpcram_burst_length_t;

enum lpcram_burst_type_e {
    LinearBurstType = 0,
    WrapBurstType = 1
};
typedef uint8_t lpcram_burst_type_t;

enum lpcram_latency_type_e {
    NotFixLatency = 0,              //!< Variable initial latency, 1 or 2 times initial latency depending on RWDS during CA cycles
    FixLatency = 1                  //!< Fixed 2 times initial latency (default setting of lpcram)
};
typedef uint8_t lpcram_latency_type_t;

enum lpcram_latency_e {
    FiveClockLentency = 0,
    SixClockLentency = 1,
    ThreeClockLentency = 14,
    FourClockLentency = 15
};
typedef uint8_t lpcram_latency_t;

enum lpcram_drive_strength_e {
    FiftyOhms = 0,
    ThirtyFiveOhms = 1,
    OneHundredOhms = 2,
    TwoHundredOhms = 3
};
typedef uint8_t lpcram_drive_strength_t;

enum lpcram_deep_power_down_e {
    EnterDeepPowerDown = 0,
    ExitDeepPowerDown = 1
};
typedef uint8_t lpcram_deep_power_down_t;

enum lpcram_hybrid_sleep_mode_e {
    ExitHybridSleepMode = 0,
    EnterHybridSleepMode = 1 
};
typedef uint8_t lpcram_hybrid_sleep_mode_t;


/**
  \brief  Enumeration to define the range of partial array self refresh
*/
enum lpcram_pasr_e {
    FullArray = 0,
    BottomHalfArray = 1,
    BottomOneFourthArray = 2,
    BottomOneEigththArray = 3,
    None = 4,
    TopHalfArray = 5,
    TopOneFourthArray = 6,
    TopOneEighthArray = 7
};
typedef uint8_t lpcram_pasr_t;

enum lpcram_refresh_rate_e {
    FastRate = 0,
    NormalRate = 1
};
typedef uint8_t lpcram_refresh_rate_t;

enum lpcram_dpin_mode_e {
    DpinRead = 0,
    DpinWrite = 1
};
typedef uint8_t lpcram_dpin_mode_t;

enum lpcram_dpdri_e {
    DpinReadIndex0 = 0,
    DpinReadIndex1 = 1,
    DpinReadIndex2 = 2,
    DpinReadIndex3 = 3,
    DpinWriteIndex0 = 4,
    DpinWriteIndex1 = 5,
    DpinWriteIndex2 = 6,
    DpinWriteIndex3 = 7,
    DpinWriteByteEn = 8
};
typedef uint8_t lpcram_dpdr_index_t;

void hal_lpcram_resume_memory_access(VOID);
void hal_lpcram_reg_write(pcommand_address_t ca_bytes, u16 data);
u16 hal_lpcram_reg_read(pcommand_address_t ca_bytes);
void hal_lpcram_deep_power_down_ctrl(u8 en);
void hal_lpcram_hybrid_sleep_ctrl(u8 en);
void hal_lpcram_write_cr0(lpcram_mr0_t cr0);
void hal_lpcram_write_cr1(lpcram_mr1_t cr1);
u16 hal_lpcram_read_cr0(VOID);
u16 hal_lpcram_read_cr1(VOID);
void hal_lpcram_set_cr0(u8 burst_len, u8 burst_type, u8 fix_laten_en, u8 ini_laten, u8 drive_stren);
void hal_lpcram_set_cr1(u8 pasr, u8 refresh_rate);
void hal_lpcram_set_latency(u8 latency);
void hal_lpcram_set_iocr0(plpcramc_latency_t latency_info);
void hal_lpcram_set_drr(plpcram_timing_t timing_info);
void hal_lpcram_pin_ctl(u8 en);
void hal_lpcram_init(VOID);
void hal_lpcram_deinit(VOID);
hal_status_t hal_lpcram_is_valid (void);

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_LPCRAM_H_"

