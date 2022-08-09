/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_bca_reg.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_BCA_REG_H
#define __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_BCA_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/audio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA_STATE_BIT_AC_DONE        1
#define DMA_STATE_BIT_AC_ERR         2
#define DMA_STATE_BIT_AC_CMB         8
#define DMA_STATE_BIT_I2S_OUT_DONE   1
#define DMA_STATE_BIT_I2S_OUT_ERR    2
#define DMA_STATE_BIT_I2S_IN_DONE    4
#define DMA_STATE_BIT_I2S_IN_ERR     8
#define DMA_STATE_BIT_I2S_CMB        32

#define DMA_MSTATE_START             1
#define DMA_MSTART_READY             0

#define DMA_MSTATE_ERR_OK
#define DMA_MSTATE_ERR_NO_ENABLE_CH  1
#define DMA_MSTATE_ERR_CH1_4_INVALID 2
#define DMA_MSTATE_ERR_CH5_8_INVALID 4

#define DMA_MSTATE_BUF_EMPTY         3

#define DMA_CMD_FIFO_NOT_FULL        1

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  BCA_MIC_IN_START_ADR,
  BCA_MIC_IN_SAMPLE_NO,
  BCA_MIC_IN_RTD_TRG,
  BCA_MIC_IN_NOINTR,
  BCA_MIC_IN_BITWT,
  BCA_MIC_IN_CH8_SEL,
  BCA_MIC_IN_CH7_SEL,
  BCA_MIC_IN_CH6_SEL,
  BCA_MIC_IN_CH5_SEL,
  BCA_MIC_IN_CH4_SEL,
  BCA_MIC_IN_CH3_SEL,
  BCA_MIC_IN_CH2_SEL,
  BCA_MIC_IN_CH1_SEL,
  BCA_MIC_IN_START,
  BCA_MIC_IN_ERROR_SETTING,
  BCA_MIC_IN_MONBUF,
  BCA_I2S1_IN_START_ADR,
  BCA_I2S1_IN_SAMPLE_NO,
  BCA_I2S1_IN_RTD_TRG,
  BCA_I2S1_IN_NOINTR,
  BCA_I2S1_IN_BITWT,
  BCA_I2S1_IN_CH2_SEL,
  BCA_I2S1_IN_CH1_SEL,
  BCA_I2S1_IN_MON_START,
  BCA_I2S1_IN_MON_ERROR_SETTING,
  BCA_I2S1_IN_MON_MONBUF,
  BCA_I2S2_IN_START_ADR,
  BCA_I2S2_IN_SAMPLE_NO,
  BCA_I2S2_IN_RTD_TRG,
  BCA_I2S2_IN_NOINTR,
  BCA_I2S2_IN_BITWT,
  BCA_I2S2_IN_CH2_SEL,
  BCA_I2S2_IN_CH1_SEL,
  BCA_I2S2_IN_MON_START,
  BCA_I2S2_IN_MON_ERROR_SETTING,
  BCA_I2S2_IN_MON_MONBUF,
  BCA_I2S1_OUT_START_ADR,
  BCA_I2S1_OUT_SAMPLE_NO,
  BCA_I2S1_OUT_RTD_TRG,
  BCA_I2S1_OUT_NOINTR,
  BCA_I2S1_OUT_BITWT,
  BCA_I2S1_OUT_SD1_R_SEL,
  BCA_I2S1_OUT_SD1_L_SEL,
  BCA_I2S1_OUT_MON_START,
  BCA_I2S1_OUT_MON_ERROR_SETTING,
  BCA_I2S1_OUT_MON_MONBUF,
  BCA_I2S2_OUT_START_ADR,
  BCA_I2S2_OUT_SAMPLE_NO,
  BCA_I2S2_OUT_RTD_TRG,
  BCA_I2S2_OUT_NOINTR,
  BCA_I2S2_OUT_BITWT,
  BCA_I2S2_OUT_SD1_R_SEL,
  BCA_I2S2_OUT_SD1_L_SEL,
  BCA_I2S2_OUT_MON_START,
  BCA_I2S2_OUT_MON_ERROR_SETTING,
  BCA_I2S2_OUT_MON_MONBUF,
  BCA_I2S_ENSEL,
  BCA_MIC_IN_PRDAT_U,
  BCA_I2S1_IN_PRDAT_U,
  BCA_I2S2_IN_PRDAT_U,
  BCA_I2S1_OUT_PRDAT_D,
  BCA_I2S2_OUT_PRDAT_D,
  BCA_MIC_INT_CTRL_DONE_MIC,
  BCA_MIC_INT_CTRL_ERR_MIC,
  BCA_MIC_INT_CTRL_SMP_MIC,
  BCA_MIC_INT_CTRL_CMB_MIC,
  BCA_I2S1_INT_CTRL_DONE_I2SO,
  BCA_I2S1_INT_CTRL_ERR_I2SO,
  BCA_I2S1_INT_CTRL_DONE_I2SI,
  BCA_I2S1_INT_CTRL_ERR_I2SI,
  BCA_I2S1_INT_CTRL_SMP_I2S,
  BCA_I2S1_INT_CTRL_CMB_I2S,
  BCA_I2S2_INT_CTRL_DONE_I2SO,
  BCA_I2S2_INT_CTRL_ERR_I2SO,
  BCA_I2S2_INT_CTRL_DONE_I2SI,
  BCA_I2S2_INT_CTRL_ERR_I2SI,
  BCA_I2S2_INT_CTRL_SMP_I2S,
  BCA_I2S2_INT_CTRL_CMB_I2S,
  BCA_MIC_INT_MASK_DONE_MIC,
  BCA_MIC_INT_MASK_ERR_MIC,
  BCA_MIC_INT_MASK_SMP_MIC,
  BCA_MIC_INT_MASK_CMB_MIC,
  BCA_MIC_INT_MASK_NOSTPMSK,
  BCA_MIC_INT_MASK_SRST_MIC,
  BCA_I2S1_INT_MASK_DONE_I2SO,
  BCA_I2S1_INT_MASK_ERR_I2SO,
  BCA_I2S1_INT_MASK_DONE_I2SI,
  BCA_I2S1_INT_MASK_ERR_I2SI,
  BCA_I2S1_INT_MASK_SMP_I2S,
  BCA_I2S1_INT_MASK_CMB_I2S,
  BCA_I2S1_INT_MASK_NOSTPMSK,
  BCA_I2S1_INT_MASK_SRST_I2S,
  BCA_I2S2_INT_MASK_DONE_I2SO,
  BCA_I2S2_INT_MASK_ERR_I2SO,
  BCA_I2S2_INT_MASK_DONE_I2SI,
  BCA_I2S2_INT_MASK_ERR_I2SI,
  BCA_I2S2_INT_MASK_SMP_I2S,
  BCA_I2S2_INT_MASK_CMB_I2S,
  BCA_I2S2_INT_MASK_NOSTPMSK,
  BCA_I2S2_INT_MASK_SRST_I2S,
  BCA_INT_M_HRESP_ERR,
  BCA_INT_M_I2S1_BCL_ERR1,
  BCA_INT_M_I2S1_BCL_ERR2,
  BCA_INT_M_ANC_FAINT,
  BCA_INT_M_OVF_SMASL,
  BCA_INT_M_OVF_SMASR,
  BCA_INT_M_OVF_DNC1L,
  BCA_INT_M_OVF_DNC1R,
  BCA_INT_M_OVF_DNC2L,
  BCA_INT_M_OVF_DNC2R,
  BCA_INT_CLR_HRESP_ERR,
  BCA_INT_CLR_I2S1_BCK_ERR1,
  BCA_INT_CLR_I2S1_BCK_ERR2,
  BCA_INT_CLR_ANC_FAINT,
  BCA_INT_CLR_OVF_SMASL,
  BCA_INT_CLR_OVF_SMASR,
  BCA_INT_CLR_OVF_DNC1L,
  BCA_INT_CLR_OVF_DNC1R,
  BCA_INT_CLR_OVF_DNC2L,
  BCA_INT_CLR_OVF_DNC2R,
  BCA_INT_HRESP_ERR,
  BCA_INT_I2S_BCK_ERR1,
  BCA_INT_I2S_BCK_ERR2,
  BCA_INT_ANC_FAINT,
  BCA_INT_OVF_SMASL,
  BCA_INT_OVF_SMASR,
  BCA_INT_OVF_DNC1L,
  BCA_INT_OVF_DNC1R,
  BCA_INT_OVF_DNC2L,
  BCA_INT_OVF_DNC2R,
  BCA_DBG_MIC_CH1_DATA,
  BCA_DBG_MIC_CH2_DATA,
  BCA_DBG_MIC_CH3_DATA,
  BCA_DBG_MIC_CH4_DATA,
  BCA_DBG_MIC_CH5_DATA,
  BCA_DBG_MIC_CH6_DATA,
  BCA_DBG_MIC_CH7_DATA,
  BCA_DBG_MIC_CH8_DATA,
  BCA_DBG_I2S1_U_CH1_DATA,
  BCA_DBG_I2S1_U_CH2_DATA,
  BCA_DBG_I2S1_D_CH1_DATA,
  BCA_DBG_I2S1_D_CH2_DATA,
  BCA_DBG_I2S2_U_CH1_DATA,
  BCA_DBG_I2S2_U_CH2_DATA,
  BCA_DBG_I2S2_D_CH1_DATA,
  BCA_DBG_I2S2_D_CH2_DATA,
  BCA_DBG_CTRL_MIC_DBG_EN,
  BCA_DBG_CTRL_I2S1_DBG_U_EN,
  BCA_DBG_CTRL_I2S1_DBG_D_EN,
  BCA_DBG_CTRL_I2S2_DBG_U_EN,
  BCA_DBG_CTRL_I2S2_DBG_D_EN,
  BCA_CLK_EN_AHBMASTER_MIC_EN,
  BCA_CLK_EN_AHBMASTER_I2S1_EN,
  BCA_CLK_EN_AHBMASTER_I2S2_EN,
  BCA_MCLK_MON_THRESH,
  AHB_MASTER_MIC_MASK,
  AHB_MASTER_I2S1_MASK,
  AHB_MASTER_I2S2_MASK,
  BCA_REG_MAX_ENTRY
} BCA_REG_ID;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void cxd56_audio_bca_reg_clear_bck_err_int(void);
void cxd56_audio_bca_reg_set_smaster(void);
void cxd56_audio_bca_reg_set_datarate(uint8_t clk_mode);

void cxd56_audio_bca_reg_en_fmt24(cxd56_audio_dma_t handle, uint8_t ch_num);
void cxd56_audio_bca_reg_en_fmt16(cxd56_audio_dma_t handle, uint8_t ch_num);

void cxd56_audio_bca_reg_en_bus_err_int(void);
void cxd56_audio_bca_reg_dis_bus_err_int(void);

void cxd56_audio_bca_reg_get_dma_mstate(cxd56_audio_dma_t handle,
                                        cxd56_audio_dma_mstate_t *state);

uint32_t cxd56_audio_bca_reg_get_dma_done_state_mic(void);
uint32_t cxd56_audio_bca_reg_get_dma_done_state_i2s1(void);
uint32_t cxd56_audio_bca_reg_get_dma_done_state_i2s2(void);

void cxd56_audio_bca_reg_mask_done_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_unmask_done_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_clear_done_int(cxd56_audio_dma_t handle);

void cxd56_audio_bca_reg_clear_dma_done_state_mic(uint32_t value);
void cxd56_audio_bca_reg_clear_dma_done_state_i2s1(uint32_t value);
void cxd56_audio_bca_reg_clear_dma_done_state_i2s2(uint32_t value);

bool cxd56_audio_bca_reg_is_dma_fifo_empty(cxd56_audio_dma_t handle);

void cxd56_audio_bca_reg_mask_err_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_unmask_err_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_clear_err_int(cxd56_audio_dma_t handle);

void cxd56_audio_bca_reg_mask_cmb_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_unmask_cmb_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_clear_cmb_int(cxd56_audio_dma_t handle);

uint32_t cxd56_audio_bca_reg_get_int_status(void);
void cxd56_audio_bca_reg_clear_int_status(uint32_t int_au);

void cxd56_audio_bca_reg_mask_bus_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_unmask_bus_int(cxd56_audio_dma_t handle);

void cxd56_audio_bca_reg_set_start_addr(cxd56_audio_dma_t handle,
                                        uint32_t addr);
void cxd56_audio_bca_reg_set_sample_no(cxd56_audio_dma_t handle,
                                       uint32_t sample);
void cxd56_audio_bca_reg_start_dma(cxd56_audio_dma_t handle,
                                   bool nointr);
void cxd56_audio_bca_reg_stop_dma(cxd56_audio_dma_t handle);

bool cxd56_audio_bca_reg_is_done_int(cxd56_audio_dma_t handle);
bool cxd56_audio_bca_reg_is_err_int(cxd56_audio_dma_t handle);

bool cxd56_audio_bca_reg_is_smp_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_mask_smp_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_unmask_smp_int(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_clear_smp_int(cxd56_audio_dma_t handle);

uint32_t cxd56_audio_bca_reg_get_mon_state_err(cxd56_audio_dma_t handle);
uint32_t cxd56_audio_bca_reg_get_mon_state_start(cxd56_audio_dma_t handle);
uint32_t cxd56_audio_bca_reg_get_mon_state_buf(cxd56_audio_dma_t handle);
uint32_t cxd56_audio_bca_reg_get_dma_state(cxd56_audio_dma_t handle);
void cxd56_audio_bca_reg_reset_chsel(cxd56_audio_dma_t handle);

#endif /* __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_BCA_REG_H */
