/***************************************************************************
 * arch/arm/src/cxd56xx/cxd56_audio_bca_reg.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_AUDIO_BCA_REG_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_AUDIO_BCA_REG_H

/***************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/audio.h>

/***************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef enum
{
  BCA_Mic_In_start_adr,
  BCA_Mic_In_sample_no,
  BCA_Mic_In_rtd_trg,
  BCA_Mic_In_nointr,
  BCA_Mic_In_bitwt,
  BCA_Mic_In_ch8_sel,
  BCA_Mic_In_ch7_sel,
  BCA_Mic_In_ch6_sel,
  BCA_Mic_In_ch5_sel,
  BCA_Mic_In_ch4_sel,
  BCA_Mic_In_ch3_sel,
  BCA_Mic_In_ch2_sel,
  BCA_Mic_In_ch1_sel,
  BCA_Mic_In_start,
  BCA_Mic_In_error_setting,
  BCA_Mic_In_monbuf,
  BCA_I2s1_In_start_adr,
  BCA_I2s1_In_sample_no,
  BCA_I2s1_In_rtd_trg,
  BCA_I2s1_In_nointr,
  BCA_I2s1_In_bitwt,
  BCA_I2s1_In_ch2_sel,
  BCA_I2s1_In_ch1_sel,
  BCA_I2s1_In_Mon_start,
  BCA_I2s1_In_Mon_error_setting,
  BCA_I2s1_In_Mon_monbuf,
  BCA_I2s2_In_start_adr,
  BCA_I2s2_In_sample_no,
  BCA_I2s2_In_rtd_trg,
  BCA_I2s2_In_nointr,
  BCA_I2s2_In_bitwt,
  BCA_I2s2_In_ch2_sel,
  BCA_I2s2_In_ch1_sel,
  BCA_I2s2_In_Mon_start,
  BCA_I2s2_In_Mon_error_setting,
  BCA_I2s2_In_Mon_monbuf,
  BCA_I2s1_Out_start_adr,
  BCA_I2s1_Out_sample_no,
  BCA_I2s1_Out_rtd_trg,
  BCA_I2s1_Out_nointr,
  BCA_I2s1_Out_bitwt,
  BCA_I2s1_Out_sd1_r_sel,
  BCA_I2s1_Out_sd1_l_sel,
  BCA_I2s1_Out_Mon_start,
  BCA_I2s1_Out_Mon_error_setting,
  BCA_I2s1_Out_Mon_monbuf,
  BCA_I2s2_Out_start_adr,
  BCA_I2s2_Out_sample_no,
  BCA_I2s2_Out_rtd_trg,
  BCA_I2s2_Out_nointr,
  BCA_I2s2_Out_bitwt,
  BCA_I2s2_Out_sd1_r_sel,
  BCA_I2s2_Out_sd1_l_sel,
  BCA_I2s2_Out_Mon_start,
  BCA_I2s2_Out_Mon_error_setting,
  BCA_I2s2_Out_Mon_monbuf,
  BCA_I2s_ensel,
  BCA_Mic_In_prdat_u,
  BCA_I2s1_In_prdat_u,
  BCA_I2s2_In_prdat_u,
  BCA_I2s1_Out_prdat_d,
  BCA_I2s2_Out_prdat_d,
  BCA_Mic_Int_Ctrl_done_mic,
  BCA_Mic_Int_Ctrl_err_mic,
  BCA_Mic_Int_Ctrl_smp_mic,
  BCA_Mic_Int_Ctrl_cmb_mic,
  BCA_I2s1_Int_Ctrl_done_i2so,
  BCA_I2s1_Int_Ctrl_err_i2so,
  BCA_I2s1_Int_Ctrl_done_i2si,
  BCA_I2s1_Int_Ctrl_err_i2si,
  BCA_I2s1_Int_Ctrl_smp_i2s,
  BCA_I2s1_Int_Ctrl_cmb_i2s,
  BCA_I2s2_Int_Ctrl_done_i2so,
  BCA_I2s2_Int_Ctrl_err_i2so,
  BCA_I2s2_Int_Ctrl_done_i2si,
  BCA_I2s2_Int_Ctrl_err_i2si,
  BCA_I2s2_Int_Ctrl_smp_i2s,
  BCA_I2s2_Int_Ctrl_cmb_i2s,
  BCA_Mic_Int_Mask_done_mic,
  BCA_Mic_Int_Mask_err_mic,
  BCA_Mic_Int_Mask_smp_mic,
  BCA_Mic_Int_Mask_cmb_mic,
  BCA_Mic_Int_Mask_nostpmsk,
  BCA_Mic_Int_Mask_srst_mic,
  BCA_I2s1_Int_Mask_done_i2so,
  BCA_I2s1_Int_Mask_err_i2so,
  BCA_I2s1_Int_Mask_done_i2si,
  BCA_I2s1_Int_Mask_err_i2si,
  BCA_I2s1_Int_Mask_smp_i2s,
  BCA_I2s1_Int_Mask_cmb_i2s,
  BCA_I2s1_Int_Mask_nostpmsk,
  BCA_I2s1_Int_Mask_srst_i2s,
  BCA_I2s2_Int_Mask_done_i2so,
  BCA_I2s2_Int_Mask_err_i2so,
  BCA_I2s2_Int_Mask_done_i2si,
  BCA_I2s2_Int_Mask_err_i2si,
  BCA_I2s2_Int_Mask_smp_i2s,
  BCA_I2s2_Int_Mask_cmb_i2s,
  BCA_I2s2_Int_Mask_nostpmsk,
  BCA_I2s2_Int_Mask_srst_i2s,
  BCA_Int_m_hresp_err,
  BCA_Int_m_i2s1_bck_err1,
  BCA_Int_m_i2s1_bck_err2,
  BCA_Int_m_anc_faint,
  BCA_Int_m_ovf_smasl,
  BCA_Int_m_ovf_smasr,
  BCA_Int_m_ovf_dnc1l,
  BCA_Int_m_ovf_dnc1r,
  BCA_Int_m_ovf_dnc2l,
  BCA_Int_m_ovf_dnc2r,
  BCA_Int_clr_hresp_err,
  BCA_Int_clr_i2s1_bck_err1,
  BCA_Int_clr_i2S1_bck_err2,
  BCA_Int_clr_anc_faint,
  BCA_Int_clr_ovf_smasl,
  BCA_Int_clr_ovf_smasr,
  BCA_Int_clr_ovf_dnc1l,
  BCA_Int_clr_ovf_dnc1r,
  BCA_Int_clr_ovf_dnc2l,
  BCA_Int_clr_ovf_dnc2r,
  BCA_Int_hresp_err,
  BCA_Int_i2s_bck_err1,
  BCA_Int_i2s_bck_err2,
  BCA_Int_anc_faint,
  BCA_Int_ovf_smasl,
  BCA_Int_ovf_smasr,
  BCA_Int_ovf_dnc1l,
  BCA_Int_ovf_dnc1r,
  BCA_Int_ovf_dnc2l,
  BCA_Int_ovf_dnc2r,
  BCA_Dbg_Mic_ch1_data,
  BCA_Dbg_Mic_ch2_data,
  BCA_Dbg_Mic_ch3_data,
  BCA_Dbg_Mic_ch4_data,
  BCA_Dbg_Mic_ch5_data,
  BCA_Dbg_Mic_ch6_data,
  BCA_Dbg_Mic_ch7_data,
  BCA_Dbg_Mic_ch8_data,
  BCA_Dbg_I2s1_u_ch1_data,
  BCA_Dbg_I2s1_u_ch2_data,
  BCA_Dbg_I2s1_d_ch1_data,
  BCA_Dbg_I2s1_d_ch2_data,
  BCA_Dbg_I2s2_u_ch1_data,
  BCA_Dbg_I2s2_u_ch2_data,
  BCA_Dbg_I2s2_d_ch1_data,
  BCA_Dbg_I2s2_d_ch2_data,
  BCA_Dbg_Ctrl_mic_dbg_en,
  BCA_Dbg_Ctrl_I2s1_dbg_u_en,
  BCA_Dbg_Ctrl_I2s1_dbg_d_en,
  BCA_Dbg_Ctrl_I2s2_dbg_u_en,
  BCA_Dbg_Ctrl_I2s2_dbg_d_en,
  BCA_Clk_En_ahbmstr_mic_en,
  BCA_Clk_En_ahbmstr_I2s1_en,
  BCA_Clk_En_ahbmstr_I2s2_en,
  BCA_Mclk_Mon_thresh,
  AHB_Master_Mic_Mask,
  AHB_Master_I2s1_Mask,
  AHB_Master_I2s2_Mask,
  BCA_REG_MAX_ENTRY
} BCA_REG_ID;

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

/***************************************************************************
 * Public Types
 ****************************************************************************/

/***************************************************************************
 * Public Data
 ****************************************************************************/

/***************************************************************************
 * Inline Functions
 ****************************************************************************/

/***************************************************************************
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
                                        FAR cxd56_audio_dma_mstate_t *state);

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

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_AUDIO_BCA_REG_H */
