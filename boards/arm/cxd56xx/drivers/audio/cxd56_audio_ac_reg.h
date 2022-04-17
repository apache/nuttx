/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_ac_reg.h
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

#ifndef __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_AC_REG_H
#define __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_AC_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/audio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum audio_codec_register_id_e
{
  RI_REVID = 0,
  RI_DEVICEID,
  RI_PDN_AMICEXT,
  RI_PDN_AMIC1,
  RI_PDN_AMIC2,
  RI_PDN_DAC,
  RI_PDN_LINEIN,
  RI_PDN_MIC,
  RI_PDN_DMIC,
  RI_PDN_DSPB,
  RI_PDN_ANC,
  RI_PDN_DNC1,
  RI_PDN_DNC2,
  RI_PDN_SMSTR,
  RI_PDN_DSPS2,
  RI_PDN_DSPS1,
  RI_PDN_DSPC,
  RI_FS_FS,
  RI_DECIM0_EN,
  RI_DECIM1_EN,
  RI_SDES_EN,
  RI_MCK_AHBMSTR_EN,
  RI_AU_DAT_SEL2,
  RI_AU_DAT_SEL1,
  RI_COD_INSEL3,
  RI_COD_INSEL2,
  RI_COD_INSEL1,
  RI_DSR_RATE,
  RI_DIGSFT,
  RI_SRC1,
  RI_SRC1IN_SEL,
  RI_SRC2,
  RI_SRC2IN_SEL,
  RI_DIF2,
  RI_DIF1,
  RI_SD2MASTER,
  RI_SD1MASTER,
  RI_SDCK_OUTENX,
  RI_HI_RES_MODE,
  RI_HPF2_MODE,
  RI_CIC2IN_SWAP,
  RI_INV_DMIC2L,
  RI_INV_DMIC2R,
  RI_CIC2_GAIN_MODE,
  RI_CIC2IN_SEL,
  RI_ADC2_BOOST,
  RI_INV_AMIC2L,
  RI_INV_AMIC2R,
  RI_HPF1_MODE,
  RI_CIC1IN_SWAP,
  RI_INV_DMIC1L,
  RI_INV_DMIC1R,
  RI_CIC1_GAIN_MODE,
  RI_CIC1IN_SEL,
  RI_ADC1_BOOST,
  RI_ADC_FS,
  RI_INV_AMIC1L,
  RI_INV_AMIC1R,
  RI_HPF4_MODE,
  RI_CIC4IN_SWAP,
  RI_INV_DMIC4L,
  RI_INV_DMIC4R,
  RI_CIC4IN_SEL,
  RI_ADC4_BOOST,
  RI_INV_AMIC4L,
  RI_INV_AMIC4R,
  RI_HPF3_MODE,
  RI_CIC3IN_SWAP,
  RI_INV_DMIC3L,
  RI_INV_DMIC3R,
  RI_CIC3IN_SEL,
  RI_ADC_3_BOOST,
  RI_INV_AMIC3L,
  RI_INV_AMIC3R,
  RI_CIC1_RGAIN,
  RI_CIC1_LGAIN,
  RI_CIC2_RGAIN,
  RI_CIC2_LGAIN,
  RI_CIC3_RGAIN,
  RI_CIC3_LGAIN,
  RI_CIC4_RGAIN,
  RI_CIC4_LGAIN,
  RI_SPC_LIMIT,
  RI_SPC_EN,
  RI_ALC_KNEE,
  RI_ALCTARGET,
  RI_ALC_REC,
  RI_ALC_EN,
  RI_INV_ASP2R,
  RI_INV_ASP2L,
  RI_INV_ASP1R,
  RI_INV_ASP1L,
  RI_ARC,
  RI_ARC_VOL,
  RI_CS_VOL,
  RI_CS_SIGN,
  RI_SDOUT_VOL,
  RI_SDIN2_VOL,
  RI_SDIN1_VOL,
  RI_SDIN1_EN,
  RI_SDIN2_EN,
  RI_SDOUT1_EN,
  RI_SDOUT2_EN,
  RI_MUTE_B,
  RI_BLF_EN,
  RI_TRANS_MODE,
  RI_DAC_VOL,
  RI_LINEIN_VOL,
  RI_BEEP_VOL,
  RI_BEEP_FREQ,
  RI_BEEP_ON,
  RI_M_SPCLKERR1,
  RI_M_SPCLKERR2,
  RI_ADC1L_VOL,
  RI_ADC1R_VOL,
  RI_ADC2L_VOL,
  RI_ADC2R_VOL,
  RI_SMS_INTIM,
  RI_DNC2_AVF,
  RI_DNC2_MONION1,
  RI_DNC2_MONIEN1,
  RI_DNC2_MONION0,
  RI_DNC2_MONIEN0,
  RI_DNC1_AVF,
  RI_DNC1_MONION1,
  RI_DNC1_MONIEN1,
  RI_DNC1_MONION0,
  RI_DNC1_MONIEN0,
  RI_DNC2_CFMD,
  RI_DNC2_ESS,
  RI_DNC2_ZWR,
  RI_DNC2_MUTE,
  RI_DNC2_START,
  RI_DNC1_CFMD,
  RI_DNC1_ESS,
  RI_DNC1_ZWR,
  RI_DNC1_MUTE,
  RI_DNC1_START,
  RI_DNC_STB,
  RI_DCMFS_34,
  RI_DNC_512,
  RI_DCMFS,
  RI_DNC1_CANVOL1,
  RI_DNC1_CANVOL0,
  RI_DNC2_CANVOL1,
  RI_DNC2_CANVOL0,
  RI_DNC1_MONVOL1,
  RI_DNC1_MONVOL0,
  RI_DNC2_MONVOL1,
  RI_DNC2_MONVOL0,
  RI_DNC1_ALGAIN1,
  RI_DNC1_ALGAIN0,
  RI_DNC2_ALGAIN1,
  RI_DNC2_ALGAIN0,
  RI_DNC_PHD,
  RI_DNC1_LIMIYT,
  RI_DNC1_LMTON0,
  RI_DNC1_LIMITR,
  RI_DNC1_LIMITA,
  RI_DNC1_INSTMD,
  RI_DNC2_LIMIYT,
  RI_DNC2_LMTON0,
  RI_DNC2_LIMITR,
  RI_DNC2_LIMITA,
  RI_DNC2_INSTMD,
  RI_ANC_FALVL,
  RI_ANC_TST,
  RI_ANC_FATST,
  RI_ENVREG_RESET,
  RI_ANC_CHSEL,
  RI_ANC_TR,
  RI_ANC_TA,
  RI_ANC_SOUT,
  RI_ANC_FASPN,
  RI_ANC_ZWR,
  RI_ANC_MUTE,
  RI_ANC_START,
  RI_ANC_FASTART,
  RI_ANC_FAWTB,
  RI_ANC_FAWTA,
  RI_ANC_ENV1,
  RI_ANC_ENV0,
  RI_ANC_CURST,
  RI_ANC_FAST,
  RI_ANC_ENV2,
  RI_NS_AMMD,
  RI_BPGAIN,
  RI_BPSEL,
  RI_NSDI,
  RI_NSII,
  RI_BPON,
  RI_NSMS,
  RI_CHSEL,
  RI_NSADJON,
  RI_NSX2,
  RI_NSPMUTE,
  RI_NSDD,
  RI_OUT2DLY,
  RI_NSAD,
  RI_PWMMD,
  RI_NSAS,
  RI_NSADJ,
  RI_VCONT,
  RI_TEST_OUT,
  RI_TEST_OUT_SEL0,
  RI_TEST_OUT_SEL1,
  RI_TEST_IN,
  RI_S_RESET,
  RI_HALT_INHIBIT,
  RI_FSRDBGMD,
  RI_BEEP_TEST,
  RI_ARWPHSET,
  RI_DSPRAM4_CLR,
  RI_DSPRAM3_CLR,
  RI_DSPRAM2_CLR,
  RI_DSPRAM1_CLR,
  RI_ALC_DELAY,
  RI_ALC_ALG,
  RI_ARC_TIMER,
  RI_ARC_DLY,
  RI_SPC_AWEIGHT,
  RI_SPC_ALC_ATTACK,
  RI_SPC_ALC_RELEASE,
  RI_ALC_LPF,
  RI_SPC_ENERGY,
  RI_W_RSRV,
  RI_R_RSRV,
  RI_SER_MODE,
  RI_PDM_OUT_EN,
  RI_FS_CLK_EN,
  RI_SEL_OUT4_R,
  RI_SEL_OUT4_L,
  RI_SEL_OUT3_R,
  RI_SEL_OUT3_L,
  RI_SEL_OUT2_R,
  RI_SEL_OUT2_L,
  RI_SEL_OUT1_R,
  RI_SEL_OUT1_L,
  RI_OUTEN_MIC1L_B,
  RI_OUTEN_MIC1R_B,
  RI_OUTEN_MIC2L_B,
  RI_OUTEN_MIC2R_B,
  RI_OUTEN_MIC1L_A,
  RI_OUTEN_MIC1R_A,
  RI_OUTEN_MIC2L_A,
  RI_OUTEN_MIC2R_A,
  RI_SEL_OUTF,
  RI_SEL_INF,
  RI_SEL_DECIM,
  RI_DEQ_COEF_1B0,
  RI_DEQ_EN,
  RI_DEQ_COEF_1B1,
  RI_DEQ_COEF_1B2,
  RI_DEQ_COEF_1A1,
  RI_DEQ_COEF_1A2,
  RI_DEQ_COEF_2B0,
  RI_DEQ_COEF_2B1,
  RI_DEQ_COEF_2B2,
  RI_DEQ_COEF_2A1,
  RI_DEQ_COEF_2A2,
  RI_DEQ_COEF_3B0,
  RI_DEQ_COEF_3B1,
  RI_DEQ_COEF_3B2,
  RI_DEQ_COEF_3A1,
  RI_DEQ_COEF_3A2,
  RI_DEQ_COEF_4B0,
  RI_DEQ_COEF_4B1,
  RI_DEQ_COEF_4B2,
  RI_DEQ_COEF_4A1,
  RI_DEQ_COEF_4A2,
  RI_DEQ_COEF_5B0,
  RI_DEQ_COEF_5B1,
  RI_DEQ_COEF_5B2,
  RI_DEQ_COEF_5A1,
  RI_DEQ_COEF_5A2,
  RI_DEQ_COEF_6B0,
  RI_DEQ_COEF_6B1,
  RI_DEQ_COEF_6B2,
  RI_DEQ_COEF_6A1,
  RI_DEQ_COEF_6A2,
  RI_LR_SWAP1,
  RI_LR_SWAP2,
  RI_DUMMY_0,
  RI_DUMMY_1,
  RI_DUMMY_2,
  RI_DUMMY_3,
  RI_DUMMY_4,
  RI_DUMMY_5,
  RI_DUMMY_6,
  RI_DUMMY_7,
  RI_DUMMY_8,
  RI_DUMMY_9,
  RI_DUMMY_10,
  RI_DUMMY_11,
  RI_DUMMY_12,
  RI_DUMMY_13,
  RI_DUMMY_14,
  RI_DUMMY_15,
  RI_DUMMY_16,
  RI_DUMMY_17,
  RI_DUMMY_18,
  RI_DUMMY_19,
  RI_DUMMY_20,
  RI_DUMMY_21,
  RI_DUMMY_22,
  RI_DUMMY_23,
  RI_DUMMY_24,
  RI_DUMMY_25,
  RI_DUMMY_26,
  RI_DUMMY_27,
  RI_DUMMY_28,
  RI_DUMMY_29,
  RI_DUMMY_30,
  RI_DUMMY_31,
  RI_RAM_RW_EN,
  RI_REG_MAX_ENTRY
};

typedef enum audio_codec_register_id_e AC_REG_ID;

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cxd56_audio_ac_reg_seloutch_s
{
  uint8_t ch[CXD56_AUDIO_MIC_CH_MAX];
};

typedef struct cxd56_audio_ac_reg_seloutch_s cxd56_audio_ac_reg_seloutch_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_checkid(void);
void cxd56_audio_ac_reg_resetdsp(void);
void cxd56_audio_ac_reg_initdsp(void);
void cxd56_audio_ac_reg_poweron_sdes(void);
CXD56_AUDIO_ECODE cxd56_audio_ac_reg_set_micmode(uint8_t mic_mode);
void cxd56_audio_ac_reg_poweron_codec(void);
void cxd56_audio_ac_reg_poweroff_codec(void);
void cxd56_audio_ac_reg_enable_serialif(void);
void cxd56_audio_ac_reg_init_selector(void);
CXD56_AUDIO_ECODE cxd56_audio_ac_reg_set_alcspc(void);
void cxd56_audio_ac_reg_poweron_dnc(void);
void cxd56_audio_ac_reg_poweroff_dnc(void);
void cxd56_audio_ac_reg_disable_dnc(cxd56_audio_dnc_id_t id);
void cxd56_audio_ac_reg_enable_dnc(cxd56_audio_dnc_id_t id);
void cxd56_audio_ac_reg_mute_dnc(cxd56_audio_dnc_id_t id);
void cxd56_audio_ac_reg_unmute_dnc(cxd56_audio_dnc_id_t id);
void cxd56_audio_ac_reg_set_dncram(cxd56_audio_dnc_id_t id,
                                   cxd56_audio_dnc_bin_t *bin);
void cxd56_audio_ac_reg_enable_deq(void);
void cxd56_audio_ac_reg_disable_deq(void);
void cxd56_audio_ac_reg_set_deq_param(cxd56_audio_deq_coef_t *deq);
CXD56_AUDIO_ECODE cxd56_audio_ac_reg_poweron_cic(
                                uint8_t mic_in,
                                uint8_t mic_mode,
                                uint8_t cic_num,
                                cxd56_audio_mic_gain_t *gain);
void cxd56_audio_ac_reg_poweroff_cic(void);
CXD56_AUDIO_ECODE cxd56_audio_ac_reg_poweron_decim(uint8_t mic_mode,
                                                   uint8_t clk_mode);
void cxd56_audio_ac_reg_poweroff_decim(void);
void cxd56_audio_ac_reg_poweron_smaster(uint8_t clk_mode);
void cxd56_audio_ac_reg_poweroff_smaster(void);
void cxd56_audio_ac_reg_enable_smaster(void);
void cxd56_audio_ac_reg_disable_smaster(void);
void cxd56_audio_ac_reg_enable_beep(void);
void cxd56_audio_ac_reg_disable_beep(void);
void cxd56_audio_ac_reg_set_beep_freq(uint32_t freq);
void cxd56_audio_ac_reg_set_beep_vol(uint32_t vol);
void cxd56_audio_ac_reg_set_cicgain(uint8_t cic_num,
                                    cxd56_audio_mic_gain_t *gain);
void cxd56_audio_ac_reg_enable_digsft(void);
void cxd56_audio_ac_reg_disable_digsft(void);
void cxd56_audio_ac_reg_set_dsrrate(uint32_t rate);
void cxd56_audio_ac_reg_set_seloutch(
                    cxd56_audio_ac_reg_seloutch_t *seloutch);
void cxd56_audio_ac_reg_enable_dma(void);

void cxd56_audio_ac_reg_poweron_i2s(uint8_t clk_mode);
void cxd56_audio_ac_reg_enable_i2s_src1(void);
void cxd56_audio_ac_reg_enable_i2s_src2(void);
void cxd56_audio_ac_reg_enable_i2s_bcklrckout(void);
void cxd56_audio_ac_reg_disable_i2s_bcklrckout(void);

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_set_selector(
                                     cxd56_audio_signal_t sig,
                                     cxd56_audio_sel_t sel);

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_enable_cstereo(bool sign_inv,
                                                    int16_t vol);
void cxd56_audio_ac_reg_disable_cstereo(void);
void cxd56_audio_ac_reg_set_vol_sdin1(uint32_t vol);
void cxd56_audio_ac_reg_set_vol_sdin2(uint32_t vol);
void cxd56_audio_ac_reg_set_vol_dac(uint32_t vol);

#endif /* __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_AC_REG_H */
