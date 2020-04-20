/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_ac_reg.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <sys/types.h>

#include <math.h>

#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_ac_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AC_REG_BASE  (0x0c000000 + 0x02300000)

#define AC_REVID     0x20
#define AC_DEVICEID  0x02

#define DNC1_IRAM_BASE  0x3000
#define DNC1_CRAM_BASE  0x3800
#define DNC2_IRAM_BASE  0x3c00
#define DNC2_CRAM_BASE  0x4400

#define MTBR_GAIN_MAX        0
#define MTBR_GAIN_MIN        -630
#define MTBR_GAIN_MUTE       -635

#define CIC_GAIN_MAX         0
#define CIC_GAIN_MIN         -7850
#define CIC_GAIN_MUTE        -7855

#define ALC_TARGET_MIN       -63
#define ALC_TARGET_MAX       0

#define ALC_KNEE_MIN         -635
#define ALC_KNEE_MAX         0

#define SPC_LIMIT_MIN        -250
#define SPC_LIMIT_MAX        0

#define DEQ_BAND_NUM         6
#define DEQ_COEF_NUM         5
#define DNC_IRAM_SIZE        (256 * 2)
#define DNC_CRAM_SIZE        128

#define AC_CIC_NUM           4
#define AC_CIC_MIC_CH_NUM    2

#define AC_REG_MAX_BIT 32

#define I2S_DATA_RATE_LOW   48000
#define I2S_DATA_RATE_MID   96000
#define I2S_DATA_RATE_HIGH  192000

#define CS_VOL_MAX           -195
#define CS_VOL_MIN           -825
#define CS_VOL_OFFSET         190

#define AU_DAT_SEL_MIC1      0
#define AU_DAT_SEL_MIC2      1
#define AU_DAT_SEL_MIC3      2
#define AU_DAT_SEL_MIC4      3
#define AU_DAT_SEL_BUSIF1    4
#define AU_DAT_SEL_BUSIF2    4

#define COD_INSEL_SRC1        0
#define COD_INSEL_SRC2        1
#define COD_INSEL_AU_DAT_SEL1 2
#define COD_INSEL_AU_DAT_SEL2 3

#define SRCIN_SEL_AU_DAT_SEL1  0
#define SRCIN_SEL_AU_DAT_SEL2  1
#define SRCIN_SEL_CODECDSP_MIX 3

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct audio_ac_reg_s
{
  uint32_t addr;
  uint32_t pos;
  uint32_t len;
  uint32_t init;
};

const struct audio_ac_reg_s g_ac_reg[RI_REG_MAX_ENTRY] =
{
  {0x0000, 16,  8, 0x00000020},  /* REVID (0x10) */
  {0x0000, 24,  8, 0x00000002},  /* DEVICEID (0x01) */
  {0x0100,  4,  1, 0x00000001},  /* PDN_AMICEXT (0x01) */
  {0x0100,  5,  1, 0x00000001},  /* PDN_AMIC1 (0x01) */
  {0x0100,  6,  1, 0x00000001},  /* PDN_AMIC2 (0x01) */
  {0x0100,  8,  1, 0x00000001},  /* PDN_DAC (0x01) */
  {0x0100,  9,  1, 0x00000001},  /* PDN_LINEIN (0x01) */
  {0x0100, 10,  1, 0x00000001},  /* PDN_MIC (0x01) */
  {0x0100, 15,  1, 0x00000001},  /* PDN_DMIC (0x01) */
  {0x0100, 16,  1, 0x00000001},  /* PDN_DSPB (0x01) */
  {0x0100, 17,  1, 0x00000001},  /* PDN_ANC (0x01) */
  {0x0100, 18,  1, 0x00000001},  /* PDN_DNC1 (0x01) */
  {0x0100, 19,  1, 0x00000001},  /* PDN_DNC2 (0x01) */
  {0x0100, 20,  1, 0x00000001},  /* PDN_SMSTR (0x01) */
  {0x0100, 21,  1, 0x00000001},  /* PDN_DSPS2 (0x01) */
  {0x0100, 22,  1, 0x00000001},  /* PDN_DSPS1 (0x01) */
  {0x0100, 23,  1, 0x00000001},  /* PDN_DSPC (0x01) */
  {0x0104,  0,  1, 0x00000000},  /* FS_FS (0x00) */
  {0x0104, 16,  1, 0x00000000},  /* DECIM0_EN (0x00) */
  {0x0104, 17,  1, 0x00000000},  /* DECIM1_EN (0x00) */
  {0x0104, 18,  1, 0x00000000},  /* SDES_EN (0x00) */
  {0x0104, 19,  1, 0x00000000},  /* MCK_AHBMSTR_EN (0x00) */
  {0x0108, 16,  3, 0x00000000},  /* AU_DAT_SEL2 (0x00) */
  {0x0108, 20,  3, 0x00000000},  /* AU_DAT_SEL1 (0x00) */
  {0x0108, 24,  2, 0x00000000},  /* COD_INSEL3 (0x00) */
  {0x0108, 26,  2, 0x00000000},  /* COD_INSEL2 (0x00) */
  {0x0108, 28,  2, 0x00000000},  /* COD_INSEL1 (0x00) */
  {0x0200,  0,  3, 0x00000000},  /* DSR_RATE (0x00) */
  {0x0200, 12,  1, 0x00000001},  /* DIGSFT (0x01) */
  {0x0200, 16,  2, 0x00000001},  /* SRC1 (0x01) */
  {0x0200, 18,  2, 0x00000002},  /* SRC1IN_SEL (0x02) */
  {0x0200, 20,  2, 0x00000001},  /* SRC2 (0x01) */
  {0x0200, 22,  2, 0x00000002},  /* SRC2IN_SEL (0x02) */
  {0x0200, 26,  1, 0x00000001},  /* DIF2 (0x01) */
  {0x0200, 27,  1, 0x00000001},  /* DIF1 (0x01) */
  {0x0200, 28,  1, 0x00000000},  /* SD2MASTER (0x00) */
  {0x0200, 29,  1, 0x00000000},  /* SD1MASTER (0x00) */
  {0x0200, 30,  1, 0x00000000},  /* SDCK_OUTENX (0x00) */
  {0x0200, 31,  1, 0x00000000},  /* HI_RES_MODE (0x00) */
  {0x0204,  0,  2, 0x00000000},  /* HPF2_MODE (0x00) */
  {0x0204,  5,  1, 0x00000000},  /* CIC2IN_SWAP (0x00) */
  {0x0204,  6,  1, 0x00000000},  /* INV_DMIC2L (0x00) */
  {0x0204,  7,  1, 0x00000000},  /* INV_DMIC2R (0x00) */
  {0x0204,  8,  1, 0x00000000},  /* CIC2_GAIN_MODE (0x00) */
  {0x0204,  9,  1, 0x00000000},  /* CIC2IN_SEL (0x00) */
  {0x0204, 10,  1, 0x00000000},  /* ADC2_BOOST (0x00) */
  {0x0204, 14,  1, 0x00000000},  /* INV_AMIC2L (0x00) */
  {0x0204, 15,  1, 0x00000000},  /* INV_AMIC2R (0x00) */
  {0x0204, 16,  2, 0x00000000},  /* HPF1_MODE (0x00)  */
  {0x0204, 21,  1, 0x00000000},  /* CIC1IN_SWAP (0x00) */
  {0x0204, 22,  1, 0x00000000},  /* INV_DMIC1L (0x00) */
  {0x0204, 23,  1, 0x00000000},  /* INV_DMIC1R (0x00) */
  {0x0204, 24,  1, 0x00000000},  /* CIC1_GAIN_MODE (0x00) */
  {0x0204, 25,  1, 0x00000000},  /* CIC1IN_SEL (0x00) */
  {0x0204, 26,  1, 0x00000000},  /* ADC1_BOOST (0x00) */
  {0x0204, 28,  2, 0x00000000},  /* ADC_FS (0x00) */
  {0x0204, 30,  1, 0x00000000},  /* INV_AMIC1L (0x00) */
  {0x0204, 31,  1, 0x00000000},  /* INV_AMIC1R (0x00) */
  {0x0208,  0,  2, 0x00000000},  /* HPF4_MODE (0x00)  */
  {0x0208,  5,  1, 0x00000000},  /* CIC4IN_SWAP (0x00) */
  {0x0208,  6,  1, 0x00000000},  /* INV_DMIC4L (0x00) */
  {0x0208,  7,  1, 0x00000000},  /* INV_DMIC4R (0x00) */
  {0x0208,  9,  1, 0x00000000},  /* CIC4IN_SEL (0x00) */
  {0x0208, 10,  1, 0x00000000},  /* ADC4_BOOST (0x00) */
  {0x0208, 14,  1, 0x00000000},  /* INV_AMIC4L (0x00) */
  {0x0208, 15,  1, 0x00000000},  /* INV_AMIC4R (0x00) */
  {0x0208, 16,  2, 0x00000000},  /* HPF3_MODE (0x00) */
  {0x0208, 21,  1, 0x00000000},  /* CIC3IN_SWAP (0x00) */
  {0x0208, 22,  1, 0x00000000},  /* INV_DMIC3L (0x00) */
  {0x0208, 23,  1, 0x00000000},  /* INV_DMIC3R (0x00) */
  {0x0208, 25,  1, 0x00000000},  /* CIC3IN_SEL (0x00) */
  {0x0208, 26,  1, 0x00000000},  /* ADC_3_BOOST (0x00) */
  {0x0208, 30,  1, 0x00000000},  /* INV_AMIC3L (0x00) */
  {0x0208, 31,  1, 0x00000000},  /* INV_AMIC3R (0x00) */
  {0x020c,  0, 16, 0x00004000},  /* CIC1_RGAIN (0x4000) */
  {0x020c, 16, 16, 0x00004000},  /* CIC1_LGAIN (0x4000) */
  {0x0210,  0, 16, 0x00004000},  /* CIC2_RGAIN (0x4000) */
  {0x0210, 16, 16, 0x00004000},  /* CIC2_LGAIN (0x4000) */
  {0x0214,  0, 16, 0x00004000},  /* CIC3_RGAIN (0x4000) */
  {0x0214, 16, 16, 0x00004000},  /* CIC3_LGAIN (0x4000) */
  {0x0218,  0, 16, 0x00004000},  /* CIC4_RGAIN (0x4000) */
  {0x0218, 16, 16, 0x00004000},  /* CIC4_LGAIN (0x4000) */
  {0x021c,  8,  7, 0x00000000},  /* SPC_LIMIT (0x00) */
  {0x021c, 15,  1, 0x00000000},  /* SPC_EN (0x00) */
  {0x021c, 16,  7, 0x00000008},  /* ALC_KNEE (0x08) */
  {0x021c, 24,  6, 0x00000000},  /* ALCTARGET (0x00) */
  {0x021c, 30,  1, 0x00000001},  /* ALC_REC (0x01) */
  {0x021c, 31,  1, 0x00000000},  /* ALC_EN (0x00) */
  {0x0220, 16,  1, 0x00000000},  /* INV_ASP2R (0x00) */
  {0x0220, 17,  1, 0x00000000},  /* INV_ASP2L (0x00) */
  {0x0220, 18,  1, 0x00000000},  /* INV_ASP1R (0x00) */
  {0x0220, 19,  1, 0x00000000},  /* INV_ASP1L (0x00) */
  {0x0220, 29,  3, 0x00000004},  /* ARC (0x04) */
  {0x0224,  8, 24, 0x00800000},  /* ARC_VOL (0x800000) */
  {0x0228,  0,  7, 0x00000000},  /* CS_VOL (0x00) */
  {0x0228,  7,  1, 0x00000000},  /* CS_SIGN (0x00) */
  {0x0228,  8,  8, 0x00000033},  /* SDOUT_VOL (0x33) */
  {0x0228, 16,  8, 0x00000033},  /* SDIN2_VOL (0x33) */
  {0x0228, 24,  8, 0x00000033},  /* SDIN1_VOL (0x33) */
  {0x022c,  0,  1, 0x00000000},  /* SDIN1_EN (0x00) */
  {0x022c,  1,  1, 0x00000000},  /* SDIN2_EN (0x00) */
  {0x022c,  2,  1, 0x00000000},  /* SDOUT1_EN (0x00) */
  {0x022c,  3,  1, 0x00000000},  /* SDOUT2_EN (0x00) */
  {0x022c,  4,  1, 0x00000000},  /* MUTE_B (0x00) */
  {0x022c,  5,  1, 0x00000000},  /* BLF_EN (0x00) */
  {0x022c,  6,  1, 0x00000000},  /* TRANS_MODE (0x00) */
  {0x022c,  8,  8, 0x00000033},  /* DAC_VOL (0x33) */
  {0x022c, 16,  8, 0x00000033},  /* LINEIN_VOL (0x33) */
  {0x0230, 16,  5, 0x00000004},  /* BEEP_VOL (0x04) */
  {0x0230, 24,  6, 0x00000010},  /* BEEP_FREQ (0x10) */
  {0x0230, 31,  1, 0x00000000},  /* BEEP_ON (0x00) */
  {0x0234,  0,  1, 0x00000001},  /* M_SPCLKERR1 (0x01) */
  {0x0234,  1,  1, 0x00000001},  /* M_SPCLKERR2 (0x01) */
  {0x0240,  0,  7, 0x00000000},  /* ADC1L_VOL (0x00) */
  {0x0240,  8,  7, 0x00000000},  /* ADC1R_VOL (0x00) */
  {0x0240, 16,  7, 0x00000000},  /* ADC2L_VOL (0x00) */
  {0x0240, 24,  7, 0x00000000},  /* ADC2R_VOL (0x00) */
  {0x0300,  0,  8, 0x00000000},  /* SMS_INTIM (0x00) */
  {0x0304,  0,  3, 0x00000000},  /* DNC2_AVF (0x00)  */
  {0x0304,  4,  1, 0x00000000},  /* DNC2_MONION1 (0x00) */
  {0x0304,  5,  1, 0x00000000},  /* DNC2_MONIEN1 (0x00) */
  {0x0304,  6,  1, 0x00000000},  /* DNC2_MONION0 (0x00) */
  {0x0304,  7,  1, 0x00000000},  /* DNC2_MONIEN0 (0x00) */
  {0x0304,  8,  3, 0x00000000},  /* DNC1_AVF (0x00) */
  {0x0304, 12,  1, 0x00000000},  /* DNC1_MONION1 (0x00) */
  {0x0304, 13,  1, 0x00000000},  /* DNC1_MONIEN1 (0x00) */
  {0x0304, 14,  1, 0x00000000},  /* DNC1_MONION0 (0x00) */
  {0x0304, 15,  1, 0x00000000},  /* DNC1_MONIEN0 (0x00) */
  {0x0304, 16,  3, 0x00000000},  /* DNC2_CFMD (0x00) */
  {0x0304, 20,  1, 0x00000000},  /* DNC2_ESS (0x00) */
  {0x0304, 21,  1, 0x00000000},  /* DNC2_ZWR (0x00) */
  {0x0304, 22,  1, 0x00000001},  /* DNC2_MUTE (0x01) */
  {0x0304, 23,  1, 0x00000000},  /* DNC2_START (0x00) */
  {0x0304, 24,  3, 0x00000000},  /* DNC1_CFMD (0x00) */
  {0x0304, 28,  1, 0x00000000},  /* DNC1_ESS (0x00) */
  {0x0304, 29,  1, 0x00000000},  /* DNC1_ZWR (0x00) */
  {0x0304, 30,  1, 0x00000001},  /* DNC1_MUTE (0x01) */
  {0x0304, 31,  1, 0x00000000},  /* DNC1_START (0x00) */
  {0x0308, 16,  1, 0x00000000},  /* DNC_STB (0x00) */
  {0x0308, 22,  2, 0x00000001},  /* DCMFS_34 (0x01) */
  {0x0308, 29,  1, 0x00000000},  /* DNC_512 (0x00) */
  {0x0308, 30,  2, 0x00000001},  /* DCMFS (0x01) */
  {0x030c,  0, 16, 0x00000000},  /* DNC1_CANVOL1 (0x0000) */
  {0x030c, 16, 16, 0x00000000},  /* DNC1_CANVOL0 (0x0000) */
  {0x0310,  0, 16, 0x00000000},  /* DNC2_CANVOL1 (0x0000) */
  {0x0310, 16, 16, 0x00000000},  /* DNC2_CANVOL0 (0x0000) */
  {0x0314,  0, 16, 0x00000000},  /* DNC1_MONVOL1 (0x0000) */
  {0x0314, 16, 16, 0x00000000},  /* DNC1_MONVOL0 (0x0000) */
  {0x0318,  0, 16, 0x00000000},  /* DNC2_MONVOL1 (0x0000) */
  {0x0318, 16, 16, 0x00000000},  /* DNC2_MONVOL0 (0x0000) */
  {0x031c,  0, 16, 0x00000000},  /* DNC1_ALGAIN1 (0x0000) */
  {0x031c, 16, 16, 0x00000000},  /* DNC1_ALGAIN0 (0x0000) */
  {0x0320,  0, 16, 0x00000000},  /* DNC2_ALGAIN1 (0x0000) */
  {0x0320, 16, 16, 0x00000000},  /* DNC2_ALGAIN0 (0x0000) */
  {0x0324, 24,  8, 0x00000000},  /* DNC_PHD (0x00) */
  {0x0328,  8,  5, 0x0000001f},  /* DNC1_LIMIYT (0x1f) */
  {0x0328, 15,  1, 0x00000000},  /* DNC1_LMTON0 (0x00) */
  {0x0328, 16,  5, 0x00000000},  /* DNC1_LIMITR (0x00) */
  {0x0328, 24,  5, 0x00000000},  /* DNC1_LIMITA (0x00) */
  {0x0328, 30,  2, 0x00000000},  /* DNC1_INSTMD (0x00) */
  {0x032c,  8,  5, 0x0000001f},  /* DNC2_LIMIYT (0x1f) */
  {0x032c, 15,  1, 0x00000000},  /* DNC2_LMTON0 (0x00) */
  {0x032c, 16,  5, 0x00000000},  /* DNC2_LIMITR (0x00) */
  {0x032c, 24,  5, 0x00000000},  /* DNC2_LIMITA (0x00) */
  {0x032c, 30,  2, 0x00000000},  /* DNC2_INSTMD (0x00) */
  {0x0330,  0, 16, 0x00000100},  /* ANC_FALVL (0x0100) */
  {0x0330, 24,  1, 0x00000000},  /* ANC_TST (0x00) */
  {0x0330, 25,  1, 0x00000000},  /* ANC_FATST (0x00) */
  {0x0330, 26,  1, 0x00000000},  /* ENVREG_RESET (0x00) */
  {0x0330, 30,  2, 0x00000000},  /* ANC_CHSEL (0x00) */
  {0x0334,  8,  2, 0x00000000},  /* ANC_TR (0x00) */
  {0x0334, 10,  2, 0x00000000},  /* ANC_TA (0x00) */
  {0x0334, 16,  1, 0x00000000},  /* ANC_SOUT (0x00) */
  {0x0334, 17,  3, 0x00000003},  /* ANC_FASPN (0x03) */
  {0x0334, 20,  1, 0x00000000},  /* ANC_ZWR (0x00) */
  {0x0334, 21,  1, 0x00000001},  /* ANC_MUTE (0x01) */
  {0x0334, 22,  1, 0x00000000},  /* ANC_START (0x00) */
  {0x0334, 23,  1, 0x00000000},  /* ANC_FASTART (0x00) */
  {0x0334, 24,  3, 0x00000000},  /* ANC_FAWTB (0x00) */
  {0x0334, 28,  3, 0x00000000},  /* ANC_FAWTA (0x00) */
  {0x0338,  0, 16, 0x00000000},  /* ANC_ENV1 (0x0000) */
  {0x0338, 16, 16, 0x00000000},  /* ANC_ENV0 (0x0000) */
  {0x033c,  8,  4, 0x00000000},  /* ANC_CURST (0x00) */
  {0x033c, 12,  2, 0x00000000},  /* ANC_FAST (0x00) */
  {0x033c, 16, 16, 0x00000000},  /* ANC_ENV2 (0x0000) */
  {0x0340,  0,  4, 0x00000000},  /* NS_AMMD (0x00) */
  {0x0340,  8,  8, 0x000000ff},  /* BPGAIN (0xff) */
  {0x0340, 16,  4, 0x00000000},  /* BPSEL (0x00) */
  {0x0340, 21,  1, 0x00000000},  /* NSDI (0x00) */
  {0x0340, 22,  1, 0x00000000},  /* NSII (0x00) */
  {0x0340, 23,  1, 0x00000000},  /* BPON (0x00) */
  {0x0340, 24,  3, 0x00000002},  /* NSMS (0x02) */
  {0x0340, 27,  1, 0x00000000},  /* CHSEL (0x00) */
  {0x0340, 29,  1, 0x00000000},  /* NSADJON (0x00) */
  {0x0340, 30,  1, 0x00000000},  /* NSX2 (0x00) */
  {0x0340, 31,  1, 0x00000001},  /* NSPMUTE (0x01) */
  {0x0344,  0, 20, 0x00000000},  /* NSDD (0x000000) */
  {0x0344, 20,  4, 0x00000000},  /* OUT2DLY (0x00) */
  {0x0348,  0, 20, 0x00000000},  /* NSAD (0x000000) */
  {0x0348, 20,  2, 0x00000001},  /* PWMMD (0x01) */
  {0x0348, 22,  2, 0x00000000},  /* NSAS (0x00) */
  {0x034c, 16,  8, 0x00000000},  /* NSADJ (0x00) */
  {0x034c, 24,  8, 0x00000000},  /* VCONT (0x00) */
  {0x0400,  0,  6, 0x00000000},  /* TEST_OUT (0x00) */
  {0x0400,  6,  1, 0x00000000},  /* RI_TEST_OUT_SEL0 (0x00) */
  {0x0400,  7,  1, 0x00000000},  /* RI_TEST_OUT_SEL1 (0x00) */
  {0x0400,  8,  8, 0x00000000},  /* TEST_IN (0x00) */
  {0x0400, 16,  1, 0x00000000},  /* S_RESET (0x00) */
  {0x0400, 19,  1, 0x00000001},  /* HALT_INHIBIT (0x01) */
  {0x0400, 21,  1, 0x00000000},  /* FSRDBGMD (0x00) */
  {0x0400, 22,  1, 0x00000000},  /* BEEP_TEST (0x00) */
  {0x0400, 23,  1, 0x00000000},  /* ARWPHSET (0x00) */
  {0x0400, 28,  1, 0x00000000},  /* DSPRAM4_CLR (0x00) */
  {0x0400, 29,  1, 0x00000000},  /* DSPRAM3_CLR (0x00) */
  {0x0400, 30,  1, 0x00000000},  /* DSPRAM2_CLR (0x00) */
  {0x0400, 31,  1, 0x00000000},  /* DSPRAM1_CLR (0x00) */
  {0x0404,  8,  7, 0x00000010},  /* ALC_DELAY (0x10) */
  {0x0404, 15,  1, 0x00000000},  /* ALC_ALG (0x00) */
  {0x0404, 16,  4, 0x00000000},  /* ARC_TIMER (0x00) */
  {0x0404, 20,  2, 0x00000002},  /* ARC_DLY (0x02) */
  {0x0404, 23,  1, 0x00000001},  /* SPC_AWEIGHT (0x01) */
  {0x0408,  0, 24, 0x00400000},  /* SPC_ALC_ATTACK (0x400000) */
  {0x040c,  0, 24, 0x00000100},  /* SPC_ALC_RELEASE (0x000100) */
  {0x0410,  0, 24, 0x000141b3},  /* ALC_LPF (0x0141b3) */
  {0x0414,  0, 24, 0x00000000},  /* SPC_ENERGY (0x000000) */
  {0x0418, 16, 16, 0x00000000},  /* W_RSRV (0x0000) */
  {0x041c,  0,  8, 0x00000000},  /* R_RSRV (0x00) */
  {0x0500,  0,  1, 0x00000000},  /* SER_MODE (0x00) */
  {0x0500, 16,  1, 0x00000000},  /* PDM_OUT_EN (0x00) */
  {0x0500, 24,  1, 0x00000000},  /* FS_CLK_EN (0x00) */
  {0x0504,  0,  3, 0x00000007},  /* SEL_OUT4_R (0x07) */
  {0x0504,  4,  3, 0x00000006},  /* SEL_OUT4_L (0x06) */
  {0x0504,  8,  3, 0x00000005},  /* SEL_OUT3_R (0x05) */
  {0x0504, 12,  3, 0x00000004},  /* SEL_OUT3_L (0x04) */
  {0x0504, 16,  3, 0x00000003},  /* SEL_OUT2_R (0x03) */
  {0x0504, 20,  3, 0x00000002},  /* SEL_OUT2_L (0x02) */
  {0x0504, 24,  3, 0x00000001},  /* SEL_OUT1_R (0x01) */
  {0x0504, 28,  3, 0x00000000},  /* SEL_OUT1_L (0x00) */
  {0x0580,  0,  1, 0x00000000},  /* OUTEN_MIC1L_B (0x00) */
  {0x0580,  1,  1, 0x00000000},  /* OUTEN_MIC1R_B (0x00) */
  {0x0580,  2,  1, 0x00000000},  /* OUTEN_MIC2L_B (0x00) */
  {0x0580,  3,  1, 0x00000000},  /* OUTEN_MIC2R_B (0x00) */
  {0x0580,  4,  1, 0x00000000},  /* OUTEN_MIC1L_A (0x00) */
  {0x0580,  5,  1, 0x00000000},  /* OUTEN_MIC1R_A (0x00) */
  {0x0580,  6,  1, 0x00000000},  /* OUTEN_MIC2L_A (0x00) */
  {0x0580,  7,  1, 0x00000000},  /* OUTEN_MIC2R_A (0x00) */
  {0x0580, 16,  2, 0x00000000},  /* SEL_OUTF (0x00) */
  {0x0580, 20,  1, 0x00000000},  /* SEL_INF (0x00) */
  {0x0580, 24,  1, 0x00000000},  /* SEL_DECIM (0x00) */
  {0x0600,  0, 24, 0x00200000},  /* DEQ_COEF_1B0 (0x200000) */
  {0x0600, 31,  1, 0x00000000},  /* DEQ_EN (0x00) */
  {0x0604,  0, 24, 0x00000000},  /* DEQ_COEF_1B1 (0x000000) */
  {0x0608,  0, 24, 0x00000000},  /* DEQ_COEF_1B2 (0x000000) */
  {0x060c,  0, 24, 0x00000000},  /* DEQ_COEF_1A1 (0x000000) */
  {0x0610,  0, 24, 0x00000000},  /* DEQ_COEF_1A2 (0x000000) */
  {0x0614,  0, 24, 0x00200000},  /* DEQ_COEF_2B0 (0x200000) */
  {0x0618,  0, 24, 0x00000000},  /* DEQ_COEF_2B1 (0x000000) */
  {0x061c,  0, 24, 0x00000000},  /* DEQ_COEF_2B2 (0x000000) */
  {0x0620,  0, 24, 0x00000000},  /* DEQ_COEF_2A1 (0x000000) */
  {0x0624,  0, 24, 0x00000000},  /* DEQ_COEF_2A2 (0x000000) */
  {0x0628,  0, 24, 0x00200000},  /* DEQ_COEF_3B0 (0x200000) */
  {0x062c,  0, 24, 0x00000000},  /* DEQ_COEF_3B1 (0x000000) */
  {0x0630,  0, 24, 0x00000000},  /* DEQ_COEF_3B2 (0x000000) */
  {0x0634,  0, 24, 0x00000000},  /* DEQ_COEF_3A1 (0x000000) */
  {0x0638,  0, 24, 0x00000000},  /* DEQ_COEF_3A2 (0x000000) */
  {0x063c,  0, 24, 0x00200000},  /* DEQ_COEF_4B0 (0x200000) */
  {0x0640,  0, 24, 0x00000000},  /* DEQ_COEF_4B1 (0x000000) */
  {0x0644,  0, 24, 0x00000000},  /* DEQ_COEF_4B2 (0x000000) */
  {0x0648,  0, 24, 0x00000000},  /* DEQ_COEF_4A1 (0x000000) */
  {0x064c,  0, 24, 0x00000000},  /* DEQ_COEF_4A2 (0x000000) */
  {0x0650,  0, 24, 0x00200000},  /* DEQ_COEF_5B0 (0x200000) */
  {0x0654,  0, 24, 0x00000000},  /* DEQ_COEF_5B1 (0x000000) */
  {0x0658,  0, 24, 0x00000000},  /* DEQ_COEF_5B2 (0x000000) */
  {0x065c,  0, 24, 0x00000000},  /* DEQ_COEF_5A1 (0x000000) */
  {0x0660,  0, 24, 0x00000000},  /* DEQ_COEF_5A2 (0x000000) */
  {0x0664,  0, 24, 0x00200000},  /* DEQ_COEF_6B0 (0x200000) */
  {0x0668,  0, 24, 0x00000000},  /* DEQ_COEF_6B1 (0x000000) */
  {0x066c,  0, 24, 0x00000000},  /* DEQ_COEF_6B2 (0x000000) */
  {0x0670,  0, 24, 0x00000000},  /* DEQ_COEF_6A1 (0x000000) */
  {0x0674,  0, 24, 0x00000000},  /* DEQ_COEF_6A2 (0x000000) */
  {0x0678,  0,  1, 0x00000000},  /* LR_SWAP1 (0x000000) */
  {0x0678,  1,  1, 0x00000000},  /* LR_SWAP2 (0x000000) */
  {0x0700,  0,  1, 0x00000000},  /* DUMMY_0 (0x00) */
  {0x0700,  1,  1, 0x00000000},  /* DUMMY_1 (0x00) */
  {0x0700,  2,  1, 0x00000000},  /* DUMMY_2 (0x00) */
  {0x0700,  3,  1, 0x00000000},  /* DUMMY_3 (0x00) */
  {0x0700,  4,  1, 0x00000000},  /* DUMMY_4 (0x00) */
  {0x0700,  5,  1, 0x00000000},  /* DUMMY_5 (0x00) */
  {0x0700,  6,  1, 0x00000000},  /* DUMMY_6 (0x00) */
  {0x0700,  7,  1, 0x00000000},  /* DUMMY_7 (0x00) */
  {0x0700,  8,  1, 0x00000000},  /* DUMMY_8 (0x00) */
  {0x0700,  9,  1, 0x00000000},  /* DUMMY_9 (0x00) */
  {0x0700, 10,  1, 0x00000000},  /* DUMMY_10 (0x00) */
  {0x0700, 11,  1, 0x00000000},  /* DUMMY_11 (0x00) */
  {0x0700, 12,  1, 0x00000000},  /* DUMMY_12 (0x00) */
  {0x0700, 13,  1, 0x00000000},  /* DUMMY_13 (0x00) */
  {0x0700, 14,  1, 0x00000000},  /* DUMMY_14 (0x00) */
  {0x0700, 15,  1, 0x00000000},  /* DUMMY_15 (0x00) */
  {0x0700, 16,  1, 0x00000000},  /* DUMMY_16 (0x00) */
  {0x0700, 17,  1, 0x00000000},  /* DUMMY_17 (0x00) */
  {0x0700, 18,  1, 0x00000000},  /* DUMMY_18 (0x00) */
  {0x0700, 19,  1, 0x00000000},  /* DUMMY_19 (0x00) */
  {0x0700, 20,  1, 0x00000000},  /* DUMMY_20 (0x00) */
  {0x0700, 21,  1, 0x00000000},  /* DUMMY_21 (0x00) */
  {0x0700, 22,  1, 0x00000000},  /* DUMMY_22 (0x00) */
  {0x0700, 23,  1, 0x00000000},  /* DUMMY_23 (0x00) */
  {0x0700, 24,  1, 0x00000000},  /* DUMMY_24 (0x00) */
  {0x0700, 25,  1, 0x00000000},  /* DUMMY_25 (0x00) */
  {0x0700, 26,  1, 0x00000000},  /* DUMMY_26 (0x00) */
  {0x0700, 27,  1, 0x00000000},  /* DUMMY_27 (0x00) */
  {0x0700, 28,  1, 0x00000000},  /* DUMMY_28 (0x00) */
  {0x0700, 29,  1, 0x00000000},  /* DUMMY_29 (0x00) */
  {0x0700, 30,  1, 0x00000000},  /* DUMMY_30 (0x00) */
  {0x0700, 31,  1, 0x00000000},  /* DUMMY_31 (0x00) */
  {0x0780,  0,  1, 0x00000000},  /* RAM_RW_EN (0x00) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

uint32_t write_ac_reg(AC_REG_ID reg_id, uint32_t data)
{
  volatile uint32_t *addr;
  uint32_t mask = (g_ac_reg[reg_id].len == AC_REG_MAX_BIT) ?
                    0xffffffff : (1 << g_ac_reg[reg_id].len) - 1;
  uint32_t curr;

  addr = (volatile uint32_t *)(AC_REG_BASE + g_ac_reg[reg_id].addr);

  curr = *addr & ~(mask << g_ac_reg[reg_id].pos);
  *addr = curr | ((data & mask) << g_ac_reg[reg_id].pos);

  return 0;
}

uint32_t read_ac_reg(AC_REG_ID reg_id)
{
  volatile uint32_t *addr;
  uint32_t mask = (g_ac_reg[reg_id].len == AC_REG_MAX_BIT) ?
                    0xffffffff : (1 << g_ac_reg[reg_id].len) - 1;
  uint32_t data = 0;

  addr = (volatile uint32_t *)(AC_REG_BASE + g_ac_reg[reg_id].addr);
  data = (*addr >> g_ac_reg[reg_id].pos) & mask;

  return data;
}

uint32_t write32_ac_reg(uint32_t offset, uint32_t data)
{
  volatile uint32_t *addr;

  addr = (volatile uint32_t *)(AC_REG_BASE + offset);
  *addr = data;

  return 0;
}

uint32_t read32_ac_reg(uint32_t offset)
{
  volatile uint32_t *addr;
  uint32_t data = 0;

  addr = (volatile uint32_t *)(AC_REG_BASE + offset);
  data = *addr;

  return data;
}

CXD56_AUDIO_ECODE set_alc_param(void)
{
  /* Set Alc mode play */

  write_ac_reg(RI_ALC_REC, 0);

  /* Set Alc target. */

  if (CXD56_AUDIO_CFG_ALC_TARGET < ALC_TARGET_MIN ||
      CXD56_AUDIO_CFG_ALC_TARGET > ALC_TARGET_MAX)
    {
      return CXD56_AUDIO_ECODE_REG_AC_ALCTGT;
    }

  write_ac_reg(RI_ALCTARGET, -(CXD56_AUDIO_CFG_ALC_TARGET));

  /* Set Alc knee point. */

  if (CXD56_AUDIO_CFG_ALC_KNEE < ALC_KNEE_MIN ||
      CXD56_AUDIO_CFG_ALC_KNEE > ALC_KNEE_MAX)
    {
      return CXD56_AUDIO_ECODE_REG_AC_ALCKNEE;
    }

  write_ac_reg(RI_ALC_KNEE, -(CXD56_AUDIO_CFG_ALC_KNEE) / 5);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE set_spc_param(void)
{
  /* Set Alc target. */

  if (CXD56_AUDIO_CFG_SPC_LIMIT < SPC_LIMIT_MIN ||
      CXD56_AUDIO_CFG_SPC_LIMIT > SPC_LIMIT_MAX)
    {
      return CXD56_AUDIO_ECODE_REG_AC_SPCLIMT;
    }

  write_ac_reg(RI_SPC_LIMIT, -(CXD56_AUDIO_CFG_SPC_LIMIT) / 5);

  return CXD56_AUDIO_ECODE_OK;
}

void write_dnc_ram(uint32_t offset, uint32_t *data, uint32_t size)
{
  uint32_t i;

  for (i = 0; i < size; i++)
    {
      write32_ac_reg(offset + (i * 4), *(data + i));
    }
}

void set_deq_coef(uint32_t reg_id, cxd56_audio_deq_coef_func_t *coef)
{
  uint32_t reg_addr = g_ac_reg[reg_id].addr;

  write32_ac_reg(reg_addr,      coef->b0);
  write32_ac_reg(reg_addr +  4, coef->b1);
  write32_ac_reg(reg_addr +  8, coef->b2);
  write32_ac_reg(reg_addr + 12, coef->a0);
  write32_ac_reg(reg_addr + 16, coef->a1);
}

static void set_cic_gain(uint8_t cic_num,
                         FAR cxd56_audio_mic_gain_t *gain)
{
  uint32_t val;
  uint8_t  write_num;
  uint8_t i;

  const uint32_t cic_gain[CXD56_AUDIO_MIC_CH_MAX] =
    {
      RI_CIC1_LGAIN,
      RI_CIC1_RGAIN,
      RI_CIC2_LGAIN,
      RI_CIC2_RGAIN,
      RI_CIC3_LGAIN,
      RI_CIC3_RGAIN,
      RI_CIC4_LGAIN,
      RI_CIC4_RGAIN
    };

  write_num = (CXD56_AUDIO_MIC_CH_MAX >= (cic_num * 2)) ?
                (cic_num * 2) : CXD56_AUDIO_MIC_CH_MAX;

  for (i = 0; i < write_num; i++)
    {
      val = (uint32_t)(pow(10.0f,
                           ((float)gain->gain[i] /
                            100.0f / 20.0f)) * 0x4000 +
                            0x4000);

      write_ac_reg(cic_gain[i], val);
    }
}

static uint32_t get_data_rate(uint32_t rate)
{
  uint32_t data_rate;

  if (rate <= I2S_DATA_RATE_LOW)
    {
      data_rate = 1;
    }
  else if (rate <= I2S_DATA_RATE_MID)
    {
      data_rate = 2;
    }
  else
    {
      data_rate = 3;
    }

  return data_rate;
}

#ifdef CONFIG_CXD56_I2S0
static void poweron_i2s0(void)
{
  /* Power on SRC1. */

  write_ac_reg(RI_PDN_DSPS1, 0);

  /* Set I2S mode of SRC1. */

  uint32_t is_master =
    (CXD56_AUDIO_CFG_I2S1_MODE ==
     CXD56_AUDIO_CFG_I2S_MODE_MASTER) ? 1 : 0;

  write_ac_reg(RI_SD1MASTER, is_master);

  /* Set I2S format of SRC1. */

  uint32_t is_left =
    (CXD56_AUDIO_CFG_I2S1_FORMAT ==
     CXD56_AUDIO_CFG_I2S_FORMAT_LEFT) ? 1 : 0;

  write_ac_reg(RI_DIF1, is_left);
  write_ac_reg(RI_LR_SWAP1, is_left);

  /* Set data rate of SRC1. */

  uint32_t data_rate =
    get_data_rate(CXD56_AUDIO_CFG_I2S1_DATA_RATE);
  write_ac_reg(RI_SRC1, data_rate);

  /* Set bypass mode of SRC1. */

  uint32_t is_bypass =
    (CXD56_AUDIO_CFG_I2S1_BYPASS ==
     CXD56_AUDIO_CFG_I2S_BYPASS_ENABLE) ? 1 : 0;

  write_ac_reg(RI_TEST_OUT_SEL0, is_bypass);
}
#endif /* #ifdef CONFIG_CXD56_I2S0 */

#ifdef CONFIG_CXD56_I2S1
static void poweron_i2s1(void)
{
  /* Power on SRC2. */

  write_ac_reg(RI_PDN_DSPS2, 0);

  /* Set I2S mode of SRC2. */

  uint32_t is_master =
    (CXD56_AUDIO_CFG_I2S2_MODE ==
     CXD56_AUDIO_CFG_I2S_MODE_MASTER) ? 1 : 0;
  write_ac_reg(RI_SD2MASTER, is_master);

  /* Set I2S format of SRC2. */

  uint32_t is_left =
    (CXD56_AUDIO_CFG_I2S2_FORMAT ==
     CXD56_AUDIO_CFG_I2S_FORMAT_LEFT) ? 1 : 0;
  write_ac_reg(RI_DIF2, is_left);
  write_ac_reg(RI_LR_SWAP2, is_left);

  /* Set data rate of SRC2. */

  uint32_t data_rate = get_data_rate(CXD56_AUDIO_CFG_I2S2_DATA_RATE);
  write_ac_reg(RI_SRC2, data_rate);

  /* Set bypass mode of SRC2. */

  uint32_t is_bypass =
    (CXD56_AUDIO_CFG_I2S2_BYPASS ==
     CXD56_AUDIO_CFG_I2S_BYPASS_ENABLE) ? 1 : 0;
  write_ac_reg(RI_TEST_OUT_SEL0, is_bypass);
}
#endif /* #ifdef CONFIG_CXD56_I2S1 */

static CXD56_AUDIO_ECODE set_au_dat_sel(AC_REG_ID ac_reg_id,
                                        cxd56_audio_signal_t sig)
{
  uint32_t val = 0;

  switch (sig)
    {
      case CXD56_AUDIO_SIG_MIC1:
        val = AU_DAT_SEL_MIC1;
        break;

      case CXD56_AUDIO_SIG_MIC2:
        val = AU_DAT_SEL_MIC2;
        break;

      case CXD56_AUDIO_SIG_MIC3:
        val = AU_DAT_SEL_MIC3;
        break;

      case CXD56_AUDIO_SIG_MIC4:
        val = AU_DAT_SEL_MIC4;
        break;

      case CXD56_AUDIO_SIG_BUSIF1:
        if (ac_reg_id == RI_AU_DAT_SEL1)
          {
            val = AU_DAT_SEL_BUSIF1;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      case CXD56_AUDIO_SIG_BUSIF2:
        if (ac_reg_id == RI_AU_DAT_SEL2)
          {
            val = AU_DAT_SEL_BUSIF2;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      default:
        return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
    }

  write_ac_reg(ac_reg_id, val);

  return CXD56_AUDIO_ECODE_OK;
}

static CXD56_AUDIO_ECODE set_cod_insel(AC_REG_ID ac_reg_id,
                                       cxd56_audio_signal_t sig,
                                       cxd56_audio_sel_t sel)
{
  uint32_t val = 0;

  switch (sig)
    {
      case CXD56_AUDIO_SIG_MIC1:
      case CXD56_AUDIO_SIG_MIC2:
      case CXD56_AUDIO_SIG_MIC3:
      case CXD56_AUDIO_SIG_MIC4:
        if (sel.au_dat_sel1)
          {
            val = COD_INSEL_AU_DAT_SEL1;
          }
        else if(sel.au_dat_sel2)
          {
            val = COD_INSEL_AU_DAT_SEL2;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      case CXD56_AUDIO_SIG_BUSIF1:
        if (sel.au_dat_sel1)
          {
            val = COD_INSEL_AU_DAT_SEL1;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      case CXD56_AUDIO_SIG_BUSIF2:
        if (sel.au_dat_sel2)
          {
            val = COD_INSEL_AU_DAT_SEL2;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      case CXD56_AUDIO_SIG_I2S0:
        val = COD_INSEL_SRC1;
        break;

      case CXD56_AUDIO_SIG_I2S1:
        val = COD_INSEL_SRC2;
        break;

      default:
        return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
    }

  write_ac_reg(ac_reg_id, val);

  return CXD56_AUDIO_ECODE_OK;
}

static CXD56_AUDIO_ECODE set_srcin_sel(AC_REG_ID ac_reg_id,
                                       cxd56_audio_signal_t sig,
                                       cxd56_audio_sel_t sel)
{
  uint32_t val = 0;

  switch (sig)
    {
      case CXD56_AUDIO_SIG_MIC1:
      case CXD56_AUDIO_SIG_MIC2:
      case CXD56_AUDIO_SIG_MIC3:
      case CXD56_AUDIO_SIG_MIC4:
        if (sel.au_dat_sel1)
          {
            val = SRCIN_SEL_AU_DAT_SEL1;
          }
        else if(sel.au_dat_sel2)
          {
            val = SRCIN_SEL_AU_DAT_SEL2;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      case CXD56_AUDIO_SIG_BUSIF1:
        if (sel.au_dat_sel1)
          {
            val = SRCIN_SEL_AU_DAT_SEL1;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      case CXD56_AUDIO_SIG_BUSIF2:
        if (sel.au_dat_sel2)
          {
            val = SRCIN_SEL_AU_DAT_SEL2;
          }
        else
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        break;

      case CXD56_AUDIO_SIG_MIX:
        if ((read_ac_reg(RI_COD_INSEL2) == COD_INSEL_SRC1)
         || (read_ac_reg(RI_COD_INSEL3) == COD_INSEL_SRC1))
          {
            return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
          }
        else
          {
            val = SRCIN_SEL_CODECDSP_MIX;
          }
        break;

      default:
        return CXD56_AUDIO_ECODE_REG_AC_SEL_INV;
    }

  write_ac_reg(ac_reg_id, val);

  return CXD56_AUDIO_ECODE_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_checkid(void)
{
  if (read_ac_reg(RI_REVID) != AC_REVID)
    {
      return CXD56_AUDIO_ECODE_REG_AC_REVID;
    }

  if (read_ac_reg(RI_DEVICEID) != AC_DEVICEID)
    {
      return CXD56_AUDIO_ECODE_REG_AC_DEVID;
    }

  return CXD56_AUDIO_ECODE_OK;
}

void cxd56_audio_ac_reg_resetdsp(void)
{
  write_ac_reg(RI_S_RESET, 1);
  write_ac_reg(RI_S_RESET, 0);

  /* Wait 20ms. */

  up_mdelay(20);
}

void cxd56_audio_ac_reg_initdsp(void)
{
  write_ac_reg(RI_PDN_DSPB,  0);
  write_ac_reg(RI_PDN_DSPS2, 0);
  write_ac_reg(RI_PDN_DSPS1, 0);
  write_ac_reg(RI_PDN_DSPC,  0);

  write_ac_reg(RI_DSPRAM4_CLR, 1);
  write_ac_reg(RI_DSPRAM2_CLR, 1);
  write_ac_reg(RI_DSPRAM1_CLR, 1);

  /* Wait 512 cycle @24.576MHz */

  up_mdelay(1);

  write_ac_reg(RI_DSPRAM4_CLR, 0);
  write_ac_reg(RI_DSPRAM2_CLR, 0);
  write_ac_reg(RI_DSPRAM1_CLR, 0);

  write_ac_reg(RI_PDN_DSPB,  1);
  write_ac_reg(RI_PDN_DSPS2, 1);
  write_ac_reg(RI_PDN_DSPS1, 1);
  write_ac_reg(RI_PDN_DSPC,  1);

  write_ac_reg(RI_S_RESET, 1);
  write_ac_reg(RI_S_RESET, 0);
}

void cxd56_audio_ac_reg_poweron_sdes(void)
{
  write_ac_reg(RI_SDES_EN, 1);
}

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_set_micmode(uint8_t mic_mode)
{
  switch (mic_mode)
    {
      case CXD56_AUDIO_CFG_MIC_MODE_128FS:
        write_ac_reg(RI_FS_FS,    0);
        write_ac_reg(RI_SER_MODE, 1);
        write_ac_reg(RI_ADC_FS,   1);
        break;

      case CXD56_AUDIO_CFG_MIC_MODE_64FS:
        write_ac_reg(RI_FS_FS,    1);
        write_ac_reg(RI_SER_MODE, 0);
        write_ac_reg(RI_ADC_FS,   0);
        break;

      default:
        return CXD56_AUDIO_ECODE_REG_AC_MICMODE;
    }

  write_ac_reg(RI_SEL_OUT1_L, 0);
  write_ac_reg(RI_SEL_OUT1_R, 1);
  write_ac_reg(RI_SEL_OUT2_L, 2);
  write_ac_reg(RI_SEL_OUT2_R, 3);
  write_ac_reg(RI_SEL_OUT3_L, 4);
  write_ac_reg(RI_SEL_OUT3_R, 5);
  write_ac_reg(RI_SEL_OUT4_L, 6);
  write_ac_reg(RI_SEL_OUT4_R, 7);

  return CXD56_AUDIO_ECODE_OK;
}

void cxd56_audio_ac_reg_poweron_codec(void)
{
  write_ac_reg(RI_PDN_DSPC, 0);
  write_ac_reg(RI_DSR_RATE, 1);
  write_ac_reg(RI_DIGSFT,   1);
}

void cxd56_audio_ac_reg_poweroff_codec(void)
{
  /* Disable AHBMASTER. */

  write_ac_reg(RI_MCK_AHBMSTR_EN, 0);

  /* Disable CODEC. */

  write_ac_reg(RI_ALC_EN, 0);
  write_ac_reg(RI_SPC_EN, 0);
  write_ac_reg(RI_DEQ_EN, 0);

  /* Disable DNC. */

  write_ac_reg(RI_DNC1_MUTE, 1);
  write_ac_reg(RI_DNC2_MUTE, 1);
  write_ac_reg(RI_DNC1_START, 0);
  write_ac_reg(RI_DNC2_START, 0);

  /* Disable SRC. */

  write_ac_reg(RI_SDIN1_EN, 0);
  write_ac_reg(RI_SDIN2_EN, 0);
  write_ac_reg(RI_SDOUT1_EN, 0);
  write_ac_reg(RI_SDOUT2_EN, 0);
  write_ac_reg(RI_SDCK_OUTENX, 1);
  write_ac_reg(RI_BLF_EN, 0);

  /* Disable SDES. */

  write_ac_reg(RI_PDM_OUT_EN, 0);
  write_ac_reg(RI_FS_CLK_EN, 0);
  write_ac_reg(RI_SDES_EN, 0);

  /* Power off SRC. */

  write_ac_reg(RI_PDN_DSPS1, 1);
  write_ac_reg(RI_PDN_DSPS2, 1);
  write_ac_reg(RI_PDN_DSPB, 1);

  /* Power off CODEC. */

  write_ac_reg(RI_PDN_DSPC, 1);

  /* Power off DNC. */

  write_ac_reg(RI_PDN_DNC1, 1);
  write_ac_reg(RI_PDN_DNC2, 1);
  write_ac_reg(RI_PDN_ANC, 1);
}

void cxd56_audio_ac_reg_enable_serialif(void)
{
  write_ac_reg(RI_FS_CLK_EN,  1);
  write_ac_reg(RI_PDM_OUT_EN, 1);
}

void cxd56_audio_ac_reg_init_selector(void)
{
  write_ac_reg(RI_AU_DAT_SEL1, AU_DAT_SEL_BUSIF1);
  write_ac_reg(RI_AU_DAT_SEL2, AU_DAT_SEL_BUSIF2);
  write_ac_reg(RI_COD_INSEL2,  COD_INSEL_AU_DAT_SEL1);
  write_ac_reg(RI_COD_INSEL3,  COD_INSEL_AU_DAT_SEL2);
  write_ac_reg(RI_SRC1IN_SEL,  SRCIN_SEL_AU_DAT_SEL1);
  write_ac_reg(RI_SRC2IN_SEL,  SRCIN_SEL_AU_DAT_SEL2);
}

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_set_alcspc(void)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Clear enable status. */

  write_ac_reg(RI_ALC_EN, 0);
  write_ac_reg(RI_SPC_EN, 0);

  if (CXD56_AUDIO_CFG_ALCSPC == CXD56_AUDIO_CFG_ALCSPC_ALC)
    {
      /* Set auto level control parameter. */

      ret = set_alc_param();
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          return ret;
        }

      write_ac_reg(RI_ALC_EN, 1);
    }
  else if (CXD56_AUDIO_CFG_ALCSPC == CXD56_AUDIO_CFG_ALCSPC_SPC)
    {
      /* Set sound pressure conter */

      ret = set_spc_param();
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          return ret;
        }

      write_ac_reg(RI_SPC_EN, 1);
    }

  return ret;
}

void cxd56_audio_ac_reg_poweron_dnc(void)
{
  write_ac_reg(RI_PDN_DNC1, 0);
  write_ac_reg(RI_PDN_DNC2, 0);
  write_ac_reg(RI_PDN_ANC,  0);
}

void cxd56_audio_ac_reg_poweroff_dnc(void)
{
  write_ac_reg(RI_PDN_DNC1, 1);
  write_ac_reg(RI_PDN_DNC2, 1);
  write_ac_reg(RI_PDN_ANC,  1);

  /* Clear DNC SRAM. */

  write_ac_reg(RI_DSPRAM3_CLR, 1);

  /* Wait 512 cycle @24.576MHz */

  up_mdelay(1);

  write_ac_reg(RI_DSPRAM3_CLR, 0);
}

void cxd56_audio_ac_reg_disable_dnc(cxd56_audio_dnc_id_t id)
{
  if (CXD56_AUDIO_DNC_ID_FB == id)
    {
      write_ac_reg(RI_DNC1_MUTE,  1);
      write_ac_reg(RI_DNC1_START, 0);
    }
  else
    {
      write_ac_reg(RI_DNC2_MUTE,  1);
      write_ac_reg(RI_DNC2_START, 0);
    }
}

void cxd56_audio_ac_reg_enable_dnc(cxd56_audio_dnc_id_t id)
{
  if (CXD56_AUDIO_DNC_ID_FB == id)
    {
      write_ac_reg(RI_DNC1_START, 1);
      write_ac_reg(RI_DNC1_MUTE,  0);
    }
  else
    {
      write_ac_reg(RI_DNC2_START, 1);
      write_ac_reg(RI_DNC2_MUTE,  0);
    }
}

void cxd56_audio_ac_reg_set_dncram(cxd56_audio_dnc_id_t id,
                                   FAR cxd56_audio_dnc_bin_t *bin)
{
  uint32_t iram_reg;
  uint32_t cram_reg;

  if (CXD56_AUDIO_DNC_ID_FB == id)
    {
      iram_reg  = DNC1_IRAM_BASE;
      cram_reg  = DNC1_CRAM_BASE;
    }
  else
    {
      iram_reg  = DNC2_IRAM_BASE;
      cram_reg  = DNC2_CRAM_BASE;
    }

  write_ac_reg(RI_RAM_RW_EN, 1);

  /* Write to SRAM. */

  write_dnc_ram(iram_reg, bin->firm,   DNC_IRAM_SIZE);
  write_dnc_ram(cram_reg, bin->config, DNC_CRAM_SIZE);

  write_ac_reg(RI_RAM_RW_EN, 0);
}

void cxd56_audio_ac_reg_enable_deq(void)
{
  write_ac_reg(RI_DEQ_EN, 1);
}

void cxd56_audio_ac_reg_disable_deq(void)
{
  write_ac_reg(RI_DEQ_EN, 0);
}

void cxd56_audio_ac_reg_set_deq_param(FAR cxd56_audio_deq_coef_t *deq)
{
  set_deq_coef(RI_DEQ_COEF_1B0, &deq->coef[0]);
  set_deq_coef(RI_DEQ_COEF_2B0, &deq->coef[1]);
  set_deq_coef(RI_DEQ_COEF_3B0, &deq->coef[2]);
  set_deq_coef(RI_DEQ_COEF_4B0, &deq->coef[3]);
  set_deq_coef(RI_DEQ_COEF_5B0, &deq->coef[4]);
  set_deq_coef(RI_DEQ_COEF_6B0, &deq->coef[5]);
}

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_poweron_cic(
                                     uint8_t mic_in,
                                     uint8_t mic_mode,
                                     uint8_t cic_num,
                                     FAR cxd56_audio_mic_gain_t *gain)
{
  /* Power on CIC. */

  if (mic_in == CXD56_AUDIO_CFG_CIC_IN_SEL_CXD)
    {
      if (cic_num > 3)
        {
          write_ac_reg(RI_CIC4IN_SEL, 0);
          write_ac_reg(RI_HPF4_MODE,  1);
        }

      if (cic_num > 2)
        {
          if (read_ac_reg(RI_PDN_AMICEXT) == 1)
            {
              write_ac_reg(RI_PDN_AMICEXT, 0);
            }

          write_ac_reg(RI_CIC3IN_SEL, 0);
          write_ac_reg(RI_HPF3_MODE,  1);
        }

      if (cic_num > 1)
        {
          write_ac_reg(RI_PDN_AMIC2,  0);
          write_ac_reg(RI_CIC2IN_SEL, 0);
          write_ac_reg(RI_HPF2_MODE,  1);
          write_ac_reg(RI_CIC2_GAIN_MODE, 1);
        }

      if (cic_num > 0)
        {
          write_ac_reg(RI_PDN_AMIC1,  0);
          write_ac_reg(RI_CIC1IN_SEL, 0);
          write_ac_reg(RI_HPF1_MODE,  1);
          write_ac_reg(RI_CIC1_GAIN_MODE, 1);
        }
    }
  else if(mic_in == CXD56_AUDIO_CFG_CIC_IN_SEL_DMICIF)
    {
      if (read_ac_reg(RI_PDN_DMIC) == 1)
        {
          write_ac_reg(RI_PDN_DMIC, 0);
        }

      if (cic_num > 3)
        {
          write_ac_reg(RI_CIC4IN_SEL, 1);
          write_ac_reg(RI_HPF4_MODE,  1);
        }

      if (cic_num > 2)
        {
          write_ac_reg(RI_CIC3IN_SEL, 1);
          write_ac_reg(RI_HPF3_MODE,  1);
        }

      if (cic_num > 1)
        {
          write_ac_reg(RI_CIC2IN_SEL, 1);
          write_ac_reg(RI_HPF2_MODE,  1);
          write_ac_reg(RI_CIC2_GAIN_MODE, 1);
        }

      if (cic_num > 0)
        {
          write_ac_reg(RI_CIC1IN_SEL, 1);
          write_ac_reg(RI_HPF1_MODE,  1);
          write_ac_reg(RI_CIC1_GAIN_MODE, 1);
        }
    }
  else
    {
      /* Do nothing. */
    }

  if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_128FS)
    {
      write_ac_reg(RI_ADC_FS, 1);
    }
  else if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_64FS)
    {
      write_ac_reg(RI_ADC_FS, 0);
    }
  else
    {
      return CXD56_AUDIO_ECODE_REG_AC_MICMODE;
    }

  set_cic_gain(cic_num, gain);

  return CXD56_AUDIO_ECODE_OK;
}

void cxd56_audio_ac_reg_poweroff_cic(void)
{
  /* Power off CIC. */

  write_ac_reg(RI_PDN_AMIC1,   1);
  write_ac_reg(RI_PDN_AMIC2,   1);
  write_ac_reg(RI_PDN_AMICEXT, 1);
  write_ac_reg(RI_PDN_DMIC,    1);
}

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_poweron_decim(uint8_t mic_mode,
                                                   uint8_t clk_mode)
{
  /* Enable AHBMASTER.
   * Because the output of DecimationFilter is input to BusIF.
   */

  write_ac_reg(RI_MCK_AHBMSTR_EN, 1);

  /* Power on DECIM. */

  write_ac_reg(RI_DECIM0_EN, 1);

  /* DECIM param */

  if ((mic_mode == CXD56_AUDIO_CFG_MIC_MODE_64FS) &&
      (clk_mode == CXD56_AUDIO_CLKMODE_HIRES))
    {
      write_ac_reg(RI_SEL_DECIM, 0);
    }
  else
    {
      write_ac_reg(RI_SEL_DECIM, 1);
    }

  if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_128FS)
    {
      write_ac_reg(RI_SEL_INF,  1);
      write_ac_reg(RI_DCMFS,    2);
      write_ac_reg(RI_DCMFS_34, 2);
    }
  else if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_64FS)
    {
      write_ac_reg(RI_SEL_INF,  0);
      write_ac_reg(RI_DCMFS,    1);
      write_ac_reg(RI_DCMFS_34, 1);
    }
  else
    {
      return CXD56_AUDIO_ECODE_REG_AC_MICMODE;
    }

  if (clk_mode == CXD56_AUDIO_CLKMODE_NORMAL)
    {
      write_ac_reg(RI_SEL_OUTF, 0);
    }
  else if (clk_mode == CXD56_AUDIO_CLKMODE_HIRES)
    {
      write_ac_reg(RI_SEL_OUTF, 2);
    }
  else
    {
      return CXD56_AUDIO_ECODE_REG_AC_CLKMODE;
    }

  /* DECIM_SEL */

  write_ac_reg(RI_OUTEN_MIC2R_A, ((0x0f >> 3) & 0x01));
  write_ac_reg(RI_OUTEN_MIC2L_A, ((0x0f >> 2) & 0x01));
  write_ac_reg(RI_OUTEN_MIC1R_A, ((0x0f >> 1) & 0x01));
  write_ac_reg(RI_OUTEN_MIC1L_A, ((0x0f >> 0) & 0x01));

  write_ac_reg(RI_OUTEN_MIC2R_B, ((0x0f >> 3) & 0x01));
  write_ac_reg(RI_OUTEN_MIC2L_B, ((0x0f >> 2) & 0x01));
  write_ac_reg(RI_OUTEN_MIC1R_B, ((0x0f >> 1) & 0x01));
  write_ac_reg(RI_OUTEN_MIC1L_B, ((0x0f >> 0) & 0x01));

  return CXD56_AUDIO_ECODE_OK;
}

void cxd56_audio_ac_reg_poweroff_decim(void)
{
  write_ac_reg(RI_DECIM0_EN, 0);
}

void cxd56_audio_ac_reg_poweron_smaster(uint8_t clk_mode)
{
  /* Power on S-Mster. */

  write_ac_reg(RI_PDN_SMSTR, 0);

  /* Set NSDD. */

  write_ac_reg(RI_NSDD, 0x07fb5);

  /* Set NSX2. */

  if ((clk_mode == CXD56_AUDIO_CLKMODE_HIRES) &&
      (CXD56_AUDIO_CFG_MCLK == CXD56_AUDIO_CFG_XTAL_49_152MHZ))
    {
      write_ac_reg(RI_NSX2, 1);
    }
  else
    {
      write_ac_reg(RI_NSX2, 0);
    }
}

void cxd56_audio_ac_reg_poweroff_smaster(void)
{
  write_ac_reg(RI_PDN_SMSTR, 1);
}

void cxd56_audio_ac_reg_enable_smaster(void)
{
  write_ac_reg(RI_NSPMUTE, 0);
}

void cxd56_audio_ac_reg_disable_smaster(void)
{
  write_ac_reg(RI_NSPMUTE, 1);
}

void cxd56_audio_ac_reg_enable_beep(void)
{
  write_ac_reg(RI_BEEP_ON, 1);
}

void cxd56_audio_ac_reg_disable_beep(void)
{
  write_ac_reg(RI_BEEP_ON, 0);
}

void cxd56_audio_ac_reg_set_beep_freq(uint32_t freq)
{
  write_ac_reg(RI_BEEP_FREQ, freq);
}

void cxd56_audio_ac_reg_set_beep_vol(uint32_t vol)
{
  write_ac_reg(RI_BEEP_VOL, vol);
}

void cxd56_audio_ac_reg_set_cicgain(uint8_t cic_num,
                                   FAR cxd56_audio_mic_gain_t *gain)
{
  set_cic_gain(cic_num, gain);
}

void cxd56_audio_ac_reg_enable_digsft(void)
{
  write_ac_reg(RI_DIGSFT, 1);
}

void cxd56_audio_ac_reg_disable_digsft(void)
{
  write_ac_reg(RI_DIGSFT, 0);
}

void cxd56_audio_ac_reg_set_dsrrate(uint32_t rate)
{
  write_ac_reg(RI_DSR_RATE, rate);
}

void cxd56_audio_ac_reg_set_seloutch(
                        FAR cxd56_audio_ac_reg_seloutch_t *seloutch)
{
  write_ac_reg(RI_SEL_OUT1_L, seloutch->ch[0]);
  write_ac_reg(RI_SEL_OUT1_R, seloutch->ch[1]);
  write_ac_reg(RI_SEL_OUT2_L, seloutch->ch[2]);
  write_ac_reg(RI_SEL_OUT2_R, seloutch->ch[3]);
  write_ac_reg(RI_SEL_OUT3_L, seloutch->ch[4]);
  write_ac_reg(RI_SEL_OUT3_R, seloutch->ch[5]);
  write_ac_reg(RI_SEL_OUT4_L, seloutch->ch[6]);
  write_ac_reg(RI_SEL_OUT4_R, seloutch->ch[7]);
}

void cxd56_audio_ac_reg_enable_dma(void)
{
  write_ac_reg(RI_MCK_AHBMSTR_EN, 1);
}

void cxd56_audio_ac_reg_poweron_i2s(uint8_t clk_mode)
{
#ifdef CONFIG_CXD56_I2S0
  /* Power on I2S0 device. */

  poweron_i2s0();
#endif /* CONFIG_CXD56_I2S0 */

#ifdef CONFIG_CXD56_I2S1
  /* Power on I2S1 device. */

  poweron_i2s1();
#endif /* CONFIG_CXD56_I2S1 */

  /* Power on output filter. */

  write_ac_reg(RI_PDN_DSPB, 0);

  /* Set hi-res mode. */

  uint32_t is_hires =
    (clk_mode == CXD56_AUDIO_CLKMODE_HIRES) ? 1 : 0;

  write_ac_reg(RI_HI_RES_MODE, is_hires);

  /* Enable BLF block. */

  write_ac_reg(RI_BLF_EN, 1);

  /* Disable auto mute of SRC. */

  write_ac_reg(RI_ARWPHSET, 0);

  /* Enable clock halt of SRC. */

  write_ac_reg(RI_HALT_INHIBIT, 0);
}

void cxd56_audio_ac_reg_enable_i2s_src1(void)
{
  write_ac_reg(RI_SDIN1_EN, 1);
  write_ac_reg(RI_SDOUT1_EN, 1);
}

void cxd56_audio_ac_reg_enable_i2s_src2(void)
{
  write_ac_reg(RI_SDIN2_EN, 1);
  write_ac_reg(RI_SDOUT2_EN, 1);
}

void cxd56_audio_ac_reg_enable_i2s_bcklrckout(void)
{
  write_ac_reg(RI_SDCK_OUTENX, 0);
}

void cxd56_audio_ac_reg_disable_i2s_bcklrckout(void)
{
  write_ac_reg(RI_SDCK_OUTENX, 1);
}

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_set_selector(cxd56_audio_signal_t sig,
                                                  cxd56_audio_sel_t sel)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (sel.au_dat_sel1)
    {
      ret = set_au_dat_sel(RI_AU_DAT_SEL1, sig);
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  if (sel.au_dat_sel2)
    {
      ret = set_au_dat_sel(RI_AU_DAT_SEL2, sig);
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  if (sel.cod_insel2)
    {
      ret = set_cod_insel(RI_COD_INSEL2, sig, sel);
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  if (sel.cod_insel3)
    {
      ret = set_cod_insel(RI_COD_INSEL3, sig, sel);
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  if (sel.src1in_sel)
    {
      ret = set_srcin_sel(RI_SRC1IN_SEL, sig, sel);
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  if (sel.src2in_sel)
    {
      ret = set_srcin_sel(RI_SRC2IN_SEL, sig, sel);
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_ac_reg_enable_cstereo(bool sign_inv,
                                                    int16_t vol)
{
  /* Set Clear Stereo data inverse. false: Positive, true: Negative */

  write_ac_reg(RI_CS_SIGN, sign_inv);

  if ((CS_VOL_MIN <= vol) && (vol <= CS_VOL_MAX))
    {
      uint32_t val = ((vol + CS_VOL_OFFSET) / 5) & 0x7f;
      write_ac_reg(RI_CS_VOL, val);
    }
  else
    {
      return CXD56_AUDIO_ECODE_REG_AC_CSTE_VOL;
    }

  return CXD56_AUDIO_ECODE_OK;
}

void cxd56_audio_ac_reg_disable_cstereo(void)
{
  write_ac_reg(RI_CS_VOL, 0x00);
}

void cxd56_audio_ac_reg_set_vol_sdin1(uint32_t vol)
{
  write_ac_reg(RI_SDIN1_VOL, vol);
}

void cxd56_audio_ac_reg_set_vol_sdin2(uint32_t vol)
{
  write_ac_reg(RI_SDIN2_VOL, vol);
}

void cxd56_audio_ac_reg_set_vol_dac(uint32_t vol)
{
  write_ac_reg(RI_DAC_VOL, vol);
}
