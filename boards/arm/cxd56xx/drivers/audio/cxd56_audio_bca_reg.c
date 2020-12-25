/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_bca_reg.c
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
#include <arch/chip/audio.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_bca_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCA_REG_BASE  (0x0c000000 + 0x02300000)

#define TRANS_CH_NUM_MAX   8
#define TRANS_CH_NUM_24BIT 8
#define TRANS_CH_NUM_16BIT 4
#define TRANS_CH_NO_SELECT 8

enum dma_i2s_in_sel_e
{
  I2S_IN_SEL_SRC1L = 0,
  I2S_IN_SEL_SRC1R = 1,
  I2S_IN_SEL_UNUSE
};

enum dma_i2s_out_sel_e
{
  I2S_OUT_SEL_SD1L  = 0,
  I2S_OUT_SEL_SD1R  = 1,
  I2S_OUT_SEL_UNUSE
};

#define BCA_REG_MAX_BIT 32

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct audio_bca_reg_s
{
  uint32_t addr;
  uint32_t pos;
  uint32_t len;
  uint32_t init;
};

const struct audio_bca_reg_s g_bca_reg[BCA_REG_MAX_ENTRY] =
{
  {0x1000,  2, 30, 0x00000000},  /* Mic_In_start_adr (0x00000000)     */
  {0x1004,  0, 32, 0x00000000},  /* Mic_In_sample_no (0x00000000)     */
  {0x1008,  0,  2, 0x00000000},  /* Mic_In_rtd_trg (0x00)             */
  {0x1008,  2,  1, 0x00000000},  /* Mic_In_nointr (0x00)              */
  {0x100c,  0,  1, 0x00000000},  /* Mic_In_bitwt (0x00)               */
  {0x1010,  0,  4, 0x00000000},  /* Mic_In_ch8_sel (0x00)             */
  {0x1010,  4,  4, 0x00000000},  /* Mic_In_ch7_sel (0x00)             */
  {0x1010,  8,  4, 0x00000000},  /* Mic_In_ch6_sel (0x00)             */
  {0x1010, 12,  4, 0x00000000},  /* Mic_In_ch5_sel (0x00)             */
  {0x1010, 16,  4, 0x00000000},  /* Mic_In_ch4_sel (0x00)             */
  {0x1010, 20,  4, 0x00000000},  /* Mic_In_ch3_sel (0x00)             */
  {0x1010, 24,  4, 0x00000000},  /* Mic_In_ch2_sel (0x00)             */
  {0x1010, 28,  4, 0x00000000},  /* Mic_In_ch1_sel (0x00)             */
  {0x1014,  0,  1, 0x00000000},  /* Mic_In_start (0x00)               */
  {0x1014,  8,  8, 0x00000000},  /* Mic_In_error_setting (0x00)       */
  {0x1014, 16,  4, 0x00000000},  /* Mic_In_monbuf (0x00)              */
  {0x1080,  2, 30, 0x00000000},  /* I2s1_In_start_adr (0x00000000)    */
  {0x1084,  0, 32, 0x00000000},  /* I2s1_In_sample_no (0x00000000)    */
  {0x1088,  0,  2, 0x00000000},  /* I2s1_In_rtd_trg (0x00)            */
  {0x1088,  2,  1, 0x00000000},  /* I2s1_In_nointr (0x00)             */
  {0x108c,  0,  1, 0x00000000},  /* I2s1_In_bitwt (0x00)              */
  {0x1090,  0,  2, 0x00000000},  /* I2s1_In_ch2_sel (0x00)            */
  {0x1090,  4,  2, 0x00000000},  /* I2s1_In_ch1_sel (0x00)            */
  {0x1094,  0,  1, 0x00000000},  /* I2s1_In_Mon_start (0x00)          */
  {0x1094,  8,  8, 0x00000000},  /* I2s1_In_Mon_error_setting (0x00)  */
  {0x1094, 16,  4, 0x00000000},  /* I2s1_In_Mon_monbuf (0x00)         */
  {0x10a0,  2, 30, 0x00000000},  /* I2s2_In_start_adr (0x00000000)    */
  {0x10a4,  0, 32, 0x00000000},  /* I2s2_In_sample_no (0x00000000)    */
  {0x10a8,  0,  2, 0x00000000},  /* I2s2_In_rtd_trg (0x00)            */
  {0x10a8,  2,  1, 0x00000000},  /* I2s2_In_nointr (0x00)             */
  {0x10ac,  0,  1, 0x00000000},  /* I2s2_In_bitwt (0x00)              */
  {0x10b0,  0,  2, 0x00000000},  /* I2s2_In_ch2_sel (0x00)            */
  {0x10b0,  4,  2, 0x00000000},  /* I2s2_In_ch1_sel (0x00)            */
  {0x10b4,  0,  1, 0x00000000},  /* I2s2_In_Mon_start (0x00)          */
  {0x10b4,  8,  8, 0x00000000},  /* I2s2_In_Mon_error_setting (0x00)  */
  {0x10b4, 16,  4, 0x00000000},  /* I2s2_In_Mon_monbuf (0x00)         */
  {0x10c0,  2, 30, 0x00000000},  /* I2s1_Out_start_adr (0x00000000)   */
  {0x10c4,  0, 32, 0x00000000},  /* I2s1_Out_sample_no (0x00000000)   */
  {0x10c8,  0,  2, 0x00000000},  /* I2s1_Out_rtd_trg (0x00)           */
  {0x10c8,  2,  1, 0x00000000},  /* I2s1_Out_nointr (0x00)            */
  {0x10cc,  0,  1, 0x00000000},  /* I2s1_Out_bitwt (0x00)             */
  {0x10d0,  0,  2, 0x00000000},  /* I2s1_Out_sd1_r_sel (0x00)         */
  {0x10d0,  4,  2, 0x00000000},  /* I2s1_Out_sd1_l_sel (0x00)         */
  {0x10d4,  0,  1, 0x00000000},  /* I2s1_Out_Mon_start (0x00)         */
  {0x10d4,  8,  8, 0x00000000},  /* I2s1_Out_Mon_error_setting (0x00) */
  {0x10d4, 16,  4, 0x00000000},  /* I2s1_Out_Mon_monbuf (0x00)        */
  {0x10e0,  2, 30, 0x00000000},  /* I2s2_Out_start_adr (0x00000000)   */
  {0x10e4,  0, 32, 0x00000000},  /* I2s2_Out_sample_no (0x00000000)   */
  {0x10e8,  0,  2, 0x00000000},  /* I2s2_Out_rtd_trg (0x00)           */
  {0x10e8,  2,  1, 0x00000000},  /* I2s2_Out_nointr (0x00)            */
  {0x10ec,  0,  1, 0x00000000},  /* I2s2_Out_bitwt (0x00)             */
  {0x10f0,  0,  2, 0x00000000},  /* I2s2_Out_sd1_r_sel (0x00)         */
  {0x10f0,  4,  2, 0x00000000},  /* I2s2_Out_sd1_l_sel (0x00)         */
  {0x10f4,  0,  1, 0x00000000},  /* I2s2_Out_Mon_start (0x00)         */
  {0x10f4,  8,  8, 0x00000000},  /* I2s2_Out_Mon_error_setting (0x00) */
  {0x10f4, 16,  4, 0x00000000},  /* I2s2_Out_Mon_monbuf (0x00)        */
  {0x1110,  0,  1, 0x00000000},  /* I2s_ensel (0x00)                  */
  {0x1120,  0, 32, 0x00000000},  /* Mici_prdat_u (0x00000000)         */
  {0x1130,  0, 32, 0x00000000},  /* I2s1_In_prdat_u (0x00000000)      */
  {0x1134,  0, 32, 0x00000000},  /* I2s2_In_prdat_u (0x00000000)      */
  {0x1138,  0, 32, 0x00000000},  /* I2s1_Out_prdat_d (0x00000000)     */
  {0x113c,  0, 32, 0x00000000},  /* I2s2_Out_prdat_d (0x00000000)     */
  {0x1140,  0,  1, 0x00000000},  /* Mic_Int_Ctrl_done_mic (0x00)      */
  {0x1140,  1,  1, 0x00000000},  /* Mic_Int_Ctrl_err_mic (0x00)       */
  {0x1140,  2,  1, 0x00000000},  /* Mic_Int_Ctrl_smp_mic (0x00)       */
  {0x1140,  3,  1, 0x00000000},  /* Mic_Int_Ctrl_cmb_mic (0x00)       */
  {0x1144,  0,  1, 0x00000000},  /* I2s1_Int_Ctrl_done_i2so (0x00)    */
  {0x1144,  1,  1, 0x00000000},  /* I2s1_Int_Ctrl_err_i2so (0x00)     */
  {0x1144,  2,  1, 0x00000000},  /* I2s1_Int_Ctrl_done_i2si (0x00)    */
  {0x1144,  3,  1, 0x00000000},  /* I2s1_Int_Ctrl_err_i2si (0x00)     */
  {0x1144,  4,  1, 0x00000000},  /* I2s1_Int_Ctrl_smp_i2s (0x00)      */
  {0x1144,  5,  1, 0x00000000},  /* I2s1_Int_Ctrl_cmb_i2s (0x00)      */
  {0x1148,  0,  1, 0x00000000},  /* I2s2_Int_Ctrl_done_i2so (0x00)    */
  {0x1148,  1,  1, 0x00000000},  /* I2s2_Int_Ctrl_err_i2so (0x00)     */
  {0x1148,  2,  1, 0x00000000},  /* I2s2_Int_Ctrl_done_i2si (0x00)    */
  {0x1148,  3,  1, 0x00000000},  /* I2s2_Int_Ctrl_err_i2si (0x00)     */
  {0x1148,  4,  1, 0x00000000},  /* I2s2_Int_Ctrl_smp_i2s (0x00)      */
  {0x1148,  5,  1, 0x00000000},  /* I2s2_Int_Ctrl_cmb_i2s (0x00)      */
  {0x114c,  0,  1, 0x00000001},  /* Mic_Int_Mask_done_mic (0x00)      */
  {0x114c,  1,  1, 0x00000001},  /* Mic_Int_Mask_err_mic (0x00)       */
  {0x114c,  2,  1, 0x00000001},  /* Mic_Int_Mask_smp_mic (0x00)       */
  {0x114c,  3,  1, 0x00000001},  /* Mic_Int_Mask_cmb_mic (0x00)       */
  {0x114c, 30,  1, 0x00000000},  /* Mic_Int_Mask_nostpmsk (0x00)      */
  {0x114c, 31,  1, 0x00000000},  /* Mic_Int_Mask_srst_mic (0x00)      */
  {0x1150,  0,  1, 0x00000001},  /* I2s1_Int_Mask_done_i2so (0x00)    */
  {0x1150,  1,  1, 0x00000001},  /* I2s1_Int_Mask_err_i2so (0x00)     */
  {0x1150,  2,  1, 0x00000001},  /* I2s1_Int_Mask_done_i2si (0x00)    */
  {0x1150,  3,  1, 0x00000001},  /* I2s1_Int_Mask_err_i2si (0x00)     */
  {0x1150,  4,  1, 0x00000001},  /* I2s1_Int_Mask_smp_i2s (0x00)      */
  {0x1150,  5,  1, 0x00000001},  /* I2s1_Int_Mask_cmb_i2s (0x00)      */
  {0x1150, 30,  1, 0x00000000},  /* I2s1_Int_Mask_nostpmsk_i2s (0x00) */
  {0x1150, 31,  1, 0x00000000},  /* I2s1_Int_Mask_srst_i2s (0x00)     */
  {0x1154,  0,  1, 0x00000001},  /* I2s2_Int_Mask_done_i2so (0x00)    */
  {0x1154,  1,  1, 0x00000001},  /* I2s2_Int_Mask_err_i2so (0x00)     */
  {0x1154,  2,  1, 0x00000001},  /* I2s2_Int_Mask_done_i2si (0x00)    */
  {0x1154,  3,  1, 0x00000001},  /* I2s2_Int_Mask_err_i2si (0x00)     */
  {0x1154,  4,  1, 0x00000001},  /* I2s2_Int_Mask_smp_i2s (0x00)      */
  {0x1154,  5,  1, 0x00000001},  /* I2s2_Int_Mask_cmb_i2s (0x00)      */
  {0x1154, 30,  1, 0x00000000},  /* I2s2_Int_Mask_nostpmsk_i2s (0x00) */
  {0x1154, 31,  1, 0x00000000},  /* I2s2_Int_Mask_srst_i2s (0x00)     */
  {0x1158,  0,  1, 0x00000001},  /* Int_m_hresp_err (0x01)            */
  {0x1158,  8,  1, 0x00000001},  /* Int_m_I2s1_bck_err1 (0x01)        */
  {0x1158,  9,  1, 0x00000001},  /* Int_m_I2s1_bck_err2 (0x01)        */
  {0x1158, 10,  1, 0x00000001},  /* Int_m_anc_faint (0x01)            */
  {0x1158, 17,  1, 0x00000001},  /* Int_m_ovf_smasl (0x01)            */
  {0x1158, 18,  1, 0x00000001},  /* Int_m_ovf_smasr (0x01)            */
  {0x1158, 21,  1, 0x00000001},  /* Int_m_ovf_dnc1l (0x01)            */
  {0x1158, 22,  1, 0x00000001},  /* Int_m_ovf_dnc1r (0x01)            */
  {0x1158, 23,  1, 0x00000001},  /* Int_m_ovf_dnc2l (0x01)            */
  {0x1158, 24,  1, 0x00000001},  /* Int_m_ovf_dnc2r (0x01)            */
  {0x115c,  0,  1, 0x00000000},  /* Int_clr_hresp_err (0x00)          */
  {0x115c,  8,  1, 0x00000000},  /* Int_clr_I2s1_bck_err1 (0x00)      */
  {0x115c,  9,  1, 0x00000000},  /* Int_clr_I2s1_bck_err2 (0x00)      */
  {0x115c, 10,  1, 0x00000000},  /* Int_clr_anc_faint (0x00)          */
  {0x115c, 17,  1, 0x00000000},  /* Int_clr_ovf_smasl (0x00)          */
  {0x115c, 18,  1, 0x00000000},  /* Int_clr_ovf_smasr (0x00)          */
  {0x115c, 21,  1, 0x00000000},  /* Int_clr_ovf_dnc1l (0x00)          */
  {0x115c, 22,  1, 0x00000000},  /* Int_clr_ovf_dnc1r (0x00)          */
  {0x115c, 23,  1, 0x00000000},  /* Int_clr_ovf_dnc2l (0x00)          */
  {0x115c, 24,  1, 0x00000000},  /* Int_clr_ovf_dnc2r (0x00)          */
  {0x1160,  0,  1, 0x00000000},  /* Int_hresp_err (0x00)              */
  {0x1160,  8,  1, 0x00000000},  /* Int_i2s_bck_err1 (0x00)           */
  {0x1160,  9,  1, 0x00000000},  /* Int_i2s_bck_err2 (0x00)           */
  {0x1160, 10,  1, 0x00000000},  /* Int_anc_faint (0x00)              */
  {0x1160, 17,  1, 0x00000000},  /* Int_ovf_smasl (0x00)              */
  {0x1160, 18,  1, 0x00000000},  /* Int_ovf_smasr (0x00)              */
  {0x1160, 21,  1, 0x00000000},  /* Int_ovf_dnc1l (0x00)              */
  {0x1160, 22,  1, 0x00000000},  /* Int_ovf_dnc1r (0x00)              */
  {0x1160, 23,  1, 0x00000000},  /* Int_ovf_dnc2l (0x00)              */
  {0x1160, 24,  1, 0x00000000},  /* Int_ovf_dnc2r (0x00)              */
  {0x1180,  8, 24, 0x00000000},  /* Dbg_Mic_ch1_data (0x00)           */
  {0x1184,  8, 24, 0x00000000},  /* Dbg_Mic_ch2_data (0x00)           */
  {0x1188,  8, 24, 0x00000000},  /* Dbg_Mic_ch3_data (0x00)           */
  {0x118c,  8, 24, 0x00000000},  /* Dbg_Mic_ch4_data (0x00)           */
  {0x1190,  8, 24, 0x00000000},  /* Dbg_Mic_ch5_data (0x00)           */
  {0x1194,  8, 24, 0x00000000},  /* Dbg_Mic_ch6_data (0x00)           */
  {0x1198,  8, 24, 0x00000000},  /* Dbg_Mic_ch7_data (0x00)           */
  {0x119c,  8, 24, 0x00000000},  /* Dbg_Mic_ch8_data (0x00)           */
  {0x11a0,  8, 24, 0x00000000},  /* Dbg_I2s1_u_ch1_data (0x00)        */
  {0x11a4,  8, 24, 0x00000000},  /* Dbg_I2s1_u_ch2_data (0x00)        */
  {0x11a8,  8, 24, 0x00000000},  /* Dbg_I2s1_d_ch1_data (0x00)        */
  {0x11ac,  8, 24, 0x00000000},  /* Dbg_I2s1_d_ch2_data (0x00)        */
  {0x11b0,  8, 24, 0x00000000},  /* Dbg_I2s2_u_ch1_data (0x00)        */
  {0x11b4,  8, 24, 0x00000000},  /* Dbg_I2s2_u_ch2_data (0x00)        */
  {0x11b8,  8, 24, 0x00000000},  /* Dbg_I2s2_d_ch1_data (0x00)        */
  {0x11bc,  8, 24, 0x00000000},  /* Dbg_I2s2_d_ch2_data (0x00)        */
  {0x11c0,  0,  1, 0x00000000},  /* Dbg_Ctrl_mic_dbg_en (0x00)        */
  {0x11c0,  1,  1, 0x00000000},  /* Dbg_Ctrl_I2s1_dbg_u_en (0x00)     */
  {0x11c0,  2,  1, 0x00000000},  /* Dbg_Ctrl_I2s1_dbg_d_en (0x00)     */
  {0x11c0,  3,  1, 0x00000000},  /* Dbg_Ctrl_I2s2_dbg_u_en (0x00)     */
  {0x11c0,  4,  1, 0x00000000},  /* Dbg_Ctrl_I2s2_dbg_d_en (0x00)     */
  {0x11f0,  0,  1, 0x00000000},  /* Clk_En_ahbmstr_mic_en (0x00)      */
  {0x11f0,  1,  1, 0x00000000},  /* Clk_En_ahbmstr_I2s1_en (0x00)     */
  {0x11f0,  2,  1, 0x00000000},  /* Clk_En_ahbmstr_I2s2_en (0x00)     */
  {0x11fc,  0,  8, 0x00000064},  /* Mclk_Mon_thresh (0x64)            */
  {0x1730,  0, 32, 0x00000000},  /* AHB MASTER MIC MASK (0x00)        */
  {0x1f30,  0, 32, 0x00000000},  /* AHB MASTER I2S1 MASK (0x00)       */
  {0x2730,  0, 32, 0x00000000},  /* AHB MASTER I2S2 MASK (0x00)       */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

uint32_t write_bca_reg(BCA_REG_ID reg_id, uint32_t data)
{
  volatile uint32_t *addr;
  uint32_t curr;
  uint32_t mask = (g_bca_reg[reg_id].len == BCA_REG_MAX_BIT) ?
                    0xffffffff : (1 << g_bca_reg[reg_id].len) - 1;

  addr = (volatile uint32_t *)(BCA_REG_BASE + g_bca_reg[reg_id].addr);

  curr = *addr & ~(mask << g_bca_reg[reg_id].pos);
  *addr = curr | ((data & mask) << g_bca_reg[reg_id].pos);

  return 0;
}

uint32_t write_bca_reg_mask(BCA_REG_ID reg_id)
{
  volatile uint32_t *addr;
  uint32_t mask = (g_bca_reg[reg_id].len == BCA_REG_MAX_BIT) ?
                    0xffffffff : (1 << g_bca_reg[reg_id].len) - 1;

  addr = (volatile uint32_t *)(BCA_REG_BASE + g_bca_reg[reg_id].addr);
  *addr = mask << g_bca_reg[reg_id].pos;

  return 0;
}

uint32_t read_bca_reg(BCA_REG_ID reg_id)
{
  volatile uint32_t *addr;
  uint32_t data = 0;
  uint32_t mask = (g_bca_reg[reg_id].len == BCA_REG_MAX_BIT) ?
                    0xffffffff : (1 << g_bca_reg[reg_id].len) - 1;

  addr = (volatile uint32_t *)(BCA_REG_BASE + g_bca_reg[reg_id].addr);
  data = (*addr >> g_bca_reg[reg_id].pos) & mask;

  return data;
}

uint32_t write32_bca_reg(uint32_t offset, uint32_t data)
{
  volatile uint32_t *addr;

  addr = (volatile uint32_t *)(BCA_REG_BASE + offset);
  *addr = data;

  return 0;
}

uint32_t read32_bca_reg(uint32_t offset)
{
  volatile uint32_t *addr;
  uint32_t data = 0;

  addr = (volatile uint32_t *)(BCA_REG_BASE + offset);
  data = *addr;

  return data;
}

void enable_mic_in_fmt24(uint8_t mic_num)
{
  uint8_t i;

  BCA_REG_ID mic_ch_sell[TRANS_CH_NUM_MAX] =
    {
      BCA_MIC_IN_CH1_SEL,
      BCA_MIC_IN_CH2_SEL,
      BCA_MIC_IN_CH3_SEL,
      BCA_MIC_IN_CH4_SEL,
      BCA_MIC_IN_CH5_SEL,
      BCA_MIC_IN_CH6_SEL,
      BCA_MIC_IN_CH7_SEL,
      BCA_MIC_IN_CH8_SEL
    };

  mic_num = (mic_num > TRANS_CH_NUM_24BIT) ? TRANS_CH_NUM_24BIT : mic_num;

  write_bca_reg(BCA_MIC_IN_BITWT, 0);

  for (i = 0; i < mic_num; i++)
    {
      write_bca_reg(mic_ch_sell[i], i);
    }

  for (i = mic_num; i < TRANS_CH_NUM_24BIT; i++)
    {
      write_bca_reg(mic_ch_sell[i], TRANS_CH_NO_SELECT);
    }

  write_bca_reg(BCA_CLK_EN_AHBMASTER_MIC_EN, 1);
  write_bca_reg(BCA_MIC_IN_START_ADR,      0x00000000);
  write_bca_reg(BCA_MIC_IN_SAMPLE_NO,      0);
}

void enable_mic_in_fmt16(uint8_t mic_num)
{
  uint8_t i;

  BCA_REG_ID mic_ch_sell[TRANS_CH_NUM_MAX] =
    {
      BCA_MIC_IN_CH1_SEL,
      BCA_MIC_IN_CH2_SEL,
      BCA_MIC_IN_CH3_SEL,
      BCA_MIC_IN_CH4_SEL,
      BCA_MIC_IN_CH5_SEL,
      BCA_MIC_IN_CH6_SEL,
      BCA_MIC_IN_CH7_SEL,
      BCA_MIC_IN_CH8_SEL
    };

  mic_num = (mic_num > (TRANS_CH_NUM_16BIT * 2)) ?
              TRANS_CH_NUM_16BIT : (mic_num + 1) / 2;

  write_bca_reg(BCA_MIC_IN_BITWT, 1);

  for (i = 0; i < mic_num; i++)
    {
      write_bca_reg(mic_ch_sell[i], i);
    }

  for (i = mic_num; i < TRANS_CH_NUM_MAX; i++)
    {
      write_bca_reg(mic_ch_sell[i], TRANS_CH_NO_SELECT);
    }

  write_bca_reg(BCA_CLK_EN_AHBMASTER_MIC_EN, 1);
  write_bca_reg(BCA_MIC_IN_START_ADR,      0x00000000);
  write_bca_reg(BCA_MIC_IN_SAMPLE_NO,      0);
}

void enable_i2s1_out_fmt24(void)
{
  write_bca_reg(BCA_I2S1_OUT_SD1_L_SEL,     I2S_OUT_SEL_SD1L);
  write_bca_reg(BCA_I2S1_OUT_SD1_R_SEL,     I2S_OUT_SEL_SD1R);
  write_bca_reg(BCA_I2S1_OUT_BITWT,         0);
  write_bca_reg(BCA_CLK_EN_AHBMASTER_I2S1_EN, 1);
  write_bca_reg(BCA_I2S1_OUT_START_ADR,     0x00000000);
  write_bca_reg(BCA_I2S2_OUT_SAMPLE_NO,     0);
}

void enable_i2s1_out_fmt16(void)
{
  write_bca_reg(BCA_I2S1_OUT_SD1_L_SEL,     I2S_OUT_SEL_SD1L);
  write_bca_reg(BCA_I2S1_OUT_SD1_R_SEL,     I2S_OUT_SEL_SD1R);
  write_bca_reg(BCA_I2S1_OUT_BITWT,         1);
  write_bca_reg(BCA_CLK_EN_AHBMASTER_I2S1_EN, 1);
  write_bca_reg(BCA_I2S1_OUT_START_ADR,     0x00000000);
  write_bca_reg(BCA_I2S2_OUT_SAMPLE_NO,     0);
}

void enable_i2s2_out_fmt24(void)
{
  write_bca_reg(BCA_I2S2_OUT_SD1_L_SEL,     I2S_OUT_SEL_SD1L);
  write_bca_reg(BCA_I2S2_OUT_SD1_R_SEL,     I2S_OUT_SEL_SD1R);
  write_bca_reg(BCA_I2S2_OUT_BITWT,         0);
  write_bca_reg(BCA_CLK_EN_AHBMASTER_I2S2_EN, 1);
  write_bca_reg(BCA_I2S2_OUT_START_ADR,     0x00000000);
  write_bca_reg(BCA_I2S2_OUT_SAMPLE_NO,     0);
}

void enable_i2s2_out_fmt16(void)
{
  write_bca_reg(BCA_I2S2_OUT_SD1_L_SEL,     I2S_OUT_SEL_SD1L);
  write_bca_reg(BCA_I2S2_OUT_SD1_R_SEL,     I2S_OUT_SEL_SD1R);
  write_bca_reg(BCA_I2S2_OUT_BITWT,         1);
  write_bca_reg(BCA_CLK_EN_AHBMASTER_I2S2_EN, 1);
  write_bca_reg(BCA_I2S2_OUT_START_ADR,     0x00000000);
  write_bca_reg(BCA_I2S2_OUT_SAMPLE_NO,     0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cxd56_audio_bca_reg_clear_bck_err_int(void)
{
  write_bca_reg(BCA_INT_M_I2S1_BCL_ERR1, 0);
  write_bca_reg(BCA_INT_M_I2S1_BCL_ERR2, 0);
}

void cxd56_audio_bca_reg_set_smaster(void)
{
  write_bca_reg(BCA_INT_M_OVF_SMASL, 0);
  write_bca_reg(BCA_INT_M_OVF_SMASR, 0);
}

void cxd56_audio_bca_reg_set_datarate(uint8_t clk_mode)
{
  if (clk_mode == CXD56_AUDIO_CLKMODE_HIRES)
    {
      /* Set 4fs. */

      write_bca_reg(BCA_I2S_ENSEL, 1);
    }
  else
    {
      /* Set 1fs. */

      write_bca_reg(BCA_I2S_ENSEL, 0);
    }
}

void cxd56_audio_bca_reg_en_fmt24(cxd56_audio_dma_t handle, uint8_t ch_num)
{
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        enable_mic_in_fmt24(ch_num);
        break;
      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        enable_i2s1_out_fmt24();
        break;
      default:
        enable_i2s2_out_fmt24();
        break;
    }
}

void cxd56_audio_bca_reg_en_fmt16(cxd56_audio_dma_t handle, uint8_t ch_num)
{
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        enable_mic_in_fmt16(ch_num);
        break;
      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        enable_i2s1_out_fmt16();
        break;
      default:
        enable_i2s2_out_fmt16();
        break;
    }
}

void cxd56_audio_bca_reg_get_dma_mstate(cxd56_audio_dma_t handle,
                                        FAR cxd56_audio_dma_mstate_t *state)
{
  BCA_REG_ID reg_id_start;
  BCA_REG_ID reg_id_error;
  BCA_REG_ID reg_id_monbuf;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id_start  = BCA_MIC_IN_START;
        reg_id_error  = BCA_MIC_IN_ERROR_SETTING;
        reg_id_monbuf = BCA_MIC_IN_MONBUF;
        break;
      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id_start  = BCA_I2S1_OUT_MON_START;
        reg_id_error  = BCA_I2S1_OUT_MON_ERROR_SETTING;
        reg_id_monbuf = BCA_I2S1_OUT_MON_MONBUF;
        break;
      default:
        reg_id_start  = BCA_I2S2_OUT_MON_START;
        reg_id_error  = BCA_I2S2_OUT_MON_ERROR_SETTING;
        reg_id_monbuf = BCA_I2S2_OUT_MON_MONBUF;
        break;
    }

  state->start       = (uint8_t)read_bca_reg(reg_id_start);
  state->err_setting = (uint8_t)read_bca_reg(reg_id_error);
  state->buf_state   = (uint8_t)read_bca_reg(reg_id_monbuf);
}

void cxd56_audio_bca_reg_en_bus_err_int(void)
{
  write_bca_reg(BCA_INT_M_HRESP_ERR, 0);
}

void cxd56_audio_bca_reg_dis_bus_err_int(void)
{
  write_bca_reg(BCA_INT_M_HRESP_ERR, 1);
}

uint32_t cxd56_audio_bca_reg_get_dma_done_state_mic(void)
{
  uint32_t int_ac =
    read32_bca_reg(g_bca_reg[BCA_MIC_INT_CTRL_DONE_MIC].addr)
    & ~(read32_bca_reg(g_bca_reg[BCA_MIC_INT_MASK_DONE_MIC].addr))
    & 0x0f;

  return int_ac;
}

uint32_t cxd56_audio_bca_reg_get_dma_done_state_i2s1(void)
{
  uint32_t int_i2s =
    read32_bca_reg(g_bca_reg[BCA_I2S1_INT_CTRL_DONE_I2SO].addr)
    & ~(read32_bca_reg(g_bca_reg[BCA_I2S1_INT_MASK_DONE_I2SO].addr))
    & 0x3f;

  return int_i2s;
}

uint32_t cxd56_audio_bca_reg_get_dma_done_state_i2s2(void)
{
  uint32_t int_i2s2 =
    read32_bca_reg(g_bca_reg[BCA_I2S2_INT_CTRL_DONE_I2SO].addr)
    & ~(read32_bca_reg(g_bca_reg[BCA_I2S2_INT_MASK_DONE_I2SO].addr))
    & 0x3f;

  return int_i2s2;
}

void cxd56_audio_bca_reg_clear_dma_done_state_mic(uint32_t value)
{
  write32_bca_reg(g_bca_reg[BCA_MIC_INT_CTRL_DONE_MIC].addr, value);
}

void cxd56_audio_bca_reg_clear_dma_done_state_i2s1(uint32_t value)
{
  write32_bca_reg(g_bca_reg[BCA_I2S1_INT_CTRL_DONE_I2SO].addr, value);
}

void cxd56_audio_bca_reg_clear_dma_done_state_i2s2(uint32_t value)
{
  write32_bca_reg(g_bca_reg[BCA_I2S2_INT_CTRL_DONE_I2SO].addr, value);
}

void cxd56_audio_bca_reg_mask_done_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_DONE_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_DONE_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_MASK_DONE_I2SO;
        break;
    }

  write_bca_reg(reg_id, 1);
}

void cxd56_audio_bca_reg_unmask_done_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_DONE_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_DONE_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_MASK_DONE_I2SO;
        break;
    }

  write_bca_reg(reg_id, 0);
}

void cxd56_audio_bca_reg_clear_done_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_CTRL_DONE_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_CTRL_DONE_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_CTRL_DONE_I2SO;
        break;
    }

  write_bca_reg_mask(reg_id);
}

bool cxd56_audio_bca_reg_is_dma_fifo_empty(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_I2S1_IN_RTD_TRG;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_IN_RTD_TRG;
        break;

      default:
        reg_id = BCA_I2S2_OUT_RTD_TRG;
        break;
    }

  if (1 == read_bca_reg(reg_id))
    {
      return true;
    }

  return false;
}

void cxd56_audio_bca_reg_mask_err_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_ERR_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_ERR_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_MASK_ERR_I2SO;
        break;
    }

  write_bca_reg(reg_id, 1);
}

void cxd56_audio_bca_reg_unmask_err_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_ERR_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_ERR_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_MASK_ERR_I2SO;
        break;
    }

  write_bca_reg(reg_id, 0);
}

void cxd56_audio_bca_reg_clear_err_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_CTRL_ERR_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_CTRL_ERR_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_MASK_ERR_I2SO;
        break;
    }

  write_bca_reg_mask(reg_id);
}

void cxd56_audio_bca_reg_mask_cmb_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_CMB_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_CMB_I2S;
        break;

      default:
        reg_id = BCA_I2S2_INT_MASK_CMB_I2S;
        break;
    }

  write_bca_reg(reg_id, 1);
}

void cxd56_audio_bca_reg_unmask_cmb_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_CMB_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_CMB_I2S;
        break;

      default:
        reg_id = BCA_I2S2_INT_MASK_CMB_I2S;
        break;
    }

  write_bca_reg(reg_id, 0);
}

void cxd56_audio_bca_reg_clear_cmb_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_CTRL_CMB_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_CTRL_CMB_I2S;
        break;

      default:
        reg_id = BCA_I2S2_INT_CTRL_CMB_I2S;
        break;
    }

  write_bca_reg_mask(reg_id);
}

uint32_t cxd56_audio_bca_reg_get_int_status(void)
{
  return read32_bca_reg(g_bca_reg[BCA_INT_HRESP_ERR].addr);
}

void cxd56_audio_bca_reg_clear_int_status(uint32_t int_au)
{
  write32_bca_reg(g_bca_reg[BCA_INT_CLR_HRESP_ERR].addr, int_au);
}

void cxd56_audio_bca_reg_mask_bus_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = AHB_MASTER_MIC_MASK;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = AHB_MASTER_I2S1_MASK;
        break;

      default:
        reg_id = AHB_MASTER_I2S2_MASK;
        break;
    }

  write_bca_reg(reg_id, 0);
}

void cxd56_audio_bca_reg_unmask_bus_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  uint32_t   val = 0;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = AHB_MASTER_MIC_MASK;
        val    = 0x00000303;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = AHB_MASTER_I2S1_MASK;
        val    = 0x00000202;
        break;

      default:
        reg_id = AHB_MASTER_I2S2_MASK;
        val    = 0x00000202;
        break;
    }

  write_bca_reg(reg_id, val);
}

void cxd56_audio_bca_reg_set_start_addr(cxd56_audio_dma_t handle,
                                        uint32_t addr)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_START_ADR;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_START_ADR;
        break;

      default:
        reg_id = BCA_I2S2_OUT_START_ADR;
        break;
    }

  write_bca_reg(reg_id, addr >> 2);
}

void cxd56_audio_bca_reg_set_sample_no(cxd56_audio_dma_t handle,
                                       uint32_t sample)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_SAMPLE_NO;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_SAMPLE_NO;
        break;

      default:
        reg_id = BCA_I2S2_OUT_SAMPLE_NO;
        break;
    }

  write_bca_reg(reg_id, sample - 1);
}

void cxd56_audio_bca_reg_start_dma(cxd56_audio_dma_t handle,
                                   bool nointr)
{
  BCA_REG_ID reg_id;
  uint32_t val = nointr ? 0x05 : 0x01;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_RTD_TRG;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_RTD_TRG;
        break;

      default:
        reg_id = BCA_I2S2_OUT_RTD_TRG;
        break;
    }

  write_bca_reg(reg_id, val);
}

void cxd56_audio_bca_reg_stop_dma(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_RTD_TRG;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_RTD_TRG;
        break;

      default:
        reg_id = BCA_I2S2_OUT_RTD_TRG;
        break;
    }

  write_bca_reg(reg_id, 0x04);
}

bool cxd56_audio_bca_reg_is_done_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_CTRL_DONE_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_CTRL_DONE_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_CTRL_DONE_I2SO;
        break;
    }

  if (read_bca_reg(reg_id) == 0)
    {
      return false;
    }

  return true;
}

bool cxd56_audio_bca_reg_is_err_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_CTRL_ERR_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_CTRL_ERR_I2SO;
        break;

      default:
        reg_id = BCA_I2S2_INT_CTRL_ERR_I2SO;
        break;
    }

  if (read_bca_reg(reg_id) == 0)
    {
      return false;
    }

  return true;
}

bool cxd56_audio_bca_reg_is_smp_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;

  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_CTRL_SMP_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_CTRL_SMP_I2S;
        break;

      default:
        reg_id = BCA_I2S2_INT_CTRL_SMP_I2S;
        break;
    }

  if (read_bca_reg(reg_id) == 0)
    {
      return false;
    }

  return true;
}

void cxd56_audio_bca_reg_mask_smp_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_SMP_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_SMP_I2S;
        break;

      default:
        reg_id = BCA_I2S1_INT_MASK_SMP_I2S;
        break;
    }

  write_bca_reg(reg_id, 1);
}

void cxd56_audio_bca_reg_unmask_smp_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_MASK_SMP_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_MASK_SMP_I2S;
        break;

      default:
        reg_id = BCA_I2S1_INT_MASK_SMP_I2S;
        break;
    }

  write_bca_reg(reg_id, 0);
}

void cxd56_audio_bca_reg_clear_smp_int(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_INT_CTRL_SMP_MIC;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_INT_CTRL_SMP_I2S;
        break;

      default:
        reg_id = BCA_I2S2_INT_CTRL_SMP_I2S;
        break;
    }

  write_bca_reg_mask(reg_id);
}

uint32_t cxd56_audio_bca_reg_get_mon_state_err(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_ERROR_SETTING;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_MON_ERROR_SETTING;
        break;

      default:
        reg_id = BCA_I2S2_OUT_MON_ERROR_SETTING;
        break;
    }

  return read_bca_reg(reg_id);
}

uint32_t cxd56_audio_bca_reg_get_mon_state_start(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_START;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_MON_START;
        break;

      default:
        reg_id = BCA_I2S2_OUT_MON_START;
        break;
    }

  return read_bca_reg(reg_id);
}

uint32_t cxd56_audio_bca_reg_get_mon_state_buf(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_MONBUF;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_MON_MONBUF;
        break;

      default:
        reg_id = BCA_I2S2_OUT_MON_MONBUF;
        break;
    }

  return read_bca_reg(reg_id);
}

uint32_t cxd56_audio_bca_reg_get_dma_state(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_RTD_TRG;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_RTD_TRG;
        break;

      default:
        reg_id = BCA_I2S2_OUT_RTD_TRG;
        break;
    }

  return read_bca_reg(reg_id);
}

void cxd56_audio_bca_reg_reset_chsel(cxd56_audio_dma_t handle)
{
  BCA_REG_ID reg_id;
  uint32_t   chsel;
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        reg_id = BCA_MIC_IN_CH8_SEL;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        reg_id = BCA_I2S1_OUT_SD1_R_SEL;
        break;

      default:
        reg_id = BCA_I2S2_OUT_SD1_R_SEL;
        break;
    }

  chsel = read32_bca_reg(g_bca_reg[reg_id].addr);

  /* Clear ChSel. */

  write32_bca_reg(g_bca_reg[reg_id].addr, 0xffffffff);

  /* Set ChSel. */

  write32_bca_reg(g_bca_reg[reg_id].addr, chsel);
}
