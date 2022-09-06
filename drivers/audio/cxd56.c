/****************************************************************************
 * drivers/audio/cxd56.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>

#include <arch/board/cxd56_clock.h>
#include <arch/board/board.h>
#include <arch/chip/audio.h>
#include <arch/chip/chip.h>

#include "cxd56.h"

#ifdef CONFIG_AUDIO_CXD56_SRC
#include "cxd56_src.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_BASE     0x0e300000
#define REG_BASE_INT 0xe0045000

#define CXD56_GEN_MASK(len, pos) (((len) == 32) ? 0xffffffff : \
                                  ((1 << (len)) - 1) << (pos))

#define CXD56_IRQ1_BIT_MIC  (1 << 6)   /* AU0 */
#define CXD56_IRQ1_BIT_I2S1 (1 << 7)   /* AU1 */

#define CXD56_VOL_MIN            (-1020)
#define CXD56_VOL_MAX            (-30)
#define CXD56_VOL_MUTE           (CXD56_VOL_MIN - 1)
#define CXD56_VOL_RANGE          ((CXD56_VOL_MAX - CXD56_VOL_MIN) / 2)
#define CXD56_VOL_NX_TO_CXD56(v) ((int)((float)((v) / 1000.0) * CXD56_VOL_RANGE) \
                                 + CXD56_VOL_MIN + CXD56_VOL_RANGE)
#define CXD56_VOL_WAIT_TIME      20
#define CXD56_VOL_TO_REG(vol)    (((vol) / 5) & 0xff)
#define CXD56_VOL_MUTE_REG       0x33
#define CXD56_VOL_MUTE_TIME(vol, cycles) \
                                 (((CXD56_VOL_TO_REG(vol) - CXD56_VOL_MUTE_REG) & 0xff) \
                                 * ((cycles) + 1) * 4 / 48 * 1000)

#define CXD56_IN_CHANNELS_MAX  8
#define CXD56_OUT_CHANNELS_MAX 2

/* Samplerates field is split into low and high byte */

#ifdef CONFIG_AUDIO_CXD56_SRC
#define CXD56_SUPP_RATES_L  (AUDIO_SAMP_RATE_8K  | AUDIO_SAMP_RATE_11K | \
                             AUDIO_SAMP_RATE_16K | AUDIO_SAMP_RATE_22K | \
                             AUDIO_SAMP_RATE_32K | AUDIO_SAMP_RATE_44K | \
                             AUDIO_SAMP_RATE_48K)
#define CXD56_SUPP_RATES_H  ((AUDIO_SAMP_RATE_96K  | AUDIO_SAMP_RATE_128K | \
                              AUDIO_SAMP_RATE_192K) >> 8)
#define CXD56_SUPP_RATES    (CXD56_SUPP_RATES_L | CXD56_SUPP_RATES_H)
#else
/* No sample rate converter, only support system rate of 48kHz */
#define CXD56_SUPP_RATES_L  AUDIO_SAMP_RATE_48K
#define CXD56_SUPP_RATES_H  0x0
#define CXD56_SUPP_RATES    (CXD56_SUPP_RATES_L | CXD56_SUPP_RATES_H)
#endif

/* Mic setting definitions */

#define CXD56_ACA_MIC_AMIC       1  /* Analog MIC */
#define CXD56_MIC_TRANS_CH_24BIT 8
#define CXD56_MIC_TRANS_CH_16BIT 4

#define CXD56_AUDIO_CFG_MIC_MODE_64FS  0
#define CXD56_AUDIO_CFG_MIC_MODE_128FS 1

#define CXD56_MIC_GAIN_MAX     150
#define CXD56_MIC_PGA_GAIN_MAX 60
#define CXD56_MIC_CH_BITNUM    4
#define CXD56_MIC_CH_BITMAP    0xf
#define CXD56_CIC_MIC_CH_NUM   2

/* External XTAL */

#define CXD56_AUD_MCLK_EXT (0u<<16)

/* Oscillator modes */

#define CXD56_ACA_OSC_24_576MHZ       1  /* 24.576MHz */
#define CXD56_ACA_OSC_24_576MHZ_HIRES 2  /* 24.576MHz, Hi-Res */
#define CXD56_ACA_OSC_49_152MHZ       3  /* 49.152MHz */
#define CXD56_ACA_OSC_49_152MHZ_HIRES 4  /* 49.152MHz, Hi-Res */

/* Control IDs for external fw_as_acacontrol */
#define CXD56_ACA_CTL_CHECK_ID          0
#define CXD56_ACA_CTL_POWER_ON_COMMON   1
#define CXD56_ACA_CTL_POWER_ON_INPUT    2
#define CXD56_ACA_CTL_POWER_ON_OUTPUT   3
#define CXD56_ACA_CTL_SET_SERDES        4
#define CXD56_ACA_CTL_SET_SMASTER       5
#define CXD56_ACA_CTL_POWER_OFF_COMMON  6
#define CXD56_ACA_CTL_POWER_OFF_INPUT   7
#define CXD56_ACA_CTL_POWER_OFF_OUTPUT  8
#define CXD56_ACA_CTL_POWER_ON_MICBIAS  9
#define CXD56_ACA_CTL_POWER_OFF_MICBIAS 10
#define CXD56_ACA_CTL_SET_OUTPUT_DEVICE 13

#define CXD56_EXP_REVID    0x20
#define CXD56_EXP_DEVICEID 0x02

#define CXD56_OUT_DEV_OFF (0)
#define CXD56_OUT_DEV_SP  (1)

#define CXD56_SMSTR_MODE_FS_16         1
#define CXD56_SMSTR_MODE_FS_32         2
#define CXD56_SMSTR_MCK_FS_512         1
#define CXD56_SMSTR_MCK_FS_1024        2
#define CXD56_SMSTR_CHSEL_NORMAL       1
#define CXD56_SMSTR_PWMMD_BOTH         2
#define CXD56_ACA_OUT_OFF              6
#define CXD56_ACA_PWMOUT_UNKNOWN       0
#define CXD56_ACA_SP_DELAY_SEL_UNKNOWN 0
#define CXD56_ACA_SP_LOOP_MODE_UNKNOWN 0
#define CXD56_ACA_SP_DLY_FREE_UNKNOWN  0

#define CXD56_AUDAT_SEL_MIC1   0
#define CXD56_AUDAT_SEL_MIC2   1
#define CXD56_AUDAT_SEL_MIC3   2
#define CXD56_AUDAT_SEL_MIC4   3
#define CXD56_AUDAT_SEL_BUSIF1 4
#define CXD56_AUDAT_SEL_BUSIF2 4

#define CXD56_DMA_MSTATE_ERR_NO_ENABLE_CH  1
#define CXD56_DMA_MSTATE_ERR_CH1_4_INVALID 2
#define CXD56_DMA_MSTATE_ERR_CH5_8_INVALID 3
#define CXD56_DMA_MSTATE_BUF_EMPTY         3

#define CXD56_DMA_TIMEOUT           10000
#define CXD56_DMA_START_RETRY_CNT   10
#define CXD56_DMA_SMP_WAIT_HIRES    10 /* usec per sample. */
#define CXD56_DMA_SMP_WAIT_NORMALT  40 /* usec per sample. */
#define CXD56_DMA_CMD_FIFO_NOT_FULL 1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Located in arch/arm/src/cxd56xx/cxd56_clock.c */

extern void cxd56_audio_clock_enable(uint32_t clk, uint32_t div);
extern void cxd56_audio_clock_disable(void);
extern bool cxd56_audio_clock_is_enabled(void);

/* Located in arch/arm/src/cxd56xx/cxd56_farapistub.S */

extern uint32_t fw_as_acacontrol(uint8_t type, uint32_t param);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CXD56 Audio register value definition */

struct cxd56_aureg_s
{
  uint32_t addr;
  uint8_t  pos;
  uint8_t  len;
};
typedef struct cxd56_aureg_s cxd56_aureg_t;

/* DMA interrupt types */

enum cxd56_dma_int_e
{
  CXD56_DMA_INT_DONE = 0x01,
  CXD56_DMA_INT_ERR  = 0x02,
  CXD56_DMA_INT_SMP  = 0x10,
  CXD56_DMA_INT_CMB  = 0x20
};

/* Volume setting IDs */

enum cxd56_vol_id_e
{
  CXD56_VOL_ID_MIXER_IN1,  /* SDIN1_VOL */
  CXD56_VOL_ID_MIXER_IN2,  /* SDIN2_VOL */
  CXD56_VOL_ID_MIXER_OUT   /* DAC_VOL */
};

enum cxd56_pulco_ser_mode_id_e
{
  CXD56_SER_MODE_UNKNOWN,
  CXD56_SER_MODE_8CH,         /* 8ch */
  CXD56_SER_MODE_4CH,         /* 4ch */
  CXD56_SER_MODE_MAX_ENTRY
};

enum csd56_pulco_ser_fs_id_e
{
  CXD56_SER_FS_UNKNOWN,
  CXD56_SER_FS_128,           /* 128fs */
  CXD56_SER_FS_64,            /* 64fs */
  CXD56_SER_FS_MAX_ENTRY
};

enum cxd56_pulco_ser_sel_ch_id_e
{
  CXD56_SER_SEL_FIX0 = 0,
  CXD56_SER_SEL_AMIC1 = 1,
  CXD56_SER_SEL_AMIC2 = 2,
  CXD56_SER_SEL_AMIC3 = 3,
  CXD56_SER_SEL_AMIC4 = 4,
  CXD56_SER_SEL_DMIC1 = 5,
  CXD56_SER_SEL_DMIC2 = 6,
  CXD56_SER_SEL_DMIC3 = 7,
  CXD56_SER_SEL_DMIC4 = 8,
  CXD56_SER_SEL_DMIC5 = 9,
  CXD56_SER_SEL_DMIC6 = 10,
  CXD56_SER_SEL_DMIC7 = 11,
  CXD56_SER_SEL_DMIC8 = 12,
  CXD56_SER_SEL_UNKNOWN = 15,
  CXD56_SER_SEL_MAX_ENTRY = 16
};

enum cxd56_sdes_des_sel_out_id_e
{
  CXD56_SDES_DES_SEL_UNKNOWN,
  CXD56_SDES_DES_SEL_CH1,
  CXD56_SDES_DES_SEL_CH2,
  CXD56_SDES_DES_SEL_CH3,
  CXD56_SDES_DES_SEL_CH4,
  CXD56_SDES_DES_SEL_CH5,
  CXD56_SDES_DES_SEL_CH6,
  CXD56_SDES_DES_SEL_CH7,
  CXD56_SDES_DES_SEL_CH8,
  CXD56_SDES_DES_SEL_MAX_ENTRY
};

struct cxd56_ser_des_param_s
{
  enum cxd56_pulco_ser_mode_id_e ser_mode;
  enum csd56_pulco_ser_fs_id_e   ser_fs;
  union
  {
    enum cxd56_pulco_ser_sel_ch_id_e in[CXD56_IN_CHANNELS_MAX];
    enum cxd56_sdes_des_sel_out_id_e out[CXD56_IN_CHANNELS_MAX];
  } sel_ch;
};

enum cxd56_mic_type_e
{
  CXD56_AUDIO_CFG_MIC_DEV_NONE = 0,
  CXD56_AUDIO_CFG_MIC_DEV_ANALOG,
  CXD56_AUDIO_CFG_MIC_DEV_DIGITAL,
  CXD56_AUDIO_CFG_MIC_DEV_ANADIG
};

#if 0
/* TODO: Implement mic gain handling */

struct cxd56_audio_mic_gain_s
{
  int32_t gain[CXD56_IN_CHANNELS_MAX];
};
#endif

struct cxd56_aca_pwinput_param_s
{
  enum cxd56_mic_type_e mic_dev;
  uint8_t               mic_bias_sel;
  uint32_t              mic_gain[4];
  uint32_t              pga_gain[4];
  int32_t               vgain[4];
};

struct cxd56_aca_pwon_param_s
{
  uint8_t osc_mode;
  uint8_t mic_dev;
  uint8_t gpo_ds;
  uint8_t ad_data_ds;
  uint8_t dmic_clk_ds;
  uint8_t mclk_ds;
};

struct cxd56_aca_smaster_param_s
{
  uint8_t mode;
  uint8_t mck_fs;
  uint8_t pwm_mode;
  uint8_t ch_sel;
  uint8_t out2dly;
};

struct cxd56_aca_pwoutput_param_s
{
  uint8_t out_dev;
  uint8_t pwm_out[2];
  uint8_t sp_delay;
  uint8_t loop_mode;
  uint8_t mode;
  uint8_t sp_dly_free;
  uint8_t sp_spliton;
  uint8_t sp_drv;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interface functions */

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int     cxd56_configure(FAR struct audio_lowerhalf_s *lower,
                 FAR void *session, FAR const struct audio_caps_s *caps);
static int     cxd56_start(FAR struct audio_lowerhalf_s *lower,
                 FAR void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int     cxd56_stop(FAR struct audio_lowerhalf_s *lower,
                 FAR void *session);
#endif  /* CONFIG_AUDIO_EXCLUDE_STOP */
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int     cxd56_pause(FAR struct audio_lowerhalf_s *lower,
                 FAR void *session);
static int     cxd56_resume(FAR struct audio_lowerhalf_s *lower,
                 FAR void *session);
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
static int     cxd56_reserve(FAR struct audio_lowerhalf_s *lower,
                 FAR void** session);
static int     cxd56_release(FAR struct audio_lowerhalf_s *lower,
                 FAR void *session);
#else

static int     cxd56_configure(FAR struct audio_lowerhalf_s *lower,
                 FAR const struct audio_caps_s *caps);
static int     cxd56_start(FAR struct audio_lowerhalf_s *lower);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int     cxd56_stop(FAR struct audio_lowerhalf_s *lower);
#endif  /* CONFIG_AUDIO_EXCLUDE_STOP */
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int     cxd56_pause(FAR struct audio_lowerhalf_s *lower);
static int     cxd56_resume(FAR struct audio_lowerhalf_s *lower);
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
static int     cxd56_reserve(FAR struct audio_lowerhalf_s *lower);
static int     cxd56_release(FAR struct audio_lowerhalf_s *lower);
#endif /* CONFIG_AUDIO_MULTI_SESSION */

static int     cxd56_getcaps(FAR struct audio_lowerhalf_s *lower, int type,
                 FAR struct audio_caps_s *caps);
static int     cxd56_shutdown(FAR struct audio_lowerhalf_s *lower);
static int     cxd56_enqueuebuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb);
static int     cxd56_cancelbuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb);
static int     cxd56_ioctl(FAR struct audio_lowerhalf_s *lower, int cmd,
                 unsigned long arg);

/* Non-interface functions */

static int cxd56_attach_irq(bool attach);
static void cxd56_enable_irq(bool enable);
static uint32_t cxd56_get_i2s_rate(uint32_t samplerate);
static void cxd56_get_mic_config(uint8_t *count,
                                 uint8_t *dev,
                                 uint8_t *mode);
static void cxd56_init_dma(FAR struct cxd56_dev_s *dev);
static void cxd56_init_i2s1_output(uint8_t bits);
static void cxd56_init_mic_input(uint8_t mic_num, uint8_t bits);
static int cxd56_init_worker(FAR struct audio_lowerhalf_s *lower);
static uint8_t cxd56_get_mic_mode(void);
static int cxd56_power_off(FAR struct cxd56_dev_s *dev);
static int cxd56_power_on(FAR struct cxd56_dev_s *dev);
static int cxd56_power_on_aca(uint32_t samplerate);
static int cxd56_power_on_analog_output(FAR struct cxd56_dev_s *dev);
static void cxd56_audio_power_on_cic(uint8_t mic_in,
                                    uint8_t mic_mode,
                                    uint8_t cic_num,
                                    FAR struct cxd56_audio_mic_gain_s *gain);
static int cxd56_power_on_decim(uint8_t mic_mode, uint16_t samplerate);
static void cxd56_power_on_i2s1(FAR struct cxd56_dev_s *dev);
static int cxd56_power_on_input(FAR struct cxd56_dev_s *dev);
static int cxd56_power_on_micbias(FAR struct cxd56_dev_s *dev);
static int cxd56_start_dma(FAR struct cxd56_dev_s *dev);
static int cxd56_stop_dma(FAR struct cxd56_dev_s *priv);
static void cxd56_set_dma_int_en(bool enabled);
static void cxd56_set_dma_running(cxd56_dmahandle_t handle, bool running);
static void cxd56_set_mic_gains(uint8_t gain,
                                struct cxd56_aca_pwinput_param_s *param);
static void cxd56_set_mic_out_channel(FAR struct cxd56_dev_s *dev);
static int cxd56_set_volume(enum cxd56_vol_id_e id, int16_t vol);
static void cxd56_swap_buffer_rl(uint32_t addr, uint16_t size);
static void *cxd56_workerthread(pthread_addr_t pvarg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_dev_s *g_dev[CXD56_AUDIO_DMA_COUNT] =
{
  NULL
};

static uint16_t g_codec_start_count = 0;

static const struct audio_ops_s g_audioops =
{
  cxd56_getcaps,       /* getcaps        */
  cxd56_configure,     /* configure      */
  cxd56_shutdown,      /* shutdown       */
  cxd56_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  cxd56_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  cxd56_pause,         /* pause          */
  cxd56_resume,        /* resume         */
#endif
  NULL,                /* alloc_buffer   */
  NULL,                /* free_buffer    */
  cxd56_enqueuebuffer, /* enqueue_buffer */
  cxd56_cancelbuffer,  /* cancel_buffer  */
  cxd56_ioctl,         /* ioctl          */
  NULL,                /* read           */
  NULL,                /* write          */
  cxd56_reserve,       /* reserve        */
  cxd56_release        /* release        */
};

/* AC registers */

const cxd56_aureg_t  REG_AC_REVID =
{
  REG_BASE + 0x0000, 16,  8
};
const cxd56_aureg_t  REG_AC_DEVICEID =
{
  REG_BASE + 0x0000, 24,  8
};
const cxd56_aureg_t  REG_AC_PDN_AMICEXT =
{
  REG_BASE + 0x0100, 4,  1
};
const cxd56_aureg_t  REG_AC_PDN_AMIC1 =
{
  REG_BASE + 0x0100, 5,  1
};
const cxd56_aureg_t  REG_AC_PDN_AMIC2 =
{
  REG_BASE + 0x0100, 6,  1
};
const cxd56_aureg_t  REG_AC_PDN_DMIC =
{
  REG_BASE + 0x0100, 15,  1
};
const cxd56_aureg_t  REG_AC_PDN_DSPB =
{
  REG_BASE + 0x0100, 16,  1
};
const cxd56_aureg_t  REG_AC_PDN_ANC =
{
  REG_BASE + 0x0100, 17,  1
};
const cxd56_aureg_t  REG_AC_PDN_DNC1 =
{
  REG_BASE + 0x0100, 18,  1
};
const cxd56_aureg_t  REG_AC_PDN_DNC2 =
{
  REG_BASE + 0x0100, 19,  1
};
const cxd56_aureg_t  REG_AC_PDN_SMSTR =
{
  REG_BASE + 0x0100, 20,  1
};
const cxd56_aureg_t  REG_AC_PDN_DSPS2 =
{
  REG_BASE + 0x0100, 21,  1
};
const cxd56_aureg_t  REG_AC_PDN_DSPS1 =
{
  REG_BASE + 0x0100, 22,  1
};
const cxd56_aureg_t  REG_AC_PDN_DSPC =
{
  REG_BASE + 0x0100, 23,  1
};
const cxd56_aureg_t  REG_AC_FS_FS =
{
  REG_BASE + 0x0104,  0,  1
};
const cxd56_aureg_t  REG_AC_DECIM0_EN =
{
  REG_BASE + 0x0104, 16,  1
};
const cxd56_aureg_t  REG_AC_DECIM1_EN =
{
  REG_BASE + 0x0104, 17,  1
};
const cxd56_aureg_t  REG_AC_SDES_EN =
{
  REG_BASE + 0x0104, 18,  1
};
const cxd56_aureg_t  REG_AC_MCK_AHBMSTR_EN =
{
  REG_BASE + 0x0104, 19,  1
};
const cxd56_aureg_t  REG_AC_AU_DAT_SEL2 =
{
  REG_BASE + 0x0108, 16,  3
};
const cxd56_aureg_t  REG_AC_AU_DAT_SEL1 =
{
  REG_BASE + 0x0108, 20,  3
};
const cxd56_aureg_t  REG_AC_AU_COD_INSEL3 =
{
  REG_BASE + 0x0108, 24,  2
};
const cxd56_aureg_t  REG_AC_AU_COD_INSEL2 =
{
  REG_BASE + 0x0108, 26,  2
};
const cxd56_aureg_t  REG_AC_DSR_RATE =
{
  REG_BASE + 0x0200,  0,  3
};
const cxd56_aureg_t  REG_AC_DIGSFT =
{
  REG_BASE + 0x0200, 12,  1
};
const cxd56_aureg_t  REG_AC_SRC1 =
{
  REG_BASE + 0x0200, 16,  2
};
const cxd56_aureg_t  REG_AC_SRC1IN_SEL =
{
  REG_BASE + 0x0200, 18,  2
};
const cxd56_aureg_t  REG_AC_SRC2IN_SEL =
{
  REG_BASE + 0x0200, 22,  2
};
const cxd56_aureg_t  REG_AC_DIF1 =
{
  REG_BASE + 0x0200, 27,  1
};
const cxd56_aureg_t  REG_AC_SD1MASTER =
{
  REG_BASE + 0x0200, 29,  1
};
const cxd56_aureg_t  REG_AC_SDCK_OUTENX =
{
  REG_BASE + 0x0200, 30,  1
};
const cxd56_aureg_t  REG_AC_HPF2_MODE =
{
  REG_BASE + 0x0204, 0,  2
};
const cxd56_aureg_t  REG_AC_CIC2_GAIN_MODE =
{
  REG_BASE + 0x0204, 8,  1
};
const cxd56_aureg_t  REG_AC_CIC2IN_SEL =
{
  REG_BASE + 0x0204, 9,  1
};
const cxd56_aureg_t  REG_AC_HPF1_MODE =
{
  REG_BASE + 0x0204, 16,  2
};
const cxd56_aureg_t  REG_AC_CIC1_GAIN_MODE =
{
  REG_BASE + 0x0204, 24,  1
};
const cxd56_aureg_t  REG_AC_CIC1IN_SEL =
{
  REG_BASE + 0x0204, 25,  1
};
const cxd56_aureg_t  REG_AC_ADC_FS =
{
  REG_BASE + 0x0204, 28,  2
};
const cxd56_aureg_t  REG_AC_HPF4_MODE =
{
  REG_BASE + 0x0208, 0,  2
};
const cxd56_aureg_t  REG_AC_CIC4IN_SEL =
{
  REG_BASE + 0x0208, 9,  1
};
const cxd56_aureg_t  REG_AC_HPF3_MODE =
{
  REG_BASE + 0x0208, 16,  2
};
const cxd56_aureg_t  REG_AC_CIC3IN_SEL =
{
  REG_BASE + 0x0208, 25,  1
};
const cxd56_aureg_t  REG_AC_CIC1_RGAIN =
{
  REG_BASE + 0x020c,  0, 16
};
const cxd56_aureg_t  REG_AC_CIC1_LGAIN =
{
  REG_BASE + 0x020c, 16, 16
};
const cxd56_aureg_t  REG_AC_CIC2_RGAIN =
{
  REG_BASE + 0x0210,  0, 16
};
const cxd56_aureg_t  REG_AC_CIC2_LGAIN =
{
  REG_BASE + 0x0210, 16, 16
};
const cxd56_aureg_t  REG_AC_CIC3_RGAIN =
{
  REG_BASE + 0x0214,  0, 16
};
const cxd56_aureg_t  REG_AC_CIC3_LGAIN =
{
  REG_BASE + 0x0214, 16, 16
};
const cxd56_aureg_t  REG_AC_CIC4_RGAIN =
{
  REG_BASE + 0x0218,  0, 16
};
const cxd56_aureg_t  REG_AC_CIC4_LGAIN =
{
  REG_BASE + 0x0218, 16, 16
};
const cxd56_aureg_t  REG_AC_SPC_EN =
{
  REG_BASE + 0x021c, 15,  1
};
const cxd56_aureg_t  REG_AC_ALC_EN =
{
  REG_BASE + 0x021c, 31,  1
};
const cxd56_aureg_t  REG_AC_CS_VOL =
{
  REG_BASE + 0x0228,  0,  7
};
const cxd56_aureg_t  REG_AC_CS_SIGN =
{
  REG_BASE + 0x0228,  7,  1
};
const cxd56_aureg_t  REG_AC_SDIN2_VOL =
{
  REG_BASE + 0x0228, 16,  8
};
const cxd56_aureg_t  REG_AC_SDIN1_VOL =
{
  REG_BASE + 0x0228, 24,  8
};
const cxd56_aureg_t  REG_AC_SDIN1_EN =
{
  REG_BASE + 0x022c,  0,  1
};
const cxd56_aureg_t  REG_AC_SDIN2_EN =
{
  REG_BASE + 0x022c,  1,  1
};
const cxd56_aureg_t  REG_AC_SDOUT1_EN =
{
  REG_BASE + 0x022c,  2,  1
};
const cxd56_aureg_t  REG_AC_SDOUT2_EN =
{
  REG_BASE + 0x022c,  3,  1
};
const cxd56_aureg_t  REG_AC_BLF_EN =
{
  REG_BASE + 0x022c,  5,  1
};
const cxd56_aureg_t  REG_AC_DAC_VOL =
{
  REG_BASE + 0x022c,  8,  8
};
const cxd56_aureg_t  REG_AC_DNC2_MUTE =
{
  REG_BASE + 0x0304, 22,  1
};
const cxd56_aureg_t  REG_AC_DNC2_START =
{
  REG_BASE + 0x0304, 23,  1
};
const cxd56_aureg_t  REG_AC_DNC1_MUTE =
{
  REG_BASE + 0x0304, 30,  1
};
const cxd56_aureg_t  REG_AC_DNC1_START =
{
  REG_BASE + 0x0304, 31,  1
};
const cxd56_aureg_t  REG_AC_DCMFS_34 =
{
  REG_BASE + 0x0308, 22,  2
};
const cxd56_aureg_t  REG_AC_DCMFS =
{
  REG_BASE + 0x0308, 30,  2
};
const cxd56_aureg_t  REG_AC_NSX2 =
{
  REG_BASE + 0x0340, 30,  1
};
const cxd56_aureg_t  REG_AC_NSPMUTE =
{
  REG_BASE + 0x0340, 31,  1
};
const cxd56_aureg_t  REG_AC_NSDD =
{
  REG_BASE + 0x0344,  0, 20
};
const cxd56_aureg_t  REG_AC_TEST_OUT_SEL0 =
{
  REG_BASE + 0x0400,  6,  1
};
const cxd56_aureg_t  REG_AC_SER_MODE =
{
  REG_BASE + 0x0500,  0,  1
};
const cxd56_aureg_t  REG_AC_PDM_OUT_EN =
{
  REG_BASE + 0x0500, 16,  1
};
const cxd56_aureg_t  REG_AC_FS_CLK_EN =
{
  REG_BASE + 0x0500, 24,  1
};
const cxd56_aureg_t  REG_AC_SEL_OUT4_R =
{
  REG_BASE + 0x0504,  0,  3
};
const cxd56_aureg_t  REG_AC_SEL_OUT4_L =
{
  REG_BASE + 0x0504,  4,  3
};
const cxd56_aureg_t  REG_AC_SEL_OUT3_R =
{
  REG_BASE + 0x0504,  8,  3
};
const cxd56_aureg_t  REG_AC_SEL_OUT3_L =
{
  REG_BASE + 0x0504, 12,  3
};
const cxd56_aureg_t  REG_AC_SEL_OUT2_R =
{
  REG_BASE + 0x0504, 16,  3
};
const cxd56_aureg_t  REG_AC_SEL_OUT2_L =
{
  REG_BASE + 0x0504, 20,  3
};
const cxd56_aureg_t  REG_AC_SEL_OUT1_R =
{
  REG_BASE + 0x0504, 24,  3
};
const cxd56_aureg_t  REG_AC_SEL_OUT1_L =
{
  REG_BASE + 0x0504, 28,  3
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC1L_B =
{
  REG_BASE + 0x0580,  0,  1
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC1R_B =
{
  REG_BASE + 0x0580,  1,  1
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC2L_B =
{
  REG_BASE + 0x0580,  2,  1
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC2R_B =
{
  REG_BASE + 0x0580,  3,  1
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC1L_A =
{
  REG_BASE + 0x0580,  4,  1
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC1R_A =
{
  REG_BASE + 0x0580,  5,  1
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC2L_A =
{
  REG_BASE + 0x0580,  6,  1
};
const cxd56_aureg_t  REG_AC_OUTEN_MIC2R_A =
{
  REG_BASE + 0x0580,  7,  1
};
const cxd56_aureg_t  REG_AC_SEL_OUTF =
{
  REG_BASE + 0x0580, 16,  2
};
const cxd56_aureg_t  REG_AC_SEL_INF =
{
  REG_BASE + 0x0580, 20,  1
};
const cxd56_aureg_t  REG_AC_SEL_DECIM =
{
  REG_BASE + 0x0580, 24,  1
};
const cxd56_aureg_t  REG_AC_DEQ_EN =
{
  REG_BASE + 0x0600, 31,  1
};
const cxd56_aureg_t  REG_AC_LR_SWAP1 =
{
  REG_BASE + 0x0678,  0,  1
};

/* BCA registers */

const cxd56_aureg_t  REG_MIC_IN_START_ADR =
{
  REG_BASE + 0x1000,  0,  30
};
const cxd56_aureg_t  REG_MIC_IN_SAMPLE_NO =
{
  REG_BASE + 0x1004,  0,  32
};
const cxd56_aureg_t  REG_MIC_RTD_TRG =
{
  REG_BASE + 0x1008,  0,  3
};
const cxd56_aureg_t  REG_MIC_IN_BITWT =
{
  REG_BASE + 0x100c,  0,  1
};
const cxd56_aureg_t  REG_MIC_CH8_SEL =
{
  REG_BASE + 0x1010,  0,  4
};
const cxd56_aureg_t  REG_MIC_CH7_SEL =
{
  REG_BASE + 0x1010,  4,  4
};
const cxd56_aureg_t  REG_MIC_CH6_SEL =
{
  REG_BASE + 0x1010,  8,  4
};
const cxd56_aureg_t  REG_MIC_CH5_SEL =
{
  REG_BASE + 0x1010, 12,  4
};
const cxd56_aureg_t  REG_MIC_CH4_SEL =
{
  REG_BASE + 0x1010, 16,  4
};
const cxd56_aureg_t  REG_MIC_CH3_SEL =
{
  REG_BASE + 0x1010, 20,  4
};
const cxd56_aureg_t  REG_MIC_CH2_SEL =
{
  REG_BASE + 0x1010, 24,  4
};
const cxd56_aureg_t  REG_MIC_CH1_SEL =
{
  REG_BASE + 0x1010, 28,  4
};
const cxd56_aureg_t  REG_MIC_MON_START =
{
  REG_BASE + 0x1014,  0,  1
};
const cxd56_aureg_t  REG_MIC_MON_ERRSET =
{
  REG_BASE + 0x1014,  8,  8
};
const cxd56_aureg_t  REG_MIC_MON_MONBUF =
{
  REG_BASE + 0x1014, 16,  4
};
const cxd56_aureg_t  REG_I2S1_OUT_START_ADR =
{
  REG_BASE + 0x10c0,  0, 30
};
const cxd56_aureg_t  REG_I2S1_OUT_SAMPLE_NO =
{
  REG_BASE + 0x10c4,  0, 32
};
const cxd56_aureg_t  REG_I2S1_OUT_RTD_TRG =
{
  REG_BASE + 0x10c8,  0,  3
};
const cxd56_aureg_t  REG_I2S1_OUT_BITWT =
{
  REG_BASE + 0x10cc,  0,  1
};
const cxd56_aureg_t  REG_I2S1_OUT_SD1_R_SEL =
{
  REG_BASE + 0x10d0,  0,  2
};
const cxd56_aureg_t  REG_I2S1_OUT_SD1_L_SEL =
{
  REG_BASE + 0x10d0,  4,  2
};
const cxd56_aureg_t  REG_I2S1_OUT_MON_START =
{
  REG_BASE + 0x10d4,  0,  1
};
const cxd56_aureg_t  REG_I2S1_OUT_MON_ERRSET =
{
  REG_BASE + 0x10d4,  8,  8
};
const cxd56_aureg_t  REG_I2S1_OUT_MON_MONBUF =
{
  REG_BASE + 0x10d4, 16,  4
};
const cxd56_aureg_t  REG_I2S_ENSEL =
{
  REG_BASE + 0x1110,  0,  1
};
const cxd56_aureg_t  REG_MIC_INT_CTRL_DONE =
{
  REG_BASE + 0x1140,  0,  1
};
const cxd56_aureg_t  REG_MIC_INT_CTRL_ERR =
{
  REG_BASE + 0x1140,  1,  1
};
const cxd56_aureg_t  REG_MIC_INT_CTRL_SMP =
{
  REG_BASE + 0x1140,  2,  1
};
const cxd56_aureg_t  REG_MIC_INT_CTRL_CMB =
{
  REG_BASE + 0x1140,  3,  1
};
const cxd56_aureg_t  REG_I2S1_INT_CTRL_DONE =
{
  REG_BASE + 0x1144,  0,  1
};
const cxd56_aureg_t  REG_I2S1_INT_CTRL_ERR =
{
  REG_BASE + 0x1144,  1,  1
};
const cxd56_aureg_t  REG_I2S1_INT_CTRL_SMP =
{
  REG_BASE + 0x1144,  4,  1
};
const cxd56_aureg_t  REG_I2S1_INT_CTRL_CMB =
{
  REG_BASE + 0x1144,  5,  1
};
const cxd56_aureg_t  REG_MIC_INT_MASK_DONE =
{
  REG_BASE + 0x114c,  0,  1
};
const cxd56_aureg_t  REG_MIC_INT_MASK_ERR =
{
  REG_BASE + 0x114c,  1,  1
};
const cxd56_aureg_t  REG_MIC_INT_MASK_CMB =
{
  REG_BASE + 0x114c,  3,  1
};
const cxd56_aureg_t  REG_I2S1_INT_MASK_DONE =
{
  REG_BASE + 0x1150,  0,  1
};
const cxd56_aureg_t  REG_I2S1_INT_MASK_ERR =
{
  REG_BASE + 0x1150,  1,  1
};
const cxd56_aureg_t  REG_I2S1_INT_MASK_SMP =
{
  REG_BASE + 0x1150,  4,  1
};
const cxd56_aureg_t  REG_I2S1_INT_MASK_CMB =
{
  REG_BASE + 0x1150,  5,  1
};
const cxd56_aureg_t  REG_INT_M_I2S1_BCL_ERR1 =
{
  REG_BASE + 0x1158,  8,  1
};
const cxd56_aureg_t  REG_INT_M_I2S1_BCL_ERR2 =
{
  REG_BASE + 0x1158,  9,  1
};
const cxd56_aureg_t  REG_INT_M_OVF_SMASL =
{
  REG_BASE + 0x1158, 17,  1
};
const cxd56_aureg_t  REG_INT_M_OVF_SMASR =
{
  REG_BASE + 0x1158, 18,  1
};
const cxd56_aureg_t  REG_INT_HRESP_ERR =
{
  REG_BASE + 0x1160,  0,  1
};
const cxd56_aureg_t  REG_CLK_EN_AHBMSTR_MIC =
{
  REG_BASE + 0x11f0,  0,  1
};
const cxd56_aureg_t  REG_CLK_EN_AHBMSTR_I2S1 =
{
  REG_BASE + 0x11f0,  1,  1
};
const cxd56_aureg_t  REG_AHB_MASTER_MIC_MASK =
{
  REG_BASE + 0x1730,  0, 32
};
const cxd56_aureg_t  REG_AHB_MASTER_I2S1_MASK =
{
  REG_BASE + 0x1f30,  0, 32
};

/* Interrupt registers */

const cxd56_aureg_t  REG_INT_IRQ1 =
{
  REG_BASE_INT + 0x30 + 3 * 4, 0, 32
};
const cxd56_aureg_t  REG_INT_EN1 =
{
  REG_BASE_INT + 0x10 + 3 * 4, 0, 32
};
const cxd56_aureg_t  REG_INT_EN1_BITS =
{
  REG_BASE_INT + 0x10 + 3 * 4, 6,  4
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t read_reg_addr(const cxd56_aureg_t *reg)
{
  uint32_t mask;

  mask = CXD56_GEN_MASK(reg->len, reg->pos);
  return (*((volatile uint32_t *)reg->addr) & mask) >> reg->pos;
}

static void write_reg_addr(const cxd56_aureg_t *reg, uint32_t val)
{
  uint32_t mask;
  uint32_t curr;

  mask = CXD56_GEN_MASK(reg->len, reg->pos);
  curr = *((volatile uint32_t *)reg->addr) & ~mask;
  *((volatile uint32_t *)reg->addr) = curr | ((val << reg->pos) & mask);
}

#define read_reg(reg)         (read_reg_addr(&(reg)))
#define read_reg32(reg)       (*((volatile uint32_t *)(reg).addr))
#define write_reg(reg, val)   (write_reg_addr(&(reg), (val)))
#define write_reg32(reg, val) (*((volatile uint32_t *)(reg).addr) = (val))

static void cxd56_int_clear(cxd56_dmahandle_t handle, uint8_t intbits)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      if (intbits > CXD56_DMA_INT_ERR)
        {
          intbits = (intbits & 0x0f) | ((intbits & 0xf0) >> 2);
        }

      write_reg32(REG_MIC_INT_CTRL_DONE, intbits);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg32(REG_I2S1_INT_CTRL_DONE, intbits);
    }
}

static void cxd56_int_mask(cxd56_dmahandle_t handle, uint8_t intbits)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      if (intbits > CXD56_DMA_INT_ERR)
        {
          intbits = (intbits & 0x0f) | ((intbits & 0xf0) >> 2);
        }

      intbits |= read_reg32(REG_MIC_INT_MASK_DONE);
      write_reg32(REG_MIC_INT_MASK_DONE, intbits);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      intbits |= read_reg32(REG_I2S1_INT_MASK_DONE);
      write_reg32(REG_I2S1_INT_MASK_DONE, intbits);
    }
}

static void cxd56_int_unmask(cxd56_dmahandle_t handle, uint8_t intbits)
{
  uint32_t curr;

  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      if (intbits > CXD56_DMA_INT_ERR)
        {
          intbits = (intbits & 0x0f) | ((intbits & 0xf0) >> 2);
        }

      curr = read_reg32(REG_MIC_INT_MASK_DONE);
      write_reg32(REG_MIC_INT_MASK_DONE, curr & ~intbits);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      curr = read_reg32(REG_I2S1_INT_MASK_DONE);
      write_reg32(REG_I2S1_INT_MASK_DONE, curr & ~intbits);
    }
}

static void cxd56_int_unmask_ahb(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      write_reg32(REG_AHB_MASTER_MIC_MASK, 0x00000303);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg32(REG_AHB_MASTER_I2S1_MASK, 0x00000202);
    }
}

static uint32_t cxd56_int_get_state(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      return (0x3f & read_reg32(REG_MIC_INT_CTRL_DONE)
              & ~(read_reg32(REG_MIC_INT_MASK_DONE)));
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return (0x3f & read_reg32(REG_I2S1_INT_CTRL_DONE)
              & ~(read_reg32(REG_I2S1_INT_MASK_DONE)));
    }

  return 0;
}

static uint32_t cxd56_int_has_error(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      return read_reg(REG_MIC_INT_CTRL_ERR);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return read_reg(REG_I2S1_INT_CTRL_ERR);
    }

  return 0;
}

static uint32_t cxd56_int_is_done(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      return read_reg(REG_MIC_INT_CTRL_DONE);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return read_reg(REG_I2S1_INT_CTRL_DONE);
    }

  return 0;
}

static uint32_t cxd56_int_has_smp(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      return read_reg(REG_MIC_INT_CTRL_SMP);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return read_reg(REG_I2S1_INT_CTRL_SMP);
    }

  return 0;
}

static uint8_t cxd56_get_mon_buf(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      return read_reg(REG_MIC_MON_MONBUF);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return read_reg(REG_I2S1_OUT_MON_MONBUF);
    }

  return 0;
}

static uint8_t cxd56_get_mon_err(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      return read_reg(REG_MIC_MON_ERRSET);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return read_reg(REG_I2S1_OUT_MON_ERRSET);
    }

  return 0;
}

static uint8_t cxd56_dma_is_busy(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      return read_reg(REG_MIC_RTD_TRG) != CXD56_DMA_CMD_FIFO_NOT_FULL;
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return read_reg(REG_I2S1_OUT_RTD_TRG) != CXD56_DMA_CMD_FIFO_NOT_FULL;
    }

  return 0;
}

static void cxd56_reset_channel_sel(cxd56_dmahandle_t handle)
{
  uint32_t sel;

  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      sel = read_reg32(REG_MIC_CH8_SEL);
      write_reg32(REG_MIC_CH8_SEL, 0xffffffff);
      write_reg32(REG_MIC_CH8_SEL, sel);
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      sel = read_reg32(REG_I2S1_OUT_SD1_R_SEL);
      write_reg32(REG_I2S1_OUT_SD1_R_SEL, 0xffffffff);
      write_reg32(REG_I2S1_OUT_SD1_R_SEL, sel);
    }
}

#ifdef CONFIG_AUDIO_CXD56_SRC
static void _process_audio_with_src(cxd56_dmahandle_t hdl, uint16_t err_code)
{
  struct audio_msg_s msg;
  struct cxd56_dev_s *dev;
  irqstate_t flags;
  bool request_buffer = true;
  int ret;

  dev = g_dev[hdl];

  /* Trigger new DMA job */

  flags = spin_lock_irqsave(&dev->lock);

  if (err_code == CXD56_AUDIO_ECODE_DMA_TRANS)
    {
      /* Notify end of data */

      if (dev->state != CXD56_DEV_STATE_PAUSED &&
          dev->state != CXD56_DEV_STATE_STOPPED &&
          dq_count(&dev->down_pendq) == 0)
        {
          msg.msg_id = AUDIO_MSG_STOP;
          msg.u.data = 0;
          spin_unlock_irqrestore(&dev->lock, flags);
          ret = file_mq_send(&dev->mq, (FAR const char *)&msg,
                             sizeof(msg), CONFIG_CXD56_MSG_PRIO);
          flags = spin_lock_irqsave(&dev->lock);
          if (ret != OK)
            {
              auderr("ERROR: file_mq_send to stop failed (%d)\n", ret);
            }
        }
    }

  if (dq_count(&dev->down_runq) > 0)
    {
      FAR struct ap_buffer_s *src_apb;

      src_apb = (struct ap_buffer_s *) dq_get(&dev->down_runq);
      src_apb->nbytes = 0;
      dq_put(&dev->down_doneq, &src_apb->dq_entry);

      if (src_apb->flags & AUDIO_APB_SRC_FINAL)
        {
          struct ap_buffer_s *apb;

          apb = (struct ap_buffer_s *) dq_get(&dev->up_runq);
          spin_unlock_irqrestore(&dev->lock, flags);
          dev->dev.upper(dev->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
          flags = spin_lock_irqsave(&dev->lock);

          /* End of data? */

          if ((apb->flags & AUDIO_APB_FINAL) != 0)
            {
              msg.msg_id = AUDIO_MSG_STOP;
              msg.u.data = 0;
              spin_unlock_irqrestore(&dev->lock, flags);
              ret = file_mq_send(&dev->mq, (FAR const char *)&msg,
                                 sizeof(msg), CONFIG_CXD56_MSG_PRIO);
              flags = spin_lock_irqsave(&dev->lock);
              if (ret != OK)
                {
                  auderr("ERROR: file_mq_send to stop failed (%d)\n", ret);
                }

              request_buffer = false;
            }
        }
    }

  if (request_buffer && dev->mq.f_inode != NULL)
    {
      /* Request more data */

      msg.msg_id = AUDIO_MSG_DATA_REQUEST;
      msg.u.data = 0;
      spin_unlock_irqrestore(&dev->lock, flags);
      ret = file_mq_send(&dev->mq, (FAR const char *) &msg,
                         sizeof(msg), CONFIG_CXD56_MSG_PRIO);
      flags = spin_lock_irqsave(&dev->lock);
      if (ret != OK)
        {
          auderr("ERROR: file_mq_send to request failed (%d)\n", ret);
        }
    }

  spin_unlock_irqrestore(&dev->lock, flags);
}

#else
static void _process_audio(cxd56_dmahandle_t hdl, uint16_t err_code)
{
  struct audio_msg_s msg;
  struct cxd56_dev_s *dev;
  irqstate_t flags;
  int ret;

  dev = g_dev[hdl];

  /* Trigger new DMA job */

  flags = spin_lock_irqsave(&dev->lock);

  if (dq_count(&dev->up_runq) > 0)
    {
      FAR struct ap_buffer_s *apb;

      apb = (struct ap_buffer_s *) dq_get(&dev->up_runq);
      spin_unlock_irqrestore(&dev->lock, flags);
      dev->dev.upper(dev->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
      flags = spin_lock_irqsave(&dev->lock);
    }

  spin_unlock_irqrestore(&dev->lock, flags);

  if (err_code == CXD56_AUDIO_ECODE_DMA_TRANS)
    {
      /* Notify end of data */

      if (dev->state != CXD56_DEV_STATE_PAUSED &&
          dev->state != CXD56_DEV_STATE_STOPPED)
        {
          audinfo("DMA_TRANS up_pendq=%d\n",
                 dq_count(&dev->up_pendq));
          msg.msg_id = AUDIO_MSG_STOP;
          msg.u.data = 0;
          ret = file_mq_send(&dev->mq, (FAR const char *)&msg,
                             sizeof(msg), CONFIG_CXD56_MSG_PRIO);
          if (ret != OK)
            {
              auderr("ERROR: file_mq_send to stop failed (%d)\n", ret);
            }
        }
    }
  else if (dev->mq.f_inode != NULL)
    {
      /* Request more data */

      msg.msg_id = AUDIO_MSG_DATA_REQUEST;
      msg.u.data = 0;
      ret = file_mq_send(&dev->mq, (FAR const char *) &msg,
                         sizeof(msg), CONFIG_CXD56_MSG_PRIO);
      if (ret != OK)
        {
          auderr("ERROR: file_mq_send to request failed (%d)\n", ret);
        }
    }
}
#endif

static void cxd56_dma_int_handler(void)
{
  uint16_t err_code;
  uint32_t int_irq;
  uint32_t int_i2s;
  uint32_t int_mic;
  cxd56_dmahandle_t hdl;

  err_code = CXD56_AUDIO_ECODE_DMA_HANDLE_INV;
  int_irq = read_reg32(REG_INT_IRQ1);
  int_i2s = cxd56_int_get_state(CXD56_AUDIO_DMA_I2S0_DOWN);
  int_mic = cxd56_int_get_state(CXD56_AUDIO_DMA_MIC);

  if ((int_irq & CXD56_IRQ1_BIT_MIC) && (int_mic != 0))
    {
      hdl = CXD56_AUDIO_DMA_MIC;

      write_reg32(REG_MIC_INT_CTRL_DONE, int_mic);

      if (int_mic & (1 << REG_MIC_INT_CTRL_DONE.pos))
        {
          err_code = CXD56_AUDIO_ECODE_DMA_CMPLT;
        }

      if (int_mic & (1 << REG_MIC_INT_CTRL_ERR.pos))
        {
          /* Mask and clear transfer error interrupt */

          write_reg(REG_MIC_INT_MASK_ERR, 1);
          write_reg(REG_MIC_INT_CTRL_ERR, 1);

          err_code = CXD56_AUDIO_ECODE_DMA_TRANS;
        }

      if (int_mic & (1 << REG_MIC_INT_CTRL_CMB.pos))
        {
          /* Mask and clear bus error interrupt */

          write_reg(REG_MIC_INT_MASK_CMB, 1);
          write_reg(REG_MIC_INT_CTRL_CMB, 1);

          err_code = CXD56_AUDIO_ECODE_DMA_CMB;
        }
    }
  else if ((int_irq & CXD56_IRQ1_BIT_I2S1) && (int_i2s != 0))
    {
      hdl = CXD56_AUDIO_DMA_I2S0_DOWN;

      write_reg32(REG_I2S1_INT_CTRL_DONE, int_i2s);

      if (int_i2s & (1 << REG_I2S1_INT_CTRL_DONE.pos))
        {
          err_code = CXD56_AUDIO_ECODE_DMA_CMPLT;
        }

      if (int_i2s & (1 << REG_I2S1_INT_CTRL_ERR.pos))
        {
          /* Mask and clear transfer error interrupt */

          write_reg(REG_I2S1_INT_MASK_ERR, 1);
          write_reg(REG_I2S1_INT_CTRL_ERR, 1);

          err_code = CXD56_AUDIO_ECODE_DMA_TRANS;

          auderr("ERROR: I2S0 transfer failed.\n");
        }

      if (int_i2s & (1 << REG_I2S1_INT_CTRL_CMB.pos))
        {
          /* Mask and clear bus error interrupt */

          write_reg(REG_I2S1_INT_MASK_CMB, 1);
          write_reg(REG_I2S1_INT_CTRL_CMB, 1);

          err_code = CXD56_AUDIO_ECODE_DMA_CMB;

          auderr("ERROR: I2S0 bus error.\n");
        }
    }
  else
    {
      audinfo("Unhandled interrupt\n");

      return;
    }

  if (err_code != CXD56_AUDIO_ECODE_DMA_HANDLE_INV)
    {
#ifdef CONFIG_AUDIO_CXD56_SRC
    _process_audio_with_src(hdl, err_code);
#else
    _process_audio(hdl, err_code);
#endif
    }
}

static int cxd56_attach_irq(bool attach)
{
  int ret;
  int cur_irq;
  const int audio_irqs[4] = {
    CXD56_IRQ_AUDIO_0,
    CXD56_IRQ_AUDIO_1,
    CXD56_IRQ_AUDIO_2,
    CXD56_IRQ_AUDIO_3
  };

  cur_irq = 4;
  if (attach)
    {
      do
        {
          cur_irq--;
          ret = irq_attach(audio_irqs[cur_irq],
                           (xcpt_t)cxd56_dma_int_handler, NULL);
          if (ret != OK)
            {
              auderr("ERROR: Failed to attach handler to irq %d. (%d)\n",
                     cur_irq, ret);
              return ret;
            }
        }
      while (cur_irq > 0);
    }
  else
    {
      do
        {
          cur_irq--;
          ret = irq_detach(audio_irqs[cur_irq]);
          if (ret != OK)
            {
              auderr("ERROR: Failed to detach handler from irq %d. (%d)\n",
                     cur_irq, ret);
              return ret;
            }
        }
      while (cur_irq > 0);
    }

  return OK;
}

static void cxd56_enable_irq(bool enable)
{
  if (enable)
    {
      up_enable_irq(CXD56_IRQ_AUDIO_0);
      up_enable_irq(CXD56_IRQ_AUDIO_1);
      up_enable_irq(CXD56_IRQ_AUDIO_2);
    }
  else
    {
      up_disable_irq(CXD56_IRQ_AUDIO_0);
      up_disable_irq(CXD56_IRQ_AUDIO_1);
      up_disable_irq(CXD56_IRQ_AUDIO_2);
    }
}

static int cxd56_set_volume(enum cxd56_vol_id_e id, int16_t vol)
{
  int ret;

  if (vol == CXD56_VOL_MUTE)
    {
      vol = CXD56_VOL_MUTE_REG;
    }
  else
    {
      vol = CXD56_VOL_TO_REG(vol);
    }

  switch (id)
    {
      case CXD56_VOL_ID_MIXER_IN1:
        write_reg(REG_AC_SDIN1_VOL, vol);
        break;
      case CXD56_VOL_ID_MIXER_IN2:
        write_reg(REG_AC_SDIN2_VOL, vol);
        break;
      case CXD56_VOL_ID_MIXER_OUT:
        write_reg(REG_AC_DAC_VOL, vol);
        break;
    }

  if (vol == CXD56_VOL_MUTE_REG)
    {
      /* Disable analog out */

      ret = fw_as_acacontrol(CXD56_ACA_CTL_SET_OUTPUT_DEVICE,
                           (uint32_t)CXD56_OUT_DEV_OFF);
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          auderr("ERROR: Set output device for mute failed (%d)\n", ret);
          return -EBUSY;
        }
    }
  else
    {
      /* Enable analog out */

      ret = fw_as_acacontrol(CXD56_ACA_CTL_SET_OUTPUT_DEVICE,
                           (uint32_t)CXD56_OUT_DEV_SP);
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          auderr("ERROR: Set output device for volume failed (%d)\n", ret);
          return -EBUSY;
        }
    }

  return OK;
}

static void cxd56_init_mic_input(uint8_t mic_num, uint8_t bits)
{
  uint8_t i;

  cxd56_aureg_t mic_ch_sel[] =
    {
      REG_MIC_CH1_SEL,
      REG_MIC_CH2_SEL,
      REG_MIC_CH3_SEL,
      REG_MIC_CH4_SEL,
      REG_MIC_CH5_SEL,
      REG_MIC_CH6_SEL,
      REG_MIC_CH7_SEL,
      REG_MIC_CH8_SEL
    };

  if (bits == 16)
    {
      mic_num = (mic_num > (CXD56_MIC_TRANS_CH_16BIT * 2)) ?
                CXD56_MIC_TRANS_CH_16BIT : (mic_num + 1) / 2;

      write_reg(REG_MIC_IN_BITWT, 1);
    }
  else
    {
      mic_num = (mic_num > CXD56_MIC_TRANS_CH_24BIT) ?
                CXD56_MIC_TRANS_CH_24BIT : mic_num;

      write_reg(REG_MIC_IN_BITWT, 0);
    }

  for (i = 0; i < mic_num; i++)
    {
      write_reg(mic_ch_sel[i], i);
    }

  for (i = mic_num; i < CXD56_IN_CHANNELS_MAX; i++)
    {
      write_reg(mic_ch_sel[i], 8);
    }

  write_reg(REG_CLK_EN_AHBMSTR_MIC, 1);
  write_reg(REG_MIC_IN_START_ADR,   0x00000000);
  write_reg(REG_MIC_IN_SAMPLE_NO,   0);
}

static void cxd56_init_i2s1_output(uint8_t bits)
{
  write_reg(REG_I2S1_OUT_SD1_L_SEL, 1);
  write_reg(REG_I2S1_OUT_SD1_R_SEL, 0);
  write_reg(REG_I2S1_OUT_BITWT, (bits == 16));
  write_reg(REG_CLK_EN_AHBMSTR_I2S1, 1);
  write_reg(REG_I2S1_OUT_START_ADR, 0);
  write_reg(REG_I2S1_OUT_SAMPLE_NO, 0);
}

static void cxd56_set_dma_int_en(bool enabled)
{
  if (enabled)
    {
      write_reg(REG_INT_EN1_BITS, 0xf);
      write_reg(REG_INT_HRESP_ERR, 1);
    }
  else
    {
      write_reg32(REG_INT_EN1_BITS, 0x0);
      write_reg32(REG_INT_HRESP_ERR, 0);
    }
}

static void cxd56_set_dma_running(cxd56_dmahandle_t handle, bool running)
{
  if (handle == CXD56_AUDIO_DMA_MIC)
    {
      write_reg(REG_MIC_RTD_TRG, (running ? 0x01 : 0x04));
    }
  else if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg(REG_I2S1_OUT_RTD_TRG, (running ? 0x01 : 0x04));
    }
}

static void cxd56_init_dma(FAR struct cxd56_dev_s *dev)
{
  uint8_t err;
  uint8_t ints;

  audinfo("cxd56_init_dma: state = %d, hdl = %d.\n",
          dev->state,
          dev->dma_handle);

  dq_clear(&dev->up_pendq);
  dq_clear(&dev->up_runq);
#ifdef CONFIG_AUDIO_CXD56_SRC
  dq_clear(&dev->down_pendq);
  dq_clear(&dev->down_runq);
  dq_clear(&dev->down_doneq);
#endif

  ints = CXD56_DMA_INT_DONE | CXD56_DMA_INT_ERR | CXD56_DMA_INT_CMB;

  /* Enable DMA */

  write_reg(REG_AC_MCK_AHBMSTR_EN, 1);

  /* Setup output, bit width etc */

  if (dev->dma_handle == CXD56_AUDIO_DMA_MIC)
    {
      cxd56_set_mic_out_channel(dev);
      cxd56_init_mic_input(dev->channels, dev->bitwidth);
    }
  else
    {
      cxd56_init_i2s1_output(dev->bitwidth);
    }

  /* Clear interrupt states */

  cxd56_int_clear(dev->dma_handle, ints);

  /* Enable interrupts */

  cxd56_int_unmask(dev->dma_handle, ints);
  cxd56_int_unmask_ahb(dev->dma_handle);

  /* Check channel setting. */

  err = cxd56_get_mon_err(dev->dma_handle);
  if (err == CXD56_DMA_MSTATE_ERR_NO_ENABLE_CH)
    {
      auderr("ERROR: No enabled channel for %d\n", dev->dma_handle);
    }
  else if (err == CXD56_DMA_MSTATE_ERR_CH1_4_INVALID)
    {
      auderr("ERROR: Channel 1-4 invalid for %d\n", dev->dma_handle);
    }
  else if (err == CXD56_DMA_MSTATE_ERR_CH5_8_INVALID)
    {
      auderr("ERROR: Channel 5-8 invalid for %d\n", dev->dma_handle);
    }

  cxd56_set_dma_int_en(true);
}

static uint32_t cxd56_get_i2s_rate(uint32_t samplerate)
{
  if (samplerate <= 48000)
    {
      return 1;   /* low */
    }
  else if (samplerate <= 96000)
    {
      return 2;   /* medium */
    }

  return 3;       /* high */
}

static void cxd56_power_on_i2s1(FAR struct cxd56_dev_s *dev)
{
  uint32_t rate;

  write_reg(REG_AC_PDN_DSPS1, 0);                     /* Power on SRC1 */
  write_reg(REG_AC_SD1MASTER, CXD56_I2S1_MODE);       /* I2S1 mode */
  write_reg(REG_AC_DIF1, CXD56_I2S1_FORMAT);          /* I2S1 format */
  write_reg(REG_AC_LR_SWAP1, CXD56_I2S1_FORMAT);
  write_reg(REG_AC_TEST_OUT_SEL0, CXD56_I2S1_BYPASS); /* I2S1 bypass mode */

  rate = cxd56_get_i2s_rate(CXD56_I2S1_DATA_RATE);
  write_reg(REG_AC_SRC1, rate);                       /* I2S1 rate */
}

static int cxd56_power_on_aca(uint32_t samplerate)
{
  struct cxd56_ser_des_param_s ser_param;
  struct cxd56_aca_pwon_param_s pwon_param;
  uint8_t mic_mode;
  uint8_t mic_sel;
  uint8_t i;

  if (fw_as_acacontrol(CXD56_ACA_CTL_CHECK_ID, 0) != 0)
    {
      return -ENXIO;
    }

  if (samplerate > 48000)
    {
      pwon_param.osc_mode = (CXD56_AUDIO_MCLK == CXD56_XTAL_24_576MHZ ?
                             CXD56_ACA_OSC_24_576MHZ_HIRES :
                             CXD56_ACA_OSC_49_152MHZ_HIRES);
    }
  else
    {
      pwon_param.osc_mode = (CXD56_AUDIO_MCLK == CXD56_XTAL_24_576MHZ ?
                             CXD56_ACA_OSC_24_576MHZ :
                             CXD56_ACA_OSC_49_152MHZ);
    }

  pwon_param.dmic_clk_ds = CXD56_DMIC_CLK_DS;
  pwon_param.ad_data_ds = CXD56_DA_DS;
  pwon_param.mic_dev = CXD56_ACA_MIC_AMIC;
  pwon_param.mclk_ds = CXD56_MCLKOUT_DS;
  pwon_param.gpo_ds = CXD56_GPO_A_DS;

  if (fw_as_acacontrol(CXD56_ACA_CTL_POWER_ON_COMMON,
                    (uint32_t)&pwon_param) != 0)
    {
      return -EBUSY;
    }

  /* MIC SETUP */

  mic_mode = cxd56_get_mic_mode();

  if (CXD56_AUDIO_CFG_MIC_MODE_128FS == mic_mode)
    {
      ser_param.ser_mode = CXD56_SER_MODE_4CH;
      ser_param.ser_fs   = CXD56_SER_FS_128;
    }
  else
    {
      ser_param.ser_mode = CXD56_SER_MODE_8CH;
      ser_param.ser_fs   = CXD56_SER_FS_64;
    }

  for (i = 0; i < CXD56_IN_CHANNELS_MAX; i++)
    {
      mic_sel = (CXD56_AUDIO_CFG_MIC >> (i * CXD56_MIC_CH_BITNUM)) &
                CXD56_MIC_CH_BITMAP;
      ser_param.sel_ch.in[i] = (enum cxd56_pulco_ser_sel_ch_id_e)mic_sel;
    }

  if (fw_as_acacontrol(CXD56_ACA_CTL_SET_SERDES, (uint32_t)&ser_param) != 0)
    {
      return -EBUSY;
    }

  return OK;
}

static int cxd56_power_on_analog_output(FAR struct cxd56_dev_s *dev)
{
  struct cxd56_aca_smaster_param_s smaster_param;
  struct cxd56_aca_pwoutput_param_s pwon_param;

  if (dev->samplerate > 48000)
    {
      smaster_param.mode   = CXD56_SMSTR_MODE_FS_32;
      smaster_param.mck_fs = CXD56_SMSTR_MCK_FS_1024;
      pwon_param.mode      = CXD56_SMSTR_MODE_FS_32;
    }
  else
    {
      smaster_param.mode   = CXD56_SMSTR_MODE_FS_16;
      smaster_param.mck_fs = CXD56_SMSTR_MCK_FS_512;
      pwon_param.mode      = CXD56_SMSTR_MODE_FS_16;
    }

  smaster_param.ch_sel   = CXD56_SMSTR_CHSEL_NORMAL;
  smaster_param.out2dly  = 0x00;
  smaster_param.pwm_mode = CXD56_SMSTR_PWMMD_BOTH;

  pwon_param.out_dev     = CXD56_ACA_OUT_OFF;
  pwon_param.pwm_out[0]  = CXD56_ACA_PWMOUT_UNKNOWN;
  pwon_param.pwm_out[1]  = CXD56_ACA_PWMOUT_UNKNOWN;
  pwon_param.sp_delay    = CXD56_ACA_SP_DELAY_SEL_UNKNOWN;
  pwon_param.loop_mode   = CXD56_ACA_SP_LOOP_MODE_UNKNOWN;
  pwon_param.sp_dly_free = CXD56_ACA_SP_DLY_FREE_UNKNOWN;
  pwon_param.sp_spliton  = CXD56_SP_SPLIT_ON;
  pwon_param.sp_drv      = CXD56_SP_DRIVER;

  if (fw_as_acacontrol(CXD56_ACA_CTL_SET_SMASTER,
                    (uint32_t)&smaster_param) != 0)
    {
      return -EBUSY;
    }

  if (fw_as_acacontrol(CXD56_ACA_CTL_POWER_ON_OUTPUT,
                    (uint32_t)&pwon_param) != 0)
    {
      return -EBUSY;
    }

  /* Power on S-Mster. */

  write_reg(REG_AC_PDN_SMSTR, 0);
  write_reg(REG_AC_NSDD, 0x07fb5);

  if (dev->samplerate > 48000 && CXD56_AUDIO_MCLK == CXD56_XTAL_49_152MHZ)
    {
      write_reg(REG_AC_NSX2, 1);
    }
  else
    {
      write_reg(REG_AC_NSX2, 0);
    }

  /* Set smaster and enable */

  write_reg(REG_INT_M_OVF_SMASL, 0);
  write_reg(REG_INT_M_OVF_SMASR, 0);
  write_reg(REG_AC_NSPMUTE, 0);

  cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_OUT, dev->volume);

  return OK;
}

static void cxd56_set_mic_gains(uint8_t gain,
                                struct cxd56_aca_pwinput_param_s *param)
{
  uint8_t i;
  uint8_t pga_gain;
  uint8_t mic_id = 0;
  uint8_t mic_sel = 0;

  /* TODO: Replace gain param with array in dev. How to configure? */

  for (i = 0; i < CXD56_IN_CHANNELS_MAX; i++)
    {
      mic_sel = (CXD56_AUDIO_CFG_MIC >> (i * CXD56_MIC_CH_BITNUM)) &
                CXD56_MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          mic_id = mic_sel - 1;
          param->mic_gain[mic_id] = (gain >= CXD56_MIC_GAIN_MAX) ?
                                     CXD56_MIC_GAIN_MAX :
                                    (gain / 30) * 30;

          pga_gain = gain - param->mic_gain[mic_id];
          param->pga_gain[mic_id] = (pga_gain >= CXD56_MIC_PGA_GAIN_MAX) ?
                                     CXD56_MIC_PGA_GAIN_MAX : pga_gain;
        }
    }
}

static void cxd56_get_mic_config(uint8_t *count, uint8_t *dev, uint8_t *mode)
{
  uint8_t i;
  uint8_t is_dmic;
  uint8_t is_amic;
  uint8_t mic_sel = 0;
  uint8_t mic_count = 0;

  *dev = 0;
  *mode = 0;

  for (i = 0; i < CXD56_IN_CHANNELS_MAX; i++)
    {
      mic_sel = (CXD56_AUDIO_CFG_MIC >> (i * CXD56_MIC_CH_BITNUM)) &
                CXD56_MIC_CH_BITMAP;
      if ((mic_sel >= 1) && (mic_sel <= 4))
        {
          is_amic = true;
          mic_count++;
        }
      else if ((mic_sel >= 5) && (mic_sel <= 12))
        {
          is_dmic = true;
          mic_count++;
        }
    }

  if (is_amic)
    {
      if (is_dmic)
        {
          *dev = CXD56_AUDIO_CFG_MIC_DEV_ANADIG;
          *mode = CXD56_AUDIO_CFG_MIC_MODE_64FS;
        }
      else
        {
          *dev = CXD56_AUDIO_CFG_MIC_DEV_ANALOG;
          *mode = CXD56_AUDIO_CFG_MIC_MODE_128FS;
        }
    }
  else
    {
      if (is_dmic)
        {
          *dev = CXD56_AUDIO_CFG_MIC_DEV_DIGITAL;
          *mode = CXD56_AUDIO_CFG_MIC_MODE_64FS;
        }
      else
        {
          *dev = CXD56_AUDIO_CFG_MIC_DEV_NONE;
          *mode = CXD56_AUDIO_CFG_MIC_MODE_64FS;
        }
    }

  *count = mic_count;
}

static uint8_t cxd56_get_mic_mode(void)
{
  uint8_t count;
  uint8_t dev;
  uint8_t mode;

  cxd56_get_mic_config(&count, &dev, &mode);

  return mode;
}

static int cxd56_power_on_micbias(FAR struct cxd56_dev_s *dev)
{
  struct timespec start;

  if (fw_as_acacontrol(CXD56_ACA_CTL_POWER_ON_MICBIAS, 0) != 0)
    {
      return -EBUSY;
    }

  /* Set mic boot time */

  if (clock_systime_timespec(&start) < 0)
    {
      dev->mic_boot_start = 0x0ull;
    }
  else
    {
      dev->mic_boot_start = (uint64_t)start.tv_sec * 1000 +
                            (uint64_t)start.tv_nsec / 1000000;
    }

  return OK;
}

static void cxd56_audio_power_on_cic(uint8_t mic_in,
                                     uint8_t mic_mode,
                                     uint8_t cic_num,
                                     FAR struct cxd56_audio_mic_gain_s *gain)
{
  /* Power on CIC. */

  if (mic_in == CXD56_AUDIO_CFG_CIC_IN_SEL_CXD)
    {
      if (cic_num > 3)
        {
          write_reg(REG_AC_CIC4IN_SEL, 0);
          write_reg(REG_AC_HPF4_MODE,  1);
        }

      if (cic_num > 2)
        {
          if (read_reg(REG_AC_PDN_AMICEXT) == 1)
            {
              write_reg(REG_AC_PDN_AMICEXT, 0);
            }

          write_reg(REG_AC_CIC3IN_SEL, 0);
          write_reg(REG_AC_HPF3_MODE,  1);
        }

      if (cic_num > 1)
        {
          write_reg(REG_AC_PDN_AMIC2,  0);
          write_reg(REG_AC_CIC2IN_SEL, 0);
          write_reg(REG_AC_HPF2_MODE,  1);
          write_reg(REG_AC_CIC2_GAIN_MODE, 1);
        }

      if (cic_num > 0)
        {
          write_reg(REG_AC_PDN_AMIC1,  0);
          write_reg(REG_AC_CIC1IN_SEL, 0);
          write_reg(REG_AC_HPF1_MODE,  1);
          write_reg(REG_AC_CIC1_GAIN_MODE, 1);
        }
    }
  else if(mic_in == CXD56_AUDIO_CFG_CIC_IN_SEL_DMICIF)
    {
      if (read_reg(REG_AC_PDN_DMIC) == 1)
        {
          write_reg(REG_AC_PDN_DMIC, 0);
        }

      if (cic_num > 3)
        {
          write_reg(REG_AC_CIC4IN_SEL, 1);
          write_reg(REG_AC_HPF4_MODE,  1);
        }

      if (cic_num > 2)
        {
          write_reg(REG_AC_CIC3IN_SEL, 1);
          write_reg(REG_AC_HPF3_MODE,  1);
        }

      if (cic_num > 1)
        {
          write_reg(REG_AC_CIC2IN_SEL, 1);
          write_reg(REG_AC_HPF2_MODE,  1);
          write_reg(REG_AC_CIC2_GAIN_MODE, 1);
        }

      if (cic_num > 0)
        {
          write_reg(REG_AC_CIC1IN_SEL, 1);
          write_reg(REG_AC_HPF1_MODE,  1);
          write_reg(REG_AC_CIC1_GAIN_MODE, 1);
        }
    }

  if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_128FS)
    {
      write_reg(REG_AC_ADC_FS, 1);
    }
  else if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_64FS)
    {
      write_reg(REG_AC_ADC_FS, 0);
    }
}

static int cxd56_power_on_decim(uint8_t mic_mode, uint16_t samplerate)
{
  /* Enable AHBMASTER.
   * Because the output of DecimationFilter is input to BusIF.
   */

  write_reg(REG_AC_MCK_AHBMSTR_EN, 1);

  /* Power on DECIM. */

  write_reg(REG_AC_DECIM0_EN, 1);

  /* DECIM param */

  if ((mic_mode == CXD56_AUDIO_CFG_MIC_MODE_64FS) && (samplerate > 48000))
    {
      write_reg(REG_AC_SEL_DECIM, 0);
    }
  else
    {
      write_reg(REG_AC_SEL_DECIM, 1);
    }

  if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_128FS)
    {
      write_reg(REG_AC_SEL_INF,  1);
      write_reg(REG_AC_DCMFS,    2);
      write_reg(REG_AC_DCMFS_34, 2);
    }
  else if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_64FS)
    {
      write_reg(REG_AC_SEL_INF,  0);
      write_reg(REG_AC_DCMFS,    1);
      write_reg(REG_AC_DCMFS_34, 1);
    }
  else
    {
      return -EINVAL;
    }

  if (samplerate <= 48000)
    {
      write_reg(REG_AC_SEL_OUTF, 0);
    }
  else
    {
      write_reg(REG_AC_SEL_OUTF, 2);
    }

  /* DECIM_SEL */

  write_reg(REG_AC_OUTEN_MIC2R_A, ((0x0f >> 3) & 0x01));
  write_reg(REG_AC_OUTEN_MIC2L_A, ((0x0f >> 2) & 0x01));
  write_reg(REG_AC_OUTEN_MIC1R_A, ((0x0f >> 1) & 0x01));
  write_reg(REG_AC_OUTEN_MIC1L_A, ((0x0f >> 0) & 0x01));

  write_reg(REG_AC_OUTEN_MIC2R_B, ((0x0f >> 3) & 0x01));
  write_reg(REG_AC_OUTEN_MIC2L_B, ((0x0f >> 2) & 0x01));
  write_reg(REG_AC_OUTEN_MIC1R_B, ((0x0f >> 1) & 0x01));
  write_reg(REG_AC_OUTEN_MIC1L_B, ((0x0f >> 0) & 0x01));

  return OK;
}

static void cxd56_set_mic_out_channel(FAR struct cxd56_dev_s *dev)
{
  uint8_t i;
  uint8_t mic_num;
  uint8_t ch_sel[CXD56_IN_CHANNELS_MAX];

  mic_num = dev->channels;

  if ((dev->bitwidth == 16) &&
      (CXD56_DMA_FORMAT == CXD56_DMA_FORMAT_RL))
    {
      for (i = 0; i < CXD56_IN_CHANNELS_MAX; i++)
        {
          ch_sel[i] = (i & 1) ? i - 1 : i + 1;
        }
    }
  else
    {
      for (i = 0; i < CXD56_IN_CHANNELS_MAX; i++)
        {
          ch_sel[i] = i;
        }
    }

  /* For uneven mic counts, duplicate last channel (e.g. dual mono) */

  if ((dev->bitwidth == 16) && ((mic_num & 1) == 1))
    {
      if (CXD56_DMA_FORMAT == CXD56_DMA_FORMAT_LR)
        {
          ch_sel[mic_num] = ch_sel[mic_num - 1];
        }
      else
        {
          ch_sel[mic_num - 1] = ch_sel[mic_num];
        }
    }

  write_reg(REG_AC_SEL_OUT1_L, ch_sel[0]);
  write_reg(REG_AC_SEL_OUT1_R, ch_sel[1]);
  write_reg(REG_AC_SEL_OUT2_L, ch_sel[2]);
  write_reg(REG_AC_SEL_OUT2_R, ch_sel[3]);
  write_reg(REG_AC_SEL_OUT3_L, ch_sel[4]);
  write_reg(REG_AC_SEL_OUT3_R, ch_sel[5]);
  write_reg(REG_AC_SEL_OUT4_L, ch_sel[6]);
  write_reg(REG_AC_SEL_OUT4_R, ch_sel[7]);
}

/****************************************************************************
 * Name: cxd56_power_on_input
 *
 * Description:
 *    Configure and enable input with selected samplerate and mics.
 *
 ****************************************************************************/

static int cxd56_power_on_input(FAR struct cxd56_dev_s *dev)
{
  uint8_t i;
  uint8_t cic_num;
  uint8_t cic_write_num;
  uint8_t mic_dev;
  uint8_t mic_mode;
  uint8_t mic_num;
  uint8_t ret;
  uint32_t val;
  struct cxd56_audio_mic_gain_s cic_gain;
  struct cxd56_aca_pwinput_param_s param;

  const cxd56_aureg_t cic_gain_reg[CXD56_IN_CHANNELS_MAX] =
    {
      REG_AC_CIC1_LGAIN,
      REG_AC_CIC1_RGAIN,
      REG_AC_CIC2_LGAIN,
      REG_AC_CIC2_RGAIN,
      REG_AC_CIC3_LGAIN,
      REG_AC_CIC3_RGAIN,
      REG_AC_CIC4_LGAIN,
      REG_AC_CIC4_RGAIN
    };

  memset((void *)&param, 0, sizeof(param));

  cxd56_get_mic_config(&mic_num, &mic_dev, &mic_mode);

  param.mic_dev = mic_dev;
  if (param.mic_dev == CXD56_AUDIO_CFG_MIC_DEV_ANALOG ||
      param.mic_dev == CXD56_AUDIO_CFG_MIC_DEV_ANADIG)
    {
      cxd56_power_on_micbias(dev);
    }

  param.mic_bias_sel = CXD56_MIC_BIAS;

  /* TODO: Replace hardcoded mic gain with configuration */

  cxd56_set_mic_gains(120, &param);

  if (fw_as_acacontrol(CXD56_ACA_CTL_POWER_ON_INPUT,
                     (uint32_t)&param) != 0)
    {
      return -EBUSY;
    }

  /* Power on CIC */

  cic_num = (mic_num + 1) / CXD56_CIC_MIC_CH_NUM;

  for (i = 0; i < CXD56_IN_CHANNELS_MAX; i++)
    {
      val = (CXD56_AUDIO_CFG_MIC >> (i * CXD56_MIC_CH_BITNUM)) &
            CXD56_MIC_CH_BITMAP;
      cic_gain.gain[i] = (val > 4) ? param.mic_gain[i] : 0;
    }

  cic_write_num = (CXD56_IN_CHANNELS_MAX >= (cic_num * 2)) ?
                (cic_num * 2) : CXD56_IN_CHANNELS_MAX;

  for (i = 0; i < cic_write_num; i++)
    {
      val = (uint32_t)(pow(10.0f,
                           ((float)cic_gain.gain[i] /
                            100.0f / 20.0f)) * 0x4000 +
                            0x4000);

      write_reg(cic_gain_reg[i], val);
    }

  cxd56_audio_power_on_cic(CXD56_AUDIO_CFG_CIC_IN,
                           mic_mode, cic_num, &cic_gain);

  /* Power on decim */

  ret = cxd56_power_on_decim(mic_mode, dev->samplerate);
  if (ret != OK)
    {
      auderr("ERROR: Decim power on failed (%d)\n", ret);
    }

  return ret;
}

static int cxd56_power_on(FAR struct cxd56_dev_s *dev)
{
  uint8_t mic_mode;

  if (g_codec_start_count == 0)
    {
      uint32_t val;

      board_audio_i2s_enable();
      board_audio_initialize();

      /* Power on analog audio */

      if (board_aca_power_control(CXD5247_AVDD | CXD5247_DVDD, true) != 0)
        {
          return -EBUSY;
        }

      if (!board_aca_power_monitor(CXD5247_AVDD | CXD5247_DVDD))
        {
          return -EBUSY;
        }

      cxd56_power_on_aca(dev->samplerate);

      cxd56_audio_clock_enable(CXD56_AUD_MCLK_EXT, 0);

      /* Power_on_codec */

      val = read_reg(REG_AC_REVID);
      if (val != CXD56_EXP_REVID)
        {
          auderr("ERROR: Power on REVID mismatch (%" PRIx32 " vs. %x)\n",
                 val, CXD56_EXP_REVID);
          return -ENXIO;
        }

      val = read_reg(REG_AC_DEVICEID);
      if (val != CXD56_EXP_DEVICEID)
        {
          auderr("ERROR: Power on DEVICEID mismatch (%" PRIx32 " vs. %x)\n",
                 val, CXD56_EXP_DEVICEID);
          return -ENXIO;
        }

      /* Power on serializeer */

      write_reg(REG_AC_SDES_EN, 1);

      /* Set mic mode */

      mic_mode = cxd56_get_mic_mode();
      if (mic_mode == CXD56_AUDIO_CFG_MIC_MODE_128FS)
        {
          write_reg(REG_AC_FS_FS,    0);
          write_reg(REG_AC_SER_MODE, 1);
          write_reg(REG_AC_ADC_FS,   1);
        }
      else
        {
          write_reg(REG_AC_FS_FS,    1);
          write_reg(REG_AC_SER_MODE, 0);
          write_reg(REG_AC_ADC_FS,   0);
        }

      /* Power on codec */

      write_reg(REG_AC_PDN_DSPC, 0);
      write_reg(REG_AC_DSR_RATE, 1);
      write_reg(REG_AC_DIGSFT,   1);

      /* Clear interrupt status of bck_err */

      write_reg(REG_INT_M_I2S1_BCL_ERR1, 0);
      write_reg(REG_INT_M_I2S1_BCL_ERR2, 0);

      cxd56_power_on_i2s1(dev);

      /* Enable I2S data input and output of SRC1 */

      write_reg(REG_AC_SDIN1_EN, 1);
      write_reg(REG_AC_SDOUT1_EN, 1);

      /* Enable BCK, LRCK output if master (1). */

      write_reg(REG_AC_SDCK_OUTENX, CXD56_I2S1_MODE);

      /* Enable serial interface */

      write_reg(REG_AC_FS_CLK_EN,  1);
      write_reg(REG_AC_PDM_OUT_EN, 1);

      /* Initialize data path selection */

      write_reg(REG_AC_AU_DAT_SEL1, 4);
      write_reg(REG_AC_AU_DAT_SEL2, 4);
      write_reg(REG_AC_AU_COD_INSEL2,  2);
      write_reg(REG_AC_AU_COD_INSEL3,  3);
      write_reg(REG_AC_SRC1IN_SEL,  0);
      write_reg(REG_AC_SRC2IN_SEL,  1);

      /* Set BCA data rate */

      write_reg(REG_I2S_ENSEL, ((dev->samplerate > 48000) ? 1 : 0));

      /* Disable DEQ */

      write_reg(REG_AC_DEQ_EN, 0);

      /* Disable DNC. */

      write_reg(REG_AC_DNC1_MUTE, 1);
      write_reg(REG_AC_DNC2_MUTE, 1);
      write_reg(REG_AC_DNC1_START, 0);
      write_reg(REG_AC_DNC2_START, 0);

      /* Disable ALC/SPC */

      write_reg(REG_AC_ALC_EN, 0);
      write_reg(REG_AC_SPC_EN, 0);

      /* Disable Clear Stereo */

      write_reg(REG_AC_CS_SIGN, 0);
      write_reg(REG_AC_CS_VOL, 0x00);

      cxd56_attach_irq(true);
      cxd56_enable_irq(true);
    }

  g_codec_start_count++;

  return OK;
}

static int cxd56_power_off(FAR struct cxd56_dev_s *dev)
{
  /* Disable AHBMASTER. */

  write_reg(REG_AC_MCK_AHBMSTR_EN, 0);

  /* Disable SRC. */

  write_reg(REG_AC_SDIN1_EN, 0);
  write_reg(REG_AC_SDIN2_EN, 0);
  write_reg(REG_AC_SDOUT1_EN, 0);
  write_reg(REG_AC_SDOUT2_EN, 0);
  write_reg(REG_AC_SDCK_OUTENX, 1);
  write_reg(REG_AC_BLF_EN, 0);

  /* Disable SDES. */

  write_reg(REG_AC_PDM_OUT_EN, 0);
  write_reg(REG_AC_FS_CLK_EN, 0);
  write_reg(REG_AC_SDES_EN, 0);

  /* Power off SRC. */

  write_reg(REG_AC_PDN_DSPS1, 1);
  write_reg(REG_AC_PDN_DSPS2, 1);
  write_reg(REG_AC_PDN_DSPB, 1);

  /* Power off CODEC. */

  write_reg(REG_AC_PDN_DSPC, 1);

  /* Power off DNC. */

  write_reg(REG_AC_PDN_DNC1, 1);
  write_reg(REG_AC_PDN_DNC2, 1);
  write_reg(REG_AC_PDN_ANC, 1);

  /* Disable audio clock */

  cxd56_audio_clock_disable();

  if (fw_as_acacontrol(CXD56_ACA_CTL_POWER_OFF_COMMON, 0) != 0)
    {
      return -EBUSY;
    }

  board_aca_power_control(CXD5247_AVDD | CXD5247_DVDD, false);

  /* Disable interrupts */

  cxd56_attach_irq(false);
  cxd56_enable_irq(false);

  board_audio_finalize();

  return OK;
}

/****************************************************************************
 * Name: cxd56_getcaps
 *
 * Description: Get the audio device capabilities.
 *
 ****************************************************************************/

static int cxd56_getcaps(FAR struct audio_lowerhalf_s *lower, int type,
                 FAR struct audio_caps_s *caps)
{
  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Query for supported types of units */

      case AUDIO_TYPE_QUERY:

        /* Stereo output */

        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* The types of audio units we implement */

              caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT |
                                       AUDIO_TYPE_INPUT |
                                       AUDIO_TYPE_FEATURE;
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }
        break;

      /* Output capabilities */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = CXD56_OUT_CHANNELS_MAX;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report supported output sample rates */

              caps->ac_controls.b[0] = CXD56_SUPP_RATES_L;
              caps->ac_controls.b[1] = CXD56_SUPP_RATES_H;
              break;

            default:
              break;
          }
        break;

      /* Output capabilities */

      case AUDIO_TYPE_INPUT:

        caps->ac_channels = CXD56_IN_CHANNELS_MAX;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report supported input sample rates */

              caps->ac_controls.b[0] = CXD56_SUPP_RATES_L;
              caps->ac_controls.b[1] = CXD56_SUPP_RATES_H;
              break;

            default:
              break;
          }
        break;

      /* Feature capabilities */

      case AUDIO_TYPE_FEATURE:

        /* Report supported feature units */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_MUTE;
            caps->ac_controls.b[1] = AUDIO_FU_INP_GAIN >> 8;
          }
        break;

      /* Others are unsupported */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype = 0;
        caps->ac_channels = 0;
        break;
    }

  /* Return the length of the audio_caps_s struct for validation */

  return caps->ac_len;
}

/****************************************************************************
 * Name: cxd56_shutdown
 *
 * Description: Shutdown the chip and puts it in the lowest power
 *              state possible.
 *
 ****************************************************************************/

static int cxd56_shutdown(FAR struct audio_lowerhalf_s *lower)
{
  int ret;
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;

  if (priv->state != CXD56_DEV_STATE_OFF)
    {
      ret = cxd56_power_off(priv);
      if (ret != OK)
        {
          auderr("ERROR: Power off failed (%d)\n", ret);
          return ret;
        }

      g_codec_start_count = 0;
      priv->state = CXD56_DEV_STATE_OFF;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_configure
 *
 * Description: Configure the audio device for the specified mode.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_configure(FAR struct audio_lowerhalf_s *lower,
                 FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int cxd56_configure(FAR struct audio_lowerhalf_s *lower,
                 FAR const struct audio_caps_s *caps)
#endif
{
  int ret = 0;
  uint8_t poweron = 0;
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;

  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:

      switch (caps->ac_format.hw)
        {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
        case AUDIO_FU_VOLUME:
          {
            uint16_t volume = caps->ac_controls.hw[0];

            if (volume >= 0 && volume <= 1000)
              {
                /* Scale the volume setting to the range {-1020..120} */

                priv->volume = CXD56_VOL_NX_TO_CXD56(volume);

                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_OUT, priv->volume);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN1, 0);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN2, 0);
              }
            else
              {
                return -EDOM;
              }
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */
#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
        case AUDIO_FU_MUTE:
          {
            /* Set mic mute/unmute status */

            bool mute = (bool) caps->ac_controls.hw[0];
            audinfo("    Mute: %d\n", mute);
            if (mute)
              {
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_OUT,
                                 CXD56_VOL_MUTE);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN1,
                                 CXD56_VOL_MUTE);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN2,
                                 CXD56_VOL_MUTE);
              }
            else
              {
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_OUT, priv->volume);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN1, 0);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN2, 0);
              }

            if (CXD56_AUDIO_ECODE_OK != ret)
              {
                return ret;
              }
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */
        case AUDIO_FU_INP_GAIN:
          {
            /* Set the mic gain */

            priv->mic_gain = caps->ac_controls.hw[0];
            audinfo("    Mic gain: %d\n", priv->mic_gain);

            /* TODO: How to set individual mic gains?  */
          }
          break;

        default:
          auderr("ERROR: Unknown feature unit: %d\n", caps->ac_format.hw);
          return -ENOTTY;
        }
      break;

    case AUDIO_TYPE_OUTPUT:
      {
        if (caps->ac_controls.b[2] != 16 && caps->ac_controls.b[2] != 24)
          {
            auderr("ERROR: Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            return -ERANGE;
          }

        /* Save the configuration */

        priv->dma_handle = CXD56_AUDIO_DMA_I2S0_DOWN;
        priv->samplerate = caps->ac_controls.hw[0];
        priv->channels = caps->ac_channels;
        priv->bitwidth = caps->ac_controls.b[2];

#ifdef CONFIG_AUDIO_CXD56_SRC
        ret = cxd56_src_init(priv, &priv->down_doneq, &priv->down_pendq);
        if (ret != OK)
          {
            auderr("ERROR: Could not initialize SRC (%d)\n", ret);
            return -ENOMEM;
          }

#endif
        g_dev[priv->dma_handle] = priv;
        poweron = 1;

        audinfo("Configured output using %d:\n", priv->dma_handle);
        audinfo("  Channels:    %d\n", priv->channels);
        audinfo("  Samplerate:  %d\n", priv->samplerate);
        audinfo("  Bit width:   %d\n", priv->bitwidth);
      }
      break;

    case AUDIO_TYPE_INPUT:
      {
        /* Save the configuration */

        priv->dma_handle = CXD56_AUDIO_DMA_MIC;
        priv->samplerate = caps->ac_controls.hw[0];
        priv->channels = caps->ac_channels;
        priv->bitwidth = caps->ac_controls.b[2];

        g_dev[priv->dma_handle] = priv;
        poweron = 1;

        audinfo("Configured input using %d:\n", priv->dma_handle);
        audinfo("  Channels:    %d\n", priv->channels);
        audinfo("  Samplerate:  %d\n", priv->samplerate);
        audinfo("  Bit width:   %d\n", priv->bitwidth);
      }
      break;
    }

  if (poweron)
    {
      /* Get ready to start receiving buffers */

      ret = cxd56_power_on(priv);
      if (ret != OK)
        {
          auderr("ERROR: Power on error (%d)\n", ret);
          return ret;
        }

      cxd56_init_dma(priv);

      priv->state = CXD56_DEV_STATE_STOPPED;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_start
 *
 * Description: Starts playback with the current configuration.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_start(FAR struct audio_lowerhalf_s *lower,
                       FAR void *session)
#else
static int cxd56_start(FAR struct audio_lowerhalf_s *lower)
#endif
{
  int ret;
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;

  /* Set audio path and enable analog input/output */

  if (priv->dma_handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg(REG_AC_AU_DAT_SEL1, CXD56_AUDAT_SEL_BUSIF1);

      ret = cxd56_power_on_analog_output(priv);
      if (ret != OK)
        {
          auderr("ERROR: Power on analog output failed (%d)\n", ret);
          return ret;
        }
    }
  else if (priv->dma_handle == CXD56_AUDIO_DMA_MIC)
    {
#if 0
      /* TODO: Check configuration. From audio_manager.cpp:221 */

      sel_info.au_dat_sel1 = true;
      if (AS_THROUGH_PATH_OUT_MIXER1 == out_path)
          sel_info.cod_insel2  = true;
      else if (AS_THROUGH_PATH_OUT_MIXER2 == out_path)
          sel_info.cod_insel3  = true;
      else if (AS_THROUGH_PATH_OUT_I2S1 == out_path)
          sel_info.src1in_sel  = true;
      else
          sel_info.src2in_sel  = true;
#endif
      write_reg(REG_AC_AU_DAT_SEL1, CXD56_AUDAT_SEL_MIC1);
      write_reg(REG_AC_AU_DAT_SEL2, CXD56_AUDAT_SEL_MIC1);

      ret = cxd56_power_on_input(priv);
      if (ret != OK)
        {
          auderr("ERROR: Power on analog input failed (%d)\n", ret);
          return ret;
        }

      if (priv->mic_boot_start != 0x0ull)
        {
          struct timespec end;
          if (clock_systime_timespec(&end) == 0)
            {
              uint64_t time = (uint64_t)end.tv_sec * 1000 +
                              (uint64_t)end.tv_nsec / 1000000 -
                               priv->mic_boot_start;

              if (time < CXD56_MIC_BOOT_WAIT)
                {
                  nxsig_usleep((CXD56_MIC_BOOT_WAIT - time) * 1000);
                }
            }
        }
    }

  ret = cxd56_init_worker(lower);
  if (ret != OK)
    {
      auderr("ERROR: Could not feed DMA (%d)\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_stop
 *
 * Description: Stops playback with the current configuration.
 *
 ****************************************************************************/

static int cxd56_stop_dma(FAR struct cxd56_dev_s *priv)
{
  int ret;

  if (priv->state != CXD56_DEV_STATE_STOPPED)
    {
      /* Stop DMA */

      cxd56_set_dma_running(priv->dma_handle, false);

      if (priv->dma_handle == CXD56_AUDIO_DMA_MIC)
        {
          /* Power off decimator */

          write_reg(REG_AC_DECIM0_EN, 0);

          /* Power off CIC. */

          write_reg(REG_AC_PDN_AMIC1,   1);
          write_reg(REG_AC_PDN_AMIC2,   1);
          write_reg(REG_AC_PDN_AMICEXT, 1);
          write_reg(REG_AC_PDN_DMIC,    1);

          /* Disable input */

          if (fw_as_acacontrol(CXD56_ACA_CTL_POWER_OFF_INPUT,
                             0) != 0)
            {
              return -EBUSY;
            }

          priv->mic_boot_start = 0x0ull;
        }
      else if (priv->dma_handle == CXD56_AUDIO_DMA_I2S0_DOWN)
        {
          /* Turn off amplifier */

          ret = board_external_amp_mute_control(true);
          if (ret != CXD56_AUDIO_ECODE_OK)
            {
              auderr("ERROR: Couldn't mute amplifier (%d)\n", ret);
              return -EBUSY;
            }

          /* Mute and disable output */

          write_reg(REG_AC_NSPMUTE, 1);
          write_reg(REG_AC_PDN_SMSTR, 1);

          if (fw_as_acacontrol(CXD56_ACA_CTL_POWER_OFF_OUTPUT,
                               0) != 0)
            {
              return -EBUSY;
            }
        }
    }

  return OK;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_stop(FAR struct audio_lowerhalf_s *lower, FAR void *session)
#else
static int cxd56_stop(FAR struct audio_lowerhalf_s *lower)
#endif
{
  int ret;
  FAR void *value;
  struct audio_msg_s msg;
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;

  audinfo("cxd56_stop\n");

  msg.msg_id = AUDIO_MSG_STOP;
  msg.u.data = 0;
  ret = file_mq_send(&priv->mq, (FAR const char *)&msg,
                     sizeof(msg), CONFIG_CXD56_MSG_PRIO);
  if (ret != OK)
    {
      auderr("ERROR: file_mq_send stop message failed (%d)\n", ret);
      return ret;
    }

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

/****************************************************************************
 * Name: cxd56_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_pause(FAR struct audio_lowerhalf_s *lower,
                       FAR void *session)
#else
static int cxd56_pause(FAR struct audio_lowerhalf_s *lower)
#endif
{
  int ret;
  FAR struct cxd56_dev_s *dev = (FAR struct cxd56_dev_s *)lower;

  if (dev->state == CXD56_DEV_STATE_STARTED)
    {
      dev->state = CXD56_DEV_STATE_PAUSED;

      ret = cxd56_stop_dma(dev);
      if (ret != OK)
        {
          auderr("ERROR: Could not stop DMA transfer (%d)\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_resume(FAR struct audio_lowerhalf_s *lower,
                        FAR void *session)
#else
static int cxd56_resume(FAR struct audio_lowerhalf_s *lower)
#endif
{
  int ret;
  FAR struct cxd56_dev_s *dev = (FAR struct cxd56_dev_s *)lower;

  if (dev->state == CXD56_DEV_STATE_PAUSED ||
      dev->state == CXD56_DEV_STATE_BUFFERING)
    {
      if (dev->state == CXD56_DEV_STATE_PAUSED)
        {
          dev->state = CXD56_DEV_STATE_STARTED;
          cxd56_power_on_analog_output(dev);
          board_external_amp_mute_control(false);
        }
      else
        {
          /* NOTE: only power on the analog output
           * when resumed from buffering
           */

          cxd56_power_on_analog_output(dev);
        }

      audinfo("START DMA up_pendq=%d\n", dq_count(&dev->up_pendq));
      ret = cxd56_start_dma(dev);
      if (ret != OK)
        {
          auderr("ERROR: Could not resume DMA transfer (%d)\n", ret);
          return ret;
        }
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: cxd56_release
 *
 * Description: Releases the session.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_release(FAR struct audio_lowerhalf_s *lower,
                  FAR void *session)
#else
static int cxd56_release(FAR struct audio_lowerhalf_s *lower)
#endif
{
  return OK;
}

/****************************************************************************
 * Name: cxd56_reserve
 *
 * Description: Reserves a session.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_reserve(FAR struct audio_lowerhalf_s *lower,
                  FAR void **session)
#else
static int cxd56_reserve(FAR struct audio_lowerhalf_s *lower)
#endif
{
  return OK;
}

static void cxd56_swap_buffer_rl(uint32_t addr, uint16_t size)
{
  uint32_t i = 0;
  uint16_t tmp_buffer;
  uint16_t *p_lch = (uint16_t *)addr;
  uint16_t *p_rch = p_lch + 1;

  while (i++ < size / 4)
    {
      tmp_buffer = *p_lch;
      *p_lch = *p_rch;
      *p_rch = tmp_buffer;

      p_lch += 2;
      p_rch += 2;
    }
}

static int cxd56_start_dma(FAR struct cxd56_dev_s *dev)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  int retry;
  int timeout;
  uint32_t addr;
  uint32_t size;
  int ret = OK;

  flags = spin_lock_irqsave(&dev->lock);
#ifdef CONFIG_AUDIO_CXD56_SRC
  FAR struct ap_buffer_s *src_apb;

  if (dq_count(&dev->down_pendq) == 0)
#else
  if (dq_count(&dev->up_pendq) == 0)
#endif
    {
      /* Underrun occurred, stop DMA and change state for buffering */

      audwarn("Underrun\n");

      spin_unlock_irqrestore(&dev->lock, flags);
      ret = cxd56_stop_dma(dev);
      flags = spin_lock_irqsave(&dev->lock);
      audwarn("STOP DMA due to underrun\n");
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          auderr("ERROR: Could not stop DMA transfer (%d)\n", ret);
          dev->running = false;
        }

      dev->state = CXD56_DEV_STATE_BUFFERING;
    }
  else
    {
      /* Fill up with as many DMA requests as we can */

#ifdef CONFIG_AUDIO_CXD56_SRC
      while (dq_count(&dev->down_pendq) > 0)
#else
      while (dq_count(&dev->up_pendq) > 0)
#endif
        {
          if (cxd56_dma_is_busy(dev->dma_handle))
            {
              /* DMA busy, will retry next time */

              ret = OK;
              goto exit;
            }

#ifdef CONFIG_AUDIO_CXD56_SRC
          src_apb = (struct ap_buffer_s *) dq_peek(&dev->down_pendq);
          addr = CXD56_PHYSADDR(src_apb->samp);
          size = (src_apb->nbytes / (dev->bitwidth / 8) / dev->channels) - 1;
#else
          apb = (struct ap_buffer_s *) dq_peek(&dev->up_pendq);
          addr = CXD56_PHYSADDR(apb->samp);
          size = (apb->nbytes / (dev->bitwidth / 8) / dev->channels) - 1;
#endif

          if (dev->dma_handle == CXD56_AUDIO_DMA_MIC)
            {
              write_reg(REG_MIC_IN_START_ADR, addr);
              write_reg(REG_MIC_IN_SAMPLE_NO, size);
            }
          else
            {
              if (dev->bitwidth == 16 &&
                  CXD56_DMA_FORMAT == CXD56_DMA_FORMAT_RL)
                {
#ifdef CONFIG_AUDIO_CXD56_SRC
                  cxd56_swap_buffer_rl((uint32_t)src_apb->samp,
                                       src_apb->nbytes);
#else
                  cxd56_swap_buffer_rl((uint32_t)apb->samp,
                                        apb->nbytes);
#endif
                }

              write_reg(REG_I2S1_OUT_START_ADR, addr);
              write_reg(REG_I2S1_OUT_SAMPLE_NO, size);
            }

          /* Start DMA, use workaround with first buffer */

          if (dev->state != CXD56_DEV_STATE_STARTED)
            {
              if (dev->dma_handle == CXD56_AUDIO_DMA_I2S0_DOWN)
                {
                  /* Turn on amplifier */

                  spin_unlock_irqrestore(&dev->lock, flags);
                  board_external_amp_mute_control(false);
                  flags = spin_lock_irqsave(&dev->lock);
                }

              /* Mask interrupts */

              cxd56_int_mask(dev->dma_handle, CXD56_DMA_INT_ERR);
              cxd56_int_mask(dev->dma_handle, CXD56_DMA_INT_DONE);

              /* Sync workaround loop */

              for (retry = 0; retry < CXD56_DMA_START_RETRY_CNT; retry++)
                {
                  /* Clear interrupt status */

                  cxd56_int_clear(dev->dma_handle, CXD56_DMA_INT_ERR);
                  cxd56_int_clear(dev->dma_handle, CXD56_DMA_INT_SMP);

                  for (timeout = 0; timeout < CXD56_DMA_TIMEOUT; timeout++)
                    {
                      if (cxd56_int_has_smp(dev->dma_handle))
                        {
                          break;
                        }
                    }

                  if (timeout == CXD56_DMA_TIMEOUT)
                    {
                      ret = -ETIME;
                      goto exit;
                    }

                  /* Reset channel select */

                  cxd56_reset_channel_sel(dev->dma_handle);

                  /* Start DMA */

                  cxd56_set_dma_running(dev->dma_handle, true);

                  /* Wait for 1sample tramsfer */

                  if (dev->samplerate > 48000)
                    {
                      up_udelay(CXD56_DMA_SMP_WAIT_HIRES);
                    }
                  else
                    {
                      up_udelay(CXD56_DMA_SMP_WAIT_NORMALT);
                    }

                  /* Check if an error interrupt has occurred */

                  if (cxd56_int_has_error(dev->dma_handle))
                    {
                      cxd56_set_dma_running(dev->dma_handle, false);
                      cxd56_int_clear(dev->dma_handle, CXD56_DMA_INT_ERR);

                      for (timeout = 0;
                           timeout < CXD56_DMA_TIMEOUT;
                           timeout++)
                        {
                          if (CXD56_DMA_MSTATE_BUF_EMPTY ==
                              cxd56_get_mon_buf(dev->dma_handle))
                            {
                              if (cxd56_int_is_done(dev->dma_handle))
                                {
                                  cxd56_int_clear(dev->dma_handle,
                                                  CXD56_DMA_INT_DONE);
                                  break;
                                }
                            }
                        }
                    }
                      else
                    {
                      break;
                    }
                }

              if (retry == CXD56_DMA_START_RETRY_CNT)
                {
                  audinfo("Workaround retries maxed out\n");
                }

              cxd56_int_unmask(dev->dma_handle, CXD56_DMA_INT_DONE);
              cxd56_int_clear(dev->dma_handle, CXD56_DMA_INT_ERR);
              cxd56_int_unmask(dev->dma_handle, CXD56_DMA_INT_ERR);
            }
          else
            {
              /* start DMA */

              cxd56_set_dma_running(dev->dma_handle, true);
            }

#ifdef CONFIG_AUDIO_CXD56_SRC
          dq_get(&dev->down_pendq);
          dq_put(&dev->down_runq, &src_apb->dq_entry);

          apb = (struct ap_buffer_s *) dq_get(&dev->up_pendq);
#else
          dq_get(&dev->up_pendq);
#endif
          dq_put(&dev->up_runq, &apb->dq_entry);

          dev->state = CXD56_DEV_STATE_STARTED;

#ifndef CONFIG_AUDIO_CXD56_SRC
          if ((apb->flags & AUDIO_APB_FINAL) != 0)
            {
              /* If the apb is final, send stop message */

              audinfo("Final apb\n");
              struct audio_msg_s msg;
              msg.msg_id = AUDIO_MSG_STOP;
              msg.u.data = 0;

              spin_unlock_irqrestore(&dev->lock, flags);
              ret = file_mq_send(&dev->mq, (FAR const char *)&msg,
                                 sizeof(msg), CONFIG_CXD56_MSG_PRIO);
              flags = spin_lock_irqsave(&dev->lock);

              if (ret != OK)
                {
                  auderr("ERROR: file_mq_send for stop failed (%d)\n", ret);
                  goto exit;
                }
            }
#endif
        }
    }

exit:
  spin_unlock_irqrestore(&dev->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: cxd56_enqueuebuffer
 *
 * Description: Enqueue an audio buffer for playback.
 *
 ****************************************************************************/

static int cxd56_enqueuebuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb)
{
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_AUDIO_CXD56_SRC
  ret = cxd56_src_enqueue(apb);
  if (ret != OK)
    {
      auderr("ERROR: SRC processing failed (%d)\n", ret);
    }
  else
    {
#endif
      flags = spin_lock_irqsave(&priv->lock);

      apb->dq_entry.flink = NULL;
      dq_put(&priv->up_pendq, &apb->dq_entry);

      spin_unlock_irqrestore(&priv->lock, flags);

      if (priv->mq.f_inode != NULL)
        {
          msg.msg_id = AUDIO_MSG_ENQUEUE;
          msg.u.data = 0;

          ret = file_mq_send(&priv->mq, (FAR const char *) &msg,
                             sizeof(msg), CONFIG_CXD56_MSG_PRIO);
          if (ret != OK)
            {
              auderr("ERROR: file_mq_send to enqueue failed (%d)\n", ret);
              return ret;
            }
        }
#ifdef CONFIG_AUDIO_CXD56_SRC
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: cxd56_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int cxd56_cancelbuffer(FAR struct audio_lowerhalf_s *lower,
                 FAR struct ap_buffer_s *apb)
{
  return OK;
}

/****************************************************************************
 * Name: cxd56_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int cxd56_ioctl(FAR struct audio_lowerhalf_s *lower, int cmd,
                 unsigned long arg)
{
  int ret = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  /* Handle ioctl commands from the upper-half driver */

  switch (cmd)
    {
       /* Return our preferred buffer size and count */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_CXD56_AUDIO_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_CXD56_AUDIO_NUM_BUFFERS;
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        audinfo("Unhandled ioctl: %d\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *cxd56_workerthread(pthread_addr_t pvarg)
{
  FAR struct cxd56_dev_s *priv = (struct cxd56_dev_s *)pvarg;
  struct audio_msg_s msg;
  unsigned int prio;
  int size;
  int ret;

  audinfo("Workerthread started.\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

  /* Mark ourself as running */

  priv->running = true;

  /* Initial buffering */

  ret = cxd56_start_dma(priv);
  if (ret != OK)
    {
      auderr("ERROR: Could not start DMA transfer (%d)\n", ret);
      priv->running = false;
    }

  while (priv->running)
    {
      size = file_mq_receive(&priv->mq, (FAR char *)&msg,
                             sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (size == 0)
        {
          priv->running = false;
          break;
        }

      /* Process the message */

      switch (msg.msg_id)
        {
          case AUDIO_MSG_STOP:
            ret = cxd56_stop_dma(priv);
            if (ret != CXD56_AUDIO_ECODE_OK)
              {
                auderr("ERROR: Could not stop DMA transfer (%d)\n", ret);
                priv->running = false;
              }

#ifdef CONFIG_AUDIO_CXD56_SRC
            ret = cxd56_src_stop();
            if (ret != OK)
              {
                auderr("ERROR: Could not stop SRC (%d)\n", ret);
              }

            ret = cxd56_src_deinit();
            if (ret != OK)
              {
                auderr("ERROR: Could not deinit SRC (%d)\n", ret);
              }

#endif
            priv->state = CXD56_DEV_STATE_STOPPED;
            priv->running = false;
            audinfo("Workerthread stopped.\n");
            break;

          case AUDIO_MSG_DATA_REQUEST:
            if (priv->state == CXD56_DEV_STATE_STARTED)
            {
                cxd56_start_dma(priv);
            }
            break;

          case AUDIO_MSG_ENQUEUE:
            if (priv->state == CXD56_DEV_STATE_BUFFERING)
              {
                audwarn("Buffering up_pendq=%d\n",
                        dq_count(&priv->up_pendq));

                FAR struct ap_buffer_s *apb;
                apb = (struct ap_buffer_s *)(&priv->up_pendq)->tail;

                bool final = (apb != NULL) &&
                  ((apb->flags & AUDIO_APB_FINAL) != 0);

                /* If up_pendq exceeds the threshold or up_pendq
                 * contains the final buffer, then start dma.
                 */

                if (CONFIG_CXD56_AUDIO_NUM_BUFFERS <=
                    dq_count(&priv->up_pendq) || final)
                  {
                    cxd56_resume((FAR struct audio_lowerhalf_s *)priv);
                  }
              }
            break;

          default:
            break;
        }
    }

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);

  /* Send AUDIO_MSG_COMPLETE to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  return NULL;
}

/* Setup worker thread and message queue */

static int cxd56_init_worker(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr m_attr;
  pthread_attr_t t_attr;
  void *value;
  int ret;

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%" PRIXPTR,
           (uintptr_t)priv);

  m_attr.mq_maxmsg  = 16;
  m_attr.mq_msgsize = sizeof(struct audio_msg_s);
  m_attr.mq_curmsgs = 0;
  m_attr.mq_flags   = 0;

  ret = file_mq_open(&priv->mq, priv->mqname,
                     O_RDWR | O_CREAT, 0644, &m_attr);
  if (ret < 0)
    {
      auderr("ERROR: Could not allocate message queue.\n");
      return ret;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
    }

  pthread_attr_init(&t_attr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&t_attr, &sparam);
  pthread_attr_setstacksize(&t_attr,
                            CONFIG_CXD56_AUDIO_WORKER_STACKSIZE);

  ret = pthread_create(&priv->threadid, &t_attr, cxd56_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed (%d)\n", ret);
      return ret;
    }

  pthread_setname_np(priv->threadid, "cxd56");

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_initialize
 *
 * Description:
 *   Initialize audio on the CXD56 device
 *
 * Input Parameters:
 *
 ****************************************************************************/

struct audio_lowerhalf_s *cxd56_initialize(
                         FAR const struct cxd56_lower_s *lower)
{
  FAR struct cxd56_dev_s *priv;

  audinfo("cxd56_initialize\n");
  priv = (FAR struct cxd56_dev_s *)kmm_zalloc(sizeof(struct cxd56_dev_s));
  if (priv)
    {
      priv->dev.ops = &g_audioops;
      priv->lower   = lower;
      priv->state   = CXD56_DEV_STATE_OFF;

      dq_init(&priv->up_pendq);
      dq_init(&priv->up_runq);
#ifdef CONFIG_AUDIO_CXD56_SRC
      dq_init(&priv->down_pendq);
      dq_init(&priv->down_runq);
      dq_init(&priv->down_doneq);
#endif
    }

  return &priv->dev;
}
