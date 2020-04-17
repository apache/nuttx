/****************************************************************************
 * drivers/audio/cxd56.c
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <fcntl.h>
#include <errno.h>
#include <queue.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>

#include <arch/board/cxd56_clock.h>
#include <arch/board/board.h>
#include <arch/chip/audio.h>

#include "cxd56.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_BASE     0x0e300000
#define REG_BASE_INT 0xe0045000

#define CXD56_GEN_MASK(len, pos) (((len) == 32) ? 0xffffffff : \
                                  ((1 << (len)) - 1) << (pos))

#define CXD56_IRQ1_BIT_MIC  (1 << 6)   /* AU0 */
#define CXD56_IRQ1_BIT_I2S1 (1 << 7)   /* AU1 */

#define CXD56_VOL_MIN            -1020
#define CXD56_VOL_MAX            120
#define CXD56_VOL_MUTE           (CXD56_VOL_MIN - 1)
#define CXD56_VOL_RANGE          (CXD56_VOL_MAX - CXD56_VOL_MIN)
#define CXD56_VOL_NX_TO_CXD56(v) ((int)((float)((v) / 1000.0) * CXD56_VOL_RANGE) \
                                 + CXD56_VOL_MIN)
#define CXD56_VOL_WAIT_TIME      20
#define CXD56_VOL_TO_REG(vol)    (((vol) / 5) & 0xff)
#define CXD56_VOL_MUTE_REG       0x33
#define CXD56_VOL_MUTE_TIME(vol, cycles) \
                                 (((CXD56_VOL_TO_REG(vol) - CXD56_VOL_MUTE_REG) & 0xff) \
                                 * ((cycles) + 1) * 4 / 48 * 1000)

#define CXD56_IN_CHANNELS_MAX  8
#define CXD56_OUT_CHANNELS_MAX 2

/* Samplerates field is split into low and high byte */

#define CXD56_SUPP_RATES_L  (AUDIO_SAMP_RATE_8K  | AUDIO_SAMP_RATE_11K | \
                             AUDIO_SAMP_RATE_16K | AUDIO_SAMP_RATE_22K | \
                             AUDIO_SAMP_RATE_32K | AUDIO_SAMP_RATE_44K | \
                             AUDIO_SAMP_RATE_48K)
#define CXD56_SUPP_RATES_H  ((AUDIO_SAMP_RATE_96K  | AUDIO_SAMP_RATE_128K | \
                              AUDIO_SAMP_RATE_192K) >> 8)
#define CXD56_SUPP_RATES    (CXD56_SUPP_RATES_L | CXD56_SUPP_RATES_H)

/* Mic selections */

#define CXD56_ACA_MIC_AMIC 1  /* Analog MIC */

/* External XTAL */

#define CXD56_AUD_MCLK_EXT (0u<<16)

/* Oscillator modes */

#define CXD56_ACA_OSC_24_576MHZ       1  /* 24.576MHz */
#define CXD56_ACA_OSC_24_576MHZ_HIRES 2  /* 24.576MHz, Hi-Res */
#define CXD56_ACA_OSC_49_152MHZ       3  /* 49.152MHz */
#define CXD56_ACA_OSC_49_152MHZ_HIRES 4  /* 49.152MHz, Hi-Res */

/* Control IDs for external as_aca_control */
#define CXD56_ACA_CTL_CHECK_ID          0
#define CXD56_ACA_CTL_POWER_ON_COMMON   1
#define CXD56_ACA_CTL_POWER_ON_OUTPUT   3
#define CXD56_ACA_CTL_SET_SMASTER       5
#define CXD56_ACA_CTL_POWER_OFF_COMMON  6
#define CXD56_ACA_CTL_POWER_OFF_OUTPUT  8
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

#define CXD56_AUDAT_SEL_BUSIF1 4
#define CXD56_AUDAT_SEL_BUSIF2 4

#define CXD56_DMA_MSTATE_BUF_EMPTY  3
#define CXD56_DMA_TIMEOUT           10000
#define CXD56_DMA_START_RETRY_CNT   10
#define CXD56_DMA_SMP_WAIT_HIRES    10 /* usec per sample. */
#define CXD56_DMA_SMP_WAIT_NORMALT  40 /* usec per sample. */
#define CXD56_DMA_CMD_FIFO_NOT_FULL 1
#define CXD56_DMA_START_ADDR_MASK   0x3fffffff

/* Queue extensions */

#define dq_push(q,n) (dq_addlast((dq_entry_t*)n,(q)))
#define dq_pop(q)    (dq_remfirst(q))
#define dq_clear(q) \
  do \
    { \
      dq_remlast(q); \
    } \
  while (!dq_empty(q))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Located in arch/arm/src/cxd56xx/cxd56_clock.c */

extern void cxd56_audio_clock_enable(uint32_t clk, uint32_t div);
extern void cxd56_audio_clock_disable(void);
extern bool cxd56_audio_clock_is_enabled(void);

/* Located in arch/arm/src/cxd56xx/cxd56_farapistub.S */

extern uint32_t as_aca_control(uint8_t type, uint32_t param);

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

static void cxd56_attach_irq(bool attach);
static void cxd56_enable_irq(bool enable);
static uint32_t cxd56_get_i2s_rate(uint32_t samplerate);
static int cxd56_init_dma(FAR struct cxd56_dev_s *dev);
static void cxd56_init_i2s1_output(uint8_t bits);
static int cxd56_init_worker(FAR struct audio_lowerhalf_s *lower);
static uint32_t cxd56_power_off(FAR struct cxd56_dev_s *dev);
static uint32_t cxd56_power_on(FAR struct cxd56_dev_s *dev);
static uint32_t cxd56_power_on_aca(uint32_t samplerate);
static uint32_t cxd56_power_on_analog_output(FAR struct cxd56_dev_s *dev);
static void cxd56_power_on_i2s1(FAR struct cxd56_dev_s *dev);
static int cxd56_start_dma(FAR struct cxd56_dev_s *dev);
static int cxd56_stop_dma(FAR struct cxd56_dev_s *priv);
static void cxd56_set_dma_int_en(bool enabled);
static void cxd56_set_dma_running(cxd56_dmahandle_t handle, bool running);
static void cxd56_set_volume(enum cxd56_vol_id_e id, int16_t vol, bool fade);
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
const cxd56_aureg_t  REG_AC_PDN_DSPS1 =
{
  REG_BASE + 0x0100, 21,  1
};
const cxd56_aureg_t  REG_AC_PDN_DSPS2 =
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
const cxd56_aureg_t  REG_AC_DECIM0 =
{
  REG_BASE + 0x0104, 16,  1
};
const cxd56_aureg_t  REG_AC_DECIM1 =
{
  REG_BASE + 0x0104, 17,  1
};
const cxd56_aureg_t  REG_AC_SDES_EN =
{
  REG_BASE + 0x0104, 18,  1
};
const cxd56_aureg_t  REG_AC_MCK_AMBMSTR_EN =
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
const cxd56_aureg_t  REG_AC_SDIN1_VOL =
{
  REG_BASE + 0x0228, 16,  8
};
const cxd56_aureg_t  REG_AC_SDIN2_VOL =
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
const cxd56_aureg_t  REG_AC_DSPRAM4_CLR =
{
  REG_BASE + 0x0400, 28,  1
};
const cxd56_aureg_t  REG_AC_DSPRAM2_CLR =
{
  REG_BASE + 0x0400, 29,  1
};
const cxd56_aureg_t  REG_AC_DSPRAM1_CLR =
{
  REG_BASE + 0x0400, 31,  1
};
const cxd56_aureg_t  REG_AC_S_RESET =
{
  REG_BASE + 0x0400, 16,  1
};
const cxd56_aureg_t  REG_AC_PDM_OUT_EN =
{
  REG_BASE + 0x0500, 16,  1
};
const cxd56_aureg_t  REG_AC_FS_CLK_EN =
{
  REG_BASE + 0x0500, 24,  1
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

const cxd56_aureg_t  REG_MIC_RTD_TRG =
{
  REG_BASE + 0x1008,  0,  3
};
const cxd56_aureg_t  REG_MIC_CH8_SEL =
{
  REG_BASE + 0x1010,  0,  4
};
const cxd56_aureg_t  REG_MIC_MON_START =
{
  REG_BASE + 0x1014,  3,  1
};
const cxd56_aureg_t  REG_MIC_MON_ERROR_SET =
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
const cxd56_aureg_t  REG_CLK_EN_AHBMSTR_MIC =
{
  REG_BASE + 0x11f0,  0,  1
};
const cxd56_aureg_t  REG_CLK_EN_AHBMSTR_I2S1 =
{
  REG_BASE + 0x11f0,  1,  1
};
const cxd56_aureg_t  REG_MIC_INT_CTRL_DONE =
{
  REG_BASE + 0x1140,  0,  1
};
const cxd56_aureg_t  REG_MIC_INT_CTRL_ERR =
{
  REG_BASE + 0x1140,  1,  1
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

/* Aquire semaphore */

static void cxd56_take_sem(sem_t *sem)
{
  int ret;

  do
    {
      ret = nxsem_wait(sem);
      if (ret != 0 && ret != -EINTR)
        {
          auderr("cxd56_take_sem failed. (%d)\n", ret);
        }
    }
  while (ret == -EINTR);
}

/* Release semaphore */

static void cxd56_give_sem(sem_t *sem)
{
  int ret;

  do
    {
      ret = nxsem_post(sem);
      if (ret != 0 && ret != -EINTR)
        {
          auderr("cxd56_give_sem failed. (%d)\n", ret);
        }
    }
  while (ret == -EINTR);
}

static void cxd56_int_clear(cxd56_dmahandle_t handle, uint8_t intbits)
{
  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg32(REG_I2S1_INT_CTRL_DONE, intbits);
    }
}

static void cxd56_int_mask(cxd56_dmahandle_t handle, uint8_t intbits)
{
  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      intbits |= read_reg32(REG_I2S1_INT_MASK_DONE);
      write_reg32(REG_I2S1_INT_MASK_DONE, intbits);
    }
}

static void cxd56_int_unmask(cxd56_dmahandle_t handle, uint8_t intbits)
{
  uint32_t curr;

  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      curr = read_reg32(REG_I2S1_INT_MASK_DONE);
      write_reg32(REG_I2S1_INT_MASK_DONE, curr & ~intbits);
    }
}

static void cxd56_int_unmask_ahb(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg32(REG_AHB_MASTER_I2S1_MASK, 0x00000202);
    }
}

static uint32_t cxd56_int_get_state(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return (0x3f & read_reg32(REG_I2S1_INT_CTRL_DONE)
              & ~(read_reg32(REG_I2S1_INT_MASK_DONE)));
    }

  return 0;
}

static uint8_t cxd56_get_monbuf_state(cxd56_dmahandle_t handle)
{
  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      return read_reg(REG_I2S1_OUT_MON_MONBUF);
    }

  return 0;
}

static void cxd56_reset_channel_sel(cxd56_dmahandle_t handle)
{
  uint32_t sel;

  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      sel = read_reg32(REG_I2S1_OUT_SD1_R_SEL);
      write_reg32(REG_I2S1_OUT_SD1_R_SEL, 0xffffffff);
      write_reg32(REG_I2S1_OUT_SD1_R_SEL, sel);
    }
}

static void cxd56_dma_int_handler(void)
{
  uint16_t          ecode = CXD56_AUDIO_ECODE_DMA_HANDLE_INV;
  cxd56_dmahandle_t hdl;

  uint32_t int_irq = read_reg32(REG_INT_IRQ1);
  uint32_t int_i2s = cxd56_int_get_state(CXD56_AUDIO_DMA_I2S0_DOWN);

  if ((int_irq & CXD56_IRQ1_BIT_I2S1) && (int_i2s != 0))
    {
      hdl = CXD56_AUDIO_DMA_I2S0_DOWN;

      write_reg32(REG_I2S1_INT_CTRL_DONE, int_i2s);

      if (int_i2s & (1 << REG_I2S1_INT_CTRL_DONE.pos))
        {
          ecode = CXD56_AUDIO_ECODE_DMA_CMPLT;
        }

      if (int_i2s & (1 << REG_I2S1_INT_CTRL_ERR.pos))
        {
          /* Mask and clear transfer error interrupt */

          write_reg(REG_I2S1_INT_MASK_ERR, 1);
          write_reg(REG_I2S1_INT_CTRL_ERR, 1);

          ecode = CXD56_AUDIO_ECODE_DMA_TRANS;
        }

      if (int_i2s & (1 << REG_I2S1_INT_CTRL_CMB.pos))
        {
          /* Mask and clear bus error interrupt */

          write_reg(REG_I2S1_INT_MASK_CMB, 1);
          write_reg(REG_I2S1_INT_CTRL_CMB, 1);

          ecode = CXD56_AUDIO_ECODE_DMA_CMB;
        }
    }
  else
    {
      _warn("cxd56_dma_int_handler: Unhandled interrupt\n");
      return;
    }

  if (ecode != CXD56_AUDIO_ECODE_DMA_HANDLE_INV)
    {
      struct audio_msg_s msg;
      struct cxd56_dev_s *dev;

      dev = g_dev[hdl];

      /* Trigger new DMA job */

      cxd56_take_sem(&dev->pendsem);
      if (dq_count(&dev->runningq) > 0)
        {
          FAR struct ap_buffer_s *apb;
          apb = (struct ap_buffer_s *) dq_pop(&dev->runningq);
          dev->dev.upper(dev->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
        }

      cxd56_give_sem(&dev->pendsem);

      if (dev->mq != NULL)
        {
          msg.msg_id = AUDIO_MSG_DATA_REQUEST;
          msg.u.data = 0;
          (void)nxmq_send(dev->mq, (FAR const char *) &msg,
                          sizeof(msg), CONFIG_CXD56_MSG_PRIO);
        }

      if (ecode == CXD56_AUDIO_ECODE_DMA_TRANS)
        {
          /* End of data */

          msg.msg_id = AUDIO_MSG_STOP;
          msg.u.data = 0;
          (void)nxmq_send(dev->mq, (FAR const char *)&msg,
                          sizeof(msg), CONFIG_CXD56_MSG_PRIO);
        }
    }
}

static void cxd56_attach_irq(bool attach)
{
  if (attach)
    {
      irq_attach(CXD56_IRQ_AUDIO_0, (xcpt_t)cxd56_dma_int_handler, NULL);
      irq_attach(CXD56_IRQ_AUDIO_1, (xcpt_t)cxd56_dma_int_handler, NULL);
      irq_attach(CXD56_IRQ_AUDIO_2, (xcpt_t)cxd56_dma_int_handler, NULL);
      irq_attach(CXD56_IRQ_AUDIO_3, (xcpt_t)cxd56_dma_int_handler, NULL);
    }
  else
    {
      irq_detach(CXD56_IRQ_AUDIO_0);
      irq_detach(CXD56_IRQ_AUDIO_1);
      irq_detach(CXD56_IRQ_AUDIO_2);
      irq_detach(CXD56_IRQ_AUDIO_3);
    }
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

static void cxd56_set_volume(enum cxd56_vol_id_e id, int16_t vol, bool fade)
{
  uint32_t waittime = 0;

  if (vol == CXD56_VOL_MUTE)
    {
      vol = CXD56_VOL_MUTE_REG;
    }
  else
    {
      CXD56_AUDIO_ECODE ret;

      vol = CXD56_VOL_TO_REG(vol);

      /* Enable analog out */

      ret = as_aca_control(CXD56_ACA_CTL_SET_OUTPUT_DEVICE,
                           (uint32_t)CXD56_OUT_DEV_SP);
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          auderr("cxd56_set_volume analog out enable failed. (%d)\n", ret);
        }
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

  waittime = (fade ? CXD56_VOL_MUTE_TIME(vol, 1) : CXD56_VOL_WAIT_TIME);

  nxsig_usleep(waittime);

  if (vol == CXD56_VOL_MUTE_REG)
    {
      /* Disable analog out */

      as_aca_control(CXD56_ACA_CTL_SET_OUTPUT_DEVICE,
                     (uint32_t)CXD56_OUT_DEV_OFF);
    }
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
  if (handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg(REG_I2S1_OUT_RTD_TRG, (running ? 0x01 : 0x04));
    }
}

static int cxd56_init_dma(FAR struct cxd56_dev_s *dev)
{
  int ret = CXD56_AUDIO_ECODE_OK;
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)dev;
  uint8_t ints = CXD56_DMA_INT_DONE | CXD56_DMA_INT_ERR | CXD56_DMA_INT_CMB;

  audinfo("cxd56_init_dma: state = %d, hdl = %d.\n",
          dev->state,
          dev->dma_handle);

  dq_clear(&dev->pendingq);
  dq_clear(&dev->runningq);

  /* Enable DMA */

  write_reg(REG_AC_MCK_AMBMSTR_EN, 1);

  /* Setup output, bit width etc */

  cxd56_init_i2s1_output(priv->bitwidth);

  /* Clear interrupt states */

  cxd56_int_clear(dev->dma_handle, ints);

  /* Enable interrupts */

  cxd56_int_unmask(dev->dma_handle, ints);
  cxd56_int_unmask_ahb(dev->dma_handle);

  cxd56_set_dma_int_en(true);

  return ret;
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

static uint32_t cxd56_power_on_aca(uint32_t samplerate)
{
  struct cxd56_aca_pwon_param_s pwon_param;

  if (as_aca_control(CXD56_ACA_CTL_CHECK_ID, (uint32_t)NULL) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_CHKID;
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
  pwon_param.mic_dev = CXD56_ACA_MIC_AMIC;  /* TODO: analog/digital mic select */
  pwon_param.mclk_ds = CXD56_MCLKOUT_DS;
  pwon_param.gpo_ds = CXD56_GPO_A_DS;
  if (as_aca_control(CXD56_ACA_CTL_POWER_ON_COMMON,
                    (uint32_t)&pwon_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWON;
    }

  return CXD56_AUDIO_ECODE_OK;
}

static uint32_t cxd56_power_on_analog_output(FAR struct cxd56_dev_s *dev)
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

  if (as_aca_control(CXD56_ACA_CTL_SET_SMASTER,
                    (uint32_t)&smaster_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_SET_SMASTER;
    }

  if (as_aca_control(CXD56_ACA_CTL_POWER_ON_OUTPUT,
                    (uint32_t)&pwon_param) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWON_OUTPUT;
    }

  /* Power on S-Mster. */

  write_reg(REG_AC_PDN_SMSTR, 0);

  /* Set NSDD. */

  write_reg(REG_AC_NSDD, 0x07fb5);

  /* Set NSX2. */

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

  cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_OUT, dev->volume, true);

  return CXD56_AUDIO_ECODE_OK;
}

static uint32_t cxd56_power_on(FAR struct cxd56_dev_s *dev)
{
  if (g_codec_start_count == 0)
    {
      board_audio_i2s_enable();
      board_audio_initialize();

      /* Power on analog audio */

      if (board_aca_power_control(CXD5247_AVDD | CXD5247_DVDD, true) != 0)
        {
          return CXD56_AUDIO_ECODE_ANA_PWON;
        }

      if (!board_aca_power_monitor(CXD5247_AVDD | CXD5247_DVDD))
        {
          return CXD56_AUDIO_ECODE_ANA_PWON;
        }

      cxd56_power_on_aca(dev->samplerate);

      cxd56_audio_clock_enable(CXD56_AUD_MCLK_EXT, 0);

      /* Power_on_codec */

      if (read_reg(REG_AC_REVID) != CXD56_EXP_REVID)
        {
          auderr("cxd56_power_on REVID mismatch (%x vs. %x).\n",
               REG_AC_REVID, CXD56_EXP_REVID);
          return CXD56_AUDIO_ECODE_ANA_CHKID;
        }

      if (read_reg(REG_AC_DEVICEID) != CXD56_EXP_DEVICEID)
        {
          auderr("cxd56_power_on DEVICEID mismatch (%x vs. %x).\n",
               REG_AC_DEVICEID, CXD56_EXP_DEVICEID);
          return CXD56_AUDIO_ECODE_ANA_CHKID;
        }

      /* Power on serializeer */

      write_reg(REG_AC_SDES_EN, 1);

      /* Power on codec */

      write_reg(REG_AC_PDN_DSPC, 0);
      write_reg(REG_AC_DSR_RATE, 1);
      write_reg(REG_AC_DIGSFT,   1);

#if defined(CONFIG_CXD56_I2S0) || defined(CONFIG_CXD56_I2S1)
      /* Clear interrupt status of bck_err */

      write_reg(REG_INT_M_I2S1_BCL_ERR1, 0);
      write_reg(REG_INT_M_I2S1_BCL_ERR2, 0);

      cxd56_power_on_i2s1(dev);
#endif /* defined(CONFIG_CXD56_I2S0) || defined(CONFIG_CXD56_I2S1) */

#ifdef CONFIG_CXD56_I2S0
      /* Enable I2S data input and output of SRC1 */

      write_reg(REG_AC_SDIN1_EN, 1);
      write_reg(REG_AC_SDOUT1_EN, 1);
#endif /* CONFIG_CXD56_I2S0 */

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

      write_reg(REG_I2S_ENSEL, (dev->samplerate > 48000) ? 1 : 0);

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

      /* Attach interrupts */
      cxd56_attach_irq(true);
      cxd56_enable_irq(true);
    }

  g_codec_start_count++;
  dev->state = CXD56_DEV_STATE_STOPPED;

  return CXD56_AUDIO_ECODE_OK;
}

static uint32_t cxd56_power_off(FAR struct cxd56_dev_s *dev)
{
  audinfo("cxd56_power_off\n");

  /* Disable AHBMASTER. */

  write_reg(REG_AC_MCK_AMBMSTR_EN, 0);

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
  if (as_aca_control(CXD56_ACA_CTL_POWER_OFF_COMMON, (uint32_t)NULL) != 0)
    {
      return CXD56_AUDIO_ECODE_ANA_PWOFF;
    }

  board_aca_power_control(CXD5247_AVDD | CXD5247_DVDD, false);

  /* Disable interrupts */

  cxd56_attach_irq(false);
  cxd56_enable_irq(false);

  board_audio_finalize();

  return CXD56_AUDIO_ECODE_OK;
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

      /* Feature capabilities */

      case AUDIO_TYPE_FEATURE:

        /* Report supported feature units */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            caps->ac_controls.b[0] = AUDIO_FU_VOLUME;
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
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;
  int ret = OK;

  audinfo("cxd56_shutdown\n");

  if (priv->state != CXD56_DEV_STATE_OFF)
    {
      ret = cxd56_power_off(priv);
      if (ret != CXD56_AUDIO_ECODE_OK)
        {
          auderr("cxd56_audio_poweroff() failed. (%d)\n", ret);
          goto error;
        }

      g_codec_start_count = 0;
      priv->state = CXD56_DEV_STATE_OFF;
    }

error:
  return ret;
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
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;
  int ret = OK;

  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:
      audinfo("  AUDIO_TYPE_FEATURE\n");

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
                audinfo("Volume: %d (priv = %d)\n", volume, priv->volume);

                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_OUT,
                                 priv->volume, false);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN1,
                                 priv->volume, false);
                cxd56_set_volume(CXD56_AUDIO_VOLID_MIXER_IN2,
                                 priv->volume, false);
              }
            else
              {
                ret = -EDOM;
              }
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

        default:
          auderr("Unrecognized feature unit\n");
          ret = -ENOTTY;
          break;
        }
      break;

    case AUDIO_TYPE_OUTPUT:
      {
        if (caps->ac_controls.b[2] != 16 && caps->ac_controls.b[2] != 24)
          {
            auderr("Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            ret = -ERANGE;
            break;
          }

        /* Save the configuration */

        priv->dma_handle = CXD56_AUDIO_DMA_I2S0_DOWN;
        priv->samplerate = caps->ac_controls.hw[0];
        priv->channels = caps->ac_channels;
        priv->bitwidth = caps->ac_controls.b[2];

        g_dev[priv->dma_handle] = priv;

        audinfo("Configured output using %d:\n", priv->dma_handle);
        audinfo("  Channels:    %d\n", priv->channels);
        audinfo("  Samplerate:  %d\n", priv->samplerate);
        audinfo("  Bit width:   %d\n", priv->bitwidth);

        ret = cxd56_power_on(priv);
        if (ret != CXD56_AUDIO_ECODE_OK)
          {
            auderr("Power on error. (%d)\n", ret);
            goto error;
          }

        ret = cxd56_init_dma(priv);
        if (ret != CXD56_AUDIO_ECODE_OK)
          {
            auderr("DMA init failed. (%d)\n", ret);
            goto error;
          }

        ret = cxd56_power_on_analog_output(priv);
        if (ret != CXD56_AUDIO_ECODE_OK)
          {
            auderr("Power on analog output failed- (%d)\n", ret);
            goto error;
          }
      }
      break;
    }

error:
  return ret;
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

  audinfo("cxd56_start\n");

  /* Select audio data path */

  if (priv->dma_handle == CXD56_AUDIO_DMA_I2S0_DOWN)
    {
      write_reg(REG_AC_AU_DAT_SEL1, CXD56_AUDAT_SEL_BUSIF1);
    }

  ret = cxd56_init_worker(lower);
  if (ret != CXD56_AUDIO_ECODE_OK)
    {
      auderr("Could not feed DMA. (%d)\n", ret);
      goto error;
    }

  priv->state = CXD56_DEV_STATE_STARTING;

error:
  return ret;
}

/****************************************************************************
 * Name: cxd56_stop
 *
 * Description: Stops playback with the current configuration.
 *
 ****************************************************************************/

static int cxd56_stop_dma(FAR struct cxd56_dev_s *priv)
{
  int ret = OK;

  if (priv->state != CXD56_DEV_STATE_STOPPED)
    {
      audinfo("Stopping DMA for handle %d.\n", priv->dma_handle);

      /* Stop DMA */

      cxd56_set_dma_running(priv->dma_handle, false);

      if (priv->dma_handle == CXD56_AUDIO_DMA_I2S0_DOWN)
        {
          /* Turn off amplifier */

          board_external_amp_mute_control(true);

          /* Mute and disable output */

          write_reg(REG_AC_NSPMUTE, 1);
          write_reg(REG_AC_PDN_SMSTR, 1);
          as_aca_control(CXD56_ACA_CTL_POWER_OFF_OUTPUT, (uint32_t)NULL);
        }

      priv->state = CXD56_DEV_STATE_STOPPED;
    }

  return ret;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cxd56_stop(FAR struct audio_lowerhalf_s *lower, FAR void *session)
#else
static int cxd56_stop(FAR struct audio_lowerhalf_s *lower)
#endif
{
  int ret = OK;
  FAR void *value;
  struct audio_msg_s msg;
  FAR struct cxd56_dev_s *priv = (FAR struct cxd56_dev_s *)lower;

  audinfo("cxd56_stop\n");

  if (priv->state == CXD56_DEV_STATE_STOPPED)
    {
      goto error;
    }

  priv->state = CXD56_DEV_STATE_STOPPING;

  msg.msg_id = AUDIO_MSG_STOP;
  msg.u.data = 0;
  (void)nxmq_send(priv->mq, (FAR const char *)&msg,
                  sizeof(msg), CONFIG_CXD56_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;
error:
  return ret;
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
  audinfo("cxd56_pause\n");
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
  audinfo("cxd56_resume\n");
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
  audinfo("cxd56_release\n");
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
  audinfo("cxd56_reserve\n");
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
  int retry;
  int timeout;
  uint32_t addr;
  uint32_t size;
  static int bufcount = 0;
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  cxd56_take_sem(&dev->pendsem);
  if (dq_count(&dev->pendingq) == 0)
    {
    audinfo("Pending queue empty! Played %d buffers.\n", bufcount);
    dev->state = CXD56_DEV_STATE_STOPPING;
    }
  else
    {
      /* Fill up with as many DMA requests as we can */

      while (dq_count(&dev->pendingq) > 0)
        {
          if (read_reg(REG_I2S1_OUT_RTD_TRG) != CXD56_DMA_CMD_FIFO_NOT_FULL)
            {
              /* DMA busy, will retry next time */

              ret = CXD56_AUDIO_ECODE_OK;
              goto exit;
            }

          apb = (struct ap_buffer_s *) dq_peek(&dev->pendingq);
          addr = ((uint32_t)apb->samp) & CXD56_DMA_START_ADDR_MASK;
          size = (apb->nbytes / ((dev->bitwidth / 8) + dev->channels)) - 1;

          if (dev->bitwidth == 16 && CXD56_DMA_FORMAT == CXD56_DMA_FORMAT_RL)
            {
              cxd56_swap_buffer_rl((uint32_t)apb->samp, apb->nbytes);
            }

          write_reg(REG_I2S1_OUT_START_ADR, addr);
          write_reg(REG_I2S1_OUT_SAMPLE_NO, size);

          /* Start DMA, use workaround with first buffer */

          if (dev->state != CXD56_DEV_STATE_STARTED)
            {
              /* Turn on amplifier */

              board_external_amp_mute_control(false);

              /* Mask interrupts */

              cxd56_int_mask(dev->dma_handle, CXD56_DMA_INT_ERR);
              cxd56_int_mask(dev->dma_handle, CXD56_DMA_INT_DONE);

              /* Sync workaround loop */

              for (retry = 0; retry < CXD56_DMA_START_RETRY_CNT; retry++)
                {
                  /* Clear interrupt status */

                  cxd56_int_clear(dev->dma_handle, CXD56_DMA_INT_ERR);
                  cxd56_int_clear(dev->dma_handle, CXD56_DMA_INT_SMP);

                  /* Lock interrupt */

                  up_irq_disable();
                  sched_lock();

                  for (timeout = 0; timeout < CXD56_DMA_TIMEOUT; timeout++)
                    {
                      if (read_reg(REG_I2S1_INT_CTRL_SMP))
                        {
                          break;
                        }
                    }

                  if (timeout == CXD56_DMA_TIMEOUT)
                    {
                      ret = CXD56_AUDIO_ECODE_DMA_SMP_TIMEOUT;
                      goto exit;
                    }

                  /* Reset channel select */

                  cxd56_reset_channel_sel(dev->dma_handle);

                  /* Start DMA */

                  cxd56_set_dma_running(CXD56_AUDIO_DMA_I2S0_DOWN, true);

                  /* Unlock interrupt */

                  sched_unlock();
                  up_irq_enable();

                  /* Wait for 1sample tramsfer */

                  if (dev->samplerate > 48000)
                    {
                      up_udelay(CXD56_DMA_SMP_WAIT_HIRES);
                    }
                  else
                    {
                      up_udelay(CXD56_DMA_SMP_WAIT_NORMALT);
                    }

                  /* Check whether an error interrupt has occurred */

                  if (read_reg(REG_I2S1_INT_CTRL_ERR))
                    {
                      cxd56_set_dma_running(CXD56_AUDIO_DMA_I2S0_DOWN,
                                            false);
                      cxd56_int_clear(dev->dma_handle,
                                      CXD56_DMA_INT_ERR);

                      for (timeout = 0;
                           timeout < CXD56_DMA_TIMEOUT;
                           timeout++)
                        {
                          if (CXD56_DMA_MSTATE_BUF_EMPTY ==
                              cxd56_get_monbuf_state(dev->dma_handle))
                            {
                              if (read_reg(REG_I2S1_INT_CTRL_DONE))
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

              cxd56_int_unmask(dev->dma_handle, CXD56_DMA_INT_DONE);
              cxd56_int_clear(dev->dma_handle, CXD56_DMA_INT_ERR);
              cxd56_int_unmask(dev->dma_handle, CXD56_DMA_INT_ERR);
            }
          else
            {
              /* start DMA */

              cxd56_set_dma_running(CXD56_AUDIO_DMA_I2S0_DOWN, true);
            }

          dq_pop(&dev->pendingq);
          dq_push(&dev->runningq, &apb->dq_entry);
          dev->state = CXD56_DEV_STATE_STARTED;
          bufcount++;
        }
    }

exit:
  cxd56_give_sem(&dev->pendsem);
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
  int                ret = OK;

  cxd56_take_sem(&priv->pendsem);
  dq_push(&priv->pendingq, &apb->dq_entry);
  cxd56_give_sem(&priv->pendsem);

  if (priv->mq != NULL)
    {
      msg.msg_id = AUDIO_MSG_ENQUEUE;
      msg.u.data = 0;
      (void)nxmq_send(priv->mq, (FAR const char *) &msg,
                      sizeof(msg), CONFIG_CXD56_MSG_PRIO);
    }

  return ret;
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
  audinfo("cxd56_cancelbuffer\n");
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
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  audinfo("cxd56_ioctl\n");

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
        audinfo("Unhandled ioctl: %d\n", cmd);
        break;
    }

  return OK;
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
  struct audio_msg_s     msg;
  unsigned int           prio;
  int                    ret;
  int                    size;

  audinfo("Workerthread started.\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

  /* Mark ourself as running */

  priv->running = true;

  /* Initial buffering */

  ret = cxd56_start_dma(priv);
  if (ret != CXD56_AUDIO_ECODE_OK)
    {
      auderr("Could not start DMA transfer. (%d)\n", ret);
      priv->running = false;
    }

  while (priv->running)
    {
      size = nxmq_receive(priv->mq, (FAR char *)&msg, sizeof(msg), &prio);

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
            cxd56_stop_dma(priv);
            priv->running = false;
            audinfo("Workerthread stopped.\n");
            break;

          case AUDIO_MSG_DATA_REQUEST:
            cxd56_start_dma(priv);
            break;

          case AUDIO_MSG_ENQUEUE:
          default:
            break;
        }
    }

  mq_close(priv->mq);
  mq_unlink(priv->mqname);
  priv->mq = NULL;

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
  struct mq_attr     m_attr;
  pthread_attr_t     t_attr;
  void               *value;
  int                ret = CXD56_AUDIO_ECODE_OK;

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%X", priv);

  m_attr.mq_maxmsg  = 16;
  m_attr.mq_msgsize = sizeof(struct audio_msg_s);
  m_attr.mq_curmsgs = 0;
  m_attr.mq_flags   = 0;

  priv->mq = mq_open(priv->mqname, O_RDWR | O_CREAT, 0644, &m_attr);
  if (priv->mq == NULL)
    {
      auderr("Could not allocate message queue.\n");
      ret = -ENOMEM;
      goto error;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
    }

  pthread_attr_init(&t_attr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  (void)pthread_attr_setschedparam(&t_attr, &sparam);
  (void)pthread_attr_setstacksize(&t_attr,
                                  CONFIG_CXD56_AUDIO_WORKER_STACKSIZE);

  ret = pthread_create(&priv->threadid, &t_attr, cxd56_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("pthread_create failed. (%d)\n", ret);
      goto error;
    }
  else
    {
      pthread_setname_np(priv->threadid, "cxd56");
    }

error:
  return ret;
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

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendingq);
      dq_init(&priv->runningq);
    }

  return &priv->dev;
}
