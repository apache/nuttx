/****************************************************************************
 * arch/arm/src/sama5/sam_classd.c
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

#include "nuttx/arch.h"

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <mqueue.h>
#include <nuttx/mutex.h>
#include <debug.h>
#include <assert.h>
#include <arch/board/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/audio.h>
#include <hardware/sam_classd.h>

#include "sam_dmac.h"
#include "sam_memories.h"
#include "sam_periphclks.h"
#include "sam_clockconfig.h"

#ifdef CONFIG_SAMA5D2_CLASSD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_SAMA5D2_CLASSD_INFLIGHT
#  define CONFIG_SAMA5D2_CLASSD_INFLIGHT 2
#endif

/* DMA is required */

#ifdef CONFIG_SAMA5_HAVE_XDMA
#  if !defined(CONFIG_SAMA5_XDMAC0) && !defined(CONFIG_SAMA5_XDMAC1)
#    error XDMAC required by CLASSD
#endif

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.
 */

#define DMA_TIMEOUT_MS    (800)
#define DMA_TIMEOUT_TICKS MSEC2TICK(DMA_TIMEOUT_MS)

/* DMA Bus configuration - only the SAMA5D2 has Class D */

#define DMA8_FLAGS \
        (DMACH_FLAG_PERIPHAHB_AHB_IF1  | \
         DMACH_FLAG_FIFOCFG_LARGEST    | \
         DMACH_FLAG_PERIPHH2SEL        | \
         DMACH_FLAG_PERIPHISPERIPH     | \
         DMACH_FLAG_PERIPHWIDTH_8BITS  | \
         DMACH_FLAG_PERIPHCHUNKSIZE_1  | \
         DMACH_FLAG_MEMPID_MAX         | \
         DMACH_FLAG_MEMAHB_AHB_IF0     | \
         DMACH_FLAG_MEMWIDTH_8BITS     | \
         DMACH_FLAG_MEMINCREMENT       | \
         DMACH_FLAG_MEMCHUNKSIZE_1     | \
         DMACH_FLAG_MEMBURST_4         | \
         DMACH_FLAG_PERIPHPID(SAM_PID_CLASSD))
#define DMA16_FLAGS \
        (DMACH_FLAG_PERIPHAHB_AHB_IF1  | \
         DMACH_FLAG_FIFOCFG_LARGEST    | \
         DMACH_FLAG_PERIPHH2SEL        | \
         DMACH_FLAG_PERIPHISPERIPH     | \
         DMACH_FLAG_PERIPHWIDTH_16BITS | \
         DMACH_FLAG_PERIPHCHUNKSIZE_1  | \
         DMACH_FLAG_MEMPID_MAX         | \
         DMACH_FLAG_MEMAHB_AHB_IF0     | \
         DMACH_FLAG_MEMWIDTH_8BITS     | \
         DMACH_FLAG_MEMINCREMENT       | \
         DMACH_FLAG_MEMCHUNKSIZE_1     | \
         DMACH_FLAG_MEMBURST_4         | \
         DMACH_FLAG_PERIPHPID(SAM_PID_CLASSD))
#define DMA32_FLAGS \
        (DMACH_FLAG_PERIPHAHB_AHB_IF1  | \
         DMACH_FLAG_FIFOCFG_LARGEST    | \
         DMACH_FLAG_PERIPHH2SEL        | \
         DMACH_FLAG_PERIPHISPERIPH     | \
         DMACH_FLAG_PERIPHWIDTH_32BITS | \
         DMACH_FLAG_PERIPHCHUNKSIZE_1  | \
         DMACH_FLAG_MEMPID_MAX         | \
         DMACH_FLAG_MEMAHB_AHB_IF0     | \
         DMACH_FLAG_MEMWIDTH_8BITS     | \
         DMACH_FLAG_MEMINCREMENT       | \
         DMACH_FLAG_MEMCHUNKSIZE_1     | \
         DMACH_FLAG_MEMBURST_4         | \
         DMACH_FLAG_PERIPHPID(SAM_PID_CLASSD))

#define DMA_INITIAL      0
#define DMA_AFTER_SETUP  1
#define DMA_AFTER_START  2
#define DMA_CALLBACK     3
#define DMA_TIMEOUT      3
#define DMA_END_TRANSFER 4
#define DMA_NSAMPLES     5

/* DSP clock settings as per Datasheet
 *
 * PCM Clock = (Crystal * (ND + 1 + FRACR/2^22) / (QDPMC + 1)) / 8
 */

#if BOARD_MAINOSC_FREQUENCY == (12000000)
#  define DSP_CK_ND_12M288     56
#  define DSP_CK_FRACR_12M288  1442841
#  define DSP_CK_QDPMC_12M288  6
#  define DSP_CK_DIV_12M288    0
#  define DSP_QDAUDIO_12M288   0
#  define DSP_CK_ND_11M1896    59
#  define DSP_CK_FRACR_11M1896 885837
#  define DSP_CK_QDPMC_11M1896 7
#  define DSP_CK_DIV_11M1896   0
#  define DSP_QDAUDIO_11M1896  0
#  define DSP_CK_ND_5M6498     50
#  define DSP_CK_FRACR_5M6498  7066563
#  define DSP_CK_QDPMC_5M6498  13
#  define DSP_CK_DIV_5M6498    0
#  define DSP_QDAUDIO_5M6498   0

#elif BOARD_MAINOSC_FREQUENCY == (24000000)
#  define DSP_CK_ND_12M288     27
#  define DSP_CK_FRACR_12M288  2818572
#  define DSP_CK_QDPMC_12M288  6
#  define DSP_CK_DIV_12M288    0
#  define DSP_QDAUDIO_12M288   0
#  define DSP_CK_ND_11M1896    29
#  define DSP_CK_FRACR_11M1896 442918
#  define DSP_CK_QDPMC_11M1896 7
#  define DSP_CK_DIV_11M1896   0
#  define DSP_QDAUDIO_11M1896  0
#  define DSP_CK_ND_5M6498     25
#  define DSP_CK_FRACR_5M6498  1436129
#  define DSP_CK_QDPMC_5M6498  13
#  define DSP_CK_DIV_5M6498    0
#  define DSP_QDAUDIO_5M6498   0
#else
#  error Unsupported board frequency for Class D clock configuration
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CLASSD driver state */

enum classd_state
{
  CLASSD_STATE_UNINIT = 0,    /* Not yet initialized */
  CLASSD_STATE_RESET,         /* Initialized, reset state */
  CLASSD_STATE_CONFIGURED,    /* classd_configure() has been called */
};

enum sam_classd_pwm_type
{
  CLASSD_PWM_TYPE_TRAILING = 0,
  CLASSD_PWM_TYPE_UNIFORM,
};

enum sam_classd_overlap_value
{
  CLASSD_OVERLAP_5NS = 0,
  CLASSD_OVERLAP_10NS,
  CLASSD_OVERLAP_15NS,
  CLASSD_OVERLAP_20NS,
};

enum sam_classd_dsp_frequency
{
  CLASSD_DSP_FREQ_12M288 = 0,
  CLASSD_DSP_FREQ_11M2896,
  CLASSD_DSP_FREQ_5M6448,
};

/* NB: 11.025kHz is an unsupported frequency */

enum sam_classd_frame_rate
{
  CLASSD_DATA_SAMPLE_FREQ_8K = 0,
  CLASSD_DATA_SAMPLE_FREQ_16K,
  CLASSD_DATA_SAMPLE_FREQ_32K,
  CLASSD_DATA_SAMPLE_FREQ_48K,
  CLASSD_DATA_SAMPLE_FREQ_96K,
  CLASSD_DATA_SAMPLE_FREQ_22K05,
  CLASSD_DATA_SAMPLE_FREQ_44K1,
  CLASSD_DATA_SAMPLE_FREQ_88K2,
  CLASSD_DATA_SAMPLE_FREQ_11K025 = CLASSD_DATA_SAMPLE_FREQ_22K05,
};

enum sam_classd_eq_mode
{
  CLASSD_EQ_FLAT = 0,
  CLASSD_EQ_BASS_BOOST_12DB,
  CLASSD_EQ_BASS_BOOST_6DB,
  CLASSD_EQ_BASS_CUT_12DB,
  CLASSD_EQ_BASS_CUT_6DB,
  CLASSD_EQ_MEDIUM_BOOST_3DB,
  CLASSD_EQ_MEDIUM_BOOST_8DB,
  CLASSD_EQ_MEDIUM_CUT_3DB,
  CLASSD_EQ_MEDIUM_CUT_8DB,
  CLASSD_EQ_TREBLE_BOOST_12DB,
  CLASSD_EQ_TREBLE_BOOST_6DB,
  CLASSD_EQ_TREBLE_CUT_12DB,
  CLASSD_EQ_TREBLE_CUT_6DB,
};

enum sam_classd_mono_mode
{
  CLASSD_MONO_MODE_MIX = 0, /* (left+right)/2 on both channels */
  CLASSD_MONO_MODE_SAT,     /* (left+roght)/2 on both channels
                             * if sum is too high the result is saturated
                             */
  CLASSD_MONO_MODE_LEFT,    /* left channel sent to both */
  CLASSD_MONO_MODE_RIGHT,   /* left channel sent to both */
};

/* This structure provides the config. data for the CLASSD peripheral */

struct sam_classd_config_s
{
  bool                          left_channel_enabled;
  bool                          right_channel_enabled;
  bool                          left_channel_muted;
  bool                          right_channel_muted;
  enum sam_classd_pwm_type      pwm_type;
  bool                          non_overlap_mode;
  enum sam_classd_overlap_value non_overlap_value;
  enum sam_classd_dsp_frequency dsp_clock_frequency;
  bool                          de_emphasis_enabled;
  bool                          left_and_right_swapped;
  uint32_t                      sample_rate; /* Configured sample rate      */
  enum sam_classd_frame_rate    frame;       /* Calculated frame rate       */
  enum sam_classd_eq_mode       eq_mode;
  bool                          mono_mode_selected;
  enum sam_classd_mono_mode     mono_mode;
  uint16_t                      volume;
  uint16_t                      balance;
  uint8_t                       nchanns; /* Number of channels (1 or 2)     */
  uint8_t                       nbits;   /* Number of bits/sample 8 or 16)  */
};

struct classd_dev_s
{
  /* We are an audio lower half driver, also the upper "half" of
   * the driver with respect to the lower half driver.
   * Terminology:
   * Our "lower" half audio instances will be called dev for the publicly
   * visible version and "priv" for the version that only this driver knows.
   * From the point of view of this driver, it is the board lower "half"
   * that is referred to as "lower"
   */

  struct audio_lowerhalf_s dev;        /* Audio lower half (this device)    */
  struct dq_queue_s        pendq;      /* Queue of pending buffers          */
  struct dq_queue_s        doneq;      /* Queue of sent buffers             */
  struct file              mq;         /* Message queue for messages        */
  char                     mqname[16]; /* Our message queue name            */
  pthread_t                threadid;   /* ID of our thread */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  volatile bool            terminate;  /* True: request to terminate        */
#endif
  bool                     running;    /* True: Worker thread is running    */
  bool                     paused;     /* True: Playing is paused           */
  bool                     mute;       /* True: Output is muted             */
#ifdef CONFIG_SAMA5D2_CLASSD_REGDEBUG
  uintptr_t                regaddr;    /* Last register address read        */
  uint32_t                 regval;     /* Last value read from the register */
#endif
  uint8_t                  state;      /* See classd_state                  */
  mutex_t                  pendlock;   /* Protect pendq                     */
  volatile uint8_t         inflight;   /* Number of audio buffers in-flight */
  bool                     reserved;   /* True: Device is reserved          */
  struct sam_classd_config_s *config;  /* Configuration                     */
  sem_t                    dmawait;    /* Used to wait for DMA completion   */
  struct wdog_s            dmadog;     /* Watchdog that for DMA timeouts    */
  int                      dmares;     /* DMA result */
  DMA_HANDLE               txdma;      /* SPI TX DMA handle */
#ifdef CONFIG_SAMA5D2_CLASSD_DMADEBUG
  struct sam_dmaregs_s    txdmaregs[DMA_NSAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

void classd_dsp_clock_config(struct classd_dev_s *priv);
void classd_enable_audio(struct classd_dev_s *priv, bool pmc_clock_enable);
void classd_configure_pmc(struct classd_dev_s *priv);
void classd_disable_audio(struct classd_dev_s *priv);
void classd_set_mode(struct classd_dev_s *priv);
void classd_set_interpolator(struct classd_dev_s *priv);
static int classd_setup(struct classd_dev_s *priv);
static uint32_t classd_getreg(struct classd_dev_s *priv, uint32_t regaddr);
static void classd_putreg(uint32_t regaddr, uint32_t regval);
#ifdef CONFIG_SAMA5D2_CLASSD_REGDEBUG
static void classd_dump_registers(const char *msg);
#else
#  define classd_dump_registers(msg);
#endif 

/* Audio lower half functions */

static int classd_getcaps(struct audio_lowerhalf_s *dev, int type,
                          struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_configure(struct audio_lowerhalf_s *dev, void *session,
                            const struct audio_caps_s *caps);
#else
static int classd_configure(struct audio_lowerhalf_s *dev,
                            const struct audio_caps_s *caps);
#endif
static int classd_shutdown(struct audio_lowerhalf_s *dev);
static void *classd_workerthread(pthread_addr_t pvarg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_start(struct audio_lowerhalf_s *dev, void *session);
#else
static int classd_start(struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_stop(struct audio_lowerhalf_s *dev, void *session);
#else
static int classd_stop(struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_pause(struct audio_lowerhalf_s *dev, void *session);
static int classd_resume(struct audio_lowerhalf_s *dev, void *session);
#else
static int classd_pause(struct audio_lowerhalf_s *dev);
static int classd_resume(struct audio_lowerhalf_s *dev);
#endif
#endif
static int classd_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                struct ap_buffer_s *apb);
static int classd_cancelbuffer(struct audio_lowerhalf_s *dev,
                               struct ap_buffer_s *apb);
static int classd_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_reserve(struct audio_lowerhalf_s *dev,
                          void **session);
#else
static int classd_reserve(struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_release(struct audio_lowerhalf_s *dev, void *session);
#else
static int classd_release(struct audio_lowerhalf_s *dev);
#endif
static void classd_reset(struct classd_dev_s *priv);
inline static uint8_t classd_get_atten(uint16_t volume, uint16_t balance);
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int classd_setvolume(struct classd_dev_s *priv, uint16_t volume);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
static int classd_setbalance(struct classd_dev_s *priv, uint16_t balance);
#endif
static int classd_sendbuffer(struct classd_dev_s *priv);
static void classd_returnbuffers(struct classd_dev_s *priv);

/* DMA support */

static void classd_txcallback(DMA_HANDLE handle, void *arg, int result);

#ifdef CONFIG_SAMA5D2_CLASSD_DMADEBUG
  struct sam_dmaregs_s txdmaregs[DMA_NSAMPLES];
#  define classd_txdma_sample(s,i) sam_dmasample((s)->txdma, &(s)->txdmaregs[i])
static void classd_dma_sampleinit(struct classd_dev_s *priv);
static void classd_dma_sampledone(struct classd_dev_s *priv);
#else
#  define classd_txdma_sample(s,i)
#  define classd_dma_sampleinit(s)
#  define classd_dma_sampledone(s)
#endif

static void classd_txcallback(DMA_HANDLE handle, void *arg, int result);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* default/constant configuration */

static struct sam_classd_config_s sam_classd_const =
{
#ifdef CONFIG_SAMA5D2_CLASSD_LEN
  .left_channel_enabled = true,
#else
  .left_channel_enabled = false,
#endif
#ifdef CONFIG_SAMA5D2_CLASSD_REN
  .right_channel_enabled = true,
#else
  .right_channel_enabled = false,
#endif
  .left_channel_muted = false,
  .right_channel_muted = false,
#ifdef CONFIG_SAMA5D2_CLASSD_PWM_UNIFORM
  .pwm_type = CLASSD_PWM_TYPE_UNIFORM,
#else
  .pwm_type = CLASSD_PWM_TYPE_TRAILING,
#endif
#ifdef CONFIG_SAMA5D2_CLASSD_NON_OVERLAP_MODE
  .non_overlap_mode = true,
#  if defined(CONFIG_SAMA5D2_CLASSD_NON_OVERLAP_5NS)
  .non_overlap_value = CLASSD_OVERLAP_5NS,
#  elif defined(CONFIG_SAMA5D2_CLASSD_NON_OVERLAP_10NS)
  .non_overlap_value = CLASSD_OVERLAP_10NS,
#  elif defined(CONFIG_SAMA5D2_CLASSD_NON_OVERLAP_15NS)
  .non_overlap_value = CLASSD_OVERLAP_15NS,
#  elif defined(CONFIG_SAMA5D2_CLASSD_NON_OVERLAP_20NS)
  .non_overlap_value = CLASSD_OVERLAP_20NS,
#  else
#  error Invalid SAMA5D2 Overlap value
#endif
#else
  .non_overlap_mode = false,
  .non_overlap_value = CLASSD_OVERLAP_5NS,
#endif
  .dsp_clock_frequency = CLASSD_DSP_FREQ_12M288,
#ifdef CONFIG_SAMA5D2_CLASSD_DEEMP
  .de_emphasis_enabled = 1,
#else
  .de_emphasis_enabled = 0,
#endif
#ifdef CONFIG_SAMA5D2_CLASSD_SWAP
  .left_and_right_swapped = 1,
#else
  .left_and_right_swapped = 0,
#endif

#if defined(CONFIG_SAMA5D2_CLASSD_FRAME_8K)
  .sample_rate = 8000,
  .frame = CLASSD_DATA_SAMPLE_FREQ_8K,
#  elif defined(CONFIG_SAMA5D2_CLASSD_FRAME_16K)
  .sample_rate = 16000,
  .frame = CLASSD_DATA_SAMPLE_FREQ_16K,
#  elif defined(CONFIG_SAMA5D2_CLASSD_FRAME_32K)
  .sample_rate = 32000,
  .frame = CLASSD_DATA_SAMPLE_FREQ_32K,
#  elif defined(CONFIG_SAMA5D2_CLASSD_FRAME_48K)
  .sample_rate = 48000,
  .frame = CLASSD_DATA_SAMPLE_FREQ_48K,
#  elif defined(CONFIG_SAMA5D2_CLASSD_FRAME_96K)
  .sample_rate = 96000,
  .frame = CLASSD_DATA_SAMPLE_FREQ_96K,
#  elif defined(CONFIG_SAMA5D2_CLASSD_FRAME_22K05)
  .sample_rate = 22050,
  .frame = CLASSD_DATA_SAMPLE_FREQ_22K05,
#  elif defined(CONFIG_SAMA5D2_CLASSD_FRAME_44K1)
  .sample_rate = 44100,
  .frame = CLASSD_DATA_SAMPLE_FREQ_44K1,
#  elif defined(CONFIG_SAMA5D2_CLASSD_FRAME_88K2)
  .sample_rate = 88200,
  .frame = CLASSD_DATA_SAMPLE_FREQ_88K2,
#  else
#error Invalid SAMA5D2 ClassD frame rate
#endif

#if defined(CONFIG_SAMA5D2_CLASSD_EQ_FLAT)
  .eq_mode = CLASSD_EQ_FLAT,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_BB12)  
  .eq_mode = CLASSD_EQ_BASS_BOOST_12DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_BB6)  
  .eq_mode = CLASSD_EQ_BASS_BOOST_6DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_BC12)
  .eq_mode = CLASSD_EQ_BASS_CUT_12DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_BC6)
  .eq_mode = CLASSD_EQ_BASS_CUT_12DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_MB3)
  .eq_mode = CLASSD_EQ_MEDIUM_BOOST_3DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_MB8)
  .eq_mode = CLASSD_EQ_MEDIUM_BOOST_8DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_MC3)
  .eq_mode = CLASSD_EQ_MEDIUM_CUT_3DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_MC8)
  .eq_mode = CLASSD_EQ_MEDIUM_CUT_8DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_TB12)
  .eq_mode = CLASSD_EQ_TREBLE_BOOST_12DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_TB6)
  .eq_mode = CLASSD_EQ_TREBLE_BOOST_6DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_TC12)
  .eq_mode = CLASSD_EQ_TREBLE_CUT_12DB,
#  elif defined(CONFIG_SAMA5D2_CLASSD_EQ_TC6)
  .eq_mode = CLASSD_EQ_TREBLE_CUT_6DB,
#  else
#error Invalid SAMA5D2 ClassD equaliser setting
#endif

#ifdef CONFIG_SAMA5D2_CLASSD_MONO
  .mono_mode_selected = true,
#  if defined(CONFIG_SAMA5D2_CLASSD_MONOMODE_MIX)
  .mono_mode = CLASSD_MONO_MODE_MIX,
#  elif defined(CONFIG_SAMA5D2_CLASSD_MONOMODE_SAT)
  .mono_mode = CLASSD_MONO_MODE_SAT,
#  elif defined(CONFIG_SAMA5D2_CLASSD_MONO_MODE_LEFT)
  .mono_mode = CLASSD_MONO_MODE_LEFT,
#  elif defined(CONFIG_SAMA5D2_CLASSD_MONO_MODE_RIGHT)
  .mono_mode = CLASSD_MONO_MODE_RIGHT,
#  else
#error Invalids SAMA5D2 ClassD Mono mode
#endif
#else
  .mono_mode_selected = false,
  .mono_mode = CLASSD_MONO_MODE_MIX,
#endif
  .balance = 500,
  .volume = 0,
  .nbits = 16,
  .nchanns = 2,
};

static const struct audio_ops_s g_audioops =
{
  classd_getcaps,       /* getcaps        */
  classd_configure,     /* configure      */
  classd_shutdown,      /* shutdown       */
  classd_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  classd_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  classd_pause,         /* pause          */
  classd_resume,        /* resume         */
#endif
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  classd_enqueuebuffer, /* enqueue_buffer */
  classd_cancelbuffer,  /* cancel_buffer  */
  classd_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  classd_reserve,       /* reserve        */
  classd_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: classd_configure_pmc
 *
 * Description:
 *   Enable ClassD audio  PMC
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void classd_configure_pmc(struct classd_dev_s *priv)
{
  uint32_t regval;

  regval = PMC_PCR_GCKEN | PMC_PCR_CMD | PMC_PCR_EN |
           PMC_GCK_DIV(0) | PMC_PCR_GCKCSS_AUDIO;
  regval |= PMC_PCR_PID(SAM_PID_CLASSD);
  putreg32(regval, SAM_PMC_PCR);
}

/****************************************************************************
 * Name: classd_disable_audio
 *
 * Description:
 *   Disable ClassD audio
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state.
 *
 * Returned Value:
 *
 ****************************************************************************/

void classd_disable_audio(struct classd_dev_s *priv)
{
#ifdef PMC_AUDIO_PLL0_PLLEN
  uint32_t regval;

  regval = classd_getreg(priv, SAM_PMC_AUDIO_PLL0);
  regval &= ~(PMC_AUDIO_PLL0_PLLEN | PMC_AUDIO_PLL0_PADEN
            | PMC_AUDIO_PLL0_PMCEN);
  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);
#endif
}

/****************************************************************************
 * Name: classd_enable_audio
 *
 * Description:
 *   Enable ClassD audio
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state.
 *   pmc_clock_enable:
 *     true:  the output clock of the audio PLL is sent to the PMC
 *     false: the output clock of the audio PLL is NOT sent to the PMC
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void classd_enable_audio(struct classd_dev_s *priv, bool pmc_clock_enable)
{
#ifdef PMC_AUDIO_PLL0_PLLEN
  uint32_t regval;

  regval = classd_getreg(priv, SAM_PMC_AUDIO_PLL0);
  regval &= ~(PMC_AUDIO_PLL0_PADEN | PMC_AUDIO_PLL0_PMCEN);
  regval |= PMC_AUDIO_PLL0_PLLEN;
#ifdef CONFIG_SAMA5D2_CLASSD_PAD_CLK
  regval |= PMC_AUDIO_PLL0_PADEN;
#endif
  if (pmc_clock_enable)
    {
      regval |= PMC_AUDIO_PLL0_PMCEN;
    }

  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

  /* wait for Audio PLL startup time */

  usleep(100);
#endif
}

/****************************************************************************
 * Name: classd_set_mode
 *
 * Description:
 *   Set overall operation mode
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void classd_set_mode(struct classd_dev_s *priv)
{
  uint32_t regval;
  struct sam_classd_config_s *config = priv->config;

  regval = classd_getreg(priv, SAM_CLASSD_MR);

  if (config->left_channel_enabled)
    {
      regval |= CLASSD_MR_LEN_BIT;
    }
  else
    {
      regval &= ~CLASSD_MR_LEN_BIT;
    }

  if (config->right_channel_enabled)
    {
      regval |= CLASSD_MR_REN_BIT;
    }
  else
    {
      regval &= ~CLASSD_MR_REN_BIT;
    }

  if (config->left_channel_muted)
    {
      regval |= CLASSD_MR_LMUTE_BIT;
    }
  else
    {
      regval &= ~CLASSD_MR_LMUTE_BIT;
    }

  if (config->right_channel_muted)
    {
      regval |= CLASSD_MR_RMUTE_BIT;
    }
  else
    {
      regval &= ~CLASSD_MR_RMUTE_BIT;
    }

  if (config->pwm_type == CLASSD_PWM_TYPE_UNIFORM)
    {
      regval |= CLASSD_MR_PWMTYP_BIT;
    }
  else
    {
      regval &= ~CLASSD_MR_PWMTYP_BIT;
    }

  if (config->non_overlap_mode)
    {
      regval |= CLASSD_MR_NOVR_BIT;
      regval &= ~CLASSD_MR_NOVRVAL_MASK;
      regval |= CLASSD_MR_NOVR(config->non_overlap_value);
    }
  else
    {
      regval &= ~CLASSD_MR_NOVR_BIT;
    }

  classd_putreg(SAM_CLASSD_MR, regval);

  classd_dump_registers("After mode set");
}

/****************************************************************************
 * Name: classd_set_interpolator
 *
 * Description:
 *   Set sample rate and other interpolator configuration
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void classd_set_interpolator(struct classd_dev_s *priv)
{
  uint32_t regval;
  struct sam_classd_config_s *config = priv->config;
  uint32_t samp_freq = config->sample_rate;

  switch (samp_freq)
    {
      case 8000:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_8K;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_12M288;
        }
        break;
      case 11025:
        {
          /* NB: 11.025kHz is an unsupported frequency, but can be made to
           * work, albeit with some audio distortion.
           */

          config->frame = CLASSD_DATA_SAMPLE_FREQ_11K025;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_5M6448;
        }
        break;
      case 16000:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_16K;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_12M288;
        }
        break;
      case 32000:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_32K;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_12M288;
        }
        break;
      case 48000:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_48K;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_12M288;
        }
        break;
      case 96000:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_96K;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_12M288;
        }
        break;
      case 22050:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_22K05;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_11M2896;
        }
        break;
      case 44100:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_44K1;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_11M2896;
        }
        break;
      case 88200:
        {
          config->frame = CLASSD_DATA_SAMPLE_FREQ_88K2;
          config->dsp_clock_frequency = CLASSD_DSP_FREQ_11M2896;
        }
        break;
      default:
        auderr("ERROR: Invalid interpolator settings\n");
        break;
    }

  regval = classd_getreg(priv, SAM_CLASSD_INTPMR);

  regval &= ~CLASSD_INTPMR_FRAME_MASK;
  regval |= CLASSD_INTRMR_FRAME(config->frame);

  if (config->dsp_clock_frequency == CLASSD_DSP_FREQ_12M288)
    {
      regval &= ~CLASSD_INTPMR_DSPCLKF_BIT;
    }
  else
    {
      /* NB. This is also used for non-supported 11.025kHz sample f */

      regval |= CLASSD_INTPMR_DSPCLKF_BIT;
    }

  if (config->de_emphasis_enabled)
    {
      regval |= CLASSD_INTPMR_DEEMP_BIT;
    }
  else
    {
      regval &= ~CLASSD_INTPMR_DEEMP_BIT;
    }

  if (config->left_and_right_swapped)
    {
      regval |= CLASSD_INTPMR_SWAP_BIT;
    }
  else
    {
      regval &= ~CLASSD_INTPMR_SWAP_BIT;
    }

  regval &= ~CLASSD_INTPMR_EQCFG_MASK;
  regval |= CLASSD_INTRMR_EQCFG(config->eq_mode);

  if (config->mono_mode_selected)
    {
      regval |= CLASSD_INTPMR_MONO_BIT;
      regval &= ~CLASSD_INTPMR_MONOMODE_MASK;
      regval |= CLASSD_INTRMR_MONOMODE(config->mono_mode);
    }
  else
    {
      regval &= ~CLASSD_INTPMR_MONO_BIT;
    }

  classd_putreg(SAM_CLASSD_INTPMR, regval);
  classd_dump_registers("After interpolator setup");
}

/****************************************************************************
 * Name: classd_dsp_clock_config
 *
 * Description:
 *   Configure the ClassD PLL clocks.
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state.
 *
 * Returned Value:
 *
 ****************************************************************************/

void classd_dsp_clock_config(struct classd_dev_s *priv)
{
  uint32_t regval;
  struct sam_classd_config_s *config;
#ifdef CONFIG_CLASSD_REGDEBUG
  /* double check clock settings */

  uint32_t nd;
  uint32_t fracr;
  uint32_t qdpmc;
  uint64_t clk = BOARD_MAINOSC_FREQUENCY;
  uint32_t pll0 = classd_getreg(priv, SAM_PMC_AUDIO_PLL0);
  uint32_t pll1 = classd_getreg(priv, SAM_PMC_AUDIO_PLL1);
#endif

  config = priv->config;

  /* reset audio clock */

  regval = classd_getreg(priv, SAM_PMC_AUDIO_PLL0);
  regval &= ~(PMC_AUDIO_PLL0_RESETN | PMC_AUDIO_PLL0_PLLEN);
  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

  regval |= PMC_AUDIO_PLL0_RESETN;
  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

  /* Configure values */

  switch (config->dsp_clock_frequency)
    {
      case CLASSD_DSP_FREQ_5M6448:
        {
          /* This is a non-standard, unsupported, mode.
           * We set up the PLL to use 22.05Khz filtering but clock it
           * at half the speed (5.6498MHz).
           */

          regval = PMC_AUDIO_PLL0_ND(DSP_CK_ND_5M6498)          |
                   PMC_AUDIO_PLL0_QDPMC(DSP_CK_QDPMC_5M6498)    |
                   PMC_AUDIO_PLL0_FLT(PMC_AUDIO_PLL0_PLLFLT_STD) |
                   PMC_AUDIO_PLL0_RESETN;
          classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

          regval = PMC_AUDIO_PLL1_FRACR(DSP_CK_FRACR_5M6498)  |
                   PMC_AUDIO_PLL1_DIV(DSP_CK_DIV_5M6498)      |
                   PMC_AUDIO_PLL1_QDAUDIO(DSP_QDAUDIO_5M6498) |
                   PMC_AUDIO_PLL1_DIV(2);
          classd_putreg(SAM_PMC_AUDIO_PLL1, regval);
        }
        break;
      case CLASSD_DSP_FREQ_11M2896:
        {
          regval = PMC_AUDIO_PLL0_ND(DSP_CK_ND_11M1896) |
                   PMC_AUDIO_PLL0_QDPMC(DSP_CK_QDPMC_11M1896) |
                   PMC_AUDIO_PLL0_FLT(PMC_AUDIO_PLL0_PLLFLT_STD) |
                   PMC_AUDIO_PLL0_RESETN;
          classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

          regval = PMC_AUDIO_PLL1_FRACR(DSP_CK_FRACR_11M1896)  |
                   PMC_AUDIO_PLL1_DIV(DSP_CK_DIV_11M1896)      |
                   PMC_AUDIO_PLL1_QDAUDIO(DSP_QDAUDIO_11M1896) |
                   PMC_AUDIO_PLL1_DIV(2);
          classd_putreg(SAM_PMC_AUDIO_PLL1, regval);
        }
        break;
      default: /* CLASSD_DSP_FREQ_12M288 */
        {
          regval = PMC_AUDIO_PLL0_ND(DSP_CK_ND_12M288) |
                   PMC_AUDIO_PLL0_QDPMC(DSP_CK_QDPMC_12M288) |
                   PMC_AUDIO_PLL0_FLT(PMC_AUDIO_PLL0_PLLFLT_STD) |
                   PMC_AUDIO_PLL0_RESETN;
          classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

          regval = PMC_AUDIO_PLL1_FRACR(DSP_CK_FRACR_12M288)  |
                   PMC_AUDIO_PLL1_DIV(DSP_CK_DIV_12M288)      |
                   PMC_AUDIO_PLL1_QDAUDIO(DSP_QDAUDIO_12M288) |
                   PMC_AUDIO_PLL1_DIV(2);
          classd_putreg(SAM_PMC_AUDIO_PLL1, regval);
        }
        break;
    }
#endif

  classd_dump_registers("After dsp clock setup");

#ifdef CONFIG_CLASSD_REGDEBUG
  nd = (pll0 & PMC_AUDIO_PLL0_ND_MASK) >> PMC_AUDIO_PLL0_ND_SHIFT;
  fracr = (pll1 & PMC_AUDIO_PLL1_FRACR_MASK) >> PMC_AUDIO_PLL1_FRACR_SHIFT;
  qdpmc = (pll0 & PMC_AUDIO_PLL0_QDPMC_MASK) >> PMC_AUDIO_PLL0_QDPMC_SHIFT;

  clk *= ((nd + 1) << 22) + fracr;
  clk /= 1 << 22;
  clk /= (qdpmc + 1);
  clk /= 8;

  audinfo("INFO: audio pll clk = %" PRId64 "\n", clk);
#endif
}

/****************************************************************************
 * Name: classd_setup
 *
 * Description:
 *   Configure the ClassD peripheral.
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state.
 *
 * Returned Value:
 * OK (or not)
 *
 ****************************************************************************/

static int classd_setup(struct classd_dev_s *priv)
{
  uint32_t regval;

  classd_set_mode(priv);
  classd_set_interpolator(priv);
  classd_dsp_clock_config(priv);

  regval = classd_getreg(priv, SAM_CLASSD_INTSR);
  if (regval != 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: classd_getreg
 *
 * Description:
 *   Read the value of a CLASSD register.
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   Value of the register requested
 *
 ****************************************************************************/

static uint32_t classd_getreg(struct classd_dev_s *priv, uint32_t regaddr)
{
  return getreg32(regaddr);
}

/****************************************************************************
 * Name: classd_putreg
 *
 * Description:
 *   Set the value of a CLASSD register.
 *
 * Input Parameters:
 *   regaddr - The register to write
 *   regval - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void classd_putreg(uint32_t regaddr, uint32_t regval)
{
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: classd_dump_registers
 *
 * Description:
 *   Dump the contents of all CLASSD control registers
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5D2_CLASSD_REGDEBUG
static void classd_dump_registers(const char *msg)
{
  audinfo("CLASSD Registers: %s\n", msg);
  putreg32(PMC_PCR_PID(SAM_PID_CLASSD), SAM_PMC_PCR);
  audinfo("\tPMC_PCR %" PRIx32 "\n", getreg32(SAM_PMC_PCR));
  audinfo(" PLL0 %" PRIx32 "\t\tPLL1 %" PRIx32 "\n",
            getreg32(SAM_PMC_AUDIO_PLL0), getreg32(SAM_PMC_AUDIO_PLL1));
  audinfo("\tCR:  %"  PRIx32 "\tMR: %" PRIx32 "\tintMR %" PRIx32 "\tintSR %"\
  PRIx32 "\t\tTHR  %" PRIx32 "\n",
          getreg32(SAM_CLASSD_CR),
          getreg32(SAM_CLASSD_MR),
          getreg32(SAM_CLASSD_INTPMR),
          getreg32(SAM_CLASSD_INTSR),
          getreg32(SAM_CLASSD_THR));
  audinfo("\tIDR: %" PRIx32 "\tIMR %" PRIx32 "\t\tISR %"\
  PRIx32 "\t\tWPMR %" PRIx32 "\n",
          getreg32(SAM_CLASSD_IDR),
          getreg32(SAM_CLASSD_IMR),
          getreg32(SAM_CLASSD_ISR),
          getreg32(SAM_CLASSD_WPMR));
}
#endif

/****************************************************************************
 * Name: classd_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers
 *     (if CONFIG_SAMA5D2_CLASSD_DMADEBUG)
 *
 * Input Parameters:
 *   *priv - Pointer to device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5D2_CLASSD_DMADEBUG
static void classd_dma_sampleinit(struct classd_dev_s *priv)
{
  /* Put contents of register samples into a known state */

  memset(priv->txdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(priv->txdma, &priv->txdmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: classd_dma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   priv - Pointer to device configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5D2_CLASSD_DMADEBUG
static void classd_dma_sampledone(struct classd_dev_s *priv)
{
  /* Sample the final registers */

  sam_dmasample(priv->txdma, &priv->txdmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */

  /* Initial register values */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_INITIAL],
              "TX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_AFTER_SETUP],
              "TX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_AFTER_START],
              "TX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timedout, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_CALLBACK],
              "TX: At DMA callback");

  /* Register values at the end of the DMA */

  sam_dmadump(priv->txdma, &priv->txdmaregs[DMA_END_TRANSFER],
              "TX: At End-of-Transfer");
}
#endif /* CONFIG_SAMA5D2_CLASSD_DMADEBUG */

/****************************************************************************
 * Name: classd_dmatimeout
 *
 * Description:
 *   The watchdog timeout setup when a has expired without completion of a
 *   DMA.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void classd_dmatimeout(wdparm_t arg)
{
  struct classd_dev_s *priv = (struct classd_dev_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Sample DMA registers at the time of the timeout */

  classd_txdma_sample(priv, DMA_CALLBACK);

  sam_dmastop(priv->txdma);

  /* Report timeout result, perhaps overwriting any failure reports from
   * the TX callback.
   */

  priv->dmares = -ETIMEDOUT;

  /* Then wake up the waiting thread */

  nxsem_post(&priv->dmawait);
}

/****************************************************************************
 * Name: classd_txcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the CLASSD DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void classd_txcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct classd_dev_s *priv = (struct classd_dev_s *)arg;
  DEBUGASSERT(priv != NULL && priv->running);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->dmadog);

  classd_txdma_sample(priv, DMA_CALLBACK);

  priv->dmares = result;

  nxsem_post(&priv->dmawait);
}

/****************************************************************************
 * Name: classd_reset
 *
 * Description:
 *   Reset peripheral to default state
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void classd_reset(struct classd_dev_s *priv)
{
  DEBUGASSERT(priv);

  audinfo("CLASSD reset\n");

  /* get exclusive access to the CLASSD peripheral */

  nxmutex_lock(&priv->pendlock);

  /* disable all interrupts and HW reset */

  classd_putreg(SAM_CLASSD_IER, 0);
  classd_putreg(SAM_CLASSD_CR, CLASSD_CR_SWRST_BIT);

  /* Release anything pending - needed? How? */

  priv->state = CLASSD_STATE_RESET;

  nxmutex_unlock(&priv->pendlock);
}

/****************************************************************************
 * Name: classd_get_atten
 *
 * Description
 *   Calculate the right and left attenuation values based on the
 *   volume and balance settings.
 *
 *   The range is limited to 0..78 since any value <77 wil be treated as
 *   mutes.
 *
 *  Input Parameters:
 *    volume - The required volume, 0-1000
 *    balnce - The required balance, 0-500 (500 is centre)
 *
 * Returned Value:
 *   Scaled and range limited attenuation value
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
inline static uint8_t classd_get_atten(uint16_t volume, uint16_t balance)
{
  int scaled_vol;
  scaled_vol = 78 - ((78 * volume * balance) / (AUDIO_VOLUME_MAX *
                                                AUDIO_BALANCE_CENTER));
  if (scaled_vol < 0)
    {
      return 0;
    }
  else

  return (uint8_t)scaled_vol & 0x7f;
}
#endif

/****************************************************************************
 * Name: classd_setvolume
 *
 * Description:
 *   Set the required volume level, taking balance into account
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state
 *   vol - the requirede volume, range 0-1000
 *
 * Returned Value:
 *   OK or -EDOM if requested volume is out of range
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int classd_setvolume(struct classd_dev_s *priv, uint16_t vol)
{
  struct sam_classd_config_s *config;
  uint32_t levl;
  uint32_t levr;
  uint32_t bal;
  uint32_t regval;

  if (vol < AUDIO_VOLUME_MIN || vol > AUDIO_VOLUME_MAX)
    {
      return -EDOM;
    }

  config = priv->config;

  config->volume = vol;
  bal = config->balance;

  /* Calculate the attenuation value to send to the peripheral */

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  levl = classd_get_atten(vol, AUDIO_BALANCE_RIGHT - bal);
  levr = classd_get_atten(vol, bal);
#else
  levl = classd_get_atten(volume, AUDIO_BALANCE_CENTER);
  levr = classd_get_atten(volume, AUDIO_BALANCE_CENTER);
#endif

  /* Set the volume */

  regval = classd_getreg(priv, SAM_CLASSD_INTPMR);
  regval &= ~CLASSD_INTPMR_ATTL_MASK;
  regval &= ~CLASSD_INTPMR_ATTR_MASK;
  regval |= CLASSD_VOL_LEFT(levl);
  regval |= CLASSD_VOL_RIGHT(levr);

  classd_putreg(SAM_CLASSD_INTPMR, regval);

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/****************************************************************************
 * Name: classd_balance
 *
 * Description:
 *   Set the balance for the CLASSD device.
 *   There is no built in balance feature so we save the value, which will
 *   be used during volume setting to offset each channel's volume.
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state
 *   bal - the requirede volume, range 0-1000
 *
 * Returned Value:
 *   OK or -EDOM if value is out of range
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
static int classd_setbalance(struct classd_dev_s *priv, uint16_t bal)
{
  struct sam_classd_config_s *config;

  if (bal < AUDIO_BALANCE_LEFT || bal > AUDIO_BALANCE_RIGHT)
    {
      return -EDOM;
    }

  config = priv->config;
  config->balance = bal;

  classd_setvolume(priv, config->volume);

  return OK;
}
#endif

/****************************************************************************
 * Name: classd_getcaps
 *
 * Description: Get the audio device capabilities
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral state
 *   type - The type of capability being requested
 *   caps - struct of capabilities to be returned
 *
 * Returned Value:
 *   OK or -EDOM if value is out of range
 *
 ****************************************************************************/

static int classd_getcaps(struct audio_lowerhalf_s *dev, int type,
                          struct audio_caps_s *caps)
{
  audinfo("type=%d\n", type);

  /* Validate the structure */

  DEBUGASSERT(caps->ac_len >= sizeof(struct audio_caps_s));

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;
  caps->ac_controls.b[0] = 0;
  caps->ac_controls.b[1] = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT |
                                       AUDIO_TYPE_FEATURE ;

              break;

            case AUDIO_FMT_PCM:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_PCM_S16_LE;
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_PCM_S16_LE;
              break;
          }

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = 2; /* stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.b[0] = (uint8_t)(AUDIO_SAMP_RATE_8K  |
                                                 AUDIO_SAMP_RATE_11K |
                                                 AUDIO_SAMP_RATE_16K |
                                                 AUDIO_SAMP_RATE_22K |
                                                 AUDIO_SAMP_RATE_32K |
                                                 AUDIO_SAMP_RATE_44K |
                                                 AUDIO_SAMP_RATE_48K |
                                                 AUDIO_SAMP_RATE_88K |
                                                 AUDIO_SAMP_RATE_96K) ;
              break;

            default:
              break;
          }

        break;

      /* Provide capabilities of our FEATURE units */

      case AUDIO_TYPE_FEATURE:
        /* If the sub-type is UNDEF,
         * then report the Feature Units we support
         */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with
             * the Feature Units we have
             */

            caps->ac_controls.b[0] = AUDIO_FU_MUTE |
                                     AUDIO_FU_VOLUME;
            caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
          }
        else
          {
            /* TODO:  Do we need to provide specific info for the
             * Feature Units, such as volume setting ranges, etc.?
             */
          }

      break;

      /* All others we don't support */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype = 0;
        caps->ac_channels = 0;

      break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  audinfo("Return %d\n", caps->ac_len);
  return caps->ac_len;
}

/****************************************************************************
 * Name: classd_configure
 *
 * Description:
 *   Configure the audio device for the specified mode of operation.
 *
 * Input Parameters:
 *   priv    - A reference to the CLASSD peripheral
 *   session - The audio session
 *   caps    - struct of capabilities to be configured
 *
 * Returned Value:
 *   OK or negated error value
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_configure(struct audio_lowerhalf_s *dev,
                            void *session, const struct audio_caps_s *caps)
#else
static int classd_configure(struct audio_lowerhalf_s *dev,
                            const struct audio_caps_s *caps)
#endif
{
  int ret = OK;

  struct classd_dev_s *priv = (struct classd_dev_s *)dev;
  DEBUGASSERT(priv != NULL);

  struct sam_classd_config_s *config = priv->config;

  DEBUGASSERT(caps != NULL);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_FEATURE:
        {
          audinfo("  AUDIO_TYPE_FEATURE\n");

          /* Process based on Feature Unit */

          switch (caps->ac_format.hw)
            {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
              case AUDIO_FU_VOLUME:
                {
                  /* Set the volume */

                  uint16_t volume = caps->ac_controls.hw[0];
                  audinfo("    Volume: %d\n", volume);
                  classd_setvolume(priv, volume);
                }
                break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
              case AUDIO_FU_BALANCE:
                {
                  uint16_t balance = caps->ac_controls.hw[0];
                  audinfo("    Balance: %d\n", balance);
                  classd_setbalance(priv, balance);
                }
                break;
#endif /* CONFIG_AUDIO_EXCLUDE_BALANCE */

              default:
                {
                  auderr("    ERROR: Unrecognized hw feature: %d\n",
                              caps->ac_format.hw);
                  ret = -ENOTTY;
                }
                break;
            }
        }
        break;

      case AUDIO_TYPE_OUTPUT:
        {
          audinfo("  AUDIO_TYPE_OUTPUT:\n");
          audinfo("    Number of channels: %u\n", caps->ac_channels);
          audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
          audinfo("    Bits per sample:    %d\n", caps->ac_controls.b[2]);

          /* Verify that all of the requested values are supported */

          ret = -EINVAL;

          if ((caps->ac_controls.hw[0] != 8000)  &&
              (caps->ac_controls.hw[0] != 11025) &&
              (caps->ac_controls.hw[0] != 16000) &&
              (caps->ac_controls.hw[0] != 22050) &&
              (caps->ac_controls.hw[0] != 32000) &&
              (caps->ac_controls.hw[0] != 44100) &&
              (caps->ac_controls.hw[0] != 48000) &&
              (caps->ac_controls.hw[0] != 88200) &&
              (caps->ac_controls.hw[0] != 96000))
            {
              auderr("ERROR: Unsupported sample rate: %d\n",
                     caps->ac_controls.hw[0]);
              break;
            }

          if (caps->ac_channels != 1 && caps->ac_channels != 2)
            {
              auderr("ERROR: Unsupported sample rate: %d\n",
                     caps->ac_controls.hw[0]);
              break;
            }

          /* Note: strictly, this peripheral needs 16 bit values but
           * by allowing for 8 bits it at least allows the file to be played
           * even if it is then rather quiet.
           */

          if (caps->ac_controls.b[2] != 16 && caps->ac_controls.b[2] != 8)
            {
              auderr("ERROR: Unsupported bits per sample: %d\n",
                     caps->ac_controls.b[2]);
              break;
            }

          /* Save the current stream configuration */

          config->sample_rate = caps->ac_controls.hw[0];
          config->nchanns = caps->ac_channels;
          config->nbits = caps->ac_controls.b[2];

          classd_disable_audio(priv);
          classd_reset(priv);
          ret = classd_setup(priv);
          classd_enable_audio(priv, true);
          classd_dump_registers("After playback setup");
          ret = OK;
        }
        break;
#if 0
      case AUDIO_TYPE_PROCESSING:
        {
          audinfo("    INFO: Configure Processing\n");
        }
        break;
#endif
      default:
        {
          auderr("    ERROR: Unsupported configure command\n");
          ret = -ENOTTY;
        }
        break;
    }

  classd_dump_registers("After system configuration");

  return ret;
}

/****************************************************************************
 * Name: classd_shutdown
 *
 * Description:
 *   Shutdown the driver .
 *
 * Description:
 *   Configure the audio device for the specified mode of operation.
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int classd_shutdown(struct audio_lowerhalf_s *dev)
{
  struct classd_dev_s *priv = (struct classd_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  classd_disable_audio(priv);
  audinfo("Shutdown OK\n");
  return OK;
}

/****************************************************************************
 * Name: classd_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 *
 ****************************************************************************/

static void classd_returnbuffers(struct classd_dev_s *priv)
{
  struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations.  Not the best design.  But we will
   * use interrupt controls to protect against that possibility.
   */

  flags = enter_critical_section();
  while (dq_peek(&priv->doneq) != NULL)
    {
      /* Take the next buffer from the queue of completed transfers */

      apb = (struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      leave_critical_section(flags);

      audinfo("Returning: apb=%p curbyte=%" PRId32 " nbytes=%" PRId32 \
              " flags=%04x\n",
              apb, apb->curbyte, apb->nbytes, apb->flags);

      /* Are we returning the final buffer in the stream? */

      if ((apb->flags & AUDIO_APB_FINAL) != 0)
        {
          /* Both the pending and the done queues should be empty and there
           * should be no buffers in-flight.
           */

          DEBUGASSERT(dq_empty(&priv->doneq) && dq_empty(&priv->pendq) &&
                      priv->inflight == 0);

          /* Set the terminating flag.  This will, eventually, cause the
           * worker thread to exit (if it is not already terminating).
           */

          audinfo("Terminating\n");
          priv->terminate = true;
        }

      /* Release our reference to the audio buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
      flags = enter_critical_section();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: classd_sendbuffer
 *
 * Description:
 *   Start the transfer of an audio buffer to the peripheral.  This
 *   will not wait for the transfer to complete but will return immediately.
 *   The classd_senddone callback will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 * Input Parameters:
 *   priv - A reference to the CLASSD peripheral.
 *
 * Returned Value:
 *   OK or negated error value
 ****************************************************************************/

static int classd_sendbuffer(struct classd_dev_s *priv)
{
  struct sam_classd_config_s *config;
  struct ap_buffer_s *apb;
  struct audio_msg_s msg;
  irqstate_t flags;
  uintptr_t samp;
  uintptr_t paddr;
  uintptr_t maddr;
  apb_samp_t nbytes;
  int ret;

  DEBUGASSERT(priv != NULL);

  config = priv->config;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_SAMA5D2_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  if (config->nchanns == 2)
    {
      if (config->nbits == 16)
        {
          sam_dmaconfig(priv->txdma, DMA32_FLAGS);
        }
      else
        {
          sam_dmaconfig(priv->txdma, DMA16_FLAGS);
        }
    }
  else
    {
      if (config->nbits == 16)
        {
          sam_dmaconfig(priv->txdma, DMA16_FLAGS);
        }
      else
        {
          sam_dmaconfig(priv->txdma, DMA8_FLAGS);
        }
    }

  classd_dma_sampleinit(priv);

  while (priv->inflight < CONFIG_SAMA5D2_CLASSD_INFLIGHT &&
         dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      /* Take next buffer from the queue of pending transfers */

      apb = (struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("Sending apb=%p, size=%" PRId32 " inflight=%d\n",
              apb, apb->nbytes, priv->inflight);

      /* Increment the number of buffers in-flight before sending in order
       * to avoid a possible race condition.
       */

      flags = enter_critical_section();
      priv->inflight++;
      leave_critical_section(flags);

      /* Send the entire audio buffer. */

      samp   = (uintptr_t)&apb->samp[apb->curbyte];
      paddr = SAM_CLASSD_THR;
      maddr = sam_physramaddr(samp);
      nbytes = apb->nbytes - apb->curbyte;

      ret = sam_dmatxsetup(priv->txdma, paddr, maddr, nbytes);
      if (ret < 0)
        {
          dmaerr("ERROR: sam_dmatxsetup failed: %d\n", ret);
          break;
        }

      classd_txdma_sample(priv, DMA_AFTER_SETUP);

      /* Start the DMA transfer.
       * The callback will clear up the DMA once completed.
       */

      priv->dmares = -EBUSY;
      ret = sam_dmastart(priv->txdma, classd_txcallback, (void *)priv);

      if (ret < 0)
        {
          dmaerr("ERROR: TX sam_dmastart failed: %d\n", ret);
          return ret;
        }

      classd_txdma_sample(priv, DMA_AFTER_START);

      ret = wd_start(&priv->dmadog, DMA_TIMEOUT_TICKS,
                     classd_dmatimeout, (wdparm_t)priv);
      if (ret < 0)
        {
           auderr("ERROR: wd_start failed: %d\n", ret);
           break;
        }

      /* Wait for DMA completion.  This is done in a loop because there my be
       * false alarm semaphore counts that cause sam_wait() not fail to wait
       * or to wake-up prematurely (for example due to the receipt of a
       * signal). We know that the DMA has completed when the result is
       * anything other than -EBUSY.
       */

      do
        {
          /* Start (or re-start) the watchdog timeout */

          ret = wd_start(&priv->dmadog, DMA_TIMEOUT_TICKS,
                         classd_dmatimeout, (wdparm_t)priv);
          if (ret < 0)
            {
               auderr("ERROR: wd_start failed: %d\n", ret);
               break;
            }

          /* Wait for the DMA complete */

          ret = nxsem_wait_uninterruptible(&priv->dmawait);

          /* Cancel the watchdog timeout */

          wd_cancel(&priv->dmadog);

          /* Check if we were awakened by an error of some kind. */

          if (ret < 0)
            {
              DEBUGPANIC();
              break;
            }

          /* Note that we might be awakened before the wait is over due to
           * residual counts on the semaphore.  So, to handle, that case,
           * we loop until something changes the DMA result to any value
           * other than -EBUSY.
           */
        }
      while (priv->dmares == -EBUSY);

      /* Dump the sampled DMA registers */

      classd_dma_sampledone(priv);

      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminateson an error condition).
       */

      sam_dmastop(priv->txdma);

      /* All we can do is complain if the DMA fails */

      if (priv->dmares < 0)
        {
          auderr("ERROR: DMA failed with result: %d\n", priv->dmares);
          break;
        }

      DEBUGASSERT(priv->inflight > 0);
      flags = enter_critical_section();
      dq_addlast((dq_entry_t *)apb, &priv->doneq);
      priv->inflight--;
      leave_critical_section(flags);

      /* Now send a message to the worker thread, informing it that there are
       * buffers in the done queue that need to be cleaned up.
       */

      msg.msg_id = AUDIO_MSG_COMPLETE;
      ret = file_mq_send(&priv->mq, (const char *)&msg, sizeof(msg),
                         CONFIG_AUDIO_SAMA5_CLASSD_MSG_PRIO);
      if (ret < 0)
        {
          auderr("ERROR: file_mq_send failed: %d\n", ret);
        }

      audinfo("apb=%p inflight=%d result=%d\n", apb, priv->inflight,
              priv->dmares);
    }

  return ret;
}

/****************************************************************************
 * Name: classd_workerthread
 *
 *  This is the thread that feeds data to the classd peripheral and keeps
 *  the audio stream going.
 *
 ****************************************************************************/

static void *classd_workerthread(pthread_addr_t pvarg)
{
  struct classd_dev_s *priv = (struct classd_dev_s *)pvarg;
  struct audio_msg_s msg;
  struct ap_buffer_s *apb;
  int msglen;
  unsigned int prio;

  DEBUGASSERT(priv != NULL);

  audinfo("Entry\n");

  /* Loop as long as we are supposed to be running */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminate = false;
#endif

  priv->running = true;

  while (priv->running || priv->inflight > 0)
    {
      /* Check if we have been asked to terminate. We have to check if we
       * still have buffers in-flight: if so, we can't stop yet.
       */

      if (priv->terminate && priv->inflight <= 0)
        {
          /* We are IDLE.  Break out of the loop and exit. */

          break;
        }

      else
        {
          /* Send the audio buffer */

          classd_sendbuffer(priv);
        }

      /* Wait for messages from our message queue */

      msglen = file_mq_receive(&priv->mq, (char *)&msg,
                               sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msg_id)
        {
          case AUDIO_MSG_DATA_REQUEST:
            break;

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:
            audinfo("AUDIO_MSG_STOP\n");
            priv->terminate = true;
            break;
#endif

          case AUDIO_MSG_ENQUEUE:
            audinfo("AUDIO_MSG_ENQUEUE\n");
            break;

          case AUDIO_MSG_COMPLETE:
            audinfo("AUDIO_MSG_COMPLETE\n");
            classd_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* reset the classD peripheral */

  classd_reset(priv);

  /* Return any pending buffers in our pending queue */

  nxmutex_lock(&priv->pendlock);
  while ((apb = (struct ap_buffer_s *)dq_remfirst(&priv->pendq)) != NULL)
    {
      /* Release our reference to the buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
    }

  nxmutex_unlock(&priv->pendlock);

  classd_returnbuffers(priv);

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audinfo("Exit\n");
  return NULL;
}

/****************************************************************************
 * Name: classd_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_start(struct audio_lowerhalf_s *dev, void *session)
#else
static int classd_start(struct audio_lowerhalf_s *dev)
#endif
{
  struct classd_dev_s *priv = (struct classd_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  void *value;
  int ret;

  DEBUGASSERT(priv != NULL);

  audinfo("Entry\n");

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%" PRIXPTR,
           (uintptr_t)priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ret = file_mq_open(&priv->mq, priv->mqname,
                     O_RDWR | O_CREAT, 0644, &attr);
  if (ret < 0)
    {
      /* Error creating message queue! */

      auderr("ERROR: Couldn't allocate message queue\n");
      return ret;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      audinfo("Joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for sending data to the device */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setstacksize(&tattr,
                            CONFIG_AUDIO_SAMA5_CLASSD_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, classd_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "SAMA5D2 classD audio");
      audinfo("Created worker thread\n");
    }

  audinfo("Return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: classd_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_stop(struct audio_lowerhalf_s *dev, void *session)
#else
static int classd_stop(struct audio_lowerhalf_s *dev)
#endif
{
  struct classd_dev_s *priv = (struct classd_dev_s *)dev;
  struct audio_msg_s term_msg;
  void *value;

  DEBUGASSERT(priv != NULL);

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  file_mq_send(&priv->mq, (const char *)&term_msg, sizeof(term_msg),
               CONFIG_AUDIO_SAMA5_CLASSD_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  audinfo("Return OK\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: classd_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_pause(struct audio_lowerhalf_s *dev, void *session)
#else
static int classd_pause(struct audio_lowerhalf_s *dev)
#endif
{
  audinfo("Return OK\n");
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: classd_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_resume(struct audio_lowerhalf_s *dev, void *session)
#else
static int classd_resume(struct audio_lowerhalf_s *dev)
#endif
{
  audinfo("Return OK\n");
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: classd_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int classd_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                struct ap_buffer_s *apb)
{
  struct classd_dev_s *priv = (struct classd_dev_s *)dev;
  struct audio_msg_s msg;
  int ret;

  ret = OK;

  DEBUGASSERT(priv && apb && priv->dev.upper);

  audinfo("Enqueuing: apb=%p curbyte=%" PRId32 " nbytes=%" PRId32 "\n",
           apb, apb->curbyte, apb->nbytes);

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the new buffer to the tail of pending audio buffers */

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  nxmutex_unlock(&priv->pendlock);

  ret = OK;

  if (priv->mq.f_inode != NULL)
    {
      msg.msg_id = AUDIO_MSG_ENQUEUE;
      msg.u.data = 0;

      ret = file_mq_send(&priv->mq, (const char *)&msg,
                         sizeof(msg), CONFIG_AUDIO_SAMA5_CLASSD_MSG_PRIO);
      if (ret < 0)
        {
          auderr("ERROR: file_mq_send failed: %d\n", ret);
        }
    }

  audinfo("Return OK\n");
  return ret;
}

/****************************************************************************
 * Name: classd_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int classd_cancelbuffer(struct audio_lowerhalf_s *dev,
                               struct ap_buffer_s *apb)
{
  audinfo("apb=%p curbyte=%" PRId32 " nbytes=%" PRId32 ", return OK\n",
          apb, apb->curbyte, apb->nbytes);

  return OK;
}

/****************************************************************************
 * Name: classd_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int classd_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  int ret = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  struct ap_buffer_info_s *bufinfo;
#endif

  audinfo("cmd=%d arg=%ld\n", cmd, arg);

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
      /* Check for AUDIOIOC_HWRESET ioctl.  This ioctl is passed straight
       * through from the upper-half audio driver.
       */

      case AUDIOIOC_HWRESET:
        {
          audinfo("AUDIOIOC_HWRESET:\n");
        }
        break;

       /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (struct ap_buffer_info_s *)arg;
          bufinfo->buffer_size = CONFIG_AUDIO_SAMA5_CLASSD_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_AUDIO_SAMA5_CLASSD_NUM_BUFFERS;
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        audinfo("Ignored\n");
        break;
    }

  audinfo("Return OK\n");
  return ret;
}

/****************************************************************************
 * Name: classd_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_reserve(struct audio_lowerhalf_s *dev,
                          void **session)
#else
static int classd_reserve(struct audio_lowerhalf_s *dev)
#endif
{
  struct classd_dev_s *priv = (struct classd_dev_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL);

  ret = nxmutex_lock(&priv->pendlock);

  if (ret < 0)
    {
      return ret;
    }

  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the session context */

      priv->inflight = 0;
      priv->running = false;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      priv->terminate = false;
#endif
      priv->reserved = true;
    }

  nxmutex_unlock(&priv->pendlock);

  audinfo("Return OK\n");
  return ret;
}

/****************************************************************************
 * Name: classd_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_release(struct audio_lowerhalf_s *dev, void *session)
#else
static int classd_release(struct audio_lowerhalf_s *dev)
#endif
{
  struct classd_dev_s *priv = (struct classd_dev_s *)dev;
  void  *value;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  nxmutex_unlock(&priv->pendlock);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sama5_classd_initialize
 *
 * Description:
 *   Initialize the CLASSD device.
 *
 * Returned Value:
 *   A new lower half audio interface for the CLASSD device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct audio_lowerhalf_s *sama5_classd_initialize(void)
{
  static struct classd_dev_s priv;

  /* Allocate a CLASSD device structure */

  if (priv.state != CLASSD_STATE_CONFIGURED)
    {
      /* one-time data initialization */

      memset(&priv, 0, sizeof(struct classd_dev_s));

      priv.config = &sam_classd_const;

      /* Initialize the ClassD device structure.  Since we used kmm_zalloc,
       * only the non-zero elements of :the structure need to be initialized.
       */

      priv.dev.ops = &g_audioops;
      nxmutex_init(&priv.pendlock);
      nxsem_init(&priv.dmawait, 0, 0);
      dq_init(&priv.pendq);
      dq_init(&priv.doneq);

      /* Pre-allocate DMA channels. */

      priv.txdma = sam_dmachannel(CONFIG_SAMA5D2_CLASSD_DMA_DMAC_NUMBER, 0);

      if (!priv.txdma)
        {
          auderr("ERROR: Failed to allocate the TX DMA channel\n");
        }

      /* Reset and set up the HW. */

      classd_dump_registers("Before reset");

      classd_reset(&priv);
      if (classd_setup(&priv) < 0)
        {
          auderr("ERROR: Failed to setup ClassD Peripheral\n");
        }

      classd_configure_pmc(&priv);
      sam_classd_enableclk();
      classd_enable_audio(&priv, true);
      classd_configure_pmc(&priv);

      classd_setvolume(&priv, 1000);

      classd_dump_registers("After init.");

      priv.state = CLASSD_STATE_CONFIGURED;
      return &priv.dev;
    }

  return NULL;
}

#endif /* CONFIG_SAMA5D2_CLASSD */
