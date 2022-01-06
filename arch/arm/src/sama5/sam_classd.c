/****************************************************************************
 * arch/arm/src/sama5/sam_flexcom_serial.c
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


/* dunno if we need these
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>


#include "arm_internal.h"

#include "chip.h"
*/
#include "arm_arch.h"
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <fixedmath.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/audio.h>
#include <hardware/sam_classd.h>

//#include "hardware/sam_sfr.h"
#include "sam_periphclks.h"


#if defined(CONFIG_SAMA5D2_CLASSD)
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Supported Extension features */

#define AUDIO_EXT_EQUALIZER_UNDEF             0x0000
#define AUDIO_EXT_EQUALIZER_FLAT              0x0001
#define AUDIO_EXT_EQUALIZER_BASS_BOOST_12     0x0002
#define AUDIO_EXT_EQUALIZER_BASS_BOOST_6      0x0004
#define AUDIO_EXT_EQUALIZER_BASS_CUT_6        0x0008
#define AUDIO_EXT_EQUALIZER_BASS_CUT_12       0x0010
#define AUDIO_EXT_EQUALIZER_MEDIUM_BOOST_8    0x0020
#define AUDIO_EXT_EQUALIZER_MEDIUM_BOOST_3    0x0040
#define AUDIO_EXT_EQUALIZER_MEDIUM_CUT_3      0x0080
#define AUDIO_EXT_EQUALIZER_MEDIUM_CUT_8      0x0100
#define AUDIO_EXT_EQUALIZER_TREBLE_BOOST_12   0x0200
#define AUDIO_EXT_EQUALIZER_TREBLE_BOOST_6    0x0400
#define AUDIO_EXT_EQUALIZER_TREBLE_CUT_6      0x0800
#define AUDIO_EXT_EQUALIZER_TREBLE_CUT_12     0x1000
#define AUDIO_EXT_EQUALIZER_VARIABLE          0x2000


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
};

enum sam_classd_data_sample_freq
{
  CLASSD_DATA_SAMPLE_FREQ_8K = 0,
  CLASSD_DATA_SAMPLE_FREQ_16K,
  CLASSD_DATA_SAMPLE_FREQ_32K,
  CLASSD_DATA_SAMPLE_FREQ_48K,
  CLASSD_DATA_SAMPLE_FREQ_96K,
  CLASSD_DATA_SAMPLE_FREQ_22K05,
  CLASSD_DATA_SAMPLE_FREQ_44K1,
  CLASSD_DATA_SAMPLE_FREQ_88K2,
};

enum sam_classd_eq_mode
{
  CLASSD_EQ_FLAT = 0,
  CLASSD_EQ_BASS_BOOST_12DB = 0,
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
                                if sum is too high the result is saturated*/
  CLASSD_MONO_MODE_LEFT,    /* left channel sent to both */
  CLASSD_MONO_MODE_RIGHT,   /* left channel sent to both */
};

/* This structure defines the audio clock configuration
 * AUDIOPLLCLK = BOARD_CLK * (nd + 1 + (fract /2^22)) / (qdpmc + 1)
 * AUDIOPINCLK = BOARD_CLK * (nc + 1 + (fracr /2^22)) / (dev *qaudio)
 */

/* This structure provides the register config. data for the CLASSD peripheral */

struct sam_classd_config_s
{
  bool left_channel_enabled;
  bool right_channel_enabled;
  bool left_channel_muted;
  bool right_channel_muted;
  enum sam_classd_pwm_type pwm_type;
  bool non_overlap_mode;
  enum sam_classd_overlap_value non_overlap_value;
  uint8_t left_channel_attenuation;
  uint8_t right_channel_attenuation;
  enum sam_classd_dsp_frequency dsp_clock_frequency;
  bool de_emphasis_enabled;
  bool left_and_right_swapped;        
  enum sam_classd_data_sample_freq sample_freq;
  enum sam_classd_eq_mode eq_mode;
  bool mono_mode_selected;
  enum sam_classd_mono_mode mono_mode;
  uint32_t balance;
  uint32_t dspclk_nd;
  uint32_t dspclk_fracr;
  uint32_t dspclk_qdpmc;
  uint32_t dspclk_div;
  uint32_t dspclk_qdaudio;
};


/* This structure provides the current config & state of the CLASSD peripheral */

struct sam_classd_s
{
  
  const struct sam_classd_config_s *config;
#ifdef CONFIG_SAMA5_CLASSD_REGDEBUG
  uintptr_t regaddr;        /* Last register address read */
  uint32_t regval;          /* Last value read from the register */
  unsigned int count;       /* Number of times that the value was read */
#endif
  uint8_t state;            /*see classd_state */ 
  sem_t                    pendsem;       /*protect pendq */
  sem_t                    locksem;       /* Enforces mutually exclusive access */  
};

struct classd_dev_s
{
  /* We are an audio lower half driver, also the upper "half" of
     the driver with respect to the lower half driver.
   * Terminology:
   * Our "lower" half audio instances will be called dev for the publically
   * visible version and "priv" for the version that only thgis driver knows.
   * From the point of view of this driver, it is the board lower "half"
   * that is referred to as "lower"
   *
   */ 
  
  struct audio_lowerhalf_s dev;           /*Audio lower half (this device) */

  uint32_t                 scaler;        /*Data bytes to sec scaler 
                                            (bytes per sec) */
  struct dq_queue_s        pendq;         /*Queue of pending buffers to be sent */
  struct dq_queue_s        doneq;         /*Queue of sent buffers to be returned */
  struct file              mq;            /*Message queue for receiving messages */
  char                     mqname[16];    /*Our message queue name */
  pthread_t                threadid;      /*ID of our thread */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  volatile bool            terminate;     /* True: request to terminate */
#endif
 
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
void classd_dsp_clock_config(struct sam_classd_s *priv);
void classd_enable_audio(bool pmc_clock_enable, FAR struct sam_classd_s *priv);
void classd_disable_audio(FAR struct sam_classd_s *priv);
static int classd_setup(FAR struct sam_classd_s *priv);

#if defined(CONFIG_SAMA5_CLASSD_REGDEBUG) 
static
#endif
       uint32_t classd_getreg(FAR struct sam_classd_s *priv,
                  uint32_t regaddr);
#ifdef CONFIG_SAMA5_CLASSD_REGDEBUG
static void classd_dump_registers(FAR const char *msg);
#else
#  define classd_dump_registers(msg);
#endif 

static void classd_putreg(uint32_t regaddr,
                        uint32_t regval);

/* Audio lower half functions */

static int      classd_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                  FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      classd_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int      classd_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR const struct audio_caps_s *caps);
#endif
static int      classd_shutdown(FAR struct audio_lowerhalf_s *dev);
static void    *classd_workerthread(pthread_addr_t pvarg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      classd_start(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      classd_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      classd_stop(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      classd_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      classd_pause(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
static int      classd_resume(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      classd_pause(FAR struct audio_lowerhalf_s *dev);
static int      classd_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int      classd_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      classd_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      classd_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                  unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      classd_reserve(FAR struct audio_lowerhalf_s *dev,
                  FAR void **session);
#else
static int      classd_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      classd_release(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      classd_release(FAR struct audio_lowerhalf_s *dev);
#endif
static int      classd_sleep(FAR struct audio_lowerhalf_s *dev,
                           FAR struct ap_buffer_s *apb);

static void classd_reset(FAR struct sam_classd_s *priv);

/* Semaphore helpers */

static int classd_dev_lock(FAR struct sam_classd_s *priv);
static int classd_dev_lock_noncancellable(FAR struct sam_classd_s *priv);
#define classd_dev_unlock(priv) nxsem_post(&priv->locksem)

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* default/constant configuration */

static const struct sam_classd_config_s sam_classd_const =
{
#if defined (CONFIG_SAMA5D2_CLASSD_LEN)
  .left_channel_enabled = 1,
#else
  .left_channel_enabled = 0,
#endif
#if defined (CONFIG_SAMA5D2_CLASSD_REN)
  .right_channel_enabled = 1,
#else
  .right_channel_enabled = 0,
#endif
#if defined (CONFIG_SAMA5D2_CLASSD_LMUTE)
  .left_channel_muted = 1,
#else
  .left_channel_muted = 0,
#endif
#if defined (CONFIG_SAMA5D2_CLASSD_RMUTE)
  .right_channel_muted = 1,
#else
  .right_channel_muted = 0,
#endif
  .pwm_type = CONFIG_SAMA5D2_CLASSD_PWMTYP,
#if defined (CONFIG_SAMA5D2_CLASSD_NON_OVERLAP_MODE)
  .non_overlap_mode = 1,
  .non_overlap_value = CONFIG_SAMA5D2_CLASSD_NON_OVERLAP_VALUE,
#else
  .non_overlap_mode = 0,
  .non_overlap_value = 0,
#endif
  .left_channel_attenuation = CONFIG_SAMA5D2_CLASSD_ATTL,
  .right_channel_attenuation = CONFIG_SAMA5D2_CLASSD_ATTR,
  .dsp_clock_frequency = CONFIG_SAMA5D2_CLASSD_DSPCLK,
#if defined (CONFIG_SAMA5D2_CLASSD_DEEMP)
  .de_emphasis_enabled = 1,
#else
  .de_emphasis_enabled = 0,
#endif
#if defined (CONFIG_SAMA5D2_CLASSD_SWAP)
  .left_and_right_swapped = 1,
#else
  .left_and_right_swapped = 0,
#endif
  .sample_freq = CONFIG_SAMA5D2_CLASSD_FRAME,
  .eq_mode = CONFIG_SAMA5D2_CLASSD_EQCFG,
#if defined (CONFIG_SAMA5D2_CLASSD_MONO)
  .mono_mode_selected = 1,
  .mono_mode = CONFIG_SAMA5D2_CLASSD_MONOMODE,
#else
  .mono_mode_selected = 0,
  .mono_mode = 0,
#endif
  .balance = 0,
  .dspclk_nd = 56,
  .dspclk_fracr = 1442841,
  .dspclk_qdpmc = 6,
  .dspclk_div = 0,
  .dspclk_qdaudio = 0,  
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
  NULL,               /* allocbuffer    */
  NULL,               /* freebuffer     */
  classd_enqueuebuffer, /* enqueue_buffer */
  classd_cancelbuffer,  /* cancel_buffer  */
  classd_ioctl,         /* ioctl          */
  NULL,               /* read           */
  NULL,               /* write          */
  classd_reserve,       /* reserve        */
  classd_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
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
void classd_disable_audio(FAR struct sam_classd_s *priv)
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
 *    true:  the output clock of the audio PLL is sent to the PMC
 *    false: the output clock of the audio PLL is NOT sent to the PMC
 *   pmc_pad_clock_enable: 
 *    true:  The external audio pin CLK_AUDIO is driven low
 *    false: The external audio pin CLK_AUDIO is driven by AUDIOPINCLK
 *
 * Returned Value:
 *
 ****************************************************************************/
void classd_enable_audio(bool pmc_clock_enable, FAR struct sam_classd_s *priv)
{
#ifdef PMC_AUDIO_PLL0_PLLEN
  uint32_t regval;
 
  regval = classd_getreg(priv, SAM_PMC_AUDIO_PLL0);
  regval &= ~(PMC_AUDIO_PLL0_PADEN | PMC_AUDIO_PLL0_PMCEN);
  regval |= PMC_AUDIO_PLL0_PLLEN;
#if defined SAMA5D2_CLASSD_PAD_CLK
  regval |= PMC_AUDIO_PLL0_PADEN;
#endif
  if (pmc_clock_enable)
    regval |= PMC_AUDIO_PLL0_PMCEN;
  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);
#endif
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
void classd_dsp_clock_config(struct sam_classd_s *priv)
{
  uint32_t regval;
  struct sam_classd_config_s *config = priv->config;
#ifdef PMC_AUDIO_PLL0_PLLEN 
  /* reset audio clock */
  regval = classd_getreg(priv, SAM_PMC_AUDIO_PLL0);
  regval &= ~(PMC_AUDIO_PLL0_RESETN | PMC_AUDIO_PLL0_PLLEN);
  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);
  

  regval |= PMC_AUDIO_PLL0_RESETN;
  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

  /*configure values*/
  regval =   PMC_AUDIO_PLL0_ND(config->dspclk_nd)
           | PMC_AUDIO_PLL0_QDPMC(config->dspclk_qdpmc)
           | PMC_AUDIO_PLL0_FLT(13)
           | PMC_AUDIO_PLL0_RESETN;
  classd_putreg(SAM_PMC_AUDIO_PLL0, regval);

  regval =   PMC_AUDIO_PLL1_FRACR(config->dspclk_fracr)
           | PMC_AUDIO_PLL1_DIV(config->dspclk_div)
           | PMC_AUDIO_PLL1_QDAUDIO(config->dspclk_qdaudio);
  classd_putreg(SAM_PMC_AUDIO_PLL1, regval);

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
static int classd_setup(FAR struct sam_classd_s *priv)
{
  uint32_t regval;
  struct sam_classd_config_s *config = priv->config;

  regval = 0;
  regval = config->left_channel_enabled
                                  ? (regval | CLASSD_MR_LEN_MASK)   : regval;
  regval = config->left_channel_muted
                                  ? (regval | CLASSD_MR_LMUTE_MASK) : regval;
  regval = config->right_channel_enabled
                                  ? (regval | CLASSD_MR_REN_MASK)   : regval;
  regval = config->right_channel_muted
                                ? (regval | CLASSD_MR_RMUTE_MASK)   : regval;
  regval = (config->pwm_type == CLASSD_PWM_TYPE_UNIFORM)
                                ? (regval | CLASSD_MR_PWMTYP_MASK)  : regval;
  regval = config->non_overlap_mode
                                ? (regval | CLASSD_MR_NOVR_MASK)    :regval;
  regval |= CLASSD_MR_NOVR(config->non_overlap_value);
  
  /* disable write protection */
#if 0
  classd_putreg(SAM_CLASSD_WPMR, CLASSD_WPMR_PASSWD);
#endif


  /* write mode register */
  classd_putreg(SAM_CLASSD_MR, regval);

  regval = 0;
  regval |= CLASSD_VOL_LEFT(config->left_channel_attenuation);
  regval |= CLASSD_VOL_RIGHT(config->right_channel_attenuation);
  regval = (config->dsp_clock_frequency == CLASSD_DSP_FREQ_11M2896)
                                ? (regval | CLASSD_INTPMR_DSPCLKF_MASK) : regval;
  regval = config->de_emphasis_enabled
                                ? (regval | CLASSD_INTPMR_DEEMP_MASK)   : regval;
  regval = config->left_and_right_swapped
                                ? (regval | CLASSD_INTPMR_SWAP_MASK)    : regval;
  regval = (config->dsp_clock_frequency == CLASSD_DSP_FREQ_11M2896)
                                ? (regval | CLASSD_INTPMR_DSPCLKF_MASK) : regval;

  regval |= CLASSD_INTRMR_FRAME(config->sample_freq);
  regval |= CLASSD_INTRMR_EQCFG(config->eq_mode);

  regval = config->mono_mode_selected     
                                ? (regval | CLASSD_INTPMR_MONO_MASK)    :regval;
  regval |= CLASSD_INTRMR_MONOMODE(config->mono_mode);

  /* write interpolator mode register */
  classd_putreg(SAM_CLASSD_INTPMR, regval);

  /* enable write protection */
#if 0
  classd_putreg(SAM_CLASSD_WPMR, CLASSD_WPMR_WPEN_MASK | CLASSD_WPMR_PASSWD);
#endif
  if (classd_getreg(priv, SAM_CLASSD_INTSR))
    return -1;
  else
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
 
#ifdef CONFIG_SAMA5_CLASSD_REGDEBUG
static uint32_t classd_getreg(FAR struct sam_classd_s *priv, uint32_t regaddr)

{
  //FAR const struct sam_classd_config_s *config = priv->config;
  uint32_t regval;

  /* Read the value from the register */

  regval  = getreg32(regaddr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (regaddr == priv->regaddr && regval == priv->regval)
    {
      if (priv->count == 0xffffffff || ++priv->count > 3)
        {
          if (priv->count == 4)
            {
              audinfo("...\n");
            }

          return regval;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (priv->count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          audinfo("[repeats %d more times]\n", priv->count - 3);
        }

      /* Save the new address, value, and count */

      priv->regaddr = regaddr;
      priv->regval  = regval;
      priv->count   = 1;
    }

  /* Show the register value read */

  audinfo("%08x->%08x\n", regaddr, regval);
  return regval;
}

#else
uint32_t classd_getreg(FAR struct sam_classd_s *priv, uint32_t regaddr)
{
  return getreg32(regaddr);
}

#endif


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

#ifdef CONFIG_SAMA5_CLASSD_REGDEBUG
static void classd_putreg(uint32_t regaddr, uint32_t regval)
{
  /* Show the register value being written */

  audinfo("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}

#else
static void classd_putreg(uint32_t regaddr, uint32_t regval)
{
  putreg32(regval, regaddr);
}

#endif

/****************************************************************************
 * Name: classd_dumpregs
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

#ifdef CONFIG_SAMA5_CLASSD_REGDEBUG
static void classd_dump_registers(FAR const char *msg)
{
  audinfo("CLASSD Registers: %s\n", msg); 


  audinfo(" PMC_PCR_PLL0 %08x    PMC_PCR_PLL1  %08x\n", 
            getreg32(SAM_PMC_AUDIO_PLL0), getreg32(SAM_PMC_AUDIO_PLL1));
   
  audinfo(" CR: %08x  MR: %08x  INTPMR %08x: INTSR %08x: THR: %08x\n", 
          getreg32(SAM_CLASSD_CR),
          getreg32(SAM_CLASSD_MR),
          getreg32(SAM_CLASSD_INTPMR),
          getreg32(SAM_CLASSD_INTSR),
          getreg32(SAM_CLASSD_THR));
  audinfo(" IER: %08x IDR: %08x   IMR: %08x  ISR: %08x  WPMR: %08x\n",
          getreg32(SAM_CLASSD_IER),
          getreg32(SAM_CLASSD_IDR),
          getreg32(SAM_CLASSD_IMR),
          getreg32(SAM_CLASSD_ISR),
          getreg32(SAM_CLASSD_WPMR));

 }
#endif

/****************************************************************************
 * Name: classd_reset
 *
 * Description:
 *   Reset peripheral to default state
 *
 ****************************************************************************/

static void classd_reset(FAR struct sam_classd_s *priv)
{

DEBUGASSERT(priv);

audinfo("CLASSD reset\n");

/* get exclusive access to the CLASSD peripheral */

classd_dev_lock_noncancellable(priv);

/* disable all interrupts */

classd_putreg(SAM_CLASSD_IER, 0);
classd_putreg(SAM_CLASSD_CR, CLASSD_CR_SWRST_MASK); //HW reset

/*Release anything pending*/

nxsem_destroy(&priv->pendsem);
nxsem_init(&priv->pendsem, 0, 1);


priv->state = CLASSD_STATE_RESET;
classd_dev_unlock(priv);

}



/****************************************************************************
 * Name: classd_scalevolume
 *
 * Description:
 *   Set the right and left volume values in the Class D device based on the
 *   current volume and balance settings.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static inline uint16_t classd_scalevolume(uint16_t volume, b16_t scale)
{
  return b16toi((b16_t)volume * scale);
}
#endif


/****************************************************************************
 * Name: classd_setvolume
 *
 * Description:
 *   Set the right and left volume values in the CLASSD device based on the
 *   current volume and balance settings.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void classd_setvolume(FAR struct sam_classd_s *priv, uint16_t volume)
{

  FAR struct sam_classd_config_s *config = priv->config;

  uint32_t leftlevel;
  uint32_t rightlevel;
  uint16_t regval;

  audinfo("volume=%u \n", volume);

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  /* Calculate the left channel volume level {0..1000} */

  if (config->balance <= 500)
    {
      leftlevel = volume;
    }
  else if (config->balance == 1000)
    {
      leftlevel = 0;
    }
  else
    {
      leftlevel = classd_scalevolume(volume, b16ONE - (b16_t)config->balance);
    }

  /* Calculate the right channel volume level {0..1000} */

  if (config->balance >= 500)
    {
      rightlevel = volume;
    }
  else if (config->balance == 0)
    {
      rightlevel = 0;
    }
  else
    {
      rightlevel = classd_scalevolume(volume, (b16_t)config->balance);
    }
#else
  leftlevel  = config->left_channel_attenuation;
  rightlevel = config->right_channel_attenuation;
#endif

  /* Set the volume */
  regval = classd_getreg(priv, SAM_CLASSD_MR);
  if (config->left_channel_muted)
    {
      regval |= CLASSD_MR_LMUTE_MASK;
    }
  else
    {
      regval &= ~CLASSD_MR_LMUTE_MASK;
    }
  if (config->right_channel_muted)
    {
      regval |= CLASSD_MR_RMUTE_MASK;
    }
  else
    {
      regval &= ~CLASSD_MR_RMUTE_MASK;
    }    
  classd_putreg(SAM_CLASSD_MR, regval);
  
  regval = classd_getreg(priv, SAM_CLASSD_INTPMR);
  regval &= CLASSD_VOL_LEFT(leftlevel);
  regval &= CLASSD_VOL_RIGHT(rightlevel);

  classd_putreg(SAM_CLASSD_INTPMR, regval);

  config->left_channel_attenuation = volume;
  config->right_channel_attenuation = volume;


}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */



/****************************************************************************
 * Name: classd_sleep
 *
 * Description: Consume audio buffer in queue
 *
 ****************************************************************************/

static int classd_sleep(FAR struct audio_lowerhalf_s *dev,
                      FAR struct ap_buffer_s *apb)
{
  FAR struct classd_dev_s *priv = (struct classd_dev_s *)dev;
  uint64_t sleep_time;

  sleep_time = USEC_PER_SEC * (uint64_t)apb->nbytes / priv->scaler;
  usleep(sleep_time);
#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                  apb, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                  apb, OK);
#endif
  if ((apb->flags & AUDIO_APB_FINAL) != 0)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv,
                      AUDIO_CALLBACK_COMPLETE,
                      NULL,
                      OK,
                      NULL);
#else
      priv->dev.upper(priv->dev.priv,
                      AUDIO_CALLBACK_COMPLETE,
                      NULL,
                      OK);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: classd_getcaps
 *
 * Description: Get the audio device capabilities
 *
 ****************************************************************************/

static int classd_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                          FAR struct audio_caps_s *caps)
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

        caps->ac_channels = 2; /*stereo output*/

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.b[0] = AUDIO_SAMP_RATE_8K |
                                       AUDIO_SAMP_RATE_16K |
                                       AUDIO_SAMP_RATE_22K |
                                       AUDIO_SAMP_RATE_32K |
                                       AUDIO_SAMP_RATE_44K |
                                       AUDIO_SAMP_RATE_48K |
                                       AUDIO_SAMP_RATE_96K ;
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
      case AUDIO_TYPE_EXTENSION:
        /* If the sub-type is UNDEF,
         * then report the Equalizer Units we support
         */

        if (caps->ac_subtype == AUDIO_EXT_EQUALIZER_UNDEF)
          {
            /* Fill in the ac_controls section with
             * the Feature Units we have
             */
          caps->ac_controls.b[0] = AUDIO_EXT_EQUALIZER_FLAT |
                                  AUDIO_EXT_EQUALIZER_BASS_BOOST_12;
          }
          else 
          {
            /* TODO:  Do we need to provide specific info for the
             * Extension Units?
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
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else
static int classd_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif
{
  int ret = OK;

  FAR struct classd_dev_s *priv = (FAR struct classd_dev_s *)dev;

  DEBUGASSERT(priv != NULL && caps != NULL);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:
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

            if (volume >= 0 && volume <= 1000)
              {
                /* Scale the volume setting 
                   If value sent is <= 77 channel is muted
                   otherwise attenuated by the value sent 
                   so scale to the range {0...76} dB */

                classd_setvolume(priv, 76-(76 * volume / 1000));
              }
            else
              {
                ret = -EDOM;
              }
           }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

       default:
          auderr("    ERROR: Unrecognized feature unit\n");
          ret = -ENOTTY;
          break;
        }
        break;

    case AUDIO_TYPE_OUTPUT:
      {
        audinfo("  AUDIO_TYPE_OUTPUT:\n");
        audinfo("    Number of channels: %u\n", caps->ac_channels);
        audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
        audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

        /* Verify that all of the requested values are supported */

        ret = -ERANGE;
        if (caps->ac_channels != 1 && caps->ac_channels != 2)
          {
            auderr("ERROR: Unsupported number of channels: %d\n",
                   caps->ac_channels);
            break;
          }

        if (caps->ac_controls.b[2] != 8 && caps->ac_controls.b[2] != 16)
          {
            auderr("ERROR: Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        /* Save the current stream configuration */

        //priv->samprate  = caps->ac_controls.hw[0];
        //priv->nchannels = caps->ac_channels;
        //priv->bpsamp    = caps->ac_controls.b[2];

        /* Reconfigure the FLL to support the resulting number or channels,
         * bits per sample, and bitrate.
         */

        //classd_setdatawidth(priv);
        //classd_setbitrate(priv);
        //classd_writereg(priv, WM8904_DUMMY, 0x55aa);

        //classd_clock_analysis(&priv->dev, "AUDIO_TYPE_OUTPUT");
        ret = OK;
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  return ret;
}
/****************************************************************************
 * Name: classd_shutdown
 *
 * Description:
 *   Shutdown the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int classd_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: classd_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *classd_workerthread(pthread_addr_t pvarg)
{
  FAR struct classd_dev_s *priv = (FAR struct classd_dev_s *) pvarg;
  struct audio_msg_s msg;
  int msglen;
  unsigned int prio;

  audinfo("Entry\n");

  /* Loop as long as we are supposed to be running */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  while (!priv->terminate)
#else
  for (; ; )
#endif
    {
      /* Wait for messages from our message queue */

      msglen = file_mq_receive(&priv->mq, (FAR char *)&msg,
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
            priv->terminate = true;
            break;
#endif

          case AUDIO_MSG_ENQUEUE:
            classd_sleep(&priv->dev, (FAR struct ap_buffer_s *)msg.u.ptr);
            break;

          case AUDIO_MSG_COMPLETE:
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);
  priv->terminate = false;

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
static int classd_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int classd_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct classd_dev_s *priv = (FAR struct classd_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

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
  pthread_attr_setstacksize(&tattr, CONFIG_AUDIO_NULL_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, classd_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "null audio");
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
static int classd_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int classd_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct classd_dev_s *priv = (FAR struct classd_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  /* REVISIT:
   * There should be a check to see if the worker thread is still  running.
   */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  file_mq_send(&priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
               CONFIG_AUDIO_NULL_MSG_PRIO);

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
static int classd_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int classd_pause(FAR struct audio_lowerhalf_s *dev)
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
static int classd_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int classd_resume(FAR struct audio_lowerhalf_s *dev)
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

static int classd_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct classd_dev_s *priv = (FAR struct classd_dev_s *)dev;
  struct audio_msg_s msg;
  int ret;

  DEBUGASSERT(priv && apb && priv->dev.upper);

  audinfo("apb=%p curbyte=%d nbytes=%d\n", apb, apb->curbyte, apb->nbytes);

  msg.msg_id = AUDIO_MSG_ENQUEUE;
  msg.u.ptr = apb;

  ret = file_mq_send(&priv->mq, (FAR const char *)&msg,
                     sizeof(msg), CONFIG_AUDIO_NULL_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: file_mq_send failed: %d\n", ret);
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

static int classd_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                             FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p curbyte=%d nbytes=%d, return OK\n",
          apb, apb->curbyte, apb->nbytes);

  return OK;
}

/****************************************************************************
 * Name: classd_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int classd_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  int ret = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
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
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_AUDIO_NULL_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_AUDIO_NULL_NUM_BUFFERS;
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
static int classd_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else
static int classd_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: classd_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int classd_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int classd_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct classd_dev_s *priv = (FAR struct classd_dev_s *)dev;
  void  *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: classd_dev_lock
 *
 * Description:
 *   Take the semaphore that enforces mutually exclusive access to device
 *   structures, handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the classd peripheral state
 *
 * Returned Value:
 *  Normally success (OK) is returned, but the error -ECANCELED may be
 *  return in the event that task has been canceled.
 *
 ****************************************************************************/

static int classd_dev_lock(FAR struct sam_classd_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->locksem);
}

/****************************************************************************
 * Name: classd_dev_lock_noncancelable
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.  This version also
 *   ignores attempts to cancel the thread.
 *
 ****************************************************************************/

static int classd_dev_lock_noncancellable(FAR struct sam_classd_s *priv)
{
  int result;
  int ret = OK;

  do
    {
      result = nxsem_wait_uninterruptible(&priv->locksem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(result == OK || result == -ECANCELED);
      if (ret == OK && result < 0)
        {
          ret = result;
        }
    }
  while (result < 0);

  return ret;
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
 *
 * Returned Value:
 *   A new lower half audio interface for the CLASSD device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/
static struct classd_dev_s classd_dev;
static struct sam_classd_s classd_priv;

FAR struct audio_lowerhalf_s * sama5_classd_initialize(void)
{
  FAR struct sam_classd_config_s *config;
  uint32_t regval;

   /* Allocate a CLASSD device structure */

  if (classd_priv.state == CLASSD_STATE_UNINIT)
  {
    /* one-time data initialization */
    config = &sam_classd_const;
    memset (&classd_priv, 0, sizeof(struct sam_classd_s));

    classd_priv.config = config;


    /* Initialize the ClassD device structure.  Since we used kmm_zalloc,
    * only the non-zero elements of :the structure need to be initialized.
    */

    classd_dev.dev.ops   = &g_audioops;
    nxsem_init(&classd_priv.pendsem, 0, 1);
    nxsem_init(&classd_priv.locksem, 0, 1);
    dq_init(&classd_dev.pendq);
    dq_init(&classd_dev.doneq);

    /* Reset. Put the HW back to the initial state. */
    classd_dsp_clock_config(&classd_priv);
    classd_enable_audio(true, &classd_priv);
    //classd_disable_audio(&classd_priv);
    classd_dump_registers("Before reset");
    classd_reset(&classd_priv); 
    classd_dump_registers("After reset");
    
    if (classd_setup(&classd_priv) != OK)
      auderr("ERROR: Incorrect ClassD Interpolator configuration.\n");
    
    classd_dump_registers("After setup");

    return &classd_dev.dev;
  }

  return NULL;

}

#endif