/****************************************************************************
 * arch/arm/src/stm32/stm32_hrtim.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_hrtim.h"

#if defined(CONFIG_STM32_HRTIM1)

/* Only STM32F33XXX  */

#if defined(CONFIG_STM32_STM32F33XX)

#warning "HRTIM UNDER DEVELOPMENT !"

#ifdef CONFIG_STM32_HRTIM_ADC
#  error HRTIM ADC Triggering not supported yet
#endif

#ifdef CONFIG_STM32_HRTIM_DAC
#  error HRTIM DAC Triggering not supported yet
#endif

#ifdef CONFIG_STM32_HRTIM_BURST
#  error HRTIM Burst mode not supported yet
#endif

#ifdef CONFIG_STM32_HRTIM_IRQ
#  error HRTIM Interrupts not supported yet
#endif

#ifdef CONFIG_STM32_HRTIM_DMA
#  error HRTIM DMA not supported yet
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HRTIM default configuration **********************************************/

#ifndef HRTIM_TIMER_MASTER
#  define HRTIM_MASTER_PRESCALER HRTIM_PRESCALER_2
#endif

#ifndef HRTIM_MASTER_MODE
#  define HRTIM_MASTER_MODE 0
#endif
#ifndef HRTIM_TIMA_MODE
#  define HRTIM_TIMA_MODE 0
#endif
#ifndef HRTIM_TIMB_MODE
#  define HRTIM_TIMB_MODE 0
#endif
#ifndef HRTIM_TIMC_MODE
#  define HRTIM_TIMC_MODE 0
#endif
#ifndef HRTIM_TIMD_MODE
#  define HRTIM_TIMD_MODE 0
#endif
#ifndef HRTIM_TIME_MODE
#  define HRTIM_TIME_MODE 0
#endif

#ifndef HRTIM_TIMA_UPDATE
#  define HRTIM_TIMA_UPDATE 0
#endif
#ifndef HRTIM_TIMB_UPDATE
#  define HRTIM_TIMB_UPDATE 0
#endif
#ifndef HRTIM_TIMC_UPDATE
#  define HRTIM_TIMC_UPDATE 0
#endif
#ifndef HRTIM_TIMD_UPDATE
#  define HRTIM_TIMD_UPDATE 0
#endif
#ifndef HRTIM_TIME_UPDATE
#  define HRTIM_TIME_UPDATE 0
#endif

#ifndef HRTIM_TIMA_RESET
#  define HRTIM_TIMA_RESET 0
#endif
#ifndef HRTIM_TIMB_RESET
#  define HRTIM_TIMB_RESET 0
#endif
#ifndef HRTIM_TIMC_RESET
#  define HRTIM_TIMC_RESET 0
#endif
#ifndef HRTIM_TIMD_RESET
#  define HRTIM_TIMD_RESET 0
#endif
#ifndef HRTIM_TIME_RESET
#  define HRTIM_TIME_RESET 0
#endif

#ifndef HRTIM_IRQ_COMMON
#  define HRTIM_IRQ_COMMON 0
#endif

/* HRTIM clock source configuration */

#ifdef CONFIG_STM32_HRTIM_CLK_FROM_PLL
#  if STM32_SYSCLK_SW == RCC_CFGR_SW_PLL
#    if (STM32_RCC_CFGR_PPRE2 != RCC_CFGR_PPRE2_HCLK) && \
        (STM32_RCC_CFGR_PPRE2 != RCC_CFGR_PPRE2_HCLKd2)
#      error "APB2 prescaler factor can not be greater than 2"
#    else
#      define HRTIM_HAVE_CLK_FROM_PLL 1
#      define HRTIM_CLOCK 2*STM32_PLL_FREQUENCY
#    endif
#  else
#    error "Clock system must be set to PLL"
#  endif
#else
#  define HRTIM_HAVE_CLK_FROM_APB2 1
#  if STM32_RCC_CFGR_PPRE2 ==  RCC_CFGR_PPRE2_HCLK
#      define HRTIM_CLOCK STM32_PCLK2_FREQUENCY
#  else
#      define HRTIM_CLOCK 2*STM32_PCLK2_FREQUENCY
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef HRTIM_HAVE_PWM

/* HRTIM Slave Timer Single Output Set/Reset Configuration */

struct stm32_hrtim_timout_s
{
  uint32_t set;                 /* Set events*/
  uint32_t rst;                 /* Reset events*/
};

/* HRTIM Slave Timer Chopper Configuration */

#ifdef HRTIM_HAVE_CHOPPER
struct stm32_hrtim_chopper_s
{
  uint16_t start:4;              /* Chopper start pulsewidth */
  uint16_t freq:4;               /* Chopper carrier frequency value */
  uint16_t duty:3;               /* Chopper duty cycle */
  uint16_t _res:5;               /* Reserved */
};
#endif

/* HRTIM Slave Timer Deadtime Configuration */

#ifdef HRTIM_HAVE_DEADTIME
struct stm32_hrtim_deadtime_s
{
  uint8_t falling_lock:2;       /* Deadtime falling value and sign lock */
  uint8_t rising_lock:2;        /* Deadtime rising value and sign lock */
  uint8_t prescaler:3;          /* Deadtime Prescaler */
  uint8_t _res:1;               /* Reserved */
};
#endif

/* HRTIM Timer PWM structure */

struct stm32_hrtim_pwm_s
{
  struct stm32_hrtim_timout_s ch1; /* Channel 1 Set/Reset configuration*/
  struct stm32_hrtim_timout_s ch2; /* Channel 2 Set/Reset configuration */

#ifdef HRTIM_HAVE_CHOPPER
  struct stm32_hrtim_chopper_s chp;
#endif
#ifdef HRTIM_HAVE_DEADTIME
  struct stm32_hrtim_deadtime_s dt;
#endif
};

#endif

#ifdef HRTIM_HAVE_CAPTURE
struct stm32_hrtim_capture_s
{
  uint32_t reserved;            /* reserved for future use */
}
#endif

/* Common data structure for Master Timer and Slave Timers*/

struct stm32_hrtim_timcmn_s
{
  uint32_t base;                /* The base adress of the timer */
  uint32_t pclk;                /* The frequency of the peripheral clock
                                 * that drives the timer module.
                                 */
  uint8_t mode;                 /* Timer mode */
  uint8_t dac:2;                /* DAC triggering */
  uint8_t reserved:6;
#ifdef HRTIM_HAVE_INTERRUPTS
  uint16_t irq;                 /* interrupts configuration */
#endif
#ifdef CONFIG_STM32_HRTIM_DMA
  uint32_t dmaburst;
#endif
};

/* Master Timer and Slave Timers structure */

struct stm32_hrtim_tim_s
{
  struct stm32_hrtim_timcmn_s tim; /* Common Timer data */
  FAR void *priv;                  /* Timer private data */
};

/* Master Timer private data structure */

struct stm32_hrtim_master_priv_s
{
  uint32_t reserved;            /* reserved for future use */
};

/* Slave Timer (A-E) private data structure */

struct stm32_hrtim_slave_priv_s
{
#ifdef HRTIM_HAVE_FAULTS
  uint8_t flt;                      /* Faults configuration.
                                     * First five bits are fault sources,
                                     * last bit is lock configuration.
                                     */
#ifdef HRTIM_HAVE_AUTO_DELAYED
  uint8_t auto_delayed;              /* Auto-delayed mode configuration */
#endif
#endif
  uint16_t update;                  /* Update configuration */
  uint32_t reset;                   /* Timer reset events */
#ifdef HRTIM_HAVE_PWM
  struct stm32_hrtim_pwm_s pwm;     /* PWM configuration */
#endif
#ifdef HRTIM_HAVE_CAPTURE
  struct stm32_hrtim_capture_s cap; /* Capture configuration */
#endif
};

#ifdef HRTIM_HAVE_FAULTS

/* Structure describes single HRTIM Fault configuration */

struct stm32_hrtim_fault_cfg_s
{
  uint8_t pol:1;                /* Fault polarity */
  uint8_t src:1;                /* Fault source */
  uint8_t filter:4;             /* Fault filter */
  uint8_t lock:1;               /* Fault lock */
  uint8_t _res:1;               /* Reserved */
};

/* Structure describes HRTIM Faults configuration */

struct stm32_hrtim_faults_s
{
#ifdef CONFIG_STM32_HRTIM_FAULT1
  struct stm32_hrtim_fault_cfg_s flt1;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT2
  struct stm32_hrtim_fault_cfg_s flt2;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT3
  struct stm32_hrtim_fault_cfg_s flt3;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT4
  struct stm32_hrtim_fault_cfg_s flt4;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT5
  struct stm32_hrtim_fault_cfg_s flt5;
#endif
};
#endif

#ifdef HRTIM_HAVE_EEV

/* Structure describes single HRTIM External Event configuration */

struct stm32_hrtim_eev_cfg_s
{
  uint8_t filter:4;             /* External Event filter */
  uint8_t src:4;                /* External Event source */
  uint8_t pol:1;                /* External Event polarity */
  uint8_t sen:1;                /* External Event sensivity */
  uint8_t mode:1;               /* External Event mode */
  uint8_t _res:5;
};

/* Structure describes HRTIM External Events configuration */

struct stm32_hrtim_eev_s
{
#ifdef CONFIG_STM32_HRTIM_EEV1
  struct stm32_hrtim_eev_cfg_s eev1;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV2
  struct stm32_hrtim_eev_cfg_s eev2;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV3
  struct stm32_hrtim_eev_cfg_s eev3;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV4
  struct stm32_hrtim_eev_cfg_s eev4;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV5
  struct stm32_hrtim_eev_cfg_s eev5;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV6
  struct stm32_hrtim_eev_cfg_s eev6;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV7
  struct stm32_hrtim_eev_cfg_s eev7;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
  struct stm32_hrtim_eev_cfg_s eev8;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV9
  struct stm32_hrtim_eev_cfg_s eev9;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV10
  struct stm32_hrtim_eev_cfg_s eev10;
#endif
};
#endif

/* Structure describes HRTIM ADC triggering configuration */

struct stm32_hrtim_adc_s
{
#ifdef CONFIG_STM32_HRTIM_ADC_TRG1
  uint32_t trg1;
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG2
  uint32_t trg2;
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG3
  uint32_t trg3;
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG4
  uint32_t trg4;
#endif
};

/* This structure describes the configuration of HRTIM device */

struct stm32_hrtim_s
{
  uint32_t base;                     /* Base adress of HRTIM block */
  struct stm32_hrtim_tim_s *master;  /* Master Timer */
#ifdef CONFIG_STM32_HRTIM_TIMA
  struct stm32_hrtim_tim_s *tima;    /* HRTIM Timer A */
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  struct stm32_hrtim_tim_s *timb;    /* HRTIM Timer B */
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  struct stm32_hrtim_tim_s *timc;    /* HRTIM Timer C */
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  struct stm32_hrtim_tim_s *timd;    /* HRTIM Timer D */
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  struct stm32_hrtim_tim_s *time;    /* HRTIM Timer E */
#endif
#ifdef HRTIM_HAVE_FAULTS
  struct stm32_hrtim_faults_s *flt;  /* Faults configuration */
#endif
#ifdef HRTIM_HAVE_EEV
  struct stm32_hrtim_eev_s *eev;     /* External Events configuration  */
#endif
#ifdef HRTIM_HAVE_ADC
  struct stm32_hrtim_adc_s *adc;     /* ADC triggering configuration */
#endif
#ifdef CONFIG_STM32_HRTIM_CMN_IRQ
  uint32_t irq;                      /* Common interrupts configuration */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* HRTIM Driver Methods */

static int stm32_hrtim_open(FAR struct file *filep);
static int stm32_hrtim_close(FAR struct file *filep);
static int stm32_hrtim_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/* HRTIM Register access */

static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits);
static uint32_t hrtim_cmn_getreg(FAR struct stm32_hrtim_s *priv, int offset);
static void hrtim_cmn_putreg(FAR struct stm32_hrtim_s *priv, int offset,
                             uint32_t value);
static void hrtim_modifyreg(FAR struct stm32_hrtim_s *priv, int offset,
                                uint32_t clrbits, uint32_t setbits);
static void hrtim_tim_putreg(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                             int offset, uint32_t value);
static void hrtim_tim_modifyreg(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                                int offset, uint32_t clrbits, uint32_t setbits);

/* HRTIM helper */

static uint32_t hrtim_tim_getreg(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                                 int offset);
static FAR struct stm32_hrtim_tim_s *hrtim_tim_get(FAR struct stm32_hrtim_s *priv,
                                                     uint8_t timer);
static uint32_t hrtim_base_get(FAR struct stm32_hrtim_s* priv, uint8_t timer);

/* Configuration */

static int hrtim_dll_cal(FAR struct stm32_hrtim_s *priv);
static int hrtim_tim_clock_config(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                                  uint8_t pre);
static int hrtim_tim_clocks_config(FAR struct stm32_hrtim_s *priv);
#if defined(HRTIM_HAVE_CAPTURE) || defined(HRTIM_HAVE_PWM) || defined(HRTIM_HAVE_SYNC)
static int hrtim_gpios_config(FAR struct stm32_hrtim_s *priv);
#endif
#if defined(HRTIM_HAVE_CAPTURE)
static int hrtim_inputs_config(FAR struct stm32_hrtim_s *priv);
#endif
#if defined(HRTIM_HAVE_SYNC)
static int hrtim_synch_config(FAR struct stm32_hrtim_s *priv);
#endif
#if defined(HRTIM_HAVE_PWM)
static int hrtim_outputs_config(FAR struct stm32_hrtim_s *priv);
static int hrtim_outputs_enable(FAR struct hrtim_dev_s *dev, uint16_t outputs,
                                bool state);
#endif
#ifdef HRTIM_HAVE_ADC
static int hrtim_adc_config(FAR struct stm32_hrtim_s *priv);
#endif
#ifdef HRTIM_HAVE_DAC
static int hrtim_dac_config(FAR struct stm32_hrtim_s *priv);
#endif
#ifdef HRTIM_HAVE_FAULTS
static int hrtim_faults_config(FAR struct stm32_hrtim_s *priv);
static int hrtim_flt_cfg(FAR struct stm32_hrtim_s *priv, uint8_t index);
static int hrtim_tim_faults_cfg(FAR struct stm32_hrtim_s *priv, uint8_t timer);
#endif
#ifdef HRTIM_HAVE_EEV
static int hrtim_events_config(FAR struct stm32_hrtim_s *priv);
static int hrtim_eev_cfg(FAR struct stm32_hrtim_s *priv, uint8_t index);
#endif
#ifdef HRTIM_HAVE_INTERRUPTS
static int hrtim_irq_config(FAR struct stm32_hrtim_s *priv);
void hrtim_irq_ack(FAR struct hrtim_dev_s *dev, uint8_t timer, int source);
#endif
static int hrtim_cmp_update(FAR struct hrtim_dev_s *dev, uint8_t timer,
                            uint8_t index, uint16_t cmp);
static int hrtim_per_update(FAR struct hrtim_dev_s *dev, uint8_t timer,
                            uint16_t per);
static uint16_t hrtim_per_get(FAR struct hrtim_dev_s *dev, uint8_t timer);
static uint16_t hrtim_cmp_get(FAR struct hrtim_dev_s *dev, uint8_t timer,
                         uint8_t index);
static int hrtim_tim_reset_set(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                               uint32_t reset);
static int hrtim_reset_config(FAR struct stm32_hrtim_s *priv);
static int hrtim_tim_update_set(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                                uint32_t update);
static int hrtim_update_config(FAR struct stm32_hrtim_s *priv);

static void hrtim_tim_mode_set(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                               uint8_t mode);
static void hrtim_mode_config(FAR struct stm32_hrtim_s *priv);

/* Initialization */

static int stm32_hrtimconfig(FAR struct stm32_hrtim_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations hrtim_fops =
{
  stm32_hrtim_open,   /* open */
  stm32_hrtim_close,  /* close */
  NULL,               /* read */
  NULL,               /* write */
  NULL,               /* seek */
  stm32_hrtim_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL              /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL              /* unlink */
#endif
};

/* Master Timer data */

static struct stm32_hrtim_tim_s g_master =
{
  .tim =
  {
    .base  = STM32_HRTIM1_MASTER_BASE,
    .pclk  = HRTIM_CLOCK/HRTIM_MASTER_PRESCALER,
    .mode  = HRTIM_MASTER_MODE,
#ifdef CONFIG_STM32_HRTIM_MASTER_DAC
    .dac   = HRTIM_MASTER_DAC,
#endif
#ifdef CONFIG_STM32_HRTIM_MASTER_IRQ
    .irq   = HRTIM_IRQ_MASTER
#endif
  },
  .priv = NULL,
};

/* NOTE: only TIMER A data defined at this time */

#ifdef CONFIG_STM32_HRTIM_TIMA

/* Timer A private data */

static struct stm32_hrtim_slave_priv_s g_tima_priv =
{
  .update = HRTIM_TIMA_UPDATE,
  .reset  = HRTIM_TIMA_RESET,
#ifdef CONFIG_STM32_HRTIM_TIMA_PWM
  .pwm =
  {
    .ch1 =
    {
      .set = HRTIM_TIMA_CH1_SET,
      .rst = HRTIM_TIMA_CH1_RST
    },
    .ch2 =
    {
      .set = HRTIM_TIMA_CH2_SET,
      .rst = HRTIM_TIMA_CH2_RST
    },
#ifdef CONFIG_STM32_HRTIM_TIMA_CHOP
    .chp =
    {
      .start_pulse = HRTIM_TIMA_CHOP_START,
      .duty        = HRTIM_TIMA_CHOP_DUTY,
      .freq        = HRTIM_TIMA_CHOP_FREQ
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_DT
    .dt =
    {
      .falling_lock = HRTIM_TIMA_DT_FLOCK,
      .rising_lock  = HRTIM_TIMA_DT_RLOCK,
      .prescaler    = HRTIM_TIMA_DT_PRESCALER,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_CAP
  .cap =
  {
    .reserved = 0
  }
#endif
};

/* Timer A data */

static struct stm32_hrtim_tim_s g_tima =
{
  .tim =
  {
    .base  = STM32_HRTIM1_TIMERA_BASE,
    .pclk  = HRTIM_CLOCK/HRTIM_TIMA_PRESCALER,
    .mode  = HRTIM_TIMA_MODE,
#ifdef CONFIG_STM32_HRTIM_TIMA_DAC
    .dac   = HRTIM_TIMA_DAC,
#endif
#ifdef CONFIG_STM32_HRTIM_MASTER_IRQ
    .irq   = HRTIM_IRQ_TIMA,
#endif
  },
  .priv = &g_tima_priv
};

#endif

/* Faults data */

#ifdef HRTIM_HAVE_FAULTS
struct stm32_hrtim_faults_s g_flt =
{
#ifdef CONFIG_STM32_HRTIM_FAULT1
  .flt1 =
  {
    .pol    = HRTIM_FAULT1_POL,
    .src    = HRTIM_FAULT1_SRC,
    .filter = HRTIM_FAULT1_FILTER,
    .lock   = HRTIM_FAULT1_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT2
  .flt2 =
  {
    .pol    = HRTIM_FAULT2_POL,
    .src    = HRTIM_FAULT2_SRC,
    .filter = HRTIM_FAULT2_FILTER,
    .lock   = HRTIM_FAULT2_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT3
  .flt3 =
  {
    .pol    = HRTIM_FAULT3_POL,
    .src    = HRTIM_FAULT3_SRC,
    .filter = HRTIM_FAULT3_FILTER,
    .lock   = HRTIM_FAULT3_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT4
  .flt2 =
  {
    .pol    = HRTIM_FAULT4_POL,
    .src    = HRTIM_FAULT4_SRC,
    .filter = HRTIM_FAULT4_FILTER,
    .lock   = HRTIM_FAULT4_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT5
  .flt2 =
  {
    .pol    = HRTIM_FAULT5_POL,
    .src    = HRTIM_FAULT5_SRC,
    .filter = HRTIM_FAULT5_FILTER,
    .lock   = HRTIM_FAULT5_LOCK,
  },
#endif
};
#endif

/* External Events data */

#ifdef HRTIM_HAVE_EEV
struct stm32_hrtim_eev_s g_eev =
{
#ifdef CONFIG_STM32_HRTIM_EEV1
  .eev1 =
  {
    .filter = HRTIM_EEV1_FILTER,
    .src    = HRTIM_EEV1_SRC,
    .pol    = HRTIM_EEV1_POL,
    .sen    = HRTIM_EEV1_SEN,
    .mode   = HRTIM_EEV1_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV2
  .eev2 =
  {
    .filter = HRTIM_EEV2_FILTER,
    .src    = HRTIM_EEV2_SRC,
    .pol    = HRTIM_EEV2_POL,
    .sen    = HRTIM_EEV2_SEN,
    .mode   = HRTIM_EEV2_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV3
  .eev3 =
  {
    .filter = HRTIM_EEV3_FILTER,
    .src    = HRTIM_EEV3_SRC,
    .pol    = HRTIM_EEV3_POL,
    .sen    = HRTIM_EEV3_SEN,
    .mode   = HRTIM_EEV3_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV4
  .eev4 =
  {
    .filter = HRTIM_EEV4_FILTER,
    .src    = HRTIM_EEV4_SRC,
    .pol    = HRTIM_EEV4_POL,
    .sen    = HRTIM_EEV4_SEN,
    .mode   = HRTIM_EEV4_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV5
  .eev5 =
  {
    .filter = HRTIM_EEV5_FILTER,
    .src    = HRTIM_EEV5_SRC,
    .pol    = HRTIM_EEV5_POL,
    .sen    = HRTIM_EEV5_SEN,
    .mode   = HRTIM_EEV5_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV6
  .eev6 =
  {
    .filter = HRTIM_EEV6_FILTER,
    .src    = HRTIM_EEV6_SRC,
    .pol    = HRTIM_EEV6_POL,
    .sen    = HRTIM_EEV6_SEN,
    .mode   = HRTIM_EEV6_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV7
  .eev7 =
  {
    .filter = HRTIM_EEV7_FILTER,
    .src    = HRTIM_EEV7_SRC,
    .pol    = HRTIM_EEV7_POL,
    .sen    = HRTIM_EEV7_SEN,
    .mode   = HRTIM_EEV7_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
  .eev8 =
  {
    .filter = HRTIM_EEV8_FILTER,
    .src    = HRTIM_EEV8_SRC,
    .pol    = HRTIM_EEV8_POL,
    .sen    = HRTIM_EEV8_SEN,
    .mode   = HRTIM_EEV8_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV9
  .eev9 =
  {
    .filter = HRTIM_EEV9_FILTER,
    .src    = HRTIM_EEV9_SRC,
    .pol    = HRTIM_EEV9_POL,
    .sen    = HRTIM_EEV9_SEN,
    .mode   = HRTIM_EEV9_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV10
  .eev10 =
  {
    .filter = HRTIM_EEV10_FILTER,
    .src    = HRTIM_EEV10_SRC,
    .pol    = HRTIM_EEV10_POL,
    .sen    = HRTIM_EEV10_SEN,
    .mode   = HRTIM_EEV10_MODE,
  }
#endif
};
#endif

/* ADC triggering data */

struct stm32_hrtim_adc_s g_adc =
{
#ifdef CONFIG_STM32_HRTIM_ADC_TRG1
  .trg1 = HRTIM_ADC_TRG1,
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG2
  .trg2 = HRTIM_ADC_TRG2,
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG3
  .trg3 = HRTIM_ADC_TRG3,
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG4
  .trg4 = HRTIM_ADC_TRG4
#endif
};

/* HRTIM1 private data */

static struct stm32_hrtim_s g_hrtim1priv =
{
  .master   = &g_master,
  .base     = STM32_HRTIM1_BASE,
#ifdef CONFIG_STM32_HRTIM_TIMA
  .tima     = &g_tima,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  .timb     = &g_timb,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  .timc     = &g_timc,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  .timd     = &g_timd,
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  .time     = &g_time,
#endif
#ifdef HRTIM_HAVE_FAULTS
  .flt      = &g_flt,
#endif
#ifdef HRTIM_HAVE_EEV
  .eev      = &g_eev,
#endif
#ifdef HRTIM_HAVE_ADC
  .adc      = &g_adc,
#endif
#ifdef CONFIG_STM32_HRTIM_CMN_IRQ
  .irq      = HRTIM_IRQ_COMMON,
#endif
};

/* HRTIM interface */

static const struct stm32_hrtim_ops_s g_hrtim1ops =
{
  .cmp_update     = hrtim_cmp_update,
  .per_update     = hrtim_per_update,
  .per_get        = hrtim_per_get,
  .cmp_get        = hrtim_cmp_get,
#ifdef HRTIM_HAVE_INTERRUPTS
  .irq_ack        = hrtim_irq_ack,
#endif
#ifdef HRTIM_HAVE_PWM
  .outputs_enable = hrtim_outputs_enable,
#endif
};

/* HRTIM device structure */

struct hrtim_dev_s g_hrtim1dev =
{
  .hd_ops   = &g_hrtim1ops,
  .hd_priv  = &g_hrtim1priv,
  .initialized = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hrtim_open
 *
 * Description:
 *   This function is called whenever the HRTIM device is opened.
 *
 ****************************************************************************/

static int stm32_hrtim_open(FAR struct file *filep)
{
#warning "stm32_hrtim_open: missing logic"
  return OK;
}

/****************************************************************************
 * Name: stm32_hrtim_close
 *
 * Description:
 *   This function is called when the HRTIM device is closed.
 *
 ****************************************************************************/

static int stm32_hrtim_close(FAR struct file *filep)
{
#warning "smt32_hrtim_close: missing logic"
  return OK;
}

/****************************************************************************
 * Name: stm32_hrtim_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the HRTIM work is done.
 *
 ****************************************************************************/

static int stm32_hrtim_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hrtim_dev_s  *dev;
  FAR struct stm32_hrtim_s *hrtim;
  int ret;

  tmrinfo("cmd: %d arg: %ld\n", cmd, arg);
  dev = inode->i_private;
  DEBUGASSERT(dev != NULL);
  hrtim = dev->hd_priv;

  UNUSED(hrtim);

#warning "smt32_hrtim_ioctl: missing logic"

  /* Handle HRTIM ioctl commands */

  switch (cmd)
    {
      default:
        {
          ret = -ENOSYS;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_modifyreg32
 *
 * Description:
 *   Modify the value of a 32-bit register (not atomic).
 *
 * Input Parameters:
 *   addr    - The address of the register
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits)
{
  putreg32((getreg32(addr) & ~clrbits) | setbits, addr);
}

/****************************************************************************
 * Name: hrtim_cmn_getreg
 *
 * Description:
 *   Read the value of an HRTIM register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t hrtim_cmn_getreg(FAR struct stm32_hrtim_s *priv, int offset)
{
  return getreg32(priv->base + STM32_HRTIM_CMN_OFFSET + offset);
}

/****************************************************************************
 * Name: hrtim_cmn_putreg
 *
 * Description:
 *   Write a value to an HRTIM register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_cmn_putreg(FAR struct stm32_hrtim_s *priv, int offset,
                             uint32_t value)
{
  putreg32(value, priv->base + STM32_HRTIM_CMN_OFFSET + offset);
}

/****************************************************************************
 * Name: hrtim__modifyreg
 *
 * Description:
 *   Modify the value of an HRTIM register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_modifyreg(FAR struct stm32_hrtim_s *priv, int offset,
                                uint32_t clrbits, uint32_t setbits)
{
  hrtim_cmn_putreg(priv, offset, (hrtim_cmn_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: hrtim_tim_get
 *
 * Description:
 *   Get Timer data structure for given HRTIM Timer index
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - An HRTIM Timer index to get
 *
 * Returned Value:
 *   Base adress offset for given Timer index
 *
 ****************************************************************************/

static FAR struct stm32_hrtim_tim_s *hrtim_tim_get(FAR struct stm32_hrtim_s *priv,
                                                   uint8_t timer)
{
  FAR struct stm32_hrtim_tim_s *tim;

  switch (timer)
    {
      case HRTIM_TIMER_MASTER:
        {
          tim = priv->master;
          break;
        }

#ifdef CONFIG_STM32_HRTIM_TIMA
      case HRTIM_TIMER_TIMA:
        {
          tim = priv->tima;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
      case HRTIM_TIMER_TIMB:
        {
          tim = &priv->timb;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
      case HRTIM_TIMER_TIMC:
        {
          tim = &priv->timc;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
      case HRTIM_TIMER_TIMD:
        {
          tim = &priv->timd;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
      case HRTIM_TIMER_TIME:
        {
          tim = &priv->time;
          break;
        }
#endif

      default:
        {
          tmrerr("ERROR: No such timer index: %d\n", timer);
          tim = NULL;
        }
    }

  return tim;
}

/****************************************************************************
 * Name: hrtim_base_get
 *
 * Description:
 *   Get base adress offset for given HRTIM Timer index
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - An HRTIM Timer index to get
 *
 * Returned Value:
 *   Base adress offset for given Timer index
 *
 ****************************************************************************/

static uint32_t hrtim_base_get(FAR struct stm32_hrtim_s* priv, uint8_t timer)
{
  FAR struct stm32_hrtim_tim_s* tim;
  uint32_t base;

  tim = hrtim_tim_get(priv, timer);
  if (tim == NULL)
    {
      base = 0;
      goto errout;
    }

  base = tim->tim.base;

errout:
  return base;
}

/****************************************************************************
 * Name: hrtim_tim_getreg
 *
 * Description:
 *   Read the value of an HRTIM Timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   tim    - An HRTIM timer index
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t hrtim_tim_getreg(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                                 int offset)
{
  uint32_t base;

  base = hrtim_base_get(priv, timer);
  if (base < 0)
    {
      return 0;
    }

  return getreg32(base + offset);
}

/****************************************************************************
 * Name: hrtim_tim_putreg
 *
 * Description:
 *   Write a value to an HRTIM Timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - An HRTIM Timer index
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_tim_putreg(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                             int offset, uint32_t value)
{
  uint32_t base;

  base = hrtim_base_get(priv, timer);
  if (base > 0)
    {
      putreg32(value, base + offset);
    }
}

/****************************************************************************
 * Name: hrtim_tim_modifyreg
 *
 * Description:
 *   Modify the value of an HRTIM Timer register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - An HRTIM Timer index
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_tim_modifyreg(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                                int offset, uint32_t clrbits, uint32_t setbits)
{
  hrtim_tim_putreg(priv, timer, offset,
                   (hrtim_tim_getreg(priv, timer, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: hrtim_dll_cal
 *
 * Description:
 *   Calibrate HRTIM DLL
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_dll_cal(FAR struct stm32_hrtim_s *priv)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32_HRTIM_PERIODIC_CAL

  /* Configure calibration rate */

  regval |= HRTIM_DLLCR_CAL_RATE;

  /* Enable Periodic calibration */

  regval |= HRTIM_DLLCR_CALEN;

  /* CALEN must not be set simultaneously with CAL bit */

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_DLLCR_OFFSET, regval);

#endif

  /* DLL Calibration Start */

  regval |= HRTIM_DLLCR_CAL;

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_DLLCR_OFFSET, regval);

  while(!(hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ISR_OFFSET) & HRTIM_ISR_DLLRDY));

  return OK;
}

/****************************************************************************
 * Name: hrtim_tim_clock_config
 *
 * Description:
 *   Configure HRTIM Timer clock
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   timer  - An HRTIM Timer index
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_clock_config(FAR struct stm32_hrtim_s *priv, uint8_t timer, uint8_t pre)
{
  int ret = OK;
  uint32_t regval = 0;

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  switch (pre)
    {
      case HRTIM_PRESCALER_1:
        {
          regval |= HRTIM_CMNCR_CKPSC_NODIV;
          break;
        }
      case HRTIM_PRESCALER_2:
        {
          regval |= HRTIM_CMNCR_CKPSC_d2;
          break;
        }
      case HRTIM_PRESCALER_4:
        {
          regval |= HRTIM_CMNCR_CKPSC_d4;
          break;
        }
      case HRTIM_PRESCALER_8:
        {
          regval |= HRTIM_CMNCR_CKPSC_d8;
          break;
        }
      case HRTIM_PRESCALER_16:
        {
          regval |= HRTIM_CMNCR_CKPSC_d16;
          break;
        }
      case HRTIM_PRESCALER_32:
        {
          regval |= HRTIM_CMNCR_CKPSC_d32;
          break;
        }
      case HRTIM_PRESCALER_64:
        {
          regval |= HRTIM_CMNCR_CKPSC_d64;
          break;
        }
      case HRTIM_PRESCALER_128:
        {
          regval |= HRTIM_CMNCR_CKPSC_d128;
          break;
        }
      default:
        {
          tmrerr("ERROR: invalid prescaler value %d for timer %d\n", timer,
                   pre);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_tim_clocks_config
 *
 * Description:
 *   Configure HRTIM Timers Clocks
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_clocks_config(FAR struct stm32_hrtim_s *priv)
{
  int ret = OK;

  /* Configure Master Timer clock */

  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_MASTER, HRTIM_MASTER_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure Timer A clock */

#ifdef CONFIG_STM32_HRTIM_TIMA
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMA, HRTIM_TIMA_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer B clock */

#ifdef CONFIG_STM32_HRTIM_TIMB
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMB, HRTIM_TIMB_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer C clock */

#ifdef CONFIG_STM32_HRTIM_TIMC
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMC, HRTIM_TIMC_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer D clock */

#ifdef CONFIG_STM32_HRTIM_TIMD
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMD, HRTIM_TIMD_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer E clock */

#ifdef CONFIG_STM32_HRTIM_TIME
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIME, HRTIM_TIME_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_gpios_config
 *
 * Description:
 *   Configure HRTIM GPIO
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(HRTIM_HAVE_CAPTURE) || defined(HRTIM_HAVE_PWM) || defined(HRTIM_HAVE_SYNC)
static int hrtim_gpios_config(FAR struct stm32_hrtim_s *priv)
{
#ifdef HRTIM_HAVE_EEV
  FAR struct stm32_hrtim_eev_s* eev = priv->eev;
#endif
#ifdef HRTIM_HAVE_FAULTS
  FAR struct stm32_hrtim_faults_s* flt = priv->flt;
#endif

  /* Configure Timer A Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMA_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHA1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMA_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHA2);
#endif

  /* Configure Timer B Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMB_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHB1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHB2);
#endif

  /* Configure Timer C Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMC_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHC1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHC2);
#endif

  /* Configure Timer D Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMD_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHD1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHD2);
#endif

  /* Configure Timer E Outputs */

#ifdef CONFIG_STM32_HRTIM_TIME_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHE1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHE2);
#endif
  /* Configure SCOUT */

#ifdef CONFIG_STM32_HRTIM_SCOUT
  stm32_configgpio(GPIO_HRTIM1_SCOUT);
#endif

  /* Configure SCIN */

#ifdef CONFIG_STM32_HRTIM_SCIN
  stm32_configgpio(GPIO_HRTIM1_SCIN);
#endif

  /* Configure Faults Inputs */

#ifdef CONFIG_STM32_HRTIM_FAULT1
  if (flt->flt1.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT1);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT2
  if (flt->flt2.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT2);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT3
  if (flt->flt3.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT3);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT4
  if (flt->flt4.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT4);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT5
  if (flt->flt5.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT5);
    }
#endif

  /* Configure External Events Inputs */

#ifdef CONFIG_STM32_HRTIM_EEV1
  if (eev->eev1.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV1);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV2
  if (eev->eev2.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV2);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV3
  if (eev->eev3.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV3);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV4
  if (eev->eev4.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV4);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV5
  if (eev->eev5.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV5);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV6
  if (eev->eev6.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV6);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV7
  if (eev->eev7.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV7);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV8
  if (eev->eev8.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV8);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV9
  if (eev->eev9.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV9);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV10
  if (eev->eev10.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV10);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: hrtim_inputs_config
 *
 * Description:
 *   Configure HRTIM Inputs
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(HRTIM_HAVE_CAPTURE)
static int hrtim_inputs_config(FAR struct stm32_hrtim_s *priv)
{
#warning "hrtim_inputs_config: missing logic"

  /* source */

  /* polarity */

  /* edge-sensitivity */

  return OK;
}
#endif

/****************************************************************************
 * Name: hrtim_synch_config
 *
 * Description:
 *   Configure HRTIM Synchronization Input/Output
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(HRTIM_HAVE_SYNC)
static int hrtim_synch_config(FAR struct stm32_hrtim_s *priv)
{
#warning "hrtim_synch_config: missing logic"
  return OK;
}
#endif

/****************************************************************************
 * Name: hrtim_tim_outputs_config
 *
 * Description:
 *   Configure HRTIM Slave Timer Outputs (CH1 and CH2)
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(HRTIM_HAVE_PWM)
static int hrtim_tim_outputs_config(FAR struct stm32_hrtim_s *priv, uint8_t timer)
{
  FAR struct stm32_hrtim_tim_s* tim;
  FAR struct stm32_hrtim_slave_priv_s* slave;

  int ret = OK;
  uint32_t regval = 0;

  /* Master Timer has no outputs */

  if (timer == HRTIM_TIMER_MASTER)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get Timer data strucutre */

  tim = hrtim_tim_get(priv, timer);
  if (tim == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  slave = (struct stm32_hrtim_slave_priv_s*)tim->priv;

  /* Configure CH1 SET events */

  regval = slave->pwm.ch1.set;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_SET1R_OFFSET, regval);

  /* Configure CH1 RESET events */

  regval = slave->pwm.ch1.rst;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RST1R_OFFSET, regval);

  /* Configure CH2 SET events */

  regval = slave->pwm.ch2.set;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_SET2R_OFFSET, regval);

  /* Configure CH2 RESET events */

  regval = slave->pwm.ch2.rst;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RST2R_OFFSET, regval);

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: hrtim_outputs_config
 *
 * Description:
 *   Configure HRTIM Outputs
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(HRTIM_HAVE_PWM)
static int hrtim_outputs_config(FAR struct stm32_hrtim_s *priv)
{
  int ret = OK;

  /* Configure HRTIM TIMER A Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMA_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMA);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER B Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMB_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMB);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER C Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMC_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMC);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER D Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMD_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMD);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER E Outputs */

#ifdef CONFIG_STM32_HRTIM_TIME_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIME);
  if (ret < 0)
    {
      goto errout;
    }
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_outputs_enable
 *
 * Description:
 *   Enable/disable HRTIM outputs (bulk operation)
 *
 * Input Parameters:
 *   dev     - HRTIM device structure
 *   outputs - outputs to set
 *   state   - Enable/disable operation
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_outputs_enable(FAR struct hrtim_dev_s *dev, uint16_t outputs,
                                bool state)
{
  FAR struct stm32_hrtim_s *priv = (FAR struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t reg = 0;

  /* Get register offset */

  if (state == true)
    {
      reg = STM32_HRTIM_CMN_OENR_OFFSET;
    }
  else
    {
      reg = STM32_HRTIM_CMN_ODISR_OFFSET;
    }

  /* Write register */

  hrtim_cmn_putreg(priv, reg, outputs);

  return OK;
}
#endif

/****************************************************************************
 * Name: hrtim_adc_config
 *
 * Description:
 *   Configure HRTIM ADC triggers
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#ifdef HRTIM_HAVE_ADC
static int hrtim_adc_config(FAR struct stm32_hrtim_s *priv)
{

#ifdef CONFIG_STM32_HRTIM_ADC_TRG1
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC1R_OFFSET, priv->adc->trg1);
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG2
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC2R_OFFSET, priv->adc->trg2);
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG3
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC3R_OFFSET, priv->adc->trg3);
#endif
#ifdef CONFIG_STM32_HRTIM_ADC_TRG4
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC4R_OFFSET, priv->adc->trg4);
#endif

  return OK;
}
#endif

#ifdef HRTIM_HAVE_DAC

/****************************************************************************
 * Name: hrtim_tim_dac_cfg
 *
 * Description:
 *   Configure single HRTIM Timer DAC synchronization event
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   timer  - Timer index
 *   dac    - DAC synchronisation event configuration
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_dac_cfg(FAS struct stm32_hrtim_s *priv, uint8_t timer,
                             uint8_t dac)
{
  FAR struct stm32_hrtim_tim_s *tim;
  uint32_t regval = 0;

  tim = hrtim_tim_get(priv, timer);

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  regval |= (dac << HRTIM_CMNCR_DACSYNC_SHIFT);

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: hrtim_dac_config
 *
 * Description:
 *   Configure HRTIM DAC triggers
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_dac_config(FAR struct stm32_hrtim_s *priv)
{
  FAR struct stm32_hrtim_slave_priv_s *slave_priv;

#ifdef CONFIG_STM32_HRTIM_MASTER_DAC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->master->priv;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_MASTER, dac);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMA_DAC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->tima->priv;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMA, dac);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_DAC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timb->priv;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMB, dac);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC_DAC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timc->priv;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMC, dac);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD_DAC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timd->priv;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMD, dac);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME_DAC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->time->priv;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIME, dac);
#endif

  return OK;
}
#endif

#ifdef HRTIM_HAVE_FAULTS

/****************************************************************************
 * Name: hrtim_tim_faults_cfg
 *
 * Description:
 *   Configure HRTIM Slave Timer faults sources.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   timer  - timer index
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_faults_cfg(FAR struct stm32_hrtim_s *priv, uint8_t timer)
{
  FAR struct stm32_hrtim_tim_s *tim;
  FAR struct stm32_hrtim_slave_priv_s *slave_priv;
  uint32_t regval = 0;

  tim = hrtim_tim_get(priv, timer);

  slave_priv = tim->priv;

  /* Get lock configuration */

  regval = ((slave_priv->flt & HRTIM_TIM_FAULT_LOCK) ? HRTIM_TIMFLT_FLTLCK : 0);

  /* Get sources configuration */

  regval |= slave_priv->flt & 0x1f;

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_FLTR_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: hrtim_faults_config
 *
 * Description:
 *   Configure single HRTIM Fault
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   index  - Fault index
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_flt_cfg(FAR struct stm32_hrtim_s *priv, uint8_t index)
{
  FAR struct stm32_hrtim_fault_cfg_s *flt;
  int ret = OK;
  uint32_t regval = 0;

  /* Get fault configuration */

  switch (index)
    {
#ifdef CONFIG_STM32_HRTIM_FAULT1
      case 1:
        {
          flt = &priv->flt->flt1;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT2
      case 2:
        {
          flt = &priv->flt->flt2;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT3
      case 3:
        {
          flt = &priv->flt->flt3;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT4
      case 4:
        {
          flt = &priv->flt->flt4;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT5
      case 5:
        {
          flt = &priv->flt->flt5;
          break;
        }
#endif
      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Configure fault */

  switch (index)
    {
      /* Fault 1-4 Configuration is located in first common fault register */

      case 1:
      case 2:
      case 3:
      case 4:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR1_OFFSET);

          /* Configure polarity */

          regval |= (((flt->pol & HRTIM_FAULT_POL_HIGH) ? HRTIM_FLTINR1_FLT1P : 0) << (index-1)*8);

          /* Config source */

          regval |= (((flt->src & HRTIM_FAULT_SRC_PIN) ? HRTIM_FLTINR1_FLT1SRC : 0) << (index-1)*8);

          /* Config filter */

          regval |= ((flt->filter << HRTIM_FLTINR1_FLT1F_SHIFT) << (index-1)*8);

          /* Fault enable */

          regval |= (HRTIM_FLTINR1_FLT1E << (index-1)*8);

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_FLTINR1_OFFSET, regval);

          break;
        }

        /* Fault 5 configuration is located in second common fault register */

      case 5:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET);

          /* Configure polarity */

          regval |= ((flt->pol & HRTIM_FAULT_POL_HIGH) ? HRTIM_FLTINR2_FLT5P : 0);

          /* Config source */

          regval |= ((flt->src & HRTIM_FAULT_SRC_PIN) ? HRTIM_FLTINR2_FLT5SRC : 0);

          /* Config filter */

          regval |= ((flt->filter << HRTIM_FLTINR2_FLT5F_SHIFT));

          /* Fault enable */

          regval |= HRTIM_FLTINR2_FLT5E;

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET, regval);

          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_faults_config
 *
 * Description:
 *  Configure HRTIM Faults
 *
 * Input Parameters:
 *  priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_faults_config(FAR struct stm32_hrtim_s *priv)
{
  uint32_t regval = 0;

  /* Configure faults */

#ifdef CONFIG_STM32_HRTIM_FAULT1
  hrtim_flt_cfg(priv, 1);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT2
  hrtim_flt_cfg(priv, 2);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT3
  hrtim_flt_cfg(priv, 3);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT4
  hrtim_flt_cfg(priv, 4);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT5
  hrtim_flt_cfg(priv, 5);
#endif

  /* Configure fault sources in Slave Timers */

#ifdef CONFIG_STM32_HRTIM_TIMA_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

  /* Configure fault sampling clock division */

  regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET);
  regval |= HRTIM_FAULT_SAMPLING << HRTIM_FLTINR1_FLT1F_SHIFT;
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET, regval);

  return OK;
}
#endif

#ifdef HRTIM_HAVE_EEV

/****************************************************************************
 * Name: hrtim_eev_cfg
 *
 * Description:
 *   Configure single HRTIM External Event
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   index  - External Event index
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_eev_cfg(FAR struct stm32_hrtim_s *priv, uint8_t index)
{
  struct stm32_hrtim_eev_cfg_s* eev;
  int ret = OK;
  uint32_t regval = 0;

  /* Get External Event configuration */

  switch (index)
    {
#ifdef CONFIG_STM32_HRTIM_EEV1
      case 1:
        {
          eev = &priv->eev->eev1;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV2
      case 2:
        {
          eev = &priv->eev->eev2;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV3
      case 3:
        {
          eev = &priv->eev->eev3;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV4
      case 4:
        {
          eev = &priv->eev->eev4;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV5
      case 5:
        {
          eev = &priv->eev->eev5;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV6
      case 6:
        {
          eev = &priv->eev->eev6;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV7
      case 7:
        {
          eev = &priv->eev->eev7;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
      case 8:
        {
          eev = &priv->eev->eev8;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
      case 9:
        {
          eev = &priv->eev->eev9;
          break;
        }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV10
      case 10:
        {
          eev = &priv->eev->eev10;
          break;
        }
#endif
      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  switch (index)
    {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_EECR1_OFFSET);

          /* Configure source */

          regval |= ((eev->src << HRTIM_EECR1_EE1SRC_SHIFT) << (index-1)*6);

          /* Configure polarity */

          regval |= ((eev->pol & HRTIM_FAULT_POL_HIGH ? HRTIM_EECR1_EE1POL : 0) << (index-1)*6);

          /* Configure sensitivity */

          regval |= (((eev->sen) << HRTIM_EECR1_EE1SNS_SHIFT) << (index-1)*6);

          /* Configure mode */

          regval |= (((eev->mode & HRTIM_EEV_MODE_FAST) ? HRTIM_EECR1_EE1FAST : 0) << (index-1)*6);

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_EECR1_OFFSET, regval);

          break;
        }
      case 7:
      case 8:
      case 9:
      case 10:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_EECR2_OFFSET);

          /* Configure source */

          regval |= ((eev->src << HRTIM_EECR2_EE6SRC_SHIFT) << (index-6)*6);

          /* Configure polarity */

          regval |= ((eev->pol & HRTIM_FAULT_POL_HIGH ? HRTIM_EECR2_EE6POL : 0) << (index-6)*6);

          /* Configure sensitivity */

          regval |= (((eev->sen) << HRTIM_EECR2_EE6SNS_SHIFT) << (index-6)*6);

          /* Configure External Event filter, only EEV6-10 */

          regval |= (((eev->filter) << HRTIM_EECR2_EE6SNS_SHIFT) << (index-6)*6);

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_EECR2_OFFSET, regval);

          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_events_config
 *
 * Description:
 *   Configure HRTIM External Events
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_events_config(FAR struct stm32_hrtim_s *priv)
{
  uint32_t regval = 0;

  /* Configure Events sources */

#ifdef CONFIG_STM32_HRTIM_EEV1
  hrtim_eev_cfg(priv, 1);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV2
  hrtim_eev_cfg(priv, 2);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV3
  hrtim_eev_cfg(priv, 3);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV4
  hrtim_eev_cfg(priv, 4);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV5
  hrtim_eev_cfg(priv, 5);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV6
  hrtim_eev_cfg(priv, 6);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV7
  hrtim_eev_cfg(priv, 7);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV8
  hrtim_eev_cfg(priv, 8);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV9
  hrtim_eev_cfg(priv, 9);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV10
  hrtim_eev_cfg(priv, 10);
#endif

  /* External Event Sampling clock */

  regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_EECR3_OFFSET);
  regval |= (HRTIM_EEV_SAMPLING << HRTIM_EECR3_EEVSD_SHIFT);
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_EECR3_OFFSET, regval);

  return OK;
}
#endif

/****************************************************************************
 * Name: hrtim_irq_config
 *
 * Description:
 *   Configure HRTIM interrupts
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#ifdef HRTIM_HAVE_INTERRUPTS
static int hrtim_irq_config(FAR struct stm32_hrtim_s *priv)
{
#warning "hrtim_irq_config: missing logic"
  return OK;
}

void hrtim_irq_ack(FAR struct hrtim_dev_s *dev, uint8_t timer, int source);
{
#warning "hrtim_irq_ack: missing logic"
}
#endif

/****************************************************************************
 * Name: hrtim_tim_mode_set
 *
 * Description:
 *  Set HRTIM Timer mode
 *
 * Input parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - HRTIM Timer index
 *   mode   - Timer mode configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_tim_mode_set(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                               uint8_t mode)
{
  uint32_t regval = 0;

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  /* Configure preload */

  if (mode & HRTIM_MODE_PRELOAD)
    {
      regval |= HRTIM_CMNCR_PREEN;
    }

  /* Configure half mode */

  if (mode & HRTIM_MODE_HALF)
    {
      regval |= HRTIM_CMNCR_HALF;
    }

  /* Configure re-triggerable mode */

  if (mode & HRTIM_MODE_RETRIG)
    {
      regval |= HRTIM_CMNCR_RETRIG;
    }

  /* Configure continuous mode */

  if (mode & HRTIM_MODE_CONT)
    {
      regval |= HRTIM_CMNCR_CONT;
    }

  /* Configure push-pull mode. Only Slaves */

  if (mode & HRTIM_MODE_PSHPLL && timer != HRTIM_TIMER_MASTER)
    {
      regval |= HRTIM_TIMCR_PSHPLL;
    }

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, regval);

}

/****************************************************************************
 * Name: hrtim_mode_config
 *
 * Description:
 *   Configure HRTIM Timers mode
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void hrtim_mode_config(FAR struct stm32_hrtim_s *priv)
{

#ifdef CONFIG_STM32_HRTIM_MASTER
  hrtim_tim_mode_set(priv, HRTIM_TIMER_MASTER, priv->master->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMA
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMA, priv->tima->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMB, priv->timb->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMC, priv->timc->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMD, priv->timd->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIME, priv->time->tim.mode);
#endif

}

/****************************************************************************
 * Name: hrtim_cmp_update
 *
 * Description:
 *  Try update HRTIM Timer compare register.
 *
 * Input parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *   index  - Compare register timer
 *   cmp    - New compare register value
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_cmp_update(FAR struct hrtim_dev_s *dev, uint8_t timer,
                            uint8_t index, uint16_t cmp)
{
  FAR struct stm32_hrtim_s *priv = (FAR struct stm32_hrtim_s *)dev->hd_priv;
  int ret = OK;
  uint32_t offset = 0;

  switch (cmp)
    {
      case HRTIM_CMP1:
        {
          offset = STM32_HRTIM_TIM_CMP1R_OFFSET;
          break;
        }

      case HRTIM_CMP2:
        {
          offset = STM32_HRTIM_TIM_CMP2R_OFFSET;
          break;
        }

      case HRTIM_CMP3:
        {
          offset = STM32_HRTIM_TIM_CMP3R_OFFSET;
          break;
        }

      case HRTIM_CMP4:
        {
          offset = STM32_HRTIM_TIM_CMP4R_OFFSET;
          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  hrtim_tim_putreg(priv, timer, offset, cmp);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_per_update
 *
 * Description:
 *  Try update HRTIM Timer period register.
 *
 * Input parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *   per    - New period register value
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_per_update(FAR struct hrtim_dev_s *dev, uint8_t timer,
                            uint16_t per)
{
  FAR struct stm32_hrtim_s *priv = (FAR struct stm32_hrtim_s *)dev->hd_priv;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_PER_OFFSET, per);

  return OK;
}

/****************************************************************************
 * Name: hrtim_per_get
 *
 * Description:
 *  Get HRTIM Timer period value
 *
 * Input parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static uint16_t hrtim_per_get(FAR struct hrtim_dev_s *dev, uint8_t timer)
{
  FAR struct stm32_hrtim_s *priv = (FAR struct stm32_hrtim_s *)dev->hd_priv;

  return (uint16_t)hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_PER_OFFSET);
}

/****************************************************************************
 * Name: hrtim_cmp_update
 *
 * Description:
 *  Get HRTIM Timer compare register
 *
 * Input parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - HRTIM Timer index
 *   index  - Compare register timer
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static uint16_t hrtim_cmp_get(FAR struct hrtim_dev_s *dev, uint8_t timer,
                         uint8_t index)
{
  FAR struct stm32_hrtim_s *priv = (FAR struct stm32_hrtim_s *)dev->hd_priv;
  uint16_t cmpx = 0;
  uint32_t offset = 0;

  switch (index)
    {
      case HRTIM_CMP1:
        {
          offset = STM32_HRTIM_TIM_CMP1R_OFFSET;
          break;
        }

      case HRTIM_CMP2:
        {
          offset = STM32_HRTIM_TIM_CMP2R_OFFSET;
          break;
        }

      case HRTIM_CMP3:
        {
          offset = STM32_HRTIM_TIM_CMP3R_OFFSET;
          break;
        }

      case HRTIM_CMP4:
        {
          offset = STM32_HRTIM_TIM_CMP4R_OFFSET;
          break;
        }

      default:
        {
          cmpx = 0;
          goto errout;
        }
    }

  cmpx = (uint16_t)hrtim_tim_getreg(priv, timer, offset);

errout:
  return cmpx;
}

/****************************************************************************
 * Name: hrtim_tim_reset_set
 *
 * Description:
 *  Set HRTIM Timer Reset events
 *
 * Input parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - HRTIM Timer index
 *   reset  - Reset configuration
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_reset_set(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                               uint32_t reset)
{
  int ret = OK;

  if (timer == HRTIM_TIMER_MASTER)
    {
      ret = -EINVAL;
      goto errout;
    }

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RSTR_OFFSET, reset);

errout:
  return ret;
}

static int hrtim_reset_config(FAR struct stm32_hrtim_s *priv)
{
  FAR struct stm32_hrtim_slave_priv_s *slave_priv;

#ifdef CONFIG_STM32_HRTIM_TIMA
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->tima->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMA, slave_priv->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timb->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMB, slave_priv->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timc->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMC, slave_priv->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timd->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMD, slave_priv->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->time->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIME, slave_priv->reset);
#endif

  return OK;
}

static int hrtim_tim_update_set(FAR struct stm32_hrtim_s *priv, uint8_t timer,
                                uint32_t update)
{
  uint32_t regval = 0;

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  /* TODO: Configure update events */

  /* TODO: Configure update gating */

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, regval);

  return OK;
}

static int hrtim_update_config(FAR struct stm32_hrtim_s *priv)
{
  FAR struct stm32_hrtim_slave_priv_s *slave_priv;

#ifdef CONFIG_STM32_HRTIM_TIMA
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->tima->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMA, slave_priv->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timb->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMB, slave_priv->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timc->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMC, slave_priv->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->timd->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMD, slave_priv->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  slave_priv = (struct stm32_hrtim_slave_priv_s*)priv->time->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIME, slave_priv->update);
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32_hrtimconfig
 *
 * Description:
 *   Configure HRTIM
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int stm32_hrtimconfig(FAR struct stm32_hrtim_s *priv)
{
  int ret;
  uint32_t regval = 0;

  /* HRTIM DLL calibration */

  ret = hrtim_dll_cal(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM DLL calibration failed!\n");
      goto errout;
    }

  /* Configure Timers Clocks */

  ret = hrtim_tim_clocks_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM timers clock configuration failed!\n");
      goto errout;
    }

  /* Configure Timers reset events */

  hrtim_reset_config(priv);

  /* Configure Timers update events */

  hrtim_update_config(priv);

  /* Configure Timers mode */

  hrtim_mode_config(priv);

  /* Configure auto-delayed mode */

#ifdef HRTIM_HAVE_AUTODELAYED
  hrtim_autodelayed_config(priv);
#endif

  /* Configure HRTIM GPIOs */

#if defined(HRTIM_HAVE_CAPTURE) || defined(HRTIM_HAVE_PWM) || defined(HRTIM_HAVE_SYNC)
  ret = hrtim_gpios_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM GPIOs configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure HRTIM inputs */

#if defined(HRTIM_HAVE_CAPTURE)
  ret = hrtim_inputs_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM inputs configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure Synchronisation IOs */

#if defined(HRTIM_HAVE_SYNC)
  ret = hrtim_synch_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM synchronisation configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure HRTIM outputs GPIOs */

#if defined(HRTIM_HAVE_PWM)
  ret = hrtim_outputs_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM outputs configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure ADC triggers */

#ifdef HRTIM_HAVE_ADC
  ret = hrtim_adc_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM ADC configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure DAC synchronization */

#ifdef HRTIM_HAVE_DAC
  ret = hrtim_dac_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM ADC configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure Faults */

#ifdef HRTIM_HAVE_FAULTS
  ret = hrtim_faults_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM faults configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure External Events */

#ifdef HRTIM_HAVE_EEV
  ret = hrtim_events_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM EEV configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure interrupts */

#ifdef HRTIM_HAVE_INTERRUPTS
  ret = hrtim_irq_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM IRQ configuration failed!\n");
      goto errout;
    }
#endif

  /* Enable Master Timer */

  regval |= HRTIM_MCR_MCEN;

  /* Enable Slave Timers */

#ifdef CONFIG_STM32_HRTIM_TIMA
  regval |= HRTIM_MCR_TACEN;
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  regval |= HRTIM_MCR_TBCEN;
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  regval |= HRTIM_MCR_TCCEN;
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  regval |= HRTIM_MCR_TDCEN;
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  regval |= HRTIM_MCR_TECEN;
#endif

  /* Write enable bits at once */

  hrtim_tim_modifyreg(priv, HRTIM_TIMER_MASTER, STM32_HRTIM_TIM_CR_OFFSET, 0, regval);

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hrtiminitialize
 *
 * Description:
 *   Initialize the HRTIM.
 *
 * Returned Value:
 *   Valid HRTIM device structure reference on succcess; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the HRTIM block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct hrtim_dev_s* stm32_hrtiminitialize(void)
{
  FAR struct hrtim_dev_s  *dev;
  FAR struct stm32_hrtim_s *hrtim;
  int ret;

  dev = &g_hrtim1dev;

  hrtim = dev->hd_priv;

  /* configure HRTIM only once */

  if (!dev->initialized)
    {
      ret = stm32_hrtimconfig(hrtim);
      if (ret < 0)
        {
          tmrerr("ERROR: Failed to initialize HRTIM1: %d\n", ret);
          errno = -ret;
          return NULL;
        }

      dev->initialized = true;
    }

  return dev;
}

/****************************************************************************
 * Name: hrtim_register
 ****************************************************************************/

int hrtim_register(FAR const char *path, FAR struct hrtim_dev_s *dev)
{
  int ret ;

  /* Initialize the HRTIM device structure */

  dev->hd_ocount = 0;

  /* Initialize semaphores */

  sem_init(&dev->hd_closesem, 0, 1);

  /* Register the HRTIM character driver */

  ret =  register_driver(path, &hrtim_fops, 0444, dev);
  if (ret < 0)
    {
      sem_destroy(&dev->hd_closesem);
    }

  return ret;
}

#endif  /* CONFIG_STM32_STM32F33XX */
#endif  /* CONFIG_STM32_HRTIM1 */
