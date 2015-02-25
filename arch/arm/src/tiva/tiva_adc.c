/****************************************************************************
 * arch/arm/src/tiva/tiva_adc.c
 *
 *   Copyright (C) 2015 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
 *
 * References:
 *
 *   TM4C123GH6PM Series Data Sheet
 *   TI Tivaware driverlib ADC sample code.
 *
 * The Tivaware sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 * Copyright (c) 2005-2014 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This is part of revision 2.1.0.12573 of the Tiva Peripheral Driver Library.
 *****************************************************************************/

/* Keep in mind that for every step there should be another entry in the
 * CONFIG_ADC_FIFOSIZE value.
 * e.g. if there are 12 steps in use; CONFIG_ADC_FIFOSIZE = 12+1 = 13
 *      if there are  3 steps in use; CONFIG_ADC_FIFOSIZE = 3+1 = 4
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/analog/adc.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "tiva_gpio.h"
#include "tiva_adc.h"
#include "chip/tiva_pinmap.h"
#include "chip/tiva_syscontrol.h"
#include "chip/tiva_adc.h"

#if defined (CONFIG_TIVA_ADC0) || defined (CONFIG_TIVA_ADC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_TIVA_ADC_CLOCK
#  define CONFIG_TIVA_ADC_CLOCK TIVA_ADC_CLOCK_MIN
#endif

#ifdef CONFIG_TIVA_ADC_VREF
#  ifndef CONFIG_ARCH_CHIP_TM4C129
#    error Voltage reference selection only supported in TM4C129 parts
#  endif
#endif

#ifdef CONFIG_TIVA_ADC_ALT_CLK
#  warning CONFIG_TIVA_ADC_ALT_CLK unsupported.
#endif

/* Are we using interrupt-based triggering (opposed to SW triggering)? Then work
 * queues are required.
 */

#if (CONFIG_TIVA_ADC0_SSE0_TRIGGER > 0) || (CONFIG_TIVA_ADC0_SSE1_TRIGGER > 0) || \
    (CONFIG_TIVA_ADC0_SSE2_TRIGGER > 0) || (CONFIG_TIVA_ADC0_SSE3_TRIGGER > 0) || \
    (CONFIG_TIVA_ADC0_SSE0_TRIGGER > 0) || (CONFIG_TIVA_ADC0_SSE1_TRIGGER > 0) || \
    (CONFIG_TIVA_ADC0_SSE2_TRIGGER > 0) || (CONFIG_TIVA_ADC0_SSE3_TRIGGER > 0)
#  define TIVA_ADC_HAVE_INTERRUPTS 1
#endif

#ifdef TIVA_ADC_HAVE_INTERRUPTS
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support is required (CONFIG_SCHED_WORKQUEUE) for ADC interrupts
#  endif
#  ifndef CONFIG_SCHED_HPWORK
#    error High priority worker threads is required (CONFIG_SCHED_HPWORK) for ADC interrupts
#  endif
#endif


#ifndef CONFIG_DEBUG
#  undef CONFIG_TIVA_ADC_REGDEBUG
#endif


/* Misc utility defines *****************************************************/

#define TIVA_ADC_ENABLE  true
#define TIVA_ADC_DISABLE false

#define TIVA_ADC_RESOLUTION 4095

#ifdef CONFIG_ARCH_CHIP_TM4C123
#  define TIVA_ADC_CLOCK_MAX (16000000)
#  define TIVA_ADC_CLOCK_MIN (16000000)
#elif CONFIG_ARCH_CHIP_TM4C129
#  define TIVA_ADC_CLOCK_MAX (32000000)
#  define TIVA_ADC_CLOCK_MIN (16000000)
#else
#  error TIVA_ADC_CLOCK: unsupported architecture
#endif

/* Allow the same function call to be used for sample rate */

#ifdef CONFIG_ARCH_CHIP_TM4C123
#  define TIVA_ADC_SAMPLE_RATE_SLOWEST  (ADC_PC_SR_125K)
#  define TIVA_ADC_SAMPLE_RATE_SLOW     (ADC_PC_SR_250K)
#  define TIVA_ADC_SAMPLE_RATE_FAST     (ADC_PC_SR_500K)
#  define TIVA_ADC_SAMPLE_RATE_FASTEST  (ADC_PC_SR_1M)
#elif CONFIG_ARCH_CHIP_TM4C129
#  define TIVA_ADC_SAMPLE_RATE_SLOWEST  (ADC_PC_MCR_1_8)
#  define TIVA_ADC_SAMPLE_RATE_SLOW     (ADC_PC_MCR_1_4)
#  define TIVA_ADC_SAMPLE_RATE_FAST     (ADC_PC_MCR_1_2)
#  define TIVA_ADC_SAMPLE_RATE_FASTEST  (ADC_PC_MCR_FULL)
#else
#  error TIVA_ADC_SAMPLE_RATE: unsupported architecture
#endif

/* Utility macros ***********************************************************/

/* PWM trigger support definitions ******************************************/

/* Decodes the PWM generator and module from trigger and converts
 * to the TSSEL_PS register
 */

#define ADC_TRIG_PWM_CFG(t) \
    (1<<(ADC_TSSEL_PS_SHIFT(ADC_TRIG_gen(t))))

/* ADC support definitions **************************************************/

#define ADC_CHN_AIN(n)   GPIO_ADC_AIN##n
#define TIVA_ADC_PIN(n)  ADC_CHN_AIN(n)

#define SSE_PROC_TRIG(n)  (1 << (n))
#define SSE_PROC_TRIG_ALL (0xF)

#define SSE_IDX(a,s)      (((a)*SSE_PER_BASE) + (s))

#define MAX_NORMAL_CHN 15
#define BASE_PER_ADC   2
#define SSE_PER_BASE   4
#define SSE_MAX_STEP   8
#define NUM_SSE(n)     (sizeof(n)/sizeof(n[0]))

#define GET_AIN(a,s,c) (uint8_t)((getreg32( \
    TIVA_ADC_BASE(a)+TIVA_ADC_SSMUX(s)) & ADC_SSMUX_MUX_MASK(c)) >> ADC_SSMUX_MUX_SHIFT(c))

#define ADC_SSE_STEP_NULL 0xFF

#define CLOCK_CONFIG(div, src) \
    ( ((((div) << ADC_CC_CLKDIV_SHIFT) & ADC_CC_CLKDIV_MASK) | \
    ((src) & ADC_CC_CS_MASK)) & (ADC_CC_CLKDIV_MASK + ADC_CC_CS_MASK) )

#define SEM_PROCESS_PRIVATE 0
#define SEM_PROCESS_SHARED  1

/* Debug ********************************************************************/

#ifndef CONFIG_DEBUG
#  undef CONFIG_TIVA_ADC_REGDEBUG
#endif

/* ADC event trace logic.  NOTE:  trace uses the internal, non-standard, low-level
 * debug interface syslog() but does not require that any other debug
 * is enabled.
 */

#ifndef CONFIG_ADC_TRACE
#  define tiva_adc_tracereset(p)
#  define tiva_adc_tracenew(p,s)
#  define tiva_adc_traceevent(p,e,a)
#  define tiva_adc_tracedump(p)
#endif

#ifndef CONFIG_ADC_NTRACE
#  define CONFIG_ADC_NTRACE 32
#endif

/****************************************************************************
 * Public Functions
 * **************************************************************************/

/* Upper level ADC driver ***************************************************/

static void tiva_adc_reset(struct adc_dev_s *dev);
static int tiva_adc_setup(struct adc_dev_s *dev);
static void tiva_adc_shutdown(struct adc_dev_s *dev);
static void tiva_adc_rxint(struct adc_dev_s *dev, bool enable);
static int tiva_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* ADC lower half device operations */

static const struct adc_ops_s g_adcops =
{
  .ao_reset = tiva_adc_reset,
  .ao_setup = tiva_adc_setup,
  .ao_shutdown = tiva_adc_shutdown,
  .ao_rxint = tiva_adc_rxint,
  .ao_ioctl = tiva_adc_ioctl,
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* tracks overall ADC peripherals one-time initialization state */
struct tiva_adc_state_s
{
  bool init[BASE_PER_ADC];
  bool sse[BASE_PER_ADC * SSE_PER_BASE];
};

struct tiva_adc_s
{
  struct adc_dev_s *dev;
  bool ena;                   /* Operation state */
  uint8_t devno;              /* ADC device number */
  struct tiva_adc_sse_s *sse[SSE_PER_BASE];   /* Sample sequencer operation
                                               * state */
#ifdef CONFIG_TIVA_ADC_REGDEBUG
  /* Debug stuff */

  bool wrlast;                /* Last was a write */
  uintptr_t addrlast;         /* Last address */
  uint32_t vallast;           /* Last value */
  int ntimes;                 /* Number of times */
#endif                        /* CONFIG_TIVA_ADC_REGDEBUG */
};

struct tiva_adc_sse_s
{
  struct tiva_adc_s *adc;     /* Parent peripheral */
#ifdef TIVA_ADC_HAVE_INTERRUPTS
  sem_t exclsem;              /* Mutual exclusion semaphore */
  struct work_s work;         /* Supports the interrupt handling "bottom half" */
#endif
  bool ena;                   /* Sample sequencer operation state */
  uint32_t irq;               /* SSE interrupt vectors */
  uint8_t num;                /* SSE number */
};

/****************************************************************************
 * Private Function Definitions
 ****************************************************************************/

/* Debug ADC functions **********************************************/

#if defined(CONFIG_TIVA_ADC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool tiva_adc_checkreg(struct tiva_adc_s *priv, bool wr,
                              uint32_t regval, uintptr_t address);
#endif

#ifdef CONFIG_TIVA_ADC_REGDEBUG
static void tiva_adc_modifyreg(struct tiva_adc_s *priv, unsigned int addr,
                               uint32_t clearbits, uint32_t setbits);
#else
#  define tiva_adc_modifyreg(priv,addr,clearbits,setbits) modifyreg32(addr,clearbits,setbits)
#endif

/* TM4C-specific ADC functions **********************************************/

/* Common peripheral level */

static int adc_state(struct tiva_adc_s *adc, bool state);
static void adc_clock(uint32_t freq);
#ifdef CONFIG_ARCH_CHIP_TM4C129
static void adc_vref(uint8_t vref);
#endif
#ifdef CONFIG_TIVA_ADC_INTERRUPTS
static void tiva_adc_read(void *arg);
#endif

/* Peripheral (base) level */

static void adc_sample_rate(uint8_t rate);
static void adc_proc_trig(struct tiva_adc_s *adc, uint8_t sse_mask);
static uint32_t adc_int_status(struct tiva_adc_s *adc);

/* Sample Sequencer (SSE) level */

static void sse_state(struct tiva_adc_s *adc, uint8_t sse, bool state);
static void sse_trigger(struct tiva_adc_s *adc, uint8_t sse, uint32_t trigger);
#ifdef CONFIG_EXPERIMENTAL
static void sse_pwm_trig_ioctl(struct tiva_adc_s *adc, uint8_t sse, uint32_t cfg);
#endif
static int sse_data(struct tiva_adc_s *adc, uint8_t sse);
static void sse_priority(struct tiva_adc_s *adc, uint8_t sse, uint8_t priority);

static void sse_int_state(struct tiva_adc_s *adc, uint8_t sse, bool state);
static bool sse_int_status(struct tiva_adc_s *adc, uint8_t sse);
static void sse_clear_int(struct tiva_adc_s *adc, uint8_t sse);

static void sse_register_chn(struct tiva_adc_s *adc, uint8_t sse, uint8_t chn,
                             uint32_t ain);
static void sse_differential(struct tiva_adc_s *adc, uint8_t sse, uint8_t chn,
                             uint32_t diff);
#ifdef CONFIG_EXPERIMENTAL
static void sse_sample_hold_time(struct tiva_adc_s *adc, uint8_t sse,
                                 uint8_t chn, uint32_t shold);
#endif
static void sse_step_cfg(struct tiva_adc_s *adc, uint8_t sse, uint8_t chn,
                         uint8_t cfg);

/* Helper functions *********************************************************/

#ifdef CONFIG_TIVA_ADC0
static void tiva_adc0_sse_init(void);
static void tiva_adc0_assign_channels(void);
#  ifdef TIVA_ADC_HAVE_INTERRUPTS
static void tiva_adc0_assign_interrupts(void);
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE0
static void adc0_sse0_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc0_sse0_interrupt(int irq, void *context);
#    endif
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE1
static void adc0_sse1_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc0_sse1_interrupt(int irq, void *context);
#    endif
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE2
static void adc0_sse2_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc0_sse2_interrupt(int irq, void *context);
#    endif
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE3
static void adc0_sse3_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc0_sse3_interrupt(int irq, void *context);
#    endif
#  endif
#endif

#ifdef CONFIG_TIVA_ADC1
static void tiva_adc1_sse_init(void);
static void tiva_adc1_assign_channels(void);
#  ifdef TIVA_ADC_HAVE_INTERRUPTS
static void tiva_adc1_assign_interrupts(void);
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE0
static void adc1_sse0_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc1_sse0_interrupt(int irq, void *context);
#    endif
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE1
static void adc1_sse1_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc1_sse1_interrupt(int irq, void *context);
#    endif
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE2
static void adc1_sse2_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc1_sse2_interrupt(int irq, void *context);
#    endif
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE3
static void adc1_sse3_chn_cfg(void);
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
static void adc1_sse3_interrupt(int irq, void *context);
#    endif
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Tracks overall peripheral state */

static struct tiva_adc_state_s adc_common =
{
  .init =
  {
    false,
    false
  },
  .sse =
  {
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false
  },
};

#ifdef CONFIG_TIVA_ADC0

/* ADC device instance 0 */

static struct adc_dev_s g_adcdev0;
static struct tiva_adc_s adc0;
#  ifdef CONFIG_TIVA_ADC0_SSE0
static struct tiva_adc_sse_s sse00;
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE1
static struct tiva_adc_sse_s sse01;
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE2
static struct tiva_adc_sse_s sse02;
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE3
static struct tiva_adc_sse_s sse03;
#  endif
#endif

#ifdef CONFIG_TIVA_ADC1

/* ADC device instance 1 */

static struct adc_dev_s g_adcdev1;
static struct tiva_adc_s adc1;
#  ifdef CONFIG_TIVA_ADC1_SSE0
static struct tiva_adc_sse_s sse10;
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE1
static struct tiva_adc_sse_s sse11;
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE2
static struct tiva_adc_sse_s sse12;
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE3
static struct tiva_adc_sse_s sse13;
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_adc_reset
 *
 * Description:
 *   Reset the ADC device. Called early to initialize the hardware. This is
 *   called before tiva_adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void tiva_adc_reset(struct adc_dev_s *dev)
{
  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;

  avdbg("Resetting...\n");

  /* Only if ADCs are active do we run the reset routine: - disable ADC
   * interrupts - clear interrupt bits - disable all active sequences
   * Otherwise, if the peripheral is inactive, perform no operations since
   * register access to a peripheral that is not active will result in a
   * segmentation fault.
   */

  if (priv->ena)
    {
      tiva_adc_rxint(dev, TIVA_ADC_DISABLE);
      uint8_t s;
      for (s = 0; s < SSE_PER_BASE; ++s)
        {
          if (adc_common.sse[SSE_IDX(priv->devno, s)])
            {
              sse_state(priv, s, TIVA_ADC_DISABLE);
            }
        }
    }
}

/****************************************************************************
 * Name: tiva_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened. This will occur when the port is first opened.
 *   this setup includes configuring and attaching ADC interrupts. Interrupts
 *   are all disabled upon return.
 *
 * Returned Value:
 *   Non negative value on success; negative value on failure.
 *
 ****************************************************************************/

static int tiva_adc_setup(struct adc_dev_s *dev)
{
  avdbg("Setup\n");

  /* Only if ADCs are active do we run the reset routine: - enable ADC
   * interrupts - clear interrupt bits - enable all active sequences - register
   * triggers and respective interrupt handlers Otherwise, if the peripheral is
   * inactive, perform no operations since register access to a peripheral that
   * is not active will result in a segmentation fault.
   */

  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;
  uint8_t s = 0;
  for (s = 0; s < SSE_PER_BASE; ++s)
    {
      if (adc_common.sse[SSE_IDX(priv->devno, s)])
        {
          sse_state(priv, s, true);
        }
    }

  tiva_adc_rxint(dev, false);
  return OK;
}

/****************************************************************************
 * Name: tiva_adc_shutdown
 *
 * Description:
 *   Disable the ADC. This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void tiva_adc_shutdown(struct adc_dev_s *dev)
{
  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;

  avdbg("Shutdown\n");

  /* Reset the ADC peripheral */

  tiva_adc_reset(dev);

  uint8_t s = 0;
  for (s = 0; s < SSE_PER_BASE; ++s)
    {
      if (adc_common.sse[SSE_IDX(priv->devno, s)])
        {
          /* Disable ADC interrupts at the level of the AIC */

          up_disable_irq(priv->sse[s]->irq);

          /* Then detach the ADC interrupt handler. */

          irq_detach(priv->sse[s]->irq);
        }
    }
}

/****************************************************************************
 * Name: tiva_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 * Input Parameters:
 *   enable - the enable state of interrupts for this device
 *
 ****************************************************************************/

static void tiva_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;

  avdbg("rx enable=%d\n", enable);

  uint8_t s = 0;
  for (s = 0; s < SSE_PER_BASE; ++s)
    {
      uint32_t trigger =
        (tiva_adc_getreg(priv, TIVA_ADC_EMUX(priv->devno)) >> s) & 0xF;
      if (adc_common.sse[SSE_IDX(priv->devno, s)] && (trigger > 0))
        {
          sse_int_state(priv, s, enable);
        }
    }
}

/****************************************************************************
 * Name: tiva_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   cmd - ADC ioctl command
 *   arg - argument for the ioctl command
 *
 * Returned Value:
 *   Non negative value on success; negative value on failure.
 *
 ****************************************************************************/

static int tiva_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;
  int ret = OK;
  uint32_t regval = 0;
  uint8_t sse = 0;

  avdbg("cmd=%d arg=%ld\n", cmd, arg);

  switch (cmd)
    {
      /* Software trigger */

      case ANIOC_TRIGGER:

        sse = (uint8_t) arg;

        /* start conversion and read to buffer */

        adc_proc_trig(priv, (uint8_t) SSE_PROC_TRIG(sse));
        regval = adc_int_status(priv) & (1 << sse);
        while (!regval)
          {
            regval = adc_int_status(priv) & (1 << sse);
          }

        sse_clear_int(priv, sse);
        sse_data(priv, sse);
        break;

      /* PWM triggering */

#warning Missing Logic

      /* TODO: Needs to be tested */

#ifdef CONFIG_EXPERIMENTAL

      case TIVA_ADC_PWM_TRIG_IOCTL:

        /* Verify input SSE trigger is a PWM trigger */

        sse = (uint8_t)(arg & 0x2);
        regval = (tiva_adc_getreg(priv, (TIVA_ADC_EMUX(adc->devno))) >>
            ADC_EMUX_SHIFT(sse)) & ADC_EMUX_MASK(sse);

        if ((regval == ADC_EMUX_PWM0) ||
            (regval == ADC_EMUX_PWM1) ||
            (regval == ADC_EMUX_PWM2) ||
            (regval == ADC_EMUX_PWM3))
          {
            sse_pwm_trig_ioctl(priv, sse, (uint32_t)(arg&0xFFFFFFFC));
          }

        break;

#endif

#warning Missing Logic

      /* Unsupported or invalid command */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: tiva_adc_read
 *
 * Description:
 *   This function executes on the worker thread.  It is scheduled by
 *   sam_adc_interrupt whenever any enabled event occurs. All interrupts
 *   are disabled when this function runs. tiva_adc_read will
 *   re-enable interrupts when it completes processing all pending events.
 *
 * Input Parameters
 *   arg - The ADC SSE data structure cast to (void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tiva_adc_read(void *arg)
{
  struct tiva_adc_sse_s *sse = (struct tiva_adc_sse_s *)arg;
  uint32_t data = 0;
  uint8_t fifo_count = 0;

  /* Get exclusive access to the driver data structure */

  tiva_adc_lock(sse->adc, sse->num);

  /* Get sampled data */

  while (!(tiva_adc_getreg(&adc0, TIVA_ADC_BASE(0) + TIVA_ADC_SSFSTAT(0)) &
          ADC_SSFSTAT_EMPTY) && fifo_count < SSE_MAX_STEP)
    {
      data = tiva_adc_getreg(&adc0, TIVA_ADC_BASE(0) + TIVA_ADC_SSFIFO(0));
      (void)adc_receive(adc0.dev, GET_AIN(0, 0, fifo_count), data);
      ++fifo_count;
    }

  /* Exit, re-enabling ADC interrupts */

  sse_int_state(&adc0, 0, TIVA_ADC_ENABLE);

  /* Release our lock on the ADC structure */

  tiva_adc_unlock(sse->adc, sse->num);
}

/****************************************************************************
 * Register Operations
 ****************************************************************************/

#ifdef CONFIG_TIVA_ADC_REGDEBUG

/****************************************************************************
 * Name: tiva_adc_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

static bool tiva_adc_checkreg(struct tiva_adc_s *priv, bool wr,
                              uint32_t regval, uintptr_t address)
{
  if (wr == priv->wrlast &&             /* Same kind of access? */
      regval == priv->vallast &&        /* Same value? */
      address == priv->addrlast)        /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast = wr;
      priv->vallast = regval;
      priv->addrlast = address;
      priv->ntimes = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif /* CONFIG_TIVA_ADC_REGDEBUG */

/****************************************************************************
 * Name: tiva_adc_modifyreg
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 * Input Parameters:
 *   addr - The address of the register to write to
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_ADC_REGDEBUG
static void tiva_adc_modifyreg(struct tiva_adc_s *priv, unsigned int addr,
                               uint32_t clearbits, uint32_t setbits)
{
  uint32_t regval = 0;
  irqstate_t flags = irqsave();
  regval = tiva_adc_getreg(priv, addr);
  regval &= ~clearbits;
  regval |= setbits;
  tiva_adc_putreg(priv, addr, regval);
  irqrestore(flags);
}
#endif /* CONFIG_TIVA_ADC_REGDEBUG */

/* TM4C-specific ADC functions **********************************************/

/* Peripheral (base) level **************************************************/

/****************************************************************************
 * Name: adc_state
 *
 * Description:
 *  Toggles the operational state of the ADC peripheral
 *
 * Input Parameters:
 *  state - operation state
 *
 ****************************************************************************/

static int adc_state(struct tiva_adc_s *adc, bool state)
{
  if (state == TIVA_ADC_ENABLE)
    {
      /* Enable clocking to the ADC peripheral */

#ifdef TIVA_SYSCON_RCGCADC
      modifyreg32(TIVA_SYSCON_RCGCADC, 0, 1 << adc->devno);
#else
      modifyreg32(TIVA_SYSCON_RCGC0, 0, SYSCON_RCGC0_ADC0);
#endif
      return OK;
    }
  else if (state == TIVA_ADC_DISABLE)
    {
      /* Disable clocking to the ADC peripheral */

#ifdef TIVA_SYSCON_RCGCADC
      modifyreg32(TIVA_SYSCON_RCGCADC, 1 << adc->devno, 0);
#else
      modifyreg32(TIVA_SYSCON_RCGC0, SYSCON_RCGC0_ADC0, 0);
#endif
      return OK;
    }

  /* ERROR! */

  return -1;
}

/****************************************************************************
 * Name: adc_clock
 *
 * Description:
 *  Sets the ADC peripherals clock to the desired frequency.
 *
 * Input Parameters:
 *  freq - ADC clock value; dependent on platform:
 *
 *  TM4C123 - Select either MOSC or PIOSC. Both result in 16 MHz operation,
 *  however the PIOSC allows the ADC to operate in deep sleep mode.
 *
 *  TM4C129 - For the 129, there is still a selection between various internal
 *  clocks, however the output frequency is variable (16 MHz - 32 MHz); so it
 *  is much more intuitive to allow the clock variable be a frequency value.
 *
 ****************************************************************************/

static void adc_clock(uint32_t freq)
{
#if defined(CONFIG_ARCH_CHIP_TM4C123)
  /* For the TM4C123, the ADC clock source does not affect the frequency, it
   * runs at 16 MHz regardless. You end up selecting between the MOSC (default)
   * or the PIOSC. The PIOSC allows the ADC to operate even in deep sleep mode.
   * Since this is the case, the clock value for
   */

  uintptr_t ccreg = (TIVA_ADC0_BASE + TIVA_ADC_CC_OFFSET);
  modifyreg32(ccreg, 0, (freq & ADC_CC_CS_MASK));

#elif defined (CONFIG_ARCH_CHIP_TM4C129)
  /* check clock bounds and specific match cases */

  uint32_t clk_src = 0;
  uint32_t div = 0;
  if (clock > TIVA_ADC_CLOCK_MAX)
    {
      clk_src = ADC_CC_CS_SYSPLL;
      div = (BOARD_FVCO_FREQUENCY / TIVA_ADC_CLOCK_MAX);
    }
  else if (clock < TIVA_ADC_CLOCK_MIN)
    {
      clk_src = ADC_CC_CS_PIOSC;
      div = 1;
    }
  else if (clock == XTAL_FREQUENCY)
    {
      clk_src = ADC_CC_CS_MOSC;
      div = 1;
    }
  else
    {
      clk_src = ADC_CC_CS_SYSPLL;
      div = (BOARD_FVCO_FREQUENCY / freq);
    }

  uintptr_t ccreg = (TIVA_ADC0_BASE + TIVA_ADC_CC_OFFSET);
  modifyreg32(ccreg, 0, CLOCK_CONFIG(div, clk_src));
#else
#  error Unsupported architecture reported
#endif
}

/****************************************************************************
 * Name: adc_vref
 *
 * Description:
 *  Sets the ADC peripherals clock to the desired frequency.
 *
 * Input Parameters:
 *  vref - ADC clock voltage reference source
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_TM4C129
static void adc_vref(uint8_t vref)
{
  uintptr_t ctlreg = (TIVA_ADC0_BASE + TIVA_ADC_CTL_OFFSET);
  if (vref == 0)
    {
      modifyreg32(ctlreg, vref, 0);
    }
  else
    {
      modifyreg32(ctlreg, 0, vref);
    }
}
#endif

/****************************************************************************
 * Name: adc_sample_rate
 *
 * Description:
 *  Sets the ADC sample rate as follows for each processor.
 *  TM4C123 - by maximum samples: 125 ksps, 250 ksps, 500 ksps or 1 Msps
 *  TM4C129 - by a divisor either being full, half, quarter or
 *  an eighth.
 *
 * Input Parameters:
 *  rate - ADC sample rate divisor
 *
 ****************************************************************************/

static void adc_sample_rate(uint8_t rate)
{
  uintptr_t pcreg = (TIVA_ADC0_BASE + TIVA_ADC_PC_OFFSET);

  /* NOTE: ADC_PC_SR_MASK is intended for use with the TM4C123, the
   * alternative is ADC_PC_MCR_MASK for the TM4C129. However both masks
   * mask off the first 4 bits (0xF) so there is no need to distinguish
   * between the two.
   */

  modifyreg32(pcreg, 0, (rate & ADC_PC_SR_MASK));
}

/****************************************************************************
 * Name: adc_proc_trig
 *
 * Description:
 *   Triggers the sample sequence to start it's conversion(s) and store them
 *   to the FIFO. This is only required when the trigger source is set to the
 *   processor.
 *
 * Input parameters:
 *   adc - which ADC peripherals' sample sequencers to trigger
 *   sse_mask - sample sequencer bitmask, each sse is 1 shifted by the sse
 *              number. e.g.
 *              SSE0 = 1 << 0
 *              SSE1 = 1 << 1
 *              SSE2 = 1 << 2
 *              SSE3 = 1 << 3
 *
 ****************************************************************************/

static void adc_proc_trig(struct tiva_adc_s *adc, uint8_t sse_mask)
{
  uintptr_t pssireg = TIVA_ADC_PSSI(adc->devno);
  tiva_adc_modifyreg(adc, pssireg, 0, sse_mask);
#ifdef CONFIG_TIVA_ADC_SYNC
#  warning CONFIG_TIVA_ADC_SYNC unsupported at this time.
#endif
}

/****************************************************************************
 * Name: adc_int_status
 *
 * Description:
 *   Returns raw interrupt status for the input ADC
 *
 * Input parameters:
 *   adc - which ADC peripherals' interrupt status to retrieve
 *
 ****************************************************************************/

static uint32_t adc_int_status(struct tiva_adc_s *adc)
{
  return tiva_adc_getreg(adc, TIVA_ADC_RIS(adc->devno));
}

/* Sample sequencer (SSE) functions *****************************************/

/****************************************************************************
 * Name: sse_state
 *
 * Description:
 *   Sets the operation state of an ADC's sample sequencer (SSE). SSEs must
 *   be configured before being enabled.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   state - sample sequencer enable/disable state
 *
 ****************************************************************************/

static void sse_state(struct tiva_adc_s *adc, uint8_t sse, bool state)
{
  uintptr_t actssreg = TIVA_ADC_ACTSS(adc->devno);
  if (state == TIVA_ADC_ENABLE)
    {
      tiva_adc_modifyreg(adc, actssreg, 0, (1 << sse));
    }
  else
    {
      tiva_adc_modifyreg(adc, actssreg, (1 << sse), 0);
    }

  adc->sse[sse]->ena = state;
}

/****************************************************************************
 * Name: sse_trigger
 *
 * Description:
 *   Sets the trigger configuration for an ADC's sample sequencer (SSE).
 *   Possible triggers are the following:
 *      - Processor
 *      - PWMs, requires that one of the PWMnn_TRIG_CFG defines be OR'd
 *        into the trigger value.
 *      - Timers
 *      - GPIO (which GPIO is platform specific, consult the datasheet)
 *      - Always
 *      - !!UNSUPPORTED: Comparators
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   trigger - interrupt trigger
 *
 ****************************************************************************/

static void sse_trigger(struct tiva_adc_s *adc, uint8_t sse, uint32_t trigger)
{
  uintptr_t emuxreg = (TIVA_ADC_EMUX(adc->devno));
  uint32_t trig = 0;
  if ((trigger & ADC_EMUX_MASK(0)) == 0)
    {
      /* The 0 value is a special case since using modifyregn() results in an
       * ORing of the register value; we need to unset those bits if it's a 0.
       */

      tiva_adc_modifyreg(adc, emuxreg, (0xF << sse), 0);
    }
  else
    {
      trig = ((trigger << ADC_EMUX_SHIFT(sse)) & ADC_EMUX_MASK(sse));
      tiva_adc_modifyreg(adc, emuxreg, 0, trig);
    }

  /* NOTE: PWM triggering needs an additional register to be set (ADC_TSSEL)
   * A platform specific IOCTL command is provided to configure the triggering.
   */
}

/****************************************************************************
 * Name: sse_pwm_trig_ioctl
 *
 * Description:
 *   Additional triggering configuration for PWM. Sets which PWM and which
 *   generator.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   cfg - which PWM modulator and generator to use, use TIVA_ADC_PWM_TRIG
 *         to encode the value correctly
 *
 ****************************************************************************/

#ifdef CONFIG_EXPERIMENTAL
static void sse_pwm_trig_ioctl(struct tiva_adc_s *adc, uint8_t sse, uint32_t cfg)
{
  /* PWM triggering needs an additional register to be set (ADC_TSSEL) */

  uintptr_t tsselreg = TIVA_ADC_TSSEL(adc->devno);

  if ((cfg & ADC_EMUX_MASK(0)) == 1)
    {
      tiva_adc_modifyreg(adc, tsselreg, 0, cfg);
    }
  else
    {
      tiva_adc_modifyreg(adc, tsselreg, cfg, 0);
    }
}
#endif

/****************************************************************************
 * Name: sse_int
 *
 * Description:
 *   Sets the interrupt state of an ADC's sample sequencer (SSE). SSEs must
 *   be enabled before setting interrupt state.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   state - sample sequencer enable/disable interrupt state
 *
 ****************************************************************************/

static void sse_int_state(struct tiva_adc_s *adc, uint8_t sse, bool state)
{
  uintptr_t imreg = TIVA_ADC_IM(adc->devno);
  if (adc->sse[sse])
    {
      sse_clear_int(adc, sse);
    }
  if (state == TIVA_ADC_ENABLE)
    {
      tiva_adc_modifyreg(adc, imreg, 0, (1 << sse));
    }
  else
    {
      tiva_adc_modifyreg(adc, imreg, (1 << sse), 0);
    }
}

/****************************************************************************
 * Name: sse_int_status
 *
 * Description:
 *   Returns interrupt status for the specificed SSE
 *
 * Input parameters:
 *   adc - which ADC peripherals' interrupt status to retrieve
 *   sse - which SSE interrupt status to retrieve
 *
 ****************************************************************************/

static bool sse_int_status(struct tiva_adc_s *adc, uint8_t sse)
{
  return (adc_int_status(adc) & (1 << sse)) > 0 ? true : false;
}

/****************************************************************************
 * Name: sse_clear_int
 *
 * Description:
 *   Clears the interrupt bit for the SSE.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   state - sample sequencer
 *
 ****************************************************************************/

static void sse_clear_int(struct tiva_adc_s *adc, uint8_t sse)
{
  uintptr_t iscreg = TIVA_ADC_ISC(adc->devno);
  tiva_adc_modifyreg(adc, iscreg, 0, (1 << sse));
}

/****************************************************************************
 * Name: sse_data
 *
 * Description:
 *   Retrieves data from the FIFOs for all steps in the given sample sequencer.
 *   The input data buffer MUST be as large or larger than the sample sequencer.
 *   otherwise
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *
 * Return value:
 *   number of steps read from FIFO.
 *
 ****************************************************************************/

static int sse_data(struct tiva_adc_s *adc, uint8_t sse)
{
  uint32_t ssfstatreg =
    tiva_adc_getreg(adc, TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSFSTAT(sse));
  int32_t data = 0;
  uint8_t fifo_count = 0;

  /* Read samples from the FIFO until it is empty */

  while (!(ssfstatreg & ADC_SSFSTAT_EMPTY) && fifo_count < SSE_MAX_STEP)
    {
      /* Read the FIFO and copy it to the destination */

      data =
        tiva_adc_getreg(adc, TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSFIFO(sse));
      (void)adc_receive(adc->dev, GET_AIN(adc->devno, sse, fifo_count), data);
      fifo_count++;

      /* refresh fifo status register state */

      ssfstatreg =
        tiva_adc_getreg(adc, TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSFSTAT(sse));
    }

  return fifo_count;
}

/****************************************************************************
 * Name: sse_priority
 *
 * Description:
 *   Sets the priority configuration for an ADC's sample sequencer (SSE). The
 *   priority value ranges from 0 to 3, 0 being the highest priority, 3 being
 *   the lowest. There can be no duplicate values.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   priority - conversion priority
 *
 ****************************************************************************/

static void sse_priority(struct tiva_adc_s *adc, uint8_t sse, uint8_t priority)
{
  uintptr_t ssprireg = TIVA_ADC_SSPRI(adc->devno);
  uint32_t sspri = 0;

  if (priority == 0)
    {
      /* The 0 value is a special case since using modifyregn() results in an
       * ORing of the register value; we need to unset those bits if it's a 0.
       */

      sspri = (ADC_SSPRI_MASK(sse) & (0x3 << ADC_SSPRI_SHIFT(sse)));
      tiva_adc_modifyreg(adc, ssprireg, sspri, 0);
    }
  else
    {
      sspri = (ADC_SSPRI_MASK(sse) & (priority << ADC_SSPRI_SHIFT(sse)));
      tiva_adc_modifyreg(adc, ssprireg, 0, sspri);
    }
}

/****************************************************************************
 * Name: sse_register_chn
 *
 * Description:
 *   Registers an input channel to an SSE. Channels are registered according
 *   to the step and channel values stored in the channel struct. If the SSE
 *   already has a channel registered, it is overwritten by the new channel.
 *
 *   *SSEMUX only supported on TM4C129 devices
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer step
 *   ain - analog input pin
 *
 ****************************************************************************/

static void sse_register_chn(struct tiva_adc_s *adc, uint8_t sse, uint8_t chn,
                             uint32_t ain)
{
  /* Configure SSE mux (SSMUX) with step number */

  uintptr_t ssmuxreg = (TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSMUX(sse));
  uint32_t step = 0;

  if (ain > 0)
    {
      step = ((ain << ADC_SSMUX_MUX_SHIFT(chn)) & ADC_SSMUX_MUX_MASK(chn));
      tiva_adc_modifyreg(adc, ssmuxreg, 0, step);
    }
  else
    {
      step = ((0xF << ADC_SSMUX_MUX_SHIFT(chn)) & ADC_SSMUX_MUX_MASK(chn));
      tiva_adc_modifyreg(adc, ssmuxreg, step, 0);
    }

#ifdef CONFIG_ARCH_CHIP_TM4C129
  /* Configure SSE extended mux (SSEMUX) with step number and configuration */

  ssmuxreg = (TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSMUX(sse));
  step = ((1 << ADC_SSEMUX_MUX_SHIFT(chn)) & ADC_SSEMUX_MUX_MASK(chn));
  if (chn > MAX_NORMAL_CHN)
    {
      tiva_adc_modifyreg(adc, ssmuxreg, 0, step);
    }
  else
    {
      tiva_adc_modifyreg(adc, ssmuxreg, step, 0);
    }
#endif
}

/****************************************************************************
 * Name: sse_differential
 *
 * Description:
 *   Sets the differential capability for a SSE. !! UNSUPPORTED
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer channel
 *   diff - differential configuration
 *
 ****************************************************************************/

static void sse_differential(struct tiva_adc_s *adc, uint8_t sse, uint8_t chn,
                             uint32_t diff)
{
#ifdef CONFIG_TIVA_ADC_DIFFERENTIAL
#  error CONFIG_TIVA_ADC_DIFFERENTIAL unsupported!!
#else

  /* for now, ensure the FIFO is used and differential sampling is disabled */

  uintptr_t ssopreg = (TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSOP(sse));
  uint32_t sdcopcfg = (1 << chn);
  tiva_adc_modifyreg(adc, ssopreg, sdcopcfg, 0);
#endif
}

/****************************************************************************
 * Name: sse_sample_hold_time
 *
 * Description:
 *  Set the sample and hold time for this step.
 *
 *  This is not available on all devices, however on devices that do not
 *  support this feature these reserved bits are ignored on write access.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer channel
 *   shold - sample and hold time
 *
 ****************************************************************************/

#ifdef CONFIG_EXPERIMENTAL
static void sse_sample_hold_time(struct tiva_adc_s *adc, uint8_t sse,
                                 uint8_t chn, uint32_t shold)
{
  uintptr_t sstshreg = (TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSTSH(sse));
  if (shold > 0)
    {
      tiva_adc_modifyreg(adc, sstshreg, 0, (shold << ADC_SSTSH_SHIFT(sse)));
    }
  else
    {
      tiva_adc_modifyreg(adc, sstshreg, ADC_SSTSH_MASK(sse), 0);
    }
}
#endif

/****************************************************************************
 * Name: sse_step_cfg
 *
 * Description:
 *   Configures the given SSE step to one of the following options:
 *      -Temperature sensor select: this step is muxed to the internal
 *       temperature sensor.
 *      -Interrupt enabled select: this step causes the interrupt bit to
 *       be set and, if the MASK0 bit in ADC_IM register is set, the
 *       interrupt is promoted to the interrupt controller.
 *      -Sequence end select: This step is the last sequence to be sampled.
 *       This MUST be set somewhere in the SSE.
 *      -*Comparator/Differential select: The analog input is differentially
 *       sampled. The corresponding ADCSSMUXn nibble must be set to the pair
 *       number "i", where the paired inputs are "2i and 2i+1". Because the
 *       temperature sensor does not have a differential option, this bit must
 *       not be set when the TS3 bit is set.
 *
 *  *Comparator/Differential functionality is unsupported and ignored.
 *
 * Input parameters:
 *   adc - peripheral state
 *   sse - sample sequencer
 *   chn - sample sequencer channel
 *   cfg - step configuration
 *
 ****************************************************************************/

static void sse_step_cfg(struct tiva_adc_s *adc, uint8_t sse, uint8_t chn,
                         uint8_t cfg)
{
  uintptr_t ssctlreg = (TIVA_ADC_BASE(adc->devno) + TIVA_ADC_SSCTL(sse));
  uint32_t ctlcfg = cfg << ADC_SSCTL_SHIFT(chn);
  tiva_adc_modifyreg(adc, ssctlreg, 0, ctlcfg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_adc_initialize
 *
 * Description:
 *   Initialize the ADC
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *tiva_adc_initialize(int adc_num)
{
  avdbg("tiva_adc_initialize\n");

  /* Initialize the private ADC device data structure */

  struct tiva_adc_s *adc;
  uint8_t s;
#ifdef CONFIG_TIVA_ADC0
  if (adc_num == 0)
    {
      adc0.ena = false;
      adc0.devno = 0;

      /* Debug stuff */

#  ifdef CONFIG_TIVA_ADC_REGDEBUG
      adc0.wrlast = false;
      adc0.addrlast = 0x0;
      adc0.vallast = 0x0;
      adc0.ntimes = 0;
#  endif                             /* CONFIG_TIVA_ADC_REGDEBUG */

      /* Initialize SSEs */

#  ifdef CONFIG_TIVA_ADC0_SSE0
      sse00.ena = false;
      sse00.irq = TIVA_IRQ_ADC0;
      sse00.num = 0;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse00.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif
      adc0.sse[0] = &sse00;
#  endif                             /* CONFIG_TIVA_ADC0_SSE0 */

#  ifdef CONFIG_TIVA_ADC0_SSE1
      sse01.ena = false;
      sse01.irq = TIVA_IRQ_ADC1;
      sse01.num = 1;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse01.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif
      adc0.sse[1] = &sse01;
#  endif                             /* CONFIG_TIVA_ADC0_SSE1 */

#  ifdef CONFIG_TIVA_ADC0_SSE2
      sse02.ena = false;
      sse02.irq = TIVA_IRQ_ADC2;
      sse02.num = 2;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse02.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif
      adc0.sse[2] = &sse02;
#  endif                             /* CONFIG_TIVA_ADC0_SSE2 */

#  ifdef CONFIG_TIVA_ADC0_SSE3
      sse03.ena = false;
      sse03.irq = TIVA_IRQ_ADC3;
      sse03.num = 3;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse03.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif
      adc0.sse[3] = &sse03;
#  endif                             /* CONFIG_TIVA_ADC0_SSE3 */

      adc0.dev = &g_adcdev0;
      adc = &adc0;

      /* Initialize the public ADC device data structure */

      g_adcdev0.ad_ops = &g_adcops;
      g_adcdev0.ad_priv = &adc0;
    }
#endif                               /* CONFIG_TIVA_ADC0 */

#ifdef CONFIG_TIVA_ADC1
  if (adc_num == 1)
    {
      adc1.ena = false;
      adc1.devno = 0;

      /* Debug stuff */

#  ifdef CONFIG_TIVA_ADC_REGDEBUG
      adc1.wrlast = false;
      adc1.addrlast = 0x0;
      adc1.vallast = 0x0;
      adc1.ntimes = 0;
#  endif                             /* CONFIG_TIVA_ADC_REGDEBUG */

      /* Initialize SSEs */

#  ifdef CONFIG_TIVA_ADC1_SSE0
      sse10.ena = false;
      sse10.irq = TIVA_IRQ_ADC1_0;
      sse10.num = 0;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse10.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif
      adc1.sse[0] = &sse10;
#  endif                             /* CONFIG_TIVA_ADC1_SSE0 */

#  ifdef CONFIG_TIVA_ADC1_SSE1
      sse11.ena = false;
      sse11.irq = TIVA_IRQ_ADC1_1;
      sse11.num = 1;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse11.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif

      adc1.sse[1] = &sse11;
#  endif                             /* CONFIG_TIVA_ADC1_SSE1 */

#  ifdef CONFIG_TIVA_ADC1_SSE2
      sse12.ena = false;
      sse12.irq = TIVA_IRQ_ADC1_2;
      sse12.num = 2;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse12.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif
      adc1.sse[2] = &sse12;
#  endif                             /* CONFIG_TIVA_ADC1_SSE2 */

#  ifdef CONFIG_TIVA_ADC1_SSE3
      sse13.ena = false;
      sse13.irq = TIVA_IRQ_ADC1_3;
      sse13.num = 3;
#    ifdef TIVA_ADC_HAVE_INTERRUPTS
      sem_init(&sse13.exclsem, SEM_PROCESS_PRIVATE, 1);
#    endif
      adc1.sse[3] = &sse13;
#  endif                             /* CONFIG_TIVA_ADC1_SSE3 */

      adc1.dev = &g_adcdev1;
      adc = &adc1;

      /* Initialize the public ADC device data structure */

      g_adcdev1.ad_ops = &g_adcops;
      g_adcdev1.ad_priv = &adc1;
    }
#endif                               /* CONFIG_TIVA_ADC1 */

  if (adc_num > 1)
    {
      adbg("ERROR: Invalid ADV devno given, must be 0 or 1! ADC Devno: %d\n",
           adc_num);
      return NULL;
    }

  /* Have the common peripheral properties already been initialized? If yes,
   * continue.
   */

  if (adc_common.init[adc->devno] == false)
    {
      /* turn on peripheral */

      if (adc_state(adc, TIVA_ADC_ENABLE) < 0)
        {
          adbg("ERROR: failure to power ADC peripheral (devno=%d)\n",
               adc_num);
          return NULL;
        }

      /* set clock */

      adc_clock(CONFIG_TIVA_ADC_CLOCK);

      /* set sampling rate */

      adc_sample_rate(TIVA_ADC_SAMPLE_RATE_FASTEST);

#ifdef CONFIG_ARCH_CHIP_TM4C129
      /* voltage reference */

      adc_vref();
#endif                               /* CONFIG_ARCH_CHIP_TM4C129 */
      adc_common.init[adc->devno] = true;
    }

  /* Initialize peripheral */

  /* Have we already been initialized? If yes, than just hand out the interface
   * one more time.
   */

  if (adc->ena == false)
    {
#if CONFIG_TIVA_ADC0
      if (adc_num == 0)
        {
          /* Configure sample sequencers */

          tiva_adc0_sse_init();

          /* Configure channels & register interrupts */

#  if TIVA_ADC_HAVE_INTERRUPTS
          tiva_adc0_assign_interrupts();
#  endif
          tiva_adc0_assign_channels();
        }
#endif

#if CONFIG_TIVA_ADC1
      if (adc_num == 1)
        {
          /* Configure sample sequencers */

          tiva_adc1_sse_init();

          /* Configure channels & register interrupts */

#  if TIVA_ADC_HAVE_INTERRUPTS
          tiva_adc1_assign_interrupts();
#  endif
          tiva_adc1_assign_channels();
        }
#endif

      /* Enable SSEs */

      for (s = 0; s < SSE_PER_BASE; ++s)
        {
          if (adc_common.sse[SSE_IDX(adc_num, s)])
            {
              sse_state(adc, s, TIVA_ADC_ENABLE);
              sse_clear_int(adc, s);
            }
        }

      /* Now we are initialized */

      adc->ena = true;
    }

  /* Return a pointer to the device structure */

  avdbg("Returning %x\n", adc->dev);
  return adc->dev;
}

/****************************************************************************
 * Name: tiva_adc_lock
 *
 * Description:
 *   Get exclusive access to the ADC interface
 *
 ****************************************************************************/

void tiva_adc_lock(FAR struct tiva_adc_s *priv, int sse)
{
#if TIVA_ADC_HAVE_INTERRUPTS
  int ret;

  avdbg("Locking\n");

  do
    {
      ret = sem_wait(&priv->sse[sse]->exclsem);

      /* This should only fail if the wait was canceled by an signal (and the
       * worker thread will receive a lot of signals).
       */

      DEBUGASSERT(ret == OK || errno == EINTR);
    }
  while (ret < 0);
#endif
}

/****************************************************************************
 * Name: tiva_adc_unlock
 *
 * Description:
 *   Relinquish the lock on the ADC interface
 *
 ****************************************************************************/

void tiva_adc_unlock(FAR struct tiva_adc_s *priv, int sse)
{
#if TIVA_ADC_HAVE_INTERRUPTS
  avdbg("Unlocking\n");
  sem_post(&priv->sse[sse]->exclsem);
#endif
}

/****************************************************************************
 * Name: tiva_adc_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_ADC_REGDEBUG
uint32_t tiva_adc_getreg(struct tiva_adc_s *priv, uintptr_t address)
{
  uint32_t regval = getreg32(address);

  if (tiva_adc_checkreg(priv, false, regval, address))
    {
      lldbg("%08x->%08x\n", address, regval);
    }

  return regval;
}

/****************************************************************************
 * Name: tiva_adc_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address.
 *
 ****************************************************************************/

void tiva_adc_putreg(struct tiva_adc_s *priv, uintptr_t address,
                     uint32_t regval)
{
  if (tiva_adc_checkreg(priv, true, regval, address))
    {
      lldbg("%08x<-%08x\n", address, regval);
    }

  putreg32(regval, address);
}
#endif                               /* CONFIG_TIVA_ADC_REGDEBUG */

/****************************************************************************
 * Name: Verbose, generated code
 *
 * Description:
 *  Generated with a python script, the following code is used to deal with
 *  the defines generated from the Kconfig menu. Read at your own risk.
 *
 ****************************************************************************/

/* Sample sequencer initialization ******************************************/

#ifdef CONFIG_TIVA_ADC0
static void tiva_adc0_sse_init(void)
{
#  ifdef CONFIG_TIVA_ADC0_SSE0
  sse_state(&adc0, 0, TIVA_ADC_DISABLE);
  sse_priority(&adc0, 0, CONFIG_TIVA_ADC0_SSE0_PRIORITY);
  sse_trigger(&adc0, 0, CONFIG_TIVA_ADC0_SSE0_TRIGGER);
  adc_common.sse[SSE_IDX(0, 0)] = true;
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE1
  sse_state(&adc0, 1, TIVA_ADC_DISABLE);
  sse_priority(&adc0, 1, CONFIG_TIVA_ADC0_SSE1_PRIORITY);
  sse_trigger(&adc0, 1, CONFIG_TIVA_ADC0_SSE1_TRIGGER);
  adc_common.sse[SSE_IDX(0, 1)] = true;
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE2
  sse_state(&adc0, 2, TIVA_ADC_DISABLE);
  sse_priority(&adc0, 2, CONFIG_TIVA_ADC0_SSE2_PRIORITY);
  sse_trigger(&adc0, 2, CONFIG_TIVA_ADC0_SSE2_TRIGGER);
  adc_common.sse[SSE_IDX(0, 2)] = true;
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE3
  sse_state(&adc0, 3, TIVA_ADC_DISABLE);
  sse_priority(&adc0, 3, CONFIG_TIVA_ADC0_SSE3_PRIORITY);
  sse_trigger(&adc0, 3, CONFIG_TIVA_ADC0_SSE3_TRIGGER);
  adc_common.sse[SSE_IDX(0, 3)] = true;
#  endif
}
#endif                               /* CONFIG_TIVA_ADC0 */

#ifdef CONFIG_TIVA_ADC1
static void tiva_adc1_sse_init(void)
{
#  ifdef CONFIG_TIVA_ADC1_SSE0
  sse_state(&adc1, 0, TIVA_ADC_DISABLE);
  sse_priority(&adc1, 0, CONFIG_TIVA_ADC1_SSE0_PRIORITY);
  sse_trigger(&adc1, 0, CONFIG_TIVA_ADC1_SSE0_TRIGGER);
  adc_common.sse[SSE_IDX(1, 0)] = true;
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE1
  sse_state(&adc1, 1, TIVA_ADC_DISABLE);
  sse_priority(&adc1, 1, CONFIG_TIVA_ADC1_SSE1_PRIORITY);
  sse_trigger(&adc1, 1, CONFIG_TIVA_ADC1_SSE1_TRIGGER);
  adc_common.sse[SSE_IDX(1, 1)] = true;
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE2
  sse_state(&adc1, 2, TIVA_ADC_DISABLE);
  sse_priority(&adc1, 2, CONFIG_TIVA_ADC1_SSE2_PRIORITY);
  sse_trigger(&adc1, 2, CONFIG_TIVA_ADC1_SSE2_TRIGGER);
  adc_common.sse[SSE_IDX(1, 2)] = true;
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE3
  sse_state(&adc1, 3, TIVA_ADC_DISABLE);
  sse_priority(&adc1, 3, CONFIG_TIVA_ADC1_SSE3_PRIORITY);
  sse_trigger(&adc1, 3, CONFIG_TIVA_ADC1_SSE3_TRIGGER);
  adc_common.sse[SSE_IDX(1, 3)] = true;
#  endif
}
#endif                               /* CONFIG_TIVA_ADC1 */

/* Sample sequencer interrupt initialization ********************************/

#ifdef TIVA_ADC_HAVE_INTERRUPTS
static void tiva_adc0_assign_interrupts(void)
{
#  ifdef CONFIG_TIVA_ADC0
  uint32_t ret = 0;
#    ifdef CONFIG_TIVA_ADC0_SSE0
  ret = irq_attach(sse00.irq, (xcpt_t)adc0_sse0_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse00.irq, ret);
      return;
    }
  up_enable_irq(sse00.irq);
#    endif
#    ifdef CONFIG_TIVA_ADC0_SSE1
  ret = irq_attach(sse01.irq, (xcpt_t)adc0_sse1_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse01.irq, ret);
      return;
    }
  up_enable_irq(sse01.irq);
#    endif
#    ifdef CONFIG_TIVA_ADC0_SSE2
  ret = irq_attach(sse02.irq, (xcpt_t)adc0_sse2_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse02.irq, ret);
      return;
    }
  up_enable_irq(sse02.irq);
#    endif
#    ifdef CONFIG_TIVA_ADC0_SSE3
  ret = irq_attach(sse03.irq, (xcpt_t)adc0_sse3_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse03.irq, ret);
      return;
    }
  up_enable_irq(sse03.irq);
#    endif
#  endif
};

static void tiva_adc1_assign_interrupts(void)
{
#  ifdef CONFIG_TIVA_ADC1
  uint32_t ret = 0;
#    ifdef CONFIG_TIVA_ADC1_SSE0
  ret = irq_attach(sse10.irq, (xcpt_t)adc1_sse0_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse10.irq, ret);
      return;
    }
  up_enable_irq(sse10.irq);
#    endif
#    ifdef CONFIG_TIVA_ADC1_SSE1
  ret = irq_attach(sse11.irq, (xcpt_t)adc1_sse1_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse11.irq, ret);
      return;
    }
  up_enable_irq(sse11.irq);
#    endif
#    ifdef CONFIG_TIVA_ADC1_SSE2
  ret = irq_attach(sse12.irq, (xcpt_t)adc1_sse2_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse12.irq, ret);
      return;
    }
  up_enable_irq(sse12.irq);
#    endif
#    ifdef CONFIG_TIVA_ADC1_SSE3
  ret = irq_attach(sse13.irq, (xcpt_t)adc1_sse3_interrupt);
  if (ret < 0)
    {
      adbg("ERROR: Failed to attach IRQ %d: %d\n", sse13.irq, ret);
      return;
    }
  up_enable_irq(sse13.irq);
#    endif
#  endif
};
#endif /* TIVA_ADC_HAVE_INTERRUPTS */

/* Sample sequencer interrupt declaration ********************************/

#ifdef TIVA_ADC_HAVE_INTERRUPTS
#  ifdef CONFIG_TIVA_ADC0
#    ifdef CONFIG_TIVA_ADC0_SSE0
static void adc0_sse0_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse00.ena == true) &&
              (adc_common.sse[SSE_IDX(0, 0)] == true));

  /* disable further  interrupts. Interrupts will be re-enabled
   * after the worker thread executes.
   */

  sse_int_state(&adc0, 0, TIVA_ADC_DISABLE);

  /* Clear interrupt status */

  sse_clear_int(&adc0, 0);

  /* Transfer processing to the worker thread.  Since interrupts are
   * disabled while the work is pending, no special action should be
   * required to protected the work queue.
   */

  DEBUGASSERT(sse00.work.worker == NULL);
  ret = work_queue(HPWORK, &sse00.work, tiva_adc_read, &sse00, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d\n", ret);
    }
}
#    endif
#    ifdef CONFIG_TIVA_ADC0_SSE1
static void adc0_sse1_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse01.ena == true) &&
              (adc_common.sse[SSE_IDX(0, 1)] == true));

  sse_int_state(&adc0, 1, TIVA_ADC_DISABLE);
  sse_clear_int(&adc0, 1);
  DEBUGASSERT(sse01.work.worker == NULL);
  ret = work_queue(HPWORK, &sse01.work, tiva_adc_read, &sse01, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, adc0.devno, sse01.num);
    }
}
#    endif
#    ifdef CONFIG_TIVA_ADC0_SSE2
static void adc0_sse2_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse02.ena == true) &&
              (adc_common.sse[SSE_IDX(0, 2)] == true));

  sse_int_state(&adc0, 2, TIVA_ADC_DISABLE);
  sse_clear_int(&adc0, 2);
  DEBUGASSERT(sse02.work.worker == NULL);
  ret = work_queue(HPWORK, &sse02.work, tiva_adc_read, &sse02, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, adc0.devno, sse02.num);
    }
}
#    endif
#    ifdef CONFIG_TIVA_ADC0_SSE3
static void adc0_sse3_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse03.ena == true) &&
              (adc_common.sse[SSE_IDX(0, 3)] == true));

  sse_int_state(&adc0, 3, TIVA_ADC_DISABLE);
  sse_clear_int(&adc0, 3);
  DEBUGASSERT(sse03.work.worker == NULL);
  ret = work_queue(HPWORK, &sse03.work, tiva_adc_read, &sse03, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, adc0.devno, sse03.num);
    }
}
#    endif
#  endif /* CONFIG_TIVA_ADC0 */

#  ifdef CONFIG_TIVA_ADC1
#    ifdef CONFIG_TIVA_ADC1_SSE0
static void adc1_sse0_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse10.ena == true) &&
              (adc_common.sse[SSE_IDX(1, 0)] == true));

  sse_int_state(&adc1, 0, TIVA_ADC_DISABLE);
  sse_clear_int(&adc1, 0);
  DEBUGASSERT(sse10.work.worker == NULL);
  ret = work_queue(HPWORK, &sse10.work, tiva_adc_read, &sse10, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, adc1.devno, sse10.num);
    }
}
#    endif
#    ifdef CONFIG_TIVA_ADC1_SSE1
static void adc1_sse1_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse11.ena == true) &&
              (adc_common.sse[SSE_IDX(1, 1)] == true));

  sse_int_state(&adc1, 1, TIVA_ADC_DISABLE);
  sse_clear_int(&adc1, 1);
  DEBUGASSERT(sse11.work.worker == NULL);
  ret = work_queue(HPWORK, &sse11.work, tiva_adc_read, &sse11, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, adc1.devno, sse11.num);
    }
}
#    endif
#    ifdef CONFIG_TIVA_ADC1_SSE2
static void adc1_sse2_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse12.ena == true) &&
              (adc_common.sse[SSE_IDX(1, 2)] == true));

  sse_int_state(&adc1, 2, TIVA_ADC_DISABLE);
  sse_clear_int(&adc1, 2);
  DEBUGASSERT(sse12.work.worker == NULL);
  ret = work_queue(HPWORK, &sse12.work, tiva_adc_read, &sse12, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, adc1.devno, sse12.num);
    }
}
#    endif
#    ifdef CONFIG_TIVA_ADC1_SSE3
static void adc1_sse3_interrupt(int irq, void *context)
{
  int ret;

  DEBUGASSERT((sse13.ena == true) &&
              (adc_common.sse[SSE_IDX(1, 3)] == true));

  sse_int_state(&adc1, 3, TIVA_ADC_DISABLE);
  sse_clear_int(&adc1, 3);
  DEBUGASSERT(sse13.work.worker == NULL);
  ret = work_queue(HPWORK, &sse13.work, tiva_adc_read, &sse13, 0);
  if (ret != 0)
    {
      adbg("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, adc1.devno, sse13.num);
    }
}
#    endif
#  endif /* CONFIG_TIVA_ADC1 */
#endif /* TIVA_ADC_HAVE_INTERRUPTS */

/* Channel assignment *******************************************************/

#ifdef CONFIG_TIVA_ADC0
static void tiva_adc0_assign_channels(void)
{
#  ifdef CONFIG_TIVA_ADC0_SSE0
  adc0_sse0_chn_cfg();
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE1
  adc0_sse1_chn_cfg();
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE2
  adc0_sse2_chn_cfg();
#  endif
#  ifdef CONFIG_TIVA_ADC0_SSE3
  adc0_sse3_chn_cfg();
#  endif
}
#endif

#ifdef CONFIG_TIVA_ADC1
static void tiva_adc1_assign_channels(void)
{
#  ifdef CONFIG_TIVA_ADC1_SSE0
  adc1_sse0_chn_cfg();
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE1
  adc1_sse1_chn_cfg();
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE2
  adc1_sse2_chn_cfg();
#  endif
#  ifdef CONFIG_TIVA_ADC1_SSE3
  adc1_sse3_chn_cfg();
#  endif
}
#endif

/* Channel config ***********************************************************/

#ifdef CONFIG_TIVA_ADC0
#  ifdef CONFIG_TIVA_ADC0_SSE0
static void adc0_sse0_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC0_SSE0_STEP0
#      ifdef CONFIG_TIVA_ADC0_SSE0_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP0_AIN));
  sse_register_chn(&adc0, 0, 0, CONFIG_TIVA_ADC0_SSE0_STEP0_AIN);
  sse_differential(&adc0, 0, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 0, ADC_SSTH_SHOLD_16);
#        endif
#        ifndef CONFIG_TIVA_ADC0_SSE0_STEP1
  sse_step_cfg(&adc0, 0, 0, chncfg | ADC_SSCTL_END);
#        else
  sse_step_cfg(&adc0, 0, 0, chncfg);
#        endif
#      ifdef CONFIG_TIVA_ADC0_SSE0_STEP1
#        ifdef CONFIG_TIVA_ADC0_SSE0_STEP1_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#        else
  chncfg = ADC_SSCTL_IE;
#        endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP1_AIN));
  sse_register_chn(&adc0, 0, 1, CONFIG_TIVA_ADC0_SSE0_STEP1_AIN);
  sse_differential(&adc0, 0, 1, 0);
#          if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 1, ADC_SSTH_SHOLD_16);
#          endif
#          ifndef CONFIG_TIVA_ADC0_SSE0_STEP2
  sse_step_cfg(&adc0, 0, 1, chncfg | ADC_SSCTL_END);
#          else
  sse_step_cfg(&adc0, 0, 1, chncfg);
#          endif
#        ifdef CONFIG_TIVA_ADC0_SSE0_STEP2
#          ifdef CONFIG_TIVA_ADC0_SSE0_STEP2_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#          else
  chncfg = ADC_SSCTL_IE;
#          endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP2_AIN));
  sse_register_chn(&adc0, 0, 2, CONFIG_TIVA_ADC0_SSE0_STEP2_AIN);
  sse_differential(&adc0, 0, 2, 0);
#            if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 2, ADC_SSTH_SHOLD_16);
#            endif
#            ifndef CONFIG_TIVA_ADC0_SSE0_STEP3
  sse_step_cfg(&adc0, 0, 2, chncfg | ADC_SSCTL_END);
#            else
  sse_step_cfg(&adc0, 0, 2, chncfg);
#            endif
#          ifdef CONFIG_TIVA_ADC0_SSE0_STEP3
#            ifdef CONFIG_TIVA_ADC0_SSE0_STEP3_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#            else
  chncfg = ADC_SSCTL_IE;
#            endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP3_AIN));
  sse_register_chn(&adc0, 0, 3, CONFIG_TIVA_ADC0_SSE0_STEP3_AIN);
  sse_differential(&adc0, 0, 3, 0);
#              if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 3, ADC_SSTH_SHOLD_16);
#              endif
#              ifndef CONFIG_TIVA_ADC0_SSE0_STEP4
  sse_step_cfg(&adc0, 0, 3, chncfg | ADC_SSCTL_END);
#              else
  sse_step_cfg(&adc0, 0, 3, chncfg);
#              endif
#            ifdef CONFIG_TIVA_ADC0_SSE0_STEP4
#              ifdef CONFIG_TIVA_ADC0_SSE0_STEP4_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#              else
  chncfg = ADC_SSCTL_IE;
#              endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP4_AIN));
  sse_register_chn(&adc0, 0, 4, CONFIG_TIVA_ADC0_SSE0_STEP4_AIN);
  sse_differential(&adc0, 0, 4, 0);
#                if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 4, ADC_SSTH_SHOLD_16);
#                endif
#                ifndef CONFIG_TIVA_ADC0_SSE0_STEP5
  sse_step_cfg(&adc0, 0, 4, chncfg | ADC_SSCTL_END);
#                else
  sse_step_cfg(&adc0, 0, 4, chncfg);
#                endif
#              ifdef CONFIG_TIVA_ADC0_SSE0_STEP5
#                ifdef CONFIG_TIVA_ADC0_SSE0_STEP5_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#                else
  chncfg = ADC_SSCTL_IE;
#                endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP5_AIN));
  sse_register_chn(&adc0, 0, 5, CONFIG_TIVA_ADC0_SSE0_STEP5_AIN);
  sse_differential(&adc0, 0, 5, 0);
#                  if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 5, ADC_SSTH_SHOLD_16);
#                  endif
#                  ifndef CONFIG_TIVA_ADC0_SSE0_STEP6
  sse_step_cfg(&adc0, 0, 5, chncfg | ADC_SSCTL_END);
#                  else
  sse_step_cfg(&adc0, 0, 5, chncfg);
#                  endif
#                ifdef CONFIG_TIVA_ADC0_SSE0_STEP6
#                  ifdef CONFIG_TIVA_ADC0_SSE0_STEP6_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#                  else
  chncfg = ADC_SSCTL_IE;
#                  endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP6_AIN));
  sse_register_chn(&adc0, 0, 6, CONFIG_TIVA_ADC0_SSE0_STEP6_AIN);
  sse_differential(&adc0, 0, 6, 0);
#                    if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 6, ADC_SSTH_SHOLD_16);
#                    endif
#                    ifndef CONFIG_TIVA_ADC0_SSE0_STEP7
  sse_step_cfg(&adc0, 0, 6, chncfg | ADC_SSCTL_END);
#                    else
  sse_step_cfg(&adc0, 0, 6, chncfg);
#                    endif
#                  ifdef CONFIG_TIVA_ADC0_SSE0_STEP7
#                    ifdef CONFIG_TIVA_ADC0_SSE0_STEP7_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#                    else
  chncfg = ADC_SSCTL_IE;
#                    endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE0_STEP7_AIN));
  sse_register_chn(&adc0, 0, 7, CONFIG_TIVA_ADC0_SSE0_STEP7_AIN);
  sse_differential(&adc0, 0, 7, 0);
#                      if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 0, 7, ADC_SSTH_SHOLD_16);
#                      endif
  sse_step_cfg(&adc0, 0, 7, chncfg | ADC_SSCTL_END);
#                  endif /* CONFIG_TIVA_ADC0_SSE0_STEP7 */
#                endif /* CONFIG_TIVA_ADC0_SSE0_STEP6 */
#              endif /* CONFIG_TIVA_ADC0_SSE0_STEP5 */
#            endif /* CONFIG_TIVA_ADC0_SSE0_STEP4 */
#          endif /* CONFIG_TIVA_ADC0_SSE0_STEP3 */
#        endif /* CONFIG_TIVA_ADC0_SSE0_STEP2 */
#      endif /* CONFIG_TIVA_ADC0_SSE0_STEP1 */
#    endif /* CONFIG_TIVA_ADC0_SSE0_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC0_SSE0 */

#  ifdef CONFIG_TIVA_ADC0_SSE1
static void adc0_sse1_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC0_SSE1_STEP0
#      ifdef CONFIG_TIVA_ADC0_SSE1_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE1_STEP0_AIN));
  sse_register_chn(&adc0, 1, 0, CONFIG_TIVA_ADC0_SSE1_STEP0_AIN);
  sse_differential(&adc0, 1, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 1, 0, ADC_SSTH_SHOLD_16);
#        endif
#        ifndef CONFIG_TIVA_ADC0_SSE1_STEP1
  sse_step_cfg(&adc0, 1, 0, chncfg | ADC_SSCTL_END);
#        else
  sse_step_cfg(&adc0, 1, 0, chncfg);
#        endif
#      ifdef CONFIG_TIVA_ADC0_SSE1_STEP1
#        ifdef CONFIG_TIVA_ADC0_SSE1_STEP1_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#        else
  chncfg = ADC_SSCTL_IE;
#        endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE1_STEP1_AIN));
  sse_register_chn(&adc0, 1, 1, CONFIG_TIVA_ADC0_SSE1_STEP1_AIN);
  sse_differential(&adc0, 1, 1, 0);
#          if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 1, 1, ADC_SSTH_SHOLD_16);
#          endif
#          ifndef CONFIG_TIVA_ADC0_SSE1_STEP2
  sse_step_cfg(&adc0, 1, 1, chncfg | ADC_SSCTL_END);
#          else
  sse_step_cfg(&adc0, 1, 1, chncfg);
#          endif
#        ifdef CONFIG_TIVA_ADC0_SSE1_STEP2
#          ifdef CONFIG_TIVA_ADC0_SSE1_STEP2_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#          else
  chncfg = ADC_SSCTL_IE;
#          endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE1_STEP2_AIN));
  sse_register_chn(&adc0, 1, 2, CONFIG_TIVA_ADC0_SSE1_STEP2_AIN);
  sse_differential(&adc0, 1, 2, 0);
#            if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 1, 2, ADC_SSTH_SHOLD_16);
#            endif
#            ifndef CONFIG_TIVA_ADC0_SSE1_STEP3
  sse_step_cfg(&adc0, 1, 2, chncfg | ADC_SSCTL_END);
#            else
  sse_step_cfg(&adc0, 1, 2, chncfg);
#            endif
#          ifdef CONFIG_TIVA_ADC0_SSE1_STEP3
#            ifdef CONFIG_TIVA_ADC0_SSE1_STEP3_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#            else
  chncfg = ADC_SSCTL_IE;
#            endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE1_STEP3_AIN));
  sse_register_chn(&adc0, 1, 3, CONFIG_TIVA_ADC0_SSE1_STEP3_AIN);
  sse_differential(&adc0, 1, 3, 0);
#              if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 1, 3, ADC_SSTH_SHOLD_16);
#              endif
  sse_step_cfg(&adc0, 1, 3, chncfg | ADC_SSCTL_END);
#          endif /* CONFIG_TIVA_ADC0_SSE1_STEP3 */
#        endif /* CONFIG_TIVA_ADC0_SSE1_STEP2 */
#      endif /* CONFIG_TIVA_ADC0_SSE1_STEP1 */
#    endif /* CONFIG_TIVA_ADC0_SSE1_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC0_SSE1 */

#  ifdef CONFIG_TIVA_ADC0_SSE2
static void adc0_sse2_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC0_SSE2_STEP0
#      ifdef CONFIG_TIVA_ADC0_SSE2_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE2_STEP0_AIN));
  sse_register_chn(&adc0, 2, 0, CONFIG_TIVA_ADC0_SSE2_STEP0_AIN);
  sse_differential(&adc0, 2, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 2, 0, ADC_SSTH_SHOLD_16);
#        endif
#        ifndef CONFIG_TIVA_ADC0_SSE2_STEP1
  sse_step_cfg(&adc0, 2, 0, chncfg | ADC_SSCTL_END);
#        else
  sse_step_cfg(&adc0, 2, 0, chncfg);
#        endif
#      ifdef CONFIG_TIVA_ADC0_SSE2_STEP1
#        ifdef CONFIG_TIVA_ADC0_SSE2_STEP1_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#        else
  chncfg = ADC_SSCTL_IE;
#        endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE2_STEP1_AIN));
  sse_register_chn(&adc0, 2, 1, CONFIG_TIVA_ADC0_SSE2_STEP1_AIN);
  sse_differential(&adc0, 2, 1, 0);
#          if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 2, 1, ADC_SSTH_SHOLD_16);
#          endif
#          ifndef CONFIG_TIVA_ADC0_SSE2_STEP2
  sse_step_cfg(&adc0, 2, 1, chncfg | ADC_SSCTL_END);
#          else
  sse_step_cfg(&adc0, 2, 1, chncfg);
#          endif
#        ifdef CONFIG_TIVA_ADC0_SSE2_STEP2
#          ifdef CONFIG_TIVA_ADC0_SSE2_STEP2_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#          else
  chncfg = ADC_SSCTL_IE;
#          endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE2_STEP2_AIN));
  sse_register_chn(&adc0, 2, 2, CONFIG_TIVA_ADC0_SSE2_STEP2_AIN);
  sse_differential(&adc0, 2, 2, 0);
#            if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 2, 2, ADC_SSTH_SHOLD_16);
#            endif
#            ifndef CONFIG_TIVA_ADC0_SSE2_STEP3
  sse_step_cfg(&adc0, 2, 2, chncfg | ADC_SSCTL_END);
#            else
  sse_step_cfg(&adc0, 2, 2, chncfg);
#            endif
#          ifdef CONFIG_TIVA_ADC0_SSE2_STEP3
#            ifdef CONFIG_TIVA_ADC0_SSE2_STEP3_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#            else
  chncfg = ADC_SSCTL_IE;
#            endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE2_STEP3_AIN));
  sse_register_chn(&adc0, 2, 3, CONFIG_TIVA_ADC0_SSE2_STEP3_AIN);
  sse_differential(&adc0, 2, 3, 0);
#              if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 2, 3, ADC_SSTH_SHOLD_16);
#              endif
  sse_step_cfg(&adc0, 2, 3, chncfg | ADC_SSCTL_END);
#          endif /* CONFIG_TIVA_ADC0_SSE2_STEP3 */
#        endif /* CONFIG_TIVA_ADC0_SSE2_STEP2 */
#      endif /* CONFIG_TIVA_ADC0_SSE2_STEP1 */
#    endif /* CONFIG_TIVA_ADC0_SSE2_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC0_SSE2 */

#  ifdef CONFIG_TIVA_ADC0_SSE3
static void adc0_sse3_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC0_SSE3_STEP0
#      ifdef CONFIG_TIVA_ADC0_SSE3_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC0_SSE3_STEP0_AIN));
  sse_register_chn(&adc0, 3, 0, CONFIG_TIVA_ADC0_SSE3_STEP0_AIN);
  sse_differential(&adc0, 3, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC0_SSE3_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc0, 3, 0, ADC_SSTH_SHOLD_16);
#        endif
  sse_step_cfg(&adc0, 3, 0, chncfg | ADC_SSCTL_END);
#    endif /* CONFIG_TIVA_ADC0_SSE3_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC0_SSE3 */
#endif /* CONFIG_TIVA_ADC0 */

#ifdef CONFIG_TIVA_ADC1
#  ifdef CONFIG_TIVA_ADC1_SSE0
static void adc1_sse0_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC1_SSE0_STEP0
#      ifdef CONFIG_TIVA_ADC1_SSE0_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP0_AIN));
  sse_register_chn(&adc1, 0, 0, CONFIG_TIVA_ADC1_SSE0_STEP0_AIN);
  sse_differential(&adc1, 0, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 0, ADC_SSTH_SHOLD_16);
#        endif
#        ifndef CONFIG_TIVA_ADC1_SSE0_STEP1
  sse_step_cfg(&adc1, 0, 0, chncfg | ADC_SSCTL_END);
#        else
  sse_step_cfg(&adc1, 0, 0, chncfg);
#        endif
#      ifdef CONFIG_TIVA_ADC1_SSE0_STEP1
#        ifdef CONFIG_TIVA_ADC1_SSE0_STEP1_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#        else
  chncfg = ADC_SSCTL_IE;
#        endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP1_AIN));
  sse_register_chn(&adc1, 0, 1, CONFIG_TIVA_ADC1_SSE0_STEP1_AIN);
  sse_differential(&adc1, 0, 1, 0);
#          if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 1, ADC_SSTH_SHOLD_16);
#          endif
#          ifndef CONFIG_TIVA_ADC1_SSE0_STEP2
  sse_step_cfg(&adc1, 0, 1, chncfg | ADC_SSCTL_END);
#          else
  sse_step_cfg(&adc1, 0, 1, chncfg);
#          endif
#        ifdef CONFIG_TIVA_ADC1_SSE0_STEP2
#          ifdef CONFIG_TIVA_ADC1_SSE0_STEP2_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#          else
  chncfg = ADC_SSCTL_IE;
#          endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP2_AIN));
  sse_register_chn(&adc1, 0, 2, CONFIG_TIVA_ADC1_SSE0_STEP2_AIN);
  sse_differential(&adc1, 0, 2, 0);
#            if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 2, ADC_SSTH_SHOLD_16);
#            endif
#            ifndef CONFIG_TIVA_ADC1_SSE0_STEP3
  sse_step_cfg(&adc1, 0, 2, chncfg | ADC_SSCTL_END);
#            else
  sse_step_cfg(&adc1, 0, 2, chncfg);
#            endif
#          ifdef CONFIG_TIVA_ADC1_SSE0_STEP3
#            ifdef CONFIG_TIVA_ADC1_SSE0_STEP3_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#            else
  chncfg = ADC_SSCTL_IE;
#            endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP3_AIN));
  sse_register_chn(&adc1, 0, 3, CONFIG_TIVA_ADC1_SSE0_STEP3_AIN);
  sse_differential(&adc1, 0, 3, 0);
#              if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 3, ADC_SSTH_SHOLD_16);
#              endif
#              ifndef CONFIG_TIVA_ADC1_SSE0_STEP4
  sse_step_cfg(&adc1, 0, 3, chncfg | ADC_SSCTL_END);
#              else
  sse_step_cfg(&adc1, 0, 3, chncfg);
#              endif
#            ifdef CONFIG_TIVA_ADC1_SSE0_STEP4
#              ifdef CONFIG_TIVA_ADC1_SSE0_STEP4_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#              else
  chncfg = ADC_SSCTL_IE;
#              endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP4_AIN));
  sse_register_chn(&adc1, 0, 4, CONFIG_TIVA_ADC1_SSE0_STEP4_AIN);
  sse_differential(&adc1, 0, 4, 0);
#                if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 4, ADC_SSTH_SHOLD_16);
#                endif
#                ifndef CONFIG_TIVA_ADC1_SSE0_STEP5
  sse_step_cfg(&adc1, 0, 4, chncfg | ADC_SSCTL_END);
#                else
  sse_step_cfg(&adc1, 0, 4, chncfg);
#                endif
#              ifdef CONFIG_TIVA_ADC1_SSE0_STEP5
#                ifdef CONFIG_TIVA_ADC1_SSE0_STEP5_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#                else
  chncfg = ADC_SSCTL_IE;
#                endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP5_AIN));
  sse_register_chn(&adc1, 0, 5, CONFIG_TIVA_ADC1_SSE0_STEP5_AIN);
  sse_differential(&adc1, 0, 5, 0);
#                  if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 5, ADC_SSTH_SHOLD_16);
#                  endif
#                  ifndef CONFIG_TIVA_ADC1_SSE0_STEP6
  sse_step_cfg(&adc1, 0, 5, chncfg | ADC_SSCTL_END);
#                  else
  sse_step_cfg(&adc1, 0, 5, chncfg);
#                  endif
#                ifdef CONFIG_TIVA_ADC1_SSE0_STEP6
#                  ifdef CONFIG_TIVA_ADC1_SSE0_STEP6_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#                  else
  chncfg = ADC_SSCTL_IE;
#                  endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP6_AIN));
  sse_register_chn(&adc1, 0, 6, CONFIG_TIVA_ADC1_SSE0_STEP6_AIN);
  sse_differential(&adc1, 0, 6, 0);
#                    if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 6, ADC_SSTH_SHOLD_16);
#                    endif
#                    ifndef CONFIG_TIVA_ADC1_SSE0_STEP7
  sse_step_cfg(&adc1, 0, 6, chncfg | ADC_SSCTL_END);
#                    else
  sse_step_cfg(&adc1, 0, 6, chncfg);
#                    endif
#                  ifdef CONFIG_TIVA_ADC1_SSE0_STEP7
#                    ifdef CONFIG_TIVA_ADC1_SSE0_STEP7_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#                    else
  chncfg = ADC_SSCTL_IE;
#                    endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE0_STEP7_AIN));
  sse_register_chn(&adc1, 0, 7, CONFIG_TIVA_ADC1_SSE0_STEP7_AIN);
  sse_differential(&adc1, 0, 7, 0);
#                      if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE0_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 0, 7, ADC_SSTH_SHOLD_16);
#                      endif
  sse_step_cfg(&adc1, 0, 7, chncfg | ADC_SSCTL_END);
#                  endif /* CONFIG_TIVA_ADC1_SSE0_STEP7 */
#                endif /* CONFIG_TIVA_ADC1_SSE0_STEP6 */
#              endif /* CONFIG_TIVA_ADC1_SSE0_STEP5 */
#            endif /* CONFIG_TIVA_ADC1_SSE0_STEP4 */
#          endif /* CONFIG_TIVA_ADC1_SSE0_STEP3 */
#        endif /* CONFIG_TIVA_ADC1_SSE0_STEP2 */
#      endif /* CONFIG_TIVA_ADC1_SSE0_STEP1 */
#    endif /* CONFIG_TIVA_ADC1_SSE0_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC1_SSE0 */

#  ifdef CONFIG_TIVA_ADC1_SSE1
static void adc1_sse1_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC1_SSE1_STEP0
#      ifdef CONFIG_TIVA_ADC1_SSE1_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE1_STEP0_AIN));
  sse_register_chn(&adc1, 1, 0, CONFIG_TIVA_ADC1_SSE1_STEP0_AIN);
  sse_differential(&adc1, 1, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 1, 0, ADC_SSTH_SHOLD_16);
#        endif
#        ifndef CONFIG_TIVA_ADC1_SSE1_STEP1
  sse_step_cfg(&adc1, 1, 0, chncfg | ADC_SSCTL_END);
#        else
  sse_step_cfg(&adc1, 1, 0, chncfg);
#        endif
#      ifdef CONFIG_TIVA_ADC1_SSE1_STEP1
#        ifdef CONFIG_TIVA_ADC1_SSE1_STEP1_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#        else
  chncfg = ADC_SSCTL_IE;
#        endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE1_STEP1_AIN));
  sse_register_chn(&adc1, 1, 1, CONFIG_TIVA_ADC1_SSE1_STEP1_AIN);
  sse_differential(&adc1, 1, 1, 0);
#          if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 1, 1, ADC_SSTH_SHOLD_16);
#          endif
#          ifndef CONFIG_TIVA_ADC1_SSE1_STEP2
  sse_step_cfg(&adc1, 1, 1, chncfg | ADC_SSCTL_END);
#          else
  sse_step_cfg(&adc1, 1, 1, chncfg);
#          endif
#        ifdef CONFIG_TIVA_ADC1_SSE1_STEP2
#          ifdef CONFIG_TIVA_ADC1_SSE1_STEP2_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#          else
  chncfg = ADC_SSCTL_IE;
#          endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE1_STEP2_AIN));
  sse_register_chn(&adc1, 1, 2, CONFIG_TIVA_ADC1_SSE1_STEP2_AIN);
  sse_differential(&adc1, 1, 2, 0);
#            if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 1, 2, ADC_SSTH_SHOLD_16);
#            endif
#            ifndef CONFIG_TIVA_ADC1_SSE1_STEP3
  sse_step_cfg(&adc1, 1, 2, chncfg | ADC_SSCTL_END);
#            else
  sse_step_cfg(&adc1, 1, 2, chncfg);
#            endif
#          ifdef CONFIG_TIVA_ADC1_SSE1_STEP3
#            ifdef CONFIG_TIVA_ADC1_SSE1_STEP3_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#            else
  chncfg = ADC_SSCTL_IE;
#            endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE1_STEP3_AIN));
  sse_register_chn(&adc1, 1, 3, CONFIG_TIVA_ADC1_SSE1_STEP3_AIN);
  sse_differential(&adc1, 1, 3, 0);
#              if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE1_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 1, 3, ADC_SSTH_SHOLD_16);
#              endif
  sse_step_cfg(&adc1, 1, 3, chncfg | ADC_SSCTL_END);
#          endif /* CONFIG_TIVA_ADC1_SSE1_STEP3 */
#        endif /* CONFIG_TIVA_ADC1_SSE1_STEP2 */
#      endif /* CONFIG_TIVA_ADC1_SSE1_STEP1 */
#    endif /* CONFIG_TIVA_ADC1_SSE1_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC1_SSE1 */

#  ifdef CONFIG_TIVA_ADC1_SSE2
static void adc1_sse2_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC1_SSE2_STEP0
#      ifdef CONFIG_TIVA_ADC1_SSE2_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE2_STEP0_AIN));
  sse_register_chn(&adc1, 2, 0, CONFIG_TIVA_ADC1_SSE2_STEP0_AIN);
  sse_differential(&adc1, 2, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 2, 0, ADC_SSTH_SHOLD_16);
#        endif
#        ifndef CONFIG_TIVA_ADC1_SSE2_STEP1
  sse_step_cfg(&adc1, 2, 0, chncfg | ADC_SSCTL_END);
#        else
  sse_step_cfg(&adc1, 2, 0, chncfg);
#        endif
#      ifdef CONFIG_TIVA_ADC1_SSE2_STEP1
#        ifdef CONFIG_TIVA_ADC1_SSE2_STEP1_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#        else
  chncfg = ADC_SSCTL_IE;
#        endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE2_STEP1_AIN));
  sse_register_chn(&adc1, 2, 1, CONFIG_TIVA_ADC1_SSE2_STEP1_AIN);
  sse_differential(&adc1, 2, 1, 0);
#          if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 2, 1, ADC_SSTH_SHOLD_16);
#          endif
#          ifndef CONFIG_TIVA_ADC1_SSE2_STEP2
  sse_step_cfg(&adc1, 2, 1, chncfg | ADC_SSCTL_END);
#          else
  sse_step_cfg(&adc1, 2, 1, chncfg);
#          endif
#        ifdef CONFIG_TIVA_ADC1_SSE2_STEP2
#          ifdef CONFIG_TIVA_ADC1_SSE2_STEP2_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#          else
  chncfg = ADC_SSCTL_IE;
#          endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE2_STEP2_AIN));
  sse_register_chn(&adc1, 2, 2, CONFIG_TIVA_ADC1_SSE2_STEP2_AIN);
  sse_differential(&adc1, 2, 2, 0);
#            if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 2, 2, ADC_SSTH_SHOLD_16);
#            endif
#            ifndef CONFIG_TIVA_ADC1_SSE2_STEP3
  sse_step_cfg(&adc1, 2, 2, chncfg | ADC_SSCTL_END);
#            else
  sse_step_cfg(&adc1, 2, 2, chncfg);
#            endif
#          ifdef CONFIG_TIVA_ADC1_SSE2_STEP3
#            ifdef CONFIG_TIVA_ADC1_SSE2_STEP3_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#            else
  chncfg = ADC_SSCTL_IE;
#            endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE2_STEP3_AIN));
  sse_register_chn(&adc1, 2, 3, CONFIG_TIVA_ADC1_SSE2_STEP3_AIN);
  sse_differential(&adc1, 2, 3, 0);
#              if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE2_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 2, 3, ADC_SSTH_SHOLD_16);
#              endif
  sse_step_cfg(&adc1, 2, 3, chncfg | ADC_SSCTL_END);
#          endif /* CONFIG_TIVA_ADC1_SSE2_STEP3 */
#        endif /* CONFIG_TIVA_ADC1_SSE2_STEP2 */
#      endif /* CONFIG_TIVA_ADC1_SSE2_STEP1 */
#    endif /* CONFIG_TIVA_ADC1_SSE2_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC1_SSE2 */

#  ifdef CONFIG_TIVA_ADC1_SSE3
static void adc1_sse3_chn_cfg(void)
{
  uint32_t chncfg = 0;
#    ifdef CONFIG_TIVA_ADC1_SSE3_STEP0
#      ifdef CONFIG_TIVA_ADC1_SSE3_STEP0_TS
  chncfg = ADC_SSCTL_IE | ADC_SSCTL_TS;
#      else
  chncfg = ADC_SSCTL_IE;
#      endif
  tiva_configgpio(TIVA_ADC_PIN(CONFIG_TIVA_ADC1_SSE3_STEP0_AIN));
  sse_register_chn(&adc1, 3, 0, CONFIG_TIVA_ADC1_SSE3_STEP0_AIN);
  sse_differential(&adc1, 3, 0, 0);
#        if defined (CONFIG_ARCH_CHIP_TM4C129) && (CONFIG_TIVA_ADC1_SSE3_TRIGGER == ADC_EMUX_PROC)
  sse_sample_hold_time(&adc1, 3, 0, ADC_SSTH_SHOLD_16);
#        endif
  sse_step_cfg(&adc1, 3, 0, chncfg | ADC_SSCTL_END);
#    endif /* CONFIG_TIVA_ADC1_SSE3_STEP0 */
}
#  endif /* CONFIG_TIVA_ADC1_SSE3 */
#endif /* CONFIG_TIVA_ADC1 */

#endif /* CONFIG_TIVA_ADC0 | CONFIG_TIVA_ADC1 */
