/****************************************************************************
 * configs/stm32f334-disco/src/stm32_smps.c
 *
 *   Copyright (C) 2017, 2018 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <dsp.h>

#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include "up_internal.h"
#include "ram_vectors.h"

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/power/smps.h>

#include "stm32_hrtim.h"
#include "stm32_adc.h"

#if defined(CONFIG_EXAMPLES_SMPS) && defined(CONFIG_DRIVERS_SMPS)

#ifndef CONFIG_LIBDSP
#  error CONFIG_LIBDSP is required
#endif

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error CONFIG_ARCH_HIPRI_INTERRUPT is required
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error CONFIG_ARCH_RAMVECTORS is required
#endif

#ifndef CONFIG_ARCH_IRQPRIO
#  error CONFIG_ARCH_IRQPRIO is required
#endif

#ifndef CONFIG_ARCH_FPU
#  warning Set CONFIG_ARCH_FPU for hardware FPU support
#endif

#if !defined(CONFIG_STM32_HRTIM1) || !defined(CONFIG_STM32_HRTIM)
#  error "SMPS example requires HRTIM1 support"
#endif

#if !defined(CONFIG_STM32_ADC1) || !defined(CONFIG_ADC)
#  error "SMPS example requires ADC1 support"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC1 channels used in this example */

#define ADC1_NCHANNELS 2

/* ADC1 injected channels numeration */

#define V_IN_ADC_INJ_CHANNEL  0
#define V_OUT_ADC_INJ_CHANNEL 1

/* Voltage reference for ADC */

#define ADC_REF_VOLTAGE ((float)3.3)

/* ADC resolution */

#define ADC_VAL_MAX     4095

/* Input voltage convertion ratio - 6.8k/(6.8k + 27k) */

#define V_IN_RATIO (float)((float)(6800+27000)/(float)6800)

/* Output voltage convertion ratio - 3.3k/(3.3k + 13.3k) */

#define V_OUT_RATIO (float)((float)(3300+13300)/(float)3300)

/* Some absolute limits */

#define SMPS_ABSOLUTE_OUT_CURRENT_LIMIT_mA 250
#define SMPS_ABSOLUTE_OUT_VOLTAGE_LIMIT_mV 15000
#define SMPS_ABSOLUTE_IN_VOLTAGE_LIMIT_mV  15000

#if CONFIG_EXAMPLES_SMPS_OUT_CURRENT_LIMIT > SMPS_ABSOLUTE_OUT_CURRENT_LIMIT_mA
#  error "Output current limit great than absolute limit!"
#endif
#if CONFIG_EXAMPLES_SMPS_OUT_VOLTAGE_LIMIT > SMPS_ABSOLUTE_OUT_VOLTAGE_LIMIT_mV
#  error "Output voltage limit greater than absolute limit!"
#endif
#if CONFIG_EXAMPLES_SMPS_IN_VOLTAGE_LIMIT > SMPS_ABSOLUTE_IN_VOLTAGE_LIMIT_mV
#  error "Input voltage limit greater than absolute limit!"
#endif

/* Maximum output voltage for boost conveter in float */

#define BOOST_VOLT_MAX ((float)CONFIG_EXAMPLES_SMPS_OUT_VOLTAGE_LIMIT/1000.0)

/* Current limit table dimmension */

#define SMPS_CURRENT_LIMIT_TAB_DIM 15

/* At this time only PID controller implemented */

#define SMPS_CONTROLLER_PID   1

/* Converter's finite accuracy */

#define SMPS_VOLTAGE_ACCURACY ((float)0.01)

/* Buck-boost mode threshold */

#define SMPS_BUCKBOOST_RANGE  ((float)0.5)

/* PID controller configuration */

#define PID_KP                ((float)1.0)
#define PID_KI                ((float)0.1)
#define PID_KD                ((float)0.0)

/* Converter frequncies:
 *   - TIMA_PWM_FREQ - buck converter 250kHz
 *   - TIMB_PWM_FREQ - boost converter 250kHz
 */

#define TIMA_PWM_FREQ 250000
#define TIMB_PWM_FREQ 250000

/* Deadtime configuration */

#define DT_RISING 0x0A0
#define DT_FALLING 0x0A0

/* Helper macros */

#define HRTIM_ALL_OUTPUTS_ENABLE(hrtim, state)                        \
  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMA_CH1|HRTIM_OUT_TIMA_CH2|  \
                       HRTIM_OUT_TIMB_CH1|HRTIM_OUT_TIMB_CH2, state);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Current converter mode */

enum converter_mode_e
{
  CONVERTER_MODE_INIT,      /* Initial mode */
  CONVERTER_MODE_BUCK,      /* Buck mode operations  (V_in > V_out) */
  CONVERTER_MODE_BOOST,     /* Boost mode operations (V_in < V_out) */
  CONVERTER_MODE_BUCKBOOST, /* Buck-boost operations (V_in near V_out)*/
};

/* SMPS lower drivers structure */

struct smps_lower_dev_s
{
  FAR struct hrtim_dev_s     *hrtim; /* PWM generation */
  FAR struct adc_dev_s       *adc;   /* input and output voltage sense */
  FAR struct comp_dev_s      *comp;  /* not used in this demo - only as reference */
  FAR struct dac_dev_s       *dac;   /* not used in this demo - only as reference */
  FAR struct opamp_dev_s     *opamp; /* not used in this demo - only as reference */
};

/* Private data for smps */

struct smps_priv_s
{
  uint8_t           conv_mode;   /* Converter mode */
  uint16_t          v_in_raw;    /* Voltage input RAW value */
  uint16_t          v_out_raw;   /* Voltage output RAW value */
  float             v_in;        /* Voltage input real value in V */
  float             v_out;       /* Voltage output real value in V  */
  bool              running;     /* Running flag */
  pid_controller_t  pid;         /* PID controller */
  float            *c_limit_tab; /* Current limit tab */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int smps_setup(FAR struct smps_dev_s *dev);
static int smps_shutdown(FAR struct smps_dev_s *dev);
static int smps_start(FAR struct smps_dev_s *dev);
static int smps_stop(FAR struct smps_dev_s *dev);
static int smps_params_set(FAR struct smps_dev_s *dev,
                           FAR struct smps_params_s *param);
static int smps_mode_set(FAR struct smps_dev_s *dev, uint8_t mode);
static int smps_limits_set(FAR struct smps_dev_s *dev,
                           FAR struct smps_limits_s *limits);
static int smps_state_get(FAR struct smps_dev_s *dev,
                          FAR struct smps_state_s *state);
static int smps_fault_set(FAR struct smps_dev_s *dev, uint8_t fault);
static int smps_fault_get(FAR struct smps_dev_s *dev,
                              FAR uint8_t *fault);
static int smps_fault_clean(FAR struct smps_dev_s *dev,
                                uint8_t fault);
static int smps_ioctl(FAR struct smps_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct smps_lower_dev_s g_smps_lower;
struct smps_priv_s g_smps_priv;
struct smps_s g_smps;

struct smps_ops_s g_smps_ops =
{
  .setup       = smps_setup,
  .shutdown    = smps_shutdown,
  .start       = smps_start,
  .stop        = smps_stop,
  .params_set  = smps_params_set,
  .mode_set    = smps_mode_set,
  .limits_set  = smps_limits_set,
  .fault_set   = smps_fault_set,
  .state_get   = smps_state_get,
  .fault_get   = smps_fault_get,
  .fault_clean = smps_fault_clean,
  .ioctl       = smps_ioctl
};

struct smps_dev_s g_smps_dev =
{
  .ops   = &g_smps_ops,
  .priv  = &g_smps,
  .lower = NULL
};

/* ADC configuration:
 *    - Input voltage (V_IN) - ADC1 Channel 2 (PA1)
 *    - Output voltage (V_OUT) - ADC1 Channel 4 (PA3)
 *
 * ADC channels configured in injected mode.
 *
 * Transistors configuration in buck mode:
 *    - T5 -  ON
 *    - T12 - OFF
 *    - T4 and T11 - buck operation
 * Transistors configuration in boost mode:
 *    - T4 - ON
 *    - T11 - OFF
 *    - T5 and T12 - boost operation
 * Transistors configuration in buck-boost mode:
 *    - T4, T11 - buck operation
 *    - T5 and T12 - boost operation
 *
 * HRTIM outputs configuration:
 *    - T4   -> PA8  -> HRTIM_CHA1
 *    - T5   -> PA11 -> HRTIM_CHB2
 *    - T11  -> PA9  -> HRTIM_CHA2
 *    - T12  -> PA10 -> HRTIM_CHB1
 *
 */

/* ADC channel list */

static const uint8_t g_adc1chan[ADC1_NCHANNELS] =
{
  2,
  4
};

/* Configurations of pins used by ADC channel */

static const uint32_t g_adc1pins[ADC1_NCHANNELS] =
{
  GPIO_ADC1_IN2,                /* PA1 - V_IN */
  GPIO_ADC1_IN4,                /* PA3 - V_OUT */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int smps_shutdown(FAR struct smps_dev_s *dev)
{
  FAR struct smps_s      *smps = (FAR struct smps_s *)dev->priv;
  FAR struct smps_priv_s *priv = (struct smps_priv_s *)smps->priv;

  /* Stop smps if running */

  if (priv->running == true)
    {
      smps_stop(dev);
    }

  /* Reset smps structure */

  memset(smps, 0, sizeof(struct smps_s));

  return OK;
}

/****************************************************************************
 * Name: smps_setup
 *
 * Description:
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int smps_setup(FAR struct smps_dev_s *dev)
{
  FAR struct smps_lower_dev_s *lower = dev->lower;
  FAR struct smps_s           *smps  = (FAR struct smps_s *)dev->priv;
  FAR struct hrtim_dev_s      *hrtim = NULL;
  FAR struct adc_dev_s        *adc   = NULL;
  FAR struct smps_priv_s      *priv;

  /* Initialize smps structure */

  smps->opmode       = SMPS_OPMODE_INIT;
  smps->state.state  = SMPS_STATE_INIT;
  smps->priv         = &g_smps_priv;

  /* Check lower half drivers */

  hrtim = lower->hrtim;
  if (hrtim == NULL)
    {
      pwrerr("ERROR:  Failed to get hrtim ");
    }

  adc = lower->adc;
  if (adc == NULL)
    {
      pwrerr("ERROR:  Failed to get ADC lower level interface");
    }

  /* TODO: create current limit table */

  UNUSED(priv);
  return OK;
}

static int smps_start(FAR struct smps_dev_s *dev)
{
  FAR struct smps_lower_dev_s *lower = dev->lower;
  FAR struct stm32_adc_dev_s  *stm32_adc =
    (FAR struct stm32_adc_dev_s *) lower->adc->ad_priv;
  FAR struct smps_s       *smps   = (FAR struct smps_s *)dev->priv;
  FAR struct smps_priv_s  *priv  = (struct smps_priv_s *)smps->priv;
  FAR struct hrtim_dev_s  *hrtim  = lower->hrtim;
  volatile uint64_t per = 0;
  uint64_t fclk = 0;
  int ret = OK;

  /* Disable HRTIM outputs */

  HRTIM_ALL_OUTPUTS_ENABLE(hrtim, false);

  /* Reset SMPS private structure */

  memset(priv, 0, sizeof(struct smps_priv_s));

#ifdef SMPS_CONTROLLER_PID
  /* Initialize PID controller */

  pid_controller_init(&priv->pid, PID_KP, PID_KI, PID_KD);

  /* Set PID controller saturation */

  pid_saturation_set(&priv->pid, 0.0, BOOST_VOLT_MAX);
#endif

  /* Get TIMA period value for given frequency */

  fclk = HRTIM_FCLK_GET(hrtim, HRTIM_TIMER_TIMA);
  per = fclk/TIMA_PWM_FREQ;
  if (per > HRTIM_PER_MAX)
    {
      pwrerr("ERROR:  Can not achieve tima pwm freq=%u if fclk=%llu\n",
             (uint32_t)TIMA_PWM_FREQ, (uint64_t)fclk);
      ret = -EINVAL;
      goto errout;
    }

  /* Set TIMA period value */

  HRTIM_PER_SET(hrtim, HRTIM_TIMER_TIMA, (uint16_t)per);

  /* Get TIMB period value for given frequency */

  fclk = HRTIM_FCLK_GET(hrtim, HRTIM_TIMER_TIMB);
  per = fclk/TIMB_PWM_FREQ;
  if (per > HRTIM_PER_MAX)
    {
      pwrerr("ERROR:  Can not achieve timb pwm freq=%u if fclk=%llu\n",
             (uint32_t)TIMB_PWM_FREQ, (uint64_t)fclk);
      ret = -EINVAL;
      goto errout;
    }

  /* Set TIMB period value */

  HRTIM_PER_SET(hrtim, HRTIM_TIMER_TIMB, (uint16_t)per);

  /* ADC trigger on TIMA CMP4 */

  HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMA, HRTIM_CMP4, 10000);

  /* Configure TIMER A and TIMER B deadtime mode
   *
   * NOTE: In deadtime mode we have to configure output 1 only (SETx1, RSTx1),
   * output 2 configuration is not significant.
   */

  HRTIM_DEADTIME_UPDATE(hrtim, HRTIM_TIMER_TIMA, HRTIM_DT_EDGE_RISING, DT_RISING);
  HRTIM_DEADTIME_UPDATE(hrtim, HRTIM_TIMER_TIMA, HRTIM_DT_EDGE_FALLING, DT_FALLING);
  HRTIM_DEADTIME_UPDATE(hrtim, HRTIM_TIMER_TIMB, HRTIM_DT_EDGE_RISING, DT_RISING);
  HRTIM_DEADTIME_UPDATE(hrtim, HRTIM_TIMER_TIMB, HRTIM_DT_EDGE_FALLING, DT_FALLING);

  /* Set T4 and T12 to a low state.
   * Deadtime mode force T11 and T5 to a high state.
   */

  HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_SET_NONE);
  HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_RST_PER);

  HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_SET_NONE);
  HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_RST_PER);

  /* Set running flag */

  priv->running = true;

  HRTIM_ALL_OUTPUTS_ENABLE(hrtim, true);

  /* Enable ADC interrupts */

  stm32_adc->ops->int_en(stm32_adc, ADC_INT_JEOS);

errout:
  return ret;
}

static int smps_stop(FAR struct smps_dev_s *dev)
{
  FAR struct smps_lower_dev_s *lower = dev->lower;
  FAR struct smps_s      *smps  = (FAR struct smps_s *)dev->priv;
  FAR struct smps_priv_s *priv  = (struct smps_priv_s *)smps->priv;
  FAR struct hrtim_dev_s *hrtim = lower->hrtim;
  FAR struct stm32_adc_dev_s  *stm32_adc  =
    (FAR struct stm32_adc_dev_s *) lower->adc->ad_priv;

  /* Disable HRTIM outputs */

  HRTIM_ALL_OUTPUTS_ENABLE(hrtim, false);

  /* Disable ADC interrupts */

  stm32_adc->ops->int_dis(stm32_adc, ADC_INT_JEOS);

  /* Reset running flag */

  priv->running = false;

  return OK;
}

static int smps_params_set(FAR struct smps_dev_s *dev,
                           FAR struct smps_params_s *param)
{
  FAR struct smps_s *smps = (FAR struct smps_s *)dev->priv;
  int ret = OK;

  /* Only output voltage */

  smps->param.v_out = param->v_out;

  /* REVISIT: use current and power parameters ? */

  if (param->i_out > 0)
    {
      pwrwarn("WARNING: Output current parameters not used in this demo\n");
    }

  if (param->p_out > 0)
    {
      pwrwarn("WARNING: Output power parameters not used in this demo\n");
    }

  return ret;
}

static int smps_mode_set(FAR struct smps_dev_s *dev, uint8_t mode)
{
  FAR struct smps_s *smps = (FAR struct smps_s *)dev->priv;
  int ret = OK;

  /* Only constant voltage mode supported */

  if (mode == SMPS_OPMODE_CV)
    {
      smps->opmode = mode;
    }
  else
    {
      pwrerr("ERROR:  Unsupported SMPS mode %d!\n", mode);
      ret = ERROR;
      goto errout;
    }

errout:
  return ret;
}

static int smps_limits_set(FAR struct smps_dev_s *dev,
                           FAR struct smps_limits_s *limits)
{
  FAR struct smps_s *smps = (FAR struct smps_s *)dev->priv;
  int ret = OK;

  /* Some assertions */

  if (limits->v_out <= 0)
    {
      pwrerr("ERROR:  Output voltage limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->v_in <= 0)
    {
      pwrerr("ERROR:  Input voltage limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->i_out <= 0)
    {
      pwrerr("ERROR:  Output current limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->v_out * 1000 > CONFIG_EXAMPLES_SMPS_OUT_VOLTAGE_LIMIT)
    {
      limits->v_out = (float)CONFIG_EXAMPLES_SMPS_OUT_VOLTAGE_LIMIT/1000.0;
      pwrwarn("WARNING: "
              "SMPS output voltage limiit > SMPS absoulute output voltage limit."
              " Set output voltage limit to %.2f.\n",
              limits->v_out);
    }

  if (limits->v_in * 1000 > CONFIG_EXAMPLES_SMPS_IN_VOLTAGE_LIMIT)
    {
      limits->v_in = (float)CONFIG_EXAMPLES_SMPS_IN_VOLTAGE_LIMIT/1000.0;
      pwrwarn("WARNING: "
              "SMPS input voltage limiit > SMPS absoulute input voltage limit."
              " Set input voltage limit to %.2f.\n",
              limits->v_in);
    }

  if (limits->i_out * 1000 > CONFIG_EXAMPLES_SMPS_OUT_CURRENT_LIMIT)
    {
      limits->i_out = (float)CONFIG_EXAMPLES_SMPS_OUT_CURRENT_LIMIT/1000.0;
      pwrwarn("WARNING: "
              "SMPS output current limiit > SMPS absoulute output current limit."
              " Set output current limit to %.2f.\n",
              limits->i_out);
    }

  /* Set output voltage limit */

  smps->limits.v_out = limits->v_out;

  /* Set input voltage limit */

  smps->limits.v_in = limits->v_in;

  /* Set current limit */

  smps->limits.i_out = limits->i_out;

  /* Lock limits */

  smps->limits.lock = true;

errout:
  return ret;
}

static int smps_state_get(FAR struct smps_dev_s *dev,
                          FAR struct smps_state_s *state)
{
  FAR struct smps_s *smps = (FAR struct smps_s *)dev->priv;

  /* Copy localy stored feedbacks data to status structure */

  smps->state.fb.v_in  = g_smps_priv.v_in;
  smps->state.fb.v_out = g_smps_priv.v_out;

  /* Return state structure to caller */

  memcpy(state, &smps->state, sizeof(struct smps_state_s));

  return OK;
}

static int smps_fault_set(FAR struct smps_dev_s *dev, uint8_t fault)
{
  return OK;
}

static int smps_fault_get(FAR struct smps_dev_s *dev, FAR uint8_t *fault)
{
  return OK;
}

static int smps_fault_clean(FAR struct smps_dev_s *dev, uint8_t fault)
{
  return OK;
}

static int smps_ioctl(FAR struct smps_dev_s *dev, int cmd, unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Name: smps_controller
 ****************************************************************************/

static float smps_controller(FAR struct smps_priv_s *priv, float err)
{
  float out = 0.0;

#ifdef SMPS_CONTROLLER_PID
  out = pid_controller(&priv->pid, err);
#else
#  error "At this time only PID controller implemented"
#endif

  return out;
}

/****************************************************************************
 * Name: smps_duty_set
 ****************************************************************************/

static void smps_duty_set(struct smps_priv_s *priv, struct smps_lower_dev_s *lower,
                          float out)
{
  FAR struct hrtim_dev_s *hrtim = lower->hrtim;
  uint8_t mode = priv->conv_mode;
  uint16_t cmp = 0;
  float duty = 0.0;
  uint16_t per = 0;

  switch (mode)
    {
      case CONVERTER_MODE_INIT:
        {
          /* Do nothing */

          break;
        }

      case CONVERTER_MODE_BUCK:
        {
          if (out >= priv->v_in) out = priv->v_in;
          if (out < 0.0) out = 0.0;

          duty = out/priv->v_in;

#warning TODO: current limit in buck mode

          per = HRTIM_PER_GET(hrtim, HRTIM_TIMER_TIMA);

          cmp = (uint16_t)(per * duty);

          if (cmp > per-30) cmp = per - 30;

          /* Set T4 duty cycle. T11 is complementary to T4 */

          HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMA, HRTIM_CMP1, cmp);

          break;
        }

      case CONVERTER_MODE_BOOST:
        {
          per = HRTIM_PER_GET(hrtim, HRTIM_TIMER_TIMA);

          if (out < priv->v_in) out = priv->v_in;
          if (out >= BOOST_VOLT_MAX) out = BOOST_VOLT_MAX;

          duty = 1.0 - priv->v_in/out;

#warning TODO: current limit in boost mode

          cmp = (uint16_t)(per * duty);

          /* Set T12 duty cycle. T5 is complementary to T12 */

          HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMB, HRTIM_CMP1, cmp);

          break;
        }

      case CONVERTER_MODE_BUCKBOOST:
        {
          /* Buck converter is set to fixed duty cycle (80%).
           * Now we need set boost converter
           */

          per = HRTIM_PER_GET(hrtim, HRTIM_TIMER_TIMA);

          if (out < priv->v_in) out = priv->v_in;
          if (out >= BOOST_VOLT_MAX) out = BOOST_VOLT_MAX;

          duty = 1.0 - priv->v_in/out;

#warning TODO: current limit in buck boost mode

          cmp = (uint16_t)(per * duty);

          /* Set T12 duty cycle. T5 is complementary to T12 */

          HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMB, HRTIM_CMP1, cmp);

          break;
        }

      default:
        {
          pwrerr("ERROR:  Unknown converter mode %d!\n", mode);
          break;
        }
    }
}

/****************************************************************************
 * Name: smps_conv_mode_set
 *
 * Description:
 *   Change converter mode (buck/boost/buck-boost).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void smps_conv_mode_set(struct smps_priv_s *priv, struct smps_lower_dev_s *lower,
                               uint8_t mode)
{
  FAR struct hrtim_dev_s *hrtim = lower->hrtim;

  /* Disable all outputs */

  HRTIM_ALL_OUTPUTS_ENABLE(hrtim, false);

  switch (mode)
    {
      case CONVERTER_MODE_INIT:
        {

          break;
        }

      case CONVERTER_MODE_BUCK:
        {
          /* Set T12 low (T5 high) on the next PER */

          HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_SET_NONE);
          HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_RST_PER);


          /* Set T4 to a high state on PER and reset on CMP1.
             T11 is complementary to T4. */

          HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_SET_PER);
          HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_RST_CMP1);

          break;
        }

      case CONVERTER_MODE_BOOST:
        {
          /* Set T4 high (T11 low) on the next PER */

          HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_SET_PER);
          HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_RST_NONE);

          /* Set T12 to a high state on PER and reset on CMP1.
             T5 is complementary to T12. */

          HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_SET_PER);
          HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_RST_CMP1);

          break;
        }

      case CONVERTER_MODE_BUCKBOOST:
        {
          /* Set T4 to a high state on PER and reset on CMP1.
             T11 is complementary to T4. */

          HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_SET_PER);
          HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMA_CH1, HRTIM_OUT_RST_CMP1);

          /* Set T12 to a high state on PER and reset on CMP1.
             T5 is complementary to T12. */

          HRTIM_OUTPUT_SET_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_SET_PER);
          HRTIM_OUTPUT_RST_SET(hrtim, HRTIM_OUT_TIMB_CH1, HRTIM_OUT_RST_CMP1);

          /* Set fixed duty cycle (80%) on buck converter (T4 and T11) */

          HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMA, HRTIM_CMP1,
                        0.8 * ((uint16_t)HRTIM_PER_GET(hrtim, HRTIM_TIMER_TIMA)));


          break;
        }

      default:
        {
          pwrerr("ERROR:  Unknown converter mode %d!\n", mode);
          break;
        }
    }

  /* Set mode in private data */

  priv->conv_mode = mode;

  /* Enable outputs */

  HRTIM_ALL_OUTPUTS_ENABLE(hrtim, true);

}

/****************************************************************************
 * Name: adc12_handler
 ****************************************************************************/

static void adc12_handler(void)
{
  FAR struct smps_dev_s  *dev = &g_smps_dev;
  FAR struct smps_s      *smps = (FAR struct smps_s *)dev->priv;
  FAR struct smps_priv_s *priv  = (struct smps_priv_s *)smps->priv;
  FAR struct smps_lower_dev_s *lower = dev->lower;
  FAR struct stm32_adc_dev_s  *adc   =
    (FAR struct stm32_adc_dev_s*)lower->adc->ad_priv;
  uint32_t pending;
  float ref = ADC_REF_VOLTAGE;
  float bit = ADC_VAL_MAX;
  float err;
  float out;
  uint8_t mode;

  pending = adc->ops->int_get(adc);

  if (pending & ADC_INT_JEOC && priv->running == true)
    {
      /* Get raw ADC values */

      priv->v_out_raw = adc->ops->inj_get(adc, V_OUT_ADC_INJ_CHANNEL);
      priv->v_in_raw  = adc->ops->inj_get(adc, V_IN_ADC_INJ_CHANNEL);

      /* Convert raw values to real values */

      priv->v_out = (priv->v_out_raw * ref / bit) * V_OUT_RATIO;
      priv->v_in  = (priv->v_in_raw * ref / bit) * V_IN_RATIO;

      /* According to measured voltages we set converter in appropriate mode */

      if (smps->param.v_out > (priv->v_in+SMPS_BUCKBOOST_RANGE))
        {
          /* Desired output voltage greather than input voltage - set boost converter */

          mode = CONVERTER_MODE_BOOST;
        }

      else if (smps->param.v_out < (priv->v_in-SMPS_BUCKBOOST_RANGE))
        {
          /* Desired output voltage lower than input voltage - set buck converter */

          mode = CONVERTER_MODE_BUCK;
        }

      else
        {
          /* Desired output voltage close to input voltage - set buck-boost converter */

          mode = CONVERTER_MODE_BUCKBOOST;
        }

      /* Configure converter to the new mode if needed */

      if (priv->conv_mode != mode)
        {
          smps_conv_mode_set(priv, lower, mode);
        }

      /* Get regualtor error */

      err = smps->param.v_out - priv->v_out;

      if (err >= SMPS_VOLTAGE_ACCURACY || err <= (-SMPS_VOLTAGE_ACCURACY))
        {
          /* PID controller */

          out = smps_controller(priv, err);

          /* Update duty cycle */

          smps_duty_set(priv, lower, out);
        }
    }

  /* Clear pending */

  adc->ops->int_ack(adc, pending);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_smps_setup
 *
 * Description:
 *   Initialize SMPS driver.
 *
 *   This function should be call by board_app_initialize().
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int stm32_smps_setup(void)
{
  FAR struct smps_lower_dev_s *lower = &g_smps_lower;
  FAR struct smps_dev_s *smps        = &g_smps_dev;
  FAR struct hrtim_dev_s *hrtim      = NULL;
  FAR struct adc_dev_s *adc          = NULL;
  static bool initialized            = false;
  int ret                            = OK;
  int i;

  /* Initialize only once */

  if (!initialized)
    {
      /* Get the HRTIM interface */

      hrtim = stm32_hrtiminitialize();
      if (hrtim == NULL)
        {
          pwrerr("ERROR:  Failed to get HRTIM1 interface\n");
          return -ENODEV;
        }

      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          stm32_configgpio(g_adc1pins[i]);
        }

      /* Get the ADC interface */

      adc = stm32_adcinitialize(1, g_adc1chan, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          pwrerr("ERROR:  Failed to get ADC %d interface\n", 1);
          return -ENODEV;
        }

      /* Initialize SMPS lower driver interfaces */

      lower->hrtim = hrtim;
      lower->adc   = adc;
      lower->comp  = NULL;
      lower->dac   = NULL;
      lower->opamp = NULL;

      /* Attach ADC12 ram vector */

      ret = up_ramvec_attach(STM32_IRQ_ADC12, adc12_handler);
      if (ret < 0)
        {
          pwrerr("ERROR:  up_ramvec_attach failed: %d\n", ret);
          ret = EXIT_FAILURE;
          goto errout;
        }

      /* Set the priority of the ADC12 interrupt vector */

      ret = up_prioritize_irq(STM32_IRQ_ADC12, NVIC_SYSH_HIGH_PRIORITY);
      if (ret < 0)
        {
          pwrerr("ERROR:  up_prioritize_irq failed: %d\n", ret);
          ret = EXIT_FAILURE;
          goto errout;
        }

      up_enable_irq(STM32_IRQ_ADC12);

      /* Setup ADC hardware */

      adc->ad_ops->ao_setup(adc);

      /* We do not need register character drivers for SMPS lower peripherals.
       * All control should be done via SMPS character driver.
       */

      ret = smps_register(CONFIG_EXAMPLES_SMPS_DEVPATH, smps, (void *)lower);
      if (ret < 0)
        {
          pwrerr("ERROR:  smps_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

errout:
  return ret;
}

#endif /* CONFIG_EXAMPLE_SMPS && CONFIG_DRIVERS_SMPS*/
