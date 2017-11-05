/****************************************************************************
 * configs/stm32f334-disco/src/stm32_smps.c
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

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
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

#warning "STM32F334-DISCO buck-boost converter example under development!"

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

#if !defined(CONFIG_STM32_HRTIM1) || !defined(CONFIG_HRTIM)
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

#define VIN_ADC_INJ_CHANNEL  0
#define VOUT_ADC_INJ_CHANNEL 1

/* Voltage reference for ADC */

#define ADC_REF_VOLTAGE ((float)3.3)

/* ADC resolution */

#define ADC_VAL_MAX     4095

/* Input voltage convertion ratio - 6.8k/(6.8k + 27k) */

#define VIN_RATIO (float)((float)(6800+27000)/(float)6800)

/* Output voltage convertion ratio - 3.3k/(3.3k + 13.3k) */

#define VOUT_RATIO (float)((float)(3300+13300)/(float)3300)

/* Some absolute limits */

#define SMPS_ABSOLUTE_OUT_CURRENT_LIMIT_mA 250
#define SMPS_ABSOLUTE_OUT_VOLTAGE_LIMIT_mV 15000
#define SMPS_ABSOLUTE_IN_VOLTAGE_LIMIT_mV  15000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Current converter mode */

enum converter_mode_e
{
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
  uint8_t  conv_mode;            /* Converter mode */
  uint16_t vin_raw;              /* Voltage input RAW value */
  uint16_t vout_raw;             /* Voltage output RAW value */
  float    vin;                  /* Voltage input real value in V */
  float    vout;                 /* Voltage output real value in V  */
  bool     running;              /* Running flag */
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
  .ops = &g_smps_ops,
  .priv = &g_smps
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
 *    - T5 and T15 - boost operation
 * Transistors configuration in buck-boost mode:
 *    - T4, T11 - buck operation
 *    - T5 and T15 - boost operation
 *
 * HRTIM outputs configuration:
 *    - T4   -> PA8  -> HRTIM_CHA1
 *    - T5   -> PA11 -> HRTIM_CHB2
 *    - T11  -> PA9  -> HRTIM_CHA2
 *    - T15  -> PA10 -> HRTIM_CHB1
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
  GPIO_ADC1_IN2,                /* PA1 - VIN */
  GPIO_ADC1_IN4,                /* PA3 - VOUT */
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
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int smps_setup(FAR struct smps_dev_s *dev)
{
  FAR struct smps_lower_dev_s *lower = dev->lower;
  FAR struct smps_s           *smps  = (FAR struct smps_s *)dev->priv;
  FAR struct hrtim_dev_s      *hrtim = NULL;
  FAR struct adc_dev_s        *adc   = NULL;

  /* Initialize smps structure */

  smps->opmode       = SMPS_OPMODE_INIT;
  smps->state.state  = SMPS_STATE_INIT;
  smps->priv         = &g_smps_priv;

  /* Check lower half drivers */

  hrtim = lower->hrtim;
  if (hrtim == NULL)
    {
      printf("ERROR: failed to get hrtim ");
    }

  adc = lower->adc;
  if (adc == NULL)
    {
      printf("ERROR: failed to get ADC lower level interface");
    }

errout:
  return OK;
}

static int smps_start(FAR struct smps_dev_s *dev)
{
  FAR struct smps_lower_dev_s *lower = dev->lower;
  FAR struct stm32_adc_dev_s  *stm32_adc =
    (FAR struct stm32_adc_dev_s *) lower->adc->ad_priv;
  FAR struct smps_s       *smps   = (FAR struct smps_s *)dev->priv;
  FAR struct hrtim_dev_s  *hrtim  = lower->hrtim;

  /* Stop HRTIM PWM */

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMA_CH1, false);
  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMA_CH2, false);

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMB_CH1, false);
  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMB_CH2, false);

  /* 1 period is 4us - 100% time */

  HRTIM_PER_SET(hrtim, HRTIM_TIMER_TIMA, 18432);
  HRTIM_PER_SET(hrtim, HRTIM_TIMER_TIMB, 18432);

  /* ADC trigger on TIMA CMP4 */

  HRTIM_CMP_SET(hrtim, HRTIM_TIMER_TIMA, HRTIM_CMP4, 10000);

  /* Enable ADC interrupts */

  stm32_adc->ops->int_en(stm32_adc, ADC_INT_JEOS);
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

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMA_CH1, false);
  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMA_CH2, false);

  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMB_CH1, false);
  HRTIM_OUTPUTS_ENABLE(hrtim, HRTIM_OUT_TIMB_CH2, false);

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
      printf("WARNING: Output current parameters not used in this demo\n");
    }

  if (param->p_out > 0)
    {
      printf("WARNING: Output power parameters not used in this demo\n");
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
      printf("ERROR: unsupported SMPS mode %d!\n", mode);
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
      printf("Output voltage limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->v_in <= 0)
    {
      printf("Input voltage limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->i_out <= 0)
    {
      printf("Output current limit must be set!\n");
      ret = ERROR;
      goto errout;
    }

  if (limits->v_out * 1000 > SMPS_ABSOLUTE_OUT_VOLTAGE_LIMIT_mV)
    {
      limits->v_out = (float)SMPS_ABSOLUTE_OUT_VOLTAGE_LIMIT_mV/1000.0;
      printf("SMPS output voltage limiit > SMPS absoulute output voltage limit."
             " Set output voltage limit to %d.\n",
             limits->v_out);
    }

  if (limits->v_in * 1000 > SMPS_ABSOLUTE_IN_VOLTAGE_LIMIT_mV)
    {
      limits->v_in = (float)SMPS_ABSOLUTE_IN_VOLTAGE_LIMIT_mV/1000.0;
      printf("SMPS input voltage limiit > SMPS absoulute input voltage limit."
             " Set input voltage limit to %d.\n",
             limits->v_in);
    }

  if (limits->i_out * 1000 > SMPS_ABSOLUTE_OUT_CURRENT_LIMIT_mA)
    {
      limits->i_out = (float)SMPS_ABSOLUTE_OUT_CURRENT_LIMIT_mA/1000.0;
      printf("SMPS output current limiit > SMPS absoulute output current limit."
             " Set output current limit to %d.\n",
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

  smps->state.fb.v_in  = g_smps_priv.vin;
  smps->state.fb.v_out = g_smps_priv.vout;

  /* Return state structure to caller */

  memcpy(state, &smps->state, sizeof(struct smps_state_s));

  return OK;
}

static int smps_fault_set(FAR struct smps_dev_s *dev, uint8_t fault)
{
#warning "missing logic"
  return OK;
}

static int smps_fault_get(FAR struct smps_dev_s *dev, FAR uint8_t *fault)
{
#warning "missing logic"
  return OK;
}

static int smps_fault_clean(FAR struct smps_dev_s *dev, uint8_t fault)
{
#warning "missing logic"
  return OK;
}

static int smps_ioctl(FAR struct smps_dev_s *dev, int cmd, unsigned long arg)
{
#warning "missing logic"
  return OK;
}

static void adc12_handler(void)
{
  FAR struct smps_lower_dev_s *lower = g_smps_dev.lower;
  FAR struct stm32_adc_dev_s  *stm32_adc   =
    (FAR struct stm32_adc_dev_s*)lower->adc->ad_priv;
  float ref = ADC_REF_VOLTAGE;
  float bit = ADC_VAL_MAX;
  uint32_t pending;

  pending = stm32_adc->ops->int_get(stm32_adc);

  if (pending & ADC_INT_JEOC)
    {
      /* Get raw ADC values */

      g_smps_priv.vout_raw = stm32_adc->ops->inj_get(stm32_adc, VOUT_ADC_INJ_CHANNEL);
      g_smps_priv.vin_raw  = stm32_adc->ops->inj_get(stm32_adc, VIN_ADC_INJ_CHANNEL);

      /* Convert raw values to real values */

      g_smps_priv.vout = (g_smps_priv.vout_raw * ref / bit) * VOUT_RATIO;
      g_smps_priv.vin  = (g_smps_priv.vin_raw * ref / bit) * VIN_RATIO;

#warning "missing regulator logic!"

    }

  /* Clear pending */

  stm32_adc->ops->int_ack(stm32_adc, pending);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_smps_setup
 *
 * Description:
 *  Initialize SMPS driver.
 *
 *  This function should be call by board_app_initialize().
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
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
          printf("ERROR: Failed to get HRTIM1 interface\n");
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
          printf("ERROR: Failed to get ADC %d interface\n", 1);
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
          fprintf(stderr, "highpri_main: ERROR: up_ramvec_attach failed: %d\n",
                  ret);
          ret = EXIT_FAILURE;
          goto errout;
        }

      /* Set the priority of the ADC12 interrupt vector */

      ret = up_prioritize_irq(STM32_IRQ_ADC12, NVIC_SYSH_HIGH_PRIORITY);
      if (ret < 0)
        {
          fprintf(stderr,
                  "highpri_main: ERROR: up_prioritize_irq failed: %d\n", ret);
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
          printf("ERROR: smps_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

errout:
  return ret;
}

#endif /* CONFIG_EXAMPLE_SMPS && CONFIG_DRIVERS_SMPS*/
