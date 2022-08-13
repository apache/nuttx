/****************************************************************************
 * arch/arm/src/tiva/common/tiva_adclow.c
 *
 *   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * The Tivaware sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 * Copyright (c) 2005-2014 Texas Instruments Incorporated.
 * All rights reserved.
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
 ****************************************************************************/

/* This is part of revision 2.1.0.12573 of the Tiva Peripheral Driver
 * Library.
 * References:
 *
 *   TM4C123GH6PM Series Data Sheet
 *   TI Tivaware driverlib ADC sample code.
 */

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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "tiva_gpio.h"
#include "tiva_adc.h"
#include "hardware/tiva_adc.h"
#include "hardware/tiva_pinmap.h"
#include "hardware/tiva_sysctrl.h"

#ifdef CONFIG_TIVA_ADC

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

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif
#ifndef CONFIG_SCHED_HPWORK
#  error High priority worker threads is required (CONFIG_SCHED_HPWORK)
#endif

/* PWM trigger support definitions ******************************************/

/* Decodes the PWM generator and module from trigger and converts
 * to the TSSEL_PS register
 */

#define ADC_TRIG_PWM_CFG(t) \
    (1 << (ADC_TSSEL_PS_SHIFT(ADC_TRIG_gen(t))))

/* ADC support definitions **************************************************/

#define SSE_PROC_TRIG(n)  (1 << (n))
#define SEM_PROCESS_PRIVATE 0
#define SEM_PROCESS_SHARED  1

/****************************************************************************
 * Public Functions
 * **************************************************************************/

/* Upper level ADC driver ***************************************************/

static int  tiva_adc_bind(struct adc_dev_s *dev,
                          const struct adc_callback_s *callback);
static void tiva_adc_reset(struct adc_dev_s *dev);
static int  tiva_adc_setup(struct adc_dev_s *dev);
static void tiva_adc_shutdown(struct adc_dev_s *dev);
static void tiva_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  tiva_adc_ioctl(struct adc_dev_s *dev, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* ADC lower half device operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = tiva_adc_bind,
  .ao_reset    = tiva_adc_reset,
  .ao_setup    = tiva_adc_setup,
  .ao_shutdown = tiva_adc_shutdown,
  .ao_rxint    = tiva_adc_rxint,
  .ao_ioctl    = tiva_adc_ioctl,
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tiva_adc_s
{
  struct adc_dev_s *dev;
  const struct adc_callback_s *cb;
  bool cfg;              /* Configuration state */
  bool ena;              /* Operation state */
  uint8_t devno;         /* ADC device number */
};

struct tiva_adc_sse_s
{
  sem_t exclsem;         /* Mutual exclusion semaphore */
  struct work_s work;    /* Supports the interrupt handling "bottom half" */
  bool cfg;              /* Configuration state */
  bool ena;              /* Sample sequencer operation state */
  uint8_t adc;           /* Parent peripheral */
  uint8_t num;           /* SSE number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tiva_adc_interrupt(struct tiva_adc_sse_s *sse);
#ifdef CONFIG_DEBUG_ANALOG
static void tiva_adc_runtimeobj_ptrs(void);
static void tiva_adc_runtimeobj_vals(void);
static void tiva_adc_dump_dev(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Run-time ADC objects */

#ifdef CONFIG_TIVA_ADC0
static struct adc_dev_s      dev0;
static struct tiva_adc_s     adc0;

static struct tiva_adc_sse_s sse00;
static struct tiva_adc_sse_s sse01;
static struct tiva_adc_sse_s sse02;
static struct tiva_adc_sse_s sse03;
#endif

#ifdef CONFIG_TIVA_ADC1
static struct adc_dev_s      dev1;
static struct tiva_adc_s     adc1;

static struct tiva_adc_sse_s sse10;
static struct tiva_adc_sse_s sse11;
static struct tiva_adc_sse_s sse12;
static struct tiva_adc_sse_s sse13;
#endif

/* Offer run-time ADC objects in array form to help reduce the reliance on
 * hard-coded, non-generic function calls.
 */

static struct adc_dev_s *g_devs[] =
{
#ifdef CONFIG_TIVA_ADC0
  &dev0,
#else
  NULL,
#endif
#ifdef CONFIG_TIVA_ADC1
  &dev1
#else
  NULL
#endif
};

static struct tiva_adc_s *g_adcs[] =
{
#ifdef CONFIG_TIVA_ADC0
  &adc0,
#else
  NULL,
#endif
#ifdef CONFIG_TIVA_ADC1
  &adc1
#else
  NULL
#endif
};

static struct tiva_adc_sse_s *g_sses[] =
{
#ifdef CONFIG_TIVA_ADC0
  &sse00, &sse01, &sse02, &sse03,
#else
  NULL, NULL, NULL, NULL,
#endif
#ifdef CONFIG_TIVA_ADC1
  &sse10, &sse11, &sse12, &sse13
#else
  NULL, NULL, NULL, NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Interrupt handlers are inevitably hard-coded and specific */

#ifdef CONFIG_TIVA_ADC0
static int tiva_adc0_sse0_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse00);
  return OK;
}

static int tiva_adc0_sse1_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse01);
  return OK;
}

static int tiva_adc0_sse2_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse02);
  return OK;
}

static int tiva_adc0_sse3_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse03);
  return OK;
}
#endif

#ifdef CONFIG_TIVA_ADC1
static int tiva_adc1_sse0_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse10);
  return OK;
}

static int tiva_adc1_sse1_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse11);
  return OK;
}

static int tiva_adc1_sse2_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse12);
  return OK;
}

static int tiva_adc1_sse3_interrupt(int irq, void *context)
{
  tiva_adc_interrupt(&sse13);
  return OK;
}
#endif

static void tiva_adc_irqinitialize(struct tiva_adc_cfg_s *cfg)
{
  ainfo("initialize irqs for ADC%d...\n", cfg->adc);

#ifdef CONFIG_TIVA_ADC0
  if (cfg->adc == 0)
    {
      if (cfg->sse[0] && (cfg->ssecfg[0].trigger > 0))
        {
          tiva_adc_irq_attach(0, 0, tiva_adc0_sse0_interrupt);
        }

      if (cfg->sse[1] && (cfg->ssecfg[1].trigger > 0))
        {
          tiva_adc_irq_attach(0, 1, tiva_adc0_sse1_interrupt);
        }

      if (cfg->sse[2] && (cfg->ssecfg[2].trigger > 0))
        {
          tiva_adc_irq_attach(0, 2, tiva_adc0_sse2_interrupt);
        }

      if (cfg->sse[3] && (cfg->ssecfg[3].trigger > 0))
        {
          tiva_adc_irq_attach(0, 3, tiva_adc0_sse3_interrupt);
        }
    }

#endif
#ifdef CONFIG_TIVA_ADC1
  if (cfg->adc == 1)
    {
      if (cfg->sse[0] && (cfg->ssecfg[0].trigger > 0))
        {
          tiva_adc_irq_attach(1, 0, tiva_adc1_sse0_interrupt);
        }

      if (cfg->sse[1] && (cfg->ssecfg[1].trigger > 0))
        {
          tiva_adc_irq_attach(1, 1, tiva_adc1_sse1_interrupt);
        }

      if (cfg->sse[2] && (cfg->ssecfg[2].trigger > 0))
        {
          tiva_adc_irq_attach(1, 2, tiva_adc1_sse2_interrupt);
        }

      if (cfg->sse[3] && (cfg->ssecfg[3].trigger > 0))
        {
          tiva_adc_irq_attach(1, 3, tiva_adc1_sse3_interrupt);
        }
    }
#endif
}

/****************************************************************************
 * Name: tiva_adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int tiva_adc_bind(struct adc_dev_s *dev,
                         const struct adc_callback_s *callback)
{
  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: tiva_adc_reset
 *
 * Description:
 *   Reset the ADC device. Called early to initialize the hardware. This is
 *   called before tiva_adc_setup() and on error conditions.
 *
 *   Resetting disables interrupts and the enabled SSEs for the ADC.
 *
 ****************************************************************************/

static void tiva_adc_reset(struct adc_dev_s *dev)
{
  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;
  struct tiva_adc_sse_s *sse;
  uint8_t s;

  ainfo("Resetting...\n");

  tiva_adc_rxint(dev, false);

  for (s = 0; s < 4; ++s)
    {
      sse = g_sses[SSE_IDX(priv->devno, s)];

      if (sse->ena == true)
        {
          tiva_adc_sse_enable(priv->devno, s, false);
          sse->ena = false;
        }
    }
}

/****************************************************************************
 * Name: tiva_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened. This will occur when the port is first opened.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int tiva_adc_setup(struct adc_dev_s *dev)
{
  struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;
  struct tiva_adc_sse_s *sse;
  uint8_t s = 0;

  ainfo("Setup\n");

  priv->ena = true;

  for (s = 0; s < 4; ++s)
    {
      sse = g_sses[SSE_IDX(priv->devno, s)];
      if (sse->cfg == true)
        {
          tiva_adc_sse_enable(priv->devno, s, true);
          sse->ena = true;
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

  ainfo("Shutdown\n");
  DEBUGASSERT(priv->ena);

  /* Resetting the ADC peripheral disables interrupts and all SSEs */

  tiva_adc_reset(dev);

  /* Currently all of the setup operations are undone in reset() */

#if 0
  struct tiva_adc_sse_s *sse;
  uint8_t s = 0;

  for (s = 0; s < 4; ++s)
    {
    }
#endif

  priv->ena = false;
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
  struct tiva_adc_sse_s *sse;
  uint32_t trigger;
  uint8_t s = 0;

  ainfo("RXINT=%d\n", enable);
  DEBUGASSERT(priv->ena);

  for (s = 0; s < 4; ++s)
    {
      trigger = tiva_adc_get_trigger(priv->devno, s);
      sse = g_sses[SSE_IDX(priv->devno, s)];
      if ((sse->ena == true)
          && (trigger > 0))
        {
          tiva_adc_sse_int_enable(priv->devno, s, enable);
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
  int ret = OK;

  ainfo("cmd=%d arg=%ld\n", cmd, arg);

  switch (cmd)
    {
      /* Software trigger */

      case ANIOC_TRIGGER:
        {
          struct tiva_adc_s *priv = (struct tiva_adc_s *)dev->ad_priv;
          uint8_t i               = 0;
          uint8_t fifo_count      = 0;
          uint8_t sse             = (uint8_t) arg;
          int32_t buf[8];

          /* Get exclusive access to the driver data structure */

          ret = tiva_adc_lock(priv, sse);
          if (ret < 0)
            {
              return ret;
            }

          /* Start conversion and wait for end of conversion */

          tiva_adc_proc_trig(priv->devno, (uint8_t)SSE_PROC_TRIG(sse));
          while (!tiva_adc_sse_int_status(priv->devno, sse))
            {
              nxsig_usleep(100);
            }

          tiva_adc_sse_clear_int(priv->devno, sse);

          /* Pass sampled data to upper ADC driver */

          fifo_count = tiva_adc_sse_data(priv->devno, sse, buf);

          /* Verify that the upper-half driver has bound its callback
           * functions
           */

          if (priv->cb != NULL)
            {
              DEBUGASSERT(priv->cb->au_receive != NULL);

              for (i = 0; i < fifo_count; ++i)
                {
                  /* Perform the data received callback */

                  priv->cb->au_receive(dev,
                                       tiva_adc_get_ain(priv->devno, sse, i),
                                       buf[i]);
                }
            }

          /* Release our lock on the ADC structure */

          tiva_adc_unlock(priv, sse);
        }
        break;

      /* PWM triggering */

#warning Missing Logic

      /* TODO: Needs to be tested */

#ifdef CONFIG_EXPERIMENTAL
      case TIVA_ADC_PWM_TRIG_IOCTL:
        {
          uint8_t sse    = (uint8_t)(arg & 0x2);
          uint8_t regval = tiva_adc_get_trigger(adc, sse);

          /* Verify input SSE trigger is a PWM trigger */

          if ((regval & TIVA_ADC_TRIG_PWM0) ||
              (regval & TIVA_ADC_TRIG_PWM1) ||
              (regval & TIVA_ADC_TRIG_PWM2) ||
              (regval & TIVA_ADC_TRIG_PWM3))
            {
              tiva_adc_sse_pwm_trig(adc, sse, (uint32_t)(arg & 0xfffffffc));
            }
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
 *   This function executes on the worker thread. It is scheduled by
 *   tiva_adc_interrupt whenever any enabled event occurs. All interrupts
 *   are disabled when this function runs. tiva_adc_read will
 *   re-enable interrupts when it completes processing all pending events.
 *
 * Input Parameters:
 *   arg - The ADC SSE data structure cast to (void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tiva_adc_read(void *arg)
{
  struct tiva_adc_s     *priv;
  struct tiva_adc_sse_s *sse        = (struct tiva_adc_sse_s *)arg;
  struct adc_dev_s      *dev        = 0;
  int                    irq        = tiva_adc_getirq(sse->adc, sse->num);
  uint8_t                i          = 0;
  uint8_t                fifo_count = 0;
  int32_t                buf[8];
  int                    ret;

  /* Get exclusive access to the driver data structure */

  ret = tiva_adc_lock(g_adcs[sse->adc], sse->num);
  if (ret < 0)
    {
      return ;
    }

  /* Get sampled data */

  fifo_count = tiva_adc_sse_data(sse->adc, sse->num, buf);

  /* Determine which adc_dev_s we need */

  dev = g_devs[sse->adc];
  if (dev == NULL)
    {
      /* This is a serious error: indicates invalid pointer indirection
       * and should cause a full system stop.
       */

      aerr("ERROR: Invalid ADC device number given %d\n", sse->adc);
      DEBUGPANIC();
      return;
    }

  priv = (struct tiva_adc_s *)dev->ad_priv;

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < fifo_count; ++i)
        {
          /* Perform the data received callback */

          priv->cb->au_receive(dev,
                               tiva_adc_get_ain(sse->adc, sse->num, i),
                               buf[i]);
          ainfo("AIN%d = 0x%04x\n",
                tiva_adc_get_ain(sse->adc, sse->num, i), buf[i]);
        }
    }

  /* Exit, re-enabling ADC interrupts */

  up_enable_irq(irq);

  /* Release our lock on the ADC structure */

  tiva_adc_unlock(g_adcs[sse->adc], sse->num);
}

/****************************************************************************
 * Name: tiva_adc_interrupt
 *
 * Description:
 *   This function is called by every interrupt handler and handles the
 *   actual worker dispatching.
 *
 ****************************************************************************/

static void tiva_adc_interrupt(struct tiva_adc_sse_s *sse)
{
  int ret;
  int irq = tiva_adc_getirq(sse->adc, sse->num);

  DEBUGASSERT(sse->ena == true);

  /* Disable further  interrupts. Interrupts will be re-enabled
   * after the worker thread executes.
   */

  up_disable_irq(irq);

  /* Clear interrupt status */

  tiva_adc_sse_clear_int(sse->adc, sse->num);

  /* Transfer processing to the worker thread.  Since interrupts are
   * disabled while the work is pending, no special action should be
   * required to protected the work queue.
   */

  DEBUGASSERT(sse->work.worker == NULL);
  ret = work_queue(HPWORK, &sse->work, tiva_adc_read, sse, 0);
  if (ret != 0)
    {
      aerr("ERROR: Failed to queue work: %d ADC.SSE: %d.%d\n",
           ret, sse->adc, sse->num);
    }
}

/****************************************************************************
 * Name: tiva_adc_struct_init
 *
 * Description:
 *   Initialize public and private adc structures their member SSE's.
 *
 ****************************************************************************/

static struct tiva_adc_s *tiva_adc_struct_init(struct tiva_adc_cfg_s *cfg)
{
  struct tiva_adc_s     *adc           = g_adcs[cfg->adc];
  struct tiva_adc_sse_s *sse           = 0;
  uint8_t                s             = 0;

  /* Do not re-initialize the run-time structures, there is a chance another
   * process is also using this ADC.
   */

  if (adc->cfg == true)
    {
      goto tiva_adc_struct_init_ok;
    }
  else
    {
      if (adc != NULL)
        {
          adc->ena = false;
          adc->devno = cfg->adc;

          for (s = 0; s < 4; s++)
            {
              /* Only configure selected SSEs */

              if (cfg->sse[s])
                {
                  sse = g_sses[SSE_IDX(cfg->adc, s)];

                  if (sse != NULL)
                    {
                      sse->adc = cfg->adc;
                      sse->num = s;
                      nxsem_init(&sse->exclsem, SEM_PROCESS_PRIVATE, 1);
                      sse->ena = false;
                      sse->cfg = true;
                    }
                  else
                    {
                      goto tiva_adc_struct_init_error;
                    }
                }
            }

          /* Initialize the public ADC device data structure */

          adc->dev = g_devs[cfg->adc];
          if (adc->dev != NULL)
            {
              adc->dev->ad_ops  = &g_adcops;
              adc->dev->ad_priv = adc;
            }
          else
            {
              goto tiva_adc_struct_init_error;
            }

          goto tiva_adc_struct_init_ok;
        }
      else
        {
          goto tiva_adc_struct_init_error;
        }
    }

tiva_adc_struct_init_error:
  ainfo("Invalid ADC device number: expected=%d actual=%d\n",
        0, cfg->adc);
  ainfo("ADC%d (CONFIG_TIVA_ADC%d) must be enabled in Kconfig first!",
        cfg->adc, cfg->adc);
  return NULL;

tiva_adc_struct_init_ok:
  adc->cfg = true;
  return adc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_adc_initialize
 *
 * Description:
 *   Configuration and bind the ADC to the ADC lower half instance and
 *   register the ADC driver at 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the ADC device.  This should be of the
 *     form /dev/adc0
 *   cfg - ADC configuration structure, configures the whole ADC.
 *   clock - clock speed for all ADC's. This is only set once for the first
 *     call to tiva_adc_initialize, otherwise the values are ignored.
 *   sample_rate - maximum sample rate of any ADC. This is only set once
 *     for the first call to tiva_adc_initialize, otherwise the values are
 *     ignored.
 *
 ****************************************************************************/

int tiva_adc_initialize(const char *devpath, struct tiva_adc_cfg_s *cfg,
                                      uint32_t clock, uint8_t sample_rate)
{
  struct tiva_adc_s *adc;
  int ret = 0;

  ainfo("initializing...\n");

  /* Initialize the private ADC device data structure */

  adc = tiva_adc_struct_init(cfg);
  if (adc == NULL)
    {
      aerr("ERROR: Invalid ADC device number: expected=%d actual=%d\n",
           0, cfg->adc);
      return -ENODEV;
    }

  /* Turn on peripheral */

  if (tiva_adc_enable(adc->devno, true) < 0)
    {
      aerr("ERROR: failure to power ADC peripheral (devno=%d)\n",
           cfg->adc);
      return ret;
    }

  /* Perform actual initialization */

  tiva_adc_one_time_init(clock, sample_rate);
  tiva_adc_configure(cfg);
  tiva_adc_irqinitialize(cfg);

  /* Now we are initialized */

  adc->ena = true;
  adc->cb  = NULL;

#ifdef CONFIG_DEBUG_ANALOG
  tiva_adc_runtimeobj_vals();
#endif

  /* Ensure our lower half is valid */

  if (adc->dev == NULL)
    {
      aerr("ERROR: Failed to get interface %s\n", devpath);
      return -ENODEV;
    }

  ainfo("adc_dev_s=0x%08x\n", adc->dev);

  /* Register the ADC driver */

  ainfo("Register the ADC driver at %s\n", devpath);

  ret = adc_register(devpath, adc->dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to register %s to character driver: %d\n",
           devpath, ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_adc_lock
 *
 * Description:
 *   Get exclusive access to the ADC interface
 *
 ****************************************************************************/

int tiva_adc_lock(struct tiva_adc_s *priv, int sse)
{
  struct tiva_adc_sse_s *s = g_sses[SSE_IDX(priv->devno, sse)];

  ainfo("Locking...\n");
  return nxsem_wait_uninterruptible(&s->exclsem);
}

/****************************************************************************
 * Name: tiva_adc_unlock
 *
 * Description:
 *   Relinquish the lock on the ADC interface
 *
 ****************************************************************************/

void tiva_adc_unlock(struct tiva_adc_s *priv, int sse)
{
  struct tiva_adc_sse_s *s = g_sses[SSE_IDX(priv->devno, sse)];
  ainfo("Unlocking\n");
  nxsem_post(&s->exclsem);
}

#ifdef CONFIG_DEBUG_ANALOG

/****************************************************************************
 * Name: tiva_adc_runtimeobj_ptrs
 *
 * Description:
 *   Dumps the address of all run-time objects for verification.
 *
 ****************************************************************************/

static void tiva_adc_runtimeobj_ptrs(void)
{
#  ifdef CONFIG_TIVA_ADC0
  ainfo("ADC0 [struct]       [global value]   [array value]\n");
  ainfo("     adc_dev_s      dev0=0x%08x   g_devs[0]=0x%08x\n",
        &dev0, g_devs[0]);
  ainfo("     tiva_adc_s     adc0=0x%08x   g_adcs[0]=0x%08x\n",
        &adc0, g_adcs[0]);
  ainfo("     tiva_adc_sse_s sse0=0x%08x   g_sses[0,0]=0x%08x\n",
        &sse00, g_sses[SSE_IDX(0, 0)]);
  ainfo("     tiva_adc_sse_s sse1=0x%08x   g_sses[0,1]=0x%08x\n",
        &sse01, g_sses[SSE_IDX(0, 1)]);
  ainfo("     tiva_adc_sse_s sse2=0x%08x   g_sses[0,2]=0x%08x\n",
        &sse02, g_sses[SSE_IDX(0, 2)]);
  ainfo("     tiva_adc_sse_s sse3=0x%08x   g_sses[0,3]=0x%08x\n",
        &sse03, g_sses[SSE_IDX(0, 3)]);
#  endif
#  ifdef CONFIG_TIVA_ADC1
  ainfo("ADC1 [struct]       [global value]   [array value]\n");
  ainfo("     adc_dev_s      dev1=0x%08x   g_devs[1]=0x%08x\n",
        &dev1, g_devs[1]);
  ainfo("     tiva_adc_s     adc1=0x%08x   g_adcs[1]=0x%08x\n",
        &adc1, g_adcs[1]);
  ainfo("     tiva_adc_sse_s sse0=0x%08x   g_sses[1,0]=0x%08x\n",
        &sse10, g_sses[SSE_IDX(1, 0)]);
  ainfo("     tiva_adc_sse_s sse1=0x%08x   g_sses[1,1]=0x%08x\n",
        &sse11, g_sses[SSE_IDX(1, 1)]);
  ainfo("     tiva_adc_sse_s sse2=0x%08x   g_sses[1,2]=0x%08x\n",
        &sse12, g_sses[SSE_IDX(1, 2)]);
  ainfo("     tiva_adc_sse_s sse3=0x%08x   g_sses[1,3]=0x%08x\n",
        &sse13, g_sses[SSE_IDX(1, 3)]);
#  endif
}

static void tiva_adc_runtimeobj_vals(void)
{
  struct tiva_adc_sse_s *sse;
  uint8_t s;

#  ifdef CONFIG_TIVA_ADC0
  ainfo("ADC0 [0x%08x] cfg=%d ena=%d devno=%d\n",
         &adc0, adc0.cfg, adc0.ena, adc0.devno);

  for (s = 0; s < 4; ++s)
    {
      sse = g_sses[SSE_IDX(0, s)];
      ainfo("SSE%d [0x%08x] adc=%d cfg=%d ena=%d num=%d\n",
             s, sse, sse->adc, sse->cfg, sse->ena, sse->num);
    }

#  endif
#  ifdef CONFIG_TIVA_ADC1
  ainfo("ADC1 [0x%08x] cfg=%d ena=%d devno=%d\n",
         &adc1, adc1.cfg, adc1.ena, adc1.devno);

  for (s = 0; s < 4; ++s)
    {
      sse = g_sses[SSE_IDX(1, s)];
      ainfo("SSE%d [0x%08x] adc=%d cfg=%d ena=%d num=%d\n",
             s, sse, sse->adc, sse->cfg, sse->ena, sse->num);
    }
#  endif
}

/****************************************************************************
 * Name: tiva_adc_unlock
 *
 * Description:
 *   umps the device level objects for verification.
 *
 ****************************************************************************/

static void tiva_adc_dump_dev(void)
{
#  ifdef CONFIG_TIVA_ADC0
  ainfo("adc_ops_s  g_adcops=0x%08x adc0.dev->ad_ops=0x%08x\n",
         &g_adcops, adc0.dev->ad_ops);
  ainfo("tiva_adc_s adc0=0x%08x      adc0.dev->ad_priv=0x%08x\n",
         &adc0, adc0.dev->ad_priv);
#  endif
#  ifdef CONFIG_TIVA_ADC1
  ainfo("adc_ops_s  g_adcops=0x%08x adc1.dev->ad_ops=0x%08x\n",
         &g_adcops, adc1.dev->ad_ops);
  ainfo("tiva_adc_s adc1=0x%08x      adc1.dev->ad_priv=0x%08x\n",
         &adc1, adc1.dev->ad_priv);
#  endif
}
#endif

#endif /* CONFIG_TIVA_ADC */
