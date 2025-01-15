/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_pcnt.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <syslog.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/capture.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <arch/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/sensors/sensor.h>

#include "riscv_internal.h"
#include "esp_pcnt.h"
#include "esp_gpio.h"

#include "hal/pcnt_hal.h"
#include "hal/pcnt_ll.h"
#include "soc/gpio_sig_map.h"
#include "periph_ctrl.h"
#include "soc/soc_caps.h"
#include "soc/pcnt_periph.h"
#include "soc/pcnt_reg.h"
#include "soc/pcnt_struct.h"
#include "soc/gpio_pins.h"
#include "esp_clk.h"
#include "esp_irq.h"
#include "esp_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCNT_UNIT_COUNT                             SOC_PCNT_GROUPS * SOC_PCNT_UNITS_PER_GROUP
#define GET_UNIT_ID_FROM_RET_CHAN(chan_id)          (int)(chan_id/SOC_PCNT_CHANNELS_PER_UNIT)
#define GET_CHAN_ID_FROM_RET_CHAN(unit_id, chan_id) (chan_id - (SOC_PCNT_CHANNELS_PER_UNIT * unit_id))
#define CREATE_RET_CHAN_ID(unit_id, chan_id)        ((SOC_PCNT_CHANNELS_PER_UNIT * unit_id) + chan_id)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PCNT unit states */

enum esp_pcntstate_e
{
  PCNT_UNIT_INIT,
  PCNT_UNIT_ENABLE,
};

struct esp_pcnt_glitch_filter_config_s
{
    uint32_t max_glitch_ns;     /* Pulse width threshold, in ns */
};

/* PCNT Unit Watch Point Private Data */

struct esp_pcnt_watch_point_priv_s
{
    pcnt_ll_watch_event_id_t event_id; /* Event type */
    int watch_point_value;             /* Value to be watched */
};

/* PCNT Unit Private Data */

struct esp_pcnt_priv_s
{
  const struct cap_ops_s *ops;
  int unit_id;                                                          /* PCNT unit id */
  volatile enum esp_pcntstate_e state;                                  /* PCNT unit work state (see enum esp_pcntstate_e) */
  struct esp_pcnt_unit_config_s config;                                 /* Configuration struct */
  bool unit_used;                                                       /* PCNT unit usage flag */
  bool intr;                                                            /* PCNT unit interrupt enable flag */
  spinlock_t lock;                                                      /* Device specific lock. */
  int (*cb)(int, void *, void *);                                       /* User defined callback */
  uint32_t accum_value;                                                 /* Accumulator value of overflowed PCNT unit */
  bool channels[SOC_PCNT_CHANNELS_PER_UNIT];                            /* Channel information of PCNT unit */
  struct esp_pcnt_watch_point_priv_s watchers[PCNT_LL_WATCH_EVENT_MAX]; /* array of PCNT watchers */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_pcnt_open(struct cap_lowerhalf_s *dev);
static int esp_pcnt_close(struct cap_lowerhalf_s *dev);
static int IRAM_ATTR esp_pcnt_isr_default(int irq, void *context,
                                          void *arg);
static int esp_pcnt_isr_register(int (*fn)(int, void *, void *),
                                 int intr_alloc_flags);
static int esp_pcnt_ioctl(struct cap_lowerhalf_s *dev, int cmd,
                          unsigned long arg);
static int esp_pcnt_unit_enable(struct cap_lowerhalf_s *dev);
static int esp_pcnt_unit_disable(struct cap_lowerhalf_s *dev);
static int esp_pcnt_unit_start(struct cap_lowerhalf_s *dev);
static int esp_pcnt_unit_stop(struct cap_lowerhalf_s *dev);
static int esp_pcnt_unit_get_count(struct cap_lowerhalf_s *dev, int *ret);
static int esp_pcnt_unit_clear_count(struct cap_lowerhalf_s *dev);
static int esp_pcnt_unit_set_glitch_filter(struct cap_lowerhalf_s *dev,
    const struct esp_pcnt_glitch_filter_config_s *config);
static int esp_pcnt_unit_register_event_callback(struct cap_lowerhalf_s *dev,
                                           int (*fn)(int, void *, void *));

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Standard file operations */

static const struct cap_ops_s g_esp_pcnt_fops =
{
  .start       = esp_pcnt_open,
  .stop        = esp_pcnt_close,
  .getduty     = NULL,
  .getfreq     = NULL,
  .ioctl       = esp_pcnt_ioctl,
};

static struct esp_pcnt_priv_s pcnt_units[PCNT_UNIT_COUNT] =
{
    0
};

static pcnt_hal_context_t ctx;                    /* Struct of the common layer */
static mutex_t g_pcnt_lock = NXMUTEX_INITIALIZER; /* Mutual exclusion mutex */
static int g_pcnt_refs = 0;                       /* Reference count */
static bool g_pcnt_intr = false;                  /* ISR register flag for peripheral */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_pcnt_close
 *
 * Description:
 *   Stop PCNT unit from counting.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the sensor
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_pcnt_close(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  int ret = OK;

  ret = esp_pcnt_unit_stop(dev);
  if (ret != OK)
    {
      cperr("Failed to stop pcnt-%d", priv->unit_id);
      return ret;
    }

  if (priv->state != PCNT_UNIT_INIT)
    {
      ret = esp_pcnt_unit_disable(dev);
      if (ret != OK)
        {
          cperr("Failed to disable pcnt-%d", priv->unit_id);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esp_pcnt_open
 *
 * Description:
 *   Start PCNT unit for counting.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the sensor
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_pcnt_open(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  int ret = OK;

  if (priv->state == PCNT_UNIT_INIT)
    {
      ret = esp_pcnt_unit_enable(dev);
      if (ret != OK)
        {
          cperr("Failed to enable pcnt-%d", priv->unit_id);
          return ret;
        }
    }

  ret = esp_pcnt_unit_start(dev);
  if (ret != OK)
    {
       cperr("Failed to start pcnt-%d", priv->unit_id);
    }

  return ret;
}

/****************************************************************************
 * Name: esp_pcnt_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor
 *   cmd   - The ioctl command
 *   arg   - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_pcnt_ioctl(struct cap_lowerhalf_s *dev, int cmd,
                          unsigned long arg)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;

  struct esp_pcnt_glitch_filter_config_s glitch_config;
  int ret = 0;
  xcpt_t handler;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      case CAPIOC_PULSES:

        /* Get PCNT unit count value */

        ret = esp_pcnt_unit_get_count(dev, (int *)arg);
        if (ret != OK)
          {
            cperr("Could not clear pcnt-%d!\n", priv->unit_id);
          }

        break;

      case CAPIOC_CLR_CNT:

        /* Clear PCNT unit count value */

        ret = esp_pcnt_unit_clear_count(dev);
        if (ret != OK)
          {
            cperr("Could not clear pcnt-%d!\n", priv->unit_id);
          }

        break;

      case CAPIOC_FILTER:

        /* Configure glitch filter */

        if (arg != 0)
          {
            glitch_config.max_glitch_ns = arg;
            ret = esp_pcnt_unit_set_glitch_filter(dev, &glitch_config);
          }
        else
          {
            ret = esp_pcnt_unit_set_glitch_filter(dev, NULL);
          }

        if (ret != OK)
          {
            cperr("Could not to set glitch filter to pcnt-%d!\n",
                  priv->unit_id);
          }

        break;

      case CAPIOC_HANDLER:

        /* Set event callback */

        handler = (xcpt_t)arg;
        ret = esp_pcnt_unit_register_event_callback(dev, handler);
        if (ret != OK)
          {
            cperr("Could not register callback-%lx to pcnt-%d!\n",
                  (uint32_t)handler, priv->unit_id);
          }

        break;

      case CAPIOC_ADD_WP:

        /* Add watch point */

        ret = esp_pcnt_unit_add_watch_point(dev, (int)arg);
        if (ret != OK)
          {
            cperr("Could not to add watch point-%d to pcnt-%d!\n",
                  (int)arg, priv->unit_id);
          }

        break;

      default:
        cperr("Unrecognized IOCTL command: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_pcnt_isr_default
 *
 * Description:
 *   Handler for the PCNT controller interrupt.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - PCNT controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int IRAM_ATTR esp_pcnt_isr_default(int irq, void *context,
                                          void *arg)
{
  struct esp_pcnt_priv_s *unit;
  int unit_id = 0;
  uint32_t event_status = 0;
  uint32_t intr_status = pcnt_ll_get_intr_status(ctx.dev);
  struct esp_pcnt_watch_event_data_s data;
  irqstate_t flags;

  for (unit_id = 0; unit_id < SOC_PCNT_UNITS_PER_GROUP; unit_id++)
    {
      if (intr_status & PCNT_LL_UNIT_WATCH_EVENT(unit_id))
        break;
    }

  if (unit_id < SOC_PCNT_UNITS_PER_GROUP)
    {
      unit = &pcnt_units[unit_id];
      pcnt_ll_clear_intr_status(ctx.dev, PCNT_LL_UNIT_WATCH_EVENT(unit_id));
      event_status = pcnt_ll_get_event_status(ctx.dev, unit_id);
      while (event_status)
        {
          int event_id = __builtin_ffs(event_status) - 1;
          event_status &= (event_status - 1);

          flags = spin_lock_irqsave(&unit->lock);
          if (unit->config.accum_count)
            {
              if (event_id == PCNT_LL_WATCH_EVENT_LOW_LIMIT)
                {
                  unit->accum_value += unit->config.low_limit;
                }
              else if (event_id == PCNT_LL_WATCH_EVENT_HIGH_LIMIT)
                {
                  unit->accum_value += unit->config.high_limit;
                }
            }

          spin_unlock_irqrestore(&unit->lock, flags);
          if (unit->cb)
            {
              data.unit_id = unit_id;
              data.watch_point_value =
                unit->watchers[event_id].watch_point_value;
              data.zero_cross_mode =
                pcnt_ll_get_zero_cross_mode(ctx.dev, unit_id);
              unit->cb(irq, context, &data);
            }
        }
    }

  return 0;
}

/****************************************************************************
 * Name: esp_pcnt_isr_register
 *
 * Description:
 *   This function registers an interrupt service routine (ISR) for the PCNT
 *   peripheral. It allocates a CPU interrupt, attaches the ISR to the
 *   interrupt, and returns the status of the operation.
 *
 * Input Parameters:
 *   fn               - Pointer to the ISR function.
 *   intr_alloc_flags - Flags for the interrupt allocation.
 *
 * Returned Value:
 *   Returns OK on successful registration of the ISR; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

static int esp_pcnt_isr_register(int (*fn)(int, void *, void *),
                                 int intr_alloc_flags)
{
  int cpuint;
  int ret;
  int cpu = this_cpu();

  DEBUGASSERT(fn);

  cpuint = esp_setup_irq(pcnt_periph_signals.groups[0].irq,
                         ESP_IRQ_PRIORITY_DEFAULT,
                         ESP_IRQ_TRIGGER_LEVEL);
  if (cpuint < 0)
    {
      cperr("Failed to allocate a CPU interrupt.\n");
      return ERROR;
    }

  ret = irq_attach(ESP_SOURCE2IRQ(pcnt_periph_signals.groups[0].irq),
                   fn,
                   0);
  if (ret < 0)
    {
      cperr("Couldn't attach IRQ to handler.\n");
      esp_teardown_irq(pcnt_periph_signals.groups[0].irq, cpuint);
      return ERROR;
    }

  up_enable_irq(ESP_SOURCE2IRQ(pcnt_periph_signals.groups[0].irq));
  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_enable
 *
 * Description:
 *   Enable PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

static int esp_pcnt_unit_enable(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;

  if (!priv->unit_used)
    {
      cperr("Invalid unit ID!\n");
      return ERROR;
    }

  if (priv->state != PCNT_UNIT_INIT)
    {
      cperr("Unit is already enabled!\n");
      return ERROR;
    }

  if (priv->intr)
    {
      pcnt_ll_enable_intr(ctx.dev, PCNT_LL_UNIT_WATCH_EVENT(priv->unit_id),
                          true);
    }

  priv->state = PCNT_UNIT_ENABLE;
  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_disable
 *
 * Description:
 *   Disable PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

static int esp_pcnt_unit_disable(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;

  if (!priv->unit_used)
    {
      cperr("Invalid unit ID!\n");
      return ERROR;
    }

  if (priv->state != PCNT_UNIT_ENABLE)
    {
      cperr("Unit is already disabled!\n");
      return ERROR;
    }

  if (priv->intr)
    {
      pcnt_ll_enable_intr(ctx.dev, PCNT_LL_UNIT_WATCH_EVENT(priv->unit_id),
                          false);
    }

  priv->state = PCNT_UNIT_INIT;
  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_start
 *
 * Description:
 *   Start PCNT unit for counting.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

static int esp_pcnt_unit_start(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  irqstate_t flags;

  if (priv->state != PCNT_UNIT_ENABLE)
    {
      cperr("Unit not enabled yet!\n");
      return ERROR;
    }

  flags = spin_lock_irqsave(&priv->lock);
  pcnt_ll_start_count(ctx.dev, priv->unit_id);
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_stop
 *
 * Description:
 *   Stop PCNT unit from counting.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

static int esp_pcnt_unit_stop(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  irqstate_t flags;

  if (priv->state != PCNT_UNIT_ENABLE)
    {
      cperr("Unit not enabled yet!\n");
      return ERROR;
    }

  flags = spin_lock_irqsave(&priv->lock);
  pcnt_ll_stop_count(ctx.dev, priv->unit_id);
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_get_count
 *
 * Description:
 *   Get count value given of PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *   ret - Returned count value
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

static int esp_pcnt_unit_get_count(struct cap_lowerhalf_s *dev, int *ret)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  irqstate_t flags;

  if (!priv->unit_used)
    {
      cperr("Invalid unit ID!\n");
      return ERROR;
    }

  flags = spin_lock_irqsave(&priv->lock);
  *ret = pcnt_ll_get_count(ctx.dev, priv->unit_id) +
      priv->accum_value;
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_clear_count
 *
 * Description:
 *   Clear count value given of PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

static int esp_pcnt_unit_clear_count(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  irqstate_t flags;

  if (!priv->unit_used)
    {
      cperr("Invalid unit ID!\n");
      return ERROR;
    }

  flags = spin_lock_irqsave(&priv->lock);
  pcnt_ll_clear_count(ctx.dev, priv->unit_id);
  priv->accum_value = 0;
  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_set_glitch_filter
 *
 * Description:
 *   Configure glitch filter of given unit.
 *
 * Input Parameters:
 *   dev    - Pointer to the pcnt driver struct
 *   config - Glitch filter configuration
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

static int esp_pcnt_unit_set_glitch_filter(struct cap_lowerhalf_s *dev,
    const struct esp_pcnt_glitch_filter_config_s *config)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  uint32_t glitch_filter_thres = 0;
  irqstate_t flags;

  if (config != NULL)
    {
      glitch_filter_thres = ((esp_clk_apb_freq() / 1000000) *
                             (config->max_glitch_ns / 1000));
      if (glitch_filter_thres > PCNT_LL_MAX_GLITCH_WIDTH)
        {
          cperr("Glitch width %" PRId32 " out of range!\n",
                config->max_glitch_ns);
          return ERROR;
        }
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (config != NULL)
    {
      pcnt_ll_set_glitch_filter_thres(ctx.dev, priv->unit_id,
                                      glitch_filter_thres);
      pcnt_ll_enable_glitch_filter(ctx.dev, priv->unit_id, true);
    }
  else
    {
      pcnt_ll_enable_glitch_filter(ctx.dev, priv->unit_id, false);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_register_event_callback
 *
 * Description:
 *   Set event callbacks for PCNT unit
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *   fn  - Pointer to the ISR function.
 *
 * Returned Value:
 *   Returns OK on successful registration of the ISR; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

static int esp_pcnt_unit_register_event_callback(struct cap_lowerhalf_s *dev,
                                           int (*fn)(int, void *, void *))
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  int ret = OK;
  int intr_flag = (fn != NULL) ? 1 : 0;
  irqstate_t flags;

  priv->intr = intr_flag;
  flags = spin_lock_irqsave(&priv->lock);
  pcnt_ll_enable_intr(ctx.dev, PCNT_LL_UNIT_WATCH_EVENT(priv->unit_id),
                      intr_flag);
  spin_unlock_irqrestore(&priv->lock, flags);
  priv->cb = fn;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_pcnt_new_unit
 *
 * Description:
 *   Request PCNT unit and config it with given parameters.
 *
 * Input Parameters:
 *   config - PCNT unit configuration
 *
 * Returned Value:
 *   PCNT unit number (>=0) if success or -1 if fail.
 *
 ****************************************************************************/

struct cap_lowerhalf_s *esp_pcnt_new_unit(
  struct esp_pcnt_unit_config_s *config)
{
  int unit_id;
  int cpuint;
  int cpu = this_cpu();
  int ret;
  int i;
  irqstate_t flags;

  if (config == NULL)
    {
      cperr("Configuration struct is NULL!\n");
      return NULL;
    }

  if (config->low_limit >= 0 && config->low_limit < PCNT_LL_MIN_LIN)
    {
      cperr("Configuration low limit is out of range!\n");
      return NULL;
    }

  if (config->high_limit < 0 && config->high_limit >= PCNT_LL_MAX_LIM)
    {
      cperr("Configuration high limit is out of range!\n");
      return NULL;
    }

  flags = spin_lock_irqsave(&pcnt_units[unit_id].lock);

  if (g_pcnt_refs++ == 0)
    {
      periph_module_enable(pcnt_periph_signals.groups[0].module);
      periph_module_reset(pcnt_periph_signals.groups[0].module);
      pcnt_hal_init(&ctx, 0);
    }

  for (unit_id = 0; unit_id < PCNT_UNIT_COUNT; unit_id++)
    {
      if (!pcnt_units[unit_id].unit_used)
        {
          pcnt_units[unit_id].unit_used = true;
          break;
        }
    }

  if (unit_id == PCNT_UNIT_COUNT)
    {
      cperr("No available PCNT unit for allocation\n");

      spin_unlock_irqrestore(&pcnt_units[unit_id].lock, flags);
      return NULL;
    }

  spin_unlock_irqrestore(&pcnt_units[unit_id].lock, flags);
  cpinfo("Allocated pcnt unit: %" PRId16 "\n", unit_id);

  if (!g_pcnt_intr)
    {
      nxmutex_lock(&g_pcnt_lock);
      ret = esp_pcnt_isr_register(esp_pcnt_isr_default, 0);
      if (ret < 0)
        {
          pcnt_units[unit_id].unit_used = false;
          nxmutex_unlock(&g_pcnt_lock);
          return NULL;
        }

      g_pcnt_intr = true;
      nxmutex_unlock(&g_pcnt_lock);
    }

  pcnt_ll_disable_all_events(ctx.dev, unit_id);
  pcnt_ll_enable_glitch_filter(ctx.dev, unit_id, false);
  pcnt_ll_set_high_limit_value(ctx.dev, unit_id, config->high_limit);
  pcnt_ll_set_low_limit_value(ctx.dev, unit_id, config->low_limit);

  pcnt_units[unit_id].config.high_limit = config->high_limit;
  pcnt_units[unit_id].config.low_limit = config->low_limit;
  pcnt_units[unit_id].config.accum_count = config->accum_count;
  pcnt_units[unit_id].intr = config->accum_count;
  pcnt_units[unit_id].accum_value = 0;
  pcnt_units[unit_id].unit_id = unit_id;
  pcnt_units[unit_id].ops = &g_esp_pcnt_fops;

  flags = spin_lock_irqsave(&pcnt_units[unit_id].lock);

  pcnt_ll_stop_count(ctx.dev, unit_id);
  pcnt_ll_clear_count(ctx.dev, unit_id);

  pcnt_ll_enable_intr(ctx.dev, PCNT_LL_UNIT_WATCH_EVENT(unit_id),
                      config->accum_count);
  pcnt_ll_clear_intr_status(ctx.dev, PCNT_LL_UNIT_WATCH_EVENT(unit_id));
  spin_unlock_irqrestore(&pcnt_units[unit_id].lock, flags);

  pcnt_units[unit_id].state = PCNT_UNIT_INIT;
  for (i = 0; i < PCNT_LL_WATCH_EVENT_MAX; i++)
    {
      pcnt_units[unit_id].watchers[i].event_id = PCNT_LL_WATCH_EVENT_INVALID;
    }

  cpinfo("Allocated pcnt unit: %" PRId16 " count range:[%d %d]\n", unit_id,
         pcnt_units[unit_id].config.low_limit,
         pcnt_units[unit_id].config.high_limit);

  return (struct cap_lowerhalf_s *)&pcnt_units[unit_id];
}

/****************************************************************************
 * Name: esp_pcnt_del_unit
 *
 * Description:
 *   Delete PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_del_unit(struct cap_lowerhalf_s *dev)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  int i;
  irqstate_t flags;

  if (priv->state != PCNT_UNIT_INIT)
    {
      cperr("Unit is running!\n");
      return ERROR;
    }

  for (i = 0; i < SOC_PCNT_CHANNELS_PER_UNIT; i++)
    {
      if (!priv->channels[i])
        {
          cperr("Channel %" PRId8
                  " still in working!\n", i);
          return ERROR;
        }
    }

  cpinfo("Delete pcnt unit: %" PRId8 "\n", priv->unit_id);

  if (priv->intr)
    {
      pcnt_ll_enable_intr(ctx.dev, PCNT_LL_UNIT_WATCH_EVENT(priv->unit_id),
                          false);
      priv->intr = false;
    }

  flags = spin_lock_irqsave(&priv->lock);
  priv->unit_used = false;
  g_pcnt_refs--;
  if (g_pcnt_refs == 0)
    {
      periph_module_disable(pcnt_periph_signals.groups[0].module);
      esp_teardown_irq(pcnt_periph_signals.groups[0].irq, -ENOMEM);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_unit_add_watch_point
 *
 * Description:
 *   Add watch point to given PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *   ret - Value to watch
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_unit_add_watch_point(struct cap_lowerhalf_s *dev,
                                  int watch_point)
{
  struct esp_pcnt_priv_s *unit = (struct esp_pcnt_priv_s *)dev;
  int ret = OK;
  struct esp_pcnt_watch_point_priv_s *wp;
  irqstate_t flags;

  if (watch_point > unit->config.high_limit ||
      watch_point < unit->config.low_limit)
    {
      cperr("Watch point is out of limit!\n");
      return ERROR;
    }

  flags = spin_lock_irqsave(&unit->lock);

  /* zero cross watch point */

  if (watch_point == 0)
    {
      if (unit->watchers[PCNT_LL_WATCH_EVENT_ZERO_CROSS].event_id !=
          PCNT_LL_WATCH_EVENT_INVALID)
        {
          cperr("Zero cross event watcher has been installed already!\n");
          ret = ERROR;
        }
      else
        {
          unit->watchers[PCNT_LL_WATCH_EVENT_ZERO_CROSS].event_id =
            PCNT_LL_WATCH_EVENT_ZERO_CROSS;
          unit->watchers[PCNT_LL_WATCH_EVENT_ZERO_CROSS].watch_point_value =
            0;
          pcnt_ll_enable_zero_cross_event(ctx.dev,
                                          unit->unit_id,
                                          true);
        }
    }

  /* high limit watch point */

  else if (watch_point == unit->config.high_limit)
    {
      if (unit->watchers[PCNT_LL_WATCH_EVENT_HIGH_LIMIT].event_id !=
          PCNT_LL_WATCH_EVENT_INVALID)
        {
          cperr("High limit event watcher has been installed already!\n");
          ret = ERROR;
        }
      else
        {
          unit->watchers[PCNT_LL_WATCH_EVENT_HIGH_LIMIT].event_id =
            PCNT_LL_WATCH_EVENT_HIGH_LIMIT;
          unit->watchers[PCNT_LL_WATCH_EVENT_HIGH_LIMIT].watch_point_value =
            unit->config.high_limit;
          pcnt_ll_enable_high_limit_event(ctx.dev,
                                          unit->unit_id,
                                          true);
        }
    }

  /* low limit watch point */

  else if (watch_point == unit->config.low_limit)
    {
      if (unit->watchers[PCNT_LL_WATCH_EVENT_LOW_LIMIT].event_id !=
          PCNT_LL_WATCH_EVENT_INVALID)
        {
          cperr("Low limit event watcher has been installed already!\n");
          ret = ERROR;
        }
      else
        {
          unit->watchers[PCNT_LL_WATCH_EVENT_LOW_LIMIT].event_id =
            PCNT_LL_WATCH_EVENT_LOW_LIMIT;
          unit->watchers[PCNT_LL_WATCH_EVENT_LOW_LIMIT].watch_point_value =
            unit->config.low_limit;
          pcnt_ll_enable_low_limit_event(ctx.dev,
                                         unit->unit_id,
                                         true);
        }
    }

  /* other threshold watch point */

  else
    {
      int thres_num = SOC_PCNT_THRES_POINT_PER_UNIT - 1;
      switch (thres_num)
        {
          case 1:
            wp = &unit->watchers[PCNT_LL_WATCH_EVENT_THRES1];
            if (wp->event_id == PCNT_LL_WATCH_EVENT_INVALID)
              {
                wp->event_id = PCNT_LL_WATCH_EVENT_THRES1;
                wp->watch_point_value = watch_point;
                pcnt_ll_set_thres_value(ctx.dev,
                                        unit->unit_id,
                                        1,
                                        watch_point);
                pcnt_ll_enable_thres_event(ctx.dev,
                                           unit->unit_id,
                                           1,
                                           true);
                break;
              }
          else if (wp->watch_point_value == watch_point)
            {
              cperr("Watcher has been installed already!\n");
              ret = ERROR;
              break;
            }

          case 0:
            wp = &unit->watchers[PCNT_LL_WATCH_EVENT_THRES0];
            if (wp->event_id == PCNT_LL_WATCH_EVENT_INVALID)
              {
                wp->event_id = PCNT_LL_WATCH_EVENT_THRES0;
                wp->watch_point_value = watch_point;
                pcnt_ll_set_thres_value(ctx.dev,
                                        unit->unit_id,
                                        0,
                                        watch_point);
                pcnt_ll_enable_thres_event(ctx.dev,
                                           unit->unit_id,
                                           0,
                                           true);
                break;
              }
            else if (wp->watch_point_value == watch_point)
              {
                cperr("Watcher has been installed already!\n");
                ret = ERROR;
                break;
              }

      default:
          cperr("No free threshold watch point available!\n");
          ret = ERROR;
          break;
      }
    }

  spin_unlock_irqrestore(&unit->lock, flags);

  if (ret != ERROR)
    {
      cpinfo("Watchpoint %d for pcnt unit-%d added successfully\n",
             watch_point,
             unit->unit_id);
    }

  return ret;
}

/****************************************************************************
 * Name: esp_pcnt_unit_remove_watch_point
 *
 * Description:
 *   Remove watch point from given PCNT unit.
 *
 * Input Parameters:
 *   dev - Pointer to the pcnt driver struct
 *   ret - Watch point value to remove
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_unit_remove_watch_point(struct cap_lowerhalf_s *dev,
                                     int watch_point)
{
  struct esp_pcnt_priv_s *unit = (struct esp_pcnt_priv_s *)dev;
  int ret = OK;
  int i;
  pcnt_ll_watch_event_id_t event_id = PCNT_LL_WATCH_EVENT_INVALID;
  irqstate_t flags;

  flags = spin_lock_irqsave(&unit->lock);

  for (i = 0; i < PCNT_LL_WATCH_EVENT_MAX; i++)
    {
      if (unit->watchers[i].event_id != PCNT_LL_WATCH_EVENT_INVALID &&
          unit->watchers[i].watch_point_value == watch_point)
        {
          event_id = unit->watchers[i].event_id;
          unit->watchers[i].event_id = PCNT_LL_WATCH_EVENT_INVALID;
          break;
        }
    }

  switch (event_id)
    {
      case PCNT_LL_WATCH_EVENT_ZERO_CROSS:
        pcnt_ll_enable_zero_cross_event(ctx.dev, unit->unit_id, false);
        break;
      case PCNT_LL_WATCH_EVENT_LOW_LIMIT:
        pcnt_ll_enable_low_limit_event(ctx.dev, unit->unit_id, false);
        break;
      case PCNT_LL_WATCH_EVENT_HIGH_LIMIT:
        pcnt_ll_enable_high_limit_event(ctx.dev, unit->unit_id, false);
        break;
      case PCNT_LL_WATCH_EVENT_THRES0:
        pcnt_ll_enable_thres_event(ctx.dev, unit->unit_id, 0, false);
        break;
      case PCNT_LL_WATCH_EVENT_THRES1:
        pcnt_ll_enable_thres_event(ctx.dev, unit->unit_id, 1, false);
        break;
      default:
        break;
  }

  spin_unlock_irqrestore(&unit->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: esp_pcnt_new_channel
 *
 * Description:
 *   Request channel on given PCNT unit and config it with given parameters.
 *
 * Input Parameters:
 *   dev    - Pointer to the pcnt driver struct
 *   config - PCNT unit channel configuration
 *
 * Returned Value:
 *   PCNT unit channel number (>=0) if success or -1 if fail.
 *
 ****************************************************************************/

int esp_pcnt_new_channel(struct cap_lowerhalf_s *dev,
                         struct esp_pcnt_chan_config_s *config)
{
  struct esp_pcnt_priv_s *priv = (struct esp_pcnt_priv_s *)dev;
  int channel_id = -1;
  int unit_id = priv->unit_id;
  int gpio_mode;
  int virt_gpio;
  int ret_id = 0;
  const pcnt_signal_conn_t *chan;

  if (!config)
    {
      cperr("Channel config is NULL!\n");
      return ERROR;
    }

  if (!priv->unit_used)
    {
      cperr("Unit did not initialize yet!\n");
      return ERROR;
    }

  if (priv->state != PCNT_UNIT_INIT)
    {
      cperr("Unit already running!\n");
      return ERROR;
    }

  for (int i = 0; i < SOC_PCNT_CHANNELS_PER_UNIT; i++)
    {
      if (!priv->channels[i])
        {
          channel_id = i;
          break;
        }
    }

  if (channel_id == -1)
    {
      cperr("no free channel in unit %" PRId8 "!\n", priv->unit_id);
      return ERROR;
    }

  gpio_mode = INPUT_FUNCTION | PULLUP |
      (config->flags && ESP_PCNT_CHAN_IO_LOOPBACK ? OUTPUT_FUNCTION : 0);
  virt_gpio = (config->flags && ESP_PCNT_CHAN_VIRT_LVL_IO_LVL) ?
      GPIO_MATRIX_CONST_ONE_INPUT : GPIO_MATRIX_CONST_ZERO_INPUT;
  chan = &pcnt_periph_signals;

  if (config->edge_gpio_num >= 0)
    {
      esp_configgpio(config->edge_gpio_num, gpio_mode);
      esp_gpio_matrix_in(config->edge_gpio_num,
        chan->groups[0].units[unit_id].channels[channel_id].pulse_sig,
        (config->flags && ESP_PCNT_CHAN_INVERT_EDGE_IN));
    }
  else
    {
      /* using virtual IO */

      esp_gpio_matrix_in(virt_gpio,
        chan->groups[0].units[unit_id].channels[channel_id].pulse_sig,
        (config->flags && ESP_PCNT_CHAN_INVERT_EDGE_IN));
    }

  if (config->level_gpio_num >= 0)
    {
      esp_configgpio(config->level_gpio_num, gpio_mode);
      esp_gpio_matrix_in(config->level_gpio_num,
        chan->groups[0].units[unit_id].channels[channel_id].control_sig,
        (config->flags && ESP_PCNT_CHAN_INVERT_LVL_IN));
    }
  else
    {
      /* using virtual IO */

      esp_gpio_matrix_in(virt_gpio,
        chan->groups[0].units[unit_id].channels[channel_id].control_sig,
        (config->flags && ESP_PCNT_CHAN_INVERT_LVL_IN));
    }

  priv->channels[channel_id] = true;

  cpinfo("Added pcnt unit: %" PRId8 " channel: %" PRId8 "\n",
         unit_id, channel_id);

  ret_id = CREATE_RET_CHAN_ID(unit_id, channel_id);
  return ret_id;
}

/****************************************************************************
 * Name: esp_pcnt_del_channel
 *
 * Description:
 *   Delete PCNT unit channel.
 *
 * Input Parameters:
 *   channel - Channel number to delete
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

int esp_pcnt_del_channel(int channel)
{
  int unit_id = GET_UNIT_ID_FROM_RET_CHAN(channel);
  int chan_id = GET_CHAN_ID_FROM_RET_CHAN(unit_id, channel);
  irqstate_t flags;

  flags = spin_lock_irqsave(&pcnt_units[unit_id].lock);
  pcnt_units[unit_id].channels[chan_id] = false;
  spin_unlock_irqrestore(&pcnt_units[unit_id].lock, flags);

  cpinfo("Deleted pcnt unit: %" PRId8 " channel: %" PRId8 "\n",
         unit_id, chan_id);
  return OK;
}

/****************************************************************************
 * Name: esp_pcnt_channel_set_edge_action
 *
 * Description:
 *   Set channel actions when edge signal changes.
 *
 * Input Parameters:
 *   channel  - Channel number to set actions
 *   post act - Action on posedge signal
 *   neg_act  - Action on negedge signal
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

void esp_pcnt_channel_set_edge_action(int channel,
    enum esp_pcnt_chan_edge_action_e pos_act,
    enum esp_pcnt_chan_edge_action_e neg_act)
{
  int unit_id = GET_UNIT_ID_FROM_RET_CHAN(channel);
  int chan_id = GET_CHAN_ID_FROM_RET_CHAN(unit_id, channel);
  irqstate_t flags;

  flags = spin_lock_irqsave(&pcnt_units[unit_id].lock);
  pcnt_ll_set_edge_action(ctx.dev, unit_id, chan_id,
                          pos_act, neg_act);
  spin_unlock_irqrestore(&pcnt_units[unit_id].lock, flags);
}

/****************************************************************************
 * Name: esp_pcnt_channel_set_level_action
 *
 * Description:
 *   Set channel actions when level signal changes.
 *
 * Input Parameters:
 *   channel  - Channel number to set actions
 *   post act - Action on posedge signal
 *   neg_act  - Action on negedge signal
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

void esp_pcnt_channel_set_level_action(int channel,
    enum esp_pcnt_chan_level_action_e pos_act,
    enum esp_pcnt_chan_level_action_e neg_act)
{
  int unit_id = GET_UNIT_ID_FROM_RET_CHAN(channel);
  int chan_id = GET_CHAN_ID_FROM_RET_CHAN(unit_id, channel);
  irqstate_t flags;

  flags = spin_lock_irqsave(&pcnt_units[unit_id].lock);
  pcnt_ll_set_level_action(ctx.dev, unit_id, chan_id,
                           pos_act, neg_act);
  spin_unlock_irqrestore(&pcnt_units[unit_id].lock, flags);
}
