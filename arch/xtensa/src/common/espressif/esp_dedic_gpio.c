/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_dedic_gpio.c
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
#include <nuttx/ioexpander/gpio.h>

#include "chip.h"
#include "esp_dedic_gpio.h"
#if defined(CONFIG_ARCH_CHIP_ESP32S3)
#include "esp32s3_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#include "esp32s2_irq.h"
#include "esp32s2_gpio.h"
#endif

#if defined(CONFIG_ARCH_CHIP_ESP32S2)
#include "soc/dedic_gpio_struct.h"
#endif
#include "hal/dedic_gpio_ll.h"
#include "hal/dedic_gpio_cpu_ll.h"
#include "soc/dedic_gpio_periph.h"
#include "soc/gpio_sig_map.h"
#include "periph_ctrl.h"
#include "soc/soc_caps.h"
#include "soc/gpio_pins.h"
#include "esp_clk.h"
#include "esp_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_ESP32S3)
#define esp_configgpio            esp32s3_configgpio
#define esp_gpio_matrix_in        esp32s3_gpio_matrix_in
#define esp_gpio_matrix_out       esp32s3_gpio_matrix_out
#define ESP_IRQ_PRIORITY_DEFAULT  ESP32S3_INT_PRIO_DEF
#define ESP_IRQ_TRIGGER_LEVEL     ESP32S3_CPUINT_LEVEL
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#define esp_setup_irq             esp32s2_setup_irq
#define esp_teardown_irq          esp32s2_teardown_irq
#define esp_configgpio            esp32s2_configgpio
#define esp_gpio_matrix_in        esp32s2_gpio_matrix_in
#define esp_gpio_matrix_out       esp32s2_gpio_matrix_out
#define ESP_IRQ_PRIORITY_DEFAULT  ESP32S2_INT_PRIO_DEF
#define ESP_IRQ_TRIGGER_LEVEL     ESP32S2_CPUINT_LEVEL
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ
enum esp_dedic_gpio_intr_type_t
{
  DEDIC_GPIO_INTR_NONE,          /* No interrupt */
  DEDIC_GPIO_INTR_LOW_LEVEL = 2, /* Interrupt on low level */
  DEDIC_GPIO_INTR_HIGH_LEVEL,    /* Interrupt on high level */
  DEDIC_GPIO_INTR_NEG_EDGE,      /* Interrupt on negedge */
  DEDIC_GPIO_INTR_POS_EDGE,      /* Interrupt on posedge */
  DEDIC_GPIO_INTR_BOTH_EDGE      /* Interrupt on both negedge and posedge */
};
#endif

struct esp_dedic_gpio_bundle_priv_s
{
  struct file_operations *ops;  /* Externally visible part of the GPIO interface */
  uint32_t out_offset;          /* Offset of output channels */
  uint32_t in_offset;           /* Offset of input channels */
  uint32_t out_mask;            /* Mask of output channels */
  uint32_t in_mask;             /* Mask of input channels */
};

struct esp_dedic_gpio_common_priv_s
{
  int refs;                       /* Reference count */
  spinlock_t spinlock;            /* Held while chip is selected for mutual exclusion */
  uint32_t out_occupied_mask;     /* Mask of output channels */
  uint32_t in_occupied_mask;      /* Mask of input channels */
#ifdef CONFIG_ARCH_CHIP_ESP32S2
  dedic_dev_t *dev;
#endif
#ifdef CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ
  bool isr_init;                   /* ISR register flag for peripheral */
  int (*cbs[SOC_DEDIC_GPIO_IN_CHANNELS_NUM])(struct file *filep, int pin);

  /* Demonstrate that which bundle belongs to for input channel */

  struct esp_dedic_gpio_bundle_priv_s *
    in_bundles[SOC_DEDIC_GPIO_IN_CHANNELS_NUM];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_dedic_gpio_bundle_read(struct file *filep,
                                      char *buffer,
                                      size_t buflen);
static int esp_dedic_gpio_bundle_write(struct file *filep,
                                       const char *buffer,
                                       size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct file_operations dedic_gpio_ops =
{
  NULL,                               /* open */
  NULL,                               /* close */
  esp_dedic_gpio_bundle_read,         /* read */
  esp_dedic_gpio_bundle_write,        /* write */
};

static struct esp_dedic_gpio_common_priv_s
  dedic_gpio_common[SOC_CPU_CORES_NUM] =
{
  0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ

/****************************************************************************
 * Name: esp_dedic_gpio_pin_get_pin_attr
 *
 * Description:
 *   Convert pin type to dedicated GPIO pin attr.
 *
 * Input Parameters:
 *   pintype - The pin type. See nuttx/ioexpander/gpio.h.
 *
 * Returned Value:
 *   Dedicated gpio driver pin attr if available, 0 otherwise.
 *
 ****************************************************************************/

static int esp_dedic_gpio_pin_get_pin_attr(enum gpio_pintype_e pintype)
{
  switch (pintype)
    {
      case GPIO_INTERRUPT_LOW_PIN:
        return DEDIC_GPIO_INTR_LOW_LEVEL;
        break;
      case GPIO_INTERRUPT_HIGH_PIN:
        return DEDIC_GPIO_INTR_HIGH_LEVEL;
        break;
      case GPIO_INTERRUPT_FALLING_PIN:
        return DEDIC_GPIO_INTR_NEG_EDGE;
        break;
      case GPIO_INTERRUPT_RISING_PIN:
        return DEDIC_GPIO_INTR_POS_EDGE;
        break;
      case GPIO_INTERRUPT_BOTH_PIN:
        return DEDIC_GPIO_INTR_BOTH_EDGE;
        break;
      default:
        return DEDIC_GPIO_INTR_NONE;
        break;
    }

  return DEDIC_GPIO_INTR_NONE;
}

/****************************************************************************
 * Name: esp_pcnt_isr_default
 *
 * Description:
 *   Handler for the dedicated GPIO controller interrupt.
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

static int IRAM_ATTR esp_dedic_gpio_isr_default(int irq, void *context,
                                                void *arg)
{
  struct esp_dedic_gpio_common_priv_s *platform =
    (struct  esp_dedic_gpio_common_priv_s *)arg;
  uint32_t flags = spin_lock_irqsave(dedic_gpio_common->spinlock);
  uint32_t status = dedic_gpio_ll_get_interrupt_status(platform->dev);

  dedic_gpio_ll_clear_interrupt_status(platform->dev, status);
  spin_unlock_irqrestore(dedic_gpio_common->spinlock, flags);

  while (status)
    {
      uint32_t channel = __builtin_ffs(status) - 1;
      if (platform->cbs[channel] != NULL)
        {
          platform->cbs[channel]((struct file *)
                                platform->in_bundles[channel],
                                channel -
                                  platform->in_bundles[channel]->in_offset);
        }

      status = status & (status - 1);
    }

  return 0;
}

/****************************************************************************
 * Name: esp_pcnt_isr_register
 *
 * Description:
 *   This function registers an interrupt service routine (ISR) for the
 *   peripheral. It allocates a CPU interrupt, attaches the ISR to the
 *   interrupt, and returns the status of the operation.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Returns OK on successful registration of the ISR; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

static int esp_dedic_gpio_isr_register(void)
{
  int cpuint;
  int ret;
  int cpu = this_cpu();
  uint32_t status;

  cpuint = esp_setup_irq(dedic_gpio_periph_signals.irq,
                         ESP_IRQ_PRIORITY_DEFAULT,
                         ESP_IRQ_TRIGGER_LEVEL);
  if (cpuint < 0)
    {
      cperr("Failed to allocate a CPU interrupt.\n");
      return ERROR;
    }

  ret = irq_attach(dedic_gpio_periph_signals.irq + XTENSA_IRQ_FIRSTPERIPH,
                   esp_dedic_gpio_isr_default,
                   &dedic_gpio_common[cpu]);
  if (ret < 0)
    {
      cperr("Couldn't attach IRQ to handler.\n");
      esp_teardown_irq(dedic_gpio_periph_signals.irq, cpuint);
      return ERROR;
    }

  status = dedic_gpio_ll_get_interrupt_status(dedic_gpio_common[cpu].dev);
  dedic_gpio_ll_clear_interrupt_status(dedic_gpio_common[cpu].dev, status);
  up_enable_irq(dedic_gpio_periph_signals.irq + XTENSA_IRQ_FIRSTPERIPH);
  return OK;
}

#endif /* CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ */

/****************************************************************************
 * Name: esp_dedic_gpio_bundle_read
 *
 * Description:
 *   Read dedicated gpio bundle data.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the device.
 *   buffer - Buffer to read.
 *   buflen - Not in use, have it for compatibility reasons.
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int esp_dedic_gpio_bundle_read(struct file *filep,
                                      char *buffer,
                                      size_t buflen)
{
  struct inode *inode = filep->f_inode;
  struct esp_dedic_gpio_bundle_priv_s *priv = inode->i_private;
  uint32_t dedic_value = 0;

  UNUSED(buflen);
  DEBUGASSERT(priv != NULL && buffer != NULL);

  gpioinfo("Reading pin...\n");

  dedic_value = dedic_gpio_cpu_ll_read_in();
  *buffer = (dedic_value & priv->in_mask) >> (priv->in_offset);
  return OK;
}

/****************************************************************************
 * Name: esp_dedic_gpio_bundle_write
 *
 * Description:
 *   Write data to the dedicated gpio bundle.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the device.
 *   buffer - Buffer to write value to GPIO bundle.
 *   buflen - Mask of the value.
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int esp_dedic_gpio_bundle_write(struct file *filep,
                                       const char *buffer,
                                       size_t buflen)
{
  struct inode *inode = filep->f_inode;
  struct esp_dedic_gpio_bundle_priv_s *priv = inode->i_private;
  uint32_t mask_value = 0;
  uint32_t write_value = 0;

  DEBUGASSERT(priv != NULL);
  gpioinfo("Writing %d with mask: %d\n", (int)*buffer, buflen);

  mask_value = priv->out_mask & (buflen << priv->out_offset);
  write_value = *(buffer) << priv->out_offset;

  dedic_gpio_cpu_ll_write_mask(mask_value, write_value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ
/****************************************************************************
 * Name: esp_dedic_gpio_set_cb
 *
 * Description:
 *   Set event callbacks
 *
 * Input Parameters:
 *   dev      - Pointer to the dedicated gpio driver struct
 *   mask     - Mask of the GPIOs to assign callback
 *   pintype  - Type of interrupt to trigger
 *   callback - Pointer to the ISR function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_dedic_gpio_set_cb(struct file *dev,
                          uint32_t mask,
                          enum gpio_pintype_e pintype,
                          int (*callback)(struct file *dev, int pin))
{
  struct esp_dedic_gpio_bundle_priv_s *priv =
    (struct esp_dedic_gpio_bundle_priv_s *)dev;
  uint32_t channel_mask = priv->in_mask & (mask << priv->in_offset);
  uint32_t channel = 0;
  irqstate_t flags;
  int cpu = this_cpu();
  int pin_attr = esp_dedic_gpio_pin_get_pin_attr(pintype);

  while (channel_mask)
    {
      channel = __builtin_ffs(channel_mask) - 1;
      flags = spin_lock_irqsave(&dedic_gpio_common[cpu].spinlock);

      dedic_gpio_ll_set_interrupt_type(dedic_gpio_common[cpu].dev,
                                       channel,
                                       pin_attr);
      if (pin_attr != DEDIC_GPIO_INTR_NONE)
        {
          dedic_gpio_ll_enable_interrupt(dedic_gpio_common[cpu].dev,
                                         1 << channel,
                                         true);
        }
      else
        {
          dedic_gpio_ll_enable_interrupt(dedic_gpio_common[cpu].dev,
                                         1 << channel,
                                         false);
        }

      spin_unlock_irqrestore(&dedic_gpio_common[cpu].spinlock, flags);

      dedic_gpio_common[cpu].cbs[channel] = callback;
      dedic_gpio_common[cpu].in_bundles[channel] = priv;
      channel_mask = channel_mask & (channel_mask - 1);
    }
}
#endif

/****************************************************************************
 * Name: esp_dedic_gpio_write
 *
 * Description:
 *   Write data to the dedicated gpio bundle with given mask value.
 *
 * Input Parameters:
 *   dev    - Pointer to the dedicated gpio driver struct
 *   mask   - Mask of the GPIOs to be written
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_dedic_gpio_write(struct file *dev,
                          uint32_t mask,
                          uint32_t value)
{
  struct esp_dedic_gpio_bundle_priv_s *priv =
    (struct esp_dedic_gpio_bundle_priv_s *)dev;
  uint32_t mask_value = priv->out_mask & (mask << priv->out_offset);
  uint32_t write_value = value << priv->out_offset;

  dedic_gpio_cpu_ll_write_mask(mask_value, write_value);
}

/****************************************************************************
 * Name: esp_dedic_gpio_read
 *
 * Description:
 *   Read dedicated gpio bundle data.
 *
 * Input Parameters:
 *   dev    - Pointer to the dedicated gpio driver struct
 *   value  - Pointer to the read data will be saved
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_dedic_gpio_read(struct file *dev, int *value)
{
  struct esp_dedic_gpio_bundle_priv_s *priv =
    (struct esp_dedic_gpio_bundle_priv_s *)dev;

  *value = dedic_gpio_cpu_ll_read_in();
  *value = ((*value) & priv->in_mask) >> (priv->in_offset);
}

/****************************************************************************
 * Name: esp_dedic_gpio_new_bundle
 *
 * Description:
 *   Request dedicated GPIO bundle and config it with given parameters.
 *
 * Input Parameters:
 *   config - Dedicated GPIO bundle configuration
 *
 * Returned Value:
 *   Valid GPIO device structure reference on success; NULL on failure.
 *
 ****************************************************************************/

struct file *esp_dedic_gpio_new_bundle(
    struct esp_dedic_gpio_config_s *config)
{
  struct esp_dedic_gpio_bundle_priv_s *priv = NULL;
  irqstate_t flags = 0;
  uint32_t out_mask = 0;
  uint32_t in_mask = 0;
  uint32_t pattern = 0;
  uint32_t out_offset = 0;
  uint32_t in_offset = 0;
  gpio_pinattr_t attr = 0;
  int cpu = this_cpu();
#ifdef CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ
  int ret;
#endif

  DEBUGASSERT(config != NULL);
  DEBUGASSERT(config->gpio_array != NULL && config->array_size > 0);
  DEBUGASSERT(config->path != NULL);
  DEBUGASSERT(config->flags->input_enable ||
              config->flags->output_enable > 0);

  if (dedic_gpio_common[cpu].refs == 0)
    {
      flags = spin_lock_irqsave(&dedic_gpio_common[cpu].spinlock);

#ifdef CONFIG_ARCH_CHIP_ESP32S2
      dedic_gpio_common[cpu].dev = &DEDIC_GPIO;
#endif
      PERIPH_RCC_ATOMIC()
        {
          dedic_gpio_ll_enable_bus_clock(true);
          dedic_gpio_ll_reset_register();
        }

      dedic_gpio_common[cpu].out_occupied_mask =
        UINT32_MAX & ~((1 << SOC_DEDIC_GPIO_OUT_CHANNELS_NUM) - 1);
      dedic_gpio_common[cpu].in_occupied_mask =
        UINT32_MAX & ~((1 << SOC_DEDIC_GPIO_IN_CHANNELS_NUM) - 1);

      spin_unlock_irqrestore(&dedic_gpio_common[cpu].spinlock, flags);
    }

  priv = kmm_zalloc(sizeof(struct esp_dedic_gpio_bundle_priv_s));
  if (priv)
    {
      pattern = (1 << config->array_size) - 1;

      /* configure outwards channels */

      priv->ops = &dedic_gpio_ops;

      out_offset = 0;
      if (config->flags->output_enable)
        {
          if (config->array_size > SOC_DEDIC_GPIO_OUT_CHANNELS_NUM)
            {
              gpioerr("ERROR: array size(%d) exceeds maximum supported out\
                       channels(%d)\n",
                       config->array_size, SOC_DEDIC_GPIO_OUT_CHANNELS_NUM);
              free(priv);
              return NULL;
            }

          flags = spin_lock_irqsave(&dedic_gpio_common[cpu].spinlock);

          for (int i = 0;
                i <= SOC_DEDIC_GPIO_OUT_CHANNELS_NUM - config->array_size;
                  i++)
            {
              if ((dedic_gpio_common[cpu].out_occupied_mask & (pattern << i))
                   == 0)
                {
                  out_mask = pattern << i;
                  out_offset = i;
                  break;
                }
            }

          if (out_mask)
            {
              dedic_gpio_common[cpu].out_occupied_mask |= out_mask;
            }

          spin_unlock_irqrestore(&dedic_gpio_common[cpu].spinlock, flags);
          if (out_mask == 0)
            {
              gpioerr("ERROR: no free outward channels\n");
              free(priv);
              return NULL;
            }

          attr |= OUTPUT;

#ifdef CONFIG_ARCH_CHIP_ESP32S2
          dedic_gpio_ll_enable_instruction_access_out(
            dedic_gpio_common[cpu].dev,
            out_mask,
            true);
#endif
          gpioinfo("New out bundle(%p), offset=%"PRIu32", mask(%"PRIx32")",
                    priv, priv->out_offset, priv->out_mask);
        }

      if (config->flags->input_enable)
        {
          if (config->array_size > SOC_DEDIC_GPIO_IN_CHANNELS_NUM)
            {
              gpioerr("ERROR: array size(%d) exceeds maximum supported in\
                       channels(%d)\n",
                       config->array_size, SOC_DEDIC_GPIO_IN_CHANNELS_NUM);
              free(priv);
              return NULL;
            }

          flags = spin_lock_irqsave(&dedic_gpio_common[cpu].spinlock);

          for (int i = 0;
                i <= SOC_DEDIC_GPIO_IN_CHANNELS_NUM - config->array_size;
                  i++)
            {
              if ((dedic_gpio_common[cpu].in_occupied_mask &
                    (pattern << i)) == 0)
                {
                  in_mask = pattern << i;
                  in_offset = i;
                  break;
                }
            }

          if (in_mask)
            {
              dedic_gpio_common[cpu].in_occupied_mask |= in_mask;
            }

          spin_unlock_irqrestore(&dedic_gpio_common[cpu].spinlock, flags);
          if (in_mask == 0)
            {
              gpioerr("ERROR: no free inward channels\n");
              free(priv);
              return NULL;
            }

          attr |= INPUT;
          gpioinfo("New in bundle(%p), offset=%"PRIu32", mask(%"PRIx32")",
                    priv, priv->in_offset, priv->in_mask);
        }

      /* Route dedicated GPIO channel signals to GPIO matrix */

      for (int i = 0; i < config->array_size; i++)
        {
          esp_configgpio(config->gpio_array[i], attr);
          if (config->flags->input_enable)
            {
              esp_gpio_matrix_in(config->gpio_array[i],
  dedic_gpio_periph_signals.cores[cpu].in_sig_per_channel[in_offset + i],
  config->flags->invert_input_enable);
            }

          if (config->flags->output_enable)
            {
              esp_gpio_matrix_out(config->gpio_array[i],
  dedic_gpio_periph_signals.cores[cpu].out_sig_per_channel[in_offset + i],
  config->flags->invert_output_enable, 0);
            }
        }

      priv->out_mask = out_mask;
      priv->in_mask = in_mask;
      priv->out_offset = out_offset;
      priv->in_offset = in_offset;

#ifdef CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ
      if (config->flags->input_enable &&
          config->flags->intr_enable &&
          dedic_gpio_common[cpu].isr_init == false)
        {
          ret = esp_dedic_gpio_isr_register();
          if (ret != OK)
            {
              gpioerr("Failed to initialize interrupt\n");
              free(priv);
              return NULL;
            }
        }
#endif
    }
  else
    {
      gpioerr("ERROR: memory allocation failed\n");
      return NULL;
    }

  dedic_gpio_common[cpu].refs++;

  register_driver(config->path, &dedic_gpio_ops,
                  0666, priv);
  return (struct file *)priv;
}

/****************************************************************************
 * Name: esp_dedic_gpio_del_bundle
 *
 * Description:
 *   Delete dedicated gpio bundle.
 *
 * Input Parameters:
 *   dev - Pointer to the dedicated gpio driver struct
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

int esp_dedic_gpio_del_bundle(struct file *dev)
{
  struct esp_dedic_gpio_bundle_priv_s *priv =
    (struct esp_dedic_gpio_bundle_priv_s *)dev;
  int cpu = this_cpu();
  irqstate_t flags = spin_lock_irqsave(&dedic_gpio_common[cpu].spinlock);

  dedic_gpio_common[cpu].refs--;

  dedic_gpio_common[cpu].out_occupied_mask &= ~(priv->out_mask);
  dedic_gpio_common[cpu].in_occupied_mask &= ~(priv->in_mask);

  if (dedic_gpio_common[cpu].refs == 0)
    {
      PERIPH_RCC_ATOMIC()
        {
          dedic_gpio_ll_enable_bus_clock(false);
        }

#ifdef CONFIG_ESPRESSIF_DEDICATED_GPIO_IRQ
      dedic_gpio_ll_enable_interrupt(dedic_gpio_common[cpu].dev,
                                     ~0UL,
                                     false);
      up_disable_irq(dedic_gpio_periph_signals.irq + XTENSA_IRQ_FIRSTPERIPH);
      dedic_gpio_common[cpu].isr_init = false;
#endif
    }

  free(priv);
  priv = NULL;
  spin_unlock_irqrestore(&dedic_gpio_common[cpu].spinlock, flags);

  return OK;
}
