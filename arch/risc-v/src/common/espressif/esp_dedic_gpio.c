/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_dedic_gpio.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/ioexpander/gpio.h>

#include "riscv_internal.h"
#include "esp_dedic_gpio.h"
#include "esp_gpio.h"
#include "soc/dedic_gpio_periph.h"
#include "hal/dedic_gpio_cpu_ll.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_dedic_gpio_common_priv_s
{
  int refs;                       /* Reference count */
  spinlock_t spinlock;            /* Held while chip is selected for mutual exclusion */
  uint32_t out_occupied_mask;     /* Mask of output channels */
  uint32_t in_occupied_mask;      /* Mask of input channels */
};

struct esp_dedic_gpio_bundle_priv_s
{
  struct file_operations *ops;  /* Externally visible part of the GPIO interface */
  uint32_t out_offset;          /* Offset of output channels */
  uint32_t in_offset;           /* Offset of input channels */
  uint32_t out_mask;            /* Mask of output channels */
  uint32_t in_mask;             /* Mask of input channels */
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

static struct esp_dedic_gpio_common_priv_s dedic_gpio_common =
{
  .refs = 0,
  .spinlock = SP_UNLOCKED,
  .out_occupied_mask = 0,
  .in_occupied_mask = 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  DEBUGASSERT(config != NULL);
  DEBUGASSERT(config->gpio_array != NULL && config->array_size > 0);
  DEBUGASSERT(config->path != NULL);
  DEBUGASSERT(config->flags->input_enable ||
              config->flags->output_enable > 0);

  if (dedic_gpio_common.refs == 0)
    {
      flags = spin_lock_irqsave(&dedic_gpio_common.spinlock);

      dedic_gpio_common.out_occupied_mask =
        UINT32_MAX & ~((1 << SOC_DEDIC_GPIO_OUT_CHANNELS_NUM) - 1);
      dedic_gpio_common.in_occupied_mask =
        UINT32_MAX & ~((1 << SOC_DEDIC_GPIO_IN_CHANNELS_NUM) - 1);

      spin_unlock_irqrestore(&dedic_gpio_common.spinlock, flags);
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

          flags = spin_lock_irqsave(&dedic_gpio_common.spinlock);

          for (int i = 0;
                i <= SOC_DEDIC_GPIO_OUT_CHANNELS_NUM - config->array_size;
                  i++)
            {
              if ((dedic_gpio_common.out_occupied_mask & (pattern << i))
                   == 0)
                {
                  out_mask = pattern << i;
                  out_offset = i;
                  break;
                }
            }

          if (out_mask)
            {
              dedic_gpio_common.out_occupied_mask |= out_mask;
            }

          spin_unlock_irqrestore(&dedic_gpio_common.spinlock, flags);
          if (out_mask == 0)
            {
              gpioerr("ERROR: no free outward channels\n");
              free(priv);
              return NULL;
            }

          attr |= OUTPUT;
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

          flags = spin_lock_irqsave(&dedic_gpio_common.spinlock);

          for (int i = 0;
                i <= SOC_DEDIC_GPIO_IN_CHANNELS_NUM - config->array_size;
                  i++)
            {
              if ((dedic_gpio_common.in_occupied_mask & (pattern << i)) == 0)
                {
                  in_mask = pattern << i;
                  in_offset = i;
                  break;
                }
            }

          if (in_mask)
            {
              dedic_gpio_common.in_occupied_mask |= in_mask;
            }

          spin_unlock_irqrestore(&dedic_gpio_common.spinlock, flags);
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
  dedic_gpio_periph_signals.cores[0].in_sig_per_channel[in_offset + i],
  config->flags->invert_input_enable);
            }

          if (config->flags->output_enable)
            {
              esp_gpio_matrix_out(config->gpio_array[i],
  dedic_gpio_periph_signals.cores[0].out_sig_per_channel[in_offset + i],
  config->flags->invert_output_enable, 0);
            }
        }

        dedic_gpio_cpu_ll_enable_output(dedic_gpio_common.out_occupied_mask);
        priv->out_mask = out_mask;
        priv->in_mask = in_mask;
        priv->out_offset = out_offset;
        priv->in_offset = in_offset;
    }
  else
    {
      gpioerr("ERROR: memory allocation failed\n");
      return NULL;
    }

  dedic_gpio_common.refs++;

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
  irqstate_t flags = spin_lock_irqsave(&dedic_gpio_common.spinlock);

  dedic_gpio_common.refs--;

  dedic_gpio_common.out_occupied_mask &= ~(priv->out_mask);
  dedic_gpio_common.in_occupied_mask &= ~(priv->in_mask);

  free(priv);
  priv = NULL;
  spin_unlock_irqrestore(&dedic_gpio_common.spinlock, flags);

  return OK;
}
