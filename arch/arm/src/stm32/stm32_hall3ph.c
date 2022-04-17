/****************************************************************************
 * arch/arm/src/stm32/stm32_hall3ph.c
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/hall3ph.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_hall3ph.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  const struct hall3_ops_s  *ops; /* Lower half callback structure */
  struct stm32_hall3ph_cfg_s cfg; /* Configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

int stm32_hall3ph_setup(struct hall3_lowerhalf_s *lower);
int stm32_hall3ph_shutdown(struct hall3_lowerhalf_s *lower);
int stm32_hall3ph_position(struct hall3_lowerhalf_s *lower,
                           uint8_t *pos);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct hall3_ops_s g_hall3ph_ops =
{
  .setup    = stm32_hall3ph_setup,
  .shutdown = stm32_hall3ph_shutdown,
  .position = stm32_hall3ph_position
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hall3ph_setup
 *
 * Description:
 *   Configure the 3-phase Hall effect sensor
 *
 ****************************************************************************/

int stm32_hall3ph_setup(struct hall3_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  /* Configure input pins */

  stm32_configgpio(priv->cfg.gpio_pha);
  stm32_configgpio(priv->cfg.gpio_phb);
  stm32_configgpio(priv->cfg.gpio_phc);

  return OK;
}

/****************************************************************************
 * Name: stm32_hall3ph_shutdown
 *
 * Description:
 *   Disable the 3-phase Hall effect sensor
 *
 ****************************************************************************/

int stm32_hall3ph_shutdown(struct hall3_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  /* Unconfigure input pins */

  stm32_unconfiggpio(priv->cfg.gpio_pha);
  stm32_unconfiggpio(priv->cfg.gpio_phb);
  stm32_unconfiggpio(priv->cfg.gpio_phc);

  return OK;
}

/****************************************************************************
 * Name: stm32_hall3ph_position
 *
 * Description:
 *   Return the current 3-phase Hall effect sensor position
 *
 ****************************************************************************/

int stm32_hall3ph_position(struct hall3_lowerhalf_s *lower,
                           uint8_t *pos)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
  uint8_t                   ha   = 0;
  uint8_t                   hb   = 0;
  uint8_t                   hc   = 0;
  uint8_t                   i    = 0;
  uint8_t                   thr  = (priv->cfg.samples / 2);

  DEBUGASSERT(priv);

  /* Sample pins */

  for (i = 0; i < priv->cfg.samples; i += 1)
    {
      ha += stm32_gpioread(priv->cfg.gpio_pha);
      hb += stm32_gpioread(priv->cfg.gpio_phb);
      hc += stm32_gpioread(priv->cfg.gpio_phc);
    }

  /* Get state based on threshold */

  *pos = ((ha > thr) << 0) |
         ((hb > thr) << 1) |
         ((hc > thr) << 2);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hall3ph_initialize
 *
 * Description:
 *   Initialize the 3-phase Hall effect driver
 *
 ****************************************************************************/

int stm32_hall3ph_initialize(const char *devpath,
                             struct stm32_hall3ph_cfg_s *cfg)
{
  struct stm32_lowerhalf_s *priv = NULL;
  int                       ret  = OK;

  /* Allocate the lower-half data structure */

  priv = (struct stm32_lowerhalf_s *)
         kmm_zalloc(sizeof(struct stm32_lowerhalf_s));
  if (priv == NULL)
    {
      snerr("ERROR: Allocation failed\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Copy configuration */

  memcpy(&priv->cfg, cfg, sizeof(struct stm32_hall3ph_cfg_s));

  /* Connect ops */

  priv->ops = &g_hall3ph_ops;

  /* Initialize a Hall effect sensor interface. */

  ret = hall3_register(devpath, (struct hall3_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: hall3ph_register failed: %d\n", ret);
    }

errout:
  return ret;
}
