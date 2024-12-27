/****************************************************************************
 * drivers/safety/reg_monitor.c
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>

#include <nuttx/safety/safety.h>
#include <nuttx/safety/reg_monitor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define getreg32(a)     (*(volatile uint32_t *)(a))
#define putreg32(v,a)   (*(volatile uint32_t *)(a) = (v))
#define modreg32(v,m,a) putreg32((getreg32(a) & ~(m)) | ((v) & (m)), (a))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Private data structure for register monitor */

typedef struct
{
  FAR struct safety_lowerhalf_s lower;    /* Lower half driver structure */
  FAR const struct reg_map_s   *map;      /* Array of register maps */
  size_t                        map_num;  /* Number of register maps */
  safety_handler_t              cb;       /* Callback function */
  FAR void                     *arg;      /* Callback argument */
  bool                          inject;   /* Fault injection flag */
  uint8_t                       reg_err_array[0];
} reg_monitor_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int reg_monitor_selftest(FAR struct safety_lowerhalf_s *lower);
static int reg_monitor_inject(FAR struct safety_lowerhalf_s *lower,
                              FAR void *arg);
static int reg_monitor_setcallback(FAR struct safety_lowerhalf_s *lower,
                                   safety_handler_t callback, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct safety_ops_s g_reg_ops =
{
  .inject       = reg_monitor_inject,
  .selftest     = reg_monitor_selftest,
  .set_callback = reg_monitor_setcallback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reg_monitor_inject
 *
 * Description:
 *   Inject a fault into the register monitor for testing purposes.
 *
 * Input Parameters:
 *   lower  - Pointer to the lower half driver structure
 *   arg    - Pointer to injection parameters
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

static int reg_monitor_inject(FAR struct safety_lowerhalf_s *lower,
                              FAR void *arg)
{
  FAR reg_monitor_priv_t *priv = (FAR reg_monitor_priv_t *)lower;

  priv->inject = true;
  return OK;
}

/****************************************************************************
 * Name: reg_monitor_setcallback
 *
 * Description:
 *   Set the callback function for fault notifications.
 *
 * Input Parameters:
 *   lower    - Pointer to the lower half driver structure
 *   callback - The callback function to be called on fault detection
 *   arg      - private data to be passed through to the callback function
 *              when it's invoked.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

static int reg_monitor_setcallback(FAR struct safety_lowerhalf_s *lower,
                                   safety_handler_t callback, FAR void *arg)
{
  FAR reg_monitor_priv_t *priv = (FAR reg_monitor_priv_t *)lower;

  DEBUGASSERT(priv);

  priv->cb  = callback;
  priv->arg = arg;
  return OK;
}

static int reg_monitor_selftest(FAR struct safety_lowerhalf_s *lower)
{
  FAR reg_monitor_priv_t *priv = (FAR reg_monitor_priv_t *)lower;
  struct reg_result_s result;
  uint32_t value;
  uint8_t type;
  size_t i;

  DEBUGASSERT(priv);

  for (i = 0; i < priv->map_num; ++i)
    {
      const struct reg_map_s *reg = &priv->map[i];
      value = getreg32(reg->addr);
      type  = ACTION_NONE;

      /* Check if the current value matches the expected value */

      if (priv->inject || (((value ^ reg->value) & reg->mask) != 0))
        {
          /* If error count is‘t over the max allowed, try to recover */

          if (priv->reg_err_array[i]++ < reg->err_max_cnt)
            {
              /* Attempt to rewrite the register value */

              modreg32(reg->value, reg->mask, reg->addr);

              /* Check again to ensure the rewrite is successful */

              value = getreg32(reg->addr);

              type = ((value ^ reg->value) & reg->mask) ?
                      ACTION_REWRITE_FAIL : ACTION_REWRITE_OK;
            }
          else
            {
              type = ACTION_RESET;
            }

          if (type != ACTION_NONE && priv->cb)
            {
              result.index = i;
              result.type  = type;
              priv->cb(priv->arg, &result, i * sizeof(struct reg_result_s),
                       sizeof(struct reg_result_s));
            }
        }
    }

  priv->inject = false;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: register_monitor_init
 *
 * Description:
 *   Initialize the register monitor
 *
 ****************************************************************************/

int reg_monitor_initialize(FAR const struct reg_map_s *map, size_t map_num)
{
  FAR reg_monitor_priv_t *priv;
  int ret;

  /* the max map_num is 8191 which is 2^13 - 1, refer to reg_result_s */

  if (!map || !map_num || map_num > (1 << 13) - 1)
    {
      saerr("invalid map or map_num!\n");
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(reg_monitor_priv_t) +
                    map_num * sizeof(uint8_t));
  if (!priv)
    {
      saerr("kmm_zalloc failed for register monitor!\n");
      return -ENOMEM;
    }

  priv->map       = map;
  priv->map_num   = map_num;
  priv->lower.ops = &g_reg_ops;

  ret = safety_register(&priv->lower, SAFETY_MODULE_REG,
                        sizeof(struct reg_result_s) * map_num);
  if (ret != OK)
    {
      saerr("register register monitor failed!\n");
      kmm_free(priv);
    }

  return ret;
}
