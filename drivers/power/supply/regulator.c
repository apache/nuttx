/****************************************************************************
 * drivers/power/supply/regulator.c
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

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/consumer.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int _regulator_is_enabled(FAR struct regulator_dev_s *rdev);
static int _regulator_do_enable(FAR struct regulator_dev_s *rdev);
static int _regulator_do_disable(FAR struct regulator_dev_s *rdev);
static int regulator_check_consumers(FAR struct regulator_dev_s *rdev,
                                     FAR int *min_uv, FAR int *max_uv);
static FAR struct regulator_dev_s *regulator_dev_lookup(const char *supply);
static int regulator_map_voltage_iterate(FAR struct regulator_dev_s *rdev,
                                         int min_uv, int max_uv);
static int _regulator_get_voltage(FAR struct regulator_dev_s *rdev);
static int _regulator_do_set_voltage(FAR struct regulator_dev_s *rdev,
                                     int min_uv, int max_uv);
static int _regulator_set_voltage_unlocked(FAR struct regulator_s *regulator,
                                           int min_uv, int max_uv);
static int _regulator_do_enable_pulldown(FAR struct regulator_dev_s *rdev);
static int _regulator_do_disable_pulldown(FAR struct regulator_dev_s *rdev);
static irqstate_t regulator_lock(FAR mutex_t *lock);
static void regulator_unlock(FAR mutex_t *lock, irqstate_t flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node g_reg_list = LIST_INITIAL_VALUE(g_reg_list);
static mutex_t g_reg_lock          = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int _regulator_is_enabled(FAR struct regulator_dev_s *rdev)
{
  if (!rdev->ops->is_enabled)
    {
      return 1;
    }

  return rdev->ops->is_enabled(rdev);
}

static int _regulator_do_enable(FAR struct regulator_dev_s *rdev)
{
  FAR struct regulator_s *supply = NULL;
  int ret = 0;

  if (rdev->desc->supply_name && rdev->supply == NULL)
    {
      supply = regulator_get(rdev->desc->supply_name);
      if (supply == NULL)
        {
          pwrerr("get supply %s failed \n", rdev->desc->supply_name);
          return -ENODEV;
        }

      rdev->supply = supply;
    }

  if (rdev->supply)
    {
      ret = regulator_enable(rdev->supply);
      if (ret < 0)
        {
          pwrerr("failed to enable supply %d\n", ret);
          goto err;
        }
    }

  if (rdev->ops->enable)
    {
      ret = rdev->ops->enable(rdev);
      if (ret < 0)
        {
          pwrerr("failed to enable %d\n", ret);
          if (rdev->supply)
            {
              regulator_disable(rdev->supply);
            }

          goto err;
        }
    }

  if (rdev->desc->enable_time > 0)
    {
      up_udelay(rdev->desc->enable_time);
    }

  return ret;

err:
  if (supply)
    {
      regulator_put(supply);
      rdev->supply = NULL;
    }

  return ret;
}

static int _regulator_do_disable(FAR struct regulator_dev_s *rdev)
{
  int ret = 0;

  if (rdev->ops->disable)
    {
      ret = rdev->ops->disable(rdev);
      if (ret < 0)
        {
          pwrerr("failed to disable %d\n", ret);
          return ret;
        }
    }

  if (rdev->supply)
    {
      ret = regulator_disable(rdev->supply);
      if (ret < 0)
        {
          pwrerr("failed to disable supply %d\n", ret);
          if (rdev->ops->enable)
            {
              rdev->ops->enable(rdev);
            }
        }
    }

  return ret;
}

static int regulator_check_consumers(FAR struct regulator_dev_s *rdev,
                                     FAR int *min_uv, FAR int *max_uv)
{
  FAR struct regulator_s *regulator;

  list_for_every_entry(&rdev->consumer_list, regulator,
                       struct regulator_s, list)
    {
      if (!regulator->min_uv && !regulator->max_uv)
        {
          continue;
        }

      if (*max_uv > regulator->max_uv)
        {
          *max_uv = regulator->max_uv;
        }

      if (*min_uv < regulator->min_uv)
        {
          *min_uv = regulator->min_uv;
        }
    }

  if (*min_uv > *max_uv)
    {
      pwrerr("Restricting voltage, %d-%d uv\n", *min_uv, *max_uv);
      return -EINVAL;
    }

  return 0;
}

static FAR struct regulator_dev_s *regulator_dev_lookup(const char *supply)
{
  irqstate_t flags;
  FAR struct regulator_dev_s *rdev;
  FAR struct regulator_dev_s *rdev_found = NULL;

  flags = regulator_lock(&g_reg_lock);
  list_for_every_entry(&g_reg_list, rdev, struct regulator_dev_s, list)
    {
      if (rdev->desc->name && strcmp(rdev->desc->name, supply) == 0)
        {
          rdev_found = rdev;
          break;
        }
    }

  regulator_unlock(&g_reg_lock, flags);
  return rdev_found;
}

static int regulator_map_voltage_iterate(FAR struct regulator_dev_s *rdev,
                                         int min_uv, int max_uv)
{
  int best_val = INT_MAX;
  int selector = 0;
  int i;
  int ret;

  for (i = 0; i < rdev->desc->n_voltages; i++)
    {
      ret = rdev->ops->list_voltage(rdev, i);
      if (ret < 0)
        {
          continue;
        }

      if (ret < best_val && ret >= min_uv && ret <= max_uv)
        {
          best_val = ret;
          selector = i;
        }
    }

  if (best_val != INT_MAX)
    {
      return selector;
    }
  else
    {
      return -EINVAL;
    }
}

static int _regulator_get_voltage(FAR struct regulator_dev_s *rdev)
{
  int sel;
  int ret;

  if (rdev->ops->get_voltage_sel)
    {
      sel = rdev->ops->get_voltage_sel(rdev);
      if (sel < 0)
        {
          return sel;
        }

      ret = rdev->ops->list_voltage(rdev, sel);
    }
  else if (rdev->ops->get_voltage)
    {
      ret = rdev->ops->get_voltage(rdev);
    }
  else if (rdev->ops->list_voltage)
    {
      ret = rdev->ops->list_voltage(rdev, 0);
    }
  else
    {
      return -EINVAL;
    }

  return ret;
}

static int _regulator_do_enable_pulldown(FAR struct regulator_dev_s *rdev)
{
  int ret = 0;

  if (rdev->ops->enable_pulldown)
    {
      ret = rdev->ops->enable_pulldown(rdev);
      if (ret < 0)
        {
          pwrerr("failed to get enable pulldown\n");
        }
    }

  return ret;
}

static int _regulator_do_disable_pulldown(FAR struct regulator_dev_s *rdev)
{
  int ret = 0;

  if (rdev->ops->disable_pulldown)
    {
      ret = rdev->ops->disable_pulldown(rdev);
      if (ret < 0)
        {
          pwrerr("failed to get disable pulldown\n");
        }
    }

  return ret;
}

static int _regulator_do_set_voltage(FAR struct regulator_dev_s *rdev,
                                     int min_uv, int max_uv)
{
  FAR const struct regulator_ops_s *ops = rdev->ops;
  unsigned int selector;
  int new_uv = 0;
  int old_uv = _regulator_get_voltage(rdev);
  int ret = 0;
  int delay = 0;
  int best_val;

  if (ops->set_voltage)
    {
      ret = ops->set_voltage(rdev, min_uv, max_uv, &selector);
      if (ret >= 0)
        {
          if (ops->list_voltage)
            {
              new_uv = ops->list_voltage(rdev, selector);
            }
          else
            {
              new_uv = _regulator_get_voltage(rdev);
            }
        }
    }
  else if (ops->set_voltage_sel)
    {
      ret = regulator_map_voltage_iterate(rdev, min_uv, max_uv);
      if (ret >= 0)
        {
          best_val = ops->list_voltage(rdev, ret);
          if (min_uv <= best_val && max_uv >= best_val)
            {
              selector = ret;
              ret = ops->set_voltage_sel(rdev, selector);
            }
        }
      else
        {
          ret = -EINVAL;
        }
    }
  else
    {
      ret = -EINVAL;
    }

  if (ret < 0)
    {
      return ret;
    }

  if (rdev->desc->ramp_delay)
    {
      delay = abs(new_uv - old_uv) / rdev->desc->ramp_delay + 1;
    }

  up_udelay(delay);

  return ret;
}

static int _regulator_set_voltage_unlocked(FAR struct regulator_s *regulator,
                                           int min_uv, int max_uv)
{
  FAR struct regulator_dev_s *rdev = regulator->rdev;
  FAR const struct regulator_ops_s *ops = rdev->ops;
  int old_min_uv;
  int old_max_uv;
  int ret = 0;

  if (min_uv > max_uv)
    {
      pwrerr("invalid min %d max %d\n", min_uv, max_uv);
      return -EINVAL;
    }

  if (regulator->min_uv == min_uv && regulator->max_uv == max_uv)
    {
      goto out;
    }

  if (!ops->set_voltage && !ops->set_voltage_sel)
    {
      pwrerr("set voltage is null\n");
      ret = -EINVAL;
      goto out;
    }

  if (max_uv > rdev->desc->max_uv)
    {
      max_uv = rdev->desc->max_uv;
    }

  if (min_uv < rdev->desc->min_uv)
    {
      min_uv = rdev->desc->min_uv;
    }

  if (min_uv > max_uv)
    {
      pwrerr("invalid min %d max %d\n", min_uv, max_uv);
      ret = -EINVAL;
      goto out;
    }

  old_min_uv = regulator->min_uv;
  old_max_uv = regulator->max_uv;
  regulator->min_uv = min_uv;
  regulator->max_uv = max_uv;

  ret = regulator_check_consumers(rdev, &min_uv, &max_uv);
  if (ret < 0)
    {
      goto out2;
    }

  ret = _regulator_do_set_voltage(rdev, min_uv, max_uv);
  if (ret < 0)
    {
      goto out2;
    }

out:
  return ret;

out2:
  regulator->min_uv = old_min_uv;
  regulator->max_uv = old_max_uv;

  return ret;
}

#ifdef CONFIG_PM
static void regulator_pm_notify(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate)
{
  FAR struct regulator_dev_s *rdev = NULL;
  FAR const struct regulator_state_s *state = NULL;

  rdev = container_of(cb, struct regulator_dev_s, pm_cb);

  if (rdev->desc->domain != domain)
    {
      return;
    }

  switch (pmstate)
    {
      case PM_RESTORE:
        if (rdev->ops->resume)
          {
            rdev->ops->resume(rdev);
          }
        break;

      case PM_NORMAL:
        state = &rdev->desc->states[PM_NORMAL];
        break;

      case PM_IDLE:
        state = &rdev->desc->states[PM_IDLE];
        break;

      case PM_STANDBY:
        state = &rdev->desc->states[PM_STANDBY];
        break;

      case PM_SLEEP:
        state = &rdev->desc->states[PM_SLEEP];
        break;

      default:
        break;
    }

  if (state)
    {
      if (rdev->ops->set_suspend_voltage && state->uv > 0)
        {
          rdev->ops->set_suspend_voltage(rdev, state->uv);
        }

      if (rdev->ops->set_suspend_mode &&
          state->mode != REGULATOR_MODE_INVALID)
        {
          rdev->ops->set_suspend_mode(rdev, state->mode);
        }
    }
}

#endif

static irqstate_t regulator_lock(FAR mutex_t *lock)
{
  if (!up_interrupt_context() && !sched_idletask())
    {
      nxmutex_lock(lock);
    }

  return enter_critical_section();
}

static void regulator_unlock(FAR mutex_t *lock, irqstate_t flags)
{
  leave_critical_section(flags);

  if (!up_interrupt_context() && !sched_idletask())
    {
      nxmutex_unlock(lock);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: regulator_get
 *
 * Description:
 *   Lookup and obtain a reference to a regulator.
 *
 * Input parameters:
 *   id - Supply name or the regulator ID.
 *
 * Returned value:
 *    A struct regulator_s pointer on success or NULL on failure
 *
 ****************************************************************************/

FAR struct regulator_s *regulator_get(FAR const char *id)
{
  irqstate_t flags;
  FAR struct regulator_dev_s *rdev;
  FAR struct regulator_s *regulator = NULL;

  if (id == NULL)
    {
      pwrerr("get() with no identifier\n");
      return NULL;
    }

  rdev = regulator_dev_lookup(id);

#if defined(CONFIG_REGULATOR_RPMSG)
  if (rdev == NULL && strchr(id, '/'))
    {
      rdev = regulator_rpmsg_get(id);
    }
#endif

  if (rdev == NULL)
    {
      pwrerr("regulator %s not found\n", id);
      return NULL;
    }

  regulator = kmm_zalloc(sizeof(struct regulator_s));
  if (regulator == NULL)
    {
      pwrerr("failed to get memory\n");
      return NULL;
    }

  regulator->rdev = rdev;
  list_initialize(&regulator->list);

  flags = regulator_lock(&rdev->regulator_lock);
  rdev->open_count++;
  list_add_tail(&rdev->consumer_list, &regulator->list);
  regulator_unlock(&rdev->regulator_lock, flags);

  return regulator;
}

/****************************************************************************
 * Name: regulator_put
 *
 * Description:
 *   Free the regulator resource.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *
 ****************************************************************************/

void regulator_put(FAR struct regulator_s *regulator)
{
  FAR struct regulator_dev_s *rdev;
  irqstate_t flags;

  if (regulator == NULL)
    {
      return;
    }

  rdev = regulator->rdev;
  flags = regulator_lock(&rdev->regulator_lock);
  list_delete(&regulator->list);
  rdev->open_count--;
  regulator_unlock(&rdev->regulator_lock, flags);

  kmm_free(regulator);
}

/****************************************************************************
 * Name: regulator_is_enabled
 *
 * Description:
 *   Is the regulator output enabled.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   1 is enabled and zero for disabled.
 *
 ****************************************************************************/

int regulator_is_enabled(FAR struct regulator_s *regulator)
{
  FAR struct regulator_dev_s *rdev;
  irqstate_t flags;
  int ret = 0;

  if (regulator == NULL)
    {
      pwrerr("regulator is null\n");
      return -EINVAL;
    }

  rdev = regulator->rdev;

  if (rdev->desc->always_on)
    {
      return 1;
    }

  flags = regulator_lock(&rdev->regulator_lock);
  ret = _regulator_is_enabled(rdev);
  regulator_unlock(&rdev->regulator_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: regulator_enable
 *
 * Description:
 *   Enable the regulator output.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_enable(FAR struct regulator_s *regulator)
{
  FAR struct regulator_dev_s *rdev;
  irqstate_t flags;
  int ret = 0;

  if (regulator == NULL)
    {
      pwrerr("enable regulator is null\n");
      return -EINVAL;
    }

  rdev = regulator->rdev;

  flags = regulator_lock(&rdev->regulator_lock);
  if (rdev->use_count == 0 && !rdev->desc->always_on)
    {
      ret = _regulator_do_enable(rdev);
      if (ret < 0)
        {
          goto err;
        }
    }

  rdev->use_count++;

err:
  regulator_unlock(&rdev->regulator_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: regulator_enable_delay
 *
 * Description:
 *   Enable the regulator output.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *   ms        - The delay ms after regulator enable
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_enable_delay(FAR struct regulator_s *regulator, int ms)
{
  int ret;

  ret = regulator_enable(regulator);
  if (!ret)
    {
      nxsig_usleep(1000 * ms);
    }

  return ret;
}

/****************************************************************************
 * Name: regulator_disable
 *
 * Description:
 *   Disable the regulator output.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_disable(FAR struct regulator_s *regulator)
{
  FAR struct regulator_dev_s *rdev;
  irqstate_t flags;
  int ret = 0;

  if (regulator == NULL)
    {
      pwrerr("disable regulator is null\n");
      return -EINVAL;
    }

  rdev = regulator->rdev;

  flags = regulator_lock(&rdev->regulator_lock);
  if (rdev->use_count <= 0)
    {
      ret = -EIO;
      goto err;
    }

  if (rdev->use_count == 1 && !rdev->desc->always_on)
    {
      ret = _regulator_do_disable(rdev);
      if (ret < 0)
        {
          goto err;
        }
    }

  rdev->use_count--;

err:
  regulator_unlock(&rdev->regulator_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: regulator_disable_deferred
 *
 * Description:
 *   Disable the regulator after ms.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *   ms        - The delay ms before disable regulator
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_disable_deferred(FAR struct regulator_s *regulator, int ms)
{
  if (!regulator)
    {
      return -EINVAL;
    }

  return work_queue(LPWORK, (FAR struct work_s *)&regulator,
                   (worker_t)regulator_disable, regulator, MSEC2TICK(ms));
}

/****************************************************************************
 * Name: regulator_set_voltage
 *
 * Description:
 *   Set the regulator output voltage.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *   min_uv - Minimum required voltage in uv
 *   max_uv - Maximum acceptable voltage in uv
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_set_voltage(FAR struct regulator_s *regulator,
                          int min_uv, int max_uv)
{
  FAR struct regulator_dev_s *rdev;
  irqstate_t flags;
  int ret = 0;

  if (regulator == NULL)
    {
      pwrerr("get regulator is null\n");
      return -EINVAL;
    }

  rdev = regulator->rdev;

  flags = regulator_lock(&rdev->regulator_lock);
  ret = _regulator_set_voltage_unlocked(regulator, min_uv, max_uv);
  regulator_unlock(&rdev->regulator_lock, flags);

  return ret;
}

/****************************************************************************
 * Name: regulator_get_voltage
 *
 * Description:
 *   Obtain the regulator output voltage.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Positive on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_get_voltage(FAR struct regulator_s *regulator)
{
  FAR struct regulator_dev_s *rdev;
  irqstate_t flags;
  int ret = 0;

  if (regulator == NULL)
    {
      pwrerr("get regulator is null\n");
      return -EINVAL;
    }

  rdev = regulator->rdev;

  flags = regulator_lock(&rdev->regulator_lock);
  ret = _regulator_get_voltage(rdev);
  regulator_unlock(&rdev->regulator_lock, flags);

  return ret;
}

/****************************************************************************
 * Name: regulator_set_mode
 *
 * Description:
 * Set regulator operating mode to increase regulator efficiency or improve
 * regulation performance.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *   mode - operating mode - one of the REGULATOR_MODE constants
 *
 * Returned value:
 *   Positive on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_set_mode(FAR struct regulator_s *regulator,
                       enum regulator_mode_e mode)
{
  FAR struct regulator_dev_s *rdev = regulator->rdev;
  unsigned int curr_mode;
  irqstate_t flags;
  int ret;

  flags = regulator_lock(&rdev->regulator_lock);
  if (!rdev->ops->set_mode || mode == REGULATOR_MODE_INVALID)
    {
      ret = -EINVAL;
      goto out;
    }

  if (rdev->ops->get_mode)
    {
      curr_mode = rdev->ops->get_mode(rdev);
      if (curr_mode == mode)
        {
          ret = 0;
          goto out;
        }
    }

  ret = rdev->ops->set_mode(rdev, mode);
out:
  regulator_unlock(&rdev->regulator_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: regulator_register
 *
 * Description:
 *   This routine is called by the specific regulator drivers to register a
 *   regulator.
 *
 ****************************************************************************/

FAR struct regulator_dev_s *
regulator_register(FAR const struct regulator_desc_s *regulator_desc,
                   FAR const struct regulator_ops_s *regulator_ops,
                   FAR void *priv)
{
  FAR struct regulator_dev_s *rdev;
  irqstate_t flags;
  int ret = 0;

  if (regulator_desc == NULL)
    {
      pwrerr("regulator desc is null\n");
      return NULL;
    }

  if (regulator_desc->name == NULL || regulator_ops == NULL)
    {
      pwrerr("regulator name or ops is null\n");
      return NULL;
    }

  if (regulator_dev_lookup(regulator_desc->name))
    {
      pwrerr("regulator name is registered\n");
      return NULL;
    }

  if (regulator_ops->get_voltage && regulator_ops->get_voltage_sel)
    {
      pwrerr("get_voltage and get_voltage_sel are both assigned\n");
      return NULL;
    }

  if (regulator_ops->set_voltage && regulator_ops->set_voltage_sel)
    {
      pwrerr("set_voltage and set_voltage_sel are both assigned\n");
      return NULL;
    }

  if (regulator_ops->get_voltage_sel && !regulator_ops->list_voltage)
    {
      pwrerr("list voltage is null\n");
      return NULL;
    }

  if (regulator_ops->set_voltage_sel && !regulator_ops->list_voltage)
    {
      pwrerr("list voltage is null\n");
      return NULL;
    }

  rdev = kmm_zalloc(sizeof(struct regulator_dev_s));
  if (rdev == NULL)
    {
      pwrerr("failed to get memory\n");
      return NULL;
    }

  rdev->desc = regulator_desc;
  rdev->ops = regulator_ops;
  rdev->priv = priv;
  nxmutex_init(&rdev->regulator_lock);
  list_initialize(&rdev->consumer_list);
  list_initialize(&rdev->list);

  if (rdev->desc->boot_on || rdev->desc->always_on)
    {
      ret = _regulator_do_enable(rdev);
      if (ret < 0)
        {
          pwrerr("failed to enable regulator\n");
          kmm_free(rdev);
          return NULL;
        }
    }
  else if (!rdev->desc->boot_on && !rdev->desc->always_on
           && _regulator_is_enabled(rdev))
    {
      _regulator_do_disable(rdev);
    }

  if (rdev->desc->apply_uv)
    {
      _regulator_do_set_voltage(rdev, rdev->desc->min_uv,
                                rdev->desc->max_uv);
    }

  if (rdev->desc->pulldown)
    {
      _regulator_do_enable_pulldown(rdev);
    }
  else
    {
      _regulator_do_disable_pulldown(rdev);
    }

#ifdef CONFIG_PM
  if (rdev->desc->auto_lp)
    {
      rdev->pm_cb.prepare = NULL;
      rdev->pm_cb.notify = regulator_pm_notify;
      pm_register(&rdev->pm_cb);
    }
#endif

  flags = regulator_lock(&g_reg_lock);
  list_add_tail(&g_reg_list, &rdev->list);
  regulator_unlock(&g_reg_lock, flags);

  return rdev;
}

/****************************************************************************
 * Name: regulator_unregister
 *
 * Description:
 *   This routine is called by the specific regulator drivers to unregister a
 *   regulator.
 *
 ****************************************************************************/

void regulator_unregister(FAR struct regulator_dev_s *rdev)
{
  irqstate_t flags;

  if (rdev == NULL)
    {
      return;
    }

  flags = regulator_lock(&g_reg_lock);
  if (rdev->open_count)
    {
      pwrerr("unregister, open %" PRIu32 "\n", rdev->open_count);
      regulator_unlock(&g_reg_lock, flags);
      return;
    }

  list_delete(&rdev->list);
  regulator_unlock(&g_reg_lock, flags);
#ifdef CONFIG_PM
  if (rdev->desc->auto_lp)
    {
      pm_unregister(&rdev->pm_cb);
    }
#endif

  if (rdev->supply)
    {
      regulator_put(rdev->supply);
    }

  kmm_free(rdev);
}
