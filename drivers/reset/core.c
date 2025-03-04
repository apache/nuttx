/****************************************************************************
 * drivers/reset/core.c
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

#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/reset/reset.h>
#include <nuttx/reset/reset-controller.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node
g_reset_controller_list = LIST_INITIAL_VALUE(g_reset_controller_list);
static mutex_t g_reset_list_mutex = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* struct reset_control_array - an array of reset controls
 * @base: reset control for compatibility with reset control API functions
 * @num_rstcs: number of reset controls
 * @rstc: array of reset controls
 */

struct reset_control_array
{
  struct reset_control base;
  unsigned int num_rstcs;
  FAR struct reset_control *rstc[];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR struct reset_control_array *
rstc_to_array(FAR struct reset_control *rstc)
{
  return container_of(rstc, struct reset_control_array, base);
}

/****************************************************************************
 * Name: reset_control_array_reset
 *
 * Description:
 *   This function is used to perform reset operation of a reset control
 *   array.
 *
 * Input Parameters:
 *   resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, otherwise error.
 ****************************************************************************/

static int reset_control_array_reset(FAR struct reset_control_array *resets)
{
  int ret = 0;
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_reset(resets->rstc[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_assert
 *
 * Description:
 *   This function is used to perform assert operation of a reset control
 *   array.
 *
 * Input Parameters:
 *    resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, otherwise error.
 ****************************************************************************/

static int reset_control_array_assert(FAR struct reset_control_array *resets)
{
  int ret = 0;
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_assert(resets->rstc[i]);
      if (ret < 0)
        {
          while (i--)
            {
              reset_control_deassert(resets->rstc[i]);
            }

          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_deassert
 *
 * Description:
 *   This function is used to perform deassert operation of a reset control
 *   array.
 *
 * Input Parameters:
 *   resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, an negative error code otherwise.
 ****************************************************************************/

static int
reset_control_array_deassert(FAR struct reset_control_array *resets)
{
  int ret = 0;
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_deassert(resets->rstc[i]);
      if (ret < 0)
        {
          while (i--)
            {
              reset_control_assert(resets->rstc[i]);
            }

          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_acquire
 *
 * Description:
 *   This function is used to perform acquire operation of a reset control
 *   array.
 *
 * Input Parameters:
 *    resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, an negative error code otherwise.
 ****************************************************************************/

static int
reset_control_array_acquire(FAR struct reset_control_array *resets)
{
  unsigned int i;
  unsigned int ret = 0;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_acquire(resets->rstc[i]);
      if (ret < 0)
        {
          while (i--)
            {
              reset_control_release(resets->rstc[i]);
            }

          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_release
 *
 * Description:
 *   This function is used to perform release operation of a reset control
 *   array.
 *
 * Input Parameters:
 *   resets - An instance of a reset_control_array.
 ****************************************************************************/

static void
reset_control_array_release(FAR struct reset_control_array *resets)
{
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      reset_control_release(resets->rstc[i]);
    }
}

/****************************************************************************
 * Name: reset_control_get_internal
 *
 * Description:
 *   This function is used to return a reset_control by rcdev,index,shared,
 *   acquired parameters.
 *
 * Input Parameters:
 *   rcdev    - An instance of reset_controller_dev type.
 *   index    - ID of the reset controller in the reset controller device.
 *   shared   - Is this a shared (1), or an exclusive (0) reset_control.
 *   acquired - Only one reset_control may be acquired for a given rcdev and
 *   index.
 *
 * Returned Value:
 *   Return reset_control if success, others NULL.
 ****************************************************************************/

static FAR struct reset_control *
reset_control_get_internal(FAR struct reset_controller_dev *rcdev,
                           unsigned int index, bool shared, bool acquired)
{
  FAR struct reset_control *rstc;

  DEBUGASSERT(nxmutex_is_locked(&g_reset_list_mutex));

  list_for_every_entry(&rcdev->reset_control_head, rstc,
                       struct reset_control, list)
    {
      if (rstc->id == index)
        {
          /* Allow creating a secondary exclusive reset_control
           * that is initially not acquired for an already
           * controlled reset line.
           */

          if (!rstc->shared && !shared && !acquired)
            {
              break;
            }

          /* shared reset controller */

          if (!rstc->shared || !shared)
            {
              rsterr("not shared reset control\n");
              return NULL;
            }

          atomic_fetch_add(&rstc->refcnt, 1);
          return rstc;
        }
    }

  rstc = kmm_zalloc(sizeof(*rstc));
  if (!rstc)
    {
      return NULL;
    }

#if defined(CONFIG_RESET_RPMSG)

  /* Only client defines this function */

  if (rcdev->ops->acquire)
    {
      int ret = rcdev->ops->acquire(rcdev, index, shared, acquired);

      if (ret < 0)
        {
          kmm_free(rstc);
          return NULL;
        }
    }
#endif

  rstc->rcdev = rcdev;
  list_add_after(&rcdev->reset_control_head, &rstc->list);
  rstc->id = index;
  atomic_set(&rstc->refcnt, 1);
  rstc->acquired = acquired;
  rstc->shared = shared;

  return rstc;
}

/****************************************************************************
 * Name: reset_control_put_internal
 *
 * Description:
 *   This is used to free a reset_control getted by reset_control_get.
 *
 * Input Parameters:
 *   rstc - An reset control
 *
 ****************************************************************************/

static void reset_control_put_internal(FAR struct reset_control *rstc)
{
  DEBUGASSERT(nxmutex_is_locked(&g_reset_list_mutex));

  if (atomic_fetch_sub(&rstc->refcnt, 1) == 1)
    {
      DEBUGASSERT(nxmutex_is_locked(&g_reset_list_mutex));
      list_delete(&rstc->list);

#if defined(CONFIG_RESET_RPMSG)

      /* Only client defines this function */

      if (rstc->rcdev->ops->release)
        {
          rstc->rcdev->ops->release(rstc->rcdev, rstc->id);
        }
#endif

      kmm_free(rstc);
    }
}

/****************************************************************************
 * Name: reset_control_array_put
 *
 * Description:
 *   This function is used to perform a free operation of reset
 *   control array.
 *
 * Input Parameters:
 *   resets - An instance of reset_control_array.
 *
 ****************************************************************************/

static void reset_control_array_put(FAR struct reset_control_array *resets)
{
  unsigned int i;

  nxmutex_lock(&g_reset_list_mutex);
  for (i = 0; i < resets->num_rstcs; i++)
    {
      reset_control_put_internal(resets->rstc[i]);
    }

  nxmutex_unlock(&g_reset_list_mutex);
  kmm_free(resets);
}

/****************************************************************************
 * Name: reset_controller_get_by_name
 *
 * Description:
 *   This function is used to get a reset_controller_dev instance register
 *   by name.
 *
 * Input Parameters:
 *   name - An name of reset_controller_dev var register.
 *
 * Returned Value:
 *   Return reset_controller_dev if success, others failed.
 ****************************************************************************/

static FAR struct reset_controller_dev *
reset_controller_get_by_name(FAR const char *name)
{
  FAR struct reset_controller_dev *rcdev;

  nxmutex_lock(&g_reset_list_mutex);
  list_for_every_entry(&g_reset_controller_list, rcdev,
                       struct reset_controller_dev, list)
  {
    if (!strcmp(name, rcdev->name))
      {
        nxmutex_unlock(&g_reset_list_mutex);
        return rcdev;
      }
  }

  nxmutex_unlock(&g_reset_list_mutex);

#if defined(CONFIG_RESET_RPMSG)
  if (strchr(name, '/'))
    {
      return reset_rpmsg_get(name);
    }
#endif

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reset_control_get
 *
 * Description:
 *   This function is used to get a reset control by reset controller name.
 *
 *   Firstly, get a reset controller device from list, and then call
 *   reset_control_get_internal function by index, shared or acquired
 *   parameters return a reset control.
 *
 * Input Parameters:
 *   name     - The reset controller name
 *   index    - The reset controller in reset controller device
 *   shared   - Is this a shared (1), or an exclusive (0) reset_control
 *   acquired - Flags that used to get a exculsive reset control
 *
 * Returned Value:
 *   Return reset_control if success, others return NULL if failed
 ****************************************************************************/

FAR struct reset_control *
reset_control_get(FAR const char *name, int index, bool shared,
                  bool acquired)
{
  FAR struct reset_control *rstc;
  FAR struct reset_controller_dev *rcdev;

  if (name == NULL)
    {
      return NULL;
    }

  /* ID of the reset controller in the reset controller device */

  if (index < 0)
    {
      rsterr("ID of the reset controller is invalid\n");
      return NULL;
    }

  if (shared && acquired)
    {
      rsterr("shared && acquired exist meanwhile\n");
      return NULL;
    }

  rcdev = reset_controller_get_by_name(name);
  if (!rcdev)
    {
      return NULL;
    }

  nxmutex_lock(&g_reset_list_mutex);

  /* g_reset_list_mutex also protects the rcdev's reset_control list */

  rstc = reset_control_get_internal(rcdev, index, shared, acquired);
  nxmutex_unlock(&g_reset_list_mutex);

  return rstc;
}

/****************************************************************************
 * Name: reset_control_reset
 *
 * Description:
 *   On a shared reset line the actual reset pulse is only triggered once for
 *   the lifetime of the reset_control instance: for all but the first caller
 *   this is a no-op.
 *   Consumers must not use reset_control_(de)assert on shared reset lines
 *   when reset_control_reset has been used.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_reset(FAR struct reset_control *rstc)
{
  int ret;

  rstinfo("Enter: reset_control_reset\n");

  if (rstc == NULL)
    {
      rsterr("rstc is null\n");
      return -EINVAL;
    }

  if (rstc->array)
    {
      return reset_control_array_reset(rstc_to_array(rstc));
    }

  if (!rstc->rcdev->ops->reset)
    {
      rsterr("rstc callback is null\n");
      return -ENOTSUP;
    }

  if (rstc->shared)
    {
      if (atomic_read(&rstc->deassert_count) != 0)
        {
          return -EINVAL;
        }

      if (atomic_fetch_add(&rstc->triggered_count, 1) != 0)
        {
          return 0;
        }
    }
  else
    {
      if (!rstc->acquired)
        {
          return -EPERM;
        }
    }

  ret = rstc->rcdev->ops->reset(rstc->rcdev, rstc->id);

  /* shared:1 and reset failed, triggered_count subtract 1 */

  if (rstc->shared && ret < 0)
    {
      atomic_fetch_sub(&rstc->triggered_count, 1);
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_assert
 *
 * Description:
 *   Asserts the reset line.
 *
 *   Calling this on an exclusive reset controller guarantees that the reset
 *   will be asserted. When called on a shared reset controller the line may
 *   still be deasserted, as long as other users keep it so.
 *
 *   For shared reset controls a driver cannot expect the hw's registers and
 *   internal state to be reset, but must be prepared for this to happen.
 *   Consumers must not use reset_control_reset on shared reset lines when
 *   reset_control_(de)assert has been used.
 *   return -EBUSY.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_assert(FAR struct reset_control *rstc)
{
  rstinfo("Enter: reset_control_assert\n");
  if (rstc == NULL)
    {
      rsterr("rstc is null\n");
      return -EINVAL;
    }

  if (rstc->array)
    {
      return reset_control_array_assert(rstc_to_array(rstc));
    }

  if (rstc->shared)
    {
      if (atomic_read(&rstc->triggered_count) != 0)
        {
          return -EINVAL;
        }

      if (atomic_read(&rstc->deassert_count) == 0)
        {
          rsterr("deassert_count = 0, invalid value\n");
          return -EINVAL;
        }

      if (atomic_fetch_sub(&rstc->deassert_count, 1) != 1)
        {
          return 0;
        }

      /* Shared reset controls allow the reset line to be in any state
       * after this call, so doing nothing is a valid option.
       */

      if (!rstc->rcdev->ops->assert)
        {
          return -EBUSY;
        }
    }
  else
    {
      /* If the reset controller does not implement .assert(), there
       * is no way to guarantee that the reset line is asserted after
       * this call.
       */

      if (!rstc->rcdev->ops->assert)
        {
          return -ENOTSUP;
        }

      if (!rstc->acquired)
        {
           rsterr("reset %s (ID: %u) is not acquired\n",
               rstc->rcdev->name, rstc->id);
          return -EPERM;
        }
    }

#undef assert
  return rstc->rcdev->ops->assert(rstc->rcdev, rstc->id);
}

/****************************************************************************
 * Name: reset_control_deassert
 *
 * Description:
 *   Deasserts the reset line.
 *   After calling this function, the reset is guaranteed to be deasserted.
 *   Consumers must not use reset_control_reset on shared reset lines when
 *   reset_control_(de)assert has been used.
 *   return -EBUSY.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 *
 ****************************************************************************/

int reset_control_deassert(FAR struct reset_control *rstc)
{
  rstinfo("Enter: reset_control_deassert\n");
  if (!rstc)
    {
      rsterr("rstc is null\n");
      return -EINVAL;
    }

  if (rstc->array)
    {
      return reset_control_array_deassert(rstc_to_array(rstc));
    }

  if (rstc->shared)
    {
      if (atomic_read(&rstc->triggered_count) != 0)
        {
          rsterr("triggered_count != 0, invalid value\n");
          return -EINVAL;
        }

      if (atomic_fetch_add(&rstc->deassert_count, 1) != 0)
        {
          return 0;
        }
    }
  else
    {
      if (!rstc->acquired)
        {
          rsterr("reset %s (ID: %u) is not acquired\n",
               rstc->rcdev->name, rstc->id);
          return -EPERM;
        }
    }

  /* If the reset controller does not implement .deassert(), we assume
   * that it handles self-deasserting reset lines via .reset(). In that
   * case, the reset lines are deasserted by default. If that is not the
   * case, the reset controller driver should implement .deassert() and
   * return -ENOTSUP.
   */

  if (!rstc->rcdev->ops->deassert)
    {
      return -ESRCH ;
    }

  return rstc->rcdev->ops->deassert(rstc->rcdev, rstc->id);
}

/****************************************************************************
 * Name: reset_control_status
 *
 * Description:
 *   Get the reset line status.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_status(FAR struct reset_control *rstc)
{
  if (rstc == NULL)
    {
      return -EINVAL;
    }

  if (rstc->array)
    {
      return -EINVAL;
    }

  if (rstc->rcdev->ops->status)
    {
      return rstc->rcdev->ops->status(rstc->rcdev, rstc->id);
    }

  return -ENOTSUP;
}

/****************************************************************************
 * Name: reset_control_acquire()
 *
 * Description:
 *   Acquires a reset control for exclusive use.
 *   This is used to explicitly acquire a reset control for exclusive use.
 *   Note that exclusive resets are requested as acquired by default.
 *   In order for a second consumer to be able to control the reset, the
 *   first consumer has to release it first.
 *   Typically the easiest way to achieve this is to call the
 *   reset_control_get_exclusive_released() to obtain an instance of the
 *   reset control. Such reset controls are not acquired by default.
 *
 *   Consumers implementing shared access to an exclusive reset need to
 *   follow a specific protocol in order to work together. Before consumers
 *   can change a reset they must acquire exclusive access using
 *   reset_control_acquire().
 *   After they are done operating the reset, they must release exclusive
 *   access with a call to reset_control_release(). Consumers are not
 *   granted exclusive access to the reset as long as another consumer
 *   hasn't released a reset.
 *   See also: reset_control_release()
 *
 * Input Parameters:
 *   rstc - Reset control
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_acquire(FAR struct reset_control *rstc)
{
  FAR struct reset_control *rc;

  if (rstc == NULL)
    {
      return 0;
    }

  if (rstc->array)
    {
      return reset_control_array_acquire(rstc_to_array(rstc));
    }

  nxmutex_lock(&g_reset_list_mutex);

  if (rstc->acquired)
    {
      nxmutex_unlock(&g_reset_list_mutex);
      return 0;
    }

  list_for_every_entry(&rstc->rcdev->reset_control_head, rc,
                        struct reset_control, list)
    {
      if (rstc != rc && rstc->id == rc->id)
        {
          if (rc->acquired)
            {
              nxmutex_unlock(&g_reset_list_mutex);
              return -EBUSY;
            }
        }
    }

  rstc->acquired = true;

  nxmutex_unlock(&g_reset_list_mutex);
  return 0;
}

/****************************************************************************
 * Name: reset_control_release()
 *
 * Discription:
 *   Releases exclusive access to a reset control.
 *
 *   Releases exclusive access right to a reset control previously obtained
 *   by a call to reset_control_acquire(). Until a consumer calls this
 *   function, no other consumers will be granted exclusive access.
 *
 *   See also: reset_control_acquire()
 *
 * Input Parameters:
 *   rstc - Reset control
 ****************************************************************************/

void reset_control_release(FAR struct reset_control *rstc)
{
  if (rstc == NULL)
    {
      return;
    }

  if (rstc->array)
    {
      reset_control_array_release(rstc_to_array(rstc));
    }
  else
    {
      rstc->acquired = false;
    }
}

/****************************************************************************
 * Name: reset_control_put
 *
 * Description:
 *   Free the reset control.
 *
 * Input Parameters:
 *   rstc - Reset controller
 ****************************************************************************/

void reset_control_put(FAR struct reset_control *rstc)
{
  if (!rstc)
    {
      return;
    }

  if (rstc->array)
    {
      reset_control_array_put(rstc_to_array(rstc));
      return;
    }

  nxmutex_lock(&g_reset_list_mutex);
  reset_control_put_internal(rstc);
  nxmutex_unlock(&g_reset_list_mutex);
}

/****************************************************************************
 * Name: reset_control_array_get
 *
 * Description:
 *   Get a list of reset controls using device node.
 *
 * Input Parameters:
 *   name     - The reset controller name
 *   shared   - Whether reset controls are shared or not
 *   acquired - Only one reset control may be acquired for a given controller
 *   and ID
 *
 * Returned Value:
 *   Returns pointer to allocated reset_control on success or error on
 *   failure
 ****************************************************************************/

FAR struct reset_control *
reset_control_array_get(FAR const char *name, const int id[],
                        const unsigned int num, bool shared, bool acquired)
{
  FAR struct reset_control_array *resets;
  FAR struct reset_control *rstc;
  unsigned int i;

  resets = kmm_zalloc(sizeof(struct reset_control_array) +
                      sizeof(struct reset_control) * num);
  if (!resets)
    {
      return NULL;
    }

  for (i = 0; i < num; i++)
    {
      rstc = reset_control_get(name, id[i], shared, acquired);
      if (rstc == NULL)
        {
          goto err_rst;
        }

      resets->rstc[i] = rstc;
    }

  resets->num_rstcs = num;
  resets->base.array = true;

  return &resets->base;

err_rst:
  nxmutex_lock(&g_reset_list_mutex);
  while (i--)
    {
      reset_control_put_internal(resets->rstc[i]);
    }

  nxmutex_unlock(&g_reset_list_mutex);
  kmm_free(resets);

  return rstc;
}

/****************************************************************************
 * Name: reset_control_device_reset
 *
 * Description:
 *   Find reset controller associated with the device and perform reset,
 *   finally free the reset control.
 *   Convenience wrapper for reset_control_get() and reset_control_reset().
 *   This is useful for the common case of devices with single, dedicated
 *   reset lines.
 *
 * Input Parameters:
 *   name - The controller name
 *
 * Returned Value:
 *   Returns a negative errno if failed, otherwise success.
 ****************************************************************************/

int reset_control_device_reset(FAR const char *name)
{
  FAR struct reset_control *rstc;
  int ret;

  rstc = reset_control_get(name, 0, false, true);
  if (!rstc)
    {
      rsterr("get rstc failed.\n");
      return -EINVAL;
    }

  ret = reset_control_reset(rstc);
  reset_control_put(rstc);

  return ret;
}

/****************************************************************************
 * Nmae: reset_controller_register
 *
 * Description:
 *   Register a reset controller device
 *
 * Input Parameters:
 *   rcdev - A pointer to the initialized reset controller device
 *
 * Returned Value:
 *   Returns 0 if success, otherwise failed.
 ****************************************************************************/

int reset_controller_register(FAR struct reset_controller_dev *rcdev)
{
  if (rcdev == NULL || rcdev->name == NULL || rcdev->ops == NULL)
    {
      rsterr("rcdev is null\n");
      return -EINVAL;
    }

  list_initialize(&rcdev->reset_control_head);

  nxmutex_lock(&g_reset_list_mutex);
  list_add_after(&g_reset_controller_list, &rcdev->list);
  nxmutex_unlock(&g_reset_list_mutex);

  return 0;
}

/****************************************************************************
 * Name: reset_controller_unregister
 *
 * Description:
 *   Unregister a reset controller device
 *
 * Input Parameters:
 *   rcdev - A pointer to the reset controller device
 ****************************************************************************/

void reset_controller_unregister(FAR struct reset_controller_dev *rcdev)
{
  nxmutex_lock(&g_reset_list_mutex);
  list_delete(&rcdev->list);
  nxmutex_unlock(&g_reset_list_mutex);
}

