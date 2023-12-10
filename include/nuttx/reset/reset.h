/****************************************************************************
 * include/nuttx/reset/reset.h
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

#ifndef __INCLUDE_NUTTX_RESET_H
#define __INCLUDE_NUTTX_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdatomic.h>
#include <stdbool.h>

#include <nuttx/list.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct reset_controller_dev;

/* struct reset_control - a reset control
 * rcdev: A pointer to the reset controller device
 *  this reset control belongs to
 * list: List entry for the rcdev's reset controller list
 * id: ID of the reset controller in the reset
 *  controller device
 * refcnt: Number of gets of this reset_control
 * acquired: Only one reset_control may be acquired for a given rcdev and
 *  id.
 * shared: Is this a shared (1), or an exclusive (0) reset_control?
 * deassert_cnt: Number of times this reset line has been deasserted
 * triggered_count: Number of times this reset line has been reset.
 *  Currently only used for shared resets, which means that the value will
 *  be either 0 or 1.
 */

struct reset_control
{
  FAR struct reset_controller_dev *rcdev;
  struct list_node list;
  unsigned int id;
  atomic_int refcnt;
  bool acquired;
  bool shared;
  bool array;
  atomic_int deassert_count;
  atomic_int triggered_count;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

struct reset_control;

/****************************************************************************
 * Name: reset_control_reset()
 *
 * Description:
 *   this function is used to execute reset ops.
 *
 * Input Parameters:
 *   rstc - a instance of reset control.
 *
 ****************************************************************************/

int reset_control_reset(FAR struct reset_control *rstc);

/****************************************************************************
 * Name: reset_control_assert()
 *
 * Description:
 *   This function is used to execute assert ops.
 *
 * Input Parameters:
 *   rstc - An instance of reset control.
 *
 ****************************************************************************/

int reset_control_assert(FAR struct reset_control *rstc);

/****************************************************************************
 * Name: reset_control_deassert()
 *
 * Description:
 *   This function is used to execute deassert ops.
 *
 * Input Parameters:
 *   rstc - An instance of reset control.
 *
 ****************************************************************************/

int reset_control_deassert(FAR struct reset_control *rstc);

/****************************************************************************
 * Name: reset_control_status()
 *
 * Description:
 *   This function is used to get reset_control ops.
 *
 * Input Parameters:
 *   rstc - An instance of reset control.
 *
 * Returned Value:
 *   Return a reset_control if success, NULL others.
 *
 ****************************************************************************/

int reset_control_status(FAR struct reset_control *rstc);

/****************************************************************************
 * Name: reset_control_get
 *
 * Description:
 *   This function is used to get a reset control by reset controller name.
 *
 *   Firstly, get a reset controller device from list, and then call
 *   reset_control_get_internal function by index, shared or acquired
 *   parameters retrun a reset control.
 *
 * Input Parameters:
 *   name     - The reset controller name
 *   index    - The reset controller in reset controller device
 *   shared   - Is this a shared (1), or an exclusive (0) reset_control
 *   acquired - flags that used to get a exculsive reset control
 *
 * Returned Value:
 *   Return reset_control if success, others return NULL if failed
 ****************************************************************************/

FAR struct reset_control *
reset_control_get(FAR const char *name, int index, bool shared,
                  bool acquired);

/****************************************************************************
 * Name: reset_control_acquire()
 *
 * Description:
 *   This function is used to acquire reset_control access ops.
 *
 * Input Parameters:
 *   rstc - An instance of reset control.
 *
 ****************************************************************************/

int reset_control_acquire(FAR struct reset_control *rstc);

/****************************************************************************
 * Name: reset_control_release()
 *
 * Description:
 *   This function is used to release reset_control access ops.
 *
 * Input Parameters:
 *   rstc - An instance of reset control.
 *
 ****************************************************************************/

void reset_control_release(FAR struct reset_control *rstc);

/****************************************************************************
 * Name: reset_control_put()
 *
 * Description:
 *   This function is used to free reset_control ops.
 *
 * Input Parameters:
 *   rstc - An instance of reset control.
 *
 ****************************************************************************/

void reset_control_put(FAR struct reset_control *rstc);

/****************************************************************************
 * Name: reset_control_device_reset()
 *
 * Description:
 *   This function is used to get and free reset_control ops.
 *
 * Input Parameters:
 *   rstc - An instance of reset control.
 *
 ****************************************************************************/

int reset_control_device_reset(FAR const char *name);

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
                        const unsigned int num, bool shared, bool acquired);

/****************************************************************************
 * Name: reset_control_get_exclusive()
 *
 * Description:
 *   Lookup and obtain an exclusive reference to a reset controller.
 *   If this function is called more than once for the same reset_control
 *   it will return NULL.
 *
 * Input Parameters:
 *   name - Reset line name.
 *
 * Returned Value:
 *   Return a reset_control if success, NULL others.
 *
 ****************************************************************************/

static FAR inline struct reset_control *
reset_control_get_exclusive(FAR const char *name)
{
  return reset_control_get(name, 0, false, true);
}

/****************************************************************************
 * Name: reset_control_get_exclusive_released()
 *
 * Description:
 *   Lookup and obtain a temoprarily exclusive reference to a reset
 *   controller.
 *   Reset-controls returned by this function must be acquired via
 *   reset_control_acquire() before they can be used and should be released
 *   via reset_control_release() afterwards
 *
 * Input Parameters:
 *   name - Reset line name.
 *
 * Returned Value:
 *   Return a reset_control if success, NULL others.
 *
 ****************************************************************************/

static inline FAR struct reset_control *
reset_control_get_exclusive_released(FAR const char *name)
{
  return reset_control_get(name, 0, false, false);
}

/****************************************************************************
 * Name: reset_control_get_shared()
 *
 * Description:
 *   Lookup and obtain a shared reference to a reset controller.
 *
 *   This function is intended for use with reset-controls which are shared
 *   between hardware blocks.
 *
 *   When a reset-control is shared, the behavior of reset_control_assert
 *   deassert is changed, the reset-core will keep track of a deassert_count
 *   and only (re-)assert the reset after reset_control_assert has been
 *   called as many times as reset_control_deassert was called. Also see the
 *   remark about shared reset-controls in the reset_control_assert docs.
 *
 * Input Parameters:
 *   name - Reset line name.
 *
 * Returned Value:
 *   Return a reset_control if success, NULL others.
 *
 ****************************************************************************/

static inline FAR struct reset_control *
reset_control_get_shared(FAR const char *name)
{
  return reset_control_get(name, 0, true, false);
}

/****************************************************************************
 * Name: reset_control_get_exclusive_by_index
 *
 * Description:
 *   Lookup and obtain an exclusive reference to a reset controller by index.
 *
 *   This is to be used to perform a list of resets for a device or power
 *   domain in whatever order. Returns a struct reset_control or NULL errno.
 * Input Parameters:
 *   name  - The controller name symble
 *   index - Index of the reset controller
 *
 * Returned Value:
 *   Return a reset_control if success, others failed.
 ****************************************************************************/

static inline FAR struct reset_control *
reset_control_get_exclusive_by_index(FAR const char *name, int index)
{
  return reset_control_get(name, index, false, true);
}

/****************************************************************************
 * Name: reset_control_get_shared_by_index
 *
 * Description:
 *   Lookup and obtain a shared reference to a reset controller by index.
 *
 *   When a reset-control is shared, the behavior of reset_control_assert
 *   deassert is changed, the reset-core will keep track of a deassert_count
 *   and only (re-)assert the reset after reset_control_assert has been
 *   called as many times as reset_control_deassert was called. Also see the
 *   remark about shared reset-controls in the reset_control_assert docs.
 *
 *   Calling reset_control_assert without first calling
 *   reset_control_deassert is not allowed on a shared reset control. Calling
 *   reset_control_reset is also not allowed on a shared reset control.
 *   Returns a struct reset_control or NULL errno.
 *
 *   This is to be used to perform a list of resets for a device or power
 *   domain in whatever order. Returns a struct reset_control or NULL errno.
 *
 * Input Parameters:
 *   node  - Device to be reset by the controller
 *   index - Index of the reset controller
 *
 * Returned Value:
 *   Return a reset_control if success, others failed.
 *
 ****************************************************************************/

static inline FAR struct reset_control *
reset_control_get_shared_by_index(FAR const char *name, int index)
{
  return reset_control_get(name, index, true, false);
}

#endif /* __INCLUDE_NUTTX_RESET_H */
