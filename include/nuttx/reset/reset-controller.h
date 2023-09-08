/****************************************************************************
 * include/nuttx/reset/reset-controller.h
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

#ifndef __INCLUDE_NUTTX_RESET_CONTROLLER_H
#define __INCLUDE_NUTTX_RESET_CONTROLLER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/list.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct reset_controller_dev;

/* struct reset_control_ops - reset controller driver operations
 * reset: for self-deasserting resets, does all necessary
 *         things to reset the device
 * assert: manually assert the reset line, if supported
 * deassert: manually deassert the reset line, if supported
 * status: return the status of the reset line, if supported
 */

struct reset_control_ops
{
  CODE int (*reset)(FAR struct reset_controller_dev *rcdev, unsigned int id);
  CODE int (*assert)(FAR struct reset_controller_dev *rcdev,
                     unsigned int id);
  CODE int (*deassert)(FAR struct reset_controller_dev *rcdev,
                       unsigned int id);
  CODE int (*status)(FAR struct reset_controller_dev *rcdev,
                       unsigned long id);
};

/* struct reset_controller_dev - reset controller entity that might
 *                               provide multiple reset controls
 * name a reset controller device name
 * ops: a pointer to device specific struct reset_control_ops
 * list: internal list of reset controller devices
 * reset_control_head: head of internal list of requested reset controls
 */

struct reset_controller_dev
{
  FAR const char *name;
  FAR const struct reset_control_ops *ops;
  struct list_node list;
  struct list_node reset_control_head;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: reset_controller_register()
 *
 * Description:
 *   this function is used to register a reset_controller_dev to list.
 *
 * Input Parameters:
 *   rcdev - a instance of reset_controller_dev.
 *
 * Return value:
 *   return 0 if success, others failed.
 ****************************************************************************/

int reset_controller_register(FAR struct reset_controller_dev *rcdev);

/****************************************************************************
 * Name: reset_controller_unregister()
 *
 * Description:
 *   this function is used to unregister a reset_controller_dev to list.
 *
 * Input Parameters:
 *   rcdev - a instance of reset_controller_dev.
 *
 * Return value:
 *   return 0 if success, others failed.
 ****************************************************************************/

void reset_controller_unregister(FAR struct reset_controller_dev *rcdev);

#endif /* __INCLUDE_NUTTX_RESET_CONTROLLER_H */
