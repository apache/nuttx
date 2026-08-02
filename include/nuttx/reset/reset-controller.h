/****************************************************************************
 * include/nuttx/reset/reset-controller.h
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

#ifndef __INCLUDE_NUTTX_RESET_CONTROLLER_H
#define __INCLUDE_NUTTX_RESET_CONTROLLER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/list.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RESET_NAME_MAX  24
#define RESET_EXTRA_MAX 32

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct reset_controller_dev;

/* What a controller can say about one reset line, beyond the asserted
 * state that status() already reports.  Both members are optional; an
 * empty name is reported as - and the line by its id alone.
 */

struct reset_lineinfo_s
{
  char name[RESET_NAME_MAX];    /* What this line resets */
  char extra[RESET_EXTRA_MAX];  /* Controller specific key:value fields,
                                 * appended to the line's /proc/reset
                                 * entry */
};

/* struct reset_control_ops - reset controller driver operations
 * reset: for self-deasserting resets, does all necessary
 *         things to reset the device
 * assert: manually assert the reset line, if supported
 * deassert: manually deassert the reset line, if supported
 * status: return the status of the reset line, if supported
 */

struct reset_control_ops
{
  CODE int (*acquire)(FAR struct reset_controller_dev *rcdev,
                      unsigned int id, bool shared, bool acquired);
  CODE int (*release)(FAR struct reset_controller_dev *rcdev,
                      unsigned int id);
  CODE int (*reset)(FAR struct reset_controller_dev *rcdev,
                    unsigned int id);
  CODE int (*assert)(FAR struct reset_controller_dev *rcdev,
                     unsigned int id);
  CODE int (*deassert)(FAR struct reset_controller_dev *rcdev,
                       unsigned int id);
  CODE int (*status)(FAR struct reset_controller_dev *rcdev,
                     unsigned int id);

  /* Describe one reset line, for /proc/reset.
   *
   * Optional; a controller without it is listed by name alone.  status()
   * already reports whether a line is asserted, so this supplies only what
   * the framework cannot derive: the line's name, and anything else the
   * controller wants shown.  Zero the structure and fill what applies.
   *
   * Returns OK, or -ENODEV for an id that names no line, which is how a
   * gap in the numbering is reported.
   */

  CODE int (*get_line)(FAR struct reset_controller_dev *rcdev,
                       unsigned int id,
                       FAR struct reset_lineinfo_s *info);
};

/* struct reset_controller_dev - reset controller entity that might
 *                               provide multiple reset controls
 * name a reset controller device name
 * ops: a pointer to device specific struct reset_control_ops
 * list: internal list of reset controller devices
 * reset_control_head: head of internal list of requested reset controls
 * nlines: the id space, so get_line() is asked about 0 to nlines - 1.
 *         Controllers commonly leave gaps, which get_line() reports, so
 *         this is a bound rather than a count of real lines.  Zero if the
 *         controller has no get_line().
 */

struct reset_controller_dev
{
  FAR const char *name;
  FAR const struct reset_control_ops *ops;
  struct list_node list;
  struct list_node reset_control_head;
  unsigned int nlines;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifdef CONFIG_RESET_RPMSG

/****************************************************************************
 * Name: reset_rpmsg_get
 *
 * Description:
 *
 * Input Parameters:
 *
 *   name - the name of the remote reset controller
 *
 * Returned Value:
 *
 *   Reset controller pointer
 *
 ****************************************************************************/

FAR struct reset_controller_dev *reset_rpmsg_get(FAR const char *name);

/****************************************************************************
 * Name: reset_rpmsg_server_init
 *
 * Description:
 * Server side rpmsg initialization
 *
 * Returned Value:
 *   Returns 0 if success.
 ****************************************************************************/

int reset_rpmsg_server_init(void);

#endif

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
