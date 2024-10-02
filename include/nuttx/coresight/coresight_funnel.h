/****************************************************************************
 * include/nuttx/coresight/coresight_funnel.h
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_FUNNEL_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_FUNNEL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct coresight_funnel_dev_s
{
  struct coresight_dev_s csdev;
  uint32_t priority;                 /* Port selection order. */
  uint8_t port_num;                  /* Port numbre. */
  uint8_t port_refcnt[0];            /* Port refcnt. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: set_funnel_priority
 *
 * Description:
 *   Set funnel ports priority. It should to be called when port has not been
 *   enabled.
 *
 * Input Parameters:
 *   fundev  - Pointer to the funnel coresight device.
 *   priority- Priority to set.
 *
 ****************************************************************************/

void set_funnel_priority(FAR struct coresight_funnel_dev_s *fundev,
                         uint32_t priority);

/****************************************************************************
 * Name: funnel_register
 *
 * Description:
 *   Register an funnel devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a funnel device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_funnel_dev_s *
funnel_register(FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: funnel_unregister
 *
 * Description:
 *   Unregister a funnel devices.
 *
 * Input Parameters:
 *   fundev  - Pointer to the funnel device.
 *
 ****************************************************************************/

void funnel_unregister(FAR struct coresight_funnel_dev_s *fundev);

#endif  //__INCLUDE_NUTTX_CORESIGHT_CORESIGHT_FUNNEL_H
