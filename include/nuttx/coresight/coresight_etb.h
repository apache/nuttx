/****************************************************************************
 * include/nuttx/coresight/coresight_etb.h
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETB_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/mutex.h>
#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct coresight_etb_dev_s
{
  struct coresight_dev_s csdev;
  uint32_t trigger_cntr;             /* Amount of words to store after a trigger */
  uint32_t buffer_depth;             /* ETB buffer depth. */
  FAR uint32_t *bufptr;              /* Buffer that ETB content sends to. */
  mutex_t lock;                      /* Mutex for driver's open/close. */
  uint8_t opencnt;                   /* ETB device's open count. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: etb_register
 *
 * Description:
 *   Register an etb devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to an etb device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_etb_dev_s *
etb_register(FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: etb_unregister
 *
 * Description:
 *   Unregister an etb devices.
 *
 * Input Parameters:
 *   etbdev  - Pointer to the etb device.
 *
 ****************************************************************************/

void etb_unregister(FAR struct coresight_etb_dev_s *etbdev);

#endif  //__INCLUDE_NUTTX_CORESIGHT_CORESIGHT_ETB_H
