/****************************************************************************
 * include/nuttx/coresight/coresight_tpiu.h
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

#ifndef __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_TPIU_H
#define __INCLUDE_NUTTX_CORESIGHT_CORESIGHT_TPIU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/coresight/coresight.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct coresight_tpiu_dev_s
{
  struct coresight_dev_s csdev;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tpiu_register
 *
 * Description:
 *   Register a tpiu devices.
 *
 * Input Parameters:
 *   desc  - A description of this coresight device.
 *
 * Returned Value:
 *   Pointer to a tpiu device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct coresight_tpiu_dev_s *
tpiu_register(FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: tpiu_unregister
 *
 * Description:
 *   Unregister a tpiu devices.
 *
 * Input Parameters:
 *   tpiudev  - Pointer to the tpiu device.
 *
 ****************************************************************************/

void tpiu_unregister(FAR struct coresight_tpiu_dev_s *tpiudev);

#endif  //__INCLUDE_NUTTX_CORESIGHT_CORESIGHT_TPIU_H
