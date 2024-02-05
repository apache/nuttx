/****************************************************************************
 * include/nuttx/motor/foc/foc_pwr.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_FOC_FOC_PWR_H
#define __INCLUDE_NUTTX_MOTOR_FOC_FOC_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/motor/foc/foc.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FOC power-stage driver operations */

struct focpwr_dev_s;
struct focpwr_ops_s
{
  CODE int (*setup)(FAR struct focpwr_dev_s *dev);
  CODE int (*shutdown)(FAR struct focpwr_dev_s *dev);
  CODE int (*calibration)(FAR struct focpwr_dev_s *dev, bool state);
  CODE int (*ioctl)(FAR struct focpwr_dev_s *dev, int cmd,
                    unsigned long arg);
};

/* FOC power-stage driver */

struct focpwr_dev_s
{
  FAR struct foc_dev_s    *dev;
  FAR struct focpwr_ops_s *ops;
  int                      devno;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int focpwr_initialize(FAR struct focpwr_dev_s *pwr,
                      int devno,
                      FAR struct foc_dev_s *dev,
                      FAR struct focpwr_ops_s *ops);

#endif /* __INCLUDE_NUTTX_MOTOR_FOC_FOC_PWR_H */
