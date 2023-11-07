/****************************************************************************
 * drivers/motor/foc/foc_pwr.c
 * Power-stage FOC logic
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

#include <assert.h>

#include <nuttx/motor/foc/foc_pwr.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: focpwr_initialize
 *
 * Description:
 *   Initialize the FOC common power-stage data
 *
 * Input Parameters:
 *   pwr   - An instance of the FOC power-stage device
 *   devno - An instance number
 *   dev   - An instance of the FOC device
 *   ops   - power stage ops
 *
 ****************************************************************************/

int focpwr_initialize(FAR struct focpwr_dev_s *pwr,
                      int devno,
                      FAR struct foc_dev_s *dev,
                      FAR struct focpwr_ops_s *ops)
{
  DEBUGASSERT(ops->setup);
  DEBUGASSERT(ops->shutdown);
  DEBUGASSERT(ops->calibration);
  DEBUGASSERT(ops->ioctl);

  pwr->dev   = dev;
  pwr->devno = devno;
  pwr->ops   = ops;

  /* Connet to FOC device */

  dev->pwr = pwr;

  return OK;
}
