/****************************************************************************
 * include/nuttx/video/max7456.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_MAX7456_H
#define __INCLUDE_NUTTX_VIDEO_MAX7456_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct spi_dev_s;

struct mx7_config_s
{
  int spi_devid;
  FAR struct spi_dev_s *spi;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: max7456_register
 *
 * Description:
 *   Registers an max7456 chip, and creates an interface 'devpath'
 *
 * Input Parameters:
 *   path    - The full path to the interface to register. E.g., "/dev/osd0"
 *   config  - Configuration information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max7456_register(FAR const char *path, FAR struct mx7_config_s *config);

#endif /* __INCLUDE_NUTTX_VIDEO_MAX7456_H */
