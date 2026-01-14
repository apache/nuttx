/****************************************************************************
 * include/nuttx/analog/mcp3008.h
 *
 * Contributed by Matteo Golin
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

#ifndef __INCLUDE_NUTTX_ANALOG_MCP3008_H
#define __INCLUDE_NUTTX_ANALOG_MCP3008_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/analog/ioctl.h>
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* IOCTL Commands
 * Cmd: ANIOC_MCP3008_DIFF      Arg: 1 for differential, 0 for single-ended
 */

#define ANIOC_MCP3008_DIFF      _ANIOC(AN_MCP3008_FIRST + 0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct adc_dev_s *mcp3008_initialize(FAR struct spi_dev_s *spi);

#endif /* __INCLUDE_NUTTX_ANALOG_MCP3008_H */
