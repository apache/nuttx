/****************************************************************************
 * include/nuttx/spi/spi_rpmsg.h
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

#ifndef __INCLUDE_NUTTX_SPI_SPI_RPMSG_H
#define __INCLUDE_NUTTX_SPI_SPI_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CONFIG_SPI_RPMSG
/****************************************************************************
 * Name: spi_rpmsg_initialize
 *
 * Description:
 *   Create an SPI client lower-half that forwards transfers to a remote
 *   RPMsg SPI server.
 *
 * Input Parameters:
 *   cpuname - Name of the remote CPU to connect to.
 *   name    - RPMsg SPI service name.
 *
 * Returned Value:
 *   A pointer to the SPI lower-half on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct spi_dev_s *spi_rpmsg_initialize(FAR const char *cpuname,
                                           FAR const char *name);
#endif

#ifdef CONFIG_SPI_RPMSG_SERVER
/****************************************************************************
 * Name: spi_rpmsg_server_initialize
 *
 * Description:
 *   Export a local SPI lower-half to remote RPMsg SPI clients.
 *
 * Input Parameters:
 *   name - RPMsg SPI service name.
 *   spi  - Local SPI lower-half to export.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int spi_rpmsg_server_initialize(FAR const char *name,
                                FAR struct spi_dev_s *spi);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SPI_SPI_RPMSG_H */
