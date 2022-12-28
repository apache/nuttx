/****************************************************************************
 * drivers/video/mipidsi/mipi_dsi.h
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

#ifndef __DRIVERS_VIDEO_MIPIDSI_MIPI_DSI_H
#define __DRIVERS_VIDEO_MIPIDSI_MIPI_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/video/mipi_dsi.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mipi_dsi_host_driver_register
 *
 * Description:
 *   Create and register the dsi host character driver.
 *
 *   The dsi host character driver is a simple character driver that
 *   supports dsi transfer.
 *
 * Input Parameters:
 *   host - An instance of the struct mipi_dsi_host
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int mipi_dsi_host_driver_register(FAR struct mipi_dsi_host *host);

/****************************************************************************
 * Name: mipi_dsi_device_driver_register
 *
 * Description:
 *   Create and register the dsi device character driver.
 *
 *   The dsi device character driver is a simple character driver that
 *   supports get dsi device params.
 *
 * Input Parameters:
 *   device - An instance of the struct mipi_dsi_device
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int mipi_dsi_device_driver_register(FAR struct mipi_dsi_device *device);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __DRIVERS_VIDEO_MIPIDSI_MIPI_DSI_H */
