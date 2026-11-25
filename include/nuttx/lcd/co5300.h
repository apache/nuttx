/****************************************************************************
 * include/nuttx/lcd/co5300.h
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

#ifndef __INCLUDE_NUTTX_LCD_CO5300_H
#define __INCLUDE_NUTTX_LCD_CO5300_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LCD_CO5300

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct lcd_dev_s;
struct qspi_dev_s;

/****************************************************************************
 * Name: co5300_lcdinitialize
 *
 * Description:
 *   Initialize the CO5300 AMOLED controller attached to a QSPI bus.
 *
 *   The board must reset the panel (RESET pin) before calling this
 *   function.  The QSPI controller must perform QSPI_COMMAND() transfers
 *   on a single line (1-1-1) and must honor QSPIMEM_QUADDATA (1-1-4) in
 *   QSPI_MEMORY() transfers.  On ESP32-S3 call
 *   esp32s3_qspibus_set_attr(qspi, 0, 1, 1) first.
 *
 *   On return the display memory is cleared to black, the panel is on and
 *   the brightness is set to CONFIG_LCD_CO5300_BRIGHTNESS.
 *
 * Input Parameters:
 *   qspi - QSPI bus the panel is connected to
 *
 * Returned Value:
 *   A reference to the LCD object on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *co5300_lcdinitialize(FAR struct qspi_dev_s *qspi);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_CO5300 */
#endif /* __INCLUDE_NUTTX_LCD_CO5300_H */
