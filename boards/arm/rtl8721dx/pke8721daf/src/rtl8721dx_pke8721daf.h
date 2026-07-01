/****************************************************************************
 * boards/arm/rtl8721dx/pke8721daf/src/rtl8721dx_pke8721daf.h
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

#ifndef __BOARDS_ARM_RTL8721DX_PKE8721DAF_SRC_RTL8721DX_PKE8721DAF_H
#define __BOARDS_ARM_RTL8721DX_PKE8721DAF_SRC_RTL8721DX_PKE8721DAF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: rtl8721dx_boardinitialize
 *
 * Description:
 *   Perform board-specific early initialization.
 *
 ****************************************************************************/

void rtl8721dx_boardinitialize(void);

/****************************************************************************
 * Name: rtl8721dx_bringup
 *
 * Description:
 *   Bring up board features.
 *
 ****************************************************************************/

int rtl8721dx_bringup(void);

#ifdef CONFIG_RTL8721DX_WIFI
/****************************************************************************
 * Name: rtl8721dx_wifi_initialize
 *
 * Description:
 *   Bring up the KM4 IPC transport and start the WHC host WiFi stack
 *   (arch/arm/src/rtl8721dx/ameba_wifi_init.c).
 *
 ****************************************************************************/

int rtl8721dx_wifi_initialize(void);
#endif

#ifdef CONFIG_RTL8721DX_FLASH_FS
/****************************************************************************
 * Name: ameba_flash_fs_initialize
 *
 * Description:
 *   Register the on-chip SPI NOR data partition as an MTD device and mount
 *   a littlefs filesystem on it at /data
 *   (arch/arm/src/rtl8721dx/ameba_flash_mtd.c).
 *
 ****************************************************************************/

int ameba_flash_fs_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RTL8721DX_PKE8721DAF_SRC_RTL8721DX_PKE8721DAF_H */
