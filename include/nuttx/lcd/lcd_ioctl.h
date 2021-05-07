/****************************************************************************
 * include/nuttx/lcd/lcd_ioctl.h
 * IOCTL commands for segment LCDs
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

#ifndef __INCLUDE_NUTTX_LCD_LCD_IOCTL_H
#define __INCLUDE_NUTTX_LCD_LCD_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL commands set aside for FT80x character driver */

#define FT80X_NIOCTL_CMDS     16
#define FT80X_NIOCTL_BASE     0x0001

/* IOCTL commands set aside for TDA19988 HDMI encoder */

#define TDA19988_NIOCTL_CMDS  1
#define TDA19988_NIOCTL_BASE  (FT80X_NIOCTL_BASE + FT80X_NIOCTL_CMDS)

#endif /* __INCLUDE_NUTTX_LCD_LCD_IOCTL_H */
