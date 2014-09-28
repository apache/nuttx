/****************************************************************************
 * include/nuttx/configdata.h
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_CONFIGDATA_H
#define __INCLUDE_NUTTX_CONFIGDATA_H

/* The configdata device details kernel level services for providing
 * application config data from kernel control objects, such as partitions
 * on shared MTD devices, etc.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_PLATFORM_CONFIGDATA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_AUDIO - Enables Audio driver support
 * CONFIG_DEBUG_AUDIO - If enabled (with CONFIG_DEBUG and, optionally,
 *   CONFIG_DEBUG_VERBOSE), this will generate output that can be used to
 *   debug Audio drivers.
 */

/* IOCTL Commands ***********************************************************/
/* The Audio module uses a standard character driver framework.  However, a
 * lot of the Audio driver functionality is configured via a device control
 * interface, such as sampling rate, volume, data format, etc.
 * The Audio ioctl commands are lised below:
 *
 * CFGDIOC_GETCONFIG - Get a specified Config Data item.
 *
 *   ioctl argument:  Pointer to a config_data_s structure to receive the
 *                    config data.  All fields of the strucure must be
 *                    specified (i.e. id, instance, pointer and len).
 *
 * CFGDIOC_SETCONFIG - Set a specified Config Data Item
 *
 *   ioctl argument:  Pointer to a config_data_s structure to receive the
 *                    config data.  All fields of the strucure must be
 *                    specified (i.e. id, instance, pointer and len).
 */

#define CFGDIOC_GETCONFIG  _CFGDIOC(1)
#define CFGDIOC_SETCONFIG  _CFGDIOC(2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure is used to get and set config data items */

struct config_data_s
{
  uint16_t    id;           /* ID of the config data item */
  int         instance;     /* Instance of the item */
  FAR uint8_t *configdata;  /* Pointer to the config data */
  size_t      len;          /* Length of the config data buffer */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mtdconfig_register
 *
 * Description:
 *   This function binds an instance of an MTD device to the /dev/config
 *   device.
 *
 *   When this function is called, the MTD device pass in should already
 *   be initialized appropriately to access the physical device or partition.
 *
 * Input parameters:
 *   mtd - Pointer to the MTD device to bind with the /dev/config device
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mtdconfig_register(FAR struct mtd_dev_s *mtd);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PLATFORM_CONFIGDATA */
#endif /* __INCLUDE_NUTTX_CONFIGDATA_H */
