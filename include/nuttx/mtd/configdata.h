/****************************************************************************
 * include/nuttx/mtd/configdata.h
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

#ifndef __INCLUDE_NUTTX_MTD_CONFIGDATA_H
#define __INCLUDE_NUTTX_MTD_CONFIGDATA_H

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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* IOCTL Commands to store and read configuration for user application.
 *
 * CFGDIOC_GETCONFIG - Get a specified Config Data item.
 *
 *   ioctl argument:  Pointer to a config_data_s structure to receive the
 *                    config data.  All fields of the structure must be
 *                    specified (i.e. id, instance, pointer and len).
 *
 * CFGDIOC_SETCONFIG - Set a specified Config Data Item
 *
 *   ioctl argument:  Pointer to a config_data_s structure to receive the
 *                    config data.  All fields of the structure must be
 *                    specified (i.e. id, instance, pointer and len).
 */

#define CFGDIOC_GETCONFIG    _CFGDIOC(1)
#define CFGDIOC_SETCONFIG    _CFGDIOC(2)
#define CFGDIOC_DELCONFIG    _CFGDIOC(3)
#define CFGDIOC_FINDCONFIG   _CFGDIOC(4)
#define CFGDIOC_FIRSTCONFIG  _CFGDIOC(5)
#define CFGDIOC_NEXTCONFIG   _CFGDIOC(6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure is used to get and set config data items */

struct config_data_s
{
#ifdef CONFIG_MTD_CONFIG_NAMED
  char        name[CONFIG_MTD_CONFIG_NAME_LEN];
#else
  uint16_t    id;           /* ID of the config data item */
  int         instance;     /* Instance of the item */
#endif
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
 * Input Parameters:
 *   mtd - Pointer to the MTD device to bind with the /dev/config device
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct mtd_dev_s;
int mtdconfig_register(FAR struct mtd_dev_s *mtd);

/****************************************************************************
 * Name: mtdconfig_unregister
 *
 * Description:
 *   This function unregisters /dev/config device.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mtdconfig_unregister(void);

/****************************************************************************
 * Name: mtdconfig_register_by_path
 *
 * Description:
 *   This function binds an instance of an MTD device to the path specified
 *   device.
 *
 *   When this function is called, the MTD device pass in should already
 *   be initialized appropriately to access the physical device or partition.
 *
 * Input Parameters:
 *   mtd - Pointer to the MTD device to bind with the path device
 *   path - Path name of the file backing the MTD device
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mtdconfig_register_by_path(FAR struct mtd_dev_s *mtd,
                               FAR const char *path);

/****************************************************************************
 * Name: mtdconfig_unregister_by_path
 *
 * Description:
 *   This function unregisters path device.
 *
 * Input Parameters:
 *   path - Path name of the file backing the MTD device
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mtdconfig_unregister_by_path(FAR const char *path);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MTD_CONFIGDATA_H */
