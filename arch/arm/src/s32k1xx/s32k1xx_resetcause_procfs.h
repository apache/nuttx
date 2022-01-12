/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_resetcause_procfs.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_RESETCAUSE_PROCFS_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_RESETCAUSE_PROCFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_S32K1XX_RESETCAUSE_PROCFS

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_resetcause_procfs_register
 *
 * Description:
 *   Register the S32K1XX Reset Cause procfs file system entry
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

int s32k1xx_resetcause_procfs_register(void);

#endif /* CONFIG_S32K1XX_RESETCAUSE_PROCFS */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_RESETCAUSE_PROCFS_H */
