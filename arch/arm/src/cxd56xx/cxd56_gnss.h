/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gnss.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* GNSS specific debug */

#ifdef CONFIG_CXD56_GNSS_DEBUG_ERROR
#  define gnsserr(fmt, ...)   _err(fmt, ## __VA_ARGS__)
#else
#  define gnsserr(fmt, ...)
#endif

#ifdef CONFIG_CXD56_GNSS_DEBUG_WARN
#  define gnsswarn(fmt, ...)  _warn(fmt, ## __VA_ARGS__)
#else
#  define gnsswarn(fmt, ...)
#endif

#ifdef CONFIG_CXD56_GNSS_DEBUG_INFO
#  define gnssinfo(fmt, ...)  _info(fmt, ## __VA_ARGS__)
#else
#  define gnssinfo(fmt, ...)
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_gnssinitialize
 *
 * Description:
 *   Initialize GNSS device
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gps"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gnssinitialize(const char *devpath);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_H */
