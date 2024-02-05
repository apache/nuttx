/****************************************************************************
 * include/nuttx/rmt/rmtchar.h
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

#ifndef __INCLUDE_NUTTX_RMT_RMTCHAR_H
#define __INCLUDE_NUTTX_RMT_RMTCHAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/rmt/rmt.h>

#ifdef CONFIG_RMTCHAR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rmtchar_register
 *
 * Description:
 *   Create and register the RMT character driver.
 *
 *   The RMT character driver is a simple character driver that supports RMT
 *   transfers via read() and write(). This driver is primarily intended to
 *   support RMT testing. It is not suitable for use in any real driver
 *   application in its current form because its buffer management heuristics
 *   are dependent on the lower half driver (device-specific).
 *
 * Input Parameters:
 *   rmt - An instance of the lower half RMT driver
 *
 * Returned Value:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int rmtchar_register(FAR struct rmt_dev_s *rmt);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_RMT */
#endif /* __INCLUDE_NUTTX_RMT_RMTCHAR_H */
