/****************************************************************************
 * include/nuttx/video/vfb.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_VFB_H
#define __INCLUDE_NUTTX_VIDEO_VFB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: vfb_register
 *
 * Description:
 *   Allocate the virtual framebuffer and register it at /dev/fbN.  There
 *   is no display behind it;  it is memory with the geometry the
 *   configuration asks for.
 *
 * Input Parameters:
 *   display - The display number, N in /dev/fbN.  A board that also has a
 *             panel of its own gives the two different numbers.
 *
 * Returned Value:
 *   Zero (OK) is returned on success;  a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_VIDEO_VFB
int vfb_register(int display);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_VFB_H */
