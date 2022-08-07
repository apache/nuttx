/****************************************************************************
 * include/sys/memfd.h
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

#ifndef __INCLUDE_SYS_MEMFD_H
#define __INCLUDE_SYS_MEMFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFD_CLOEXEC       O_CLOEXEC
#define MFD_ALLOW_SEALING (1 << 16)
#define MFD_HUGETLB       (1 << 17)

#define MFD_HUGE_SHIFT   26
#define MFD_HUGE_MASK    0x3f
#define MFD_HUGE_64KB    (16 << MFD_HUGE_SHIFT)
#define MFD_HUGE_512KB   (19 << MFD_HUGE_SHIFT)
#define MFD_HUGE_1MB     (20 << MFD_HUGE_SHIFT)
#define MFD_HUGE_2MB     (21 << MFD_HUGE_SHIFT)
#define MFD_HUGE_8MB     (23 << MFD_HUGE_SHIFT)
#define MFD_HUGE_16MB    (24 << MFD_HUGE_SHIFT)
#define MFD_HUGE_32MB    (25 << MFD_HUGE_SHIFT)
#define MFD_HUGE_256MB   (28 << MFD_HUGE_SHIFT)
#define MFD_HUGE_512MB   (29 << MFD_HUGE_SHIFT)
#define MFD_HUGE_1GB     (30 << MFD_HUGE_SHIFT)
#define MFD_HUGE_2GB     (31 << MFD_HUGE_SHIFT)
#define MFD_HUGE_16GB    (34 << MFD_HUGE_SHIFT)

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

int memfd_create(FAR const char *name, unsigned int flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SYS_MEMFD_H */
