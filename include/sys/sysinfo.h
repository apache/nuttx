/****************************************************************************
 * include/sys/sysinfo.h
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

#ifndef __INCLUDE_SYS_SYSINFO_H
#define __INCLUDE_SYS_SYSINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SI_LOAD_SHIFT 16

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct sysinfo
{
  unsigned long uptime;    /* Seconds since boot */
  unsigned long loads[3];  /* 1, 5, and 15 minute load averages */
  unsigned long totalram;  /* Total usable main memory size */
  unsigned long freeram;   /* Available memory size */
  unsigned long sharedram; /* Amount of shared memory */
  unsigned long bufferram; /* Memory used by buffers */
  unsigned long totalswap; /* Total swap space size */
  unsigned long freeswap;  /* Swap space still available */
  unsigned short procs;    /* Number of current processes */
  unsigned short pad;      /* Padding for alignment */
  unsigned long totalhigh; /* Total high memory size */
  unsigned long freehigh;  /* Available high memory size */
  unsigned mem_unit;       /* Memory unit size in bytes */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int sysinfo(FAR struct sysinfo *info);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_SYSINFO_H */
