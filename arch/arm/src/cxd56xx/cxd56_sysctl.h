/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sysctl.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_SYSCTL_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_SYSCTL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System control commands */

/* XXX: Command assignments are experimental */

#define SYSCTL_INVALID 0

#define SYSCTL_MAP       1
#define SYSCTL_UNMAP     2
#define SYSCTL_GETFWSIZE 3
#define SYSCTL_LOADFW    4
#define SYSCTL_UNLOADFW  5
#define SYSCTL_GETPGOFFSETS 6
#define SYSCTL_LOADFWUNIFY  7
#define SYSCTL_LOADFWCLONE  8
#define SYSCTL_UNLOADFWGP   9

/* Number of sub cores */

#define SYSCTL_NR_CPUS 5

/* Limit length of filename */

#define SYSCTL_FNLEN   32

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Arguments for SYSCTL_MAP */

typedef struct sysctl_map_s
{
  int cpuid;
  uint32_t virt;
  uint32_t phys;
  uint32_t size;
} sysctl_map_t;

/* Arguments for SYSCTL_UNMAP */

typedef struct sysctl_unmap_s
{
  int cpuid;
  uint32_t virt;
  uint32_t size;
} sysctl_unmap_t;

/* Arguments for SYSCTL_GETFWSIZE */

typedef struct sysctl_getfwsize_s
{
    char filename[SYSCTL_FNLEN];
} sysctl_getfwsize_t;

/* Arguments for SYSCTL_LOADFW */

typedef struct sysctl_loadfw_s
{
    int cpuid;
    uint32_t addr;
    char filename[SYSCTL_FNLEN];
} sysctl_loadfw_t;

/* Arguments for SYSCTL_UNLOADFW */

typedef struct sysctl_unloadfw_s
{
    int cpuid;
} sysctl_unloadfw_t;

/* Arguments for SYSCTL_GETPGOFFSETS */

typedef struct sysctl_getpgoffsets_s
{
    uint8_t offset[SYSCTL_NR_CPUS];
    uint8_t size[SYSCTL_NR_CPUS];
    int nr_offsets;
    char filename[SYSCTL_FNLEN];
} sysctl_getpgoffsets_t;

/* Arguments for SYSCTL_LOADFWUNIFY and SYSCTL_LOADFWCLONE */

typedef struct sysctl_loadfwgp_s
{
    uint32_t cpuids;
    uint32_t addr[SYSCTL_NR_CPUS];
    int nr_addrs;
    char filename[SYSCTL_FNLEN];
} sysctl_loadfwgp_t;

/* Arguments for SYSCTL_UNLOADFWGP */

typedef struct sysctl_unloadfwgp_s
{
    int groupid;
} sysctl_unloadfwgp_t;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* Initialize system control block */

void cxd56_sysctlinitialize(void);

/* Send system control command */

int cxd56_sysctlcmd(uint8_t id, uint32_t data);

#endif
