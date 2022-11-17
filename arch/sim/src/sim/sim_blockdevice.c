/****************************************************************************
 * arch/sim/src/sim/sim_blockdevice.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/drivers/ramdisk.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS            2048
#define LOGICAL_SECTOR_SIZE 512

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_registerblockdevice
 *
 * Description: Register the FAT ramdisk
 *
 ****************************************************************************/

void sim_registerblockdevice(void)
{
  ramdisk_register(0, (uint8_t *)sim_deviceimage(), NSECTORS,
                   LOGICAL_SECTOR_SIZE, RDFLAG_WRENABLED | RDFLAG_FUNLINK);
}
