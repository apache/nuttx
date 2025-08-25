/****************************************************************************
 * boards/arm/samv7/pic32czca70-curiosity/src/sam_progmem_partition.c
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

#include <stdbool.h>
#include <syslog.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/net/usrsock.h> /* For nitems */

#include "sam_board.h"
#include "board_progmem.h" /* For struct mtd_partition_s definition */

#ifdef CONFIG_SAMV7_PROGMEM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mtd_partition_s g_mtd_partition_table[] =
{
  {
    .offset  = 0x1e0000,
    .size    = 0x20000,
    .devpath = "dev/progmem"
  },
};

static const size_t g_mtd_partition_table_size =
    nitems(g_mtd_partition_table);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_flash_init
 *
 * Description:
 *   Initialize the embedded flash programming memory.
 *
 ****************************************************************************/

int sam_flash_init()
{
  int ret;

  /* Call SAMv7 common board function to init progmem. */

  ret = board_progmem_init(PROGMEM_MTD_MINOR, g_mtd_partition_table,
                           g_mtd_partition_table_size);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize progmem: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_SAMV7_PROGMEM */
