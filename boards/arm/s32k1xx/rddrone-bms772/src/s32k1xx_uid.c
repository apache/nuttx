/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/s32k1xx_uid.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <nuttx/compiler.h>
#include <stddef.h>
#include <stdint.h>
#include <errno.h>

#include "s32k1xx_uid.h"

#ifdef CONFIG_BOARDCTL_UNIQUEID

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_BOARDCTL_UNIQUEID_SIZE != S32K1XX_UID_SIZE
#  error CONFIG_BOARDCTL_UNIQUEID_SIZE does not match S32K1XX_UID_SIZE
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_uniqueid
 ****************************************************************************/

int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == NULL)
    {
      return -EINVAL;
    }

  s32k1xx_get_uniqueid(uniqueid);

  return OK;
}

#endif /* CONFIG_BOARDCTL_UNIQUEID */
