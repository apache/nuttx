/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/sam_classd.c
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
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "sam_classd.h"
#include "jti-toucan2.h"
#include "sam_pio.h"

#if defined (CONFIG_SAMA5D2_CLASSD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_classd_setup
 *
 * Description:
 *  Initialize CLASSD and register the CLASSD device
 *
 ****************************************************************************/ 

int sam_classd_setup(void)
{
  FAR struct classd_dev_s *dev;
  int ret;
  dev = sam_classd_initialize();
  if (dev == NULL)
    {
      auderr("ERROR:  Failed to get classd interface\n");
      return -ENODEV;
    }
  /*
  ret = can_register("/dev/can0", can0);
  if (ret < 0)
    {
      canerr("ERROR: can_register can0 failed: %d\n", ret);
      return ret;
    }     
  */
  return OK;
}

#endif /* CONFIG_SAMA5D2_CLASSD */
