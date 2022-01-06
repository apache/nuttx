/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_enc.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sensors/qencoder.h>

#include "imxrt_enc.h"
#include "imxrt_xbar.h"
#include "teensy-4.h"

#if defined(CONFIG_IMXRT_ENC) && defined(CONFIG_SENSORS_QENCODER)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_enc_initialize
 *
 * Description:
 *   Initialize the quadrature encoder driver for the given timer
 *
 ****************************************************************************/

int imxrt_enc_initialize(void)
{
  int ret;
#ifdef CONFIG_IMXRT_ENC1
  /* Initialize a quadrature encoder interface. */

  imxrt_config_gpio(GPIO_ENC1_PHASE_A);
  imxrt_config_gpio(GPIO_ENC1_PHASE_B);
  imxrt_config_gpio(GPIO_ENC1_INDEX);

  /* Connect XBAR pins */

  ret = imxrt_xbar_connect(IMXRT_XBARA1_OUT_ENC1_PHASE_AIN_SEL_OFFSET,
                           IMXRT_XBARA1_IN_IOMUX_XBAR_IO09);
  if (ret < 0)
    {
      snerr("ERROR: imxrt_xbar_connect failed: %d\n", ret);
      return -ENODEV;
    }

  ret = imxrt_xbar_connect(IMXRT_XBARA1_OUT_ENC1_PHASE_BIN_SEL_OFFSET,
                           IMXRT_XBARA1_IN_IOMUX_XBAR_IO08);
  if (ret < 0)
    {
      snerr("ERROR: imxrt_xbar_connect failed: %d\n", ret);
      return -ENODEV;
    }

  ret = imxrt_xbar_connect(IMXRT_XBARA1_OUT_ENC1_INDEX_SEL_OFFSET,
                           IMXRT_XBARA1_IN_IOMUX_XBAR_IO10);
  if (ret < 0)
    {
      snerr("ERROR: imxrt_xbar_connect failed: %d\n", ret);
      return -ENODEV;
    }

  ret = imxrt_qeinitialize("dev/qe0", 1);
  if (ret < 0)
    {
      snerr("ERROR: imxrt_qeinitialize failed: %d\n", ret);
      return -ENODEV;
    }

#endif
#ifdef CONFIG_IMXRT_ENC2
  /* Initialize a quadrature encoder interface. */

  ret = imxrt_qeinitialize("dev/qe1", 2)
  if (ret < 0)
    {
      snerr("ERROR: imxrt_qeinitialize failed: %d\n", ret);
      return -ENODEV;
    }

#endif
#ifdef CONFIG_IMXRT_ENC3
  /* Initialize a quadrature encoder interface. */

  ret = imxrt_qeinitialize("dev/qe2", 3)
  if (ret < 0)
    {
      snerr("ERROR: imxrt_qeinitialize failed: %d\n", ret);
      return -ENODEV;
    }

#endif
#ifdef CONFIG_IMXRT_ENC4
  /* Initialize a quadrature encoder interface. */

  ret = imxrt_qeinitialize("dev/qe4", 4)
  if (ret < 0)
    {
      snerr("ERROR: imxrt_qeinitialize failed: %d\n", ret);
      return -ENODEV;
    }

#endif
  UNUSED(ret);
  return OK;
}
#endif /* CONFIG_IMXRT_ENC */