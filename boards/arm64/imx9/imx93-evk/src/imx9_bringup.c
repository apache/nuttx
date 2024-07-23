/****************************************************************************
 * boards/arm64/imx9/imx93-evk/src/imx9_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#include "imx9_dma_alloc.h"

#include "imx93-evk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int imx9_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_IMX9_DMA_ALLOC
  /* Initialize the DMA memory allocator */

  ret = imx9_dma_alloc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed initialize DMA allocator: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Configure PWM outputs */

  ret = imx9_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed initialize PWM outputs: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER)
  /* Configure I2C peripheral interfaces */

  ret = imx9_i2c_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_SPI_DRIVER)
  /* Configure SPI peripheral interfaces */

  ret = imx9_spi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_MMCSD
  ret = imx9_usdhc_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to init MMCSD driver: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
