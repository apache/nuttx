/****************************************************************************
 * boards/risc-v/shakti/arty_a7/src/shakti_bringup.c
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
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <sys/stat.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/buttons.h>
#include "mindgrove_spi.h"
#include "mindgrove_i2c.h"

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/i2c/i2c_master.h>

#include "secure-iot.h"
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>




/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shakti_bringup
 ****************************************************************************/

int mindgrove_bringup(void)
{

  // int ret = OK;
//   char devpath[12];
// #ifdef CONFIG_FS_PROCFS
//   /* Mount the procfs file system */

//   ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
//   if (ret < 0)
//     {
//       serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
//     }
// #endif

// #ifdef CONFIG_DEV_GPIO

//   ret = shakti_gpio_init();

//   if (ret<0){
//     serr("ERROR: Failed to initialize GPIO\n");
//   }
  
// #endif

// #ifdef CONFIG_PWM
//   ret = board_pwm_setup();
//   if (ret < 0)
//     {
//       _err("ERROR: Failed to initialize pwm.\n");
//     }
// #endif
// FAR struct i2c_master_s *i2c;
// #if defined(CONFIG_SHAKTI_I2C) && defined(CONFIG_SENSORS_BMP280)
  
  
//   i2c = shakti_i2cbus_initialize(1);

//   if (i2c)
//     {
//       /* Then try to register the barometer sensor in I2C0 */

//       ret = bmp280_register(0, i2c);
//       if (ret < 0)
//         {
//           _alert("ERROR: Error registering BMP180 in I2C%d\n", 1);
//         }
//     }
//   else
//     {
//       _alert("ERROR: Error initializing i2c\n");
//       ret = -ENODEV;
//     }
    
// #endif 

// #if defined(CONFIG_SHAKTI_I2C) && defined(CONFIG_LM75_I2C)
  
  
//   i2c = shakti_i2cbus_initialize(1);

//   if (i2c)
//     {
//       /* Then try to register the barometer sensor in I2C0 */
//       snprintf(devpath, 12, "/dev/temp");
//       ret = lm75_register(devpath, i2c, 0x48);
//       if (ret < 0)
//         {
//           _alert("ERROR: Error registering LM75 in I2C%d\n", 1);
//         }
//     }
//   else
//     {
//       _alert("ERROR: Error initializing i2c\n");
//       ret = -ENODEV;
//     }
    
// #endif


// #if defined(CONFIG_MINDGROVE_SPI)
//   struct spi_dev_s *spi0;
// printf("debug print\n\r");
//   spi0 = mg_spibus_initialize(0);

//   if (spi0==NULL)
//     {
//       _alert("ERROR: SPI0 init failed\n");
//       return -ENODEV;
//     }
// #ifdef CONFIG_SPI_DRIVER
//   int ret = spi_register(spi0, 0); // This creates /dev/spi0
//   if (ret < 0)
//     {
//       _alert("ERROR: Failed to register /dev/spi0: %d\n", ret);
//     }
// #endif
// #endif

struct spi_dev_s *spi;
  int ret;

#if defined(CONFIG_MINDGROVE_SPI0)
#ifdef CONFIG_SPI_MASTER
  spi = mg_spibus_initialize(0);
  if (spi != NULL)
    {
#ifdef CONFIG_SPI_DRIVER
      ret = spi_register(spi, 0); /* Creates /dev/spi0 */
      if (ret < 0) _alert("ERROR: Failed to register SPI0: %d\n", ret);
#endif
    }
    #endif
#endif

#if defined(CONFIG_MINDGROVE_SPI1)
  spi = mg_spibus_initialize(1);
  if (spi != NULL)
    {
#ifdef CONFIG_SPI_DRIVER
      ret = spi_register(spi, 1); /* Creates /dev/spi1 */
      if (ret < 0) _alert("ERROR: Failed to register SPI1: %d\n", ret);
#endif
    }
#endif

#if defined(CONFIG_MINDGROVE_SPI2)

  spi = mg_spibus_initialize(2);
  if (spi != NULL)
    {
#ifdef CONFIG_SPI_DRIVER
      ret = spi_register(spi, 2); /* Creates /dev/spi2 */
      if (ret < 0) _alert("ERROR: Failed to register SPI2: %d\n", ret);
#endif
    }
#endif

#if defined(CONFIG_MINDGROVE_SPI3)
  spi = mg_spibus_initialize(3);
  if (spi != NULL)
    {
#ifdef CONFIG_SPI_DRIVER
      ret = spi_register(spi, 3); /* Creates /dev/spi3 */
      if (ret < 0) _alert("ERROR: Failed to register SPI3: %d\n", ret);
#endif
    }
#endif

#ifdef CONFIG_SPI_SLAVE
  /* Register SPI Slave character driver(s) */

#if defined(CONFIG_MINDGROVE_SPI0)
printf("inside");
  struct spi_slave_ctrlr_s *slv_ctrlr0;
  slv_ctrlr0 = mg_spislave_initialize(0);
  if (slv_ctrlr0 != NULL)
    {
      ret = spi_slave_register(slv_ctrlr0, 0); /* Creates /dev/spislv0 */
      if (ret < 0) _alert("ERROR: Failed to register SPI Slave 0: %d\n", ret);
    }

#endif
#endif

#ifdef CONFIG_I2C
#if defined(CONFIG_MINDGROVE_I2C0)

  FAR struct i2c_master_s *i2c0;

  i2c0 = mg_i2c_initialize(0);
  if (i2c0 != NULL)
    {
      ret = i2c_register(i2c0, 0);   /* Creates /dev/i2c0 */
      if (ret < 0)
        {
          _alert("ERROR: Failed to register I2C0: %d\n", ret);
        }
    }

#endif
#endif

//   /* Register the DAC driver at "/dev/dac0" */

//   ret = dac_register("/dev/dac0", g_dac);
//   if (ret < 0)
//     {
//       _alert("ERROR: dac_register failed: %d\n", ret);
//       return ret;
//     }

// #endif

// #if defined(CONFIG_SHAKTI_SPI) && defined(CONFIG_MMCSD) && defined(CONFIG_MMCSD_SPI)
//   int rv;

//   mcinfo("INFO: Initializing mmcsd card\n");

//   spi_dev = shakti_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
//   if (spi_dev == NULL)
//     {
//       mcerr("ERROR: Failed to initialize SPI port %d\n", CONFIG_NSH_MMCSDSPIPORTNO);
//       return -ENODEV;
//     }

//   rv = mmcsd_spislotinitialize(0, 0, spi_dev);
//   if (rv < 0)
//     {
//       mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
//             CONFIG_NSH_MMCSDSPIPORTNO, 0);
//       return rv;
//     }

//   spiinfo("INFO: mmcsd card has been initialized successfully\n");
//   ret = nx_mount("/dev/mmcsd0", "/mnt/sd0", "vfat", 0, NULL);
//   if (ret == 0)
//     {
//       finfo("Successfully mount a SDCARD via the MMC/SD driver\n");
//     }
//   else
//     {
//       _err("ERROR: Failed to mount the SDCARD. %d\n", ret);
//     }
// #endif
  // return ret;
  return 0;
}
