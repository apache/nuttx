/****************************************************************************
 * arch/risc-v/src/espressif/esp_random.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include "esp_random.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t esp_rng_read(struct file *filep, char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rngops =
{
  .read = esp_rng_read,       /* read */
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_rng_read
 *
 * Description:
 *   Fill a buffer with random bytes from hardware RNG.
 *
 * Input Parameters:
 *   filep         - Pointer to a file structure instance.
 *   buffer        - Pointer to buffer to fill with random numbers.
 *   buflen        - Length of buffer in bytes.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static ssize_t esp_rng_read(struct file *filep, char *buffer, size_t buflen)
{
  UNUSED(filep);

  esp_fill_random(buffer, buflen);

  return (ssize_t)buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  register_driver("/dev/random", &g_rngops, 0444, NULL);
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/urandom driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
  register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif
