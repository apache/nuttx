/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_rng.c
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
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include <arch/xtensa/core_macros.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/wdev_reg.h"
#include "esp32s3_clockconfig.h"

#if defined(CONFIG_ESP32S3_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t esp32s3_rng_read(struct file *filep, char *buffer,
                                size_t buflen);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  uint8_t *rd_buf;
  mutex_t  rd_lock;        /* mutex for read RNG data */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev =
{
  .rd_lock = NXMUTEX_INITIALIZER,
};

static const struct file_operations g_rngops =
{
  .read = esp32s3_rng_read,       /* read */
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_random
 *
 * Description:
 *   Read a random number (unsigned int 32-bit) from ESP32S3 PRNG.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   A uint32_t random number.
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp_random(void)
{
  /* The PRNG which implements WDEV_RANDOM register gets 2 bits
   * of extra entropy from a hardware randomness source every APB clock cycle
   * (provided Wi-Fi or BT are enabled). To make sure entropy is not drained
   * faster than it is added, this function needs to wait for at least 1778
   * APB clock cycles after reading previous word. This implementation may
   * actually wait a bit longer due to extra time spent in arithmetic and
   * branch statements.
   *
   * As a (probably unnecessary) precaution to avoid returning the
   * RNG state as-is, the result is XORed with additional
   * WDEV_RND_REG reads while waiting.
   */

  uint32_t cpu_to_apb_freq_ratio = esp_clk_cpu_freq() / esp_clk_apb_freq();

  static uint32_t last_ccount = 0;
  uint32_t ccount;
  uint32_t result = 0;

  do
    {
      ccount = XTHAL_GET_CCOUNT();
      result ^= getreg32(WDEV_RND_REG);
    }
  while (ccount - last_ccount < cpu_to_apb_freq_ratio * 1778);

  last_ccount = ccount;
  return result ^ getreg32(WDEV_RND_REG);
}

/****************************************************************************
 * Name: esp32s3_rng_read
 *
 * Description:
 *   Read a certain amount of random data.
 *
 * Input Parameters:
 *   filep  - File structure pointer
 *   buffer - Buffer to write data coming from RNG device
 *   buflen - Amount of bytes to read
 *
 * Returned Value:
 *   The number of random bytes generated.
 *
 ****************************************************************************/

static ssize_t esp32s3_rng_read(struct file *filep, char *buffer,
                                size_t buflen)
{
  struct rng_dev_s *priv = (struct rng_dev_s *)&g_rngdev;
  ssize_t read_len;
  uint8_t *rd_buf = (uint8_t *)buffer;

  if (nxmutex_lock(&priv->rd_lock) != OK)
    {
      return -EBUSY;
    }

  read_len = buflen;

  /* Wait until the buffer is filled */

  while (buflen > 0)
    {
      uint32_t word = esp_random();
      uint32_t to_copy = MIN(sizeof(word), buflen);

      memcpy(rd_buf, &word, to_copy);
      rd_buf += to_copy;
      buflen -= to_copy;
    }

  /* Release rd_lock for next read */

  nxmutex_unlock(&priv->rd_lock);
  return read_len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *   Must be called BEFORE devurandom_register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
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
 *   Register /dev/urandom
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
  register_driver("dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_ESP32S3_RNG */
