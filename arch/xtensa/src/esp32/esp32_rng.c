/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rng.c
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
#include <nuttx/semaphore.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include <arch/xtensa/core_macros.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/wdev_reg.h"
#include "esp32_clockconfig.h"

#if defined(CONFIG_ESP32_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32_rng_initialize(void);
static ssize_t esp32_rng_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static int esp32_rng_open(FAR struct file *filep);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  uint8_t *rd_buf;
  sem_t    rd_sem;         /* semaphore for read RNG data */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev;

static const struct file_operations g_rngops =
{
  .open  = esp32_rng_open,       /* open */
  .read  = esp32_rng_read,       /* read */
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_random
 ****************************************************************************/

uint32_t IRAM_ATTR esp_random(void)
{
  /* The PRNG which implements WDEV_RANDOM register gets 2 bits
   * of extra entropy from a hardware randomness source every APB clock cycle
   * (provided WiFi or BT are enabled). To make sure entropy is not drained
   * faster than it is added, this function needs to wait for at least 16 APB
   * clock cycles after reading previous word. This implementation may
   * actually wait a bit longer due to extra time spent in arithmetic and
   * branch statements.
   *
   * As a (probably unncessary) precaution to avoid returning the
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
  while (ccount - last_ccount < cpu_to_apb_freq_ratio * 16);

  last_ccount = ccount;
  return result ^ getreg32(WDEV_RND_REG);
}

/****************************************************************************
 * Name: esp32_rng_open
 ****************************************************************************/

static void esp32_rng_start(void)
{
  /* Nothing to do, bootloader already did it */
}

static void esp32_rng_stop(void)
{
  /* Nothing to do */
}

static int esp32_rng_initialize(void)
{
  static bool first_flag = true;

  if (false == first_flag)
    {
      return OK;
    }

  first_flag = false;

  _info("Initializing RNG\n");

  memset(&g_rngdev, 0, sizeof(struct rng_dev_s));

  nxsem_init(&g_rngdev.rd_sem, 0, 1);
  nxsem_set_protocol(&g_rngdev.rd_sem, SEM_PRIO_NONE);

  esp32_rng_stop();

  return OK;
}

/****************************************************************************
 * Name: esp32_rng_open
 ****************************************************************************/

static int esp32_rng_open(FAR struct file *filep)
{
  /* O_NONBLOCK is not supported */

  if (filep->f_oflags & O_NONBLOCK)
    {
      _err("ESP32 RNG didn't support O_NONBLOCK mode.\n");
      return -EPERM;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_rng_read
 ****************************************************************************/

static ssize_t esp32_rng_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct rng_dev_s *priv = (struct rng_dev_s *)&g_rngdev;
  ssize_t read_len;
  uint8_t *rd_buf = (uint8_t *)buffer;

  if (nxsem_wait(&priv->rd_sem) != OK)
    {
      return -EBUSY;
    }

  read_len = buflen;

  /* start RNG and Wait until the buffer is filled */

  esp32_rng_start();

  /* Now, got data */

  while (buflen > 0)
    {
      uint32_t word = esp_random();
      uint32_t to_copy = MIN(sizeof(word), buflen);

      memcpy(rd_buf, &word, to_copy);
      rd_buf += to_copy;
      buflen -= to_copy;
    }

  /* Release rd_sem for next read */

  nxsem_post(&priv->rd_sem);

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
  esp32_rng_initialize();
  register_driver("/dev/random", FAR & g_rngops, 0444, NULL);
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
#ifndef CONFIG_DEV_RANDOM
  esp32_rng_initialize();
#endif
  register_driver("dev/urandom", FAR & g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_ESP32_RNG */
