/****************************************************************************
 *  arch/x86_64/src/intel64/intel64_rng.c
 *
 *   Copyright (C) 2012 Max Holtzberg,
 *                 2020 Chung-Fan Yang.
 *   All rights reserved.
 *
 *   Author: Max Holtzberg <mh@uvc.de>
 *           Chung-Fan Yang <sonic.tw.tp@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include <arch/io.h>

#include "up_internal.h"

#include <immintrin.h>


#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int x86_rng_initialize(void);
static ssize_t x86_rngread(struct file *filep, char *buffer, size_t);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  sem_t rd_devsem;      /* Threads can only exclusively access the RNG */
  sem_t rd_readsem;     /* To block until the buffer is filled NOT used  */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev;

static const struct file_operations g_rngops =
{
  0,               /* open */
  0,               /* close */
  x86_rngread,   /* read */
  0,               /* write */
  0,               /* seek */
  0                /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0              /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0              /* unlink */
#endif
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_rng_initialize
 ****************************************************************************/

static int x86_rng_initialize(void)
{
  _info("Initializing RNG\n");

  memset(&g_rngdev, 0, sizeof(struct rng_dev_s));

  nxsem_init(&g_rngdev.rd_devsem, 0, 1);

  return OK;
}

/****************************************************************************
 * Name: x86_rngread
 ****************************************************************************/

static ssize_t x86_rngread(struct file *filep, char *buffer, size_t buflen)
{
  for(; buflen > 8; buflen -= 8)
    {
      while(_rdrand64_step((unsigned long long*)buffer))
        sched_yield();
      buffer += 8;
    }

  for(; buflen > 4; buflen -= 4)
    {
      while(_rdrand32_step((unsigned int*)buffer))
        sched_yield();
      buffer += 4;
    }

  for(; buflen > 2; buflen -= 2)
    {
      while(_rdrand16_step((unsigned short*)buffer))
        sched_yield();
      buffer += 2;
    }

  if(buflen != 0)
    {
      unsigned short temp = 0;
      while(_rdrand16_step((unsigned short*)temp))
        sched_yield();
      *buffer = (temp & 0xFF);
    }

  return buflen;
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
  x86_rng_initialize();
  (void)register_driver("/dev/random", &g_rngops, 0444, NULL);
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
  x86_rng_initialize();
#endif
  (void)register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
