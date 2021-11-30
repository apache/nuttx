/****************************************************************************
 * include/nuttx/random.h
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

#ifndef __INCLUDE_NUTTX_RANDOM_H
#define __INCLUDE_NUTTX_RANDOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size of entropy pool in 32-bit integers, must be power of two */

#define ENTROPY_POOL_SIZE        128

#define add_irq_randomness(x)    up_rngaddint(RND_SRC_IRQ, (x))
#define add_sensor_randomness(x) up_rngaddint(RND_SRC_SENSOR, (x))
#define add_time_randomness(x)   up_rngaddint(RND_SRC_TIME, (x))
#define add_hw_randomness(x)     up_rngaddint(RND_SRC_HW, (x))
#define add_sw_randomness(x)     up_rngaddint(RND_SRC_SW, (x))
#define add_ui_randomness(x)     up_rngaddint(RND_SRC_UI, (x))

/* Allow above macros to always exist in source without ifdefs */

#ifndef CONFIG_CRYPTO_RANDOM_POOL
#  define up_rngaddint(k, x)            ((void)(k),(void)(x))
#  define up_rngaddentropy(k, buf, x)   ((void)(k),(void)(buf),(void)(x))
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Entropy pool structure */

struct entropy_pool_s
{
  volatile uint32_t pool[ENTROPY_POOL_SIZE];
};

/* Randomness sources */

enum rnd_source_t
{
  RND_SRC_TIME = 0,
  RND_SRC_IRQ,
  RND_SRC_SENSOR,
  RND_SRC_HW,      /* unique per HW UID or coming from factory line. */
  RND_SRC_SW,      /* unique per SW version. */
  RND_SRC_UI       /* buttons etc. user-visible interface elements. */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BOARD_ENTROPY_POOL
/* Entropy pool structure can be provided by board source. Use for this is,
 * for example, allocate entropy pool from special area of RAM which content
 * is kept over system reset.
 */

extern struct entropy_pool_s board_entropy_pool;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_RANDOM_POOL

/****************************************************************************
 * Name: up_rngaddint
 *
 * Description:
 *   Add one integer to entropy pool, contributing a specific kind
 *   of entropy to pool.
 *
 * Input Parameters:
 *   kindof  - Enumeration constant telling where val came from
 *   val     - Integer to be added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_rngaddint(enum rnd_source_t kindof, int val);

/****************************************************************************
 * Name: up_rngaddentropy
 *
 * Description:
 *   Add buffer of integers to entropy pool.
 *
 * Input Parameters:
 *   kindof  - Enumeration constant telling where val came from
 *   buf     - Buffer of integers to be added
 *   n       - Number of elements in buf
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_rngaddentropy(enum rnd_source_t kindof, FAR const uint32_t *buf,
                      size_t n);

/****************************************************************************
 * Name: up_rngreseed
 *
 * Description:
 *   Force reseeding random number generator from entropy pool
 *
 ****************************************************************************/

void up_rngreseed(void);

/****************************************************************************
 * Name: up_randompool_initialize
 *
 * Description:
 *   Initialize entropy pool and random number generator
 *
 ****************************************************************************/

void up_randompool_initialize(void);

#endif /* CONFIG_CRYPTO_RANDOM_POOL */

#endif /* __INCLUDE_NUTTX_RANDOM_H */
