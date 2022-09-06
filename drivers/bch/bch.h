/****************************************************************************
 * drivers/bch/bch.h
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

#ifndef __DRIVERS_BCH_BCH_H
#define __DRIVERS_BCH_BCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_OPENCNT       (255)                  /* Limit of uint8_t */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bchlib_s
{
  FAR struct inode *inode; /* I-node of the block driver */
  uint32_t sectsize;       /* The size of one sector on the device */
  size_t nsectors;         /* Number of sectors supported by the device */
  size_t sector;           /* The current sector in the buffer */
  mutex_t lock;            /* For atomic accesses to this structure */
  uint8_t refs;            /* Number of references */
  bool dirty;              /* true: Data has been written to the buffer */
  bool readonly;           /* true: Only read operations are supported */
  bool unlinked;           /* true: The driver has been unlinked */
  FAR uint8_t *buffer;     /* One sector buffer */

#if defined(CONFIG_BCH_ENCRYPTION)
  uint8_t key[CONFIG_BCH_ENCRYPTION_KEY_SIZE];  /* Encryption key */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct file_operations bch_fops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN int  bchlib_flushsector(FAR struct bchlib_s *bch);
EXTERN int  bchlib_readsector(FAR struct bchlib_s *bch, size_t sector);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __DRIVERS_BCH_BCH_H */
