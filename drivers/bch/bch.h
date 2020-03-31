/****************************************************************************
 * drivers/bch/bch.h
 *
 *   Copyright (C) 2008-2009, 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __DRIVERS_BCH_BCH_H
#define __DRIVERS_BCH_BCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define bchlib_semgive(d) nxsem_post(&(d)->sem)  /* To match bchlib_semtake */
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
  sem_t sem;               /* For atomic accesses to this structure */
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

EXTERN int  bchlib_semtake(FAR struct bchlib_s *bch);
EXTERN int  bchlib_flushsector(FAR struct bchlib_s *bch);
EXTERN int  bchlib_readsector(FAR struct bchlib_s *bch, size_t sector);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __DRIVERS_BCH_BCH_H */
