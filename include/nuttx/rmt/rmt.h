/****************************************************************************
 * include/nuttx/rmt/rmt.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_RMT_RMT_H
#define __INCLUDE_NUTTX_RMT_RMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_RMT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rmt_dev_s;

/* The RMT peripheral vtable */

struct rmt_ops_s
{
  CODE int      (*open)(FAR struct rmt_dev_s *dev);
  CODE int      (*close)(FAR struct rmt_dev_s *dev);
  CODE ssize_t  (*write)(FAR struct rmt_dev_s *dev,
                         FAR const char *buffer,
                         size_t buflen);
  CODE ssize_t  (*read)(FAR struct rmt_dev_s *dev,
                        FAR char *buffer,
                        size_t buflen);
};

/* RMT private data.  This structure only defines the initial fields of the
 * structure visible to the RMT client.  The specific implementation may
 * add additional, device-specific fields.
 */

struct rmt_dev_s
{
  FAR const struct rmt_ops_s *ops;
  FAR struct circbuf_s       *circbuf;
  sem_t                      *recvsem;
  int                         minor;
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

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_RMT */
#endif /* __INCLUDE_NUTTX_RMT_RMT_H */
