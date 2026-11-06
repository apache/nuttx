/****************************************************************************
 * include/nuttx/mbox/pl320.h
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

#ifndef __INCLUDE_NUTTX_MBOX_PL320_H
#define __INCLUDE_NUTTX_MBOX_PL320_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mbox/mbox.h>
#include <nuttx/wdog.h>
#include <nuttx/circbuf.h>

#ifdef CONFIG_MBOX_PL320

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PL320_MAX_MBOX_NUM          (32)
#define PL320_MAX_CHAN_NUM          (32)
#define PL320_MAX_DATA_BUF_SIZE     (16)
#define PL320_MSG_SIZE              (28)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* For the convenience of indexing channel, we use mbox id as
 * channel id at the same time
 */

struct pl320_chan_s
{
  struct mbox_chan_s     base;  /* Nested structure to allow casting
                                 * as mbox_chan_s */
  uint8_t                mbox;  /* pl320 mailbox id */
  uint8_t                src;   /* pl320 source core */
  uint8_t                dst;   /* pl320 dest core */
  bool                   ctrl;  /* Do channel initial configuration? */

  /* Buffer to hold messages of base.msgbuf */

  uint8_t                *txbuf;
  size_t                 bufsize;
};

struct pl320_config_s
{
  FAR struct pl320_chan_s *chans;     /* Pointer to pl320 channels array */
  size_t                  num_chans;  /* Number of pl320 channels */
  int                     ipcmint;    /* ipcmint of local core */
  int                     irq_num;    /* IRQ num */
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

FAR struct mbox_dev_s *pl320_initialize(
    FAR const struct pl320_config_s *config, uintptr_t reg_base);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MBOX_PL320 */
#endif /* __INCLUDE_NUTTX_MBOX_PL320_H */
