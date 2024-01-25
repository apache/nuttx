/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_sdio_dev.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_SDIO_DEV_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_SDIO_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sdio.h>
#include <stdbool.h>
#include <sys/types.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure defines the state of the MPFS eMMCSD interface */

struct mpfs_dev_s
{
  struct sdio_dev_s dev; /* Standard, base SDIO interface */

  const uintptr_t hw_base; /* Base address */
  const int plic_irq;      /* PLIC interrupt */
  bool clk_enabled;        /* Clk state */

  /* eMMC / SD and HW parameters */

  const bool emmc;  /* eMMC or SD */
  int bus_voltage;  /* Bus voltage */
  int bus_mode;     /* eMMC Bus mode */
  bool jumpers_3v3; /* Jumper settings: 1v8 or 3v3 */

  /* Event support */

  sem_t waitsem;                      /* Implements event waiting */
  sdio_eventset_t waitevents;         /* Set of events to be waited for */
  uint32_t waitmask;                  /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s waitwdog;             /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t cdstatus;  /* Card status */
  sdio_eventset_t cbevents; /* Set of events to be cause callbacks */
  worker_t callback;        /* Registered callback function */
  void *cbarg;              /* Registered callback argument */
  struct work_s cbwork;     /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t *buffer;     /* Address of current R/W buffer */
  size_t remaining;     /* Number of bytes remaining in the transfer */
  size_t receivecnt;    /* Real count to receive */
  uint32_t xfrmask;     /* Interrupt enables for data transfer */
  uint32_t xfr_blkmask; /* Interrupt enables for SB/MB data transfer */

  bool widebus; /* Required for DMA support */
  bool onebit;  /* true: Only 1-bit transfers are supported */

  /* DMA data transfer support */

  bool polltransfer; /* Indicate a poll transfer, no DMA */
  bool multiblock;   /* Indicate a multi-block transfer */

  /* Misc */

  uint32_t blocksize;  /* Current block size */
  uint32_t fifo_depth; /* Fifo size, read from the register */
};

#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_SDIO_DEV_H */
