/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_driver.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_DRIVER_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_DRIVER_H

#include <stdbool.h>
#include <nuttx/sdio.h>

/* This structure contains the unique state of the Broadcom FullMAC driver */

struct bcmf_dev_s
{
  FAR struct sdio_dev_s *sdio_dev; /* The SDIO device bound to this instance */
  int minor;                       /* Device minor number */

  uint32_t backplane_current_addr; /* Current function 1 backplane base addr */

  uint32_t (*get_core_base_address)(unsigned int core); /* Get chip specific
                                      base address for evey cores */

  sem_t sem;                       /* Semaphore for processing thread event */
  struct wdog_s *waitdog;          /* Processing thread waitdog */
  bool ready;                      /* Current device status */
  bool sleeping;                   /* Current sleep status */
  volatile bool irq_pending;       /* True if interrupt is pending */
  uint32_t intstatus;              /* Copy of device current interrupt status */

  uint8_t max_seq;                 /* Maximum transmit sequence allowed */
  uint8_t tx_seq;                  /* Transmit sequence number (next) */
  uint8_t rx_seq;                  /* Receive sequence number (expected) */
};

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_DRIVER_H */
