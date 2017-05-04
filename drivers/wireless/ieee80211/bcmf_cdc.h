/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_cdc.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_CDC_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_CDC_H

#include "bcmf_driver.h"
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Send safe cdc request */

int bcmf_cdc_iovar_request(FAR struct bcmf_dev_s *priv, uint32_t ifidx,
                           bool set, char *name, uint8_t *data, uint32_t *len);

int bcmf_cdc_ioctl(FAR struct bcmf_dev_s *priv, uint32_t ifidx, bool set,
                   uint32_t cmd, uint8_t *data, uint32_t *len);

/* Send cdc request without locking control_mutex */

int bcmf_cdc_iovar_request_unsafe(FAR struct bcmf_dev_s *priv, uint32_t ifidx,
                           bool set, char *name, uint8_t *data, uint32_t *len);

/* Callback used by bus layer to notify cdc response frame is available */

int bcmf_cdc_process_control_frame(FAR struct bcmf_dev_s *priv,
                                   struct bcmf_frame_s *frame);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_CDC_H */