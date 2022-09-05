/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_gspi_f2_frame.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_GSPI_F2_FRAME_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_GSPI_F2_FRAME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcmf_driver.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_gspi_read_f2_frame
 *
 * Description:
 *    Read and process an F2 frame.
 *
 * Parameters:
 *    priv         - the device structure
 *    frame_length - Length of frame we are to read.  (From chip status)
 *
 * Returns:
 *    OK on success, negated error code on failure.
 ****************************************************************************/

int bcmf_gspi_read_f2_frame(FAR struct bcmf_dev_s *priv,
                            int                    frame_length);

/****************************************************************************
 * Name: bcmf_gspi_send_f2_frame
 *
 * Description:
 *    De-queue and send an F2 frame.
 *
 * Parameters:
 *    priv     - the device structure
 *
 * Returns:
 *    OK on success, negated error code on failure.
 ****************************************************************************/

int bcmf_gspi_send_f2_frame(FAR struct bcmf_dev_s *priv);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_GSPI_F2_FRAME_H */
