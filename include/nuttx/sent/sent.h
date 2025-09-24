/****************************************************************************
 * include/nuttx/sent/sent.h
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

#ifndef __INCLUDE_NUTTX_SENT_SENT_H
#define __INCLUDE_NUTTX_SENT_SENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SENTIOC_TRIGGER_SPC        _SENTIOC(0x0001)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The sent frame data structure */

enum sent_frame_type_e
{
  SENT_FRAME_SHORT,
  SENT_FRAME_ENH4,
  SENT_FRAME_ENH8,
  SENT_FRAME_FAST,
};

struct sent_frame_shrt_s
{
  uint16_t id : 4;
  uint16_t data : 8;
  uint16_t crc : 4;
};

struct sent_frame_enh4_s
{
  uint32_t id : 4;
  uint32_t data : 16;
  uint32_t crc : 6;
};

struct sent_frame_enh8_s
{
  uint32_t id : 8;
  uint32_t data : 12;
  uint32_t crc : 6;
};

struct sent_frame_fast_s
{
  uint32_t status : 4;
  uint32_t data : 24;
  uint32_t crc : 4;
};

struct sent_frame_s
{
  enum sent_frame_type_e type;
  union
  {
    struct sent_frame_shrt_s shrt;
    struct sent_frame_enh4_s enh4;
    struct sent_frame_enh8_s enh8;
    struct sent_frame_fast_s fast;
  };

  uint64_t timestamp;
};

/* This structure is the generic form of state structure used by lower half
 * Sent driver.
 */

typedef CODE void
(*sent_push_frame_t)(FAR void *priv,
                     FAR const struct sent_frame_s *frame);

/* The sent lower half driver interface */

struct sent_lowerhalf_s;
struct sent_ops_s
{
/****************************************************************************
 * Name: activate
 *
 * Description:
 *   Enable or disable the SENT device. This function allows the upper-half
 *   driver to control the operational state of the SENT peripheral at
 *   runtime.
 *
 *   When 'push_frame' is provided (non-NULL), the SENT device should be
 *   powered on, initialized, and configured to receive SENT frames. The
 *   lower-half driver will invoke this callback whenever a valid SENT frame
 *   is received.
 *
 *   When 'push_frame' is NULL, the SENT device should be powered down and
 *   disabled to save energy and release hardware resources.
 *
 *   The exact behavior depends on the hardware implementation. If the
 *   requested operation is not supported or fails, an appropriate error
 *   code must be returned.
 *
 * Input Parameters:
 *   lower      - Pointer to the instance of the SENT lower-half driver
 *   push_frame - Callback function to push received SENT frames to the
 *                upper-half driver. Pass NULL to disable reception and
 *                power down the device.
 *   priv       - Opaque pointer to the upper-half driver private context.
 *                This will be passed to the push_frame callback.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *   A negated errno value is returned on any failure.
 *
 ****************************************************************************/

  CODE int (*activate)(FAR struct sent_lowerhalf_s *lower,
                       sent_push_frame_t push_frame,
                       FAR void *priv);

/****************************************************************************
 * Name: trigger_spc
 *
 * Description:
 *   Trigger a SENT SPC (Serial Peripheral Communication) transaction on the
 *   specified SENT channel. This function is typically used by the
 *   upper-half driver or application to initiate a command or data request
 *   to the sensor via the SPC back channel defined in the SENT
 *   protocol (SAE J2716).
 *
 *   In SENT with SPC mode, the ECU communicates with the sensor by pulling
 *   the SENT line low for a specific duration after a valid SENT frame. This
 *   function initiates that timing sequence at the hardware level to send an
 *   SPC command or request, allowing bidirectional communication with the
 *   sensor.
 *
 * Input Parameters:
 *   lower - Pointer to the instance of the SENT lower-half driver that
 *           provides access to the hardware-specific implementation of
 *           the SPC mechanism.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
  ***************************************************************************/

  CODE int (*trigger_spc)(FAR struct sent_lowerhalf_s *lower);

  /**************************************************************************
   * Name: control
   *
   * With this method, the user can set some special config for the sent,
   * which are all parsed and implemented by lower half driver.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sent driver.
   *   cmd        - The special cmd for sent.
   *   arg        - The parameters associated with cmd.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *   -ENOTTY    - The cmd don't support.
   *
   **************************************************************************/

  CODE int (*control)(FAR struct sent_lowerhalf_s *lower,
                      int cmd, unsigned long arg);
};

struct sent_lowerhalf_s
{
  /* The lower half sent driver operations */

  FAR const struct sent_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sent_register
 *
 * Description:
 *   This function is used to register a SENT device driver instance.
 *   It creates and initializes the upper-half driver structure,
 *   links it with the provided lower-half driver, and registers the
 *   device node under the /dev directory (e.g., /dev/sent0_channel0).
 *
 *   The function allocates and initializes the necessary resources,
 *   including spin locks and a circular buffer for data reception.
 *   If registration succeeds, the SENT device can then be accessed
 *   via standard file  operations defined in g_sent_fops.
 *
 * Input Parameters:
 *   path    - The device path to register (e.g., "/dev/sent0_channel0").
 *   lower   - Pointer to the lower-half driver structure that provides the
 *             hardware-specific implementations and callbacks for the SENT
 *             peripheral.
 *
 * Returned Value:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sent_register(FAR const char *path, FAR struct sent_lowerhalf_s *lower);

#endif
