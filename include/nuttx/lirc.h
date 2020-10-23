/****************************************************************************
 * include/nuttx/lirc.h
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

#ifndef __INCLUDE_NUTTX_LIRC_H
#define __INCLUDE_NUTTX_LIRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PULSE_BIT                         0x01000000
#define PULSE_MASK                        0x00FFFFFF

#define LIRC_MODE2_SPACE                  0x00000000
#define LIRC_MODE2_PULSE                  0x01000000
#define LIRC_MODE2_FREQUENCY              0x02000000
#define LIRC_MODE2_TIMEOUT                0x03000000

#define LIRC_VALUE_MASK                   0x00FFFFFF
#define LIRC_MODE2_MASK                   0xFF000000

#define LIRC_SPACE(val)                   (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_SPACE)
#define LIRC_PULSE(val)                   (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_PULSE)
#define LIRC_FREQUENCY(val)               (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_FREQUENCY)
#define LIRC_TIMEOUT(val)                 (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_TIMEOUT)

#define LIRC_VALUE(val)                   ((val)&LIRC_VALUE_MASK)
#define LIRC_MODE2(val)                   ((val)&LIRC_MODE2_MASK)

#define LIRC_IS_SPACE(val)                (LIRC_MODE2(val) == LIRC_MODE2_SPACE)
#define LIRC_IS_PULSE(val)                (LIRC_MODE2(val) == LIRC_MODE2_PULSE)
#define LIRC_IS_FREQUENCY(val)            (LIRC_MODE2(val) == LIRC_MODE2_FREQUENCY)
#define LIRC_IS_TIMEOUT(val)              (LIRC_MODE2(val) == LIRC_MODE2_TIMEOUT)

/* used heavily by lirc userspace */

#define lirc_t                            int

/* lirc compatible hardware features */

#define LIRC_MODE2SEND(x)                 (x)
#define LIRC_SEND2MODE(x)                 (x)
#define LIRC_MODE2REC(x)                  ((x) << 16)
#define LIRC_REC2MODE(x)                  ((x) >> 16)

#define LIRC_MODE_RAW                     0x00000001
#define LIRC_MODE_PULSE                   0x00000002
#define LIRC_MODE_MODE2                   0x00000004
#define LIRC_MODE_SCANCODE                0x00000008
#define LIRC_MODE_LIRCCODE                0x00000010

#define LIRC_CAN_SEND_RAW                 LIRC_MODE2SEND(LIRC_MODE_RAW)
#define LIRC_CAN_SEND_PULSE               LIRC_MODE2SEND(LIRC_MODE_PULSE)
#define LIRC_CAN_SEND_MODE2               LIRC_MODE2SEND(LIRC_MODE_MODE2)
#define LIRC_CAN_SEND_SCANCODE            LIRC_MODE2SEND(LIRC_MODE_SCANCODE)
#define LIRC_CAN_SEND_LIRCCODE            LIRC_MODE2SEND(LIRC_MODE_LIRCCODE)

#define LIRC_CAN_SEND_MASK                0x0000003f

#define LIRC_CAN_SET_SEND_CARRIER         0x00000100
#define LIRC_CAN_SET_SEND_DUTY_CYCLE      0x00000200
#define LIRC_CAN_SET_TRANSMITTER_MASK     0x00000400

#define LIRC_CAN_REC_RAW                  LIRC_MODE2REC(LIRC_MODE_RAW)
#define LIRC_CAN_REC_PULSE                LIRC_MODE2REC(LIRC_MODE_PULSE)
#define LIRC_CAN_REC_MODE2                LIRC_MODE2REC(LIRC_MODE_MODE2)
#define LIRC_CAN_REC_SCANCODE             LIRC_MODE2REC(LIRC_MODE_SCANCODE)
#define LIRC_CAN_REC_LIRCCODE             LIRC_MODE2REC(LIRC_MODE_LIRCCODE)

#define LIRC_CAN_REC_MASK                 LIRC_MODE2REC(LIRC_CAN_SEND_MASK)

#define LIRC_CAN_SET_REC_CARRIER          (LIRC_CAN_SET_SEND_CARRIER << 16)
#define LIRC_CAN_SET_REC_DUTY_CYCLE       (LIRC_CAN_SET_SEND_DUTY_CYCLE << 16)

#define LIRC_CAN_SET_REC_DUTY_CYCLE_RANGE 0x40000000
#define LIRC_CAN_SET_REC_CARRIER_RANGE    0x80000000
#define LIRC_CAN_GET_REC_RESOLUTION       0x20000000
#define LIRC_CAN_SET_REC_TIMEOUT          0x10000000
#define LIRC_CAN_SET_REC_FILTER           0x08000000

#define LIRC_CAN_MEASURE_CARRIER          0x02000000
#define LIRC_CAN_USE_WIDEBAND_RECEIVER    0x04000000

#define LIRC_CAN_SEND(x)                  ((x)&LIRC_CAN_SEND_MASK)
#define LIRC_CAN_REC(x)                   ((x)&LIRC_CAN_REC_MASK)

#define LIRC_CAN_NOTIFY_DECODE            0x01000000

/* IOCTL commands for lirc driver */

#define LIRC_GET_FEATURES                 _RCIOC(0x0000)
#define LIRC_GET_SEND_MODE                _RCIOC(0x0001)
#define LIRC_GET_REC_MODE                 _RCIOC(0x0002)
#define LIRC_GET_REC_RESOLUTION           _RCIOC(0x0007)
#define LIRC_GET_MIN_TIMEOUT              _RCIOC(0x0008)
#define LIRC_GET_MAX_TIMEOUT              _RCIOC(0x0009)

/* code length in bits, currently only for LIRC_MODE_LIRCCODE */

#define LIRC_GET_LENGTH                   _RCIOC(0x000f)
#define LIRC_SET_SEND_MODE                _RCIOC(0x0011)
#define LIRC_SET_REC_MODE                 _RCIOC(0x0012)

/* Note: these can reset the according pulse_width */

#define LIRC_SET_SEND_CARRIER             _RCIOC(0x0013)
#define LIRC_SET_REC_CARRIER              _RCIOC(0x0014)
#define LIRC_SET_SEND_DUTY_CYCLE          _RCIOC(0x0015)
#define LIRC_SET_TRANSMITTER_MASK         _RCIOC(0x0017)

/* when a timeout != 0 is set the driver will send a
 * LIRC_MODE2_TIMEOUT data packet, otherwise LIRC_MODE2_TIMEOUT is
 * never sent, timeout is disabled by default
 */

#define LIRC_SET_REC_TIMEOUT              _RCIOC(0x0018)

/* 1 enables, 0 disables timeout reports in MODE2 */

#define LIRC_SET_REC_TIMEOUT_REPORTS      _RCIOC(0x0019)

/* if enabled from the next key press on the driver will send
 * LIRC_MODE2_FREQUENCY packets
 */

#define LIRC_SET_MEASURE_CARRIER_MODE     _RCIOC(0x001d)

/* to set a range use LIRC_SET_REC_CARRIER_RANGE with the
 * lower bound first and later LIRC_SET_REC_CARRIER with the upper bound
 */

#define LIRC_SET_REC_CARRIER_RANGE        _RCIOC(0x001f)

#define LIRC_SET_WIDEBAND_RECEIVER        _RCIOC(0x0023)

/* Return the recording timeout, which is either set by
 * the ioctl LIRC_SET_REC_TIMEOUT or by the kernel after setting
 * the protocols.
 */

#define LIRC_GET_REC_TIMEOUT              _RCIOC(0x0024)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* struct lirc_scancode - decoded scancode with protocol for use with
 * LIRC_MODE_SCANCODE
 *
 * timestamp: Timestamp in nanoseconds using CLOCK_MONOTONIC when IR
 *     was decoded.
 * flags: should be 0 for transmit. When receiving scancodes,
 *     LIRC_SCANCODE_FLAG_TOGGLE or LIRC_SCANCODE_FLAG_REPEAT can be set
 *     depending on the protocol
 * rc_proto: see enum rc_proto
 * keycode: the translated keycode. Set to 0 for transmit.
 * scancode: the scancode received or to be sent
 */

struct lirc_scancode
{
  uint64_t timestamp;
  uint16_t flags;
  uint16_t rc_proto;
  uint32_t keycode;
  uint64_t scancode;
};

/* Set if the toggle bit of rc-5 or rc-6 is enabled */

#define LIRC_SCANCODE_FLAG_TOGGLE         1

/* Set if this is a nec or sanyo repeat */

#define LIRC_SCANCODE_FLAG_REPEAT         2

/* enum rc_proto - the Remote Controller protocol
 *
 * RC_PROTO_UNKNOWN: Protocol not known
 * RC_PROTO_OTHER: Protocol known but proprietary
 * RC_PROTO_RC5: Philips RC5 protocol
 * RC_PROTO_RC5X_20: Philips RC5x 20 bit protocol
 * RC_PROTO_RC5_SZ: StreamZap variant of RC5
 * RC_PROTO_JVC: JVC protocol
 * RC_PROTO_SONY12: Sony 12 bit protocol
 * RC_PROTO_SONY15: Sony 15 bit protocol
 * RC_PROTO_SONY20: Sony 20 bit protocol
 * RC_PROTO_NEC: NEC protocol
 * RC_PROTO_NECX: Extended NEC protocol
 * RC_PROTO_NEC32: NEC 32 bit protocol
 * RC_PROTO_SANYO: Sanyo protocol
 * RC_PROTO_MCIR2_KBD: RC6-ish MCE keyboard
 * RC_PROTO_MCIR2_MSE: RC6-ish MCE mouse
 * RC_PROTO_RC6_0: Philips RC6-0-16 protocol
 * RC_PROTO_RC6_6A_20: Philips RC6-6A-20 protocol
 * RC_PROTO_RC6_6A_24: Philips RC6-6A-24 protocol
 * RC_PROTO_RC6_6A_32: Philips RC6-6A-32 protocol
 * RC_PROTO_RC6_MCE: MCE (Philips RC6-6A-32 subtype) protocol
 * RC_PROTO_SHARP: Sharp protocol
 * RC_PROTO_XMP: XMP protocol
 * RC_PROTO_CEC: CEC protocol
 * RC_PROTO_IMON: iMon Pad protocol
 * RC_PROTO_RCMM12: RC-MM protocol 12 bits
 * RC_PROTO_RCMM24: RC-MM protocol 24 bits
 * RC_PROTO_RCMM32: RC-MM protocol 32 bits
 * RC_PROTO_XBOX_DVD: Xbox DVD Movie Playback Kit protocol
 */

enum rc_proto
{
  RC_PROTO_UNKNOWN      = 0,
  RC_PROTO_OTHER        = 1,
  RC_PROTO_RC5          = 2,
  RC_PROTO_RC5X_20      = 3,
  RC_PROTO_RC5_SZ       = 4,
  RC_PROTO_JVC          = 5,
  RC_PROTO_SONY12       = 6,
  RC_PROTO_SONY15       = 7,
  RC_PROTO_SONY20       = 8,
  RC_PROTO_NEC          = 9,
  RC_PROTO_NECX         = 10,
  RC_PROTO_NEC32        = 11,
  RC_PROTO_SANYO        = 12,
  RC_PROTO_MCIR2_KBD    = 13,
  RC_PROTO_MCIR2_MSE    = 14,
  RC_PROTO_RC6_0        = 15,
  RC_PROTO_RC6_6A_20    = 16,
  RC_PROTO_RC6_6A_24    = 17,
  RC_PROTO_RC6_6A_32    = 18,
  RC_PROTO_RC6_MCE      = 19,
  RC_PROTO_SHARP        = 20,
  RC_PROTO_XMP          = 21,
  RC_PROTO_CEC          = 22,
  RC_PROTO_IMON         = 23,
  RC_PROTO_RCMM12       = 24,
  RC_PROTO_RCMM24       = 25,
  RC_PROTO_RCMM32       = 26,
  RC_PROTO_XBOX_DVD     = 27,
};

#endif
