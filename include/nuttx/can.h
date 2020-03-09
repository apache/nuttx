/************************************************************************************
 * include/nuttx/can/can.h
 *
 *   Copyright (C) 2008, 2009, 2011-2012, 2015-2017, 2019 Gregory Nutt. All rights
 *     reserved.
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_CAN_CAN_H
#define __INCLUDE_NUTTX_CAN_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifdef CONFIG_CAN_TXREADY
#  include <nuttx/wqueue.h>
#endif

#include <queue.h>

#ifdef CONFIG_NET_CAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Ioctl Commands *******************************************************************/

/* Ioctl commands supported by the upper half CAN driver.
 *
 * CANIOC_RTR:
 *   Description:  Send the remote transmission request and wait for the response.
 *   Argument:     A reference to struct canioc_rtr_s
 *
 * Ioctl commands that may or may not be supported by the lower half CAN driver.
 *
 * CANIOC_ADD_STDFILTER:
 *   Description:    Add an address filter for a standard 11 bit address.
 *   Argument:       A reference to struct canioc_stdfilter_s
 *   Returned Value: A non-negative filter ID is returned on success.
 *                   Otherwise -1 (ERROR) is returned with the errno
 *                   variable set to indicate the nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_ADD_EXTFILTER:
 *   Description:    Add an address filter for a extended 29 bit address.
 *   Argument:       A reference to struct canioc_extfilter_s
 *   Returned Value: A non-negative filter ID is returned on success.
 *                   Otherwise -1 (ERROR) is returned with the errno
 *                   variable set to indicate the nature of the error.
 *   Dependencies:   Requires CONFIG_CAN_EXTID=y
 *
 * CANIOC_DEL_STDFILTER:
 *   Description:    Remove an address filter for a standard 11 bit address.
 *   Argument:       The filter index previously returned by the
 *                   CANIOC_ADD_STDFILTER command
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_DEL_EXTFILTER:
 *   Description:    Remove an address filter for a standard 29 bit address.
 *   Argument:       The filter index previously returned by the
 *                   CANIOC_ADD_EXTFILTER command
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   Requires CONFIG_CAN_EXTID=y
 *
 * CANIOC_GET_BITTIMING:
 *   Description:    Return the current bit timing settings
 *   Argument:       A pointer to a write-able instance of struct
 *                   canioc_bittiming_s in which current bit timing values
 *                   will be returned.
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_SET_BITTIMING:
 *   Description:    Set new current bit timing values
 *   Argument:       A pointer to a read-able instance of struct
 *                   canioc_bittiming_s in which the new bit timing values
 *                   are provided.
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_GET_CONNMODES:
 *   Description:    Get the current bus connection modes
 *   Argument:       A pointer to a write-able instance of struct
 *                   canioc_connmodes_s in which the new bus modes will be returned.
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_SET_CONNMODES:
 *   Description:    Set new bus connection modes values
 *   Argument:       A pointer to a read-able instance of struct
 *                   canioc_connmodes_s in which the new bus modes are provided.
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_BUSOFF_RECOVERY:
 *   Description:    Initiates the BUS-OFF recovery sequence
 *   Argument:       None
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 */

#define CANIOC_RTR                _CANIOC(1)
#define CANIOC_GET_BITTIMING      _CANIOC(2)
#define CANIOC_SET_BITTIMING      _CANIOC(3)
#define CANIOC_ADD_STDFILTER      _CANIOC(4)
#define CANIOC_ADD_EXTFILTER      _CANIOC(5)
#define CANIOC_DEL_STDFILTER      _CANIOC(6)
#define CANIOC_DEL_EXTFILTER      _CANIOC(7)
#define CANIOC_GET_CONNMODES      _CANIOC(8)
#define CANIOC_SET_CONNMODES      _CANIOC(9)
#define CANIOC_BUSOFF_RECOVERY    _CANIOC(10)

#define CAN_FIRST                 0x0001         /* First common command */
#define CAN_NCMDS                 10             /* Ten common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half CAN driver to the lower-half CAN driver via the co_ioctl()
 * method fo the CAN lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 *
 * This is generally done as follows.  The first reservation for CAN driver A would
 * look like:
 *
 *   CAN_A_FIRST                 (CAN_FIRST + CAN_NCMDS)     <- First command
 *   CAN_A_NCMDS                 42                          <- Number of commands
 *
 * IOCTL commands for CAN driver A would then be defined in a CAN A header file like:
 *
 *   CANIOC_A_CMD1               _CANIOC(CAN_A_FIRST+0)
 *   CANIOC_A_CMD2               _CANIOC(CAN_A_FIRST+1)
 *   CANIOC_A_CMD3               _CANIOC(CAN_A_FIRST+2)
 *   ...
 *   CANIOC_A_CMD42              _CANIOC(CAN_A_FIRST+41)
 *
 * The next reservation would look like:
 *
 *   CAN_B_FIRST                 (CAN_A_FIRST + CAN_A_NCMDS) <- Next command
 *   CAN_B_NCMDS                 77                          <- Number of commands
 */

/* CAN payload length and DLC definitions according to ISO 11898-1 */

#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

/* CAN FD payload length and DLC definitions according to ISO 11898-7 */

#define CANFD_MAX_DLC 15
#define CANFD_MAX_DLEN 64

/* Defined bits for canfd_frame.flags
 *
 * The use of struct canfd_frame implies the Extended Data Length (EDL) bit to
 * be set in the CAN frame bitstream on the wire. The EDL bit switch turns
 * the CAN controllers bitstream processor into the CAN FD mode which creates
 * two new options within the CAN FD frame specification:
 *
 * Bit Rate Switch - to indicate a second bitrate is/was used for the payload
 * Error State Indicator - represents the error state of the transmitting node
 *
 * As the CANFD_ESI bit is internally generated by the transmitting CAN
 * controller only the CANFD_BRS bit is relevant for real CAN controllers when
 * building a CAN FD frame for transmission. Setting the CANFD_ESI bit can make
 * sense for virtual CAN interfaces to test applications with echoed frames.
 */

#define CANFD_BRS 0x01 /* bit rate switch (second bitrate for payload data) */
#define CANFD_ESI 0x02 /* error state indicator of the transmitting node */

#define CAN_INV_FILTER     0x20000000U /* to be set in can_filter.can_id */

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *CAN_HANDLE;

struct can_response_s
{
  sq_entry_t flink;

  /* Message-specific data may follow */
}; /* FIXME remove */

typedef uint32_t canid_t;

/* Controller Area Network Error Message Frame Mask structure
 *
 * bit 0-28  : error class mask (see include/uapi/linux/can/error.h)
 * bit 29-31 : set to zero
 */

typedef uint32_t can_err_mask_t;

/* struct can_frame - basic CAN frame structure
 * can_id:  CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * can_dlc: frame payload length in byte (0 .. 8) aka data length code
 *          N.B. the DLC field from ISO 11898-1 Chapter 8.4.2.3 has a 1:1
 *          mapping of the 'data length code' to the real payload length
 * __pad:   padding
 * __res0:  reserved / padding
 * __res1:  reserved / padding
 * data:    CAN frame payload (up to 8 byte)
 */

struct can_frame
{
  canid_t can_id;   /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  uint8_t  can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  uint8_t  __pad;   /* padding */
  uint8_t  __res0;  /* reserved / padding */
  uint8_t  __res1;  /* reserved / padding */
  uint8_t  data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};

/* struct canfd_frame - CAN flexible data rate frame structure
 * can_id: CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * len:    frame payload length in byte (0 .. CANFD_MAX_DLEN)
 * flags:  additional flags for CAN FD
 * __res0: reserved / padding
 * __res1: reserved / padding
 * data:   CAN FD frame payload (up to CANFD_MAX_DLEN byte)
 */

struct canfd_frame
{
  canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  uint8_t len;     /* frame payload length in byte */
  uint8_t flags;   /* additional flags for CAN FD */
  uint8_t __res0;  /* reserved / padding */
  uint8_t __res1;  /* reserved / padding */
  uint8_t data[CANFD_MAX_DLEN] __attribute__((aligned(8)));
};

/* struct can_filter - CAN ID based filter in can_register().
 * can_id:   relevant bits of CAN ID which are not masked out.
 * can_mask: CAN mask (see description)
 *
 * Description:
 * A filter matches, when
 *
 *   <received_can_id> & mask == can_id & mask
 *
 * The filter can be inverted (CAN_INV_FILTER bit set in can_id) or it can
 * filter for error message frames (CAN_ERR_FLAG bit set in mask).
 */

struct can_filter
{
  canid_t can_id;
  canid_t can_mask;
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_CAN */
#endif /* __INCLUDE_NUTTX_CAN_CAN_H */
