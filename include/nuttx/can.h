/****************************************************************************
 * include/nuttx/can.h
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

#ifndef __INCLUDE_NUTTX_CAN_H
#define __INCLUDE_NUTTX_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_CAN_TXREADY
#  include <nuttx/wqueue.h>
#endif

#include <nuttx/queue.h>

#ifdef CONFIG_NET_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ioctl Commands ***********************************************************/

/* Ioctl commands supported by the upper half CAN driver.
 *
 * CANIOC_RTR:
 *   Description:  Send the remote transmission request and wait for the
 *                 response.
 *   Argument:     A reference to struct canioc_rtr_s
 *
 * Ioctl commands that may or may not be supported by the lower half CAN
 * driver.
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
 *                   canioc_connmodes_s in which the new bus modes will be
 *                   returned.
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_SET_CONNMODES:
 *   Description:    Set new bus connection modes values
 *   Argument:       A pointer to a read-able instance of struct
 *                   canioc_connmodes_s in which the new bus modes are
 *                   provided.
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
 * by the upper-half CAN driver to the lower-half CAN driver via the
 * co_ioctl()  method fo the CAN lower-half interface.
 * However, the lower-half driver must reserve a block of commands as follows
 * in order prevent IOCTL command numbers from overlapping.
 *
 * This is generally done as follows.  The first reservation for CAN driver A
 * would look like:
 *
 *   CAN_A_FIRST                (CAN_FIRST + CAN_NCMDS) <- First command
 *   CAN_A_NCMDS                42                      <- Number of commands
 *
 * IOCTL commands for CAN driver A would then be defined in a CAN A header
 * file like:
 *
 *   CANIOC_A_CMD1       _CANIOC(CAN_A_FIRST+0)
 *   CANIOC_A_CMD2       _CANIOC(CAN_A_FIRST+1)
 *   CANIOC_A_CMD3       _CANIOC(CAN_A_FIRST+2)
 *   ...
 *   CANIOC_A_CMD42      _CANIOC(CAN_A_FIRST+41)
 *
 * The next reservation would look like:
 *
 *   CAN_B_FIRST           (CAN_A_FIRST + CAN_A_NCMDS) <- Next command
 *   CAN_B_NCMDS           77                          <- Number of commands
 */

/* CAN payload length and DLC definitions according to ISO 11898-1 */

#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

/* CAN FD payload length and DLC definitions according to ISO 11898-7 */

#define CANFD_MAX_DLC 15
#define CANFD_MAX_DLEN 64

/* Defined bits for canfd_frame.flags
 *
 * The use of struct canfd_frame implies the Extended Data Length (EDL) bit
 * to be set in the CAN frame bitstream on the wire. The EDL bit switch turns
 * the CAN controllers bitstream processor into the CAN FD mode which creates
 * two new options within the CAN FD frame specification:
 *
 * Bit Rate Switch - to indicate a second bitrate is/was used for the payload
 * Error State Indicator - represents the error state of the transmitting
 * node
 *
 * As the CANFD_ESI bit is internally generated by the transmitting CAN
 * controller only the CANFD_BRS bit is relevant for real CAN controllers
 * when building a CAN FD frame for transmission. Setting the CANFD_ESI bit
 * can make sense for virtual CAN interfaces to test applications with echoed
 * frames.
 */

#define CANFD_BRS 0x01 /* bit rate switch (second bitrate for payload data) */
#define CANFD_ESI 0x02 /* error state indicator of the transmitting node */

#define CAN_INV_FILTER     0x20000000U /* to be set in can_filter.can_id */

/* CAN Error Indications ****************************************************/

/* Error class in can_id */

#define CAN_ERR_TXTIMEOUT        (1 << 0) /* Bit 0: TX timeout */
#define CAN_ERR_LOSTARB          (1 << 1) /* Bit 1: Lost arbitration (See CAN_ERR_LOSTARB_* definitions) */
#define CAN_ERR_CRTL             (1 << 2) /* Bit 2: Controller error (See CAN_ERR_CRTL_* definitions) */
#define CAN_ERR_PROT             (1 << 3) /* Bit 3: Protocol error (see CAN_ERR_PROT_* definitions) */
#define CAN_ERR_TRX              (1 << 4) /* Bit 4: Transceiver error (See CAN_TRX_* definitions)    */
#define CAN_ERR_ACK              (1 << 5) /* Bit 5: No ACK received on transmission */
#define CAN_ERR_BUSOFF           (1 << 6) /* Bit 6: Bus off */
#define CAN_ERR_BUSERROR         (1 << 7) /* Bit 7: Bus error */
#define CAN_ERR_RESTARTED        (1 << 8) /* Bit 8: Controller restarted */

/* The remaining definitions described the error report payload that follows
 * the CAN header.
 */

#define CAN_ERR_DLC              (8)      /* DLC of error report */

/* Data[0]: Arbitration lost in ch_error. */

#define CAN_ERR_LOSTARB_UNSPEC   0x00     /* Unspecified error */
#define CAN_ERR_LOSTARB_BIT(n)   (n)      /* Bit number in the bit stream */

/* Data[1]:  Error status of CAN-controller */

#define CAN_ERR_CRTL_UNSPEC      0x00     /* Unspecified error */
#define CAN_ERR_CRTL_RX_OVERFLOW (1 << 0) /* Bit 0: RX buffer overflow */
#define CAN_ERR_CRTL_TX_OVERFLOW (1 << 1) /* Bit 1: TX buffer overflow */
#define CAN_ERR_CRTL_RX_WARNING  (1 << 2) /* Bit 2: Reached warning level for RX errors */
#define CAN_ERR_CRTL_TX_WARNING  (1 << 3) /* Bit 3: Reached warning level for TX errors */
#define CAN_ERR_CRTL_RX_PASSIVE  (1 << 4) /* Bit 4: Reached passive level for RX errors */
#define CAN_ERR_CRTL_TX_PASSIVE  (1 << 5) /* Bit 5: Reached passive level for TX errors */

/* Data[2]:  Error in CAN protocol.  This provides the type of the error. */

#define CAN_ERR_PROT_UNSPEC      0x00     /* Unspecified error */
#define CAN_ERR_PROT_BIT         (1 << 0) /* Bit 0: Single bit error */
#define CAN_ERR_PROT_FORM        (1 << 1) /* Bit 1: Frame format error */
#define CAN_ERR_PROT_STUFF       (1 << 2) /* Bit 2: Bit stuffing error */
#define CAN_ERR_PROT_BIT0        (1 << 3) /* Bit 3: Unable to send dominant bit */
#define CAN_ERR_PROT_BIT1        (1 << 4) /* Bit 4: Unable to send recessive bit */
#define CAN_ERR_PROT_OVERLOAD    (1 << 5) /* Bit 5: Bus overload */
#define CAN_ERR_PROT_ACTIVE      (1 << 6) /* Bit 6: Active error announcement */
#define CAN_ERR_PROT_TX          (1 << 7) /* Bit 7: Error occurred on transmission */

/* Data[3]:  Error in CAN protocol.  This provides the loation of the error */

#define CAN_ERR_PROT_LOC_UNSPEC  0x00 /* Unspecified error */
#define CAN_ERR_PROT_LOC_SOF     0x01 /* start of frame */
#define CAN_ERR_PROT_LOC_ID0     0x02 /* ID bits 0-4 */
#define CAN_ERR_PROT_LOC_ID1     0x03 /* ID bits 5-12 */
#define CAN_ERR_PROT_LOC_ID2     0x04 /* ID bits 13-17 */
#define CAN_ERR_PROT_LOC_ID3     0x05 /* ID bits 21-28 */
#define CAN_ERR_PROT_LOC_ID4     0x06 /* ID bits 18-20 */
#define CAN_ERR_PROT_LOC_IDE     0x07 /* Identifier extension */
#define CAN_ERR_PROT_LOC_RTR     0x08 /* RTR */
#define CAN_ERR_PROT_LOC_SRTR    0x09 /* Substitute RTR */
#define CAN_ERR_PROT_LOC_RES0    0x0a /* Reserved bit 0 */
#define CAN_ERR_PROT_LOC_RES1    0x0b /* Reserved bit 1 */
#define CAN_ERR_PROT_LOC_DLC     0x0c /* Data length code */
#define CAN_ERR_PROT_LOC_DATA    0x0d /* Data section */
#define CAN_ERR_PROT_LOC_CRCSEQ  0x0e /* CRC sequence */
#define CAN_ERR_PROT_LOC_CRCDEL  0x0f /* CRC delimiter */
#define CAN_ERR_PROT_LOC_ACK     0x10 /* ACK slot */
#define CAN_ERR_PROT_LOC_ACKDEL  0x11 /* ACK delimiter */
#define CAN_ERR_PROT_LOC_EOF     0x12 /* End of frame */
#define CAN_ERR_PROT_LOC_INTERM  0x13 /* Intermission */

/* Data[4]: Error status of CAN-transceiver */

#define CAN_ERR_TRX_UNSPEC       0x00     /* Unspecified error */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef FAR void *CAN_HANDLE;

struct can_response_s
{
  sq_entry_t flink;

  /* Message-specific data may follow */
}; /* FIXME remove */

typedef uint32_t canid_t;

/* Controller Area Network Error Message Frame Mask structure
 *
 * bit 0-28  : error class mask
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
  uint8_t  data[CAN_MAX_DLEN] aligned_data(8);
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
  uint8_t data[CANFD_MAX_DLEN] aligned_data(8);
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
#endif /* __INCLUDE_NUTTX_CAN_H */
