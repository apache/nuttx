/************************************************************************************
 * include/nuttx/can/can.h
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_CAN_CAN_H
#define __INCLUDE_NUTTX_CAN_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <nuttx/list.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>

#ifdef CONFIG_CAN_TXREADY
#  include <nuttx/wqueue.h>
#endif

#ifdef CONFIG_CAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* CONFIG_CAN - Enables CAN support (MCU-specific selections are also required.  For
 *   STM32, as an example, one or both of CONFIG_STM32_CAN1 or CONFIG_STM32_CAN2
 *   must also be defined)
 * CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
 *   Standard 11-bit IDs.
 * CONFIG_CAN_FD - Enable support for CAN FD mode.  For the upper half driver, this
 *   just means handling encoded DLC values (for values of DLC > 9).
 * CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
 *   Default: 8
 * CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
 *   Default: 4
 * CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
 *   mode for testing. If the driver does support loopback mode, the setting
 *   will enable it. (If the driver does not, this setting will have no effect).
 *   The loopback mode may be changed later by ioctl() if the driver supports the
 *   CANIOC_SET_CONNMODES ioctl command.
 * CONFIG_CAN_TXREADY - Add support for the can_txready() callback.  This is needed
 *   only for CAN hardware the supports an separate H/W TX message FIFO.  The call
 *   back is needed to keep the S/W FIFO and the H/W FIFO in sync.  Work queue
 *   support is needed for this feature.
 * CONFIG_CAN_TXREADY_HIPRI or CONFIG_CAN_TXREADY_LOPRI - Selects which work queue
 *   will be used for the can_txready() processing.
 */

/* Default configuration settings that may be overridden in the NuttX configuration
 * file or in the board configuration file.  The configured size is limited to 255
 * to fit into a uint8_t.
 */

#if !defined(CONFIG_CAN_FIFOSIZE)
#  define CONFIG_CAN_FIFOSIZE 8
#elif CONFIG_CAN_FIFOSIZE > 255
#  undef  CONFIG_CAN_FIFOSIZE
#  define CONFIG_CAN_FIFOSIZE 255
#endif

#if !defined(CONFIG_CAN_NPENDINGRTR)
#  define CONFIG_CAN_NPENDINGRTR 4
#elif CONFIG_CAN_NPENDINGRTR > 255
#  undef  CONFIG_CAN_NPENDINGRTR
#  define CONFIG_CAN_NPENDINGRTR 255
#endif

/* Ioctl Commands *******************************************************************/

/* Ioctl commands supported by the upper half CAN driver.
 *
 * CANIOC_RTR:
 *   Description:    Send the given message as a remote request. On successful
 *                   return, the passed message structure is updated with
 *                   the contents of the received message; i.e. the message
 *                   ID and the standard/extended ID indication bit stay the
 *                   same, but the DLC and data bits are updated with the
 *                   contents of the received message. If no response is
 *                   received after the specified timeout, ioctl will return.
 *
 *                   Note: Lower-half drivers that do not implement
 *                         CONFIG_CAN_USE_RTR and implement co_remoterequest
 *                         will result in EINVAL if this ioctl is called
 *                         with an extended-ID message.
 *
 *   Argument:       A pointer to struct canioc_rtr_s
 *   Returned Value: Zero (OK) is returned on success. Otherwise, -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error (for example, ETIMEDOUT)
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
 *
 * CANIOC_SET_NART:
 *   Description:    Enable/Disable NART (No Automatic Retry)
 *   Argument:       Set to 1 to enable NART, 0 to disable. Default is
 *                   disabled.
 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *                   is returned with the errno variable set to indicate the
 *                   nature of the error.
 *   Dependencies:   None
 *
 * CANIOC_SET_ABOM:
 *   Description:    Enable/Disable ABOM (Automatic Bus-off Management)
 *   Argument:       Set to 1 to enable ABOM, 0 to disable. Default is
 *                   disabled.
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
#define CANIOC_SET_NART           _CANIOC(11)
#define CANIOC_SET_ABOM           _CANIOC(12)

#define CAN_FIRST                 0x0001         /* First common command */
#define CAN_NCMDS                 12             /* Ten common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half CAN driver to the lower-half CAN driver via the co_ioctl()
 * method of the CAN lower-half interface.  However, the lower-half driver
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

/* Convenience macros ***************************************************************/

#define dev_reset(dev)            dev->cd_ops->co_reset(dev)
#define dev_setup(dev)            dev->cd_ops->co_setup(dev)
#define dev_shutdown(dev)         dev->cd_ops->co_shutdown(dev)
#define dev_txint(dev,enable)     dev->cd_ops->co_txint(dev,enable)
#define dev_rxint(dev,enable)     dev->cd_ops->co_rxint(dev,enable)
#define dev_ioctl(dev,cmd,arg)    dev->cd_ops->co_ioctl(dev,cmd,arg)
#define dev_remoterequest(dev,id) dev->cd_ops->co_remoterequest(dev,id)
#define dev_send(dev,m)           dev->cd_ops->co_send(dev,m)
#define dev_txready(dev)          dev->cd_ops->co_txready(dev)
#define dev_txempty(dev)          dev->cd_ops->co_txempty(dev)

/* CAN message support **************************************************************/

#ifdef CONFIG_CAN_FD
#  define CAN_MAXDATALEN          64
#else
#  define CAN_MAXDATALEN          8
#endif

#define CAN_MAX_STDMSGID          0x07ff
#define CAN_MAX_EXTMSGID          0x1fffffff

#define CAN_MSGLEN(nbytes)        (sizeof(struct can_msg_s) - CAN_MAXDATALEN + (nbytes))

/* CAN Error Indications ************************************************************/

#ifdef CONFIG_CAN_ERRORS
/* Bit settings in the ch_id field of the CAN error message (when ch_error is set) */

#  define CAN_ERROR_TXTIMEOUT     (1 << 0) /* Bit 0: TX timeout */
#  define CAN_ERROR_LOSTARB       (1 << 1) /* Bit 1: Lost arbitration (See CAN_ERROR0_* definitions) */
#  define CAN_ERROR_CONTROLLER    (1 << 2) /* Bit 2: Controller error (See CAN_ERROR1_* definitions) */
#  define CAN_ERROR_PROTOCOL      (1 << 3) /* Bit 3: Protocol error (see CAN_ERROR2_* and CAN_ERROR3_* definitions) */
#  define CAN_ERROR_TRANSCEIVER   (1 << 4) /* Bit 4: Transceiver error (See CAN_ERROR4_* definitions)    */
#  define CAN_ERROR_NOACK         (1 << 5) /* Bit 5: No ACK received on transmission */
#  define CAN_ERROR_BUSOFF        (1 << 6) /* Bit 6: Bus off */
#  define CAN_ERROR_BUSERROR      (1 << 7) /* Bit 7: Bus error */
#  define CAN_ERROR_RESTARTED     (1 << 8) /* Bit 8: Controller restarted */
#  define CAN_ERROR_INTERNAL      (1 << 9) /* Bit 9: Stack internal error (See CAN_ERROR5_* definitions) */
                                           /* Bit 10: Available */

/* The remaining definitions described the error report payload that follows the
 * CAN header.
 */

#  define CAN_ERROR_DLC           (8)      /* DLC of error report */

/* Data[0]: Arbitration lost in ch_error. */

#  define CAN_ERROR0_UNSPEC       0x00     /* Unspecified error */
#  define CAN_ERROR0_BIT(n)       (n)      /* Bit number in the bit stream */

/* Data[1]:  Error status of CAN-controller */

#  define CAN_ERROR1_UNSPEC       0x00     /* Unspecified error */
#  define CAN_ERROR1_RXOVERFLOW   (1 << 0) /* Bit 0: RX buffer overflow */
#  define CAN_ERROR1_TXOVERFLOW   (1 << 1) /* Bit 1: TX buffer overflow */
#  define CAN_ERROR1_RXWARNING    (1 << 2) /* Bit 2: Reached warning level for RX errors */
#  define CAN_ERROR1_TXWARNING    (1 << 3) /* Bit 3: Reached warning level for TX errors */
#  define CAN_ERROR1_RXPASSIVE    (1 << 4) /* Bit 4: Reached passive level for RX errors */
#  define CAN_ERROR1_TXPASSIVE    (1 << 5) /* Bit 5: Reached passive level for TX errors */
                                           /* Bits 6-7: Available */

/* Data[2]:  Error in CAN protocol.  This provides the type of the error. */

#  define CAN_ERROR2_UNSPEC       0x00     /* Unspecified error */
#  define CAN_ERROR2_BIT          (1 << 0) /* Bit 0: Single bit error */
#  define CAN_ERROR2_FORM         (1 << 1) /* Bit 1: Frame format error */
#  define CAN_ERROR2_STUFF        (1 << 2) /* Bit 2: Bit stuffing error */
#  define CAN_ERROR2_BIT0         (1 << 3) /* Bit 3: Unable to send dominant bit */
#  define CAN_ERROR2_BIT1         (1 << 4) /* Bit 4: Unable to send recessive bit */
#  define CAN_ERROR2_OVERLOAD     (1 << 5) /* Bit 5: Bus overload */
#  define CAN_ERROR2_ACTIVE       (1 << 6) /* Bit 6: Active error announcement */
#  define CAN_ERROR2_TX           (1 << 7) /* Bit 7: Error occurred on transmission */

/* Data[3]:  Error in CAN protocol.  This provides the loation of the error. */

#  define CAN_ERROR3_UNSPEC       0x00 /* Unspecified error */
#  define CAN_ERROR3_SOF          0x01 /* start of frame */
#  define CAN_ERROR3_ID0          0x02 /* ID bits 0-4 */
#  define CAN_ERROR3_ID1          0x03 /* ID bits 5-12 */
#  define CAN_ERROR3_ID2          0x04 /* ID bits 13-17 */
#  define CAN_ERROR3_ID3          0x05 /* ID bits 21-28 */
#  define CAN_ERROR3_ID4          0x06 /* ID bits 18-20 */
#  define CAN_ERROR3_IDE          0x07 /* Identifier extension */
#  define CAN_ERROR3_RTR          0x08 /* RTR */
#  define CAN_ERROR3_SRTR         0x09 /* Substitute RTR */
#  define CAN_ERROR3_RES0         0x0a /* Reserved bit 0 */
#  define CAN_ERROR3_RES1         0x0b /* Reserved bit 1 */
#  define CAN_ERROR3_DLC          0x0c /* Data length code */
#  define CAN_ERROR3_DATA         0x0d /* Data section */
#  define CAN_ERROR3_CRCSEQ       0x0e /* CRC sequence */
#  define CAN_ERROR3_CRCDEL       0x0f /* CRC delimiter */
#  define CAN_ERROR3_ACK          0x10 /* ACK slot */
#  define CAN_ERROR3_ACKDEL       0x11 /* ACK delimiter */
#  define CAN_ERROR3_EOF          0x12 /* End of frame */
#  define CAN_ERROR3_INTERM       0x13 /* Intermission */

/* Data[4]: Error status of CAN-transceiver */

#  define CAN_ERROR4_UNSPEC       0x00

#  define CANH_ERROR4_MASK        0x0f /* Bits 0-3: CANH */
#  define CANH_ERROR4_NOWIRE      0x01
#  define CANH_ERROR4_SHORT2BAT   0x02
#  define CANH_ERROR4_SHORT2VCC   0x03
#  define CANH_ERROR4_SHORT2GND   0x04

#  define CANL_ERROR4_MASK        0xf0 /* Bits 0-3: CANL */
#  define CANL_ERROR4_NOWIRE      0x10
#  define CANL_ERROR4_SHORT2BAT   0x20
#  define CANL_ERROR4_SHORT2VCC   0x30
#  define CANL_ERROR4_SHORT2GND   0x40
#  define CANL_ERROR4_SHORT2CANH  0x50

/* Data[5]: Error status of stack internals */

#  define CAN_ERROR5_UNSPEC       0x00     /* Unspecified error */
#  define CAN_ERROR5_RXOVERFLOW   (1 << 0) /* Bit 0: RX buffer overflow */

#endif /* CONFIG_CAN_ERRORS */

/* CAN filter support ***************************************************************/

/* Some CAN hardware supports a notion of prioritizing messages that match filters.
 * Only two priority levels are currently supported and are encoded as defined
 * below:
 */

#define CAN_MSGPRIO_LOW           0
#define CAN_MSGPRIO_HIGH          1

/* Filter type.  Not all CAN hardware will support all filter types. */

#define CAN_FILTER_MASK           0  /* Address match under a mask */
#define CAN_FILTER_DUAL           1  /* Dual address match */
#define CAN_FILTER_RANGE          2  /* Match a range of addresses */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* CAN-message Format (without Extended ID support)
 *
 *   One based CAN-message is represented with a maximum of 10 bytes.  A message is
 *   composed of at least the first 2 bytes (when there are no data bytes present).
 *
 *   Bytes 0-1:  Bits 0-3:   Data Length Code (DLC)
 *               Bit  4:     Remote Transmission Request (RTR)
 *               Bit  5:     1=Message ID is a bit-encoded error report (See NOTE)
 *               Bits 6-7:   Unused
 *   Bytes 1-2:  Bits 0-10:  The 11-bit CAN identifier  This message ID is a bit
 *                           encoded error set if ch_error is set (See NOTE).
 *               Bits 11-15: Unused
 *   Bytes 3-10: CAN data
 *
 * CAN-message Format (with Extended ID support)
 *
 *   One CAN-message consists of a maximum of 13 bytes.  A message is composed of at
 *   least the first 5 bytes (when there are no data bytes).
 *
 *   Bytes 0-3:  Bits 0-28:  Hold 11- or 29-bit CAN ID in host byte order.  This
 *                           message ID is a bit encoded error set if ch_error
 *                           is set (See NOTE).
 *               Bits 29-31: Unused
 *   Byte 4:     Bits 0-3:   Data Length Code (DLC)
 *               Bit 4:      Remote Transmission Request (RTR)
 *               Bit 5:      1=Message ID is a bit-encoded error report (See NOTE)
 *               Bit 6:      Extended ID indication
 *               Bit 7:      Unused
 *   Bytes 5-12: CAN data    Size determined by DLC
 *
 * NOTE: The error indication if valid only on message reports received from the
 * CAN driver; it is ignored on transmission.  When the error bit is set, the
 * message ID is an encoded set of error indications (see CAN_ERROR_* definitions).
 * A more detailed report of certain errors then follows in message payload.
 * CONFIG_CAN_ERRORS=y is required in order to receive error reports.
 *
 * The struct can_msg_s holds this information in a user-friendly, unpacked form.
 * This is the form that is used at the read() and write() driver interfaces.  The
 * message structure is actually variable length -- the true length is given by
 * the CAN_MSGLEN macro.
 */

#ifdef CONFIG_CAN_EXTID
begin_packed_struct struct can_hdr_s
{
  uint32_t     ch_id;         /* 11- or 29-bit ID (20- or 3-bits unused) */
  uint8_t      ch_dlc    : 4; /* 4-bit DLC */
  uint8_t      ch_rtr    : 1; /* RTR indication */
#ifdef CONFIG_CAN_ERRORS
  uint8_t      ch_error  : 1; /* 1=ch_id is an error report */
#endif
  uint8_t      ch_extid  : 1; /* Extended ID indication */
#ifdef CONFIG_CAN_FD
  uint8_t      ch_edl    : 1; /* Extended Data Length */
  uint8_t      ch_brs    : 1; /* Bit Rate Switch */
  uint8_t      ch_esi    : 1; /* Error State Indicator */
#endif
  uint8_t      ch_unused : 1; /* FIXME: This field is useless, kept for backward compatibility */
} end_packed_struct;

#else
begin_packed_struct struct can_hdr_s
{
  uint16_t     ch_id;         /* 11-bit standard ID (5-bits unused) */
  uint8_t      ch_dlc    : 4; /* 4-bit DLC.  May be encoded in CAN_FD mode. */
  uint8_t      ch_rtr    : 1; /* RTR indication */
#ifdef CONFIG_CAN_ERRORS
  uint8_t      ch_error  : 1; /* 1=ch_id is an error report */
#endif
#ifdef CONFIG_CAN_FD
  uint8_t      ch_edl    : 1; /* Extended Data Length */
  uint8_t      ch_brs    : 1; /* Bit Rate Switch */
  uint8_t      ch_esi    : 1; /* Error State Indicator */
#endif
  uint8_t      ch_unused : 1; /* FIXME: This field is useless, kept for backward compatibility */
} end_packed_struct;
#endif

begin_packed_struct struct can_msg_s
{
  struct can_hdr_s cm_hdr;                  /* The CAN header */
  uint8_t          cm_data[CAN_MAXDATALEN]; /* CAN message data (0-8 byte) */
} end_packed_struct;

/* This structure defines a CAN message FIFO. */

struct can_rxfifo_s
{
  /* Binary semaphore. Indicates whether FIFO is available for reading
   * AND not empty. Only take this sem inside a critical section to guarantee
   * exclusive access to both the semaphore and the head/tail FIFO indices.
   */

  sem_t         rx_sem;

#ifdef CONFIG_CAN_ERRORS
  bool          rx_overflow;             /* Indicates the RX FIFO overflow event */
#endif
  uint8_t       rx_head;                 /* Index to the head [IN] in the circular buffer */
  uint8_t       rx_tail;                 /* Index to the tail [OUT] in the circular buffer */
                                         /* Circular buffer of CAN messages */
  struct can_msg_s rx_buffer[CONFIG_CAN_FIFOSIZE];
};

struct can_txfifo_s
{
  sem_t         tx_sem;                  /* Counting semaphore */
  uint8_t       tx_head;                 /* Index to the head [IN] in the circular buffer */
  uint8_t       tx_queue;                /* Index to next message to send */
  uint8_t       tx_tail;                 /* Index to the tail [OUT] in the circular buffer */
                                         /* Circular buffer of CAN messages */
  struct can_msg_s tx_buffer[CONFIG_CAN_FIFOSIZE];
};

/* The following structure define the logic to handle one RTR message transaction */

struct can_rtrwait_s
{
  sem_t         cr_sem;                  /* Wait for response/is the cd_rtr entry available */
  FAR struct can_msg_s *cr_msg;          /* This is where the RTR response goes */
};

/* This structure defines all of the operations provided by the architecture specific
 * logic.  All fields must be provided with non-NULL function pointers by the
 * caller of can_register().
 */

struct can_dev_s;
struct can_ops_s
{
  /* Reset the CAN device.  Called early to initialize the hardware. This
   * is called, before co_setup() and on error conditions.
   */

  CODE void (*co_reset)(FAR struct can_dev_s *dev);

  /* Configure the CAN. This method is called the first time that the CAN
   * device is opened.  This will occur when the port is first opened.
   * This setup includes configuring and attaching CAN interrupts.  All CAN
   * interrupts are disabled upon return.
   */

  CODE int (*co_setup)(FAR struct can_dev_s *dev);

  /* Disable the CAN.  This method is called when the CAN device is closed.
   * This method reverses the operation the setup method.
   */

  CODE void (*co_shutdown)(FAR struct can_dev_s *dev);

  /* Call to enable or disable RX interrupts */

  CODE void (*co_rxint)(FAR struct can_dev_s *dev, bool enable);

  /* Call to enable or disable TX interrupts */

  CODE void (*co_txint)(FAR struct can_dev_s *dev, bool enable);

  /* All ioctl calls will be routed through this method */

  CODE int (*co_ioctl)(FAR struct can_dev_s *dev, int cmd, unsigned long arg);

  /* Send a remote request. Lower-half drivers should NOT implement this if
   * they support sending RTR messages with the regular send function
   * (i.e. CONFIG_CAN_USE_RTR). Instead, they should mention CAN_USE_RTR
   * in their Kconfig help and set this to NULL to indicate that the normal
   * send function should be used instead. Lower-half drivers must implement
   * either this or CONFIG_CAN_USE_RTR to support CANIOC_RTR.
   */

  CODE int (*co_remoterequest)(FAR struct can_dev_s *dev, uint16_t id);

  /* This method will send one message on the CAN */

  CODE int (*co_send)(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);

  /* Return true if the CAN hardware can accept another TX message. */

  CODE bool (*co_txready)(FAR struct can_dev_s *dev);

  /* Return true if all message have been sent.  If for example, the CAN
   * hardware implements FIFOs, then this would mean the transmit FIFO is
   * empty.  This method is called when the driver needs to make sure that
   * all characters are "drained" from the TX hardware before calling co_shutdown().
   */

  CODE bool (*co_txempty)(FAR struct can_dev_s *dev);
};

/* This is the device structure used by the driver.  The caller of
 * can_register() must allocate and initialize this structure.  The
 * calling logic need only set all fields to zero except:
 *
 *   The elements of 'cd_ops', and 'cd_priv'
 *
 * The common logic will initialize all semaphores.
 */

struct can_reader_s
{
  struct list_node     list;
  struct can_rxfifo_s  fifo;             /* Describes receive FIFO */
};

struct can_dev_s
{
  uint8_t              cd_crefs;         /* References counts on number of opens */
  uint8_t              cd_npendrtr;      /* Number of pending RTR messages */
  volatile uint8_t     cd_ntxwaiters;    /* Number of threads waiting to enqueue a message */
#ifdef CONFIG_CAN_ERRORS
  uint8_t              cd_error;         /* Flags to indicate internal device errors */
#endif
  struct list_node     cd_readers;       /* List of readers */
  mutex_t              cd_closelock;     /* Locks out new opens while close is in progress */
  mutex_t              cd_polllock;      /* Manages exclusive access to cd_fds[] */
  struct can_txfifo_s  cd_xmit;          /* Describes transmit FIFO */
#ifdef CONFIG_CAN_TXREADY
  struct work_s        cd_work;          /* Use to manage can_txready() work */
#endif
                                         /* List of pending RTR requests */
  struct can_rtrwait_s cd_rtr[CONFIG_CAN_NPENDINGRTR];
  FAR const struct can_ops_s *cd_ops;    /* Arch-specific operations */
  FAR void            *cd_priv;          /* Used by the arch-specific logic */

  FAR struct pollfd   *cd_fds[CONFIG_CAN_NPOLLWAITERS];
};

/* Structures used with ioctl calls */

/* CANIOC_RTR: */

struct canioc_rtr_s
{
  /* How long to wait for the response */

  struct timespec       ci_timeout;

  /* The location to return the RTR response. The arbitration fields
   * (i.e. message ID and extended ID indication, if applicable) should be
   * set to the values the driver will watch for. On return from the ioctl,
   * the DLC and data fields will be updated by the received message.
   *
   * The block of memory must be large enough to hold an message of size
   * CAN_MSGLEN(CAN_MAXDATALEN) even if a smaller DLC is requested, since
   * the response DLC may not match the requested one.
   */

  FAR struct can_msg_s *ci_msg;
};

/* CANIOC_GET_BITTIMING/CANIOC_SET_BITTIMING:
 *
 * Bit time = Tquanta * (Sync_Seg + Prop_Seq + Phase_Seg1 + Phase_Seg2)
 *          = Tquanta * (TSEG1 + TSEG2 + 1)
 * Where
 *   TSEG1 = Prop_Seq + Phase_Seg1
 *   TSEG2 = Phase_Seg2
 */

struct canioc_bittiming_s
{
  uint32_t              bt_baud;         /* Bit rate = 1 / bit time */
  uint8_t               bt_tseg1;        /* TSEG1 in time quanta */
  uint8_t               bt_tseg2;        /* TSEG2 in time quanta */
  uint8_t               bt_sjw;          /* Synchronization Jump Width in time quanta */
};

/* CANIOC_GET_CONNMODES/CANIOC_SET_CONNMODES:
 *
 * A CAN device may support loopback and silent mode. Both modes may not be
 * settable independently.
 */

struct canioc_connmodes_s
{
  uint8_t               bm_loopback : 1; /* Enable reception of messages sent
                                          * by this node. */
  uint8_t               bm_silent   : 1; /* Disable transmission of messages.
                                          * The node still receives messages. */
};

#ifdef CONFIG_CAN_EXTID
/* CANIOC_ADD_EXTFILTER: */

struct canioc_extfilter_s
{
  uint32_t              xf_id1;          /* 29-bit ID.  For dual match or for the
                                          * lower address in a range of addresses  */
  uint32_t              xf_id2;          /* 29-bit ID.  For dual match, address mask
                                          * or for upper address in address range  */
  uint8_t               xf_type;         /* See CAN_FILTER_* definitions */
  uint8_t               xf_prio;         /* See CAN_MSGPRIO_* definitions */
};
#endif

/* CANIOC_ADD_STDFILTER: */

struct canioc_stdfilter_s
{
  uint16_t              sf_id1;          /* 11-bit ID.  For dual match or for the
                                          * lower address in a range of addresses  */
  uint16_t              sf_id2;          /* 11-bit ID.  For dual match, address mask
                                          * or for upper address in address range  */
  uint8_t               sf_type;         /* See CAN_FILTER_* definitions */
  uint8_t               sf_prio;         /* See CAN_MSGPRIO_* definitions */
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

/************************************************************************************
 * Name: can_register
 *
 * Description:
 *   Register a CAN driver.
 *
 ************************************************************************************/

int can_register(FAR const char *path, FAR struct can_dev_s *dev);

/************************************************************************************
 * Name: can_receive
 *
 * Description:
 *   Called from the CAN interrupt handler when new read data is available
 *
 * Input Parameters:
 *   dev  - The specific CAN device
 *   hdr  - The 16-bit CAN header
 *   data - An array contain the CAN data.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ************************************************************************************/

int can_receive(FAR struct can_dev_s *dev, FAR struct can_hdr_s *hdr,
                FAR uint8_t *data);

/************************************************************************************
 * Name: can_txdone
 *
 * Description:
 *   Called when the hardware has processed the outgoing TX message.  This
 *   normally means that the CAN messages was sent out on the wire.  But
 *   if the CAN hardware supports a H/W TX FIFO, then this call may mean
 *   only that the CAN message has been added to the H/W FIFO.  In either
 *   case, the upper-half CAN driver can remove the outgoing message from
 *   the S/W FIFO and discard it.
 *
 *   This function may be called in different contexts, depending upon the
 *   nature of the underlying CAN hardware.
 *
 *   1. No H/W TX FIFO (CONFIG_CAN_TXREADY not defined)
 *
 *      This function is only called from the CAN interrupt handler at the
 *      completion of a send operation.
 *
 *        can_write() -> can_xmit() -> dev_send()
 *        CAN interrupt -> can_txdone()
 *
 *      If the CAN hardware is busy, then the call to dev_send() will
 *      fail, the S/W TX FIFO will accumulate outgoing messages, and the
 *      thread calling can_write() may eventually block waiting for space in
 *      the S/W TX FIFO.
 *
 *      When the CAN hardware completes the transfer and processes the
 *      CAN interrupt, the call to can_txdone() will make space in the S/W
 *      TX FIFO and will awaken the waiting can_write() thread.
 *
 *   2a. H/W TX FIFO (CONFIG_CAN_TXREADY=y) and S/W TX FIFO not full
 *
 *      This function will be called back from dev_send() immediately when a
 *      new CAN message is added to H/W TX FIFO:
 *
 *        can_write() -> can_xmit() -> dev_send() -> can_txdone()
 *
 *      When the H/W TX FIFO becomes full, dev_send() will fail and
 *      can_txdone() will not be called.  In this case the S/W TX FIFO will
 *      accumulate outgoing messages, and the thread calling can_write() may
 *      eventually block waiting for space in the S/W TX FIFO.
 *
 *   2b. H/W TX FIFO (CONFIG_CAN_TXREADY=y) and S/W TX FIFO full
 *
 *      In this case, the thread calling can_write() is blocked waiting for
 *      space in the S/W TX FIFO.  can_txdone() will be called, indirectly,
 *      from can_txready_work() running on the thread of the work queue.
 *
 *        CAN interrupt -> can_txready() -> Schedule can_txready_work()
 *        can_txready_work() -> can_xmit() -> dev_send() -> can_txdone()
 *
 *      The call dev_send() should not fail in this case and the subsequent
 *      call to can_txdone() will make space in the S/W TX FIFO and will
 *      awaken the waiting thread.
 *
 * Input Parameters:
 *   dev  - The specific CAN device
 *   hdr  - The 16-bit CAN header
 *   data - An array contain the CAN data.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 * Assumptions:
 *   Interrupts are disabled.  This is required by can_xmit() which is called
 *   by this function.  Interrupts are explicitly disabled when called
 *   through can_write().  Interrupts are expected be disabled when called
 *   from the CAN interrupt handler.
 *
 ************************************************************************************/

int can_txdone(FAR struct can_dev_s *dev);

/************************************************************************************
 * Name: can_txready
 *
 * Description:
 *   Called from the CAN interrupt handler at the completion of a send
 *   operation.  This interface is needed only for CAN hardware that
 *   supports queueing of outgoing messages in a H/W FIFO.
 *
 *   The CAN upper half driver also supports a queue of output messages in a
 *   S/W FIFO.  Messages are added to that queue when when can_write() is
 *   called and removed from the queue in can_txdone() when each TX message
 *   is complete.
 *
 *   After each message is added to the S/W FIFO, the CAN upper half driver
 *   will attempt to send the message by calling into the lower half driver.
 *   That send will not be performed if the lower half driver is busy, i.e.,
 *   if dev_txready() returns false.  In that case, the number of messages in
 *   the S/W FIFO can grow.  If the S/W FIFO becomes full, then can_write()
 *   will wait for space in the S/W FIFO.
 *
 *   If the CAN hardware does not support a H/W FIFO then busy means that
 *   the hardware is actively sending the message and is guaranteed to
 *   become non-busy (i.e, dev_txready()) when the send transfer completes
 *   and can_txdone() is called.  So the call to can_txdone() means that the
 *   transfer has completed and also that the hardware is ready to accept
 *   another transfer.
 *
 *   If the CAN hardware supports a H/W FIFO, can_txdone() is not called
 *   when the transfer is complete, but rather when the transfer is queued in
 *   the H/W FIFO.  When the H/W FIFO becomes full, then dev_txready() will
 *   report false and the number of queued messages in the S/W FIFO will grow.
 *
 *   There is no mechanism in this case to inform the upper half driver when
 *   the hardware is again available, when there is again space in the H/W
 *   FIFO.  can_txdone() will not be called again.  If the S/W FIFO becomes
 *   full, then the upper half driver will wait for space to become
 *   available, but there is no event to awaken it and the driver will hang.
 *
 *   Enabling this feature adds support for the can_txready() interface.
 *   This function is called from the lower half driver's CAN interrupt
 *   handler each time a TX transfer completes.  This is a sure indication
 *   that the H/W FIFO is no longer full.  can_txready() will then awaken
 *   the can_write() logic and the hang condition is avoided.
 *
 * Input Parameters:
 *   dev  - The specific CAN device
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 * Assumptions:
 *   Interrupts are disabled.  This function may execute in the context of
 *   and interrupt handler.
 *
 ************************************************************************************/

#ifdef CONFIG_CAN_TXREADY
int can_txready(FAR struct can_dev_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_CAN */
#endif /* __INCLUDE_NUTTX_CAN_CAN_H */
