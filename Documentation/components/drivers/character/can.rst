===========
CAN Drivers
===========

NuttX supports character device CAN a very low-level CAN driver and
SocketCAN as a more high level (and more overhead) option.
This driver supports only the data exchange and does not include any
high-level CAN protocol. The NuttX CAN driver is split into two
parts:

#. An "upper half", generic driver that provides the common CAN
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the CAN functionality.

Files supporting CAN can be found in the following locations:

-  **Interface Definition**. The header file for the NuttX CAN
   driver resides at ``include/nuttx/can/can.h``. This header file
   includes both the application level interface to the CAN driver
   as well as the interface between the "upper half" and "lower
   half" drivers. The CAN module uses a standard character driver
   framework.
-  **"Upper Half" Driver**. The generic, "upper half" CAN driver
   resides at ``drivers/can.c``.
-  **"Lower Half" Drivers**. Platform-specific CAN drivers reside
   in ``arch/<architecture>/src/<hardware>``
   directory for the specific processor ``<architecture>`` and for
   the specific ``<chip>`` CAN peripheral devices.

``struct timeval ch_ts``: This member variable that store in the
``can_hdr_s`` structure depends on ``CONFIG_CAN_TIMESTAMP`` and
is used to store the timestamp of the CAN message.

The upper half driver supports the following ``ioctl`` commands:

- **CANIOC_RTR**: Send the given message (passed as ``ioctl`` argument) as a
  remote request. On successful return, the passed message structure is updated
  with the contents of the received message; i.e. the message ID and the
  standard/extended ID indication bit stay the same, but the DLC and data bits
  are updated with the contents of the received message.  If no response is
  received after the specified timeout, ioctl will return.
- **CANIOC_GET_BITTIMING**: Return the current bit timing settings.
- **CANIOC_SET_BITTIMING**: Set new current bit timing values.
- **CANIOC_ADD_STDFILTER**: Add an address filter for a standard 11 bit address.
- **CANIOC_ADD_EXTFILTER**: Add an address filter for a extended 29 bit address.
- **CANIOC_DEL_STDFILTER**: Remove an address filter for a standard 11 bit
  address.
- **CANIOC_DEL_EXTFILTER**: Remove an address filter for a standard 29 bit
  address.
- **CANIOC_GET_CONNMODES**: Get the current bus connection modes.
- **CANIOC_SET_CONNMODES**: Set new bus connection modes values.
- **CANIOC_BUSOFF_RECOVERY**: Initiates the BUS-OFF recovery sequence.
- **CANIOC_SET_NART**: Enable/Disable NART (No Automatic Retry).
- **CANIOC_SET_ABOM**: Enable/Disable ABOM (Automatic Bus-off Management).
- **CANIOC_IFLUSH**: Flush data received but not read.
- **CANIOC_OFLUSH**: Flush data written but not transmitted.
- **CANIOC_IOFLUSH**: Flush data received but not read and data written but not
  transmitted.
- **CANIOC_SET_STATE**: Set specific can controller state.
- **CANIOC_GET_STATE**: Get specific can controller state.
- **CANIOC_SET_TRANSVSTATE**: Set specific can transceiver state.
- **CANIOC_GET_TRANSVSTATE**: Get specific can transceiver state.
- **CANIOC_SET_MSGALIGN**: Set messages alignment. Read and written messages can
  be configured to be aligned to multiple of given bytes by this. The default
  value is 1. The alignment affects both read and write operation. The value 0
  has a special meaning where write behaves the same way as with 1, but read
  will always provide only a single message.
- **CANIOC_GET_MSGALIGN**: Get messages alignment. See CANIOC_SET_MSGALIGN for
  explanation.

**Usage Note**: The default behavior of the upper half driver is to return
multiple messages on ``read``. See the `guide on this subject
</guides/reading_can_msgs.html>`_.

**Examples**: ``drivers/can/mcp2515.c``.
