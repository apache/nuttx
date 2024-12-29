====================
Reading CAN Messages
====================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Reading+CAN+Messages

Twice now, there have been complaints or issues about reading messages from the
CAN driver. The usual concern is that the driver is somehow losing or dropping
CAN messages. In these cases, it is often discovered that the CAN driver is
being used incorrectly and, as is human nature, the driver itself is blamed for
the problem.

When reading from the CAN driver, multiple messages may be returned, depending
on two factors:

1. The size of the returned CAN messages.
2. The size of the buffer provided to receive CAN messages.

It should never be assumed that a single message will be returned; making this
assumption can lead to lost CAN messages under conditions in which the read
buffer can hold more than one small message. The following example shows how to
properly handle the CAN read operation:

.. code-block:: c

   #define BUFLEN 128  /* Some arbitrary size for the CAN RX buffer */
 
    FAR struct can_msg_s *msg;
    char rxbuffer[BUFLEN];
    ssize_t nread;
    int nbytes;
    int msglen
    int i;
    
    /* Read messages into the RX buffer */
    
    nread = read(fd, rxbuffer, BUFLEN);
    
    /* Check for read errors */
    ...
    
    /* Process each message in the RX buffer */
    
    for (i = 0; i <= nread - CAN_MSGLEN(0); i += msglen)
    {
    /* Get the next message from the RX buffer */
    
        msg    = (FAR struct can_msg_s *)&rxbuffer[i];
        nbytes = can_dlc2bytes(msg->cm_hdr.ch_dlc);
        msglen = CAN_MSGLEN(nbytes);
    
        DEBUGASSERT(i + msglen < BUFLEN);
    
    /* Process the next CAN message */
    ...
    }

By looping over the read buffer and parsing out each CAN message, it is
possible to avoid losing messages that are stored contiguously in the input
buffer.
