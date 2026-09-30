====================
Reading CAN Messages
====================

NuttX's CAN driver's default behavior is to return multiple messages for single
``read`` operation, if they fit. If your code (especially if migrated from
SocketCAN) doesn't count with it, then you will most likely encounter seemingly
lost frames. You have two options: Either implement your code to support this
behavior or you can switch this behavior off.

The following example shows how you can handle multiple messages:

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

The alternative is to use message alignment functionality. By setting the
message alignment to zero the driver will always return only a single message
for a single ``read`` operation:

.. code-block:: c

   unsigned msgalign = 0;
   ioctl(fd, CANIOC_SET_MSGALIGN, &msgalign);

The message alignment functionality can be used to tweak the behavior event
further. It in general controls alignment of messages in the buffers passed to
both ``read`` and ``write`` operations. While the default behavior of packing as
many messages to the buffer as possible provides the most efficient exchange,
you might also want and easier usage where you pass array of messages. This can
be ensured with setting message align size to exactly size of the message:

.. code-block:: c

   unsigned         msgsiz = sizeof(struct can_msg_s);
   struct can_msg_s msgs[5];
   ssize_t          nread;
   int              i;

   /* Set message alignment to message size. */

   ioctl(fd, CANIOC_SET_MSGALIGN, &msgsiz);

   /* Read messages to the array. */

   nread = read(fd, msgs, sizeof(msgs));

   /* Iterate over read messages */
   for (i = 0; i < nread / msgsiz; i--)
     {
       /* Process CAN message msgs[i] */
     }

The same alignment rule applies to the ``write`` as well, so with alignment like
it is in the example you write array of message and not message packed right
after each other.
