==========================
USB (Universal Serial Bus)
==========================

NAKing USB OUT/IN Tokens
------------------------

USB supports a NAKing mechanism to pace incoming OUT data from the USB host.
The USB device class driver can control this NAKing only indirectly
by the manner in which it manages read requests.

Similarly, USB will refuse to perform the USB host's requests for IN data
by NAKing the host's IN tokens.

.. note:: USB direction naming is host-centric. This makes life difficult for
          people working USB device drivers:
          IN refers to data coming from the device INto the host.
          But for the device, this is a write operation (write being
          peripheral-centric: from-memory-to-peripheral).
          OUT refers to data sent from the host OUT to the device.
          For the device this is a read (peripheral-to-memory) operation.


NAKing USB OUT Tokens
^^^^^^^^^^^^^^^^^^^^^

Below is how the NAKing is implemented in the interaction between the USB
device class driver and USB Device Controller Drivers (DCDs):

1. At initialization time when the USB device class driver is enumerated
   by the host, the class driver will allocates a number of read requests
   (which contain request buffers).
2. The USB device class driver sends each allocated read request to the device
   DCD via ``DRVR_EPSUBMIT()``.
3. Upon receipt, the DCD will keep all of these empty read requests in a list.
4. When the OUT token is received what happens next depends upon the state
   of the list of empty read requests:

   4a. If there are no requests in the list of empty read requests, the DCD
   will have configured the endpoint to NAK the OUT token. This NAKing
   will continue until the list of read requests is non-empty and the DCD
   disables the NAKing to signal that it can accept the OUT packet data.

   4b. If the list of read requests is not empty, the DCD will remove the next
   read request from the list, receive the DATA accompanying the OUT
   token, copy the received packet data into the read request's buffer,
   and return the filled reqd request to the USB device class driver.
   The data is returned using a callback function pointer that is
   contained inside of the read request.

5. When the filled read request if returned, the USB device class driver will
   add the newly filled read request to the TAIL of a list of filled read
   requests. This will then be available to the the driver read logic when
   the application requests more data.
   This step will also wake up read logic that has been suspended waiting
   for the availability of incoming data.
6. The receive logic starts when the application requests data (or it is
   resumed after the receipt of new data) it will look at the read request
   at the HEAD of the list of filled read requests. If will return the data
   payload to the application by copying it from the read request's buffer
   into the application-provided receive buffer:

   6a. If the application's receive buffer becomes full and it cannot accept
   more data, the receive logic will save the offset in the data buffer
   where it left off. If the application asks for more data, it will
   resume transferring at the point where it left off.

   6b. If the receive logic transfers all of the data from the receive
   buffer. It will remove the read request from the HEAD of the list
   of filled read requests and return the receive buffer to the driver
   via ``DRVR_EPSUBMIT()`` (See step 3).
   The receive logic will then continue at step 6.

   6c. If the receive logic needs more data in order to complete
   the application read operation, it will wait until it is awakened
   by the receipt of a newly filled.


CDC/ACM driver
~~~~~~~~~~~~~~

The CDC/ACM driver differs from all other drivers because it is REQUIRED
to discard data if there is no application ready to receive the data.
This is how a serial device works. So there is nothing like step 5
in the CDC/ACM driver and in step 6 the CDC/ACM driver will not remember
the point where it left off; if the application cannot accept any further
data, it will return the partially emptied read request to the DCD
immediately to be filled – dropping an untransferred data into the bit bucket.

This is not an error or an oversight in the CDC/ACM. This is the intentional
design. This is the CORRECT behavior of the CDC/ACM driver. Data overrun can
only be prevented on a serial connection by using either XON/XOFF software
flow control or hardware flow control. A special endpoint 0 control request
is reserved for the CDC/ACM implementation.

.. note:: As of this writing, the logic that implements the CDC/ACM
          _hardware flow control has not been implemented.
          There is a place holder for such logic, but the implementation
          is missing._


NAKing USB IN Tokens
^^^^^^^^^^^^^^^^^^^^

The DCD will NAK IN tokens whenever it has nothing to send to the host PC.
Without going into as much detail, suffice it to say that the logic flow
is very similar to the case off OUT tokens with the following differences:

1. The list of empty write requests are not provided to the DCD at
   initialization time, but are instead are retained in a list of empty write
   requests in the USB device class driver.

2. When the USB class driver has data to send, it will remove a write request
   from this list, copy the outgoing application data into the write buffer,
   and send the filled write request to the DCD via ``DRVR_EPSUBMIT()``.
   If there are no available write requests, the application thread may block
   waiting for the next write request to be returned from the DCD.

3. The DCD will keep a list of filled write requests for each endpoint.
   When that list becomes non-empty, the DCD will disable the NAKing and
   set up to transfer data to the host. Normally this involves copying
   the IN data into an outgoing FIFO or setting up some write DMA transfers:

   3a. After the data has been transferred to the USB device hardware,
   the emptied write request will be returned to the USB device class driver.
   This is done through a callback function pointer within the write request.

   3b. If the DCD's list of write requests becomes empty, it will set up
   the endpoint to NAK any further IN tokens (after the final IN transfer
   completes).

4. When the USB device class driver receives the emptied write request from
   the DCD, it will return the write request to its list of empty write
   request and may, perhaps, wake up any thread that was waiting
   for an available write request to send more data.

So the fundamental different between IN and OUT processing is that for
IN transfers, empty write requests are saved in the USB device class driver
and then given to the DCD when they are filled with outgoing data.
For OUT transfers empty read requests are retained in the DCD and returned
to the USB device class driver when they filled with incoming data.

Similarly, filled read requests are queued for application read processing
in the USB device class driver; filled write requests are queued for transfer
to the host in the DCD.

But in either case, NAKing occurs when associated DCD request queue is empty.
