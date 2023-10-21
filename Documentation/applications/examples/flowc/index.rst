``flowc`` Serial Hardware Flow Control
======================================

A simple test of serial hardware flow control.

General Usage Instructions:

1. The receiver side enter, start the receiver program. The receiver is now
   waiting to receive data on the configured serial port.
2. On the sender side start the sender program. This will send data to the
   receiver which will verify that no data is lost.

On Linux, you can alternatively do::

  $ stty -F /dev/ttyACM0 crtscts
  $ cat testdata.dat >/dev/ttyACM0

where you need to replace ``/dev/ttyACM0`` with your selected serial device.
