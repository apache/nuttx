``tcp_ipc_client``
==================

What's this?
------------

This program consists of a client socket & custom messages that send data (hex-string formatted data) to a server (tcp_ipc_server).
Then, tcp_ipc_server send this data over LoraWAN (using Radioenge LoRaWAN module). It means using TCP/IP sockets as IPC channel to ensure controlled access to LoRaWAN connectivity.
The goals of using this approach to send LoRaWAN data are:

- Having a solid and reliable infrastructure to ensure IPC works fine for multiple applications simultaneously
- Having the possibility to host different IoT projects and solutions that use LPWAN in a single ESP32
- Having the possibility to validate, test and debug multiple IoT projects and solutions at the same time, under the same connectivity conditions (same signal strength, same antenna, same modem/transceiver, etc.)

Both client and server work on local network scope.

How do I use this?
------------------

In order to test tcp_ipc_client & tcp_ipc_server together, there are two ways to proceed:

- Init server manually (command: SERVER &), and after successfull server init, also init client manually (CLIENT 127.0.0.1)
- init server automatically after boot using NuttShell start up scripts (check: https://nuttx.apache.org/docs/latest/applications/nsh/installation.html#nuttshell-start-up-scripts )

Additional info
---------------

Both tcp_ipc_client and tcp_ipc_server examples have been full covered in NuttX International Workshop 2022. You can watch the full presentation here: https://www.youtube.com/watch?v=hr0OfTt1KeY
The tcp_ipc_server and tcp_ipc_client examples have been developed by Flavio Ipirranga and Pedro Bertoleti from Instituto de Pesquisas Eldorado (IPE) in Brazil.
