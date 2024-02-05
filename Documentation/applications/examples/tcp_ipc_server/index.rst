===========================================
``tcp_ipc_server`` Server for TCP IPC NuttX
===========================================

What's this?
------------

This program consists of a server socket & custom messages to establish IPC for multiple applications (client_tcp) and one process that controls LoRaWAN connectivity (server_tcp).
For more details about client side, please see client_tcp example.

This approach using TCP/IP sockets as IPC channel ensures controlled access to LoRaWAN connectivity.
The goals of using this approach are:

- Having a solid and reliable infrastructure to ensure IPC works fine for multiple applications simultaneously
- Having the possibility to host different IoT projects and solutions that use LPWAN in a single ESP32
- Having the possibility to validate, test and debug multiple IoT projects and solutions at the same time, under the same connectivity conditions (same signal strength, same antenna, same modem/transceiver, etc.)

Both client and server work on local network scope.

How do I use this?
-------------------

In order to test client_tcp & server_tcp together, there are two ways to proceed:

- Init server manually (command: SERVER &), and after successful server init, also init client manually (CLIENT 127.0.0.1)
- init server automatically after boot using NuttShell start up scripts (check: https://nuttx.apache.org/docs/latest/applications/nsh/installation.html#nuttshell-start-up-scripts )

Additional info
---------------

Both client_tcp and server_tcp examples have been full covered in NuttX International Workshop 2022. You can watch the full presentation here: https://www.youtube.com/watch?v=hr0OfTt1KeY
The server_tcp and client_tcp examples have been developed by Flavio Ipirranga and Pedro Bertoleti from Instituto de Pesquisas Eldorado (IPE) in Brazil.
