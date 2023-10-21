``ftpc`` FTP Client
===================

This is a simple FTP client shell used to exercise the capabilities of the FTPC
library (``apps/netutils/ftpc``).

From NSH, the startup command sequence is as follows. This is only an example,
your configuration could have different mass storage devices, mount paths, and
FTP directories::

  nsh> mount -t vfat /dev/mmcsd0 /tmp  # Mount the SD card at /tmp
  nsh> cd /tmp                         # cd into the /tmp directory
  nsh> ftpc <host> <port>              # Start the FTP client
  nfc> login <name> <password>         # Log into the FTP server
  nfc> help                            # See a list of FTP commands

where ``<host>`` is the IP address or hostname of the FTP server and ``<port>`` is
an optional port number.

**Note**: By default, FTPC uses ``readline`` to get data from ``stdin``. So your
defconfig file must have the following build path::

  CONFIG_SYSTEM_READLINE=y

**Note**: If you use the ftpc task over a telnet NSH connection, then you should
set the following configuration item::

  CONFIG_EXAMPLES_FTPC_FGETS=y

By default, the FTPC client will use ``readline()`` to get characters from the
console. Readline includes and command-line editor and echos characters received
in stdin back through ``stdout``. Neither of these behaviors are desire-able if
Telnet is used.

You may also want to define the following in your configuration file. Otherwise,
you will have not feedback about what is going on::

  CONFIG_DEBUG_FEATURES=y
  CONFIG_DEBUG_INFO=y
  CONFIG_DEBUG_FTPC=y
