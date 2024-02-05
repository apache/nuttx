====================================
``btsak`` Bluetooth Swiss Army Knife
====================================

Commands
--------

help::

  Command:      help
  Description:  Should overall command help
  Usage:        bt <ifname> help

info::

  Command:      info
  Description:  Show Bluetooth driver information
  Usage:        bt <ifname> info [-h]

features::

  Command:      features
  Description:  Show Bluetooth driver information
  Usage:        bt <ifname> features [-h] [le]
  Where:        le - Selects LE features vs BR/EDR features

scan::

  Command:      scan
  Description:  Bluetooth scan commands
  Usage:        bt <ifname> scan [-h] <start [-d]|get|stop>
  Where:        start - Starts scanning.  The -d option enables duplicate
                filtering.
                get   - Shows new accumulated scan results
                stop  - Stops scanning

advertise::

  Command:      advertise
  Description:  Bluetooth advertise commands
  Usage:        bt <ifname> advertise [-h] <start|stop>
  Where:        start - Starts advertising
                stop  - Stops advertising

security::

  Command:      security
  Description:  Enable security (encryption) for a connection:
                If device is paired, key encryption will be enabled.  If
                the link is already encrypted with sufficiently strong
                key this command does nothing.

                If the device is not paired pairing will be initiated. If
                the device is paired and keys are too weak but input output
                capabilities allow for strong enough keys pairing will be
                initiated.

                This command may return error if required level of security
                is not possible to achieve due to local or remote device
                limitation (eg input output capabilities).

bt::

  Usage:        bt <ifname> security [-h] <addr> public|random <level>
  Where:        <addr>  - The 6-byte address of the connected peer
                <level> - Security level, on of:

                  low     - No encryption and no authentication
                  medium  - Encryption and no authentication (no MITM)
                  high    - Encryption and authentication (MITM)
                  fips    - Authenticated LE secure connections and encryption

gatt::

  Command:      gatt
  Description:  Generic Attribute (GATT) commands
  Usage:        bt <ifname> gatt [-h] <cmd> [option [option [option...]]]
  Where:        See "GATT Commands" below

GATT Commands
-------------

exchange-mtu::

  Command:      exchange-mtu
  Description:  Set MTU to out maximum and negotiate MTU with peer
  Usage:        bt <ifname> gatt exchange-mtu [-h] <addr> public|random

mget::

  Command:      mget
  Description:  Get the pass/fail result of the last GATT 'exchange-mtu' command
  Usage:        bt <ifname> gatt mget [-h]

discover::

  Command:      discover
  Description:  Initiate discovery
  Usage:        bt <ifname> gatt discover [-h] <addr> public|random <uuid16> [<start> [<end>]]

characteristic::

  Command:      characteristic
  Description:  Initiate characteristics discovery
  Usage:        bt <ifname> gatt characteristic [-h] <addr> public|random [<start> [<end>]]

descriptor::

  Command:      descriptor
  Description:  Initiate characteristics discovery
  Usage:        bt <ifname> gatt descriptor [-h] <addr> public|random [<start> [<end>]]

dget::

  Command:      dget
  Description:  Get the result of the last discovery action
  Usage:        bt <ifname> gatt dget [-h]

read::

  Command:      read
  Description:  Initiate a GATT read operation.
  Usage:        bt <ifname> gatt read [-h] <addr> public|random <handle> [<offset>]

read-multiple::

  Command:      read-multiple
  Description:  Initiate a GATT read-multiple operation.
  Usage:        bt <ifname> gatt read-multiple [-h] <addr> public|random <handle> [<handle> [<handle>]..]

rget::

  Command:      rget
  Description:  Get the data resulting from the last read operation
  Usage:        bt <ifname> gatt rget [-h]

write::

  Command:      write
  Description:  Initiate a GATT write operation.
  Usage:        bt <ifname> gatt write [-h] <addr> public|random <handle> <byte> [<byte> [<byte>]..]

wget::

  Command:      wget
  Description:  Get the pass/fail result of the last GATT 'write' command
  Usage:        bt <ifname> gatt wget [-h]
