========
Commands
========

.. _cmdtest:

``test`` Evaluate Expression 
=============================

**Command Syntax:**

.. code-block:: fish

  [ <expression> ]
  test <expression>

**Synopsis**. These are two alternative forms of the same command.
They support evaluation of a boolean expression which sets
``$?``. This command is used most frequently as
the conditional command following the ``if`` in the
``if-then[-else]-fi``.

**Expression Syntax:**

::

    expression = simple-expression | !expression | expression -o expression | expression -a expression

    simple-expression = unary-expression | binary-expression

    unary-expression = string-unary | file-unary

    string-unary = -n string | -z string

    file-unary = -b file | -c file | -d file | -e file | -f file | -r file | -s file | -w file

    binary-expression = string-binary | numeric-binary

    string-binary = string = string | string == string | string != string

    numeric-binary = integer -eq integer | integer -ge integer | integer -gt integer | integer -le integer | integer -lt integer | integer -ne integer

.. _cmdaddroute:

``addroute`` Add a Routing Table Entry
======================================

**Command Syntax:**

::

  addroute <target> [<netmask>] <router>
  addroute default <ipaddr> <interface>

**Synopsis**. This command adds an entry in the routing table. The
new entry will map the IP address of a router on a local network
(<router>) to an external network characterized by the <target> IP
address and a network mask <netmask>

The netmask may also be expressed using IPv4 CIDR or IPv6 slash
notation. In that case, the netmask need not be provided.

**Example:**

::

  nsh> addroute 11.0.0.0 255.255.255.0 10.0.0.2

which is equivalent to

::

  nsh> addroute 11.0.0.0/24 10.0.0.2

The second form of the addroute command can be used to set the
default gateway.

.. _cmdarp:

``arp`` Access the ARP table
============================

**Command syntax**::

  arp [-t|-a <ipaddr> |-d <ipaddr> |-s <ipaddr> <hwaddr>]

**Synopsis**: Access the OS ARP table.

  -a <ipaddr>           Will show the hardware address that the IP address <ipaddr> is
                        mapped to.
  -d <ipaddr>           Will delete the mapping for the IP address <ipaddr> from the
                        ARP table.
  -s <ipaddr hwaddr>    Will set (or replace) the mapping of the IP address <ipaddr> to
                        the hardware address <hwaddr>.
  -t                    Will dump the entire content of the ARP table. This option is
                        only available if ``CONFIG_NETLINK_ROUTE`` is enabled.

**Example**::

  nsh> arp -a 10.0.0.1
  nsh: arp: no such ARP entry: 10.0.0.1

  nsh> arp -s 10.0.0.1 00:13:3b:12:73:e6
  nsh> arp -a 10.0.0.1
  HWAddr: 00:13:3b:12:73:e6

  nsh> arp -d 10.0.0.1
  nsh> arp -a 10.0.0.1
  nsh: arp: no such ARP entry: 10.0.0.1

.. _cmdbase64dec:

``base64dec`` Base64 Decode
===========================

**Command Syntax**::

  base64dec [-w] [-f] <string or filepath>

**Synopsis**. *To be provided.*

.. _cmdbase64enc:

``base64enc`` Base64 Encode
===========================

**Command Syntax**::

  base64enc [-w] [-f] <string or filepath>

**Synopsis**. *To be provided.*

.. _cmdbasename:

``basename`` Extract Base File/Directory Name
=============================================

**Command Syntax**::

  basename <path> [<suffix>]

**Synopsis**. Extract the final string from a ``<path>`` by
removing the preceding path segments and (optionally) removing any
trailing ``<suffix>``.

.. _cmdbreak:

``break`` Terminate a Loop
==========================

**Command Syntax**::

  break

**Synopsis**. The ``break`` command is only meaningful within the
body of the a ``while`` or ``until`` loop,
between the ``do`` and ``done`` tokens. Outside of a loop,
``break`` command does nothing. If the ``break`` command is
executed within the body of a loop, the loop will immediately
terminate and execution will continue with the next command
immediately following the ``done`` token.

.. _cmdcat:

``cat`` Concatenate Files
=========================

**Command Syntax**::

  cat <path> [<path> [<path> ...]]

**Synopsis**. This command copies and concatenates all of the
files at ``<path>`` to the console (or to another file if the
output is redirected).

.. _cmdcd:

``cd`` Change Current Working Directory
=======================================

**Command Syntax**::

  cd [<dir-path>|-|~|..]

**Synopsis**. Changes the current working directory (``PWD``).
Also sets the previous working directory environment variable
(``OLDPWD``).

**Forms:**

==================  =====================================
``cd <dir-path>``   sets the current working directory to <dir-path>.
``cd -``            sets the current working directory to the previous
                    working directory ($OLDPWD). Equivalent to cd $OLDPWD.
``cd`` or ``cd ~``  set the current working directory to the 'home' directory.
                    The home directory can be configured by setting CONFIG_LIBC_HOMEDIR
                    in the configuration file. The default home directory is /.
``cd ..`` 	        sets the current working directory to the parent directory.
==================  =====================================

.. _cmdcmp:

``cmp`` Compare Files
=====================

**Command Syntax**::

  cmp <path1> <path2>

**Synopsis**. Compare of the contents of the file at ``<path1>``
with the contents of the file at ``<path2>``. Returns an
indication only if the files differ.

.. _cmdcp:

``cp`` Copy Files
=================

**Command Syntax**::

  cp <source-path> <dest-path>

**Synopsis**. Copy of the contents of the file at
``<source-path>`` to the location in the file system indicated by
``<dest-path>``.

.. _cmddate:

``date`` Show or set the date and time
======================================

**Command Syntax**::

  date [-s "MMM DD HH:MM:SS YYYY"]

**Synopsis**. Show or set the current date and time.

To show the current system time and date, type in the ``date`` command.
The output displays the day of the week, day of the month, month, year,
current time. 24-hour time is used.
Only one format is used, both on display and when setting the date/time.
To change the system clock manually, type ``date -s MMM DD HH:MM:SS YYYY``. 

  -  ``MMM``  Short month name (e.g., Sep).
  -           Space separator.
  -  ``DD``   Day of month (e.g., 01).
  -           Space separator.
  -  ``HH``   Hour (00-23).
  -  ``:``    Colon separator.
  -  ``MM``   Minute (00-59).
  -  ``:``    Colon separator.
  -  ``SS``   Second (00-60).
  -           Space separator
  -  ``YYYY`` Year (e.g., 2023).

**Example**::

  nsh> date
  Thu, Jan 01 00:00:17 1970
  nsh> date -s "Sep 15 11:30:00 2023"
  nsh> date
  Fri, Sep 15 11:30:03 2023

.. _cmddd:

``dd`` Copy and Convert Files
=============================

**Command Syntax**::

  dd if=<infile> of=<outfile> [bs=<sectsize>] [count=<sectors>] [skip=<sectors>]

**Synopsis**. Copy blocks from <infile> to <outfile>. <infile> or
<outfile> may be the path to a standard file, a character device,
or a block device. Examples follow:

Read from character device, write to regular file. This will
create a new file of the specified size filled with zero::

  nsh> ls -l /dev
  /dev:
   crw-rw-rw-       0 zero
  nsh> dd if=/dev/zero of=/tmp/zeros bs=64 count=16
  nsh> ls -l /tmp
  /tmp:
   -rw-rw-rw-    1024 ZEROS

Read from character device, write to block device. This will fill
the entire block device with zeros::

  nsh> ls -l /dev
  /dev:
   brw-rw-rw-       0 ram0
   crw-rw-rw-       0 zero
  nsh> dd if=/dev/zero of=/dev/ram0

Read from a block device, write to a character device. This will
read the entire block device and dump the contents in the bit
bucket::

  nsh> ls -l /dev
  /dev:
   crw-rw-rw-       0 null
   brw-rw-rw-       0 ram0
  nsh> dd if=/dev/ram0 of=/dev/null

.. _cmddelroute:

``delroute`` Delete a Routing Table Entry
=========================================

**Command Syntax**::

  delroute <target> [<netmask>]

**Synopsis**. The entry removed will be the first entry in the
routing table that matches the external network characterized by
the <target> IP address and the network mask <netmask>

The netmask may also be expressed using IPv4 CIDR or IPv6 slash
notation. In that case, the netmask need not be provided.

**Example**::

  nsh> delroute 11.0.0.0 255.255.255.0

which is equivalent to::

  nsh> delroute 11.0.0.0/24

.. _cmddf:

``df`` Show Volume Status
=========================

**Command Syntax**::

  df [-h]

**Synopsis**. Show the state of each mounted volume. As an
example::

  nsh> mount
    /etc type romfs
    /tmp type vfat
  nsh> df
    Block  Number
    Size   Blocks     Used Available Mounted on
      64        6        6         0 /etc
     512      985        2       983 /tmp
  nsh>

If ``CONFIG_NSH_CMDOPT_DF_H`` is defined in the NuttX
configuration, then the ``df`` will also support an option ``-h``
which may be used to show the volume information in *human
readable* format.

.. _cmddirname:

``dirname`` Extract Path to a File/Directory 
============================================

**Command Syntax**::

  dirname <path>

**Synopsis**. Extract the path string leading up to the full
``<path>`` by removing the final directory or file name.

.. _cmddmesg:

``dmesg`` Dump Buffered SYSLOG Output
=====================================

**Command Syntax**::

  dmesg

**Synopsis**. This command can be used to dump (and clear) the
content of any buffered syslog output messages. This command is
only available if ``CONFIG_RAMLOG_SYSLOG`` is enabled. In that
case, syslog output will be collected in an in-memory, circular
buffer. Entering the ``dmesg`` command will dump the content of
that in-memory, circular buffer to the NSH console output.
``dmesg`` has the side effect of clearing the buffered data so
that entering ``dmesg`` again will show only newly buffered data.

.. _cmdecho:

``echo`` Echo Strings and Variables
===================================

**Command Syntax**::

  echo [-n] [<string|$name> [<string|$name>...]]

**Synopsis**. Copy the sequence of strings and expanded
environment variables to console output (or to a file if the
output is re-directed).

The ``-n`` option suppresses the trailing newline character.

.. _cmdenv:

``env`` Show Environment Variables
==================================

**Command Syntax**::

  env

**Synopsis**. Show the current name-value pairs in the
environment. Example::

  nsh> env
  PATH=/bin

  nsh> set foo bar
  nsh> env
  PATH=/bin
  foo=bar

  nsh> unset PATH
  nsh> env
  foo=bar

  nsh>

.. note::NSH local variables are *not* shown by the ``env``
  command.

.. _cmdexec:

``exec`` Execute User Code
==========================

**Command Syntax**::

  exec <hex-address>

**Synopsis**. Execute the user logic at address ``<hex-address>``.
NSH will pause until the execution unless the user logic is
executed in background via ``exec <hex-address> &``.

.. _cmdexit:

``exit`` Exit NSH
=================

**Command Syntax**::

  exit

**Synopsis**. Exit NSH. Only useful for the serial front end if
you have started some other tasks (perhaps using the ``exec``
command) and you would like to have NSH out of the way. For the
telnet front-end, ``exit`` terminates the telnet session.

.. _cmdexport:

``export`` Set an Environment Variable
======================================

**Command Syntax**::

  export <name> [<value>]

**Synopsis**. The ``export`` command sets an environment variable,
or promotes an NSH variable to an environment variable. As
examples:

  #. Using ``export`` to promote an NSH variable to an environment
     variable::

        nsh> env
        PATH=/bin

        nsh> set foo bar
        nsh> env
        PATH=/bin

        nsh> export foo
        nsh> env
        PATH=/bin
        foo=bar

     A group-wide environment variable is created with the same
     value as the local NSH variable; the local NSH variable is
     removed.

        .. note::This behavior differs from the Bash shell. Bash would
          retain the local Bash variable which will shadow the
          environment variable of the same name and same value.

  #. Using ``export`` to set an environment variable::

      nsh> export dog poop
      nsh> env
      PATH=/bin
      foo=bar
      dog=poop

The ``export`` command is not supported by NSH unless both
``CONFIG_NSH_VARS=y`` and ``CONFIG_DISABLE_ENVIRON``\ is not set.

.. _cmdfree:

``free`` Show Memory Manager Status
===================================

**Command Syntax**::

  free

**Synopsis**. Show the current state of the memory allocator. For
example::

  nsh> free
               total       used       free    largest  nused  nfree
  Mem:       5583024    1614784    3968240    3967792    244      4

  nsh>

**Where:**

=======  ======================================
total 	 This is the total size of memory allocated for use by malloc in bytes.
used     This is the total size of memory occupied by chunks handed out by malloc.
free     This is the total size of memory occupied by free (not in use) chunks.
largest  Size of the largest free (not in use) chunk.
nused    This is the number of allocated chunks
nfree    This is the number of free chunks
=======  ======================================

.. _cmdget:

``get`` Get File Via TFTP
=========================

**Command Syntax**::

  get [-b|-n] [-f <local-path>] -h <ip-address> <remote-path>

**Synopsis**. Copy the file at ``<remote-address>`` from the host
whose IP address is identified by ``<ip-address>``.

**Other options**

===================  ============================================
``-f <local-path>``  The file will be saved relative to the current working directory unless <local-path> is provided.
``-n``               Selects text ("netascii") transfer mode (default).
``-b``               Selects binary ("octet") transfer mode
===================  ============================================

.. _cmdhelp:

``help`` Show Usage Command Usage
=================================

**Command Syntax**::

  help [-v] [<cmd>]

**Synopsis**. Presents summary information about NSH commands to
console.

**Options**

========= ====================
``-v``    how verbose output will full command usage.
``<cmd>`` Show full command usage only for this command.
========= ====================

.. _cmdhexdump:

``hexdump`` Hexadecimal Dump of File or Device
==============================================

**Command Syntax**::

  hexdump <file or device> [skip=<bytes>] [count=<bytes>]

**Synopsis**. Dump data in hexadecimal format from a file or
character device

================= ==================================
``skip=<bytes>``  Will skip <bytes> number of bytes from the beginning.
``count=<bytes>`` Will stop after dumping <bytes> number of bytes.
================= ==================================

The ``skip`` and ``count`` options are only available if
``CONFIG_NSH_CMDOPT_HEXDUMP`` is defined in the NuttX
configuration.

.. _cmdifconfig:

``ifconfig`` Manage Network Configuration
=========================================

**Command Syntax**::

  ifconfig [nic_name [<ip-address>|dhcp]] [dr|gw|gateway <dr-address>] [netmask <net-mask>] [dns <dns-address>] [hw <hw-mac>]]

**Synopsis**. Multiple forms of the ``ifconfig`` command are
supported:

  #. With one or no arguments, ``ifconfig`` will shows the current
     configuration of the network and, perhaps, the status of
     Ethernet device::

       ifconfig
       ifconfig [nic_name]

     As an example::

       nsh> ifconfig
       eth0    HWaddr 00:18:11:80:10:06
               IPaddr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

     If network statistics are enabled (``CONFIG_NET_STATISTICS``),
     then this command will also show the detailed state of network.

  #. If both the network interface name and an IP address are
     supplied as arguments, then ``ifconfig`` will set the address
     of the Ethernet device::

      ifconfig nic_name ip_address

  #. Other forms *to be provided*

.. note:: This commands depends upon having the *procfs* file system
  configured into the system. The *procfs* file system must also
  have been mounted with a command like::

    nsh> mount -t procfs /proc

.. _cmdifdown:

``ifdown`` Take a network down
==============================

**Command Syntax**::

  ifdown <interface>

**Synopsis**. Take down the interface identified by the name
<interface>.

**Example**::

  ifdown eth0

.. _cmdifup:

``ifup`` Bring a network up
===========================

**Command Syntax**::

  ifup <interface>

**Synopsis**. Bring up down the interface identified by the name
<interface>.

**Example**::

  ifup eth0

.. _cmdinsmod:

``insmod`` Install an OS module
===============================

**Command Syntax**::

  insmod <file-path> <module-name>

**Synopsis**. Install the loadable OS module at <file-path> as
module <module-name>.

**Example**::

  nsh> ls -l /mnt/romfs
  /mnt/romfs:
   dr-xr-xr-x       0 .
   -r-xr-xr-x    9153 chardev
  nsh> ls -l /dev
  /dev:
   crw-rw-rw-       0 console
   crw-rw-rw-       0 null
   brw-rw-rw-       0 ram0
   crw-rw-rw-       0 ttyS0
  nsh> lsmod
  NAME                 INIT   UNINIT      ARG     TEXT     SIZE     DATA     SIZE
  nsh> insmod /mnt/romfs/chardev mydriver
  nsh> ls -l /dev
  /dev:
   crw-rw-rw-       0 chardev
   crw-rw-rw-       0 console
   crw-rw-rw-       0 null
   brw-rw-rw-       0 ram0
   crw-rw-rw-       0 ttyS0
  nsh> lsmod
  NAME                 INIT   UNINIT      ARG     TEXT     SIZE     DATA     SIZE
  mydriver         20404659 20404625        0 20404580      552 204047a8        0

.. _cmdirqinfo:

``irqinfo`` Show Interrupt Status
=================================

**Command Syntax**::

  irqinfo

**Synopsis**. Show the current count of interrupts taken on all
attached interrupts.

**Example**::

  nsh> irqinfo
  IRQ HANDLER  ARGUMENT    COUNT    RATE
    3 00001b3d 00000000        156   19.122
   15 0000800d 00000000        817  100.000
   30 00000fd5 20000018         20    2.490

.. _cmdcritmon:

``critmon`` Show Critical Monitor Status
========================================

**Command Syntax**::

  critmon

**Synopsis**. Show the preemption time, critical section time,
longest single run time, total run time, process ID (PID),
and thread description of each thread in the system.

**Example**::

  nsh> critmon
  PRE-EMPTION   CSECTION      RUN         TIME         PID   DESCRIPTION
  0.010265000   0.000037000   ----------- ------------ ----  CPU 0
  0.000000000   0.000000000   0.001237000 28.421047000 0     Idle Task
  0.000011000   0.000037000   0.000046000 0.034211000  1     loop_task
  0.000000000   0.000028000   0.000067000 0.236657000  2     hpwork

In this example, the output shows the preemption time, critical section time,
longest single run time, total run time, and thread description for each
thread in the system.

The output of the ``critmon`` command displays the following columns:

- PRE-EMPTION: Preemption time
- CSECTION: Critical section time
- RUN: Longest single run time of the thread
- TIME: Total run time of the thread
- PID: Process ID of the thread
- DESCRIPTION: Thread description (name)

.. _cmdkill:

``kill`` Send a signal to a task
================================

**Command Syntax**::

  kill -<signal> <pid>

**Synopsis**. Send the <signal> to the task identified by <pid>.

**Example**::

  nsh> mkfifo /dev/fifo
  nsh> cat /dev/fifo &
  cat [2:128]
  nsh> ps
  PID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK  COMMAND
    0   0 FIFO     Kthread --- Ready              00000000 Idle Task
    1 128 RR       Task    --- Running            00000000 init
    2 128 FIFO     pthread --- Waiting  Semaphore 00000000 <pthread>(51ea50)
  nsh> kill -9 2
  nsh> ps
  PID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK  COMMAND
    0   0 FIFO     Kthread --- Ready              00000000 Idle Task
    1 128 RR       Task    --- Running            00000000 init
  nsh>

.. note:: NuttX does not support a FULL POSIX signaling system. A
  few standard signal names like ``SIGCHLD``, ``SIGUSR1``,
  ``SIGUSR2``, ``SIGALRM``, and ``SIGPOLL`` exist in the system.
  However, they do not have the default actions that you might
  expect. Rather, NuttX supports only what are referred to as POSIX
  real-time signals. These signals may be used to communicate with
  running tasks, may be use to waiting waiting tasks, etc.

  If the configuration option ``CONFIG_SIG_DEFAULT`` is enabled,
  then default actions for the ``SIGINT`` and ``SIGKILL`` signals
  (only) will be supported. In that case, as an example, ``kill -9``
  (SIGKILL) will, indeed, terminate a task. Caution should be
  exercised, however, because this is likely to cause memory leaks
  and to strand resource since there is insufficient clean-up in
  certain build configurations.

.. _cmdlosetup:

``losetup`` Setup/teardown the Loop Device
==========================================

**Command Syntax 1**::

  losetup [-o <offset>] [-r] <dev-path> <file-path>

**Synopsis**. Setup the loop device at <dev-path> to access the
file at <file-path> as a block device. In the following example a
256K file is created (``dd``) and ``losetup`` is used to make the
file accessible as a block device. A FAT file system is created
(``mkfatfs``) and mounted (``mount``). Files can then be managed
on the loop-mounted file::

  nsh> dd if=/dev/zero of=/tmp/image bs=512 count=512
  nsh> ls -l /tmp
  /tmp:
   -rw-rw-rw-   262144 IMAGE
  nsh> losetup /dev/loop0 /tmp/image
  nsh> ls -l /dev
  /dev:
   brw-rw-rw-       0 loop0
  nsh> mkfatfs /dev/loop0
  nsh> mount -t vfat /dev/loop0 /mnt/example
  nsh> ls -l /mnt
  ls -l /mnt
  /mnt:
   drw-rw-rw-       0 example/
  nsh> echo "This is a test" >/mnt/example/atest.txt
  nsh> ls -l /mnt/example
  /mnt/example:
   -rw-rw-rw-      16 ATEST.TXT
  nsh> cat /mnt/example/atest.txt
  This is a test
  nsh>

**Command Syntax 2**::

  losetup d <dev-path>

**Synopsis**. Teardown the setup for the loop device at
<dev-path>.

.. _cmdln:

``ln`` Link to a File or Directory
==================================

**Command Syntax**::

  ln [-s] <target> <link>

**Synopsis**. The ``ln`` command will create a new symbolic link
at <link> for the existing file or directory, <target>. This
implementation is simplified for use with NuttX in these ways:

  -  Links may be created only within the NuttX top-level,
     :ref:`pseudo file system <file_system_overview>` No
     file system currently supported by NuttX provides symbolic
     links.
  -  For the same reason, only soft links are implemented.
  -  File privileges are ignored.
  -  ``c_time`` is not updated.

.. _cmdls:

``ls`` List Directory Contents
==============================

**Command Syntax**::

  ls [-lRs] <dir-path>

**Synopsis**. Show the contents of the directory at
``<dir-path>``. NOTE: ``<dir-path>`` must refer to a directory and
no other file system object.

**Options**

======  ================================
``-R`` 	Show the contents of specified directory and all of its sub-directories.
``-s`` 	Show the size of the files along with the filenames in the listing
``-l`` 	Show size and mode information along with the filenames in the listing.
======  ================================

.. _cmdlsmod:

``lsmod`` Show information about installed OS modules
=====================================================

**Command Syntax**::

  lsmod

**Synopsis**. Show information about the currently installed OS
modules. This information includes:

  -  The module name assigned to the module when it was installed
     (``NAME``, string).
  -  The address of the module initialization function (``INIT``,
     hexadecimal).
  -  The address of the module un-initialization function
     (``UNINIT``, hexadecimal).
  -  An argument that will be passed to the module un-initialization
     function (``ARG``, hexadecimal).
  -  The start of the .text memory region (``TEXT``, hexadecimal).
  -  The size of the .text memory region size (``SIZE``, decimal).
  -  The start of the .bss/.data memory region (``DATA``,
     hexadecimal).
  -  The size of the .bss/.data memory region size (``SIZE``,
     decimal).

**Example**::

  nsh> lsmod
  NAME                 INIT   UNINIT      ARG     TEXT     SIZE     DATA     SIZE
  mydriver         20404659 20404625        0 20404580      552 204047a8        0

.. _cmdmd5:

``md5`` Calculate MD5
=====================

**Command Syntax**::

  md5 [-f] <string or filepath>

**Synopsis**. *To be provided.*

.. _cmdmx:

``mb``, ``mh``, ``and`` ``mw`` Access Memory
============================================

**Command Syntax**::

  mb <hex-address>[=<hex-value>][ <hex-byte-count>]
  mh <hex-address>[=<hex-value>][ <hex-byte-count>]
  mw <hex-address>[=<hex-value>][ <hex-byte-count>]

**Synopsis**. Access memory using byte size access (mb), 16-bit
accesses (mh), or 32-bit access (mw). In each case,

=============================  ==============================================
``<hex-address>``              Specifies the address to be accessed. The current
                               value at that address will always be read and displayed.
``<hex-address>=<hex-value>``  Read the value, then write <hex-value> to the location.
``<hex-byte-count>``           Perform the mb, mh, or mw operation on a total of
                               <hex-byte-count> bytes, increment the <hex-address>
                               appropriately after each access.
=============================  ==============================================

**Example**::

  nsh> mh 0 16
    0 = 0x0c1e
    2 = 0x0100
    4 = 0x0c1e
    6 = 0x0110
    8 = 0x0c1e
    a = 0x0120
    c = 0x0c1e
    e = 0x0130
    10 = 0x0c1e
    12 = 0x0140
    14 = 0x0c1e
  nsh>

.. _cmdps:

``ps`` Show Current Tasks and Threads
=====================================

**Command Syntax**::

  ps

**Synopsis**. Show the currently active threads and tasks. For
example::

  nsh> ps
  PID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK  COMMAND
    0   0 FIFO     Kthread --- Ready              00000000 Idle Task
    1 128 RR       Task    --- Running            00000000 init
    2 128 FIFO     Task    --- Waiting  Semaphore 00000000 nsh_telnetmain()
    3 100 RR       pthread --- Waiting  Semaphore 00000000 <pthread>(21)
  nsh>

NOTE: This commands depends upon having the *procfs* file system
configured into the system. The *procfs* file system must also
have been mounted with a command like::

  nsh> mount -t procfs /proc

.. _cmdmkdir:

``mkdir`` Create a Directory
============================

**Command Syntax**::

  mkdir <path>

**Synopsis**. Create the directory at ``<path>``. All components
of ``<path>`` except the final directory name must exist on a
mounted file system; the final directory must not.

**Limited to Mounted File Systems**. Recall that NuttX uses a
:ref:`pseudo file system <file_system_overview>`
for its root file system. The ``mkdir`` command can only be used
to create directories in volumes set up with the
:ref:`mount <cmdmount>` command; it cannot be used to create
directories in the *pseudo* file system.

**Example**::

  nsh> mkdir /mnt/fs/tmp
  nsh> ls -l /mnt/fs
  /mnt/fs:
   drw-rw-rw-       0 TESTDIR/
   drw-rw-rw-       0 TMP/
  nsh>

.. _cmdmkfatfs:

``mkfatfs`` Create a FAT File System
====================================

**Command Syntax**

  mkfatfs [-F <fatsize>] [-r <rootdirentries>] <block-driver>

**Synopsis**. Format a fat file system on the block device
specified by ``<block-driver>`` path. The FAT size may be provided
as an option. Without the ``<fatsize>`` option, ``mkfatfs`` will
select either the FAT12 or FAT16 format. For historical reasons,
if you want the FAT32 format, it must be explicitly specified on
the command line.

The ``-r`` option may be specified to select the the number of
entries in the root directory for FAT12 and FAT16 file systems.
Typical values for small volumes would be 112 or 224; 512 should
be used for large volumes, such as hard disks or very large SD
cards. The default is 512 entries in all cases.

The reported number of root directory entries used with FAT32 is
zero because the FAT32 root directory is a cluster chain.

NSH provides this command to access the
``mkfatfs()`` NuttX API. This block device must
reside in the NuttX :ref:`pseudo file system <file_system_overview>`
and must have been created by some call to ``register_blockdriver()``
(see ``include/nuttx/fs/fs.h``).

.. _cmdmkfifo:

``mkfifo`` Create a FIFO
========================

**Command Syntax**::

  mkfifo <path>

**Synopsis**. Creates a FIFO character device anywhere in the
pseudo file system, creating whatever pseudo directories that may
be needed to complete the ``<path>``. By convention, however,
device drivers are place in the standard ``/dev`` directory. After
it is created, the FIFO device may be used as any other device
driver. NSH provides this command to access the
```mkfifo()`` NuttX API.

**Example**::

  nsh> ls -l /dev
  /dev:
   crw-rw-rw-       0 console
   crw-rw-rw-       0 null
   brw-rw-rw-       0 ram0
  nsh> mkfifo /dev/fifo
  nsh> ls -l /dev
  ls -l /dev
  /dev:
   crw-rw-rw-       0 console
   crw-rw-rw-       0 fifo
   crw-rw-rw-       0 null
   brw-rw-rw-       0 ram0
  nsh>

.. _cmdmkrd:

``mkrd`` Create a RAMDISK
=========================

**Command Syntax**::

  mkrd [-m <minor>] [-s <sector-size>] <nsectors>

**Synopsis**. Create a ramdisk consisting of ``<nsectors>``, each
of size ``<sector-size>`` (or 512 bytes if ``<sector-size>`` is
not specified. The ramdisk will be registered as
``/dev/ram<minor>``. If ``<minor>`` is not specified, ``mkrd``
will attempt to register the ramdisk as ``/dev/ram0``.

**Example**::

  nsh> ls /dev
  /dev:
   console
   null
   ttyS0
   ttyS1
  nsh> mkrd 1024
  nsh> ls /dev
  /dev:
   console
   null
   ram0
   ttyS0
   ttyS1
  nsh>

Once the ramdisk has been created, it may be formatted using the
``mkfatfs`` command and mounted using the ``mount`` command.

**Example**::

  nsh> mkrd 1024
  nsh> mkfatfs /dev/ram0
  nsh> mount -t vfat /dev/ram0 /tmp
  nsh> ls /tmp
  /tmp:
  nsh>

.. _cmdmount:

``mount`` Mount a File System
=============================

**Command Syntax**::

  mount -t <fstype> [-o <options>] <block-device> <dir-path>

**Synopsis**. The ``mount`` command performs one of two different
operations. If no parameters are provided on the command line
after the ``mount`` command, then the ``mount`` command will
enumerate all of the current mountpoints on the console.

If the mount parameters are provided on the command after the
``mount`` command, then the ``mount`` command will mount a file
system in the NuttX pseudo-file system. ``mount`` performs a three
way association, binding:

  #. **File System.** The '-t ``<fstype>``' option identifies the
     type of file system that has been formatted on the
     ``<block-device>``. As of this writing, ``vfat`` is the only
     supported value for ``<fstype>``
  #. **Block Device.** The ``<block-device>`` argument is the full
     or relative path to a block driver inode in the
     :ref:`pseudo file system <file_system_overview>`. By
     convention, this is a name under the ``/dev`` sub-directory.
     This ``<block-device>`` must have been previously formatted
     with the same file system type as specified by ``<fstype>``
  #. **Mount Point.** The mount point, ``<dir-path>``, is the
     location in the :ref:`pseudo file system <file_system_overview>`
     where the mounted volume will appear. This mount point can only
     reside in the NuttX
     :ref:`pseudo file system <file_system_overview>`. By
     convention, this mount point is a subdirectory under ``/mnt``.
     The mount command will create whatever pseudo directories that
     may be needed to complete the full path but the full path must
     not already exist.

After the volume has been mounted in the NuttX
:ref:`pseudo file system <file_system_overview>`, it may be
access in the same way as other objects in the file system.

**Examples**:

Using ``mount`` to mount a file system::

  nsh> ls -l /dev
  /dev:
   crw-rw-rw-       0 console
   crw-rw-rw-       0 null
   brw-rw-rw-       0 ram0
  nsh> ls /mnt
  nsh: ls: no such directory: /mnt
  nsh> mount -t vfat /dev/ram0 /mnt/fs
  nsh> ls -l /mnt/fs/testdir
  /mnt/fs/testdir:
   -rw-rw-rw-      15 TESTFILE.TXT
  nsh> echo "This is a test" >/mnt/fs/testdir/example.txt
  nsh> ls -l /mnt/fs/testdir
  /mnt/fs/testdir:
  -rw-rw-rw-      15 TESTFILE.TXT
   -rw-rw-rw-      16 EXAMPLE.TXT
  nsh> cat /mnt/fs/testdir/example.txt
  This is a test
  nsh>

Using ``mount`` to enumerate mounts::

  nsh> mount
    /etc type romfs
    /mnt/fs type vfat
    /tmp type vfat

.. _cmdmv:

``mv`` Rename a File
====================

**Command Syntax**::

  mv <old-path> <new-path>

**Synopsis**. Rename the file object at ``<old-path>`` to
``<new-path>``. Both paths must reside in the same mounted file
system.

.. _cmdnfsmount:

``nfsmount`` Mount an NFS file system
=====================================

**Command Syntax**::

  nfsmount <server-address> <mount-point> <remote-path>

**Synopsis**. Mount the remote NFS server directory<remote-path>
at <mount-point> on the target machine. <server-address> is the IP
address of the remote server.

.. _cmdnslookup:

``nslookup`` Lookup a network address
=====================================

**Command Syntax**::

  nslookup <host-name>

**Synopsis**. Lookup and print the IP address associated with
``<host-name>``.

.. _cmdpasswd:

``passwd`` Change a User's Password
===================================

**Command Syntax**::

  passwd <username> <password>

**Synopsis**. Set the password for the existing user <username> to
<password>.

.. _cmdpmconfig:

``pmconfig`` Manage Power Management Subsystem
==============================================

**Command Syntax**::

  pmconfig [stay|relax] [normal|idle|standby|sleep]

**Synopsis**. Control power management subsystem.

.. _cmdpoweroff:

``poweroff`` Shut the system down
=================================

**Command Syntax**::

  poweroff [<n>]

**Synopsis**. Shutdown and power off the system immediately. This
command depends on board-specific hardware support to power down
the system. The optional,decimal numeric argument may be included
to provide power off mode to board-specific power off logic.

NOTE: Supporting both the ``poweroff`` and ``shutdown`` commands
is redundant.

.. _cmdput:

``put`` Send File Via TFTP
==========================

**Command Syntax**::

  put [-b|-n] [-f <remote-path>] -h <ip-address> <local-path>

**Synopsis**. Copy the file at ``<local-address>`` to the host
whose IP address is identified by ``<ip-address>``.

**Other options:**

====================  =============================================
``-f <remote-path>``  The file will be saved relative with the same
                      name on the host unless <remote-path> is provided.
``-b|-n``             Selects either binary ("octet") or text ("netascii")
                      transfer mode. Default: text.
====================  =============================================

.. _cmdpwd:

``pwd`` Show Current Working Directory
======================================

**Command Syntax**::

  pwd

**Synopsis**. Show the current working directory::

  nsh> cd /dev
  nsh> pwd
  /dev
  nsh>

Same as ``echo $PWD``::

  nsh> echo $PWD
  /dev
  nsh>

.. _cmdreadlink:

``readlink`` Show target of a link
==================================

**Command Syntax**::

  readlink <link>

**Synopsis**. Show the target of the soft link at the path
``<link>``.

.. _cmdreboot:

``reboot`` Reboot the system
============================

**Command Syntax**::

  reboot [<n>]

**Synopsis**. Reset and reboot the system immediately. This
command depends on hardware support to reset the system. The
optional, decimal numeric argument <n> may be included to provide
a reboot mode to board-specific reboot logic.

NOTE: Supporting both the ``reboot`` and ``shutdown`` commands is
redundant.

.. _cmdrm:

``rm`` Remove a File
====================

**Command Syntax**::

  rm <file-path>

**Synopsis**. Remove the specified ``<file-path>`` name from the
mounted file system. Recall that NuttX uses a
:ref:`pseudo file system <file_system_overview>` for its root
file system. The ``rm`` command can only be used to remove
(unlink) files in volumes set up with the
:ref:`mount <cmdmount>` command; it cannot be used to remove
names in the *pseudo* file system.

**Example**::

  nsh> ls /mnt/fs/testdir
  /mnt/fs/testdir:
   TESTFILE.TXT
   EXAMPLE.TXT
  nsh> rm /mnt/fs/testdir/example.txt
  nsh> ls /mnt/fs/testdir
  /mnt/fs/testdir:
   TESTFILE.TXT
  nsh>

.. _cmdrmdir:

``rmdir`` Remove a Directory
============================

**Command Syntax**::

  rmdir <dir-path>

**Synopsis**. Remove the specified ``<dir-path>`` directory from
the mounted file system. Recall that NuttX uses a
:ref:`pseudo file system <file_system_overview>` for its root
file system. The ``rmdir`` command can only be used to remove
directories from volumes set up with the :ref:`mount <cmdmount>`
command it cannot be used to remove directories from the *pseudo*
file system.

**Example**::

  nsh> mkdir /mnt/fs/tmp
  nsh> ls -l /mnt/fs
  /mnt/fs:
   drw-rw-rw-       0 TESTDIR/
   drw-rw-rw-       0 TMP/
  nsh> rmdir /mnt/fs/tmp
  nsh> ls -l /mnt/fs
  /mnt/fs:
   drw-rw-rw-       0 TESTDIR/
  nsh>

.. _cmdrmmod:

``rmmod`` Remove on OS Module
=============================

**Command Syntax**::

  rmmod <module-name>

**Synopsis**. Remove the loadable OS module with the
<module-name>. NOTE: An OS module can only be removed if it is not
busy.

**Example**::

  nsh> lsmod
  NAME                 INIT   UNINIT      ARG     TEXT     SIZE     DATA     SIZE
  mydriver         20404659 20404625        0 20404580      552 204047a8        0
  nsh> rmmod mydriver
  nsh> lsmod
  NAME                 INIT   UNINIT      ARG     TEXT     SIZE     DATA     SIZE
  nsh>

.. _cmdroute:

``route`` Show routing table
============================

**Command Syntax**::

  route ipv4|ipv6

**Synopsis**. Show the contents of routing table for IPv4 or IPv6.

If only IPv4 or IPv6 is enabled, then the argument is optional
but, if provided, must match the enabled internet protocol
version.

.. _cmdrptun:

``rptun`` Start/Stop the OpenAMP RPC Tunnel
===========================================

**Command Syntax**::

  rptun start|stop <dev-path>

**Synopsis**. Start or stop the OpenAMP RPC tunnel device at <dev-path>.

.. _cmdset:

``set`` Set a Variable
======================

**Command Syntax**::

  set [{+|-}{e|x|xe|ex}] [<name> <value>]

**Synopsis**. Set the variable ``<name>`` to the string ``<value>`` and
or set NSH parser control options.

For example, a variable may be set like this::

  nsh> echo $foobar

  nsh> set foobar foovalue
  nsh> echo $foobar
  foovalue
  nsh>

If ``CONFIG_NSH_VARS`` is selected, the effect of this ``set`` command
is to set the local NSH variable. Otherwise, the group-wide environment
variable will be set.

If the local NSH variable has already been *promoted* to an environment
variable via the :ref:`export <cmdexport>`, then the ``set`` command
will set the value of the environment variable rather than the local NSH
variable.

.. note:: The Bash shell does not work this way. Bash would set the value
  of both the local Bash variable and the environment variable of the
  same name to the same value.

If ``CONFIG_NSH_VARS=y`` is selected and no arguments are provided, then
the ``set`` command will list all of the local NSH variables::

  nsh> set
  foolbar=foovalue

Set the *exit on error control* and/or *print a trace* of commands when
parsing scripts in NSH. The settings are in effect from the point of
execution, until they are changed again, or in the case of the
initialization script, the settings are returned to the default settings
when it exits. Included child scripts will run with the parents settings
and changes made in the child script will effect the parent on return.

  -  Use ``set -e`` to enable and ``set +e`` to disable (ignore) the exit
     condition on commands. The default is -e. Errors cause script to
     exit.

  -  Use ``set -x`` to enable and ``set +x`` to disable (silence) printing
     a trace of the script commands as they are executed. The default is
     ``+x``: no printing of a trace of script commands as they are
     executed.

Example 1 - no exit on command not found::

  set +e
  notacommand

Example 2 - will exit on command not found::

  set -e
  notacommand

Example 3 - will exit on command not found, and print a trace of the
script commands::

  set -ex

Example 4 - will exit on command not found, and print a trace of the
script commands and set foobar to foovalue::

  set -ex foobar foovalue
  nsh> echo $foobar
  foovalue

.. _cmdsh:

``sh`` Execute an NSH Script
============================

**Command Syntax**::

  sh <script-path>

**Synopsis**. Execute the sequence of NSH commands in the file referred
to by ``<script-path>``.

.. _cmdshutdown:

``shutdown`` Shut the system down
=================================

**Command Syntax**::

  shutdown [--reboot]

**Synopsis**. Shutdown and power off the system or, optionally, reset
and reboot the system immediately. This command depends on hardware
support to power down or reset the system; one, both, or neither
behavior may be supported.

NOTE: The ``shutdown`` command duplicates the behavior of the
``poweroff`` and ``eboot`` commands.

.. _cmdsleep:

``sleep`` Wait for Seconds
==========================

**Command Syntax**::

  sleep <sec>

**Synopsis**. Pause execution (sleep) for ``<sec>`` seconds.

.. _cmdtelnetd:

``telnetd`` Time Start the Telnet Daemon
========================================

**Command Syntax**::

  telnetd

**Synopsis**. Start the Telnet daemon if it is not already running.

The Telnet daemon may be started either programmatically by calling
``nsh_telnetstart()`` or it may be started from the NSH command line
using this ``telnetd`` command.

Normally this command would be suppressed without
``CONFIG_SYSTEM_TELNETD`` because the Telnet daemon is
automatically started in ``nsh_main.c``. The exception is when
``CONFIG_NSH_NETLOCAL`` is selected. In that case, the network is not
enabled at initialization but rather must be enabled from the NSH
command line or via other applications.

In that case, when ``nsh_telnetstart()`` is called before the the
network is initialized, it will fail.

.. _cmdtime:

``time`` Time execution of another command
==========================================

**Command Syntax**::

  time "<command>"

**Synopsis**. Perform command timing. This command will execute the
following <command> string and then show how much time was required to
execute the command. Time is shown with a resolution of 100 microseconds
which may be beyond the resolution of many configurations. Note that the
<command> must be enclosed in quotation marks if it contains spaces or
other delimiters.

**Example**::

  nsh> time "sleep 2"

  2.0100 sec
  nsh>

The additional 10 milliseconds in this example is due to the way that
the sleep command works: It always waits one system clock tick longer
than requested and this test setup used a 10 millisecond periodic system
timer. Sources of error could include various quantization errors,
competing CPU usage, and the additional overhead of the time command
execution itself which is included in the total.

The reported time is the elapsed time from starting of the command to
completion of the command. This elapsed time may not necessarily be just
the processing time for the command. It may included interrupt level
processing, for example. In a busy system, command processing could be
delayed if pre-empted by other, higher priority threads competing for
CPU time. So the reported time includes all CPU processing from the
start of the command to its finish possibly including unrelated
processing time during that interval.

Notice that::

  nsh> time "sleep 2 &"
  sleep [3:100]

  0.0000 sec
  nsh>

Since the sleep command is executed in background, the sleep command
completes almost immediately. As opposed to the following where the time
command is run in background with the sleep command::

  nsh> time "sleep 2" &
  time [3:100]
  nsh>
  2.0100 sec

.. _cmdtruncate:

``truncate`` Set the Size of a File
===================================

**Command Syntax**::

  truncate -s <length> <file-path>

**Synopsis**. Shrink or extend the size of the regular file at
<file-path> to the specified<length>.

A <file-path> argument that does not exist is created. The <length>
option is NOT optional.

If a <file-path> is larger than the specified size, the extra data is
lost. If a <file-path> is shorter, it is extended and the extended part
reads as zero bytes.

.. _cmdumount:

``umount`` Unmount a File System
================================

**Command Syntax**::

  umount <dir-path>

**Synopsis**. Un-mount the file system at mount point ``<dir-path>``.
The ``umount`` command can only be used to un-mount volumes previously
mounted using :ref:`mount <cmdmount>` command.

**Example**::

  nsh> ls /mnt/fs
  /mnt/fs:
   TESTDIR/
  nsh> umount /mnt/fs
  nsh> ls /mnt/fs
  /mnt/fs:
  nsh: ls: no such directory: /mnt/fs
  nsh>

.. _cmduname:

``uname`` Print system information
==================================

**Command Syntax**::

  uname [-a | -imnoprsv]

**Synopsis**. Print certain system information. With no options, the
output is the same as -s.

==========  ========================================
``-a``      Print all information, in the following
            order, except omit -p and -i if unknown:
``-s, -o``  Print the operating system name (NuttX)
``-n``      Print the network node hostname (only available if CONFIG_NET=y)
``-r``      Print the kernel release
``-v``      Print the kernel version
``-m``      Print the machine hardware name
``-i``      Print the machine platform name
``-p``      Print "unknown"
==========  ========================================

.. _cmdunset:

``unset`` Unset an Environment Variable
=======================================

**Command Syntax**:

  unset <name>

**Synopsis**. Remove the value associated with the variable ``<name>``.
This will remove the name-value pair from both the NSH local variables
and the group-wide environment variables. For example::

  nsh> echo $foobar
  foovalue
  nsh> unset foobar
  nsh> echo $foobar

  nsh>

.. _cmdurldecode:

``urldecode`` URL Decode
========================

**Command Syntax**::

  urldecode [-f] <string or filepath>

**Synopsis**. *To be provided.*

.. _cmdurlencode:

``urlencode`` URL Encode
========================

**Command Syntax**::

  urlencode [-f] <string or filepath>

**Synopsis**. *To be provided.*

.. _cmduseradd:

``useradd`` Add a New User
==========================

**Command Syntax**::

  useradd <username> <password>

**Synopsis**. Add a new user with <username> and <password>.

.. _cmduserdel:

``userdel`` Delete a user
=========================

**Command Syntax**::

  userdel <username>

**Synopsis**. Delete the user with the name <username>.

.. _cmdusleep:

``usleep`` Wait for Microseconds
================================

**Command Syntax**::

  usleep <usec>

**Synopsis**. Pause execution (sleep) of ``<usec>`` microseconds.

.. _cmdwget:

``wget`` Get File Via HTTP
==========================

**Command Syntax**

  wget [-o <local-path>] <url>

**Synopsis**. Use HTTP to copy the file at ``<url>`` to the current
directory.

**Options**

===================  =================================================
``-o <local-path>``  The file will be saved relative to the current working
                     directory and with the same name as on the HTTP server
                     unless <local-path> is provided.
===================  =================================================

.. _cmdxd:

``xd`` Hexadecimal Dump of Memory
=================================

**Command Syntax**::

  xd <hex-address> <byte-count>

**Synopsis**. Dump ``<byte-count>`` bytes of data from address
``<hex-address>``.

**Example**::

  nsh> xd 410e0 512
  Hex dump:
  0000: 00 00 00 00 9c 9d 03 00 00 00 00 01 11 01 10 06 ................
  0010: 12 01 11 01 25 08 13 0b 03 08 1b 08 00 00 02 24 ....%..........$
  ...
  01f0: 08 3a 0b 3b 0b 49 13 00 00 04 13 01 01 13 03 08 .:.;.I..........
  nsh>

Built-In Commands
=================

In addition to the commands that are part of NSH listed in the previous
section above, there can be additional, external *built-in* applications
that can be added to NSH. These are separately excecuble programs but
will appear much like the commands that are a part of NSH. The primary
difference from the user's perspective is that help information about
the built-in applications is not available directly from NSH. Rather,
you will need to execute the application with the ``-h`` option to get
help about using the built-in applications.

There are several built-in applications in the ``apps/`` repository. No
attempt is made here to enumerate all of them. But a few of the more
common, useful built-in applications are listed below.

.. _cmdping:

``ping`` and ``ping6`` Check Network Peer
=========================================

**Command Syntax**::

  ping  [-c <count>] [-i <interval>] <ip-address>
  ping6 [-c <count>] [-i <interval>] <ip-address>

**Synopsis**. Test the network communication with a remote peer.
Example::

  nsh> ping 10.0.0.1
  PING 10.0.0.1 56 bytes of data
  56 bytes from 10.0.0.1: icmp_seq=1 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=2 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=3 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=4 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=5 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=6 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=7 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=8 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=9 time=0 ms
  56 bytes from 10.0.0.1: icmp_seq=10 time=0 ms
  10 packets transmitted, 10 received, 0% packet loss, time 10190 ms
  nsh>

``ping6`` differs from ``ping`` in that it uses IPv6 addressing.

