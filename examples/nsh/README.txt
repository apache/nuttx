examples/nsh
^^^^^^^^^^^^

  This directory contains the NuttShell (NSH).  This is a simple
  shell application for NuttX.

  - Console/NSH Front End
  - Command Overview
  - Conditional Command Execution
  - Built-In Variables
  - Current Working Directory
    Environment Variables:
  - Simple Commands
  - NSH Configuration Settings
    Command Dependencies on Configuration Settings
    NSH-Specific Configuration Settings

Console/NSH Front End
^^^^^^^^^^^^^^^^^^^^^

  Using settings in the configuration file, NSH may be configured to
  use either the serial stdin/out or a telnet connection as the console
  or BOTH.  When NSH is started, you will see the following welcome on
  either console:

    NuttShell (NSH)
    nsh>

  'nsh>' is the NSH prompt and indicates that you may enter a command
   from the console.

Command Overview
^^^^^^^^^^^^^^^^

  This directory contains the NuttShell (NSH).  This is a simple
  shell-like application.  At present, NSH supports the following commands
  forms:

    Simple command:                  <cmd>
    Command with re-directed output: <cmd> > <file>
                                     <cmd> >> <file>
    Background command:              <cmd> &
    Re-directed background command:  <cmd> > <file> &
                                     <cmd> >> <file> &

  Where:

    <cmd>  is any one of the simple commands listed later.
    <file> is the full or relative path to any writable object
           in the filesystem name space (file or character driver).
           Such objects will be referred to simply as files throughout
           this README.

  NSH executes at the mid-priority (128).  Backgrounded commands can
  be made to execute at higher or lower priorities using nice:

    [nice [-d <niceness>>]] <cmd> [> <file>|>> <file>] [&]

  Where <niceness> is any value between -20 and 19 where lower
  (more negative values) correspond to higher priorities.  The
  default niceness is 10.

Conditional Command Execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  An if-then[-else]-fi construct is also supported in order to
  support conditional execution of commands.  This works from the
  command line but is primarily intended for use within NSH scripts
  (see the sh commnd).  The syntax is as follows:

    if <cmd>
    then
      [sequence of <cmd>]
    else
      [sequence of <cmd>]
    fi

Built-In Variables
^^^^^^^^^^^^^^^^^^

  $? - The result of the last simple command execution

Current Working Directory
^^^^^^^^^^^^^^^^^^^^^^^^^

  All path arguments to commands may be either an absolute path or a
  path relative to the current working directory.  The current working
  directory is set using the 'cd' command and can be queried either
  by using the 'pwd' command or by using the 'echo $PWD' command.

  Environment Variables:
  ----------------------

    PWD    - The current working directory
    OLDPWD - The previous working directory


Simple Commands
^^^^^^^^^^^^^^^

o [ <expression> ]
o test <expression>

   These are two alternative forms of the same command.  They support
   evaluation of a boolean expression which sets $?.  This command
   is used most frequently as the conditional command following the
   'if' in the if-then[-else]-fi construct.

   Expression Syntax:
   ------------------

     expression = simple-expression | !expression |
                  expression -o expression | expression -a expression

     simple-expression = unary-expression | binary-expression

     unary-expression = string-unary | file-unary

     string-unary = -n string | -z string

     file-unary = -b file | -c file | -d file | -e file | -f file |
                  -r file | -s file | -w file

     binary-expression = string-binary | numeric-binary

     string-binary = string = string | string == string | string != string

     numeric-binary = integer -eq integer | integer -ge integer |
                      integer -gt integer | integer -le integer |
                      integer -lt integer | integer -ne integer

o cat <path> [<path> [<path> ...]]

  This command copies and concatentates all of the files at <path>
  to the console (or to another file if the output is redirected).

o cd [<dir-path>|-|~|..]

  Changes the current working directory (PWD).  Also sets the
  previous working directory environment variable (OLDPWD).

  FORMS:
  ------

    'cd <dir-path>' sets the current working directory to <dir-path>.
    'cd -' sets the current working directory to the previous
       working directory ($OLDPWD).  Equivalent to 'cd $OLDPWD'.
    'cd' or 'cd ~' set the current working directory to the 'home'
       directory.  The 'home' directory can be configured by setting
       CONFIG_LIB_HOMEDIR in the configuration file.  The default
       'home' directory is '/'.
    'cd ..' sets the current working directory to the parent directory.

o cp <source-path> <dest-path>

  Copy of the contents of the file at <source-path> to the location
  in the filesystem indicated by <path-path>

o echo [<string|$name> [<string|$name>...]]

  Copy the sequence of strings and expanded environment variables to
  console out (or to a file if the output is re-directed).

o exec <hex-address>

  Execute the user logic at address <hex-address>.  NSH will pause
  until the execution unless the user logic is executed in background
  via 'exec <hex-address> &'

o exit

  Exit NSH.  Only useful if you have started some other tasks (perhaps
  using the 'exec' command') and you would like to have NSH out of the
  way.

o get [-b|-n] [-f <local-path>] -h <ip-address> <remote-path>

  Copy the file at <remote-address> from the host whose IP address is
  identified by <ip-address>.  Other options:

  -f <local-path>
     The file will be saved relative to the current working directory
      unless <local-path> is provided.
  -b|-n
      Selects either binary ("octect") or test ("netascii") transfer
      mode.  Default: text.

o help

  Presents summary information about each command to console.

o ifconfig

  Show the current configuration of the network, for example:

    nsh> ifconfig
    eth0    HWaddr 00:18:11:80:10:06
            IPaddr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

  if uIP statistics are enabled (CONFIG_NET_STATISTICS), then
  this command will also show the detailed state of uIP.

o ls [-lRs] <dir-path>

  Show the contents of the directory at <dir-path>.  NOTE:
  <dir-path> must refer to a directory and no other filesystem
  object.

  Options:
  --------

     -R Show the constents of specified directory and all of its
        sub-directories.
     -s Show the size of the files along with the filenames in the
        listing
     -l Show size and mode information along with the filenames
        in the listing.

o mb <hex-address>[=<hex-value>][ <hex-byte-count>]
o mh <hex-address>[=<hex-value>][ <hex-byte-count>]
o mw <hex-address>[=<hex-value>][ <hex-byte-count>]

  Access memory using byte size access (mb), 16-bit accesses (mh),
  or 32-bit access (mw).  In each case,

    <hex-address>. Specifies the address to be accessed.  The current
      value at that address will always be read and displayed.
    <hex-address>=<hex-value>.  Read the value, then write <hex-value>
      to the location.
    <hex-byte-count>.  Perform the mb, mh, or mw operation on a total
      of <hex-byte-count> bytes, increment the <hex-address> appropriately
      after each access

  Example

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

o mem

  Show the current state of the memory allocator.  For example,

    nsh> mem
      arena:      fe2560
      ordblks:         1
      mxordblk:   fdc3e0
      uordblks:     6180
      fordblks:   fdc3e0
    nsh>

  Where:
    arena - This is the total size of memory allocated for use
      by malloc in bytes.
    ordblks - This is the number of free (not in use) chunks.
    mxordblk - Size of the largest free (not in use) chunk
    uordblks - This is the total size of memory occupied by
      chunks handed out by malloc.
    fordblks - This is the total size of memory occupied by
      free (not in use) chunks.

o mkdir <path>

  Create the directory at <path>.  All components of of <path>
  except the final directory name must exist on a mounted file
  system; the final directory must not.

  Recall that NuttX uses a pseudo filesystem for its root file system.
  The mkdir command can only be used to create directories in volumes
  set up with the mount command; it cannot be used to create directories
  in the pseudo filesystem.

  Example:
  ^^^^^^^^

    nsh> mkdir /mnt/fs/tmp
    nsh> ls -l /mnt/fs
    /mnt/fs:
     drw-rw-rw-       0 TESTDIR/
     drw-rw-rw-       0 TMP/
    nsh>

o mkfatfs <path>

  Format a fat file system on the block device specified by path.
  NSH provides this command to access the mkfatfs() NuttX API.
  This block device must reside in the NuttX psuedo filesystem and
  must have been created by some call to register_blockdriver() (see
  include/nuttx/fs.h).

o mkfifo <path>

  Creates a FIFO character device anywhere in the pseudo file system,
  creating whatever psuedo directories that may be needed to complete
  the full path.  By convention, however, device drivers are place in
  the standard /dev directory. After it is created, the FIFO device
  may be used as any other device driver. NSH provides this command
  to access the mkfifo() NuttX API.

  Example:
  ^^^^^^^^

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

o mkrd [-m <minor>] [-s <sector-size>] <nsectors>

  Create a ramdisk consisting of <nsectors>, each of size
  <sector-size> (or 512 bytes if <sector-size> is not specified.
  The ramdisk will be registered as /dev/ram<n> (if <n> is not
  specified, mkrd will attempt to register the ramdisk as
  /dev/ram0.

  Example:
  ^^^^^^^^

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

  Once the ramdisk has been created, it may be formatted using
  the mkfatfs command and mounted using the mount command.

  Example:
  ^^^^^^^^
    nsh> mkrd 1024
    nsh> mkfatfs /dev/ram0
    nsh> mount -t vfat /dev/ram0 /tmp
    nsh> ls /tmp
    /tmp:
    nsh>

o mount -t <fstype> <block-device> <dir-path>

  The 'mount' command mounts a file system in the NuttX psuedo
  filesystem.  'mount' performs a three way associating, binding

    File system.  The '-t <fstype>' option identifies the type of
      file system that has been formatted on the <block-device>.  As
      of this writing, vfat is the only supported value for <fstype>

    Block Device.  The <block-device> argument is the full or relative
      path to a block driver inode in the psuedo filesystem.  By convention,
      this is a name under the /dev sub-directory.  This <block-device>
      must have been previously formatted with the same file system
      type as specified by <fstype>

    Mount Point.  The mount point is the location in the psuedo file
      system where the mounted volume will appear.  This mount point
      can only reside in the NuttX psuedo filesystem.  By convention, this
      mount point is a subdirectory under /mnt.  The mount command will
      create whatever psuedo directories that may be needed to complete
      the full path but the full path must not already exist.

  After the the volume has been mounted in the NuttX psuedo file
  system, it may be access in the same way as other objects in the
  file system.

  Example:
  ^^^^^^^^

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

o ps

  Show the currently active threads and tasks.  For example,

    nsh> ps
    PID   PRI SCHD TYPE   NP STATE    NAME
        0   0 FIFO TASK      READY    Idle Task()
        1 128 RR   TASK      RUNNING  init()
        2 128 FIFO TASK      WAITSEM  nsh_telnetmain()
        3 100 RR   PTHREAD   WAITSEM  <pthread>(21)
    nsh>

o ping [-c <count>] [-i <interval>] <ip-address>

  Test the network communication with a remote peer.  Example,

    nsh> 10.0.0.1
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

o put [-b|-n] [-f <remote-path>] -h <ip-address> <local-path>

  Copy the file at <local-address> to the host whose IP address is
  identified by <ip-address>.  Other options:

  -f <remote-path>
     The file will be saved with the same name on the host unless
      unless <local-path> is provided.
  -b|-n
      Selects either binary ("octect") or test ("netascii") transfer
      mode.  Default: text.

o pwd

  Show the current working directory.

    nsh> cd /dev
    nsh> pwd
    /dev
    nsh>

  Same as 'echo $PWD'

    nsh> echo $PWD
    /dev
    nsh>

o rm <file-path>

  Remove the specified <file-path> name from the mounted file system.
  Recall that NuttX uses a pseudo filesystem for its root file system.
  The rm command can only be used to remove (unlink) files in volumes
  set up with the mount command; it cannot be used to remove names from
  the pseudo filesystem.

  Example:
  ^^^^^^^^

    nsh> ls /mnt/fs/testdir
    /mnt/fs/testdir:
     TESTFILE.TXT
     EXAMPLE.TXT
    nsh> rm /mnt/fs/testdir/example.txt
    nsh> ls /mnt/fs/testdir
    /mnt/fs/testdir:
     TESTFILE.TXT
    nsh>

o rmdir <dir-path>

  Remove the specified <dir-path> directory from the mounted file system.
  Recall that NuttX uses a pseudo filesystem for its root file system. The
  rmdir command can only be used to remove directories from volumes set up
  with the mount command; it cannot be used to remove directories from the
  pseudo filesystem. 

  Example:
  ^^^^^^^^

    nsh> mkdir /mnt/fs/tmp
    nsh> ls -l /mnt/fs
    /mnt/fs:
     drw-rw-rw-       0 TESTDIR/
     drw-rw-rw-       0 TMP/
    nsh> rmdir /mnt/fs/tmp
    nsh> ls -l /mnt/fs
    ls -l /mnt/fs
    /mnt/fs:
     drw-rw-rw-       0 TESTDIR/
    nsh>

o set <name> <value>

  Set the environment variable <name> to the sting <value>.
  For example,

    nsh> echo $foobar

    nsh> set foobar foovalue
    nsh> echo $foobar
    foovalue
    nsh>

o sh <script-path>

  Execute the sequence of NSH commands in the file referred
  to by <script-path>.

o sleep <sec>

  Pause execution (sleep) of <sec> seconds.

o unset <name>

  Remove the value associated with the environment variable
  <name>.  Example:

    nsh> echo $foobar
    foovalue
    nsh> unset foobar
    nsh> echo $foobar

    nsh>

o usleep <usec>

  Pause execution (sleep) of <usec> microseconds.

o xd <hex-address> <byte-count>

  Dump <byte-count> bytes of data from address <hex-address>

  Example:
  ^^^^^^^^

    nsh> xd 410e0 512
    Hex dump:
    0000: 00 00 00 00 9c 9d 03 00 00 00 00 01 11 01 10 06 ................
    0010: 12 01 11 01 25 08 13 0b 03 08 1b 08 00 00 02 24 ....%..........$
    ...
    01f0: 08 3a 0b 3b 0b 49 13 00 00 04 13 01 01 13 03 08 .:.;.I..........
    nsh>

NSH Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^

The availability of the above commands depends upon features that
may or may not be enabled in the NuttX configuration file.  The 
following table indicates the dependency of each command on NuttX
configuration settings.  General configuration settings are discussed
in the NuttX Porting Guide.  Configuration settings specific to NSH
as discussed at the bottom of this README file.

Command Dependencies on Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Command    Depends on Configuration
  ---------- --------------------------
  [          !CONFIG_EXAMPLES_NSH_DISABLESCRIPT
  cat        CONFIG_NFILE_DESCRIPTORS > 0
  cd         !CONFIG_DISABLE_ENVIRON && CONFIG_NFILE_DESCRIPTORS > 0
  cp         CONFIG_NFILE_DESCRIPTORS > 0
  echo       --
  exec       --
  exit       --
  get        CONFIG_NET && CONFIG_NET_UDP && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NET_BUFSIZE >= 558*
  help       --
  ifconfig   CONFIG_NET
  ls         CONFIG_NFILE_DESCRIPTORS > 0
  mb,mh,mw   ---
  mem        ---
  mkdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  mkfatfs    !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  mkfifo     CONFIG_NFILE_DESCRIPTORS > 0
  mkrd       !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  mount      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  ping       CONFIG_NET && CONFIG_NET_ICMP && CONFIG_NET_ICMP_PING  && !CONFIG_DISABLE_CLOCK && !CONFIG_DISABLE_SIGNALS
  ps         --
  put        CONFIG_NET && CONFIG_NET_UDP && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NET_BUFSIZE >= 558*
  pwd        !CONFIG_DISABLE_ENVIRON && CONFIG_NFILE_DESCRIPTORS > 0
  rm         !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  rmdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  set        !CONFIG_DISABLE_ENVIRON
  sh         CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0 && !CONFIG_EXAMPLES_NSH_DISABLESCRIPT
  sleep      !CONFIG_DISABLE_SIGNALS
  test       !CONFIG_EXAMPLES_NSH_DISABLESCRIPT
  umount     !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  unset      !CONFIG_DISABLE_ENVIRON
  usleep     !CONFIG_DISABLE_SIGNALS
  xd         ---

* NOTES:
  - Because of hardware padding, the actual required size may be larger.
  - Special TFTP server start-up optionss will probably be required to permit
    creation of file for the correct operation of the put command.

NSH-Specific Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  The behavior of NSH can be modified with the following settings in
  the configs/<board-name>/defconfig file:

  * CONFIG_EXAMPLES_NSH_FILEIOSIZE
      Size of a static I/O buffer used for file access (ignored if
      there is no filesystem).

  * CONFIG_EXAMPLES_NSH_STRERROR
      strerror(errno) makes more readable output but strerror() is
      very large and will not be used unless this setting is 'y'

  * CONFIG_EXAMPLES_NSH_LINELEN
      The maximum length of one command line and of one output line.
      Default: 80

  * CONFIG_EXAMPLES_NSH_STACKSIZE
      The stack size to use when spawning new threads or tasks.  Such
      new threads are generated when a command is executed in background
      or as new TELNET connections are established.

  * CONFIG_EXAMPLES_NSH_NESTDEPTH
      The maximum number of nested if-then[-else]-fi sequences that
      are permissable.  Default: 3

  * CONFIG_EXAMPLES_NSH_DISABLESCRIPT
      This can be set to 'y' to suppress support for scripting.  This
      setting disables the 'sh', 'test', and '[' commands and the
      if-then[-else]-fi construct.  This would only be set on systems
      where a minimal footprint is a necessity and scripting is not.

  * CONFIG_EXAMPLES_NSH_DISABLEBG
      This can be set to 'y' to suppress support for background
      commands.  This setting disables the 'nice' command prefix and
      the '&' command suffix.  This would only be set on systems
      where a minimal footprint is a necessity and background command
      execution is not.

  * CONFIG_EXAMPLES_NSH_CONSOLE
      If CONFIG_EXAMPLES_NSH_CONSOLE is set to 'y', then a serial
      console front-end is selected.

  * CONFIG_EXAMPLES_NSH_TELNET
      If CONFIG_EXAMPLES_NSH_TELNET is set to 'y', then a TELENET
      server front-end is selected.  When this option is provided,
      you may log into NuttX remotely using telnet in order to
      access NSH.

  One or both of CONFIG_EXAMPLES_NSH_CONSOLE and CONFIG_EXAMPLES_NSH_TELNET
  must be defined.  If CONFIG_EXAMPLES_NSH_TELNET is selected, then there some
  other configuration settings that apply:

  * CONFIG_EXAMPLES_NSH_IOBUFFER_SIZE
      Determines the size of the I/O buffer to use for sending/
      receiving TELNET commands/reponses

  * CONFIG_EXAMPLES_NSH_DHCPC
      Obtain the the IP address via DHCP.

  * CONFIG_EXAMPLES_NSH_IPADDR
      If CONFIG_EXAMPLES_NSH_DHCPC is NOT set, then the static IP
      address must be provided.

  * CONFIG_EXAMPLES_NSH_DRIPADDR
      Default router IP address

  * CONFIG_EXAMPLES_NSH_NETMASK
      Network mask

  * CONFIG_EXAMPLES_NSH_NOMAC
      Set if your ethernet hardware has no built-in MAC address.
      If set, a bogus MAC will be assigned.

