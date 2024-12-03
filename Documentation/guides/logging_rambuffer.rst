=======================
Logging to a RAM Buffer
=======================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Logging+to+a+RAM+Buffer 


Default Debug Output
====================

By default, when you enable debug output, that output goes to the system 
console and is mixed up with the normal console output. Normally, that is 
sufficient. However there are some cases where you might want to do things 
differently. For example, if there is time critical debug output that 
interferes with the operation of the device. Or, if you would like to 
separate the normal console output from the debug output.

One particularly troublesome case is network debug output to console in a 
Telnet session. Since the telnet session remaps the console output to the 
Telnet connection, the network debug output can generate infinite loops 
because the network operation generates debug output to the console, which 
generates more debug output, ... and on and on.

With some creative configuration of the NuttX SYStem LOGging (SYSLOG) feature, 
these problems can all be eliminated.

The syslog Device
=================

Debug output goes to the `syslog` device. As mentioned above, the default syslog 
device device is the system console. However there are many options to control 
the behavior of the syslog – too many in fact. There are so many options that 
you will probably have to perform experiments to get the syslog working as you 
would like it too.

The RAMLOG Device
=================

The RAMLOG device is a special character device that can really be used for 
most any purpose. However, the RAMLOG device has some special attributes 
that make it ideal for use as a syslogging device.

* It supports the ``syslog_write`` interface needed for system logging
* It behaves much like a pipe: It implements a queue. Writing to the RAMLOG 
  device adds data to the head of the queue; reading from the RAMLOG device 
  removes data from the tail of the queue.
* It can be configured to return EOF when you try to read and there is 
  nothing available in the RAMLOG.


Using the RAMLOG as the syslog Device
=====================================

This Wiki page addresses the setup for one configuration: Using a `RAMLOG` as 
the syslog device. A RAMLOG is a circular buffer in memory. In this 
configuration, all debugout output goes to this circular buffer and can later 
be retrieved using the NSH ``dmesg`` command

Here is the summary of what I had to do to get the RAMLOG working as the 
syslog device. I use a simulation configuration, but for this feature this 
does not matter.

.. code-block:: bash

    tools/configure.sh sim:nsh
    make menuconfig

I added the following settings. First, these just give me some debug output 
to test against:

.. code-block:: c

    CONFIG_DEBUG=y
    CONFIG_DEBUG_FS=y
    CONFIG_DEBUG_SCHED=y

This configures the virtual file system to support the syslog device and is a 
necessary pre-condition for other settings:

.. code-block:: c

    CONFIG_SYSLOG=y

These enables the RAMLOG and configure it for use as the syslog device

.. code-block:: c

    CONFIG_RAMLOG=y
    CONFIG_RAMLOG_CONSOLE_BUFSIZE=8192
    CONFIG_RAMLOG_NONBLOCKING=y
    CONFIG_RAMLOG_SYSLOG=y
    #CONFIG_SYSLOG_CHAR undefined, else duplicate output with syslog_write()

Now when I run NuttX, I get output like this. The ``dmesg`` command now appears 
as an NSH command:

.. code-block:: bash

    NuttShell (NSH) NuttX-7.1
    nsh> help
    help usage:  help [-v] [<cmd>]
    [           dd          free        mkdir       mw          sleep      
    ?           df          help        mkfatfs     ps          test       
    break       dmesg       hexdump     mkfifo      pwd         true       
    cat         echo        kill        mkrd        rm          umount     
    cd          exec        losetup     mh          rmdir       unset      
    cp          exit        ls          mount       set         usleep     
    cmp         false       mb          mv          sh          xd     
    Builtin Apps:
    hello

The ``dmesg`` command dumps the contents and clears the RAMLOG:

.. code-block:: bash

    nsh> dmesg
    nx_start: Entry
    up_unblock_task: Unblocking TCB=52bc70
    up_unblock_task: New Active Task TCB=52bc70
    posix_spawn_exec: ERROR: exec failed: 22
    cmd_mkrd: RAMDISK at 52d4f0
    posix_spawn_exec: ERROR: exec failed: 22
    mkfatfs_tryfat16: Too few or too many clusters for FAT16: 4081 < 983 < 1022
    mkfatfs_clustersearch: Cannot format FAT16 at 1 sectors/cluster
    mkfatfs_configfatfs: Sector size:          512 bytes
    mkfatfs_configfatfs: Number of sectors:    1024 sectors
    mkfatfs_configfatfs: FAT size:             12 bits
    mkfatfs_configfatfs: Number FATs:          2
    mkfatfs_configfatfs: Sectors per cluster:  1 sectors
    mkfatfs_configfatfs: FS size:              3 sectors
    mkfatfs_configfatfs:                       985 clusters
    mkfatfs_configfatfs: Root directory slots: 512
    mkfatfs_configfatfs: Volume ID:            00000000
    mkfatfs_configfatfs: Volume Label:         "           "
    posix_spawn_exec: ERROR: exec failed: 22
    fat_mount: FAT12:
    fat_mount:      HW  sector size:     512
    fat_mount:          sectors:         1024
    fat_mount:      FAT reserved:        1
    fat_mount:          sectors:         1024
    fat_mount:          start sector:    1
    fat_mount:          root sector:     7
    fat_mount:          root entries:    512
    fat_mount:          data sector:     39
    fat_mount:          FSINFO sector:   0
    fat_mount:          Num FATs:        2
    fat_mount:          FAT sectors:     3
    fat_mount:          sectors/cluster: 1
    fat_mount:          max clusters:    985
    fat_mount:      FSI free count       -1
    fat_mount:          next free        0
    posix_spawn_exec: ERROR: exec failed: 22
    posix_spawn_exec: ERROR: exec failed: 22
    nsh> 

As mentioned, the dmesg command clears the RAMLOG. So when it is used again, 
only new debug output is shown:

.. code-block:: bash

    nsh> dmesg
    posix_spawn_exec: ERROR: exec failed: 22

As a side note, the ``posix_spawn_exec`` error will occur on each command in 
this configuration. That is because NSH first tries to execute a command from 
a file found in the file system on the ``PATH`` variable. You will not see 
this error in your system unless you have ``CONFIG_NSH_FILE_APPS=y`` 
defined in your configuration.
