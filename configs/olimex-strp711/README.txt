For a debug environment, I am using OpenOCD with a Wiggler-clone JTAG interface.  The
following steps worked for me with a 20081028 OpenOCD snapshot.

GENERAL STEPS:

1. Check out OpenOCD

    svn checkout svn://svn.berlios.de/openocd/trunk openocd

2. Build OpenOCD

  Read the INSTALL file from the files you just downloaded.  You probably just need
  to run:

    ./bootstrap

  Then configure OpenOCD using the configure script created by ./bootstrap.

    ./configure --enable-parport

  Build OpenOCD with:

    make

  Install OpenOCD.  Since we used the default configuration the code will be
  installed at /usr/local/bin/openocd. Other files will be installed at
  /usr/local/lib/openocd (configuration files, scripts, etc.) and /usr/local/share/info
  (online documentation accessable via 'info openocd').  You need root priviledges
  to do the following:

    make install.

3. Setup

  OpenOCD reads its configuration from the file openocd.cfg in the current directory
  when started.  You have two different options:

  * Create a symbolic link named openocd.cfg to one of the configuration files in
    /usr/local/lib/openocd, or

  * Use a custom configuration file specified with the ‘-f <conf.file>’ command line
    switch opeion when starting OpenOCD.

  For the STR-P711, I have included bash scripts in the scripts sub-directory.

4. Running OpenOCD

  Make sure the ARM7TDMI board is powered and the JTAG cable is connected

  Run 'src/openocd -d' (might be required to be root) and check for any errors
  reported. The '-d' option enables debugging info.

5. Telnet interface

  telnet into port 4444 to get a command interface: 'telnet localhost 4444'

6. GDB

  start arm-elf-gdb
  type 'file <executable.elf>' to load the executable
  type 'set debug remote 1' to enable tracing of gdb protocol (if required)
  type 'target remote localhost:3333' to connect to the target
  The same commands from the telnet interface can now be accessed through the
  'monitor' command, e.g. 'monitor help' 
