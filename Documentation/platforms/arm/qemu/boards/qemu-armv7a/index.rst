===========
qemu-armv7a
===========

This board configuration will use QEMU to emulate generic ARM v7-A series
hardware platform and provides support for these devices:

* GICv2 interrupt controllers
* ARM Generic Timer
* PL011 UART controller
* PCI ECAM
* VirtIO Device

Getting Started
===============

NSH (Single Core)
-----------------

Configuring NuttX and compile::

     $ ./tools/configure.sh -l qemu-armv7a:nsh
     $ make

Running with qemu::

     $ qemu-system-arm -cpu cortex-a7 -nographic \
     -machine virt,virtualization=off,gic-version=2 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx

KNSH (Single Core)
------------------

This is a configuration of testing the BUILD_KERNEL configuration::

  $ cd nuttx
  $ ./tools/configure.sh qemu-armv7a:knsh
  $ make V=1 -j7
  $ make export V=1
  $ cd ../apps
  $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
  $ make import V=1
  $ cd ../nuttx
  $ qemu-system-arm -semihosting -M virt -m 1024 -nographic -kernel ./nuttx

  NuttShell (NSH) NuttX-12.3.0-RC0
  nsh> uname -a
  NuttX 12.3.0-RC0 28dee592a3-dirty Oct 12 2023 03:03:07 arm qemu-armv7a
  nsh> ps
    PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK           STACK   USED  FILLED COMMAND
      0     0   0 FIFO     Kthread N-- Ready              0000000000000000 004088 000896  21.9%  Idle_Task
      1     1 100 RR       Kthread --- Waiting  Semaphore 0000000000000000 004040 000304   7.5%  lpwork 0x40119398 0x401193ac
      2     2 100 RR       Task    --- Running            0000000000000000 003032 001032  34.0%  /system/bin/init
  nsh> free
                     total       used       free    largest  nused  nfree
          Kmem:  133058556      16644  133041912  133041152     41      3
          Page:  134217728    1105920  133111808  133111808
  nsh> /system/bin/hello
  Hello, World!!
  nsh>

Inter-VM share memory Device (ivshmem)
--------------------------------------

Inter-VM shared memory support support can be found in ``drivers/pci/pci_ivshmem.c``.

This implementation is for ``ivshmem-v1`` which is compatible with QEMU and
ACRN hypervisor but won't work with Jailhouse hypervisor which uses ``ivshmem-v2``.

Please refer to the official `Qemu ivshmem documentation
<https://www.qemu.org/docs/master/system/devices/ivshmem.html>`_ for more information.

This is an example implementation for OpenAMP based on the Inter-VM share memory(ivshmem)::

  rpproxy_ivshmem:  Remote slave(client) proxy process.
  rpserver_ivshmem: Remote master(host) server process.

Steps for Using NuttX as IVSHMEM host and guest

1. Build images

  a. Build rpserver_ivshmem::

      $ cmake -B server -DBOARD_CONFIG=qemu-armv7a:rpserver_ivshmem -GNinja
      $ cmake --build server

  b. Build rpproxy_ivshmem::

      $ cmake -B proxy -DBOARD_CONFIG=qemu-armv7a:rpproxy_ivshmem -GNinja
      $ cmake --build proxy

2. Bringup firmware via Qemu:

  The Inter-VM Shared Memory device basic syntax is::

      -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
      -object memory-backend-file,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes

  a. Start rpserver_ivshmem::

      $ qemu-system-arm -cpu cortex-a7 -nographic -machine virt,highmem=off \
        -object memory-backend-file,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes \
        -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
        -kernel server/nuttx -nographic

  b. Start rpproxy_ivshmem::

      $ qemu-system-arm -cpu cortex-a7 -nographic -machine virt,highmem=off \
        -object memory-backend-file,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes \
        -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
        -kernel proxy/nuttx -nographic

  c. Check the RPMSG Syslog in rpserver shell:

    In the current configuration, the proxy syslog will be sent to the server by default.
    You can check whether there is proxy startup log in the server shell.

    RpServer bring up::

        $ qemu-system-arm -cpu cortex-a7 -nographic -machine virt,highmem=off \
          -object memory-backend-file,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes \
          -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
          -kernel server/nuttx -nographic
        [    0.000000] [ 0] [  INFO] [server] pci_register_rptun_ivshmem_driver: Register ivshmem driver, id=0, cpuname=proxy, master=0
        ...
        [    0.306127] [ 3] [  INFO] [server] rptun_ivshmem_probe: Start the wdog

    After rpproxy bring up, check the log from rpserver::

        NuttShell (NSH) NuttX-10.4.0
        server>
        [    0.000000] [ 0] [  INFO] [proxy] pci_register_rptun_ivshmem_driver: Register ivshmem driver, id=0, cpuname=server, master=1
        ...
        [    0.314039] [ 3] [  INFO] [proxy] ivshmem_probe: shmem addr=0x10400000 size=4194304 reg=0x10008000


  d. IPC test via RPMSG socket:

    Start rpmsg socket server::

        server> rpsock_server stream block test
        server: create socket SOCK_STREAM nonblock 0
        server: bind cpu , name test ...
        server: listen ...
        server: try accept ...
        server: Connection accepted -- 4
        server: try accept ...

    Switch to proxy shell and start rpmsg socket client, test start::

        proxy> rpsock_client stream block test server
        client: create socket SOCK_STREAM nonblock 0
        client: Connecting to server,test...
        client: Connected
        client send data, cnt 0, total len 64, BUFHEAD process0007, msg0000, name:test
        client recv data process0007, msg0000, name:test
        ...
        client recv done, total 4096000, endflags, send total 4096000
        client: Terminating

    Check the log on rpserver shell::

        server recv data normal exit
        server Complete ret 0, errno 0


Debugging with QEMU
===================

The nuttx ELF image can be debugged with QEMU.

1. To debug the nuttx (ELF) with symbols, make sure the following change have
   applied to defconfig::

     +CONFIG_DEBUG_SYMBOLS=y

2. Run QEMU (Single Core) at shell terminal 1::

     $ qemu-system-arm -cpu cortex-a7 -nographic \
     -machine virt,virtualization=off,gic-version=2 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx -S -s

3. Run gdb with TUI, connect to QEMU, load nuttx and continue (at shell terminal 2)::

     $ arm-none-eabi-gdb -tui --eval-command='target remote localhost:1234' nuttx
     (gdb) c
     Continuing.
     ^C
     Program received signal SIGINT, Interrupt.
     nx_start () at armv7-a/arm_head.S:209
     (gdb)

PCI support
===========

To enable PCI support, set the following options::

  CONFIG_DEVICE_TREE=y
  CONFIG_PCI=y
  CONFIG_PCI=y

Then run qemu with::

  -machine virt,highmem=off,virtualization=off,gic-version=2

The command that starts QEMU and enables the QEMU EDU device looks like this::

  qemu-system-arm -cpu cortex-a7 -nographic \
  -machine virt,highmem=off,virtualization=off,gic-version=2 \
  -chardev stdio,id=con,mux=on -serial chardev:con -mon chardev=con,mode=readline \
  -kernel nuttx -device edu
