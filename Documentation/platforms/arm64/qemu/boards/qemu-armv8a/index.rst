============
QEMU ARMv8-A
============

.. tags:: chip:virt, arch:arm64

This board configuration will use QEMU to emulate generic ARM64 v8-A series
hardware platform and provides support for these devices:

* GICv2 and GICv3 interrupt controllers
* ARM Generic Timer
* PL011 UART controller

Getting Started
===============

Compile Toolchain
-----------------

1. Host environment: GNU/Linux: Ubuntu 18.04 or greater

2. Download and Install

   .. code:: console

      $ wget https://developer.arm.com/-/media/Files/downloads/gnu/11.2-2022.02/binrel/gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf.tar.xz
      $ xz -d gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf.tar.xz
      $ tar xf gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf.tar

   Put ``gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf/bin/`` on your host
   ``PATH`` environment variable, like:

   .. code:: console

      $ export PATH=$PATH:/opt/software/arm/linaro-toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf/bin

   check the toolchain:

   .. code:: console

      $ aarch64-none-elf-gcc -v

Install QEMU
------------

In Ubuntu 18.04(or greater), install qemu:

.. code:: console

   $ sudo apt-get install qemu-system-arm qemu-efi-aarch64 qemu-utils

And verify your installation worked properly:

.. code:: console

   $ qemu-system-aarch64 --help

Configuring and Running
-----------------------

-------------------
Single Core (GICv3)
-------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:nsh
   $ make

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 -nographic \
    -machine virt,virtualization=on,gic-version=3 \
    -net none -chardev stdio,id=con,mux=on -serial chardev:con \
    -mon chardev=con,mode=readline -kernel ./nuttx

------------------------------------------------------------------
Single Core with virtio network, block, rng, serial driver (GICv3)
------------------------------------------------------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:netnsh
   $ make
   $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 -nographic \
     -machine virt,virtualization=on,gic-version=3 \
     -chardev stdio,id=con,mux=on -serial chardev:con \
     -global virtio-mmio.force-legacy=false \
     -device virtio-serial-device,bus=virtio-mmio-bus.0 \
     -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
     -device virtconsole,chardev=foo \
     -device virtio-rng-device,bus=virtio-mmio-bus.1 \
     -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
     -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
     -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
     -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
     -mon chardev=con,mode=readline -kernel ./nuttx

------------------------------------------
Single Core with virtio gpu driver (GICv3)
------------------------------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh qemu-armv8a:fb
   $ make -j

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 \
    -machine virt,virtualization=on,gic-version=3 \
    -chardev stdio,id=con,mux=on -serial chardev:con \
    -global virtio-mmio.force-legacy=false \
    -device virtio-gpu-device,xres=640,yres=480,bus=virtio-mmio-bus.0 \
    -mon chardev=con,mode=readline -kernel ./nuttx

   NuttShell (NSH) NuttX-10.4.0
   nsh> fb

------------------------------------
Single Core with virtio 9pFs (GICv3)
------------------------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh qemu-armv8a:netnsh
   $ make -j

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 -nographic \
     -machine virt,virtualization=on,gic-version=3 \
     -fsdev local,security_model=none,id=fsdev0,path=/mnt/xxx \
     -device virtio-9p-device,id=fs0,fsdev=fsdev0,mount_tag=host \
     -chardev stdio,id=con,mux=on, -serial chardev:con \
     -mon chardev=con,mode=readline  -kernel ./nuttx

   NuttShell (NSH) NuttX-10.4.0
   nsh> mkdir mnt
   nsh> mount -t v9fs -o trans=virtio,tag=host mnt
   nsh> ls
   /:
    dev/
    mnt/
    proc/

--------------------------------------
Single Core with MTE Expansion (GICv3)
--------------------------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh qemu-armv8a:mte
   $ make -j

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu max -nographic \
     -machine virt,virtualization=on,gic-version=3,mte=on \
     -chardev stdio,id=con,mux=on, -serial chardev:con \
     -mon chardev=con,mode=readline  -kernel ./nuttx/nuttx

   NuttShell (NSH) NuttX-10.4.0
   nsh> mtetest

-----------
SMP (GICv3)
-----------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:nsh_smp
   $ make

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
      -machine virt,virtualization=on,gic-version=3 \
      -net none -chardev stdio,id=con,mux=on -serial chardev:con \
      -mon chardev=con,mode=readline -kernel ./nuttx

------------------
SMP (GICv3) netnsh
------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:netnsh_smp
   $ make

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
     -machine virt,virtualization=on,gic-version=3 \
     -chardev stdio,id=con,mux=on -serial chardev:con \
     -global virtio-mmio.force-legacy=false \
     -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
     -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
     -mon chardev=con,mode=readline -kernel ./nuttx

-------------------
Single Core (GICv2)
-------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:nsh_gicv2
   $ make

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 -nographic \
     -machine virt,virtualization=on,gic-version=2 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx

.. note::

   1. Make sure the ``aarch64-none-elf`` toolchain install ``PATH`` has been
      added to the environment variables.

   2. To quit QEMU, type Ctrl + X.

   3. Nuttx default core number is 4, and Changing ``CONFIG_SMP_NCPUS > 4`` and
      setting QEMU command option ``-smp`` will boot more core. For QEMU,
      core limit is 32.

----------------------------------------
SMP + Networking with hypervisor (GICv2)
----------------------------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:netnsh_smp_hv
   $ make

Running with QEMU + kvm on raspi3b+ (ubuntu server 20.04)

.. code:: console

   $ qemu-system-aarch64 -nographic \
     -machine virt -cpu host -smp 4 -accel kvm \
     -chardev stdio,id=con,mux=on -serial chardev:con \
     -global virtio-mmio.force-legacy=false \
     -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd -device virtio-blk-device,drive=hd \
     -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
     -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
     -mon chardev=con,mode=readline -kernel ./nuttx

Running with QEMU + hvf on M1/MacBook Pro (macOS 12.6.1)

.. code:: console

   $ qemu-system-aarch64 -nographic \
     -machine virt -cpu host -smp 4 -accel hvf \
     -chardev stdio,id=con,mux=on -serial chardev:con \
     -global virtio-mmio.force-legacy=false \
     -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd -device virtio-blk-device,drive=hd \
     -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
     -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
     -mon chardev=con,mode=readline -kernel ./nuttx

----------------------------------
Single Core /w Xedge
----------------------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:xedge_demo
   $ make

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
      -machine virt,virtualization=on,gic-version=3 \
      -chardev stdio,id=con,mux=on -serial chardev:con \
      -netdev user,id=u1,hostfwd=tcp:127.0.0.1:8080-10.0.2.15:80,hostfwd=tcp:127.0.0.1:8443-10.0.2.15:443,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23 \
      -device virtio-net-device,netdev=u1 \
      -fsdev local,security_model=none,id=fsdev0,path=/mnt/xxx \
      -device virtio-9p-device,id=fs0,fsdev=fsdev0,mount_tag=host \
      -mon chardev=con,mode=readline -kernel ./nuttx

.. note:: Replace **/mnt/xxx** with your actual host directory path. This directory will be shared between your host system and the NuttX environment.

Before running Xedge, you need to create and mount a filesystem that Xedge will use for storing configuration files and web content::

.. code:: console

      nsh> mkdir mnt
      nsh> mount -t v9fs -o trans=virtio,tag=host mnt
      nsh> mkdir /mnt/lfs

Running Xedge in NuttX terminal

.. code:: console

   nsh> xedge_demo
      [   18.490000] [CPU0] Xedge: Server listening on IPv4 port 80
      [   18.500000] [CPU0] Xedge: SharkSSL server listening on IPv4 port 443
      [   18.510000] [CPU0] Xedge: Configuration file: /mnt/lfs/xcfg.bin: enoent
      [   38.240000] [CPU1] 10.0.2.2 GET "rtl/"
      [   38.240000] [CPU1] Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/138
      [   38.240000] [CPU1] .0.0.0 Safari/537.36
      [   38.240000] [CPU1] Host: 127.0.0.1:8080
      [   38.240000] [CPU1] Connection: keep-alive
      [   38.240000] [CPU1] sec-ch-ua: "Not)A;Brand";v="8", "Chromium";v="138", "Google Chrome";v="138"
      [   38.240000] [CPU1] sec-ch-ua-mobile: ?0
      [   38.240000] [CPU1] sec-ch-ua-platform: "Linux"
      [   38.240000] [CPU1] Upgrade-Insecure-Requests: 1
      [   38.240000] [CPU1] User-Agent: c
      [   38.240000] [CPU1] Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/138
      [   38.240000] [CPU1] .0.0.0 Safari/537.36
      [   38.240000] [CPU1] Sec-Purpose: prefetch;prerender
      [   38.240000] [CPU1] Purpose: prefetch
      [   38.240000] [CPU1] Accept:
      [   38.240000] [CPU1] text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image
      [   38.240000] [CPU1] /apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.7
      [   38.240000] [CPU1] Sec-Fetch-Site: none
      [   38.240000] [CPU1] Sec-Fetch-Mode: navigate
      [   38.240000] [CPU1] Sec-Fetch-User: ?1
      [   38.240000] [CPU1] Sec-Fetch-Dest: document
      [   38.240000] [CPU1] Accept-Encoding: gzip, deflate, br, zstd
      [   38.240000] [CPU1] Accept-Language: pt,en-US;q=0.9,en;q=0.8
      [   38.240000] [CPU1]
      [   38.240000] [CPU0] 10.0.2.2 Response:
      ,
      [   38.240000] [CPU0] no-store, no-cache, must-revalidate, max-age=0
      Transfer-Encoding: chunked
      Keep-
      [   38.240000] [CPU0] Alive: Keep-Alive

Launch your web browser and access 127.0.0.1:8080

You should see the Xedge IDE, which is enabled in developer mode:

Single Core /w kernel mode (GICv3)
----------------------------------

Configuring NuttX and compile:

.. code:: console

   $ ./tools/configure.sh -l qemu-armv8a:knsh
   $ make
   $ make export V=1
   $ pushd ../apps
   $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
   $ make import V=1
   $ popd

Running with QEMU:

.. code:: console

   $ qemu-system-aarch64 -semihosting -cpu cortex-a53 -nographic \
     -machine virt,virtualization=on,gic-version=3 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx

Inter-VM share memory Device (ivshmem)
--------------------------------------

Inter-VM shared memory support support can be found in
``drivers/pci/pci_ivshmem.c``.

This implementation is for ``ivshmem-v1`` which is compatible with QEMU and ACRN
hypervisor but won't work with Jailhouse hypervisor which uses ``ivshmem-v2``.

Please refer to the official `Qemu ivshmem documentation
<https://www.qemu.org/docs/master/system/devices/ivshmem.html>`_ for more information.

This is an example implementation for OpenAMP based on the Inter-VM share
memory(ivshmem)::

  rpproxy_ivshmem:  Remote slave(client) proxy process.
  rpserver_ivshmem: Remote master(host) server process.

Steps for Using NuttX as IVSHMEM host and guest

1. Build images

   a. Build ``rpserver_ivshmem``

      .. code:: console

         $ cmake -B server -DBOARD_CONFIG=qemu-armv8a:rpserver_ivshmem -GNinja
         $ cmake --build server

   b. Build ``rpproxy_ivshmem``

      .. code:: console

         $ cmake -B proxy -DBOARD_CONFIG=qemu-armv8a:rpproxy_ivshmem -GNinja
         $ cmake --build proxy

2. Bringup firmware via Qemu:

   The Inter-VM Shared Memory device basic syntax is::

      -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
      -object memory-backend-file,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes

   a. Start ``rpserver_ivshmem``

      .. code:: console

         $ qemu-system-aarch64 -cpu cortex-a53 -nographic -machine virt,virtualization=on,gic-version=3 -kernel server/nuttx \
           -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
           -object memory-backend-file,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes

   b. Start ``rpproxy_ivshmem``

      .. code:: console

         $ qemu-system-aarch64 -cpu cortex-a53 -nographic -machine virt,virtualization=on,gic-version=3 -kernel proxy/nuttx \
           -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
           -object memory-backend-file,discard-data=on,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes

   c. Check the RPMSG Syslog in rpserver shell:

      In the current configuration, the proxy syslog will be sent to the server by default.
      You can check whether there is proxy startup log in the server shell.

      RpServer bring up

      .. code:: console

         $ qemu-system-aarch64 -cpu cortex-a53 -nographic -machine virt,virtualization=on,gic-version=3 -kernel server/nuttx \
           -device ivshmem-plain,id=shmem0,memdev=shmmem-shmem0,addr=0xb \
           -object memory-backend-file,id=shmmem-shmem0,mem-path=/dev/shm/ivshmem0,size=4194304,share=yes
         [    0.000000] [ 0] [  INFO] [server] pci_register_rptun_ivshmem_driver: Register ivshmem driver, id=0, cpuname=proxy, master=1
         ...
         [    0.033200] [ 3] [  INFO] [server] ivshmem_probe: shmem addr=0x10400000 size=4194304 reg=0x10008000
         [    0.033700] [ 3] [  INFO] [server] rptun_ivshmem_probe: shmem addr=0x10400000 size=4194304

      After ``rpproxy`` bring up, check the log from ``rpserver``::

         NuttShell (NSH) NuttX-10.4.0
         server>
         [    0.000000] [ 0] [  INFO] [proxy] pci_register_rptun_ivshmem_driver: Register ivshmem driver, id=0, cpuname=server, master=0
         ...
         [    0.031400] [ 3] [  INFO] [proxy] ivshmem_probe: shmem addr=0x10400000 size=4194304 reg=0x10008000
         [    0.031800] [ 3] [  INFO] [proxy] rptun_ivshmem_probe: shmem addr=0x10400000 size=4194304
         [    0.033100] [ 3] [  INFO] [proxy] rptun_ivshmem_probe: Start the wdog

   d. IPC test via RPMSG socket:

      Start ``rpmsg`` socket server::

         server> rpsock_server stream block test
         server: create socket SOCK_STREAM nonblock 0
         server: bind cpu , name test ...
         server: listen ...
         server: try accept ...
         server: Connection accepted -- 4
         server: try accept ...

      Switch to proxy shell and start ``rpmsg`` socket client, test start::

         proxy> rpsock_client stream block test server
         client: create socket SOCK_STREAM nonblock 0
         client: Connecting to server,test...
         client: Connected
         client send data, cnt 0, total len 64, BUFHEAD process0007, msg0000, name:test
         client recv data process0007, msg0000, name:test
         ...
         client recv done, total 4096000, endflags, send total 4096000
         client: Terminating

      Check the log on ``rpserver`` shell::

         server recv data normal exit
         server Complete ret 0, errno 0

fastboot
--------
Fastboot TCP network device configuration with reference to qemu-armv8a:netnsh.
More details about usage of fastboot, please refer to `fastbootd — NuttX latest documentation <https://nuttx.apache.org/docs/latest/applications/system/fastboot/index.html>`_.

You can run the configuration procedure::

  $ ./tools/configure.sh -l qemu-armv8a:fastboot
  $ make
  $ dd if=/dev/zero of=./mydisk-1gb.img bs=1M count=1024

To test it, there are two example ways:

1. NAT::

    # a. Add "hostfwd=tcp:127.0.0.1:15002-10.0.2.15:5554" for option "-netdev"
    qemu-system-aarch64 -cpu cortex-a53 -nographic \
                    -machine virt,virtualization=on,gic-version=3 \
                    -chardev stdio,id=con,mux=on -serial chardev:con \
                    -global virtio-mmio.force-legacy=false \
                    -device virtio-serial-device,bus=virtio-mmio-bus.0 \
                    -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
                    -device virtconsole,chardev=foo \
                    -device virtio-rng-device,bus=virtio-mmio-bus.1 \
                    -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15002-10.0.2.15:5554 \
                    -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
                    -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
                    -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
                    -mon chardev=con,mode=readline -kernel ./nuttx

    # b. Run fastboot daemon on qemu device
    fastbootd &

    # c. Exec fastboot command on host side with device IP and PORT
    fastboot -s tcp:127.0.0.1:15002 getvar version

2. Bridge::

    # a. Set option "-netdev" to "-netdev bridge,br=br0,id=u1"
    qemu-system-aarch64 -cpu cortex-a53 -nographic \
                    -machine virt,virtualization=on,gic-version=3 \
                    -chardev stdio,id=con,mux=on -serial chardev:con \
                    -global virtio-mmio.force-legacy=false \
                    -device virtio-serial-device,bus=virtio-mmio-bus.0 \
                    -chardev socket,telnet=on,host=127.0.0.1,port=3450,server=on,wait=off,id=foo \
                    -device virtconsole,chardev=foo \
                    -device virtio-rng-device,bus=virtio-mmio-bus.1 \
                    -netdev bridge,br=br0,id=u1 \
                    -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.2 \
                    -drive file=./mydisk-1gb.img,if=none,format=raw,id=hd \
                    -device virtio-blk-device,bus=virtio-mmio-bus.3,drive=hd \
                    -mon chardev=con,mode=readline -kernel ./nuttx

    # b. Init network bridge and ACL for host
    sudo ip link add name br0 type bridge
    ip addr add 192.168.100.1/24 brd + dev br0
    ip link set br0 up
    sysctl -w net.ipv4.ip_forward=1
    iptables -t filter -A FORWARD -i br0 -j ACCEPT
    iptables -t filter -A FORWARD -o br0 -j ACCEPT
    echo "allow all" | sudo tee /etc/qemu/${USER}.conf
    echo "include /etc/qemu/${USER}.conf" | sudo tee --append /etc/qemu/bridge.conf
    sudo chown root:${USER} /etc/qemu/${USER}.conf
    sudo chmod 640 /etc/qemu/${USER}.conf

    # c. Configure IP address for qemu device, and run fastboot daemon
    ifconfig eth0 192.168.100.2
    fastbootd &

    # d. Exec fastboot command on host side with device IP
    fastboot -s tcp:192.168.100.2 getvar version

Status
======

**2022-11-18:**

1. Added support for GICv2.
2. Added board configuration for nsh_gicv2.

**2022-10-13:**

1. Renamed the board configuration name from qemu-a53 to qemu-v8a.
2. Added the configurations for Cortex-A57 and Cortex-A72.

**2022-07-01:**

 It's very strange to see that signal testing of ostest is PASSED at Physical
 Ubuntu PC rather than an Ubuntu at VMWare. For Physical Ubuntu PC, the ostest
 was run for 10 times at least but the crash was never seen again, but it's
 almost crashed every time running the ostest at Virtual Ubuntu in VMWare
 Checking for the the fail point. It's seem at signal routine to access another
 CPU's task context reg will get a NULL pointer, but watch the task context with
 GDB, shows everything as OK. So maybe this is a SMP cache synchronize issue?
 But synchronize operations have been done at thread switch. It is hard to
 explain why the crash is not happening with a Physical Ubuntu PC. So maybe this
 is a qemu issue at VMWare. ``arm64`` should be tested on a real hardware
 platform like IMX8 for the above issue.

**2022-06-12:**

SMP is support at QEMU. Add psci interface, armv8 cache operation(data cache)
and smccc support. The system can run into nsh shell, SMP test is PASSED, but
ostest crash at signal testing

**2022-05-22:**

  Arm64 support version for NuttX is Ready, These Features supported:

1. Cortex-a53 single core support: With the supporting of GICv3,
   Arch timer, PL101 UART, The system can run into nsh shell.
   Running ostest seem PASSED.

2. qemu-a53 board configuration support: qemu-a53 board can configuring
   and compiling, And running with qemu-system-aarch64
   at Ubuntu 18.04.

3. FPU support for armv8-a: FPU context switching in NEON/floating-point
   TRAP was supported.  FPU registers saving at vfork and independent
   FPU context for signal routine was considered but more testing
   needs to be do.

Platform Features
=================


The following hardware features are supported:

============== ============ ======================
Interface      Controller   Driver/Component
============== ============ ======================
GIC            on-chip      interrupt controller
PL011 UART     on-chip      serial port
ARM TIMER      on-chip      system clock
============== ============ ======================


The kernel currently does not support other hardware features on this
QEMU platform.

Debugging with QEMU
===================

The nuttx ELF image can be debugged with QEMU.

1. To debug the nuttx (ELF) with symbols, make sure the following change have
   applied to ``defconfig``:

.. code:: diff

   +CONFIG_DEBUG_SYMBOLS=y

2. Run QEMU(at shell terminal 1)

   **Single Core:**

   .. code:: console

      $ qemu-system-aarch64 -cpu cortex-a53 -nographic -machine virt,virtualization=on,gic-version=3 \
        -net none -chardev stdio,id=con,mux=on -serial chardev:con -mon chardev=con,mode=readline \
        -kernel ./nuttx -S -s

   **SMP**

   .. code:: console

      $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic -machine virt,virtualization=on,gic-version=3 \
        -net none -chardev stdio,id=con,mux=on -serial chardev:con -mon chardev=con,mode=readline \
        -kernel ./nuttx -S -s

3. Run ``gdb`` with TUI, connect to QEMU, load nuttx and continue (at shell
   terminal 2):

   .. code:: console
   
      $ aarch64-none-elf-gdb -tui --eval-command='target remote localhost:1234' nuttx
      (gdb) set debug aarch64
      (gdb) c
      Continuing.
      ^C
      Program received signal SIGINT, Interrupt.
      arch_cpu_idle () at common/arm64_cpu_idle.S:37
      (gdb)
      (gdb) where
      #0  arch_cpu_idle () at common/arm64_cpu_idle.S:37
      #1  0x00000000402823ec in nx_start () at init/nx_start.c:742
      #2  0x0000000040280148 in arm64_boot_primary_c_routine () at common/arm64_boot.c:184
      #3  0x00000000402a5bf8 in switch_el () at common/arm64_head.S:201
      (gdb)
      SMP Case
      Thread 1 received signal SIGINT, Interrupt.
       arch_cpu_idle () at common/arm64_cpu_idle.S:37
      (gdb) info threads
       Id   Target Id                  Frame
     * 1    Thread 1 (CPU#0 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
       2    Thread 2 (CPU#1 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
       3    Thread 3 (CPU#2 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
       4    Thread 4 (CPU#3 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
      (gdb)

   .. note::

      It will make your debugging more easier in source level if you setting
      ``CONFIG_DEBUG_FULLOPT=n``. but there is a risk of stack overflow when the
      option is disabled. Just enlarging your stack size will avoid the issue
      (eg. enlarging ``CONFIG_DEFAULT_TASK_STACKSIZE``)

   .. todo::

      ARMv8-A Supporting for ``tools/nuttx-gdbinit``


FPU Support and Performance
===========================

The FPU trap was used to handle FPU context switch. For threads accessing the
FPU (FPU instructions or registers), a trap will happen at this thread, the
FPU context will be saved/restore for the thread at the trap handler. It will
improve performance for thread switch since it's not to save/restore the FPU
context (almost 512 bytes) at the thread switch anymore. But some issue need
to be considered:

1. Floating point argument passing issue:

   In many cases, the FPU trap is triggered by ``va_start()`` that copies the
   content of FP registers used for floating point argument passing into the
   va_list object in case there were actual float arguments from the caller.
   Adding ``-mgeneral-regs-only`` option will make compiler not use the FPU
   register, we can use the following patch to ``syslog``:

   .. code:: diff

      diff --git a/libs/libc/syslog/Make.defs b/libs/libc/syslog/Make.defs
      index c58fb45512..acac6febaa
      --- a/libs/libc/syslog/Make.defs
      +++ b/libs/libc/syslog/Make.defs
      @@ -26,3 +26,4 @@ CSRCS += lib_syslog.c lib_setlogmask.c

       DEPPATH += --dep-path syslog
       VPATH += :syslog
      +syslog/lib_syslog.c_CFLAGS += -mgeneral-regs-only

   This patch cannot be merged into NuttX mainline because it is a very special
   case since ostest is using syslog for lots of information printing. but this
   is a clue for FPU performance analysis. ``va_list`` object is using for many
   C code to handle argument passing, but if it's not passing floating point
   argument indeed. Add the option to your code maybe increase FPU performance

2. memset/memcpy issue:

   For improved performance, the memset/memcpy implement for libc will use the
   neon/fpu instruction/register. The FPU trap is also triggered in this case.

   We can trace this issue with Procfs:

   .. code:: console

      nsh> cat /proc/arm64fpu
      CPU0: save: 7 restore: 8 switch: 62 exedepth: 0
      nsh>

   after ostest:

   .. code:: console

      nsh> cat /proc/arm64fpu
      CPU0: save: 1329 restore: 2262 switch: 4613 exedepth: 0
      nsh>

   .. note::

      * ``save``: the counts of save for task FPU context
      * ``restore``: the counts of restore for task FPU context
      * ``switch``:  the counts of task switch

2. FPU trap at IRQ handler:

   It's probably need to handle FPU trap at IRQ routine. Exception_depth is
   handling for this case, it will inc/dec at enter/leave exception. If the
   exception_depth > 1, that means an exception occurring when another exception
   is executing, the present implement is to switch FPU context to idle thread,
   it will handle most case for calling printf-like routine at IRQ routine. But
   in fact, this case will make uncertainty interrupt processing time sine it's
   uncertainty for trap exception handling. It would be best to add
   ``-mgeneral-regs-only`` option to compile the IRQ code avoiding accessing FP
   register. if it's necessarily for the exception routine to use FPU, calling
   function to save/restore FPU context directly maybe become a solution. Linux
   kernel introduce kernel_neon_begin/kernel_neon_end function for this case.
   Similar function will be add to NuttX if this issue need to be handle.

3. More reading:

   For Linux kernel, please reference
   https://www.kernel.org/doc/html/latest/arm/kernel_mode_neon.html

SMP Support
===========

1. Booting

   Primary core call sequence::

     arm64_start
       ->arm64_boot_primary_c_routine
         ->arm64_chip_boot
           ->set init TBBR and Enable MMU
         ->nx_start
           ->OS component initialize
             ->Initialize GIC: GICD and Primary core GICR
           ->nx_smp_start
             for every CPU core
             ->up_cpu_start
               ->arm64_start_cpu(call PCSI to boot CPU)
               ->waiting for every core to boot
           ->nx_bringup

   Secondary Core call sequence::

     arm64_start
       ->arm64_boot_secondary_c_routine
         ->Enable MMU
         ->Initialize GIC: Secondary core GICR
         ->Notify Primary core booting is Ready
         ->nx_idle_trampoline

2. Interrupt:

   **SGI**
      ``SGI_CPU_PAUSE``: for core pause request, for every core
   
   **PPI**
      ``ARM_ARCH_TIMER_IRQ``: timer interrupt, handle by primary Core
   
   **SPI**
     ``CONFIG_QEMU_UART_IRQ``: serial driver interrupt, handle by primary Core

3. Timer:

   The origin design for ARMv8-A timer is assigned private timer to every PE(CPU
   core), the ``ARM_ARCH_TIMER_IRQ`` is a PPI so it's should be enabled at every
   core.

   But for NuttX, it's design only for primary core to handle timer interrupt
   and call ``nxsched_process_timer`` at timer tick mode. So we need only enable
   timer for primary core

   IMX6 use GPT which is a SPI rather than generic timer to handle timer
   interrupt

References
==========

1. (ID050815) ARM® Cortex®-A Series - Programmer’s Guide for ARMv8-A
2. (ID020222) Arm® Architecture Reference Manual - for A profile architecture
3. (ARM062-948681440-3280) Armv8-A Instruction Set Architecture
4. AArch64 Exception and Interrupt Handling
5. AArch64 Programmer's Guides Generic Timer
6. Arm Generic Interrupt Controller v3 and v4 Overview
7. Arm® Generic Interrupt Controller Architecture Specification GIC architecture version 3 and version 4
8. (DEN0022D.b) Arm Power State Coordination Interface Platform Design Document
