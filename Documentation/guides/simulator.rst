.. include:: /substitutions.rst
.. _simulator:

Simulator
=========

Apache NuttX has a simulator that can run as a regular program on Linux, Mac, and Windows computers.
It's useful for debugging operating system features that aren't associated with particular
device drivers— for instance the TCP/IP stack itself, a web interface or API for your
application, or other communication protocols. It's also handy for trying out Apache NuttX without
having a piece of embedded hardware.

This guide assumes you're on Linux. It works on Windows and Mac too— if you know how,
submit a PR to improve this guide!

.. todo:: Add Mac and Windows instructions

Compiling
---------

#. Configure the Simulator

   There are a lot of simulator configurations available that set you up to test various
   operating system features.

   Here we'll use the ``sim:nsh`` basic NuttX Shell configuration.

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh sim:nsh

#. Compile

    .. code-block:: console

       $ make

#. Run the simulator:

    .. code-block:: console

       $ ./nuttx
       login: admin
       password: Administrator
       User Logged-in!

       NuttShell (NSH) NuttX-9.1.0
       MOTD: username=admin password=Administrator
       nsh> help
       help usage:  help [-v] [<cmd>]

         [         cp        exit      losetup   mv        rmdir     true
         ?         cmp       false     ls        mw        set       uname
         basename  dirname   free      mb        poweroff  sh        unset
         break     dd        help      mkdir     ps        sleep     usleep
         cat       echo      hexdump   mkfatfs   pwd       test      xd
         cd        exec      kill      mh        rm        time

       Builtin Apps:
         hello  nsh

       nsh>

#. Stop the simulator:

    .. code-block:: console

       nsh> poweroff
       $
       $ # we're back at the Linux prompt.

Accessing the Network
---------------------

#. Here we'll use the ``sim:tcpblaster`` configuration because it comes with networking
   that is ready to use.

    .. code-block:: console

       $ make distclean
       $ ./tools/configure.sh sim:tcpblaster
       $ make

#. Give the Simulator Privileges

   On recent Linux distributions, you need to give the ``nuttx`` program the capabilities
   (similar to permissions) to access the network:

    .. code-block:: console

       $ sudo setcap cap_net_admin+ep ./nuttx

#. Run the simulator:

    .. code-block:: console

       $ ./nuttx

#. Bring Up the Network Interfaces

   On Apache NuttX:

    .. code-block:: console

       nsh> ifup eth0

   On Linux, first you need to find your main network interface— this will usually either
   be an ethernet or wireless network adapter. Do this:

    .. code-block:: console

       $ ifconfig
       lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
               inet 127.0.0.1  netmask 255.0.0.0
               inet6 ::1  prefixlen 128  scopeid 0x10<host>
               loop  txqueuelen 1000  (Local Loopback)
               RX packets 5846  bytes 614351 (614.3 KB)
               RX errors 0  dropped 0  overruns 0  frame 0
               TX packets 5846  bytes 614351 (614.3 KB)
               TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

       wlp0s20f3: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
               inet 192.168.1.209  netmask 255.255.255.0  broadcast 192.168.1.255
               inet6 fe80::1161:c26b:af05:d784  prefixlen 64  scopeid 0x20<link>
               ether 24:41:8c:a8:30:d1  txqueuelen 1000  (Ethernet)
               RX packets 219369  bytes 176416490 (176.4 MB)
               RX errors 0  dropped 0  overruns 0  frame 0
               TX packets 108399  bytes 27213617 (27.2 MB)
               TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

   ``lo0`` is the Loopback Interface, so ``wlp0s20f3`` is the wireless interface. Note
   that it has an IP address on the local net. There may be other interfaces listed, you'll
   need to pick the one that's right for your system.

   Then, on Linux do this to set up the tap network interface and route that will let
   the Apache NuttX simulator access the network:

    .. code-block:: console

       $ sudo ./tools/simhostroute.sh wlp0s20f3 on
       $ ping -c 1 10.0.1.2  # nuttx system
       PING 10.0.1.2 (10.0.1.2) 56(84) bytes of data.
       64 bytes from 10.0.1.2: icmp_seq=1 ttl=64 time=7.52 ms

       --- 10.0.1.2 ping statistics ---
       1 packets transmitted, 1 received, 0% packet loss, time 0ms
       rtt min/avg/max/mdev = 7.529/7.529/7.529/0.000 m

#. Test that Apache NuttX can access the Internet

   First let's ping the network interface of our Linux host to prove we can see the
   gateway to the Internet:

    .. code-block:: console

       nsh> ping -c 1 10.0.1.1
       nsh> ping -c 1 10.0.1.1
       PING 10.0.1.1 56 bytes of data
       56 bytes from 10.0.1.1: icmp_seq=0 time=0 ms
       1 packets transmitted, 1 received, 0% packet loss, time 1010 ms

    Now let's ping one of Google's DNS servers to prove we can access the rest of the
    Internet:

    .. code-block:: console

       nsh> ping -c 1 8.8.8.8
       PING 8.8.8.8 56 bytes of data
       56 bytes from 8.8.8.8: icmp_seq=0 time=10 ms
       1 packets transmitted, 1 received, 0% packet loss, time 1010 ms

   Success!

Testing / capturing TCP network traffic
---------------------------------------

#. Start Wireshark (or tcpdump) on Linux and capture the appeared tap0 interface.

#. Optionally activate emulating packet loss on Linux:

    .. code-block:: console

       $ sudo iptables -A INPUT -p tcp --dport 31337 -m statistic --mode random --probability 0.01 -j DROP

#. Run netcat server on Linux:

    .. code-block:: console

       $ netcat -l -p 31337

#. Run netcat client on Apache NuttX:

    .. code-block:: console

       nsh> dd if=/dev/zero of=/tmp/test.bin count=1000
       nsh> netcat LINUX_HOST_IP_ADDRESS 31337 /tmp/test.bin

#. Observe TCP network traffic in Wireshark / tcpdump on Linux.

Stopping
--------

#. The normal way to stop:

    .. code-block:: console

       nsh> poweroff
       $
       $ # we're back at the Linux prompt.

   If you don't have an nsh prompt, the only effective way to stop the simulator is kill it from another terminal:

    .. code-block:: console

       $ pkill nuttx

#. Optionally deactivate emulating packet loss on Linux:

    .. code-block:: console

       $ sudo iptables -D INPUT -p tcp --dport 31337 -m statistic --mode random --probability 0.01 -j DROP

#. If you do not need tap0 interface anymore, it can be disabled on Linux as follows:

    .. code-block:: console

       $ sudo ./tools/simhostroute.sh wlan0 off

Debugging
---------

You can debug the simulator like any regular Linux program.
