======================
``netcat`` NetCat tool
======================

netcat TCP/IP Swiss army knife.

It was re-implemented from scratch for NuttX

DEMO
----

.. image:: https://files.mastodon.social/media_attachments/files/105/163/916/892/863/178/original/05468e28b4463f95.png

* https://mastodon.social/@rzr/105225153152922220#weboftwins-osvehicle-2020-rzr
* https://purl.org/rzr/weboftwins

USAGE
-----

Usage is straightforward::

    nsh> help ; netcat
    Usage: netcat [-l] [destination] [port] [file]

    nsh> renew eth0 ; ifconfig

    eth0    Link encap:Ethernet HWaddr 52:13:FF:FF:FF:FF at UP
            inet addr:192.168.1.42 DRaddr:192.168.1.254 Mask:255.255.255.0

In the following examples, following configuration is used:

- target (nuttx) is 192.168.1.42
- host (linux) is 192.168.1.55

Server
~~~~~~

As a server on NuttX and Linux's netcat as client::

    nsh> netcat -l

    sh> cat /proc/version | netcat 192.168.1.42 31337
    Linux ...

Default port is 31337 but it can changed::

    nsh> renew eth0 ; ifconfig ; netcat -l
    log: net: listening on :31337
    Linux ...

Client
~~~~~~

Start Server on GNU/Linux::

    sh> ip addr show && netcat -l 31337

Client side on nuttx, we create::

    nsh> help ; renew eth0 ; ifconfig
    nsh> netcat 192.168.1.55 31337 /proc/version

Using pipes
~~~~~~~~~~~

.. code-block:: bash

   mkfifo /dev/fifo
   netcat 192.168.1.55 31337 /proc/fifo
   help > /dev/fifo

   fxos8700cq > /dev/fifo &
   fxos8700cq [7:100]
   netcat 192.168.1.55 31337  /dev/fifo

Resources
~~~~~~~~~

* <https://en.wikipedia.org/wiki/Netcat>
* <https://purl.org/rzr/weboftwins>
* <https://github.com/rzr/aframe-smart-home/issues/3>
