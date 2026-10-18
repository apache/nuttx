=======================================
``wifibridge`` Wi-Fi to Ethernet bridge
=======================================

Overview
========

The ``wifibridge`` command turns a Wi-Fi interface into an access point
(SoftAP) and bridges it to an Ethernet interface at layer 2.  Ethernet
devices such as printers, PLCs or PCs then reach the wireless network
without any routing, and wireless stations reach them.  The bridge
interface carries the IP address of the board and, optionally, a DHCP
server.

The command is a thin front end for the ``netutils/wifibridge`` library,
which creates the bridge with the ``netlib`` bridge helpers (see
:doc:`/os/networking/bridge`), configures the access point with the
``wapi`` wireless API and, optionally, runs the DHCP server.  The library can
be used by an application directly (``include/netutils/wifibridge.h``).

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_WIFIBRIDGE``.  It selects
``CONFIG_NETUTILS_WIFIBRIDGE`` and needs:

* ``CONFIG_NET_BRIDGE``, ``CONFIG_NET_IPv4`` and ``CONFIG_WIRELESS_WAPI``.
* An Ethernet driver that receives frames for foreign MAC addresses
  (``CONFIG_NET_PROMISCUOUS``, if the driver supports it).
* A Wi-Fi driver that supports the master (access point) mode.

The defaults used when an option is not given on the command line are:

``CONFIG_SYSTEM_WIFIBRIDGE_BRIDGE`` / ``_ETHIF`` / ``_WLANIF``
  Names of the bridge, Ethernet and Wi-Fi interfaces (``br0``, ``eth0``
  and ``wlan0``).
``CONFIG_SYSTEM_WIFIBRIDGE_SSID`` / ``_PASSPHRASE`` / ``_CHANNEL``
  SoftAP SSID, passphrase (8 to 63 characters) and 2.4 GHz channel.
``CONFIG_SYSTEM_WIFIBRIDGE_IPADDR``
  IPv4 address of the bridge (the netmask is 255.255.255.0).
``CONFIG_NETUTILS_WIFIBRIDGE_DHCPD``
  Allow the ``-d`` option, which runs the DHCP server on the bridge
  (``CONFIG_NETUTILS_DHCPD``).

Change the default passphrase before deploying a board.

Usage
=====

.. code-block:: console

   wifibridge start [-s <ssid>] [-p <passphrase>] [-c <channel>] [-3]
                    [-a <ipaddr>] [-d]
   wifibridge stop
   wifibridge status

``start``
  Configure the Wi-Fi interface as an access point, create the bridge,
  add the Ethernet and Wi-Fi interfaces as its ports and give the bridge
  its address.  If a step fails, what was done so far is undone.

  ``-s``
    SoftAP SSID.
  ``-p``
    SoftAP passphrase.
  ``-c``
    2.4 GHz channel, 1 to 13.
  ``-3``
    Use WPA3-Personal (SAE) instead of WPA2-PSK.  The driver must support
    it.
  ``-a``
    IPv4 address of the bridge.
  ``-d``
    Run the DHCP server on the bridge.  Leave it out when another DHCP
    server is reachable through the Ethernet port.
``stop``
  Stop the DHCP server, remove the ports, delete the bridge and stop the
  access point.
``status``
  Show whether the bridge exists, its address, the state of the two
  ports and the SSID of the access point.

The Ethernet and Wi-Fi interfaces carry no IP address of their own: only
the bridge does.  The Ethernet interface needs a MAC address, which a
board without a factory programmed one has to provide.

Example
=======

.. code-block:: console

   nsh> wifibridge start -s MyBridge -p mypassphrase -d
   nsh> wifibridge status
   br0   present 10.0.0.2
   eth0  up
   wlan0 up ssid "MyBridge"
   nsh> wifibridge stop

Limitations
===========

* Only the access point mode is supported.  A station cannot be bridged
  at layer 2 because it cannot send frames with foreign source addresses.
* ``start`` fails with ``EEXIST`` if the bridge already exists.
* The SSID and the passphrase are not saved: they come from the
  command line or from the defaults of the configuration.
* The rest of the limits of the bridge apply, see
  :doc:`/os/networking/bridge`.
