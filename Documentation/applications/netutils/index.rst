=================
Network Utilities
=================

.. toctree::
   :glob:
   :maxdepth: 3
   :titlesonly:
   
   */*


- chat - Chat tool
- codecs - CODEC Library: Base64 coding, URL coding, MD5
- cwebsocket - WebSocket Client Library
- dhcp6c - DHCPv6 client
- iptables - iptables libary
- libcurl4nx - cURL4nx HTTP client library
- mqttc - MQTT-C library
- netinit - Network initialization
- netlib - Network support library
- nng - Nanomsg NG
- pppd - PPP server
- ptpd - PTPD client/server
- rexec - Remote execution client
- rexecd - Remote Execution Server
- rtptools - RTP Tools
- wakaama - Eclipse Wakaama (formerly liblwm2m) - an implementation of LWM2M protocol.
- wakeonlan - WakeOnLAN support

uIP Applications
----------------

This directory contains most of the network applications contained under the
``uIP-1.0`` apps directory. As the uIP ``apps/README.md`` says, these applications
"are not all heavily tested". These uIP-based apps include: ``dhcpc``, ``smtp``,
``webclient`` and ``webserver``

You may find additional information on these apps in the uIP forum accessible
through: http://www.sics.se/~adam/uip/index.php/Main_Page. Some of these (such
as the uIP web server) have grown some additional functionality due primarily to
NuttX user contributions.
