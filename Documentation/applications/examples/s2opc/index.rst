==================================
``s2opc`` OPC UA server example
==================================

Overview
========

``s2opc`` runs an OPC UA server on NuttX.  It provides an anonymous
``SecurityPolicy None`` endpoint and an address space generated as C source,
without XML files or embedded certificates.

Configuration
=============

Enable ``CONFIG_EXAMPLES_S2OPC``.
``CONFIG_EXAMPLES_S2OPC_ENDPOINT_URL`` selects the default endpoint URL.

Usage
=====

Start the server with the configured endpoint::

  nsh> s2opc
  S2OPC server: opc.tcp://10.0.0.2:4841
  Press Ctrl-C to stop

An endpoint URL can be supplied on the command line::

  nsh> s2opc opc.tcp://192.168.1.20:4841

The URL address must belong to a configured NuttX network interface.  Press
Ctrl-C to stop the server.

Client test
===========

Connect an OPC UA client to the printed endpoint.  The address space includes
the standard ``ServerStatus.CurrentTime`` node, ``i=2258``, and the test
variable ``ns=1;s=PubInt16``.  For example, with the AsyncUA command-line
tools installed on the host::

  $ uaread -u opc.tcp://10.0.0.2:4841 -n i=2258

This test needs no separate OPC UA server: NuttX is the server and the host
tool is the client.
