=====================
``tftpc`` TFTP Client
=====================

Overview
========

``tftpc`` is a lightweight, RFC 1350-compliant Trivial File Transfer Protocol (TFTP)
client library and utility for Apache NuttX. Designed specifically for resource-constrained
embedded systems, it facilitates low-overhead file transfers over UDP.

In NuttX environments, ``tftpc`` is widely used for:

- Bootloaders, remote firmware updates, and Over-The-Air (OTA) image transfers.
- Transferring small configuration files and system logs without requiring a complex
  TCP stack, TLS handshakes, or HTTP parsers.
- Interactive file retrieval and upload directly from the NuttShell (NSH).

.. note::

   **Security Consideration**: TFTP provides no authentication, encryption, or
   access control mechanisms. When performing firmware updates or transferring
   sensitive binaries over TFTP, operations should be restricted to trusted, isolated
   networks, and the receiving system must independently verify the image prior to
   installation using a verified digital signature (or a cryptographic digest received
   over a trusted out-of-band channel) to guarantee both origin authenticity and integrity.

It serves two primary roles:

1. **Underlying Engine for NSH ``get`` and ``put`` Commands**: Powers interactive
   terminal file transfer commands in the NuttShell.
2. **Programmatic C API**: Provides embedded applications with simple synchronous helpers
   (``tftpget()``, ``tftpput()``) as well as streaming callback interfaces (``tftpget_cb()``,
   ``tftpput_cb()``) that stream blocks directly into memory, flash partitions, or custom
   buffers without requiring a temporary local filesystem file.

Key Features
============

- **RFC 1350 Compliance**: Standard-compliant TFTP client protocol implementation.
- **Binary and Text Transfer Modes**: Supports both ``octet`` (binary) and ``netascii``
  (text) transfer modes.
- **Streaming Callback Architecture**: User-defined ``tftp_callback_t`` handlers process
  incoming or outgoing chunks on-the-fly, allowing direct flashing of MTD/flash partitions
  with minimal RAM consumption.
- **Dynamic Port Negotiation**: Connects to the server's well-known port (typically 69) and
  seamlessly adapts to the server's negotiated Transfer Identifier (TID) port.
- **Configurable Retransmission**: Built-in retransmission timeouts and retry counters to
  tolerate packet loss over wireless or congested networks.

Requirements & Dependencies
===========================

To use ``tftpc``, the following network subsystem features must be enabled:

- IPv4 Networking: ``CONFIG_NET=y`` and ``CONFIG_NET_IPv4=y``
- UDP Protocol: ``CONFIG_NET_UDP=y``
- Enable the Client Library: ``CONFIG_NETUTILS_TFTPC=y``
- For NSH Commands: ``!CONFIG_NSH_DISABLE_GET`` and ``!CONFIG_NSH_DISABLE_PUT``

Configuration Options
=====================

The behavior of the TFTP client can be customized via Kconfig and configuration settings:

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Setting
     - Default
     - Description
   * - ``CONFIG_NETUTILS_TFTPC``
     - ``n``
     - Enables the TFTP client library in ``apps/netutils/tftpc``.
   * - ``CONFIG_NETUTILS_TFTP_PORT``
     - ``69``
     - Target well-known TFTP server port used for the initial request.
   * - ``CONFIG_NETUTILS_TFTP_TIMEOUT``
     - ``10``
     - Socket receive timeout in deci-seconds (10 = 1 second).
   * - ``CONFIG_NETUTILS_TFTP_DUMPBUFFERS``
     - Defined
     - Debug option to dump transmitted and received packet buffers to stdout.

NSH Usage
=========

When ``CONFIG_NETUTILS_TFTPC`` is enabled along with NSH UDP network commands, the
``get`` and ``put`` commands are available directly in the NuttShell.

Downloading Files (``get``)
---------------------------

The ``get`` command downloads a file from a remote TFTP server:

.. code-block:: console

   get [-b|-n] [-f <local-path>] -h <ip-address> <remote-path>

Options:

- ``-b``: Transfer file in binary (``octet``) mode (default).
- ``-n``: Transfer file in text (``netascii``) mode.
- ``-f <local-path>``: Destination path on the local filesystem. If omitted, the file
  is written to the current directory using ``<remote-path>`` as the filename.
- ``-h <ip-address>``: IPv4 address of the remote TFTP server.
- ``<remote-path>``: Path or filename of the file on the remote server.

Example:

.. code-block:: console

   nsh> get -h 192.168.1.100 -f /mnt/sdcard/firmware.bin app_update.bin
   nsh> ls -l /mnt/sdcard/firmware.bin

Uploading Files (``put``)
-------------------------

The ``put`` command uploads a local file to a remote TFTP server:

.. code-block:: console

   put [-b|-n] [-f <remote-path>] -h <ip-address> <local-path>

Options:

- ``-b``: Transfer file in binary (``octet``) mode (default).
- ``-n``: Transfer file in text (``netascii``) mode.
- ``-f <remote-path>``: Destination filename on the remote TFTP server. If omitted,
  the file is written with the basename of ``<local-path>``.
- ``-h <ip-address>``: IPv4 address of the remote TFTP server.
- ``<local-path>``: Path to the local file to transmit.

Example:

.. code-block:: console

   nsh> put -h 192.168.1.100 -f remote_syslog.txt /var/log/syslog.txt

C API Reference
===============

The public C interface is declared in ``apps/include/netutils/tftp.h``.

Standard Filesystem APIs
------------------------

For standard file-to-file transfers using paths in the VFS:

.. code-block:: c

   #include <arpa/inet.h>
   #include <stdbool.h>
   #include <netutils/tftp.h>

   int tftpget(FAR const char *remote, FAR const char *local, in_addr_t addr,
               bool binary);

   int tftpput(FAR const char *local, FAR const char *remote, in_addr_t addr,
               bool binary);

Parameters:

- ``remote``: Filename on the remote TFTP server.
- ``local``: Filename in the local NuttX filesystem.
- ``addr``: IPv4 network address of the TFTP server in network byte order (e.g., from ``inet_addr()``).
- ``binary``: Pass ``true`` for binary (``octet``) mode, or ``false`` for text (``netascii``) mode.

Return Value:

- Returns ``OK`` (0) on success, or a negated errno value (or ``ERROR``) on failure.

Streaming Callback APIs
-----------------------

For transfers directly to/from memory, raw flash, or custom sinks:

.. code-block:: c

   typedef ssize_t (*tftp_callback_t)(FAR void *ctx, uint32_t offset,
                                      FAR uint8_t *buf, size_t len);

   int tftpget_cb(FAR const char *remote, in_addr_t addr, bool binary,
                  tftp_callback_t cb, FAR void *ctx);

   int tftpput_cb(FAR const char *remote, in_addr_t addr, bool binary,
                  tftp_callback_t cb, FAR void *ctx);

Callback semantics:

- **GET (Download)**: ``offset`` is zero. ``buf`` points to the received block (up to 512 bytes).
  ``len`` is the number of received bytes. The callback must return the number of bytes successfully
  processed, or a negative value to abort the transfer.
- **PUT (Upload)**: ``offset`` indicates the current stream byte position. ``buf`` points to the destination
  buffer to populate. ``len`` specifies the buffer size. The callback must return the number of bytes
  supplied, 0 on end-of-file (EOF), or a negative value to abort the transfer.

Application Example
===================

Below is a complete example demonstrating how an application can download a remote configuration
file using the streaming callback API ``tftpget_cb()``:

.. code-block:: c

   #include <nuttx/config.h>
   #include <stdio.h>
   #include <string.h>
   #include <errno.h>
   #include <arpa/inet.h>
   #include <netutils/tftp.h>

   struct memory_sink_s
   {
     FAR uint8_t *buffer;
     size_t max_size;
     size_t total_received;
   };

   static ssize_t tftp_recv_cb(FAR void *ctx, uint32_t offset,
                               FAR uint8_t *buf, size_t len)
   {
     FAR struct memory_sink_s *sink = (FAR struct memory_sink_s *)ctx;

     if (sink->total_received + len > sink->max_size)
       {
         fprintf(stderr, "Buffer overflow: payload exceeds capacity\n");
         return -ENOMEM;
       }

     memcpy(sink->buffer + sink->total_received, buf, len);
     sink->total_received += len;

     return len;
   }

   int fetch_config(FAR const char *server_ip, FAR const char *filename,
                    FAR uint8_t *dest_buf, size_t max_len)
   {
     struct memory_sink_s sink;
     in_addr_t server_addr;
     int ret;

     server_addr = inet_addr(server_ip);
     if (server_addr == INADDR_NONE)
       {
         return -EINVAL;
       }

     sink.buffer = dest_buf;
     sink.max_size = max_len;
     sink.total_received = 0;

     ret = tftpget_cb(filename, server_addr, true, tftp_recv_cb, &sink);
     if (ret < 0)
       {
         return ret;
       }

     return (int)sink.total_received;
   }
