=============================
``webclient`` HTTP Web Client
=============================

Overview
========

``webclient`` is a lightweight, efficient HTTP/1.0 and HTTP/1.1 client library
and application utility for Apache NuttX. Designed specifically for embedded and
resource-constrained environments, it provides full-featured HTTP and HTTPS client
capabilities with a minimal memory footprint.

It serves two primary roles in NuttX:

1. **Underlying Engine for NSH ``wget``**: Powers the interactive ``wget`` command
   used in the NuttShell (NSH) to download files, firmware images, and web content.
2. **Programmatic C API**: Provides embedded applications with flexible APIs ranging
   from simple one-shot HTTP GET/POST helpers to an extensible context-based API
   supporting streaming callbacks, custom request/response headers, chunked
   transfer encoding, non-blocking I/O, proxy tunneling, and pluggable TLS (HTTPS).

Key Features
============

- **HTTP/1.0 and HTTP/1.1 Compliance**: Handles standard HTTP requests, response
  status codes, header parsing, and redirects (301, 302, 307, 308).
- **Chunked Transfer Encoding**: Seamlessly consumes ``Transfer-Encoding: chunked``
  responses from modern web servers.
- **Streaming Callback Architecture**: Consumes incoming payload chunks via user-provided
  sink callbacks, avoiding the need to buffer entire response files into RAM.
- **Pluggable TLS / HTTPS Support**: Abstracted via ``struct webclient_tls_ops``, allowing
  transparent HTTPS operation using TLS engines like mbedTLS or wolfSSL.
- **Non-Blocking Operation**: Supports asynchronous, event-driven execution using
  ``WEBCLIENT_FLAG_NON_BLOCKING`` and pollable file descriptors. Note that hostname
  resolution (DNS lookup) is currently performed in a blocking manner.
- **HTTP Proxy & Tunneling**: Supports HTTP CONNECT proxies and tunnel establishment
  for secure enterprise deployments.

Requirements & Dependencies
===========================

To use ``webclient``, the following network subsystem features must be enabled:

- TCP Networking: ``CONFIG_NET=y`` and ``CONFIG_NET_TCP=y``
- Socket Options: ``CONFIG_NET_SOCKOPTS=y``
- DNS Client / Host Resolution: ``CONFIG_LIBC_NETDB=y`` and ``CONFIG_NETDB_DNSCLIENT=y``
- Generic URL Parser: ``CONFIG_NETUTILS_NETLIB_GENERICURLPARSER=y``
- Enable the Library: ``CONFIG_NETUTILS_WEBCLIENT=y``

Configuration Options
=====================

The behavior of ``webclient`` is configured through Kconfig under
``Application Configuration -> Network Utilities -> uIP web client``:

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Setting
     - Default
     - Description
   * - ``CONFIG_NETUTILS_WEBCLIENT``
     - ``n``
     - Enables the web client library.
   * - ``CONFIG_NSH_WGET_USERAGENT``
     - ``NuttX/6.xx.x``
     - HTTP User-Agent string sent in requests by NSH ``wget``.
   * - ``CONFIG_WEBCLIENT_TIMEOUT``
     - ``10``
     - Socket connect, send, and receive timeout in seconds.
   * - ``CONFIG_WEBCLIENT_MAXHTTPLINE``
     - ``200``
     - Maximum buffer size (in bytes) for a single HTTP header line.
   * - ``CONFIG_WEBCLIENT_MAXMIMESIZE``
     - ``32``
     - Maximum buffer size (in bytes) for parsed MIME type strings.
   * - ``CONFIG_WEBCLIENT_MAXHOSTNAME``
     - ``40``
     - Maximum length of destination hostnames.
   * - ``CONFIG_WEBCLIENT_MAXFILENAME``
     - ``100``
     - Maximum length of request path and filename strings.

NSH ``wget`` Usage
==================

When ``CONFIG_NETUTILS_WEBCLIENT`` is enabled, the ``wget`` command is available
directly in the NuttShell (unless disabled via ``CONFIG_NSH_DISABLE_WGET``).

Synopsis
--------

.. code-block:: console

   wget [-o <local-path>] <url>

Options:

- ``-o <local-path>``: Save the downloaded file to the specified local filesystem path.
- ``<url>``: Fully qualified URL (e.g., ``http://192.168.1.1/firmware.bin``).

Examples
--------

Downloading a configuration or payload file to the local filesystem:

.. code-block:: console

   nsh> wget -o /tmp/config.json http://192.168.1.50:8080/config.json
   nsh> cat /tmp/config.json

Fetching a remote resource and printing directly to the terminal:

.. code-block:: console

   nsh> wget http://192.168.1.1/status.txt

C API Reference
===============

The public C interface is declared in ``apps/include/netutils/webclient.h``.

Simple Convenience APIs
-----------------------

For standard HTTP GET and POST requests where streaming or basic buffering is sufficient:

.. code-block:: c

   int wget(FAR const char *url, FAR char *buffer, int buflen,
            wget_callback_t callback, FAR void *arg);

   int wget_post(FAR const char *url, FAR const char *posts, FAR char *buffer,
                 int buflen, wget_callback_t callback, FAR void *arg);

- **``url``**: Null-terminated HTTP URL string.
- **``buffer``**: Caller-allocated scratch buffer used for request headers and data chunks.
- **``buflen``**: Size of the buffer in bytes (typically 512 bytes or larger).
- **``callback``**: Callback function invoked iteratively as each chunk of data arrives.
- **``arg``**: User context pointer passed directly to the callback.

Context-Based API
-----------------

For advanced operations (custom headers, non-blocking I/O, streaming request bodies,
or TLS), applications use ``struct webclient_context``:

.. code-block:: c

   void webclient_set_defaults(FAR struct webclient_context *ctx);
   int webclient_perform(FAR struct webclient_context *ctx);
   void webclient_abort(FAR struct webclient_context *ctx);
   void webclient_set_static_body(FAR struct webclient_context *ctx,
                                  FAR const void *body,
                                  size_t bodylen);

Key context structure fields:

- ``url``: Target URL.
- ``method``: HTTP method (``"GET"``, ``"POST"``, ``"PUT"``, ``"DELETE"``, etc.).
- ``sink_callback``: Callback of type ``webclient_sink_callback_t`` to consume response body chunks.
- ``header_callback``: Optional callback of type ``webclient_header_callback_t`` to intercept headers.
- ``body_callback``: Callback of type ``webclient_body_callback_t`` for streaming request upload bodies.
- ``tls_ops``: Pointer to ``struct webclient_tls_ops`` for HTTPS connections.
- ``flags``: Bitwise flags (e.g., ``WEBCLIENT_FLAG_NON_BLOCKING``).

Application Example
===================

Below is an example showing how an embedded application can fetch a remote file
and robustly save it to the local filesystem using ``wget()``:

.. code-block:: c

   #include <nuttx/config.h>
   #include <stdio.h>
   #include <fcntl.h>
   #include <unistd.h>
   #include <errno.h>
   #include <netutils/webclient.h>

   struct download_ctx_s
   {
     int fd;
     int error;
   };

   static void file_sink(FAR char **buffer, int offset,
                         int datend, FAR int *buflen, FAR void *arg)
   {
     FAR struct download_ctx_s *ctx = (FAR struct download_ctx_s *)arg;
     FAR const char *ptr = *buffer + offset;
     int remaining = datend - offset;

     if (ctx->error != 0 || remaining <= 0)
       {
         return;
       }

     while (remaining > 0)
       {
         ssize_t written = write(ctx->fd, ptr, remaining);
         if (written < 0)
           {
             if (errno == EINTR)
               {
                 continue;
               }

             ctx->error = -errno;
             return;
           }

         ptr += written;
         remaining -= written;
       }
   }

   int download_file(FAR const char *url, FAR const char *destination)
   {
     struct download_ctx_s ctx;
     char buffer[512];
     int ret;
     int cret;

     ctx.fd = open(destination, O_WRONLY | O_CREAT | O_TRUNC, 0666);
     if (ctx.fd < 0)
       {
         return -errno;
       }

     ctx.error = 0;
     ret = wget(url, buffer, sizeof(buffer), file_sink, &ctx);
     cret = close(ctx.fd);

     if (ret < 0)
       {
         return ret;
       }

     if (ctx.error != 0)
       {
         return ctx.error;
       }

     if (cret < 0)
       {
         return -errno;
       }

     return 0;
   }

