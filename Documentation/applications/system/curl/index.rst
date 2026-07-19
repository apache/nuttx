===============================
``curl`` HTTP client command
===============================

The ``curl`` command is a small command-line HTTP client built on top of the
``netutils`` webclient library. It mimics a subset of the well known
`curl <https://curl.se/docs/manpage.html>`_ tool so the same option syntax can
be used to exercise HTTP servers from the NuttShell (NSH).

It is **HTTP only** (no HTTPS/TLS) and supports the following options:

============  ================================================================
Option        Description
============  ================================================================
``-X``        HTTP method (default ``GET``, or ``POST`` when a body is sent).
``-H``        Add a request header (e.g. ``-H Content-Type:application/json``).
``-d``        Raw request body; ``-d @file`` reads the body from a file.
``-F``        multipart/form-data field; ``-F name=@file`` uploads a file.
``-o``        Write the response body to a file instead of stdout.
``-v``        Verbose: print the request line and status.
============  ================================================================

Configuration
=============

- ``CONFIG_SYSTEM_CURL``: Enable the ``curl`` command.
- ``CONFIG_NETUTILS_WEBCLIENT``: HTTP client library (required dependency).
- ``CONFIG_NET_TCP``: TCP support (required by the webclient).

The following additional options are available:

- ``CONFIG_SYSTEM_CURL_PROGNAME`` - Program name (default: ``curl``).
- ``CONFIG_SYSTEM_CURL_PRIORITY`` - Task priority (default: 100).
- ``CONFIG_SYSTEM_CURL_STACKSIZE`` - Task stack size. A networked HTTP
  client needs a comfortable stack; 8192 is a safe value.
- ``CONFIG_SYSTEM_CURL_BUFFERSIZE`` - Transfer buffer size (default: 512).
  It must be large enough to hold the whole request/response header line.

Usage
=====

.. code-block:: text

   curl [options] <url> ...

Examples
========

Download a file and save it with ``-o`` (using the public ``httpbin.org``
HTTP test service):

.. code-block:: text

   nsh> curl -o /mnt/dl.json http://httpbin.org/json
   curl: HTTP 200
   nsh> curl -o /mnt/img.png http://httpbin.org/image/png
   curl: HTTP 200
   nsh> ls -l /mnt/img.png
    -rw-rw-rw-        8090 /mnt/img.png

Send a JSON body with POST (``-d`` selects ``POST`` automatically, ``-H`` sets
the content type). ``httpbin.org/post`` echoes back what it received:

.. code-block:: text

   nsh> curl -H Content-Type:application/json -d '{"temp":25.3,"hum":60}' http://httpbin.org/post
   {
     "data": "{\"temp\":25.3,\"hum\":60}",
     "headers": {
       "Content-Type": "application/json",
       "Host": "httpbin.org"
     },
     "json": {
       "hum": 60,
       "temp": 25.3
     },
     "url": "http://httpbin.org/post"
   }
   curl: HTTP 200

Select a different method with ``-X``:

.. code-block:: text

   nsh> curl -X PUT -d '{"x":1}' http://httpbin.org/put
   curl: HTTP 200

Upload a file with multipart/form-data (``-F name=@file``) and download it
back with ``-o``:

.. code-block:: text

   nsh> curl -F file=@/mnt/nuttx_logo.png "http://192.168.1.10:8000/upload?version=logo1"
   {"filename":"nuttx_logo.png","sha256":"ca278dcb...","size_bytes":40343}curl: HTTP 201
   nsh> curl -o /mnt/logo_back.png http://192.168.1.10:8000/download/logo1/nuttx_logo.png
   curl: HTTP 200

.. note::

   The webclient transfer buffer is small and the request has a short timeout,
   so prefer small files (a few KB up to ~100 KB). HTTPS is not supported; use
   plain ``http://`` URLs only.

See Also
========

- `curl manual page <https://curl.se/docs/manpage.html>`_
