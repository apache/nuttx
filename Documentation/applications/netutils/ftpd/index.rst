===================
``ftpd`` FTP Server
===================

The ``ftpd`` utility is a lightweight File Transfer Protocol (FTP) server implementation
for Apache NuttX, conforming to the core protocol specification defined in :rfc:`959`,
with modern extensions for IPv6 networking (:rfc:`2428`) and file metadata inspection
(:rfc:`3659`).

It provides embedded microcontrollers with bidirectional network file transfer
capabilities, allowing remote desktop computers, automated update utilities, and
monitoring tools to inspect, upload, download, and delete files on the NuttX Virtual
File System (VFS), SD cards, or flash partitions over local Ethernet or Wi-Fi links.

.. note::

   **Security Consideration**: Standard FTP (:rfc:`959`) transmits usernames, passwords,
   and file contents in unencrypted plaintext across the network. On untrusted or public
   networks, credentials and transferred data can be intercepted by network sniffers.
   When deploying ``ftpd`` in production environments:

   - Restrict FTP traffic to isolated, trusted local networks or private maintenance VLANs.
   - Where possible, enforce strong passwords via the encrypted password subsystem
     (``CONFIG_FTPD_LOGIN_PASSWD`` with ``CONFIG_FSUTILS_PASSWD``).
   - Confine guest accounts to dedicated subdirectories using home directory isolation
     (``FTPD_ACCOUNTFLAG_GUEST``).
   - For mission-critical external transfers, prefer encrypted protocols such as SSH/SFTP
     or HTTPS.

Operational Architecture
========================

The NuttX FTP server operates using a multi-threaded daemon model:

1. **Master Server Socket**: A listening TCP socket bound to the FTP control port (default
   port 21) accepts incoming client connections.
2. **Worker Pthreads**: When a remote FTP client connects, ``ftpd_session()`` accepts the
   TCP connection and spawns a dedicated worker thread (with stack size governed by
   ``CONFIG_FTPD_WORKERSTACKSIZE``) to handle control session interactions.
3. **Dual Connection Model**: The server maintains a persistent control connection
   for ASCII commands and reply codes (e.g., ``USER``, ``PASS``, ``PASV``), and
   establishes transient data connections on demand for directory listings
   (``LIST``, ``NLST``) and file transfers (``RETR``, ``STOR``).
4. **VFS Integration**: Directory navigation, file creation, reading, and writing map
   directly to standard POSIX file system system calls (``open()``, ``read()``, ``write()``,
   ``stat()``, ``opendir()``, ``unlink()``, ``rename()``).

Key Features
============

- **RFC 959 Core Protocol Support**: Full support for standard FTP file retrieval, storage,
  and directory management.
- **Active and Passive Modes**: Supports both Active (``PORT`` / ``EPRT``) and Passive
  (``PASV`` / ``EPSV``) data connections, ensuring seamless operation across client firewalls
  and NAT gateways.
- **IPv4 and IPv6 Dual-Stack**: Implements :rfc:`2428` commands (``EPRT`` and ``EPSV``) to
  support both ``AF_INET`` and ``AF_INET6`` address families.
- **Resumable Transfers**: Supports the ``REST`` (restart marker) command to resume
  interrupted file transfers without retransmitting previously transferred blocks.
- **Extended File Metadata**: Implements :rfc:`3659` commands (``SIZE`` and ``MDTM``) to
  report file byte lengths and last-modification timestamps.
- **User Account Management**: In-memory user database supporting multiple concurrent
  accounts with configurable privileges (administrator, system, or guest) and directory
  isolation.
- **Encrypted Password Integration**: Optional integration with ``apps/fsutils/passwd``
  to authenticate users against encrypted password databases.

Supported FTP Commands
======================

The NuttX FTP server implements a comprehensive command subset:

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Command
     - Description & Protocol Standard
   * - ``USER``
     - Specify client username (:rfc:`959`).
   * - ``PASS``
     - Specify client password (:rfc:`959`).
   * - ``QUIT``
     - Terminate the control connection (:rfc:`959`).
   * - ``SYST``
     - Return server system type (reports ``UNIX Type: L8``) (:rfc:`959`).
   * - ``TYPE``
     - Set transfer type: ASCII (``A``) or Image/Binary (``I``) (:rfc:`959`).
   * - ``MODE``
     - Set transfer mode: Stream (``S``) (:rfc:`959`).
   * - ``STRU``
     - Set file structure: File (``F``) (:rfc:`959`).
   * - ``PWD`` / ``XPWD``
     - Print working directory (:rfc:`959`).
   * - ``CWD`` / ``XCWD``
     - Change working directory (:rfc:`959`).
   * - ``CDUP`` / ``XCUP``
     - Change to parent directory (:rfc:`959`).
   * - ``MKD`` / ``XMKD``
     - Create a directory (:rfc:`959`).
   * - ``RMD`` / ``XRMD``
     - Remove an empty directory (:rfc:`959`).
   * - ``DELE``
     - Delete a file (:rfc:`959`).
   * - ``RNFR`` / ``RNTO``
     - Rename file from / rename file to sequence (:rfc:`959`).
   * - ``PORT``
     - Initiate active IPv4 data connection (:rfc:`959`).
   * - ``PASV``
     - Request passive IPv4 data connection (:rfc:`959`).
   * - ``EPRT``
     - Initiate extended active data connection (IPv4/IPv6) (:rfc:`2428`).
   * - ``EPSV``
     - Request extended passive data connection (IPv4/IPv6) (:rfc:`2428`).
   * - ``LIST``
     - Transmit detailed directory listing (:rfc:`959`).
   * - ``NLST``
     - Transmit filename-only directory list (:rfc:`959`).
   * - ``RETR``
     - Download a file from the server (:rfc:`959`).
   * - ``STOR``
     - Upload a file to the server (:rfc:`959`).
   * - ``APPE``
     - Append uploaded data to an existing file (:rfc:`959`).
   * - ``REST``
     - Set byte offset for resuming a transfer (:rfc:`959`).
   * - ``SIZE``
     - Return the exact byte size of a file (:rfc:`3659`).
   * - ``MDTM``
     - Return the last modification timestamp of a file (:rfc:`3659`).
   * - ``ABOR``
     - Abort the active data transfer (:rfc:`959`).
   * - ``NOOP``
     - No operation; keeps connection alive (:rfc:`959`).
   * - ``HELP``
     - Display supported commands (:rfc:`959`).

Requirements & Dependencies
===========================

To enable the FTP server in your NuttX build, ensure the following subsystem
dependencies are configured:

- Core Networking: ``CONFIG_NET=y``
- TCP Protocol: ``CONFIG_NET_TCP=y``
- POSIX Pthreads: ``CONFIG_DISABLE_PTHREAD=n`` (required for multi-session handling)
- File System: ``CONFIG_NFILE_DESCRIPTORS > 0`` (with mounted storage, e.g., ROMFS, FAT, LittleFS)
- Client Library: ``CONFIG_NETUTILS_FTPD=y``
- Password Support (Optional): ``CONFIG_FTPD_LOGIN_PASSWD=y`` and ``CONFIG_FSUTILS_PASSWD=y``
- Standalone Example Daemon (Optional): ``CONFIG_EXAMPLES_FTPD=y``

Configuration Options
=====================

The behavior and resource consumption of the FTP server can be configured via Kconfig:

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Kconfig Option
     - Default
     - Description
   * - ``CONFIG_NETUTILS_FTPD``
     - ``n``
     - Enables compilation of the FTP server library in ``apps/netutils/ftpd``.
   * - ``CONFIG_FTPD_WORKERSTACKSIZE``
     - ``DEFAULT_TASK_STACKSIZE``
     - Stack size in bytes allocated for each FTP session worker pthread.
   * - ``CONFIG_FTPD_LOGIN_PASSWD``
     - ``n``
     - Validates login credentials against an encrypted password database via
       ``apps/fsutils/passwd``.
   * - ``CONFIG_FTPD_VENDORID``
     - ``"NuttX"``
     - Vendor identification string reported in server greeting banners.
   * - ``CONFIG_FTPD_SERVERID``
     - ``"NuttX FTP Server"``
     - Server product name string reported in the ``220`` greeting message.
   * - ``CONFIG_FTPD_CMDBUFFERSIZE``
     - ``128``
     - Size in bytes of the command line buffer for incoming FTP instructions.
   * - ``CONFIG_FTPD_DATABUFFERSIZE``
     - ``512``
     - Buffer size in bytes used for I/O chunks during data socket transfers.
   * - ``CONFIG_EXAMPLES_FTPD``
     - ``n``
     - Builds the standalone ``ftpd`` / ``ftpd_stop`` command line example in NSH.
   * - ``CONFIG_EXAMPLES_FTPD_PORT``
     - ``21``
     - Default TCP port used by the standalone example daemon.

Running the FTP Daemon from NSH
===============================

When ``CONFIG_EXAMPLES_FTPD=y`` is enabled, the FTP server can be started and stopped
interactively from the NuttShell.

Starting the Daemon
-------------------

The standalone daemon requires an address family parameter specifying either ``-4`` (IPv4)
or ``-6`` (IPv6):

.. code-block:: console

   nsh> ftpd -4
   Initializing the network
   Starting the FTP daemon
   FTP daemon [3] started
   Adding accounts:
   1. Root account: USER=root PASSWORD=<configured-password> HOME=(none)
   2. User account: USER=ftp PASSWORD=(none) HOME=(none)
   3. User account: USER=anonymous PASSWORD=(none) HOME=(none)

.. warning::

   **Mandatory Credential Hardening**:
   The standalone example in ``apps/examples/ftpd`` is intended for local hardware
   testing. Before enabling ``ftpd`` in any networked or production environment,
   change the demonstration credentials to strong, unique passwords or authenticate
   against an encrypted password database via ``CONFIG_FTPD_LOGIN_PASSWD``. Never
   deploy publicly known default passwords.

The daemon initializes the network interface if not already active, opens a listening
socket on ``CONFIG_EXAMPLES_FTPD_PORT`` (default port 21), registers the configured accounts,
and awaits incoming connections in a background task.

Stopping the Daemon
-------------------

To cleanly terminate the running FTP daemon:

.. code-block:: console

   nsh> ftpd_stop
   Stopping the FTP daemon, pid=3

Connecting from an External FTP Client
======================================

Once the daemon is running, connect using standard command-line FTP clients, ``curl``,
or graphical transfer tools like FileZilla:

Using Command-Line FTP
----------------------

Authenticate using the configured ``root`` account and password:

.. code-block:: console

   $ ftp 192.168.1.150
   Connected to 192.168.1.150.
   220 NuttX FTP Server ready.
   Name (192.168.1.150:user): root
   331 Password required for root.
   Password: <your-password>
   230 User logged in.
   Remote system type is UNIX.
   Using binary mode to transfer files.
   ftp> pwd
   257 "/" is current directory.
   ftp> ls
   200 PORT command successful.
   150 Opening data connection for directory list.
   drwxr-xr-x   1 root root        0 Jan 01 00:00 dev
   drwxr-xr-x   1 root root        0 Jan 01 00:00 etc
   drwxr-xr-x   1 root root        0 Jan 01 00:00 mnt
   226 Transfer complete.
   ftp> put firmware.bin /mnt/sdcard/firmware.bin
   200 PORT command successful.
   150 Opening data connection for firmware.bin.
   226 Transfer complete. 524288 bytes sent in 2.1 secs.
   ftp> quit
   221 Goodbye.

Alternatively, connect anonymously without a password using the ``anonymous`` account:

.. code-block:: console

   $ ftp 192.168.1.150
   Connected to 192.168.1.150.
   220 NuttX FTP Server ready.
   Name (192.168.1.150:user): anonymous
   230 User logged in.
   ftp> ls
   200 PORT command successful.
   150 Opening data connection for directory list.
   drwxr-xr-x   1 root root        0 Jan 01 00:00 mnt
   226 Transfer complete.
   ftp> quit
   221 Goodbye.

Using cURL
----------

Files can also be transferred programmatically from external scripts using ``curl``
(for interactive use, omit the password to receive a prompt; for scripts, load credentials
from a protected source such as ``~/.netrc`` or environment variables):

.. code-block:: console

   # Download a file from NuttX VFS using root credentials (prompts for password):
   curl -u root ftp://192.168.1.150/mnt/sdcard/log.txt -o downloaded_log.txt

   # Upload a file to NuttX VFS using root credentials (prompts for password):
   curl -u root -T config.ini ftp://192.168.1.150/mnt/sdcard/config.ini

   # Download anonymously without credentials:
   curl ftp://192.168.1.150/mnt/sdcard/log.txt -o downloaded_log.txt

C API Reference
===============

The public C interface for ``ftpd`` is declared in ``apps/include/netutils/ftpd.h``.

Types & Constants
-----------------

.. code-block:: c

   /* Opaque FTP session handle */
   typedef FAR void *FTPD_SESSION;

   /* User Account Flags */
   #define FTPD_ACCOUNTFLAG_NONE    (0)       /* Standard unprivileged user */
   #define FTPD_ACCOUNTFLAG_ADMIN   (1 << 0)  /* Administrative privileges */
   #define FTPD_ACCOUNTFLAG_SYSTEM  (1 << 1)  /* System root access */
   #define FTPD_ACCOUNTFLAG_GUEST   (1 << 2)  /* Restricted guest access */

Functions
---------

Creating the Server Instance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   FTPD_SESSION ftpd_open(int port, sa_family_t family);

- Allocates and initializes the FTP server instance, binding the listening socket
  to the designated port and address family.
- ``port``: TCP listen port (typically 21, or any unreserved port).
- ``family``: Socket address family (``AF_INET`` for IPv4 or ``AF_INET6`` for IPv6).
- *Returns*: A non-``NULL`` ``FTPD_SESSION`` handle on success, or ``NULL`` on failure.

Adding User Accounts
~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   int ftpd_adduser(FTPD_SESSION handle, uint8_t accountflags,
                    FAR const char *user, FAR const char *passwd,
                    FAR const char *home);

- Registers an account in the FTP server's user database.
- ``handle``: Server handle returned by ``ftpd_open()``.
- ``accountflags``: Bitmask of ``FTPD_ACCOUNTFLAG_*`` defining privilege level.
- ``user``: Username string. Pass ``NULL`` to allow anonymous login without a username.
- ``passwd``: Password string. Pass ``NULL`` to permit login without a password.
- ``home``: Initial home directory path for the user (e.g., ``"/mnt/sdcard"``).
- *Returns*: ``0`` (``OK``) on success, or a negated errno value (e.g., ``-ENOMEM``) on failure.

Driving the Server Session Loop
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   int ftpd_session(FTPD_SESSION handle, int timeout);

- Listens for an incoming client connection on the server socket.
- When a connection arrives, it accepts the connection and spawns a worker thread to
  service the FTP session.
- ``handle``: Server handle returned by ``ftpd_open()``.
- ``timeout``: Connection wait timeout in milliseconds.
- *Returns*: ``0`` (``OK``) on connection acceptance and worker thread creation,
  ``-ETIMEDOUT`` if the timeout elapsed with no incoming connection (normal loop
  condition), or a negative errno value on failure.

Destroying the Server Instance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   void ftpd_close(FTPD_SESSION handle);

- Closes listening sockets, terminates active sessions, and deallocates all internal
  resources and account records associated with the handle.

Application Example
===================

Below is a complete, standalone example demonstrating how an embedded task initializes
an FTP server daemon with multiple accounts, drives the connection loop, and provides
a signal-safe, thread-safe shutdown path:

.. code-block:: c

   #include <nuttx/config.h>
   #include <stdio.h>
   #include <stdlib.h>
   #include <stdbool.h>
   #include <signal.h>
   #include <unistd.h>
   #include <errno.h>
   #include <sys/socket.h>
   #include <netinet/in.h>

   #include <netutils/ftpd.h>

   /* Signal-safe termination flag modified exclusively by signal handlers */
   static volatile sig_atomic_t g_ftp_stop = 0;

   /* PID of the running FTP server daemon */
   static pid_t g_server_pid = -1;

   /* Signal handler to request graceful shutdown (async-signal-safe) */
   static void ftp_signal_handler(int signo)
   {
     g_ftp_stop = 1;
   }

   /* Thread-safe stop API callable from other tasks/threads via POSIX kill */
   int stop_my_ftp_server(void)
   {
     if (g_server_pid > 0)
       {
         return kill(g_server_pid, SIGTERM);
       }

     return -ESRCH;
   }

   int start_my_ftp_server(void)
   {
     FTPD_SESSION handle;
     int ret;

     /* Initialize PID and clear stop flag BEFORE installing signal handlers */
     g_server_pid = getpid();
     g_ftp_stop = 0;

     /* Install signal handlers to allow clean shutdown via kill or Ctrl+C */
     signal(SIGINT, ftp_signal_handler);
     signal(SIGTERM, ftp_signal_handler);

     /* 1. Open the FTP server listening on standard port 21 (IPv4) */

     handle = ftpd_open(21, AF_INET);
     if (handle == NULL)
       {
         fprintf(stderr, "ERROR: Failed to open FTP server on port 21\n");
         g_server_pid = -1;
         return -EIO;
       }

     /* 2. Configure user accounts (replace placeholders with strong credentials) */

     /* Admin account: full access starting at filesystem root */
     ret = ftpd_adduser(handle, FTPD_ACCOUNTFLAG_ADMIN,
                        "admin", "<strong-admin-password>", "/");
     if (ret < 0)
       {
         fprintf(stderr, "Failed to add admin user: %d\n", ret);
         ftpd_close(handle);
         g_server_pid = -1;
         return ret;
       }

     /* Guest account: unprivileged access restricted to /mnt/sdcard */
     ret = ftpd_adduser(handle, FTPD_ACCOUNTFLAG_GUEST,
                        "guest", "<strong-guest-password>", "/mnt/sdcard");
     if (ret < 0)
       {
         fprintf(stderr, "Failed to add guest user: %d\n", ret);
         ftpd_close(handle);
         g_server_pid = -1;
         return ret;
       }

     printf("FTP server running on port 21. Waiting for connections...\n");

     /* 3. Server connection loop: runs until g_ftp_stop is set */

     while (!g_ftp_stop)
       {
         /* Wait up to 5000 ms for incoming connection requests */
         ret = ftpd_session(handle, 5000);
         if (ret == 0)
           {
             printf("FTP client connected and session worker spawned.\n");
           }
         else if (ret != -ETIMEDOUT)
           {
             fprintf(stderr, "ftpd_session error: %d\n", ret);
             break;
           }
       }

     /* 4. Cleanup and shutdown upon loop exit */

     printf("Shutting down FTP server...\n");
     ftpd_close(handle);
     g_server_pid = -1;
     return OK;
   }

Embedded Design & Resource Tuning
=================================

1. **Stack Sizing for Worker Threads**: Each active FTP client connection creates a
   worker thread. Ensure ``CONFIG_FTPD_WORKERSTACKSIZE`` is sized appropriately for
   the underlying filesystem driver (e.g., FAT filesystems or LittleFS typically
   require at least 2048 to 3072 bytes of stack space).
2. **Buffer Sizing for Network vs. Flash Throughput**: ``CONFIG_FTPD_DATABUFFERSIZE``
   controls the chunk size read and written during file transfers (default: 512
   bytes). Increasing this buffer to match the storage block size (e.g., 1024 or 2048
   bytes) improves throughput on high-speed Ethernet targets at the expense of RAM.
3. **Passive Mode on NAT / Firewalled Networks**: Embedded devices positioned
   behind firewalls should instruct remote clients to use **Passive Mode**
   (``PASV`` or ``EPSV``), ensuring the data connection is initiated by the client
   toward the embedded server rather than requiring the server to initiate inbound
   connections to client ports.
