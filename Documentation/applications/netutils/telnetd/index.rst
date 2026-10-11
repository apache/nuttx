================================
``telnetd`` Telnet Server Daemon
================================

The ``telnetd`` utility provides a lightweight Telnet server daemon for Apache NuttX,
implementing the core network terminal protocol defined in :rfc:`854` (Telnet Protocol
Specification) and :rfc:`855` (Telnet Option Specifications), with optional support for
terminal window size negotiation via :rfc:`1073` (Negotiate About Window Size - NAWS).

In embedded systems, ``telnetd`` enables remote interactive access over local Ethernet
or Wi-Fi networks by decoupling the network protocol logic from the shell application.
The daemon dynamically wraps accepted TCP connections into standard character device
drivers (``/dev/telnetN``) and duplicates them onto the standard input, output, and error
file descriptors (``stdin``, ``stdout``, ``stderr``) of newly spawned child tasks. This
architecture allows standard console applications—such as the NuttShell (NSH) or custom
command interpreters—to run transparently over the network without requiring any
network-specific awareness.

.. note::

   **Security Consideration**: Standard Telnet (:rfc:`854`) transmits all terminal sessions,
   user input, and authentication credentials in unencrypted plaintext across the network.
   On untrusted or open networks, traffic can be monitored or modified by unauthorized
   eavesdroppers. When deploying ``telnetd`` in production systems:

   - Confine Telnet traffic to isolated, trusted management networks, private maintenance
     VLANs, or point-to-point diagnostic interfaces.
   - Where remote access is required across untrusted links, enforce cryptographic transport
     security using secure tunnels (such as IPsec or WireGuard) or deploy an encrypted shell
     service such as Dropbear SSH (``apps/netutils/dropbear``).
   - If authentication is required, enable the NuttShell login feature
     (``CONFIG_NSH_TELNET_LOGIN`` with ``CONFIG_FSUTILS_PASSWD``).

Operational Architecture
========================

The NuttX Telnet subsystem is split across two modular layers: a user-space daemon
(``apps/netutils/telnetd/``) and a kernel character driver factory (``drivers/net/telnet.c``).

.. code-block:: text

   +-------------------------------------------------------------+
   |                        Remote Client                        |
   |           (e.g., telnet, PuTTY, netcat on TCP Port 23)      |
   +-------------------------------------------------------------+
                                  |
                                  | TCP Network Stream (RFC 854)
                                  v
   +-------------------------------------------------------------+
   |                    telnetd Daemon Loop                      |
   |              accept4() on TCP Listening Socket              |
   +-------------------------------------------------------------+
                                  |
                                  | ioctl(SIOCTELNET) via /dev/telnet
                                  v
   +-------------------------------------------------------------+
   |               Kernel Telnet Driver Factory                  |
   |                  (drivers/net/telnet.c)                     |
   |    - Allocates /dev/telnetN session character driver        |
   |    - Implements IAC protocol & option negotiation (RFC 854) |
   |    - Formats RFC 854 transmit line endings (CRLF, CR NUL)   |
   |    - Manages circular RX/TX ring buffers                    |
   +-------------------------------------------------------------+
                                  |
                                  | dup2(drvrfd, 0 / 1 / 2)
                                  v
   +-------------------------------------------------------------+
   |                   Spawned Shell Process                     |
   |               (e.g., NSH task_create / spawn)               |
   |                                                             |
   |       stdin  -----> [ Read from /dev/telnetN ]              |
   |       stdout -----> [ Write to /dev/telnetN  ]              |
   |       stderr -----> [ Write to /dev/telnetN  ]              |
   +-------------------------------------------------------------+

Lifecycle Sequence
------------------

1. **Daemon Initialization**: The user invokes ``telnetd_daemon()`` with a populated
   ``struct telnetd_config_s`` configuration. The daemon creates a stream socket, binds to
   the specified port (default port 23) using either ``AF_INET`` or ``AF_INET6``, and calls
   ``listen()`` with a connection backlog.
2. **Connection Acceptance**: When a remote Telnet client initiates a TCP connection,
   the daemon's ``accept4()`` call unblocks and returns a connected socket descriptor
   (``acceptsd``) with ``SOCK_CLOEXEC`` set.
3. **Session Driver Creation**: The daemon opens the Telnet driver factory node at
   ``/dev/telnet`` and issues an ``ioctl(drvrfd, SIOCTELNET, &session)`` request passing the
   accepted socket. The kernel driver registers a dedicated character device
   (e.g., ``/dev/telnet0``) backed by internal ring buffers and the session's TCP socket.
4. **Standard I/O Redirection**: The daemon closes its own default standard descriptors
   and uses ``dup2()`` to duplicate the newly opened ``/dev/telnetN`` file descriptor onto
   descriptors 0 (``stdin``), 1 (``stdout``), and 2 (``stderr``).
5. **Task Spawning**: The daemon spawns the target interactive task (such as ``nsh_main()``)
   via ``task_create()`` in flat and protected builds, or via ``posix_spawn()`` in builds
   with ``CONFIG_LIBC_EXECFUNCS``. The new task inherits the duplicated file descriptors.
6. **Session Termination**: When the remote client disconnects or the user types ``exit``,
   the shell task terminates, the character device is closed and unregistered by the
   kernel, and socket resources are released.

Kernel Telnet Protocol Handling
-------------------------------

The underlying character driver (``drivers/net/telnet.c``) handles all Telnet protocol
control sequences transparently:

- **Interpret As Command (IAC)**: Decodes Telnet escape sequences beginning with byte
  ``0xFF`` (``IAC``), including negotiation commands ``WILL`` (``0xFB``), ``WONT`` (``0xFC``),
  ``DO`` (``0xFD``), and ``DONT`` (``0xFE``).
- **Subnegotiation (SB / SE)**: Handles option subnegotiation sequences bracketed by ``IAC SB``
  and ``IAC SE``.
- **Window Size Negotiation (NAWS)**: When ``CONFIG_TELNET_SUPPORT_NAWS`` is enabled, the
  driver negotiates window dimensions (:rfc:`1073`) and provides window size queries
  via terminal ``ioctl()`` calls (such as ``TIOCGWINSZ``).
- **Line Ending Handling**: Adheres to :rfc:`854` end-of-line conventions. On the transmit
  path, outgoing newline characters (``\n``) are automatically formatted as standard network
  ``CR LF`` pairs. If an application outputs a carriage return alone, the driver defers
  the trailing ``NUL`` byte until the next non-newline character is processed (note that if a
  session terminates immediately following a standalone ``CR`` with no subsequent output,
  the driver closes without flushing a trailing ``NUL``). On the receive path, ordinary data
  bytes are passed through transparently to the application buffer, leaving input line
  discipline (such as stripping carriage returns or handling ``\r\n`` / ``\n``) to the
  interactive command interpreter (e.g., readline or NSH).

Key Features
============

- **Modular Kernel/Userland Separation**: Protocol parsing and STDIO virtualization are
  handled in the kernel driver, keeping the application daemon minimal and robust.
- **Universal Application Support**: Any standard CLI or shell can be used as the Telnet
  backend; applications interact with standard POSIX ``read()`` and ``write()`` without
  network socket code.
- **Dual-Stack Networking**: Full support for both IPv4 (``AF_INET``) and IPv6 (``AF_INET6``)
  address families.
- **Configurable Concurrency**: Supports multiple simultaneous Telnet sessions up to
  ``CONFIG_TELNET_MAXLCLIENTS``, each running in an isolated task context.
- **Low Footprint**: Highly optimized memory usage tailored for constrained embedded
  targets with configurable RX and TX buffer sizes.
- **NuttShell (NSH) Native Integration**: Turnkey remote console support via built-in NSH
  configuration flags.

Configuration Options
=====================

Core Telnet Daemon Configurations
---------------------------------

.. list-table::
   :widths: 30 15 55
   :header-rows: 1

   * - Configuration Option
     - Default
     - Description
   * - ``CONFIG_NETUTILS_TELNETD``
     - ``n``
     - Enables the Telnet daemon library in ``apps/netutils/telnetd``. Automatically selects
       ``CONFIG_NETDEV_TELNET``. Requires ``CONFIG_NET`` and ``CONFIG_NET_TCP``.
   * - ``CONFIG_NETDEV_TELNET``
     - ``n``
     - Enables the kernel-level Telnet character driver factory in ``drivers/net/telnet.c``,
       exposing the ``/dev/telnet`` device node.
   * - ``CONFIG_TELNET_RXBUFFER_SIZE``
     - ``256``
     - Size (in bytes) of the internal receive ring buffer per active Telnet session.
   * - ``CONFIG_TELNET_TXBUFFER_SIZE``
     - ``256``
     - Size (in bytes) of the internal transmit ring buffer per active Telnet session.
   * - ``CONFIG_TELNET_MAXLCLIENTS``
     - ``8``
     - Maximum number of concurrent Telnet client sessions allowed by the driver.
   * - ``CONFIG_TELNET_SUPPORT_NAWS``
     - ``n``
     - Enables Telnet Negotiate About Window Size (:rfc:`1073`) support to report terminal
       column and row dimensions.
   * - ``CONFIG_TELNET_DUMPBUFFER``
     - ``n``
     - Enables raw hexadecimal buffer dumps for debugging network transmissions (depends on
       ``CONFIG_DEBUG_NET``).

NSH Telnet Console Configurations
---------------------------------

When using Telnet as a remote console for NuttShell (NSH), the following settings in
``apps/nshlib/`` govern behavior:

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Configuration Option
     - Default
     - Description
   * - ``CONFIG_NSH_TELNET``
     - ``n``
     - Enables the NSH Telnet remote console service.
   * - ``CONFIG_NSH_TELNETD_PORT``
     - ``23``
     - TCP port on which the NSH Telnet daemon listens for incoming connections.
   * - ``CONFIG_NSH_TELNETD_DAEMONPRIO``
     - ``100``
     - Execution priority of the NSH Telnet listener daemon task.
   * - ``CONFIG_NSH_TELNETD_DAEMONSTACKSIZE``
     - ``2048``
     - Stack size (in bytes) allocated for the NSH Telnet listener daemon task.
   * - ``CONFIG_NSH_TELNETD_CLIENTPRIO``
     - ``100``
     - Execution priority of each spawned client shell session.
   * - ``CONFIG_NSH_TELNETD_CLIENTSTACKSIZE``
     - ``2048``
     - Stack size (in bytes) allocated for each spawned client shell session.
   * - ``CONFIG_NSH_TELNET_LOGIN``
     - ``n``
     - Enforces username and password authentication before granting shell access over Telnet.

Recommended STDIO Tuning
------------------------

To achieve smooth interactive performance over Telnet, ensure the following STDIO buffering
options are configured in NuttX:

- **``CONFIG_STDIO_BUFFER_SIZE >= 64``**: Using buffered I/O avoids generating tiny TCP
  packets for each individual printed character.
- **``CONFIG_STDIO_LINEBUFFER=y``**: Ensures line-oriented console output flushes immediately
  on newline boundaries (``\n``), offering responsive terminal feedback.

Application Programming Interface (API)
=======================================

Data Types
----------

``struct telnetd_config_s``
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This configuration structure is passed to ``telnetd_daemon()`` to define the daemon's
listening parameters and the characteristics of child tasks to be spawned upon connection:

.. code-block:: c

   #include <netutils/telnetd.h>

   struct telnetd_config_s
   {
     /* Network connection parameters */

     uint16_t          d_port;      /* Port to listen on (in network byte order) */
     sa_family_t       d_family;    /* Address family (AF_INET or AF_INET6) */

     /* Spawned task characteristics */

     uint8_t           t_priority;  /* Execution priority of the spawned task */
     size_t            t_stacksize; /* Stack size (in bytes) for the spawned task */

   #ifndef CONFIG_BUILD_KERNEL
     main_t            t_entry;     /* Task entrypoint function (flat / protected mode) */
   #endif

   #ifdef CONFIG_LIBC_EXECFUNCS
     FAR const char   *t_path;      /* Binary path to execute (kernel / binfmt mode) */
   #endif

     FAR char * const *t_argv;      /* Argument vector passed to the spawned task */
   };

Functions
---------

``telnetd_daemon()``
~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   int telnetd_daemon(FAR const struct telnetd_config_s *config);

**Description**:
Runs the Telnet daemon loop. The function sets up the listening socket, binds to the
designated port, and continuously accepts incoming client connections. For each accepted
connection, it instantiates a ``/dev/telnetN`` character driver, redirects standard I/O,
and spawns a child task running the specified entrypoint or binary.

Because this function runs an infinite loop accepting connections, it should normally be
invoked inside a dedicated background task (e.g., started with ``task_create()``).

**Parameters**:

- ``config``: Pointer to a ``struct telnetd_config_s`` instance defining the listener port,
  address family, child task priority, stack size, and entry point. The configuration
  structure may reside on the caller's stack because it is copied internally during startup.

**Return Value**:
Does not return under normal operation. If a fatal initialization error occurs (e.g., failure
to create or bind the listening socket), a negated ``errno`` value is returned (such as
``-EINVAL``, ``-EADDRINUSE``, or ``-ENOMEM``).

Application Examples
====================

Custom Standalone Telnet Daemon
-------------------------------

The following complete, runnable example demonstrates how to configure and start a
custom Telnet daemon in an application. The daemon listens on TCP port 2323 and spawns
an interactive session whenever a client connects. It demonstrates both in-memory
entrypoint configuration for flat and protected builds (``t_entry``) and binary executable
path configuration for kernel builds (``t_path``):

.. code-block:: c

   #include <nuttx/config.h>

   #include <stdio.h>
   #include <stdlib.h>
   #include <string.h>
   #include <unistd.h>
   #include <sched.h>
   #include <errno.h>
   #include <arpa/inet.h>

   #include <netutils/telnetd.h>

   #define TELNET_PORT           2323
   #define TELNET_DAEMON_PRIO    100
   #define TELNET_DAEMON_STACK   2048
   #define TELNET_CLIENT_PRIO    100
   #define TELNET_CLIENT_STACK   2048

   /****************************************************************************
    * Name: custom_shell_session
    *
    * Description:
    *   Interactive session spawned per client. Standard I/O (stdin, stdout,
    *   stderr) is already mapped to the Telnet network connection.
    ****************************************************************************/

   static int custom_shell_session(int argc, FAR char *argv[])
   {
     char line[64];

     printf("\n========================================\n");
     printf(" Welcome to NuttX Custom Telnet Console \n");
     printf(" Type 'help', 'ping', or 'exit' to quit \n");
     printf("========================================\n\n");

     for (; ; )
       {
         printf("telnet> ");
         fflush(stdout);

         if (fgets(line, sizeof(line), stdin) == NULL)
           {
             break; /* Client closed connection */
           }

         /* Trim trailing newline and carriage return characters */

         line[strcspn(line, "\r\n")] = '\0';

         if (strcmp(line, "exit") == 0)
           {
             printf("Disconnecting session...\n");
             break;
           }
         else if (strcmp(line, "ping") == 0)
           {
             printf("pong!\n");
           }
         else if (strcmp(line, "help") == 0)
           {
             printf("Available commands: help, ping, exit\n");
           }
         else if (strlen(line) > 0)
           {
             printf("Unknown command: '%s'\n", line);
           }
       }

     printf("Session closed. Goodbye!\n");
     return OK;
   }

   /****************************************************************************
    * Name: telnet_listener_task
    *
    * Description:
    *   Background listener task running the telnetd daemon loop.
    ****************************************************************************/

   static int telnet_listener_task(int argc, FAR char *argv[])
   {
     struct telnetd_config_s config;
     int ret;

     memset(&config, 0, sizeof(config));
     config.d_port       = HTONS(TELNET_PORT);
     config.d_family     = AF_INET;
     config.t_priority   = TELNET_CLIENT_PRIO;
     config.t_stacksize  = TELNET_CLIENT_STACK;

   #ifndef CONFIG_BUILD_KERNEL
     /* In flat and protected builds, provide the in-memory entrypoint function */

     config.t_entry      = custom_shell_session;
     config.t_argv       = NULL;
   #else
     /* In kernel builds (CONFIG_BUILD_KERNEL), specify the binary executable path
      * to be spawned via posix_spawn() (e.g., located on ROMFS or binfs).
      */

     config.t_path       = "/system/bin/custom_shell";
     config.t_argv       = NULL;
   #endif

     printf("Starting Telnet daemon on port %d...\n", TELNET_PORT);

     ret = telnetd_daemon(&config);
     if (ret < 0)
       {
         fprintf(stderr, "telnetd_daemon failed: %d\n", ret);
         return ret;
       }

     return OK;
   }

   /****************************************************************************
    * Name: telnet_server_start
    *
    * Description:
    *   Launches the Telnet listener in a non-blocking background task.
    ****************************************************************************/

   int telnet_server_start(void)
   {
     int pid;

     pid = task_create("custom_telnetd",
                       TELNET_DAEMON_PRIO,
                       TELNET_DAEMON_STACK,
                       telnet_listener_task,
                       NULL);
     if (pid < 0)
       {
         fprintf(stderr, "Failed to start listener task: %d\n", errno);
         return -errno;
       }

     printf("Telnet daemon task created with PID %d\n", pid);
     return OK;
   }

Starting the NuttShell (NSH) Telnet Console
-------------------------------------------

To run the standard NuttShell command prompt over Telnet, configure NSH to launch its
built-in Telnet service:

In your board's ``defconfig``:

.. code-block:: make

   CONFIG_NETUTILS_NETLIB=y
   CONFIG_NETUTILS_TELNETD=y
   CONFIG_NSH_TELNET=y
   CONFIG_NSH_TELNETD_PORT=23

When the system boots and initializes the network interface (either statically or via
DHCP), NSH starts the Telnet listener automatically, or you can invoke it from the serial
console:

.. code-block:: bash

   nsh> telnetd &

Client Connection Walkthrough
=============================

Connecting from Linux or macOS Terminal
---------------------------------------

From a development workstation on the same local subnet, connect using the standard
``telnet`` client.

.. note::

   **macOS Prerequisite**: Starting with macOS High Sierra, the ``telnet`` client utility
   is no longer pre-installed. It can be installed via `Homebrew <https://brew.sh>`__:

   .. code-block:: bash

      $ brew install telnet

   Alternatively, standard ``nc`` (netcat) can be used to establish raw TCP terminal connections:

   .. code-block:: bash

      $ nc 192.168.1.50 23

Connect to the NuttX device:

.. code-block:: bash

   $ telnet 192.168.1.50 23
   Trying 192.168.1.50...
   Connected to 192.168.1.50.
   Escape character is '^]'.

   NuttShell (NSH) NuttX-13.1.0
   nsh> help
   help usage:  help [-v] [<cmd>]

     .         cat       dd        free      kill      mkdir     ps        rmdir     time      usleep    
     [         cd        df        get       ls        mount     pwd       sleep     umount    xd        
     ?         cp        echo      help      mkrd      mv        rm        test      uname     

   nsh> ps
     PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK            STACK   USED  FILLED COMMAND
       0     0   0 FIFO     Kthread   - Ready              0000000000000000 0002048 0000720  35.1% Idle Task
       1     1 100 RR       Task      - Waiting  Semaphore 0000000000000000 0003072 0001248  40.6% init_main
       2     2 100 RR       Task      - Waiting  Socket    0000000000000000 0002048 0000840  41.0% telnetd
       3     3 100 RR       Task      - Running            0000000000000000 0002048 0001128  55.0% Telnet session
   nsh> exit
   Connection closed by foreign host.

Connecting from Windows (PowerShell / Command Prompt)
-----------------------------------------------------

On Windows systems, enable the optional Telnet Client or use PuTTY:

.. code-block:: powershell

   # Using PuTTY:
   putty.exe -telnet 192.168.1.50 23

   # Or using standard Windows Telnet (if enabled):
   telnet 192.168.1.50 23

Troubleshooting and Best Practices
==================================

.. list-table::
   :widths: 25 35 40
   :header-rows: 1

   * - Issue / Symptom
     - Probable Cause
     - Recommended Solution
   * - Sluggish typing response or high latency per character
     - Standard I/O buffering disabled or lack of TCP transmit bundling.
     - Enable ``CONFIG_STDIO_LINEBUFFER=y`` and set ``CONFIG_STDIO_BUFFER_SIZE >= 64``
       in ``defconfig``.
   * - Connection refused on port 23
     - The daemon is not running or the interface has no IP address assigned.
     - Verify network connectivity with ``ifconfig`` and ensure ``telnetd`` was started.
   * - Remote client disconnects immediately after connecting
     - Out of memory or stack overflow during task spawning.
     - Increase ``CONFIG_NSH_TELNETD_CLIENTSTACKSIZE`` (or custom ``t_stacksize``) to
       at least 2048 or 4096 bytes.
   * - Maximum clients reached
     - Concurrent connections exceed ``CONFIG_TELNET_MAXLCLIENTS``.
     - Increase ``CONFIG_TELNET_MAXLCLIENTS`` (default is 8) in your configuration.
   * - Client cannot escape or disconnect
     - Telnet escape command not recognized by client.
     - Use standard escape sequence: press ``Ctrl+]``, then type ``quit`` and press Enter.
