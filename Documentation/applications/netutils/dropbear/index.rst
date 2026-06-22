=====================================
``dropbear`` SSH Server
=====================================

`Dropbear <https://matt.ucc.asn.au/dropbear/dropbear.html>`__ is a lightweight
SSH server well-suited for embedded environments. The NuttX port exposes an NSH
session over an encrypted SSH connection, enabling remote access to NuttX-based
hardware over a network.

Overview
--------

The port runs Dropbear as a persistent NuttX task, accepting successive SSH
connections without restarting. It can be used to open interactive NSH sessions
remotely, inspect system state, run commands, or integrate into automated
workflows that require secure shell access to an embedded target.

All cryptographic operations are routed through NuttX's native crypto stack,
replacing the bundled libtomcrypt modules and leveraging any available hardware
acceleration.

Prerequisites
-------------

Dropbear requires the board to have a reachable IP address. The underlying
network interface can be any transport supported by NuttX — Wi-Fi station mode,
Ethernet, or a point-to-point link — as long as the interface is up and has an
assigned address visible in ``ifconfig``::

    nsh> ifconfig
    wlan0   Link encap:Ethernet HWaddr 84:f7:03:xx:xx:xx at RUNNING mtu 1504
            inet addr:192.168.1.xx DRaddr:192.168.1.x Mask:255.255.255.0

The SSH client must be able to reach that address over TCP port 22 (or whichever
port is configured via ``CONFIG_NETUTILS_DROPBEAR_PORT``).

A persistent writable filesystem (e.g. SPIFFS or LittleFS) must be mounted and
accessible at the path used by ``CONFIG_NETUTILS_DROPBEAR_HOSTKEY_PATH`` and
``CONFIG_FSUTILS_PASSWD_PATH``. These files are created automatically on the
first run, but the filesystem must already be mounted.

Enabling Dropbear
-----------------

Enable the following options in ``menuconfig``::

  CONFIG_NETUTILS_DROPBEAR=y

Key configuration options:

- ``CONFIG_NETUTILS_DROPBEAR_PORT`` — TCP port to listen on (default: ``22``).
- ``CONFIG_NETUTILS_DROPBEAR_HOSTKEY_PATH`` — path where the ECDSA host key is
  stored (default: ``/data/dropbear_ecdsa_host_key``).
- ``CONFIG_NETUTILS_DROPBEAR_STACKSIZE`` — task stack size. A minimum of
  ``65536`` bytes is recommended;
- ``CONFIG_NETUTILS_DROPBEAR_PRIORITY`` — task priority.

The user database is read from the path configured by
``CONFIG_FSUTILS_PASSWD_PATH`` (default ``/data/passwd``).

Usage
-----

Once the network is up, the daemon starts automatically. The first run
generates an ECDSA host key and writes it to ``CONFIG_NETUTILS_DROPBEAR_HOSTKEY_PATH``.
Add a user account from the NSH console before connecting::

    nsh> useradd <username> <password>

Check the board's IP address::

    nsh> ifconfig

Then open an SSH session from a host on the same network::

    $ ssh <username>@<ip_address>
    <username>@<ip_address>'s password:
    nsh>

Notes
-----

- The host key and the user database persist across reboots when stored on a
  non-volatile partition (e.g. SPIFFS mounted at ``/data``).
- Only password authentication is supported; public-key authentication is
  disabled.
- Only ECDSA P-256 host keys are supported.
- Enabled ciphers: ``chacha20-poly1305@openssh.com`` and ``aes128-ctr``.
- Port forwarding, agent forwarding, and X11 forwarding are disabled.
- Multiple clients can connect sequentially; the daemon re-enters the accept
  loop after each session ends.

Troubleshooting
---------------

**Lost password / cannot log in**

The user database is stored in the file pointed to by ``CONFIG_FSUTILS_PASSWD_PATH``
on the persistent partition (e.g. ``/data/passwd`` on SPIFFS). If the password
is lost and the board can still be reached over a serial console, remove the
file and recreate the account::

    nsh> rm /data/passwd
    nsh> useradd <username> <new_password>

If the serial console is also inaccessible, the only recovery path is to erase
the persistent partition (or the full flash) and reflash the firmware.

**Host key mismatch after reflashing**

When the persistent partition is erased as part of a reflash, Dropbear generates
a new host key on the next boot. The SSH client on the host machine will then
refuse to connect because the key no longer matches the entry stored in
``~/.ssh/known_hosts``, printing a warning such as
``WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED``.

Remove the stale entry with::

    $ ssh-keygen -R <ip_address>

Then reconnect normally. The client will prompt to accept the new host key.

**Connection refused**

Check that the board has obtained an IP address and that the Dropbear task is
running::

    nsh> ifconfig
    nsh> ps
