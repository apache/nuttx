====
moxa
====

Configurations
==============

nsh
---

This configuration enables NSH with Telnet and telnet login.  Credentials
are verified against a PBKDF2-HMAC-SHA256 hash in ROMFS ``/etc/passwd``, not
a fixed plaintext password compiled into the firmware.

Before building:

1. ``./tools/configure.sh moxa:nsh``
2. Set the root password in menuconfig (**Board Selection** →
   **Auto-generate /etc/passwd at build time** → **Root password**), or
   export ``NUTTX_ROMFS_PASSWD_PASSWORD`` before ``make``.

The former fixed telnet password ``nuttx`` is no longer embedded in the
image.  For a local test build you may use (must meet complexity rules)::

  export NUTTX_ROMFS_PASSWD_PASSWORD='NuttX1!nut'
  make

See :ref:`mkpasswd_autogen` for details.
