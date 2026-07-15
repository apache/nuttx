==========
Host Tools
==========

This page discusses the ``tools/`` directory containing miscellaneous scripts
and host C programs that are important parts of the NuttX build system:

.. toctree::
   :caption: Tool documentation pages
   :maxdepth: 1
   :glob:

   ./*

.. _mkpasswd_autogen:

mkpasswd: Build-time ``/etc/passwd`` generation
===============================================

``tools/mkpasswd`` (``tools/mkpasswd.c``) writes one ``/etc/passwd`` line at
build time when ``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y``.

Quick start
~~~~~~~~~~~

.. code:: kconfig

   CONFIG_ETC_ROMFS=y
   CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y
   CONFIG_FSUTILS_PASSWD=y
   CONFIG_FSUTILS_PASSWD_READONLY=y
   CONFIG_NSH_CONSOLE_LOGIN=y

Set **Board Selection → Auto-generate /etc/passwd at build time → Root password**
in menuconfig or at the ``make`` prompt, then build and log in as ``root``.

See :doc:`/applications/nsh/login` for NSH login details.

Build flow
~~~~~~~~~~

**Makefile builds** use ``tools/board_romfs_mkpasswd.sh``, which validates the
password (``tools/promptpasswd.sh`` if needed) and invokes ``tools/mkpasswd``.

**CMake builds** invoke ``tools/mkpasswd`` directly; set
``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD`` in ``.config`` (configure fails if
it is missing).

In both cases:

1. ``tools/mkpasswd`` hashes with PBKDF2-HMAC-SHA256 (same algorithm as
   ``apps/fsutils/passwd``, which uses kernel cryptodev at runtime).
2. The hash is written to ``etctmp/.../passwd`` and embedded in ROMFS.

The plaintext password is used on the host during ``make`` only.  On an
interactive terminal, ``make`` prompts for the root password (and confirmation)
when it is not already configured; non-interactive builds must set
``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD`` or ``NUTTX_ROMFS_PASSWD_PASSWORD``.

Build-time credentials (CI / automation)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD`` is **not** written to defconfig by
``make savedefconfig``.  For scripted or CI builds, export the root password
before ``configure.sh`` or ``make``:

.. code:: bash

   export NUTTX_ROMFS_PASSWD_PASSWORD="your-password-here"

``tools/update_romfs_password.sh`` copies this into ``.config`` when ROMFS
passwd generation is enabled and the password field is still empty.  NuttX
CI sets ``NuttXSim1!`` for the ``sim/login`` configuration (a documented
sim-only test credential, not a product secret).  The password must meet the
complexity rules below.

Password rules
~~~~~~~~~~~~~~

Minimum 8 characters; at least one uppercase, lowercase, digit, and special
character from ``!@#$%^&*()_+-=[]{}|;:,.<>?``.

Hash format
~~~~~~~~~~~

::

   $pbkdf2-sha256$<iterations>$<base64url-salt>$<base64url-hash>

Example::

   root:$pbkdf2-sha256$10000$zhoo4phwEzyNFUAkB7asfw$P8qsjd9RQmZBLfM5zugiJeE5gKjI-CmTxyaVyOX2mE4

**Breaking change:** TEA-encoded entries are not compatible.  Regenerate with
``mkpasswd`` or NSH ``passwd`` / ``useradd``.

Kconfig
~~~~~~~

.. code:: kconfig

   CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y
   CONFIG_CRYPTO_CRYPTODEV=y
   CONFIG_BOARD_ETC_ROMFS_PASSWD_USER="root"
   CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD="<secret>"
   CONFIG_FSUTILS_PASSWD_PBKDF2_ITERATIONS=10000

``make savedefconfig`` omits ``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD`` and
``CONFIG_FSUTILS_PASSWD_PBKDF2_ITERATIONS`` to avoid leaking credentials.

Host files
~~~~~~~~~~

* ``tools/mkpasswd.c`` - PBKDF2 hash generation
* ``tools/promptpasswd.sh`` - interactive root password prompt, validation,
  and confirmation
* ``tools/board_romfs_mkpasswd.sh`` - Makefile ROMFS build wrapper
