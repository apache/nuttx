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

mkpasswd — Build-time ``/etc/passwd`` Generation
-------------------------------------------------

``tools/mkpasswd`` (``tools/mkpasswd.c``) is a host program that writes a
single ``/etc/passwd`` entry with a TEA-encrypted password hash.  The
plaintext password is **not** stored in the firmware image.

When ``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y``, the build invokes
``mkpasswd`` automatically from ``boards/Board.mk`` (Make) or
``cmake/nuttx_add_romfs.cmake`` (CMake).  You normally configure the
password and keys in menuconfig; you do not run ``mkpasswd`` by hand.

Prerequisites
~~~~~~~~~~~~~

Enable all of the following (via ``make menuconfig``):

* **Board Selection** → **Auto-generate /etc/passwd at build time**
  (``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE``)
* **Application Configuration** → **NSH Library** → **Console Login**
  (``CONFIG_NSH_CONSOLE_LOGIN``) with verification method **Encrypted
  password file** (``CONFIG_NSH_LOGIN_PASSWD``)
* **Application Configuration** → **File System Utilities** → **Password file
  support** (``CONFIG_FSUTILS_PASSWD``)

Setup workflow
~~~~~~~~~~~~~~

1. ``tools/configure.sh <board>:<config>``
2. ``make menuconfig``:

   * **Board Selection** → Auto-generate /etc/passwd

     * **Admin password** — required, at least 8 characters.  There is no
       Kconfig default (the legacy ``Administrator`` password is rejected).
     * **Generate random TEA encryption keys automatically** — recommended;
       or disable this and set keys manually (see below).

   * Confirm NSH uses **Encrypted password file** verification (above).

3. ``make`` — on the first build, ``tools/passwd_keys.mk`` validates the
   password and keys, may generate TEA keys, then ``mkpasswd`` runs before
   ``config.h`` is created so the hash and firmware always agree.

Build-time credentials (CI / automation)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

These values are **not** written to defconfig by ``make savedefconfig``:

* ``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD``
* ``CONFIG_FSUTILS_PASSWD_KEY1`` … ``CONFIG_FSUTILS_PASSWD_KEY4``

For scripted or CI builds, export the admin password before ``configure.sh``
or ``make``:

.. code:: bash

   export NUTTX_ROMFS_PASSWD_PASSWORD="your-password-here"

``tools/update_romfs_password.sh`` copies this into ``.config`` when ROMFS
passwd generation is enabled and the password field is still empty.  NuttX
CI sets ``NuttXSimLogin1`` for the ``sim/login`` configuration (a documented
sim-only test credential, not a product secret).

TEA encryption keys
~~~~~~~~~~~~~~~~~~~

``mkpasswd`` and the firmware must use the **same** four 32-bit key words
(``CONFIG_FSUTILS_PASSWD_KEY1`` … ``KEY4``).  Choose one approach:

**Random generation** (``CONFIG_BOARD_ETC_ROMFS_PASSWD_RANDOMIZE_KEYS=y``):

  On the first ``make`` when keys are missing or still placeholders,
  ``tools/gen_passwd_keys.sh`` writes random values to ``.config``.  A
  build warning explains where to view or change them in menuconfig.  Key
  values are never printed in the build log.

**Manual keys** (``CONFIG_BOARD_ETC_ROMFS_PASSWD_RANDOMIZE_KEYS`` disabled):

  Set ``CONFIG_FSUTILS_PASSWD_KEY1`` … ``KEY4`` under **Application
  Configuration** → **File System Utilities** → **Password file support**.
  Each must be a unique non-zero value.  The legacy published defaults
  ``0x12345678`` / ``0x9abcdef0`` are rejected.

If random generation is enabled but keys are already present in ``.config``,
they are **not** regenerated (subsequent builds stay consistent).

How the build enforces security
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+---------------------------+-----------------------------------------------+
| Check                     | Where                                         |
+===========================+===============================================+
| Password set, min 8 chars | ``tools/passwd_keys.mk`` (before ``config.h``)|
| Not ``Administrator``     | ``tools/mkpasswd.c`` (last line of defence)   |
| TEA keys configured       | ``tools/check_passwd_keys.sh``                |
| Not legacy default keys   | ``check_passwd_keys.sh`` + ``mkpasswd.c``     |
+---------------------------+-----------------------------------------------+

Standalone ``mkpasswd`` (advanced)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For debugging only, you may run the host binary directly.  You must pass all
four ``--key`` options with non-default values and a password of at least 8
characters:

.. code:: bash

   ./tools/mkpasswd --user root --password 'my-secret' \
     --key1 0x11111111 --key2 0x22222222 \
     --key3 0x33333333 --key4 0x44444444

Kconfig summary
~~~~~~~~~~~~~~~~~

Example ``.config`` fragment (password and keys normally **not** in defconfig):

.. code:: kconfig

   CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y
   CONFIG_BOARD_ETC_ROMFS_PASSWD_RANDOMIZE_KEYS=y
   CONFIG_BOARD_ETC_ROMFS_PASSWD_USER="root"
   CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD="<set in menuconfig or env>"
   CONFIG_NSH_CONSOLE_LOGIN=y
   CONFIG_NSH_LOGIN_PASSWD=y
   CONFIG_FSUTILS_PASSWD=y

``/etc/passwd`` file format
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: text

   user:encrypted_hash:uid:gid:home

Notes on ``savedefconfig``
~~~~~~~~~~~~~~~~~~~~~~~~~~

``make savedefconfig`` omits the password and ``KEY1``–``KEY4`` from the
generated defconfig on purpose.  ``CONFIG_BOARD_ETC_ROMFS_PASSWD_RANDOMIZE_KEYS``
may remain in defconfig (policy, not a secret).  Do not commit passwords or
key values to version control.
