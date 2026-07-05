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

``tools/mkpasswd`` is a host tool (``tools/mkpasswd.c``) that generates a
single ``/etc/passwd`` entry at build time. It runs automatically when
``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y``.

The plaintext password is hashed with TEA and is not stored in the firmware.
The build fails if the password is empty or uses known insecure defaults.

Setup
~~~~~

1. Run ``make menuconfig`` and configure:

   * **Board Selection** → Auto-generate /etc/passwd at build time
     * Set the admin password (required, minimum 8 characters)
     * Enable random TEA key generation, or set keys manually (see below)
   * **Application Configuration** → NSH Library → Console Login
     * Set verification method to **Encrypted password file**
   * **Application Configuration** → File System Utilities → Password file support
     * Enable password file support

2. Run ``make``.

TEA encryption keys
~~~~~~~~~~~~~~~~~~~

Keys must match between the build (``mkpasswd``) and the firmware
(``CONFIG_FSUTILS_PASSWD_KEY1..4``). Choose one option:

* **Random generation** (``CONFIG_BOARD_ETC_ROMFS_PASSWD_RANDOMIZE_KEYS=y``):
  On the first build, four keys are generated from ``/dev/urandom`` and
  written to ``.config``. Key values are not printed in the build log.
  Search ``.config`` for ``CONFIG_FSUTILS_PASSWD_KEY`` to view them.

* **Manual**: Set ``CONFIG_FSUTILS_PASSWD_KEY1..4`` under Password file
  support to unique non-zero values.

Kconfig options
~~~~~~~~~~~~~~~

.. code:: kconfig

   CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y
   CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD="<secret>"
   CONFIG_BOARD_ETC_ROMFS_PASSWD_RANDOMIZE_KEYS=y
   CONFIG_NSH_CONSOLE_LOGIN=y
   CONFIG_NSH_LOGIN_PASSWD=y
   CONFIG_FSUTILS_PASSWD=y

How it works
~~~~~~~~~~~~

1. ``tools/passwd_keys.mk`` checks the password and TEA keys before
   ``config.h`` is generated.
2. If random generation is enabled and keys are not set, ``gen_passwd_keys.sh``
   writes new keys to ``.config``.
3. ``mkpasswd`` hashes the password with the configured TEA keys.
4. The entry is embedded in the ROMFS image as ``/etc/passwd``.

``/etc/passwd`` file format
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: text

   user:encrypted_hash:uid:gid:home

Notes on ``savedefconfig``
~~~~~~~~~~~~~~~~~~~~~~~~~~

``make savedefconfig`` does not save these options:

* ``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD``
* ``CONFIG_FSUTILS_PASSWD_KEY1`` through ``CONFIG_FSUTILS_PASSWD_KEY4``

Do not copy them into a defconfig or commit them to version control.