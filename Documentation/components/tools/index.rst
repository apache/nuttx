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

``tools/mkpasswd`` is a C host tool (compiled from ``tools/mkpasswd.c``) that
generates a single ``/etc/passwd`` entry at build time.  It is invoked
automatically by the ROMFS build step when
``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y`` is set.

Why build-time generation?
~~~~~~~~~~~~~~~~~~~~~~~~~~

Shipping a hard-coded default password in firmware is a well-known security
weakness (CWE-798).  By generating the ``/etc/passwd`` entry from a
user-supplied plaintext password at build time, each firmware image carries
unique credentials.  The build will fail if the password is left empty,
preventing accidental deployments with no credentials.

How it works
~~~~~~~~~~~~

1. The host tool reads the plaintext password from
   ``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD``.
2. The password is hashed using the Tiny Encryption Algorithm (TEA) — the
   same implementation used at runtime in
   ``libs/libc/misc/lib_tea_encrypt.c`` — with custom base64 encoding
   matching ``apps/fsutils/passwd/passwd_encrypt.c``.
3. The resulting hashed entry is written to
   ``etctmp/<mountpoint>/passwd`` and then embedded into the ROMFS image.
4. The **plaintext password is never stored in the firmware image**.

Kconfig options
~~~~~~~~~~~~~~~

Enable the feature and configure credentials via ``make menuconfig``:

.. code:: kconfig

   CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y
   CONFIG_BOARD_ETC_ROMFS_PASSWD_USER="admin"         # default: admin
   CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD="<secret>"  # required, build fails if empty
   CONFIG_BOARD_ETC_ROMFS_PASSWD_UID=0
   CONFIG_BOARD_ETC_ROMFS_PASSWD_GID=0
   CONFIG_BOARD_ETC_ROMFS_PASSWD_HOME="/"

The TEA encryption keys can be changed from their defaults via
``CONFIG_FSUTILS_PASSWD_KEY1..4``.

``/etc/passwd`` file format
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: text

   user:x:uid:gid:home

Where:

* ``user`` — user name
* ``x`` — TEA-hashed, base64-encoded password
* ``uid`` — numeric user ID
* ``gid`` — numeric group ID
* ``home`` — login directory

Verifying the generated entry
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After enabling ``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE`` and setting a
password, rebuild and verify:

1. **Configure and build**:

   .. code:: console

      $ make menuconfig   # enable BOARD_ETC_ROMFS_PASSWD_ENABLE and set password
      $ make

2. **Inspect the generated passwd line** (written to the board build tree):

   .. code:: console

      $ cat boards/<arch>/<chip>/<board>/src/etctmp/etc/passwd
      admin:s1IZjGjjmo/x8u5m5uY2jB:0:0:/

3. **Verify the plaintext is absent from firmware**:

   .. code:: console

      $ grep <your-password> boards/<arch>/<chip>/<board>/src/etctmp.c
      # must print nothing
