===========
Shell Login
===========

Enabling Shell Logins
=====================

NuttShell sessions can be protected by requiring that the user supply
username and password credentials at the beginning of the session.
Logins can be enabled for standard USB or serial consoles with::

  CONFIG_NSH_CONSOLE_LOGIN=y

Logins for Telnet sessions can be enabled separately with::

  CONFIG_NSH_TELNET_LOGIN=y

Logins can be enabled for either or both session types. On a successful
login, the user will have access to the NSH session::

  login: root
  password:
  User Logged-in!

  NuttShell (NSH)
  nsh>

ROMFS password file (recommended)
==================================

Boards with ROMFS ``/etc`` should auto-generate ``/etc/passwd`` at build time::

  CONFIG_ETC_ROMFS=y
  CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y
  CONFIG_CRYPTO_CRYPTODEV=y
  CONFIG_FSUTILS_PASSWD=y
  CONFIG_FSUTILS_PASSWD_READONLY=y
  CONFIG_NSH_CONSOLE_LOGIN=y

``CONFIG_FSUTILS_PASSWD`` depends on ``CONFIG_CRYPTO_CRYPTODEV``.  The
``sim:login`` defconfig also enables software cryptodev
(``CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_CRYPTO`` and ``CONFIG_CRYPTO_SW_AES``).

Set **Board Selection → Auto-generate /etc/passwd at build time → Root password**
in menuconfig, in a local defconfig, or at the ``make`` prompt
(:ref:`mkpasswd_autogen`).  Only a PBKDF2-HMAC-SHA256 hash is stored in flash.

When ``CONFIG_NSH_LOGIN_SETUID`` is enabled (the default when
``CONFIG_SCHED_USER_IDENTITY`` is selected), NSH looks up the
authenticated user name in the passwd database and sets the session
identity after a successful login. See also ```id`` <#cmdid>`__,
```su`` <#cmdsu>`__, and ```whoami`` <#cmdwhoami>`__.

Session Identity and Prompt
===========================

When ``CONFIG_SCHED_USER_IDENTITY`` is enabled, NSH tracks the calling
task's real and effective UID/GID for the shell session. File
permission checks use the **effective** identity.

**Prompt markers.** After login or a successful ``su``, NSH updates the
command prompt to show the effective privilege level of the session:

===================  ================================================
Prompt marker        Meaning
===================  ================================================
``#``                Effective UID is zero (root session)
``$``                Effective UID is non-zero (non-root session)
===================  ================================================

The marker replaces the character before the closing ``>`` in the
default prompt (for example, ``nsh>`` becomes ``nsh#`` or ``nsh$``), or
is appended when the configured prompt does not end with ``>``. The
prompt is refreshed by ``nsh_update_prompt()`` after ``su`` and after
login when ``CONFIG_NSH_LOGIN_SETUID`` is enabled.

**Flat builds.** NSH may retain a real UID of zero while only the
effective UID/GID are changed. In that case ``id`` can report
``uid=0 euid=1000`` after logging in as a normal user. Permission
checks and the prompt marker follow the effective identity.

**Example**::

  login: testuser
  password:
  User Logged-in!

  NuttShell (NSH)
  nsh$ id
  uid=0 euid=1000 gid=0 egid=1000
  nsh$ whoami
  testuser
  nsh$ su root
  Password:
  nsh# id
  uid=0 euid=0 gid=0 egid=0
  nsh# whoami
  root

After each failed login attempt, a delay can be set up. The purpose of
this delay is to discourage attempts to crack the password by brute
force. That delay is configured with::

  CONFIG_NSH_LOGIN_FAILDELAY=0

This setting provides the login failure delay in units of milliseconds.
The system will pause this amount of time after each failed login
attempt. After a certain number of failed login attempts, the session
will be closed. That number is controlled by::

  CONFIG_NSH_LOGIN_FAILCOUNT=3

.. _nsh_login_verification:

Verification of Credentials
===========================

There are two ways to verify credentials at login:

.. list-table:: NSH credential verification methods
   :header-rows: 1
   :widths: 20 15 65

   * - Method
     - Kconfig
     - Summary
   * - Password file (recommended)
     - ``CONFIG_NSH_LOGIN_PASSWD=y``
     - Verifies against ``/etc/passwd`` using PBKDF2-HMAC-SHA256 hashes.
       Use with ``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE`` for ROMFS boards.
   * - Platform callback
     - ``CONFIG_NSH_LOGIN_PLATFORM=y``
     - Board-specific ``platform_user_verify()`` function.

When ``CONFIG_FSUTILS_PASSWD=y`` is enabled, NSH defaults to
``CONFIG_NSH_LOGIN_PASSWD=y`` automatically.  Console and telnet login
require ``CONFIG_FSUTILS_PASSWD``; the removed fixed-login options
``CONFIG_NSH_LOGIN_FIXED`` and ``CONFIG_NSH_LOGIN_PASSWORD`` are rejected
at build time if still present in ``.config``.

  #. **Platform-specific verification.**  NSH calls ``platform_user_verify()``
     when ``CONFIG_NSH_LOGIN_PLATFORM=y``.  Prototype in ``apps/include/nsh.h``.

Password Files
==============

When ``CONFIG_NSH_LOGIN_PASSWD=y`` is selected, NSH reads user names and
password hashes from a passwd file (default ``/etc/passwd``).  Enable the
file-access layer with::

  CONFIG_FSUTILS_PASSWD=y

And this determines the location of the password file in a mounted
volume::

  CONFIG_FSUTILS_PASSWD_PATH="/etc/passwd"

``/etc/passwd`` is a *standard* location, but you will need to locate
the password where ever you have a mounted volume.

The password file can be a fixed list of users in a ROMFS file system or
a modifiable list maintained in a file in some writable file system. If
the password file lies in a read-only file system like ROMFS, then you
should also indicate that the password file is read-only.

  CONFIG_FSUTILS_PASSWD_READONLY=y

If the password file is writable, then additional NSH commands will be
enabled to modify the password file: ```useradd`` <#cmduseradd>`__,
```userdel`` <#cmduserdel>`__, and ```passwd`` <#cmdpasswd>`__. If you
do not wish you have these commands available, then they should be
specifically disabled.

The password file logic requires a few additional settings:

  #. **Kernel cryptodev**: ``CONFIG_FSUTILS_PASSWD`` requires
     ``CONFIG_CRYPTO_CRYPTODEV`` (PBKDF2 via ``/dev/crypto`` at runtime).

  #. **I/O buffer size**: size of the dynamically allocated buffer used for
     file access::

       CONFIG_FSUTILS_PASSWD_IOBUFFER_SIZE=512

  #. **PBKDF2 iteration count**: applied when **setting** new passwords
     (via ``useradd``, ``passwd``, or build-time ``mkpasswd``)::

       CONFIG_FSUTILS_PASSWD_PBKDF2_ITERATIONS=10000

     Valid range: 1000 to 200000.  Higher values resist brute-force attacks
     but increase login latency on low-MHz MCUs.  The iteration count is
     stored inside each hash string, so changing this option only affects
     newly-created passwords.

  #. **Random salt source**: new salts require random bytes.  Enable a
     platform random source such as::

       CONFIG_DEV_URANDOM=y

     ``passwd_encrypt()`` uses ``getrandom()`` or ``/dev/urandom`` when
     generating salts for ``useradd`` and ``passwd``.

Password complexity rules
~~~~~~~~~~~~~~~~~~~~~~~~~

The same rules are enforced everywhere a **new** password is set:

* Build-time ``tools/mkpasswd`` (ROMFS autogen)
* NSH commands ``useradd`` and ``passwd``
* Runtime API ``passwd_encrypt()`` in ``apps/fsutils/passwd``

Rules:

* At least **8** characters
* At least one **uppercase** letter (``A`` to ``Z``)
* At least one **lowercase** letter (``a`` to ``z``)
* At least one **digit** (``0`` to ``9``)
* At least one **special** character from::

     ! @ # $ % ^ & * ( ) _ + - = [ ] { } | ; : , . < > ?

If ``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD`` is missing or does not meet
these rules, an interactive ``make`` prompts via ``tools/promptpasswd.sh``
(root password, then confirm password).  Non-interactive builds must set the
password in menuconfig or export ``NUTTX_ROMFS_PASSWD_PASSWORD`` first.

Password hash format
~~~~~~~~~~~~~~~~~~~~

Each password field uses **PBKDF2-HMAC-SHA256** in modular crypt format
(MCF).  The hash string is self-contained: it stores the iteration count
and salt, so verification does not depend on separate key material in
firmware::

  $pbkdf2-sha256$<iterations>$<base64url-salt>$<base64url-hash>

Where:

* ``<iterations>``: PBKDF2 round count (parsed at verify time)
* ``<base64url-salt>``: 16-byte random salt (RFC 4648 section 5, no padding)
* ``<base64url-hash>``: 32-byte PBKDF2-HMAC-SHA256 output (same encoding)

Example ``/etc/passwd`` line::

  root:$pbkdf2-sha256$10000$zhoo4phwEzyNFUAkB7asfw$P8qsjd9RQmZBLfM5zugiJeE5gKjI-CmTxyaVyOX2mE4:0:0:/

Full ``/etc/passwd`` record format::

  user:hash:uid:gid:home

.. note::

   **Breaking change:** this replaces the former TEA-based password storage.
   Existing TEA-encoded entries will **not** verify.  Regenerate every entry
   with ``mkpasswd``, NSH ``passwd``, or ``useradd`` after upgrading.

``passwd_verify()`` returns ``0`` on match, ``-1`` on mismatch or invalid
hash format, and a negated ``errno`` on I/O errors.

When ``CONFIG_NSH_LOGIN_PASSWD=y`` is enabled, NSH verifies logins against
the password file rather than compile-time credentials.

Notes on ``savedefconfig``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To avoid leaking credentials into board defconfigs, ``make savedefconfig``
**omits** these options from the generated defconfig:

* ``CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD``
* ``CONFIG_FSUTILS_PASSWD_PBKDF2_ITERATIONS``

Add them manually to a local defconfig after ``make savedefconfig`` if
needed for development.

Creating a Password File for a ROMFS File System
================================================

Boards with ``CONFIG_ETC_ROMFS`` can auto-generate ``/etc/passwd`` at
**build time** when ``CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y``.

Build-time flow
~~~~~~~~~~~~~~~

1. You configure the root password in menuconfig, at the interactive
   ``make`` prompt, or via ``NUTTX_ROMFS_PASSWD_PASSWORD``.
2. The build runs ``tools/mkpasswd`` (Makefile builds use
   ``tools/board_romfs_mkpasswd.sh`` as a wrapper).
3. Only the **hash** is written into the ROMFS image at ``/etc/passwd``.
4. At boot, NSH login verifies the password through kernel cryptodev
   (``/dev/crypto``).

See :ref:`mkpasswd_autogen` in :doc:`/components/tools/index` for the full
tool description, Kconfig list, and verification steps.

The following describes the **manual** approach for creating or updating a
password file when build-time autogen is **not** used.

What we want to accomplish is a ROMFS file system, mounted at ``/etc``
and containing the password file, ``passwd`` like::

  NuttShell (NSH)
  nsh> ls -Rl /etc
  /etc:
   dr-xr-xr-x       0 .
   dr-xr-xr-x       0 init.d/
   -r--r--r--      39 passwd
  /etc/init.d:
   dr-xr-xr-x       0 ..
   -r--r--r--     110 rcS
   -r--r--r--     110 rc.sysinit
  nsh>

Where ``/etc/init.d/rc.sysinit`` is the system init script and
``/etc/init.d/rcS`` is the start-up script; ``/etc/passwd`` is the
password file. Note that here we assume that you are already using a
start-up script. We can then piggyback the passwd file into the ``/etc``
file system already mounted for the NSH start up file as described
`above <#custinit>`__.

The sim/nsh configuration can be used to create a new password file, but other
configurations could also be used. That configuration already supports a
ROMFS file system, passwords, and login prompts. First make these
changes to that configuration.

  #. Disable logins:

    .. code-block:: diff

      - CONFIG_NSH_CONSOLE_LOGIN=y
      + # CONFIG_NSH_CONSOLE_LOGIN is not set
        # CONFIG_NSH_TELNET_LOGIN is not set

  #. Move the password file to a write-able file system:

    .. code-block:: diff

      - CONFIG_FSUTILS_PASSWD_PATH="/etc/passwd"
      + CONFIG_FSUTILS_PASSWD_PATH="/tmp/passwd"

  #. Make the password file modifiable

    .. code-block:: diff

      - CONFIG_FSUTILS_PASSWD_READONLY=y
      # CONFIG_FSUTILS_PASSWD_READONLY is not set

Now rebuild the simulation. No login should be required to enter the
shell and you should find the ```useradd`` <#cmduseradd>`__,
```userdel`` <#cmduserdel>`__, and ```passwd`` <#cmdpasswd>`__ commands
available in the help summary, provided that they are enabled. Make
certain that the ``useradd`` command is not disabled::

  # CONFIG_NSH_DISABLE_USERADD is not set

Use the NSH ```useradd`` <#cmduseradd>`__ command to add new uses with
new user passwords like::

  nsh> useradd <username> <password>

Do this as many times as you would like. Each time that you do this a
new entry with a hashed password will be added to the ``passwd``
file at ``/tmp/passwd``. You can see the content of the password file
like::

  nsh> cat /tmp/passwd

When you are finished, you can simply copy the ``/tmp/passwd`` content
from the ``cat`` command and paste it into an editor. Make sure to
remove any carriage returns that may have ended up on the file if you
are using Windows.

Then create/re-create the ``nsh_romfsimg.h`` file as described below.

  #. The content on the ``nsh_romfsimg.h`` header file is generated from a
     template directory structure. Create the directory structure::

      mkdir etc
      mkdir etc/init.d

     And copy your existing startup script into ``etc/init.d/`` as ``rcS``.

  #. Save your new password file in the ``etc/`` directory as ``passwd``.
     Each line must use the PBKDF2 MCF format described above.

  #. Create the new ROMFS image::

      genromfs -f romfs_img -d etc -V MyVolName

  #. Convert the ROMFS image to a C header file::

      xxd -i romfs_img >nsh_romfsimg.h

  #. Edit ``nsh_romfsimg.h``: mark both data definitions as ``const`` so
     that the data will be stored in FLASH.

There is a good example of how to do this in the NSH simulation
configuration at
`boards/sim/sim/sim/configs/nsh <https://github.com/apache/nuttx/blob/master/boards/sim/sim/sim/configs/nsh/>`__.
The ROMFS support files are provided at
`boards/sim/include <https://github.com/apache/nuttx/blob/master/boards/sim/sim/sim/include/>`__
and the :doc:`/platforms/sim/sim/boards/sim/index` page provides detailed
information about creating and modifying the ROMFS file system.
