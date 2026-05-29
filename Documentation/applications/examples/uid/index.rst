=======================
``uid`` UID/GID example
=======================

This example demonstrates how to query user and group information using the
POSIX ``getpwuid_r()``, ``getpwnam_r()``, ``getgrgid_r()``, and
``getgrnam_r()`` functions.

**Command Syntax**::

  uid -uid <uid>
  uid -uname <name>
  uid -gid <gid>
  uid -gname <name>
  uid -h

**Synopsis**. The ``uid`` command queries and displays user or group
information from the system's password and group databases.

**Options**

===================  ====================================================
``-uid <uid>``       Show user information by numeric user ID.
``-uname <name>``    Show user information by user name.
``-gid <gid>``       Show group information by numeric group ID.
``-gname <name>``    Show group information by group name.
``-h``               Show help statement with all available options.
===================  ====================================================

Query User by ID
----------------

To query user information by numeric UID::

  nsh> uid -uid 0
  Name:   root
  UID:    0
  GID:    0
  Home:   /
  Shell:  /bin/sh

The output displays:

- **Name**: The login name
- **UID**: The numeric user ID
- **GID**: The primary group ID
- **Home**: The home directory
- **Shell**: The default shell

Query User by Name
------------------

To query user information by login name::

  nsh> uid -uname root
  Name:   root
  UID:    0
  GID:    0
  Home:   /
  Shell:  /bin/sh

Query Group by ID
-----------------

To query group information by numeric GID::

  nsh> uid -gid 0
  Name:    root
  Passwd:  
  GID:     0
  Members: 

The output displays:

- **Name**: The group name
- **Passwd**: The group password (usually empty)
- **GID**: The numeric group ID
- **Members**: Comma-separated list of group members

Query Group by Name
-------------------

To query group information by group name::

  nsh> uid -gname root
  Name:    root
  Passwd:  
  GID:     0
  Members: 

Help
----

To display usage information::

  nsh> uid -h
  USAGE:
  	uid -uid <uid>    - Show user info by ID
  	uid -uname <name> - Show user info by name
  	uid -gid <gid>    - Show group info by ID
  	uid -gname <name> - Show group info by name
  	uid -h            - Show this help info

Configuration
-------------

The ``uid`` example can be enabled in the NuttX configuration::

  CONFIG_EXAMPLES_UID=y

Optional configuration:

- ``CONFIG_EXAMPLES_UID_PROGNAME`` – Program name. Default ``uid``.
- ``CONFIG_EXAMPLES_UID_PRIORITY`` – Task priority. Default ``100``.
- ``CONFIG_EXAMPLES_UID_STACKSIZE`` – Stack size. Default ``DEFAULT_TASK_STACKSIZE``.
