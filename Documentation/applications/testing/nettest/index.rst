================
Net Test Module
================

The ``apps/testing/nettest`` directory is used to build testcases related
to the network protocol stack. You can use it on any device with network
protocol stack capabilities.

Directory Structure
===================

::

    ├──tcp
    │  ├── test_tcp_connect_ipv4.c  # The testcase of TCP connect
    │  ├── ...
    │  ├── test_tcp_common.c        # TCP testcase common function
    │  ├── test_tcp.h               # TCP testcases declaration
    │  └── test_tcp.c               # TCP testcase execution entry
    ├──udp
    │  ├── ...
    │  ├── test_udp.h               # UDP testcases declaration
    │  └── test_udp.c               # UDP testcase execution entry
    ├── ...
    ├──utils                        # Utils for testcases
    │  ├── ...
    │  ├── nettest_netdump.c
    │  ├── nettest_tcpserver.c
    │  └── utils.h
    ├── CmakeLists.txt
    ├── Kconfig                     # Enable module testcases
    ├── Make.defs
    └── Makefile

How to Build
============

Firstly, all testcases in this directory rely on the cmocka framework,
so please ensure that the following cmocka configuration is turned on:

- ``CONFIG_ALLOW_MIT_COMPONENTS=y``
- ``CONFIG_LIBC_REGEX=y``
- ``CONFIG_TESTING_CMOCKA=y``

Then, open the network part testcase configuration:

- ``CONFIG_TESTING_NET_TEST=y``

Finally, you can choose which protocol testcase to enable through
configuration. Some protocol configurations depend on other network
configurations; you can view the dependencies in the ``Kconfig`` file. Take
the TCP testcase as an example:

- ``CONFIG_NET_TCPBACKLOG=y``  – TCP testcase compilation dependencies
- ``CONFIG_NET_TCP=y``         – TCP testcase compilation dependencies
- ``CONFIG_TESTING_NET_TCP=y`` – TCP testcase configuration items; for other
  protocols, just replace the trailing protocol name

How to Run
==========

After the testcase is built, you can run the testcase at the command line through
the following commands:

- ``cmocka --test test_tcp_connect_ipv4`` – Run a single testcase

- ``cmocka_net_tcp`` – Run TCP testcase (for other protocols, just replace
  the trailing protocol name)

In addition, when you run ``cmocka``, the above commands will also be included.

How to Add Testcases
====================

If you want to add testcases, please follow these steps:

1. Add the testcase source file to the corresponding directory. Please follow
   the following naming rules for source files:

   ``test_<protocol>_<function description>_<additional description>.c``

   For example: ``test_tcp_connect_ipv4.c``

   Test case names are the same as source files, such as ``test_tcp_connect_ipv4()``.

2. Add the testcase source file to the ``CMakeLists.txt`` file.

3. Add the testcase source file to the ``Makefile`` file.

4. Add the testcase to the corresponding declaration file and execute entry file.
   For example, ``test_tcp_connect_ipv4()`` needs to be added to ``test_tcp.h``
   and ``test_tcp.c``. If you have a separate setup function, also add it to
   the above two files.