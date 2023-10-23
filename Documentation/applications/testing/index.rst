=======
Testing
=======

The ``apps/testing`` directory is used to build NuttX-specific tests and to
include external testing frameworks.

There is overlap between what you will find in ``apps/examples`` and
``apps/testing`` in the sense that there are also tests in ``apps/examples`` as
well. Those tests, however, can also be used to illustrate usage of a NuttX
feature. Most of the tests in ``apps/testing``, on the other hand, are pure tests
with little value as usage examples.

.. toctree::
   :glob:
   :maxdepth: 3
   :titlesonly:
   
   */*

- atomic - "Test atomic" testing
- batterydump - Battery dump for test
- cmocka - libcmocka
- cpuload - cpuload test
- crypto - crypto test
- drivertest - vela cmocka driver test
- fatutf8 - FAT UTF8 test
- fdsantest - vela cmocka fdsan test
- getprime - getprime example
- iozone - IOzone, filesystem benchmark tool
- irtest - IR driver test
- ltp - Linux Test Project
- memtester - utils_memtester
- monkey - Monkey test
- nist-sts - NIST Statistical Test Suite
- osperf - System performance profiling
- scanftest - sscanf() test
- sensortest - Sensor driver test
- setest - Secure Element driver test
- superpi - SuperPI test
- uclibcxx_test - uclibcxx test
