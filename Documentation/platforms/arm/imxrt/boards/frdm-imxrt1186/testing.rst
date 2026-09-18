========
Testing
========

This page records hardware tests performed on the FRDM-IMXRT1186.  Keep it
updated when board support, boot paths, or peripheral drivers change.  A build
result alone is not hardware proof; each entry identifies the image, boot
method, board path, result, and retained evidence.

Test environment
================

* Board: FRDM-IMXRT1186 (MIMXRT1186)
* Probe: on-board MCU-Link, serial ``WGUPS4RWFPGOT``
* Console: LPUART1 through MCU-Link VCOM at 115200 baud
* XIP flash: FlexSPI2 secure alias, image base ``0x14000000``
* XIP text: ``0x1400b000``

Results
========

.. list-table::
   :header-rows: 1
   :widths: 16 18 18 13 35

   * - Date
     - Configuration
     - Boot method
     - Result
     - Evidence
   * - 2026-08-16
     - ``nsh-xip`` plus ``apps/testing/ostest``
     - FlexSPI2 XIP after a true board power cycle
     - **PASS**
     - Full suite completed with
       ``ostest_main: Exiting with status 0`` in approximately 2 minutes.
       See the complete serial log: `ostest-xip.log <./ostest-xip.log>`__.
   * - 2026-08-16
     - ``nsh``
     - LinkServer debugger RAM load
     - **PASS**
     - ``nsh>``, ``help``, ``uname -a``, clock status and ``uptime``.
   * - 2026-08-16
     - ``usbnsh``
     - LinkServer load; USB CDC/ACM on J63
     - **PASS**
     - ``0525:a4a7`` enumeration, ``nsh>`` and ``uname -a``.
   * - 2026-08-16
     - ``netnsh``
     - LinkServer debugger RAM load
     - **PARTIAL**
     - YT8531 discovery, 1-Gbit link and DHCP lease passed; final
       bidirectional ping was not re-verified.

XIP ``ostest``
==============

The XIP test used an ``nsh-xip`` image with ``apps/testing/ostest`` enabled.
The image was programmed at FlexSPI2 offset zero, ISP control was left
undriven, and the board supply was cycled before opening the LPUART1 console.
The test was started from NSH with::

  nsh> ostest

Pass criteria were:

* execution from the XIP image after a real power cycle, without a debugger
  load;
* all ``ostest`` sections reaching their completion path;
* final ``user_main: Exiting``; and
* ``ostest_main: Exiting with status 0``.

The retained log includes the complete serial output, including harmless
pre-console bytes emitted while the UART and serial bridge settle:
`ostest-xip.log <./ostest-xip.log>`__.

Updating this record
====================

For each material board change, add or update a result above and retain the
complete serial log when runtime behaviour is part of the claim.  Record
failures and partial results explicitly.  Do not replace a power-on-reset XIP
result with a debugger-load result, and do not infer peripheral support from a
successful build alone.
