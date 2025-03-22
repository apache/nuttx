=============================
Analyzing Cortex-M Hardfaults
=============================

.. epigraph::

  > I have a build of PX4 (NuttX 6.29 with some patches) with new
  > lpc43xx chip files on 4337 chip running from FLASH (master
  > vanilla NuttX has no such problem). This gives me a hardfault
  > below if I stress NSH console (UART2) with some big output.
  >
  > I read some threads but can't get a clue how to analyze the
  > dump and where to look first:
  >
  > 1bXXX and 1aXXX addresses are FLASH. 100XXX addresses are RAM

.. code-block:: console

  Assertion failed at file:armv7-m/up_hardfault.c line: 184 task: hpwork
  sp:     10001eb4
  IRQ stack:
    base: 10001f00
    size: 000003fc
  10001ea0: 1b02d961 1b03f07e 10001eb4 10005ed8 1a0312ab 1b03f600 000000b8 1b02d961
  10001ec0: 00000010 10001f40 00000003 00000000 1a03721d 1a037209 1b02d93b 00000000
  10001ee0: 1a0371f5 00000000 00000000 00000000 00000000 00000000 1a0314a5 10005d7c
  sp:     10005e50
  User stack:
    base: 10005ed8
    size: 00000f9c
  10005e40: 00000000 00000000 00000000 1b02d587 10004900 00000000 005b8d7f 00000000
  10005e60: 1a030f2e 00000000 00000000 00001388 00000000 00000005 10001994 00000000
  10005e80: 00000000 00000000 00000000 1b02c359 00000000 00000000 00000000 004c4b40
  10005ea0: 000002ff 00000000 00000000 1a030f2f 00000000 00000000 00000000 00000000
  10005ec0: 00000000 1a030f41 00000000 1b02c2a5 00000000 00000000 ffffffff 00bdeb39
  R0: ffffffff 00000000 00000016 00000000 00000000 00000000 00000000 00000000
  R8: 100036d8 00000000 00000000 004c4b40 10001370 10005e50 1b02b20b 1b02d596
  xPSR: 41000000 BASEPRI: 00000000 CONTROL: 00000000
  EXC_RETURN: ffffffe9

This question was asked in the old Yahoo! Group for NuttX, before the
project joined the Apache Software Foundation. The old forum no longer
exists, but the thread has been archived at
`Narkive <https://nuttx.yahoogroups.narkive.com/QNbG3r5l/hardfault-help-analysing-where-to-start>`_
(third party external link).

Analyzing the Register Dump
===========================

First, in the register dump:

.. code-block:: console

  R0: ffffffff 00000000 00000016 00000000 00000000 00000000 00000000 00000000
  R8: 100036d8 00000000 00000000 004c4b40 10001370 10005e50 1b02b20b 1b02d596
  xPSR: 41000000 BASEPRI: 00000000 CONTROL: 00000000

``R15`` is the PC at the time of the crash (``1b02d596``). In order to
see where this is, I do this:

.. code-block:: console

  arm-none-eabi-objdump -d nuttx | vi -

Of course, you can use any editor you prefer. In any case, this will
provide a full assembly language listing of your FLASH content along
with complete symbolic information.

**TIP:** Not comfortable with ARM assembly language? Try the
``objdump --source`` (or just ``-S``) option. That will intermix the C
and the assembly language code so that you can see which C statements
the assembly language is implementing.

Once you have the FLASH image in the editor, it is then a simple thing
to do the search in order to find the instruction at ``1b02d596``. The
symbolic information will show you exactly which function the address
is in and also the context of the instruction that can be used to
associate it to the exact line of code in the original C source file.

You also have all of the register contents so it is pretty easy to see
what happened (assuming you have some basic knowledge of Thumb2
assembly language and the ARM EABI). But it is usually not so easy to
see why it happened.

The rest of the instructions apply to finding out why the fault
happened.

``R14`` often contains the return address to the caller of the
offending functions. Bit one is set in this return address, but ignore
that (I.e., use ``1b02b20a`` instead of ``1b02b20b``). Use the objdump
command above to see where that is.

Sometimes, however, ``R14`` is not the caller of the offending
function. If the offending functions calls some other function then
``R14`` will be overwritten. But no problem, it will also then have
pushed the return address on the stack where we can find it by
analyzing the stack dump.

Analyzing the Stack Dump
========================

The Task Stack
--------------

To go further back in the time, you have to analyze the stack. It is a
push down stack so older events are at higher stack addresses; the
most recent things that happened will be at lower stack addresses.

Analyzing the stack is done in basically the same way:

1. Start at the highest stack addresses (oldest) and work forward in
   time (lower addresses)

2. Find interesting addresses,

3. Use ``arm-none-eabi-objdump`` to determine where those addresses
   are in the code.

An interesting address has these properties:

1. It lies in FLASH in your architecture. In your case these are the
   addresses that begin with ``0x1a`` and ``0x1b``. Other
   architectures may have different FLASH addresses or even addresses
   in RAM.

2. The interesting addresses are all odd for Cortex-M, that is, bit 0
   will be set. This is because as the code progresses, the return
   address (``R14``) will be pushed on the stack. All of the return
   addresses will lie in FLASH and will be odd.

Even FLASH addresses in the stack dump usually are references to
``.rodata`` in FLASH but are sometimes of interest as well. Below are
examples of interesting addresses (in brackets):

.. code-block:: console

  sp:     10005e50
  User stack:
    base: 10005ed8
    size: 00000f9c
  10005e40: 00000000  00000000  00000000 [1b02d587] 10004900 00000000 005b8d7f 00000000
  10005e60: 1a030f2e  00000000  00000000  00001388  00000000 00000005 10001994 00000000
  10005e80: 00000000  00000000  00000000 [1b02c359] 00000000 00000000 00000000 004c4b40
  10005ea0: 000002ff  00000000  00000000 [1a030f2f] 00000000 00000000 00000000 00000000
  10005ec0: 00000000 [1a030f41] 00000000 [1b02c2a5] 00000000 00000000 ffffffff 00bdeb39

That will give the full backtrace up to the point of the failure.

The Interrupt Stack
-------------------

Note that in some cases there are two stacks listed. The interrupt
stack will be present if (1) the interrupt stack is enabled, and (2)
you are in an interrupt handler at the time that the failure occurred:

.. code-block:: console

  Assertion failed at file:armv7-m/up_hardfault.c line: 184 task: hpwork
  sp:     10001eb4
  IRQ stack:
   base: 10001f00
   size: 000003fc
  10001ea0: [1b02d961] 1b03f07e 10001eb4 10005ed8  1a0312ab   1b03f600   000000b8 [1b02d961]
  10001ec0:  00000010  10001f40 00000003 00000000 [1a03721d] [1a037209] [1b02d93b] 00000000
  10001ee0: [1a0371f5] 00000000 00000000 00000000  00000000   00000000  [1a0314a5] 10005d7c

(Interesting addresses again in brackets).

The interrupt stack is sometimes interesting, for example when the
interrupt was caused by logic operating at the interrupt level. In
this case, it is probably not so interesting since fault was probably
caused by normal task code and the interrupt stack probably just shows
the normal operation of the interrupt handling logic.

Full Stack Analysis
-------------------

What I have proposed here is just skimming through the stack, finding
and interpreting interesting addresses. Sometimes you need more
information and you need to analyze the stack in more detail. That is
also possible because every word on the stack is there because of an
explicit push instruction in the code (usually a push instruction on
Cortex-M or an stmdb instruction in other ARM architectures). This is
painstaking work but can also be done to provide a more detailed
answer to "what happened?"

Recovering State at the Time of the Hardfault
=============================================

Here is another tip from Mike Smith:

.. epigraph::

  "... for systems like NuttX where catching hardfaults is difficult,
  you can recover the faulting PC, LR and SP (by examining the
  exception stack), then write these values back into the appropriate
  processor registers (adjust the PC as necessary for the fault).

  "This will put you back in the application code at the point at
  which the fault occurred. Some local variables will show as having
  invalid values (because at the time of the fault they were live in
  registers and have been overwritten by the exception handler), but
  the stack frame, function arguments etc. should all show correctly."
