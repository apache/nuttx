.. include:: /substitutions.rst
===========================================
How to port
===========================================

This guide explains "How to port NuttX".

At this guide, the scope of porting is adding a new SoC or board directory,
which can be built and has a working example that boots to the NuttShell (NSH).

The goal of porting is to comfirm the booting NuttShell(NSH), the pass of "ostest"
and the timer.

Porting a new SoC and board to NuttX requires modifications on arch,
SoC and board directories.
For the details,
see :doc:`/quickstart/organization`,
:doc:`/components/arch/index`,
:doc:`/components/boards`.

To port NuttX properly, we have to understand the boot sequence and
related kernel configurations.
Following links explain them, although these depend on
specific kernel version and configurations.
(To understand them, we have to read the code deeply.)

.. toctree::
  port_bootsequence.rst
  port_relatedkernelconfigrations.rst

About the implementation, the build system tells you minimal implementation of SoC/Board directory,
these implementations are almost done by copying and pasting from other SoC/Board
directory if there is the source code for the target HW IP in upstream.
If there is not the source code in upstream, the porter has to implement it by himself.

Porting procedure
=================

At first, you have to read and execute :doc:`/quickstart/install` and :doc:`/quickstart/compiling_make`.
After that, try follwoing procedure.

+------+---------------+--------------------------------------------------------------------+
| Step | Process       | Comment                                                            |
+------+---------------+--------------------------------------------------------------------+
| 1    | Add the SoC   |                                                                    |
+------+---------------+--------------------------------------------------------------------+
| 2    | Add the Board | If the board was not sold in the market,                           |
|      |               | the board directory should be located out-of-tree.                 |
|      |               | For details see :doc:`/guides/customboards`.                       |
|      |               | And if you wanted to add own apps, see :doc:`/guides/customapps`.  |
+------+---------------+--------------------------------------------------------------------+
| 3    | Configure     | The configure needs to understand related kernel configurations.   |
|      |               | see :doc:`/guides/port_relatedkernelconfigrations`.                |
+------+---------------+--------------------------------------------------------------------+
| 4    | Compile       | The compile tells you which source files are needed in SoC/Board   |
|      |               | directory to pass the compile.                                     |
+------+---------------+--------------------------------------------------------------------+
| 5    | Link          | The link tells you which symbols are needed in SoC/Board directory |
|      |               | to pass the link.                                                  |
+------+---------------+--------------------------------------------------------------------+
| 6    | Implement     | Do implement the symbols which are needed by the link.             |
+------+---------------+--------------------------------------------------------------------+
| 7    | Verify        | Do "apps/testing/ostest".                                          |
|      |               | I think the pass of ostest is the one of proof for proper porting. |
|      |               | And check the timer implementation whether the kernel could        | 
|      |               | count the time accurately or not.                                  |
+------+---------------+--------------------------------------------------------------------+

Porting Case Studies
===============================

These porting guides depend on specific kernel versions, as some code structures have changed over time. They will still
provide a general idea on how to port.

.. toctree::
   :glob:
   :maxdepth: 1

   porting-case-studies/*
