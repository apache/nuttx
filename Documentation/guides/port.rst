.. include:: /substitutions.rst
===========================================
How to port
===========================================

This guide explains "How to port NuttX".  At this guide, the scope of porting is to add new SoC/Board directory, the goal of porting is to comfirm to boot NuttShell(NSH).

The poring NuttX mainly consists of processor arch directory, SoC directory and Board directory.
For the details, see :doc:`/quickstart/organization`, :doc:`/components/arch/index`, :doc:`/components/boards`.

To port NuttX properly, we have to understand the boot sequence and related kernel configurations.
Following links explain them, although these depend on specific kernel version and configurations.
(To understand them deeply, we have to read the code deeply.)

.. toctree::
  port_bootsequence.rst
  port_relatedkernelconfigrations.rst

About the implementation, the build system will teach the least implementation for SoC/Board directory,
these implementations are almost done by copy and paste from the other SoC/Board directory
if there is the source code for the target HW IP in upstream.
If there is not the source code in upstream, the porter has to implement it by himself.

Porting procedure
=================

Previously you have to read and execute :doc:`/quickstart/install` and :doc:`/quickstart/compiling_make`.

.. list-table::

    * - Step
      - Process
      - Comments
    * - 1
      - Add and register the SoC directory
      - 
    * - 2
      - Add and register the Board directory
      - If the board was not sold in the market, the board directory should be located out-of-tree. For details see :doc:`/guides/customboards`. And if you wanted to add own apps, see :doc:`/guides/customapps`.
    * - 3
      - Configuring
      - The configuring is needed to understand related kernel configurations. see :doc:`/guides/port_relatedkernelconfigrations`.
    * - 4
      - Compiling
      - The compiling will teach what source files are needed by SoC/Board directory to pass the compiling.
    * - 5
      - Linking
      - The linking will teach what symbols are needed by SoC/Board directory to pass the linking.
    * - 6
      - Implementing
      - Do implement the symbols which are needed by Linking.
    * - 7
      - Verifying
      - Do "apps/testing/ostest". I think the pass of ostest is the one of proof for proper porting. And check the timer implementation whether the kernel could count the time accurately or not.

The result of porting procedure
===============================

Although these depend on specific kernel version.

.. toctree::
  port_arm_cm4.rst
