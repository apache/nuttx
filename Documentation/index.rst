===================
NuttX Documentation
===================

NuttX is a real-time operating system (RTOS) with an emphasis on standards
compliance and small footprint. Scalable from 8-bit to 64-bit microcontroller
environments, the primary governing standards in NuttX are POSIX and ANSI
standards. Additional standard APIs from Unix and other common RTOS’s (such as
VxWorks) are adopted for functionality not available under these standards, or
for functionality that is not appropriate for deeply-embedded environments (such
as fork()).

Where to start
==============

.. grid:: 1 2 2 3
   :gutter: 3

   .. grid-item-card:: :octicon:`rocket;1.2em;sd-text-primary` Getting Started
      :link: quickstart/index
      :link-type: doc

      Install the toolchain, build for a board, get a shell.
      Start here if you have never built NuttX.

   .. grid-item-card:: :octicon:`cpu;1.2em;sd-text-primary` Supported Platforms
      :link: platforms/index
      :link-type: doc

      Every architecture, chip and board NuttX runs on.
      Start here if you have hardware in your hand.

   .. grid-item-card:: :octicon:`checklist;1.2em;sd-text-primary` Guides
      :link: guides/index
      :link-type: doc

      How to do a particular thing: port, write a driver, debug.
      Start here when you know what you want to build.

Understanding the system
========================

.. grid:: 1 2 2 3
   :gutter: 3

   .. grid-item-card:: :octicon:`stack;1.2em;sd-text-primary` OS Design
      :link: os/index
      :link-type: doc

      How NuttX is built, subsystem by subsystem.

   .. grid-item-card:: :octicon:`book;1.2em;sd-text-primary` API Reference
      :link: reference/index
      :link-type: doc

      The POSIX and NuttX calls an application may make.

   .. grid-item-card:: :octicon:`apps;1.2em;sd-text-primary` Applications
      :link: applications/index
      :link-type: doc

      The programs that ship with NuttX, from NSH onwards.

Working on NuttX
================

.. grid:: 1 2 2 2
   :gutter: 3

   .. grid-item-card:: :octicon:`tools;1.2em;sd-text-primary` Developing NuttX
      :link: developing/index
      :link-type: doc

      Getting a change accepted, the build system, porting, testing.

   .. grid-item-card:: :octicon:`info;1.2em;sd-text-primary` About
      :link: about/index
      :link-type: doc

      Questions, vocabulary, security reports, release notes.

How this documentation is organised
===================================

Four ideas decide where anything goes.  They are written down here because
knowing them turns "where do I look?" into a question with an answer -- and,
for anyone writing documentation, "where do I put this?" as well.

**The sections follow what you are doing, not what NuttX contains.**
Arriving, finding your hardware, doing a task, understanding a subsystem,
looking up a call: those are different activities, and each has a section.
Guides comes before OS Design on purpose, because people want something
running before they want to know how the scheduler works.

**Three different questions, three different sections.**  It is the same
subject seen three ways, and mixing them is what makes documentation hard to
search:

.. list-table::
   :header-rows: 1
   :widths: 22 30 48

   * - Section
     - Answers
     - For example
   * - :doc:`Guides <guides/index>`
     - *How do I do X?*
     - How to mount a ROMFS image at ``/etc``
   * - :doc:`OS Design <os/index>`
     - *How does X work?*
     - How the scheduler picks the next thread
   * - :doc:`API Reference <reference/index>`
     - *What may I call?*
     - What ``sched_setscheduler()`` takes and returns

**OS Design follows the source tree.**  Each top level directory of NuttX --
``sched/``, ``fs/``, ``net/``, ``mm/``, ``drivers/`` -- is a section, and
each subsystem is described once, going from what it is, to how it works, to
the interfaces it offers.  That is why there is one page about SMP rather
than three, and why a contributor knows which page to add to.

**What can be derived is derived, and checked.**  Board pages, the
architecture and chip each one is filed under, and the tags that let you
search for them all come from the source tree rather than being typed by
hand, and the build fails when the two disagree.  A page describing a board
that no longer exists, or a board with no page, does not survive the next
pull request.

.. note::
   Something wrong on a page, or missing?  The documentation lives in the
   same repository as the code, and every page has an *Edit on GitHub* link
   at the top right.  See :doc:`contributing/documentation` for how to build
   it locally.

.. toctree::
   :caption: Table of Contents
   :maxdepth: 2
   :hidden:

   Home <self>
   introduction/index.rst
   quickstart/index.rst
   platforms/index.rst
   guides/index.rst
   os/index.rst
   reference/index.rst
   applications/index.rst
   developing/index.rst
   about/index.rst

.. include:: substitutions.rst
