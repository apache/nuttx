=====================
Power-related Drivers
=====================

.. toctree::
  :caption: Supported Drivers

  pm/index.rst
  regulator.rst
  battery/fakegauge.rst

What is under ``drivers/power/``
================================

Four areas, and the pages above do not yet cover all of them:

* ``pm/`` -- the power management subsystem, documented below.
* ``supply/`` -- regulators, plus ``act8945a``, ``powerled`` and ``smps``.
* ``battery/`` -- chargers, gauges and monitors for a dozen parts.  Only the
  fake gauge has a page so far, and it is a test device, not the real
  interface.
* ``relay/`` -- relay control, with no page yet.

Design
======

.. note::

   The page below and :doc:`pm/index` are both called *Power Management* and
   they overlap, particularly on callbacks.  They come from the two halves
   of the old documentation -- one from ``components/``, describing how the
   subsystem is put together, the other from ``implementation/``, describing
   how it works -- and neither links to the other.  Deciding what to keep
   needs somebody who knows which half is current, so they are left as they
   are for now rather than merged into something that reads as one document
   but is not.

.. toctree::
   :maxdepth: 1

   power_management.rst
