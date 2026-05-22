============
micro-ROS
============

Phase 2: Library Skeleton
==========================

This phase establishes the foundational library structure for micro-ROS integration in Apache NuttX.

Structure
---------

The micro-ROS library is located in ``apps/system/microros/`` with the following components:

- ``Make.defs`` — Build configuration
- ``Kconfig`` — Configuration options
- ``Makefile`` — Build orchestration
- ``.gitignore`` — Excluded build artifacts

Configuration Options
---------------------

Enable via Kconfig ``CONFIG_SYSTEM_MICROROS``:

**Core Options:**

- ``MICROROS_DISTRO`` — ROS 2 distribution (default: ``jazzy``)
- ``MICROROS_TRANSPORT_UDP`` or ``MICROROS_TRANSPORT_SERIAL`` — Transport method

**UDP Transport (if selected):**

- ``MICROROS_AGENT_IP`` — micro-ROS agent IP (default: ``10.42.0.214``)
- ``MICROROS_AGENT_PORT`` — Agent UDP port (default: ``8888``)

**Serial Transport (if selected):**

- ``MICROROS_SERIAL_DEVICE`` — Serial device path (default: ``/dev/ttyS0``)
- ``MICROROS_SERIAL_BAUD`` — Serial baud rate (default: ``115200``)

**Resource Limits:**

- ``MICROROS_MAX_NODES`` — Maximum ROS nodes (default: 1)
- ``MICROROS_MAX_PUBLISHERS`` — Publishers per node (default: 5)
- ``MICROROS_MAX_SUBSCRIPTIONS`` — Subscriptions per node (default: 5)
- ``MICROROS_MAX_SERVICES`` — Services per node (default: 1)
- ``MICROROS_MAX_CLIENTS`` — Service clients per node (default: 1)

Build Status
------------

Phase 2 provides configuration infrastructure. Subsequent phases will add:

- Phase 3: libmicroros.a build from upstream sources
- Phase 4: Transport layer implementations
- Phase 5+: Examples, board support, CI integration
