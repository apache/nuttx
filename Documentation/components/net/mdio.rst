.. _mdio_bus:

.. include:: /substitutions.rst

=================
MDIO Bus Driver
=================

The NuttX MDIO bus driver provides a standardized interface for communicating with Ethernet PHY (Physical Layer) transceivers.
It employs a classic upper-half/lower-half architecture to abstract hardware-specific logic from the generic MDIO protocol,
which is currently compliant with Clause 22 of the IEEE 802.3 standard.
The primary implementation of the upper-half can be found in ``drivers/net/mdio.c``.

Driver Architecture
===================

The MDIO driver framework serves as an intermediary layer between a network device driver and the physical bus.
The intended operational model is ``netdev -> phydev -> mdio``, where the network device communicates with a dedicated PHY driver,
which in turn uses the MDIO bus driver for low-level hardware access.
Direct interaction between the network device and the MDIO bus is discouraged.

Upper-Half Implementation
-------------------------

The upper-half driver contains the core logic for the MDIO bus, including bus locking mechanisms to ensure safe transactions.
It exposes a generic API for managing the bus lifecycle and is capable of handling multiple, independent MDIO bus instances concurrently.
This abstracts implementation details from both the PHY driver and the underlying hardware-specific code.

Lower-Half Implementation
-------------------------

A lower-half MDIO driver serves as a thin layer that maps the generic operations defined by the upper-half to hardware-specific register manipulations.
It is not intended to contain complex logic, but rather to provide a direct translation for bus operations.

Implementing a Lower-Half Driver
================================

Integrating MDIO support for new hardware requires the implementation of a lower-half driver.
The contract between the upper and lower halves is defined in ``include/nuttx/net/mdio.h`` and is centered around two key structures.

Key Data Structures
-------------------

1.  ``struct mdio_ops_s``: A structure containing function pointers that the lower-half driver must implement to perform hardware-level operations.
    *   ``read``: Performs a Clause 22 MDIO read operation.
    *   ``write``: Performs a Clause 22 MDIO write operation.
    *   ``reset``: An optional function to execute a hardware-specific PHY reset.

2.  ``struct mdio_lowerhalf_s``: The container for the lower-half instance, which holds a pointer to the ``mdio_ops_s`` vtable 
    and an optional private data pointer for the driver's internal state.

Registration and Unregistration
-------------------------------

The board-level initialization logic is responsible for instantiating the lower-half driver and registering it with the upper-half via the ``mdio_register()`` function.
Each call to this function with a distinct lower-half driver creates a new, unique bus handle, allowing the system to manage several MDIO buses concurrently.

.. code-block:: c

    FAR struct mdio_dev_s *mdio_register(FAR struct mdio_lowerhalf_s *lower);

This function accepts the lower-half instance and returns an opaque handle (``FAR struct mdio_dev_s *``),
which is subsequently used by the PHY driver to interact with the bus.

When a bus instance is no longer required, it should be deallocated by calling the ``mdio_unregister()`` function to ensure proper cleanup of resources.

.. code-block:: c

    int mdio_unregister(FAR struct mdio_dev_s *dev);

This function takes the handle returned by ``mdio_register()`` and releases the associated bus instance.

A (mostly) complete reference implementation for a lower-half driver is available in ``arch/arm/src/stm32h7/stm32_mdio.c``.
