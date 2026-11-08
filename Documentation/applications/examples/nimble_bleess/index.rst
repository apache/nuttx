=========================================================
``nimble_bleess`` NimBLE Environmental Sensing peripheral
=========================================================

The ``nimble_bleess`` example implements a Bluetooth LE Environmental
Sensing peripheral using Apache NimBLE. It advertises the standard
Environmental Sensing service and exposes readable and notifiable temperature
and humidity characteristics.

The example generates slowly changing sample values every two seconds, so it
can be used without sensor hardware. A real sensor application can replace
the generated values by calling ``bleess_gatt_update()``.

GATT service
============

The example uses the following standard Bluetooth UUIDs and value formats:

.. list-table::
   :header-rows: 1

   * - Attribute
     - UUID
     - Format
   * - Environmental Sensing service
     - ``0x181a``
     - Service
   * - Temperature characteristic
     - ``0x2a6e``
     - Signed 16-bit little-endian value in 0.01 degrees Celsius
   * - Humidity characteristic
     - ``0x2a6f``
     - Unsigned 16-bit little-endian value in 0.01 percent

Configuration
=============

Enable ``CONFIG_EXAMPLES_NIMBLE_BLEESS`` together with Apache NimBLE and
the peripheral role:

.. code-block:: Kconfig

   CONFIG_NET_BLUETOOTH=y
   CONFIG_NIMBLE=y
   CONFIG_NIMBLE_ROLE_PERIPHERAL=y
   CONFIG_EXAMPLES_NIMBLE_BLEESS=y

NimBLE requires a Bluetooth controller exposed through the NuttX HCI socket.
See :doc:`/applications/wireless/nimble/index` for the common NimBLE
configuration requirements.

Usage
=====

Run the example from NSH:

.. code-block:: console

   nsh> bleess

An optional numeric argument selects the HCI device:

.. code-block:: console

   nsh> bleess 0

The peripheral advertises as ``NuttX NimBLE ESS``. A Bluetooth LE client
can discover the Environmental Sensing service, read both characteristics,
and subscribe to temperature and humidity notifications.
