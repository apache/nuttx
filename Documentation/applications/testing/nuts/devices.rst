=======================
Devices Category (NUTS)
=======================

This test category tests the behaviour of NuttX devices under `/dev` by
interacting with their character driver interface. The category can be enabled
with ``CONFIG_TESTING_NUTS_DEVICES``.

The following test suites are included in this category:

* ``CONFIG_TESTING_NUTS_DEVICES_DEVASCII``: Tests for ``/dev/ascii``
* ``CONFIG_TESTING_NUTS_DEVICES_DEVCONSOLE``: Tests for ``/dev/console``
* ``CONFIG_TESTING_NUTS_DEVICES_DEVNULL``: Tests for ``/dev/null``
* ``CONFIG_TESTING_NUTS_DEVICES_DEVURANDOM``: Tests for ``/dev/urandom``
* ``CONFIG_TESTING_NUTS_DEVICES_DEVZERO``: Tests for ``/dev/zero``
