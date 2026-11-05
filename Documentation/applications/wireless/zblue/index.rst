=======================================================
``zblue`` Zephyr Bluetooth host in userspace
=======================================================

This application builds the Bluetooth host of the `Zephyr project
<https://github.com/zephyrproject-rtos/zephyr>`_ as a userspace library. The
Zephyr source is downloaded from a Zephyr release tag and is used without
modification; only a small porting layer in ``apps/wireless/bluetooth/zblue``
is NuttX code.

It is an optional stack. The NuttX Bluetooth stack
(``wireless/bluetooth``, ``net/bluetooth``) and NimBLE stay available; only
one host can be enabled at a time.

Applications use the Zephyr Bluetooth API (``<zephyr/bluetooth/...>``) as
documented by Zephyr.

Only the simulator is supported for now. On other targets the initialized
iterable RAM sections of the host (``net_buf`` pools) need support in the
board linker scripts, which is not there yet.

Configuration
-------------

``CONFIG_ZBLUE``
  Enables the stack. It needs ``CONFIG_UART_BTH4``: the host talks to the
  controller through ``/dev/ttyHCIx``, so any driver registered with
  ``bt_driver_register()`` can be used.

``CONFIG_ZBLUE_REF``
  Zephyr release tag to download, for example ``v4.5.0``.

``CONFIG_ZBLUE_CONF``
  Zephyr configuration fragment with the ``CONFIG_BT_*`` options, relative
  to the application directory (default ``zblue.conf``). The options are
  the original Zephyr ones. They are resolved with the Zephyr Kconfig
  after download and do not appear in the NuttX menuconfig.

``CONFIG_ZBLUE_HCI_DEVICE``
  HCI device, default ``/dev/ttyHCI0``.

``CONFIG_ZBLUE_THREAD_PRIORITY``
  Zephyr cooperative priorities are mapped above this NuttX priority,
  preemptive priorities below it.

Cryptography uses the PSA API of Mbed TLS (``apps/crypto/mbedtls``); the
options the host needs are selected automatically and
``CONFIG_MBEDTLS_MINIMAL`` is enabled by default. Mbed TLS needs an entropy
source, e.g. ``CONFIG_DEV_URANDOM`` on the simulator or the hardware random
number generator of the board.

Download and build requirements: ``curl``, ``tar`` and ``python3``.

Updating the Zephyr release
---------------------------

Change ``CONFIG_ZBLUE_REF`` and remove the ``zephyr`` directory and the
downloaded archive (``make distclean``). Fixes to the Zephyr host itself are
made in Zephyr, not here.

Porting layer
-------------

* Zephyr kernel services used by the host (threads, semaphores, mutexes,
  queues, memory slabs, work queues, timeouts) are implemented on POSIX
  threads, without NuttX internal interfaces.
* Zephyr cooperative threads are serialized with one lock that is
  released only while a thread blocks; ``k_sched_lock()`` takes the same
  lock.
* Iterable sections (``BT_CONN_CB_DEFINE()``, ``BT_GATT_SERVICE_DEFINE()``,
  ...) are collected by the linker without changes to the board linker
  script. Entries of a type are in link order.
* ``SYS_INIT()`` functions run before ``bt_enable()`` starts the host.

Example
-------

``CONFIG_ZBLUE_PERIPHERAL_EXAMPLE`` builds ``zblue_peripheral``: it
advertises as ``CONFIG_BT_DEVICE_NAME`` and accepts connections, with the
GAP and GATT services. See the ``sim:zblue`` configuration of the simulator.
