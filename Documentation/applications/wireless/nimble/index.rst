===================================================
``nimble`` Apache NimBLE (BLE host-layer) for NuttX
===================================================

This application will build nimBLE stack (host-only) as a library/application
in NuttX.

For a Central/Peripheral example that tests connections, GATT service
discovery, and subscriptions without Bluetooth hardware, see
:ref:`simulator_bluetooth_testing`.

Configuration
-------------

NimBLE needs ``CONFIG_NET_BLUETOOTH`` and a Bluetooth controller exposed
through the NuttX HCI socket, and conflicts with the NuttX host
(``CONFIG_WIRELESS_BLUETOOTH_HOST`` must be off). Enable the GAP roles the
application needs with ``CONFIG_NIMBLE_ROLE_*``.

Cryptography uses Mbed TLS (``CONFIG_MBEDTLS_MINIMAL`` is enabled by
default). ``CONFIG_NIMBLE_TINYCRYPT`` builds the legacy bundled TinyCrypt
instead.

``CONFIG_NIMBLE_EVENTQ_SIZE`` sets the depth of the host event queue. Hosts
that scan need room for the advertising reports queued by the transport;
the default is 16, or 32 with mesh.

Bluetooth Mesh
--------------

``CONFIG_NIMBLE_MESH`` builds the NimBLE mesh stack. Provisioning bearers
and node features:

- ``CONFIG_NIMBLE_MESH_PB_ADV`` - PB-ADV, needs the broadcaster and
  observer roles.
- ``CONFIG_NIMBLE_MESH_PB_GATT`` - PB-GATT, needs the peripheral role.
- ``CONFIG_NIMBLE_MESH_GATT_PROXY``, ``CONFIG_NIMBLE_MESH_RELAY``,
  ``CONFIG_NIMBLE_MESH_FRIEND``, ``CONFIG_NIMBLE_MESH_LOW_POWER``,
  ``CONFIG_NIMBLE_MESH_PROVISIONER``.
- ``CONFIG_NIMBLE_MESH_DEVICE_NAME`` - name in the PB-GATT and proxy scan
  response.

Resource limits: ``CONFIG_NIMBLE_MESH_SUBNET_COUNT``,
``CONFIG_NIMBLE_MESH_APP_KEY_COUNT``, ``CONFIG_NIMBLE_MESH_MODEL_KEY_COUNT``,
``CONFIG_NIMBLE_MESH_MODEL_GROUP_COUNT``, ``CONFIG_NIMBLE_MESH_ADV_BUF_COUNT``,
``CONFIG_NIMBLE_MESH_TX_SEG_MAX`` and ``CONFIG_NIMBLE_MESH_RX_SEG_MAX``.

A mesh node scans continuously and the host is busy for up to a second
during provisioning (ECDH), so the Bluetooth socket must absorb a burst
of advertising reports: set ``CONFIG_NET_BLUETOOTH_BACKLOG=32`` and
``CONFIG_IOB_NBUFFERS=64``. With the defaults (8 and 8) the socket drops
the oldest queued HCI event, which can be a Command Complete, and the
host resets on an HCI timeout.

The controller must support scanning and advertising at the same time.
With the nRF52 SoftDevice Controller the peripheral-only configuration
has no scanning, so mesh needs ``CONFIG_NRF52_SDC_MULTIROLE`` and
``CONFIG_BLUETOOTH_MAX_CONN=2`` (one peripheral plus one central role).

Mesh applications create the HCI socket task, register the mesh GATT
services with ``bt_mesh_register_gatt()`` before the host starts, call
``bt_mesh_init()`` from the host sync callback, run ``mesh_adv_thread()``
on its own thread and enable provisioning with ``bt_mesh_prov_enable()``.
See the :doc:`node </applications/examples/nimble_mesh_node/index>`,
:doc:`sensor </applications/examples/nimble_mesh_sensor/index>` and
:doc:`vendor </applications/examples/nimble_mesh_vendor/index>` examples.

Nodes can be provisioned from Linux with BlueZ ``bluetooth-meshd`` and
``mesh-cfgclient``. ``meshd`` needs exclusive access to the adapter, so
``bluetoothd`` must not manage it at the same time.

.. toctree::
  :maxdepth: 2
  :caption: Contents

  porting.rst
