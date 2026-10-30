==================================================
``nimble_mesh_sensor`` NimBLE Mesh Sensor Server
==================================================

A provisionable Bluetooth Mesh node with a Sensor Server on its primary
element serving one property, Present Ambient Temperature (``0x004F``).
Sensor Descriptor Get and Sensor Get answer with Sensor Descriptor Status
and Sensor Status; the value is a simulated temperature that increases by
0.5 C on every read. An unknown Property ID gets the empty reply defined by
the Mesh Model specification.

Needs ``CONFIG_NIMBLE_MESH`` with PB-ADV or PB-GATT, see
:doc:`/applications/wireless/nimble/index`. Start the node from NSH and
provision it with a mesh provisioner::

  nsh> nimble_mesh_sensor &
  nimble_mesh_sensor: provisioning enabled

Device UUID ``ddaa0002-4d45-5348-5345-4e5300000001``.
