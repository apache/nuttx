==============================================
``nimble_mesh_node`` NimBLE Mesh node
==============================================

A provisionable Bluetooth Mesh node (company ``0x05F1``) with the
Configuration Server, Health Server and Generic OnOff Server models on its
primary element. Generic OnOff Get, Set and Set Unacknowledged are handled
and the state is reported with Generic OnOff Status.

Needs ``CONFIG_NIMBLE_MESH`` with PB-ADV or PB-GATT, see
:doc:`/applications/wireless/nimble/index`. Start the node from NSH and
provision it with a mesh provisioner::

  nsh> nimble_mesh_node &
  nimble_mesh_node: provisioning enabled

Device UUID ``ddaa0001-4d45-5348-4f4e-4f4646000001``.
