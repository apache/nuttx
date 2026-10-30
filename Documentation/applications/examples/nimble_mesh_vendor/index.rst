=================================================
``nimble_mesh_vendor`` NimBLE Mesh vendor model
=================================================

A provisionable Bluetooth Mesh node with a vendor model (company
``0x05F1``, model ``0x0001``) on its primary element. The model stores one
byte: vendor opcode ``0x01`` (Get) and ``0x03`` (Set, one byte) answer with
opcode ``0x02`` (Status) carrying the value.

Needs ``CONFIG_NIMBLE_MESH`` with PB-ADV or PB-GATT, see
:doc:`/applications/wireless/nimble/index`. Start the node from NSH and
provision it with a mesh provisioner::

  nsh> nimble_mesh_vendor &
  nimble_mesh_vendor: provisioning enabled

Device UUID ``ddaa0003-4d45-5348-564e-440000000001``.
