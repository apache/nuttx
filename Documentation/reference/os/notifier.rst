.. _notifier_chain:

==============
Notifier Chain
==============

NuttX provides a callback list mechanism called *Notifier Chain*.
Notifier chain is essentially a list of callbacks used at certain times,
such as system asserting, powering off and restarting.

**Notifier chain** is very much like the Linux notifier chains, except
for some implementation differences.

Classes of Notifier Chain
=========================

There are currently two different classes of notifier.

Atomic notifier chains
----------------------

Atomic notifier chains: Chain callbacks run in interrupt/atomic context.
In Nuttx, callouts are allowed to block(In Linux, callouts in atomic
notifier chain are not allowed to block). One example of an Atomic notifier
chain is turning off FPU when asserting.

Blocking notifier chains
------------------------

Blocking notifier chains: Chain callbacks run in process context.
Callouts are allowed to block. One example of a blocking notifier chain
is when an orderly powering off is needed.

Common Notifier Chain Interfaces
================================

Notifier Block Types
--------------------

-  ``struct notifier_block``. Defines one notifier callback entry.

Notifier Chain Interfaces
-------------------------

.. c:function:: void panic_notifier_chain_register(FAR struct notifier_block *nb)

  Add notifier to the panic notifier chain.

  The panic notifier chain is an atomic notifier chain. It will be called
  when asserting.

  :param nb: New entry in notifier chain.

.. c:function:: void panic_notifier_chain_unregister(FAR struct notifier_block *nb)

  Remove notifier from the panic notifier chain.

  The panic notifier chain is an atomic notifier chain. It will be called
  when asserting.

  :param nh: Entry to remove from notifier chain.

.. c:function:: void panic_notifier_call_chain(unsigned long action, FAR void *data)

  Call functions in the panic notifier chain.

  The panic notifier chain is an atomic notifier chain. It will be called
  when asserting.

  :param action: Value passed unmodified to notifier function.
  :param data: Pointer passed unmodified to notifier function.

.. c:function:: void register_reboot_notifier(FAR struct notifier_block *nb)

  Add notifier to the reboot notifier chain.

  The reboot notifier chain is an atomic notifier chain.

  :param nb: New entry in notifier chain.

.. c:function:: void unregister_reboot_notifier(FAR struct notifier_block *nb)

  Remove notifier from the reboot notifier chain.

  The reboot notifier chain is an atomic notifier chain.

  :param nh: Entry to remove from notifier chain.

.. c:function:: void reboot_notifier_call_chain(unsigned long action, FAR void *data)

  Call functions in the reboot notifier chain.

  The reboot notifier chain is an atomic notifier chain.

  :param action: Value passed unmodified to notifier function.
  :param data: Pointer passed unmodified to notifier function.

