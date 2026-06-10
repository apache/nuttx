==============================
``ip6tables`` IPv6 firewall
==============================

The ``ip6tables`` command is used to set up, maintain, and inspect the
tables of IPv6 packet filter rules in the NuttX kernel.

Configuration
=============

- :kconfig:option:`CONFIG_SYSTEM_IP6TABLES`
- :kconfig:option:`CONFIG_NET_IPTABLES`
- :kconfig:option:`CONFIG_NET_IPv6`

The following additional options are available:

- :kconfig:option:`CONFIG_SYSTEM_IPTABLES_PRIORITY` - Task priority
  (default: 100)
- :kconfig:option:`CONFIG_SYSTEM_IPTABLES_STACKSIZE` - Stack size
  (default: ``DEFAULT_TASK_STACKSIZE``)

Usage
=====

.. code-block:: text

   ip6tables -t table -[AD] chain rule-specification
   ip6tables -t table -I chain [rulenum] rule-specification
   ip6tables -t table -D chain rulenum
   ip6tables -t table -P chain target
   ip6tables -t table -[FL] [chain]

Commands
========

``-A, --append chain``
   Append one or more rules to the end of the selected chain.

``-D, --delete chain [rulenum]``
   Delete one or more rules from the selected chain. If ``rulenum`` is
   specified, delete the rule at that position.

``-I, --insert chain [rulenum]``
   Insert one or more rules at the given position in the selected chain.
   If ``rulenum`` is not specified, the rule is inserted at position 1.

``-L, --list [chain]``
   List all rules in the selected chain. If no chain is specified, all
   chains in the table are listed.

``-F, --flush [chain]``
   Delete all rules in the selected chain. If no chain is specified, all
   chains in the table are flushed.

``-P, --policy chain target``
   Set the policy for the built-in chain to the specified target. The
   target must be ``ACCEPT`` or ``DROP``.

Options
=======

``-t, --table table``
   Specify the table to manipulate. The default table is ``filter``.

   Only the ``filter`` table is currently supported (requires
   :kconfig:option:`CONFIG_NET_IPFILTER`).

``-j, --jump target``
   Specify the target of the rule; i.e., what to do if the packet
   matches it. The target can be ``ACCEPT``, ``DROP``, or a custom
   target name.

``[!] -s, --source address[/mask]``
   Source specification. ``address`` can be a network name, hostname,
   network IPv6 address (with ``/mask``), or plain IPv6 address. The
   mask is specified as a prefix length (e.g., ``/64``). The ``!``
   argument inverts the match.

``[!] -d, --destination address[/mask]``
   Destination specification. Same format as ``--source``.

``[!] -p, --protocol protocol``
   Protocol of the rule or of the packet to check. The specified
   protocol can be one of ``tcp``, ``udp``, ``icmp6``, ``icmpv6``,
   ``ipv6-icmp``, ``esp``, or ``all``, or a numeric protocol number.
   The ``!`` argument inverts the match.

``[!] -i, --in-interface dev``
   Name of an interface via which a packet was received. The ``!``
   argument inverts the match.

``[!] -o, --out-interface dev``
   Name of an interface via which a packet is going to be sent. The
   ``!`` argument inverts the match.

``[!] --sport, --source-port port[:port]``
   Source port specification. Can be a single port or a port range
   (e.g., ``1024:65535``). Only valid with ``-p tcp`` or ``-p udp``.
   The ``!`` argument inverts the match.

``[!] --dport, --destination-port port[:port]``
   Destination port specification. Same format as ``--source-port``.

``[!] --icmpv6-type type``
   ICMPv6 type specification. Can be a numeric type (0-255). Only
   valid with ``-p icmp6`` (or ``-p icmpv6``, ``-p ipv6-icmp``). The
   ``!`` argument inverts the match.

``!``
   Inverts the following match criterion.

Examples
========

List all rules in the filter table:

.. code-block:: text

   nsh> ip6tables -L

Append a rule to allow TCP traffic on port 22:

.. code-block:: text

   nsh> ip6tables -A INPUT -p tcp --dport 22 -j ACCEPT

Insert a rule at position 1 to drop UDP traffic from a specific source:

.. code-block:: text

   nsh> ip6tables -I INPUT 1 -s fc00::1 -p udp -j DROP

Delete a specific rule by rule number:

.. code-block:: text

   nsh> ip6tables -D INPUT 1

Set the default policy for the INPUT chain to DROP:

.. code-block:: text

   nsh> ip6tables -P INPUT DROP

Flush all rules in the INPUT chain:

.. code-block:: text

   nsh> ip6tables -F INPUT

Use negation to match all protocols except TCP:

.. code-block:: text

   nsh> ip6tables -A INPUT ! -p tcp -j ACCEPT

See Also
========

- :doc:`../iptables/index`
