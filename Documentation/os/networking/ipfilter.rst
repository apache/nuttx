================
IP Packet Filter
================

NuttX supports IP packet filter (firewall) compatible with Linux's iptables and
netfilter. It is a stateless packet filter that can be used to filter packets
based on source and destination IP addresses, source and destination ports,
protocol, TCP flags, and interface. A rule can also have a rate limit.

Workflow
========

Similar to Linux's iptables, NuttX's IP packet filter defines chains at similar
points in the packet processing path. The following diagram shows the packet
processing path and the chains that are defined in NuttX's IP packet filter.

::

   NIC ──> ipv[46]_input ─┬─> ipv[46]_forward ──> [FORWARD CHAIN] ──> devif_poll_out ──> NIC
                          │                                                 ^
                          │                  ┌─>  tcp  ─┐                   │
                          │                  ├─>  udp  ─┤                   │
                          └─> [INPUT CHAIN] ─┼─> icmp  ─┼─> [OUTPUT CHAIN] ─┘
                                             ├─> icmp6 ─┤
                                             └─>  ...  ─┘

Configuration Options
=====================

``CONFIG_NET_IPFILTER``
  Enable this option to enable the IP packet filter (firewall).

``CONFIG_NET_IPTABLES``
  Enable or disable iptables compatible interface (including ip6tables).

``CONFIG_SYSTEM_IPTABLES``
  Enable support for the 'iptables' command.

``CONFIG_SYSTEM_IP6TABLES``
  Enable support for the 'ip6tables' command.

Usage
=====

With `iptables` command, we can add, delete, and list rules in the IP packet
filter. It's similar to the `iptables` command in Linux.

The following examples show the commands we support:

..  code-block:: shell

  > iptables -h

  USAGE: iptables -t table -[AD] chain rule-specification
         iptables -t table -I chain [rulenum] rule-specification
         iptables -t table -D chain rulenum
         iptables -t table -P chain target
         iptables -t table -[FL] [chain]

  Commands:
      --append        -A chain            Append a rule to chain
      --insert        -I chain [rulenum]  Insert a rule to chain at rulenum (default = 1)
      --delete        -D chain [rulenum]  Delete matching rule from chain
      --policy        -P chain target     Set policy for chain to target
      --flush         -F [chain]          Delete all rules in chain or all chains
      --list          -L [chain]          List all rules in chain or all chains

  Options:
      --table         -t table            Table to manipulate (default: filter)
      --jump          -j target           Target for rule
  [!] --in-interface  -i dev              Input network interface name
  [!] --out-interface -o dev              Output network interface name
  [!] --source        -s address[/mask]   Source address
  [!] --destination   -d address[/mask]   Destination address
  [!] --protocol      -p proto            Protocol (tcp, udp, icmp, esp, all)
  [!] --source-port,--sport
                         port[:port]      Source port
  [!] --destination-port,--dport
                         port[:port]      Destination port
  [!] --icmp-type        type             ICMP type
  [!] --icmpv6-type      type             ICMPv6 type
  [!] --tcp-flags        mask comp        TCP flags (SYN,ACK,FIN,RST,URG,PSH,ALL,NONE)
  [!] --syn                               TCP SYN (--tcp-flags SYN,RST,ACK,FIN SYN)
      --match         -m limit            Match a packet rate
      --limit            rate[/unit]      Packets per s, m, h or d (default: 3/hour)
      --limit-burst      number           Packets that can come at once (default: 5)

..  code-block:: shell

  > iptables -P FORWARD DROP
  > iptables -I INPUT -i eth0 ! -p icmp -j DROP
  > iptables -t filter -A FORWARD -p tcp -s 10.0.1.2/24 -d 10.0.3.4/24 -i eth0 -o eth1 --sport 3000:3200 --dport 123:65535 -j ACCEPT
  > iptables -t filter -I FORWARD 2 -p icmp ! -s 123.123.123.123 ! -i eth0 -o eth1 ! --icmp-type 255 -j REJECT

  > iptables -L
  Chain INPUT (policy ACCEPT)
  target        prot  idev  odev  source              destination
  DROP         !icmp  eth0  any   anywhere            anywhere

  Chain FORWARD (policy DROP)
  target        prot  idev  odev  source              destination
  ACCEPT        tcp   eth0  eth1  10.0.1.2/24         10.0.3.4/24        tcp spts:3000:3200 dpts:123:65535
  REJECT        icmp !eth0  eth1 !123.123.123.123/32  anywhere           icmp !type 255

  Chain OUTPUT (policy ACCEPT)
  target        prot  idev  odev  source              destination

..  code-block:: shell

  > ip6tables -P FORWARD DROP
  > ip6tables -t filter -I FORWARD -p tcp -s fc00::2/64 -d 2001:da8::2:4/64 -i eth0 -o eth1 --sport 3000:3200 --dport 123:65535 -j ACCEPT
  > ip6tables -t filter -I FORWARD -p icmpv6 -s fc00::2/64 -d 2001:da8::2:4/64 -i eth0 -o eth1 --icmpv6-type 123 -j ACCEPT
  > ip6tables -t filter -I FORWARD -p tcp -i eth0 -o eth1 --sport 3000 -j ACCEPT
  > ip6tables -t filter -I FORWARD 1 ! -p tcp ! -s fc00::2/64 ! -d 2001:da8::2:4/64 ! -i eth0 ! -o eth1 ! --sport 3000:3200 ! --dport 0:123 -j DROP
  > ip6tables -t filter -I FORWARD 3 ! -p icmpv6 ! -s fc00::2/64 -d 2001:da8::2:4/64 ! -i eth0 -o eth1 ! --icmpv6-type 255 -j REJECT

  > ip6tables -L
  Chain INPUT (policy ACCEPT)
  target        prot  idev  odev  source              destination

  Chain FORWARD (policy DROP)
  target        prot  idev  odev  source              destination
  DROP         !tcp  !eth0 !eth1 !fc00::2/64         !2001:da8::2:4/64   tcp spts:!3000:3200 dpts:!0:123
  ACCEPT        tcp   eth0  eth1  anywhere            anywhere           tcp spts:3000:3000 dpts:0:65535
  REJECT       !ipv6-icmp !eth0  eth1 !fc00::2/64          2001:da8::2:4/64   ipv6-icmp !type 255
  ACCEPT        ipv6-icmp  eth0  eth1  fc00::2/64          2001:da8::2:4/64   ipv6-icmp type 123
  ACCEPT        tcp   eth0  eth1  fc00::2/64          2001:da8::2:4/64   tcp spts:3000:3200 dpts:123:65535

  Chain OUTPUT (policy ACCEPT)
  target        prot  idev  odev  source              destination

A rule can match TCP flags (``--tcp-flags``, ``--syn``) and can have a rate
limit (``-m limit``). The filter keeps one token bucket per rule: the rule
matches at most ``--limit`` packets on average, and up to ``--limit-burst``
packets at once. A packet over the rate does not match the rule, and the next
rule of the chain applies. The bucket is full again after each change of the
table. The limit works alone or after a TCP, UDP or ICMP match.

..  code-block:: shell

  > iptables -A INPUT -p tcp --syn -m limit --limit 5/s --limit-burst 10 -j ACCEPT
  > iptables -A INPUT -p tcp --syn -j DROP
  > iptables -L INPUT
  Chain INPUT (policy ACCEPT)
  target        prot  idev  odev  source              destination
  ACCEPT        tcp   any   any   anywhere            anywhere           tcp spts:0:65535 dpts:0:65535 flags:0x17/0x02 limit: avg 5/second burst 10
  DROP          tcp   any   any   anywhere            anywhere           tcp spts:0:65535 dpts:0:65535 flags:0x17/0x02
