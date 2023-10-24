==========================
``ping`` ICMP ping support
==========================

This is an unfinished implementation of ping and ping6 using raw
sockets. It is not yet hooked into the configuration or build systems.

Current ``ping`` / ``ping6`` logic in NSH makes illegal calls into the OS in order
to implement ``ping`` / ``ping6``. One correct implementation would be to use raw
sockets to implement ``ping`` / ``ping6`` as a user application. This is a first cut
at such an implementation.
