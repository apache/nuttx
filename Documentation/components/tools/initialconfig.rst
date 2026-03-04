===================
``initialconfig.c``
===================

This is a C file that can be used to create an initial configuration. This
permits creating a new configuration from scratch, without relying on any
existing board configuration in place. This utility will create a barebones
``.config`` file sufficient only for instantiating the symbolic links necessary
to do a real configuration.
