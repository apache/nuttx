
-include $(TOPDIR)/Make.defs

ASRCS =
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = gapuino_appinit.c gapuino_sysinit.c

COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  CFLAGS += -I$(ARCH_SRCDIR) -I$(ARCH_SRCDIR)\common -I$(ARCH_SRCDIR)\rv32im
else
ifeq ($(WINTOOL),y)
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}" \
            -I "${shell cygpath -w $(ARCH_SRCDIR)/common}"
else
  CFLAGS += -I$(ARCH_SRCDIR)/$(CONFIG_ARCH_CHIP) -I$(ARCH_SRCDIR)/common -I$(ARCH_SRCDIR)/rv32im
endif
endif

all: libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
