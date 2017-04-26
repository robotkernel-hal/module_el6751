includedir = $(prefix)/include/el6751

lib_LTLIBRARIES = libmodule_el6751.la
include_HEADERS = el6751.h

libmodule_el6751_la_SOURCES  = el6751.cpp
libmodule_el6751_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) \
                               @ROBOTKERNEL_CFLAGS@ @LIBSTRING_UTIL_CFLAGS@ @YAML_CPP_CFLAGS@
libmodule_el6751_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

