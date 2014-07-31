ACLOCAL_AMFLAGS = -I m4 

bindir = $(prefix)/bin
libdir = $(prefix)/lib/modules/robotkernel
includedir = $(prefix)/include/el6751

lib_LTLIBRARIES = libmodule_el6751.la

libmodule_el6751_la_SOURCES  = module_el6751.cpp el6751.cpp
libmodule_el6751_la_CXXFLAGS = -Wall -I$(top_srcdir) -I$(top_builddir) @ROBOTKERNEL_CFLAGS@ @LN_CFLAGS@ @YAMLCPP_CFLAGS@
libmodule_el6751_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

include_HEADERS = el6751.h \
				  module_el6751.h

