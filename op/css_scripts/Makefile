CC = gcc
EPICSDIR = /home/peter/epics
BASEDIR = $(EPICSDIR)/base
EXTENDIR = $(EPICSDIR)/extensions
EZCAINCDIR = -I$(EXTENDIR)/include
EPICSINCDIR = -I$(BASEDIR)/include -I$(BASEDIR)/include/os/Linux
INCLUDEDIRS = $(EZCAINCDIR) $(EPICSINCDIR)
EZCALIBDIR = $(EXTENDIR)/lib/linux-x86_64
EZCALIB = -lezca
CALIBDIR = $(BASEDIR)/lib/linux-x86_64
CALIBS = -lca -lCom
LIBDIRS = -L$(CALIBDIR) -L$(EZCALIBDIR)
LIBS = $(EZCALIB) $(CALIBS) -lminpack
CFLAGS = -c -fPIC

testenable: testenable.o
	$(CC) -o testenable testenable.o $(LIBDIRS) $(LIBS)
clean:
	rm -f *.o testenable
.c.o: $*.c
	$(CC) $(CFLAGS) $(INCLUDEDIRS) $*.c
