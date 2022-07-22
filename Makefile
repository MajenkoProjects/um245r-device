CC = m68k-amigaos-gcc
CFLAGS += -m68000 -O2 
LDFLAGS += -m68000  -nostartfiles 
LIBS += 

# For debug uncomment these two lines
CFLAGS += -DDEBUG -mcrt=clib2
LIBS += -ldebug

um245r.device: um245r.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean: 
	rm -f um245r.device um245r.o um245r.adf

um245r.adf: um245r.device
	xdftool um245r.adf create
	xdftool um245r.adf format UM245R
	xdftool um245r.adf write um245r.device
