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
	rm -f um245r.device um245r.o
