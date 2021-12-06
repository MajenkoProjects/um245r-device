#include <proto/exec.h>
#include <proto/dos.h>
#include <proto/alib.h>

#include <exec/resident.h>
#include <exec/errors.h>
#include <libraries/dos.h>

#include <devices/serial.h>

#if DEBUG
#include <clib/debug_protos.h>
#endif

#include <utility/tagitem.h>
#include <dos/dostags.h>
#include <dos/dosextens.h>

#define STR(s) #s
#define XSTR(s) STR(s)

#define DEVICE_NAME "um245r.device"
#define DEVICE_DATE "(30 Nov 2021)"
#define DEVICE_ID_STRING "um245r " XSTR(DEVICE_VERSION) "." XSTR(DEVICE_REVISION) " " DEVICE_DATE
#define DEVICE_VERSION 1
#define DEVICE_REVISION 0
#define DEVICE_PRIORITY 0

#if defined(DEBUG) 
#define DBG(...) KPrintF("%s:%ld ", __FILE__, __LINE__); KPrintF(__VA_ARGS__) 
#define TRACE DBG("%s\r\n", __FUNCTION__);
#else
#define DBG(...)
#define TRACE
#endif



// This is where I placed my FT245R in memory.
#define FT_BASE (volatile unsigned char *)0xf23000

// The three active bits of the status register
#define FT_PWE 0x01
#define FT_RXF 0x02
#define FT_TXE 0x04

// The default buffer size for the device. Expect this to be changed
// by the software opening the device.
#define FT_BUFSIZ 64

// Special non-standard commands for controlling the communications tasks.
#define CMD_KILLPROC (CMD_NONSTD + 50)
#define CMD_ABORT (CMD_NONSTD + 60)
#define CMD_ABORT_READ (CMD_ABORT + CMD_READ)
#define CMD_ABORT_WRITE (CMD_ABORT + CMD_WRITE)

struct FTUnit {
    struct Unit ft_Unit;
    volatile unsigned char *ft_Status;
    volatile unsigned char *ft_Fifo;
    unsigned char *ft_Buffer;
    unsigned long ft_BufferSize;
    unsigned long ft_Head;
    unsigned long ft_Tail;
    unsigned long ft_Terminator1;
    unsigned long ft_Terminator2;
    unsigned char ft_Flags;
    struct IOExtSer *ft_Reader;
    struct IOExtSer *ft_Writer;
    struct Process *ft_Task;
    volatile struct MsgPort *ft_ReadPort;
    volatile struct MsgPort *ft_WritePort;
    volatile struct MsgPort *ft_CommandPort;
};

struct FTUnit units[1] = {
    { .ft_Status = FT_BASE, .ft_Fifo = FT_BASE + 1 }
};

#define NUM_UNITS (sizeof(units) / sizeof(units[0]))

unsigned long ft_Available(struct FTUnit *);
int ft_Read(struct FTUnit *);
void writeChar(struct FTUnit *, const char);
int ft_SetDefaultOptions(struct FTUnit *);



void syncMsg(struct FTUnit *, unsigned long);

void commsManager();
inline int isTerminator(struct FTUnit *u, char c);


struct ExecBase *SysBase;
struct DosLibrary *DOSBase;
BPTR saved_seg_list;

/*-----------------------------------------------------------
A library or device with a romtag should start with moveq #-1,d0 (to
safely return an error if a user tries to execute the file), followed by a
Resident structure.
------------------------------------------------------------*/
int __attribute__((no_reorder)) _start()
{
    return -1;
}

/*----------------------------------------------------------- 
A romtag structure.  After your driver is brought in from disk, the
disk image will be scanned for this structure to discover magic constants
about you (such as where to start running you from...).

endcode is a marker that shows the end of your code. Make sure it does not
span hunks, and is not before the rom tag! It is ok to put it right after
the rom tag -- that way you are always safe.
Make sure your program has only a single code hunk if you put it at the 
end of your code.
------------------------------------------------------------*/
asm("romtag:                                \n"
    "       dc.w    "XSTR(RTC_MATCHWORD)"   \n"
    "       dc.l    romtag                  \n"
    "       dc.l    endcode                 \n"
    "       dc.b    "XSTR(RTF_AUTOINIT)"    \n"
    "       dc.b    "XSTR(DEVICE_VERSION)"  \n"
    "       dc.b    "XSTR(NT_DEVICE)"       \n"
    "       dc.b    "XSTR(DEVICE_PRIORITY)" \n"
    "       dc.l    _device_name            \n"
    "       dc.l    _device_id_string       \n"
    "       dc.l    _auto_init_tables       \n"
    "endcode:                               \n");

extern void *DUMmySeg;

char device_name[] = DEVICE_NAME;
char device_id_string[] = DEVICE_ID_STRING;

/*------- init_device ---------------------------------------
FOR RTF_AUTOINIT:
  This routine gets called after the device has been allocated.
  The device pointer is in d0. The AmigaDOS segment list is in a0.
  If it returns the device pointer, then the device will be linked
  into the device list.  If it returns NULL, then the device
  will be unloaded.

IMPORTANT:
  If you don't use the "RTF_AUTOINIT" feature, there is an additional
  caveat. If you allocate memory in your Open function, remember that
  allocating memory can cause an Expunge... including an expunge of your
  device. This must not be fatal. The easy solution is don't add your
  device to the list until after it is ready for action.

CAUTION: 
This function runs in a forbidden state !!!                   
This call is single-threaded by Exec
------------------------------------------------------------*/
static struct Library __attribute__((used)) * init_device(BPTR seg_list asm("a0"), struct Library *dev asm("d0")) { TRACE
    /* !!! required !!! save a pointer to exec */
    SysBase = *(struct ExecBase **)4UL;
    DOSBase  = (struct DosLibrary *) OpenLibrary("dos.library",0);

    /* save pointer to our loaded code (the SegList) */
    saved_seg_list = seg_list;

    dev->lib_Node.ln_Type = NT_DEVICE;
    dev->lib_Node.ln_Name = device_name;
    dev->lib_Flags = LIBF_SUMUSED | LIBF_CHANGED;
    dev->lib_Version = DEVICE_VERSION;
    dev->lib_Revision = DEVICE_REVISION;
    dev->lib_IdString = (APTR)device_id_string;

    return dev;
}

/* device dependent expunge function 
!!! CAUTION: This function runs in a forbidden state !!! 
This call is guaranteed to be single-threaded; only one task 
will execute your Expunge at a time. */
static BPTR __attribute__((used)) expunge(struct Library *dev asm("a6")) { TRACE
    if (dev->lib_OpenCnt != 0)
    {
        dev->lib_Flags |= LIBF_DELEXP;
        return 0;
    }

    BPTR seg_list = saved_seg_list;
    Remove(&dev->lib_Node);
    FreeMem((char *)dev - dev->lib_NegSize, dev->lib_NegSize + dev->lib_PosSize);
    return seg_list;
}

/* device dependent open function 
!!! CAUTION: This function runs in a forbidden state !!!
This call is guaranteed to be single-threaded; only one task 
will execute your Open at a time. */
static void __attribute__((used)) open(struct Library *dev asm("a6"), struct IORequest *ioreq asm("a1"), ULONG unitnum asm("d0"), ULONG flags asm("d1")) { TRACE
    struct IOExtSer *sreq = (struct IOExtSer *)ioreq;


    if (unitnum >= NUM_UNITS) {
        sreq->IOSer.io_Error = IOERR_OPENFAIL;
        sreq->IOSer.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
        return;
    }

    struct FTUnit *thisUnit = &units[unitnum];

    if (thisUnit != &units[0]) {
        sreq->IOSer.io_Error = IOERR_OPENFAIL;
        sreq->IOSer.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
        return;
    }

    if (thisUnit->ft_Unit.unit_OpenCnt != 0) {
        sreq->IOSer.io_Error = IOERR_UNITBUSY;
        sreq->IOSer.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
        return;
    }

    thisUnit->ft_Unit.unit_OpenCnt++;

    thisUnit->ft_Buffer = NULL;

    int r = ft_SetDefaultOptions(thisUnit);
    if (r != 0) {
        sreq->IOSer.io_Error = r;
        sreq->IOSer.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
        return;
    }


    thisUnit->ft_Writer = NULL;
    thisUnit->ft_Reader = NULL;
    thisUnit->ft_ReadPort = NULL;
    thisUnit->ft_WritePort = NULL;
    thisUnit->ft_CommandPort = NULL;

    thisUnit->ft_Task = CreateNewProcTags(
        NP_Name, (unsigned long)"FT245R Comms Server",
        NP_Entry, (unsigned long)commsManager,
        TAG_END
    );

    if (thisUnit->ft_Task == NULL) {
        sreq->IOSer.io_Error = IOERR_OPENFAIL;
        sreq->IOSer.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
        return;
    }

    thisUnit->ft_Task->pr_Task.tc_UserData = thisUnit;
    DBG("Sending ^D\r\n");
    Signal(&thisUnit->ft_Task->pr_Task, SIGBREAKF_CTRL_D);

    DBG("Wait for port\r\n");
    while (thisUnit->ft_CommandPort == NULL) {
        Delay(1);
    }

    DBG("System up\r\n");

    ioreq->io_Unit = (struct Unit *)thisUnit;
    

    dev->lib_OpenCnt++;
    sreq->IOSer.io_Error = 0; 
    sreq->IOSer.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
    DBG("Open complete\r\n");
}

/* device dependent close function 
!!! CAUTION: This function runs in a forbidden state !!!
This call is guaranteed to be single-threaded; only one task 
will execute your Close at a time. */
static BPTR __attribute__((used)) close(struct Library *dev asm("a6"), struct IORequest *ioreq asm("a1")) { TRACE
    ioreq->io_Device = NULL;

    struct FTUnit *thisUnit = (struct FTUnit *)ioreq->io_Unit;

    ioreq->io_Unit = NULL;

    thisUnit->ft_Unit.unit_OpenCnt--;

    dev->lib_OpenCnt--;

    if (thisUnit->ft_Unit.unit_OpenCnt == 0) {
        syncMsg(thisUnit, CMD_KILLPROC);

        while (thisUnit->ft_CommandPort != NULL) {
            Delay(1);
        }

        FreeMem((char *)thisUnit->ft_Buffer, thisUnit->ft_BufferSize);

        
    }

    if (dev->lib_OpenCnt == 0 && (dev->lib_Flags & LIBF_DELEXP))
        return expunge(dev);

    return 0;
}

/* device dependent beginio function */
static void __attribute__((used)) begin_io(struct Library *dev asm("a6"), struct IORequest *ioreq asm("a1")) { TRACE
    struct IOExtSer *sreq = (struct IOExtSer *)ioreq;
    unsigned long i;
    const char *ptr;
    sreq->IOSer.io_Error = 0;

    char *data = (char *)(sreq->IOSer.io_Data);
    struct FTUnit *thisUnit = (struct FTUnit *)sreq->IOSer.io_Unit;

    switch (sreq->IOSer.io_Command) {

        case CMD_RESET:
            syncMsg(thisUnit, CMD_ABORT_WRITE);
            syncMsg(thisUnit, CMD_ABORT_READ);
            i = ft_SetDefaultOptions(thisUnit);
            if (i != 0) {
                sreq->IOSer.io_Error = i;
            } 
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

        case CMD_READ:
            sreq->IOSer.io_Actual = 0;

            // If there is enough data in the buffer we'll treat this like
            // an IOF_QUICK request regardless of if it were actually asked
            // for. 
#if 0
            DBG("Read request: %ld bytes\r\n", sreq->IOSer.io_Length);
            if (ft_Available(thisUnit) >= sreq->IOSer.io_Length) {
                Forbid();
                for (i = 0; i < sreq->IOSer.io_Length; i++) {
                    int c = ft_Read(thisUnit);
                    sreq->IOSer.io_Actual++;
                    data[i] = c;
                    if (isTerminator(thisUnit, c) == 1) {
                        break;
                    }
                }
                sreq->IOSer.io_Flags |= IOF_QUICK;
                ReplyMsg(&sreq->IOSer.io_Message);
                Permit();
                return;
            }
#endif

            // Even if we've been asked for IOF_QUICK we should ignore it
            // since there isn't enough data available to directly honour it.
            // Task switching will still be needed to get the data, so blocking
            // here would not have any benefit. Instead we'll just submit the
            // request to the unit and return.
            DBG("Submitting MSG %08lx\r\n", (unsigned long)sreq);
            sreq->IOSer.io_Flags &= ~IOF_QUICK;
            PutMsg(thisUnit->ft_ReadPort, &sreq->IOSer.io_Message);
            return;

        case CMD_WRITE:

            // Here we do want to treat IOF_QUICK specially.
            // If there's no writer in progress then we can just
            // blast the data out through the hardware. If there is
            // a writer in progress then we should block and wait
            // for it to finish first, then we can blast it out.

            if ((sreq->IOSer.io_Flags & IOF_QUICK) == IOF_QUICK) {
                // Only if there isn't a write in progress
                if (thisUnit->ft_Writer == NULL) {
                    for (i = 0; i < sreq->IOSer.io_Length; i++) {
                        writeChar(thisUnit, data[i]);
                    }
                    sreq->IOSer.io_Actual = sreq->IOSer.io_Length;
                    ReplyMsg(&sreq->IOSer.io_Message);
                    return;
                }
            }

            // Any other write we'll just pass straight over to
            // the processing task for this unit.
            
            DBG("Sending write to task\r\n");
            sreq->IOSer.io_Flags &= ~IOF_QUICK;
            PutMsg(thisUnit->ft_WritePort, &sreq->IOSer.io_Message);
            return;

        case CMD_UPDATE: 
            // We don't do aything here. Just treat it as if we did
            // and claim it was done as a quick DoIO call.
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;
        
        case CMD_CLEAR:
            // Here we'll just zap the head and tail of the circular
            // buffer and convert it to a quick call.
            thisUnit->ft_Head = thisUnit->ft_Tail = 0;
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

        case CMD_STOP:
            // Stop doesn't do anything, but we'll pretend it did.
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

        case CMD_START:
            // Start doesn't do anything, but we'll pretend it did.
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

        case CMD_FLUSH:
            // Flush is the same as Clear.
            thisUnit->ft_Head = thisUnit->ft_Tail = 0;
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

        case SDCMD_QUERY:
            DBG("SDCMD_QUERY\r\n");
            // Query will sort of fudge a few signals using the tx
            // and rx fifo indicators from the FT245R.
            sreq->io_Status = (
                (0 << 0) | // Reserved
                (0 << 1) | // Reserved
                (0 << 2) | // RI
                (0 << 3) | // DSR
                (0 << 4) | // CTS
                ((*thisUnit->ft_Status & FT_PWE) ? 1 << 5 : 0) | // CD
                ((*thisUnit->ft_Status & FT_TXE) ? 1 << 6 : 0) | // RTS
                ((*thisUnit->ft_Status & FT_TXE) ? 1 << 7 : 0) | // DTR
                (0 << 8) | // Read Overrun
                (0 << 9) | // Break Sent
                (0 << 10) | // Break Received
                (0 << 11) | // Transmit x-OFFed
                (0 << 12) | // Receive x-OFFed
                (0 << 13) | // Reserved
                (0 << 14) | // Reserved
                (0 << 15) // Reserved
            );
            sreq->IOSer.io_Actual = ft_Available(thisUnit);
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

        case SDCMD_BREAK:
            // Break is meaningless here. We'll just fake it.
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

       case SDCMD_SETPARAMS:

            // If a new buffer size has been requested then zap the old one
            // and allocate a new one. At the moment bad things will happen
            // if there isn't enough memory to allocate.
            if (sreq->io_RBufLen != thisUnit->ft_BufferSize) {
                FreeMem((char *)thisUnit->ft_Buffer, thisUnit->ft_BufferSize);
                thisUnit->ft_Head = thisUnit->ft_Tail = 0;
                thisUnit->ft_BufferSize = sreq->io_RBufLen;
                thisUnit->ft_Buffer = AllocMem(thisUnit->ft_BufferSize, 0);
                if (!thisUnit->ft_Buffer) {
                    // Um... something bad?
                    sreq->IOSer.io_Error = SerErr_BufErr;
                    sreq->IOSer.io_Flags |= IOF_QUICK;
                    ReplyMsg(&sreq->IOSer.io_Message);
                    return;
                }
            }

            // Next update the flags and terminator characters.
            thisUnit->ft_Flags = sreq->io_SerFlags;
            thisUnit->ft_Terminator1 = sreq->io_TermArray.TermArray0;
            thisUnit->ft_Terminator2 = sreq->io_TermArray.TermArray1;

            // Whatever we did this is a fast operation.
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;

        default:
            // We don't know what the request was here, so we'll
            // just pretend like we did it and did it "quick".
            sreq->IOSer.io_Flags |= IOF_QUICK;
            ReplyMsg(&sreq->IOSer.io_Message);
            return;
    }
}

/* device dependent abortio function */
static ULONG __attribute__((used)) abort_io(struct Library *dev asm("a6"), struct IORequest *ioreq asm("a1")) { TRACE
    struct IOExtSer *sreq = (struct IOExtSer *)ioreq;
    sreq->IOSer.io_Flags |= IOF_QUICK;
    struct FTUnit *thisUnit = (struct FTUnit *)ioreq->io_Unit;

    switch (sreq->IOSer.io_Command) {
        case CMD_READ:
            syncMsg(thisUnit, CMD_ABORT_READ);
            break;
        case CMD_WRITE:
            syncMsg(thisUnit, CMD_ABORT_WRITE);
            break;
    }
    

    return 0;
}

static ULONG device_vectors[] =
    {
        (ULONG)open,
        (ULONG)close,
        (ULONG)expunge,
        0, //extFunc not used here
        (ULONG)begin_io,
        (ULONG)abort_io,
        -1}; //function table end marker

/*-----------------------------------------------------------
The romtag specified that we were "RTF_AUTOINIT".  This means
that the RT_INIT structure member points to one of these
tables below. If the AUTOINIT bit was not set then RT_INIT
would point to a routine to run. 

MyDev_Sizeof    data space size
device_vectors  pointer to function initializers
dataTable       pointer to data initializers
init_device     routine to run
------------------------------------------------------------*/
const ULONG auto_init_tables[4] =
    {
        sizeof(struct Library),
        (ULONG)device_vectors,
        0,
        (ULONG)init_device};



/* Main functions below here */



// Test a character to see if it's a termination character
// or not. Always fails (returns 0) if termination checking is
// turned off. 
inline int isTerminator(struct FTUnit *u, char c) {
    if ((u->ft_Flags & SERF_EOFMODE) == 0) return 0;
    if (((u->ft_Terminator1 >> 24) & 0xFF) == c) return 1;
    if (((u->ft_Terminator1 >> 16) & 0xFF) == c) return 1;
    if (((u->ft_Terminator1 >> 8) & 0xFF) == c) return 1;
    if (((u->ft_Terminator1 >> 0) & 0xFF) == c) return 1;
    if (((u->ft_Terminator2 >> 24) & 0xFF) == c) return 1;
    if (((u->ft_Terminator2 >> 16) & 0xFF) == c) return 1;
    if (((u->ft_Terminator2 >> 8) & 0xFF) == c) return 1;
    if (((u->ft_Terminator2 >> 0) & 0xFF) == c) return 1;
    return 0;
}

// Send a message to a unit and block waiting for a reply.
void syncMsg(struct FTUnit *u, unsigned long command) {
    struct IOExtSer msg;

    msg.IOSer.io_Message.mn_ReplyPort = CreateMsgPort();
    msg.IOSer.io_Command = command;
    PutMsg(u->ft_CommandPort, &msg.IOSer.io_Message);
    WaitPort(msg.IOSer.io_Message.mn_ReplyPort);
    GetMsg(msg.IOSer.io_Message.mn_ReplyPort);
    DeleteMsgPort(msg.IOSer.io_Message.mn_ReplyPort);
}

// Return the number of byte available to read in the RX buffer
// of a unit.
inline unsigned long ft_Available(struct FTUnit *u) {
    return (u->ft_BufferSize + u->ft_Head - u->ft_Tail) % u->ft_BufferSize;
}

// Read the next byte from the RX buffer of a unit, or return -1 if
// no data is available to read.
int ft_Read(struct FTUnit *u) {
    unsigned char theChar;
    if (u->ft_Head == u->ft_Tail) {
        return -1;
    } else {
        Forbid();
        theChar = u->ft_Buffer[u->ft_Tail];
        u->ft_Tail = (u->ft_Tail + 1) % u->ft_BufferSize;
        Permit();
        return theChar;
    }
}

int ft_SetDefaultOptions(struct FTUnit *u) { TRACE
    if (u->ft_Buffer != NULL) {
        FreeMem((char *)u->ft_Buffer, u->ft_BufferSize);
    }
    u->ft_Buffer = AllocMem(FT_BUFSIZ, 0);
    if (u->ft_Buffer == NULL) {
        return SerErr_BufErr;
    }
    u->ft_BufferSize = FT_BUFSIZ;

    u->ft_Flags = 0x84;
    u->ft_Terminator1 = 0x00;
    u->ft_Terminator2 = 0x00;
    return 0;
}

// Pump a single byte out to the hardware for a unit.
void writeChar(struct FTUnit *u, char c) {
    // Wait for space in the fifo
    while ((*u->ft_Status & FT_TXE) != 0);
    // Send the character
    *u->ft_Fifo = c;
}

// This is the main processing routine. It is spawned once
// per unit and sits looking for incoming data and processes
// any messages sent to it by the device driver.
void commsManager() { //TRACE

    struct IOExtSer *msg;   // The current incoming message cast as IOExtSer
    char done = 0;          // Flag to allow termination of the main loop
    char didSomething = 0;  // Has something been processed in this pass?

    // Wait for the signal that userdata is set
    //DBG("Wait for ^D\r\n");
    Wait(SIGBREAKF_CTRL_D);
    //DBG("^D received\r\n");

    // Get the unit structure for this task
    struct Task *self = FindTask(NULL);
    struct FTUnit *thisUnit = self->tc_UserData;

    if (thisUnit != &units[0]) {
        return;
    }

    //DBG("Make ports\r\n");

    // Create a fresh message port. It is (supposedly) important that
    // the task waiting on the port creates the port, which is why we're
    // not using the default unit port.
    Forbid();
    thisUnit->ft_ReadPort = CreateMsgPort();
    thisUnit->ft_WritePort = CreateMsgPort();
    thisUnit->ft_CommandPort = CreateMsgPort();
    Permit();

    //DBG("Ports made\r\n");


    while(!done) {

        didSomething = 0;

        // The first thing to do is grab a byte from the FT245R's FIFO if there
        // is anything available, and of course only if there is room in the RX
        // buffer to store it.
        if ((*thisUnit->ft_Status & FT_RXF) == 0) { // There is something to read
            Forbid();
            unsigned long bufIndex = (thisUnit->ft_Head + 1) % thisUnit->ft_BufferSize;
            if (bufIndex != thisUnit->ft_Tail) { // There is room to read it into the buffer
                char c = *thisUnit->ft_Fifo;
                DBG("RX %ld\r\n", c);
                thisUnit->ft_Buffer[thisUnit->ft_Head] = c;
                thisUnit->ft_Head = bufIndex;
                didSomething = 1;
                DBG("Avail: %ld\r\n", ft_Available(thisUnit));
            }
            Permit();
        }

        // Now we'll get some data for the active read message if there is one.
        if (thisUnit->ft_Reader != NULL) {
            if (ft_Available(thisUnit) > 0) {
                unsigned char *data = (unsigned char *)thisUnit->ft_Reader->IOSer.io_Data;
                int c = ft_Read(thisUnit);
                data[thisUnit->ft_Reader->IOSer.io_Actual++] = c;
                DBG("Add %ld\r\n", c);
                if (isTerminator(thisUnit, c) == 1) {
                    ReplyMsg(&thisUnit->ft_Reader->IOSer.io_Message);
                    thisUnit->ft_Reader = NULL;
                } else if (thisUnit->ft_Reader->IOSer.io_Actual == thisUnit->ft_Reader->IOSer.io_Length) {
                    ReplyMsg(&thisUnit->ft_Reader->IOSer.io_Message);
                    DBG("Rd done: %ld\r\n", thisUnit->ft_Reader->IOSer.io_Actual);
                    thisUnit->ft_Reader = NULL;
                }
                didSomething = 1;
            }
        } else { // Look for a new message
            thisUnit->ft_Reader = (struct IOExtSer *)GetMsg(thisUnit->ft_ReadPort);
            if (thisUnit->ft_Reader != NULL) {
                thisUnit->ft_Reader->IOSer.io_Actual = 0;
                DBG("New reader\r\n");
            }
        }

        // And if there's an active write message we'll send the next byte.
        if (thisUnit->ft_Writer != NULL) {
            unsigned char *data = (unsigned char *)thisUnit->ft_Writer->IOSer.io_Data;
            writeChar(thisUnit, data[thisUnit->ft_Writer->IOSer.io_Actual++]);

            if (thisUnit->ft_Writer->IOSer.io_Length == -1) {
                if (data[thisUnit->ft_Writer->IOSer.io_Actual] == 0) {
                    ReplyMsg(&thisUnit->ft_Writer->IOSer.io_Message);
                    thisUnit->ft_Writer = NULL;
                }
            } else if (thisUnit->ft_Writer->IOSer.io_Actual >= thisUnit->ft_Writer->IOSer.io_Length) {
                ReplyMsg(&thisUnit->ft_Writer->IOSer.io_Message);
                thisUnit->ft_Writer = NULL;
            }
            didSomething = 1;
        } else { // Look for a new message
            thisUnit->ft_Writer = (struct IOExtSer *)GetMsg(thisUnit->ft_WritePort);
            if (thisUnit->ft_Writer != NULL) {
                thisUnit->ft_Writer->IOSer.io_Actual = 0;
            }
        }

        msg = (struct IOExtSer *)GetMsg(thisUnit->ft_CommandPort);
        if (msg != NULL) {

            switch (msg->IOSer.io_Command) {

                case CMD_ABORT_READ:
                    // A request to abort the current read operation. Terminate the read
                    // message and error it with an ABORTED error.
                    if (thisUnit->ft_Reader != NULL) { 
                        thisUnit->ft_Reader->IOSer.io_Error = IOERR_ABORTED;
                        ReplyMsg(&thisUnit->ft_Reader->IOSer.io_Message);
                        thisUnit->ft_Reader = NULL;
                    }
                    ReplyMsg(&msg->IOSer.io_Message);
                    break;

                case CMD_ABORT_WRITE:
                    // And the same with a write abort request.
                    if (thisUnit->ft_Writer != NULL) { 
                        thisUnit->ft_Writer->IOSer.io_Error = IOERR_ABORTED;
                        ReplyMsg(&thisUnit->ft_Writer->IOSer.io_Message);
                        thisUnit->ft_Writer = NULL;
                    }
                    ReplyMsg(&msg->IOSer.io_Message);
                    break;

                case CMD_KILLPROC:
                    // This will request the termination of this task. It basically
                    // means stop executing the loop and fall through to finish
                    // the function off.
                    ReplyMsg(&msg->IOSer.io_Message);
                    done = 1;
                    break;
        
            }
            didSomething = 1;
        }

        // If nothing was done during this pass we'll introduce a very small delay.
        // This might (though don't quote me as I don't understand the scheduler)
        // allow other tasks more processing and "lighten" this one while idling.
        if (didSomething == 0) { 
//            DBG("Tick\r\n");
//            Delay(1);
        }
    }


    // We're all done now, so we'll delete the message port we made
    DeleteMsgPort(thisUnit->ft_WritePort);
    DeleteMsgPort(thisUnit->ft_ReadPort);
    DeleteMsgPort(thisUnit->ft_CommandPort);
    // and NULL the pointer out so the calling process can see we've finished.
    thisUnit->ft_WritePort = NULL;
    thisUnit->ft_ReadPort = NULL;
    thisUnit->ft_CommandPort = NULL;
//    Wait(0);
}


