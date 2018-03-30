#ifndef LPC_MEM_DEFS_H__
#define LPC_MEM_DEFS_H__

//SD: Defines for Managing SRAM locations on the LPC



//Aliases for ram banks when using "new"
#define RAM0 (AHB0)
#define STATICRAM0 __attribute__ ((section ("AHBSRAM0")))

#if defined(COMBINE_AHBRAM)
//combined AHB0+1
// keep compatible with existing code using both RAM1 and RAM2
#define RAM1 RAM0
#define STATICRAM1 STATICRAM0
#else

#define RAM1 (AHB1)
#define STATICRAM1 __attribute__ ((section ("AHBSRAM1"),aligned))


#endif


//max_files to 3 - move Reprap, scratchstring and gcodebuffers to ram0
// memory sizes are based on early 1.21RC2 could have change now

//Static Allocations
#define REPRAP_STATIC              STATICRAM0            //Main RepRap Object (936 Bytes)
#define SCRATCHSTRING_STATIC       STATICRAM0            //scratch String Buffer (1760 Bytes)
#define LWIP_STATICRAM             STATICRAM1            //LWIP Memory Pools to use AHB1
#define LWIPHEAP_STATIC            STATICRAM1            //LWIP ram_heap object

//#define TCPSENDINGWINDOW_STATIC    //STATICRAM0             //In Network.cpp
//Testing Allocate Certain Objects into Specified RAM

//Main RepRap Modules
#define PLATFORM_RAM                RAM0       //(1164 Bytes)
#define NETWORK_RAM                 RAM1
#define GCODES_RAM                              //(1012 Bytes)
#define MOVE_RAM                                //(1256 Bytes)
#define HEAT_RAM                               //(128 Bytes)
#define PM_RAM                                 //PrintMonitor RAM (660 Bytes)

//GCode Buffers
//buf 192 bytes
#define GCODEBUFFER_RAM    RAM0//NB: this only includes selected objects: httpGCode, telnetGcode, fileGcode, serialGCode, auxGCode, daemonGCode, and autoPauseGCode from GCodes.cpp

//Platform objects
#define AUXOUT_RAM                          //Aux OutputStack (24 Bytes)
#define USBOUT_RAM                          //USB OutputStack (24 Bytes)
#define MS_RAM                      RAM0   //Mass Storage (6932 Bytes)
#define FILEWRITEBUFFER                    // File WRite Buffer for Mass Storage (?? Bytes)
#define LOG_RAM                           //Logger (24 Bytes)


//Movement Objects
#define DDA_RAM                     RAM0   //DDA Objects (3680 Bytes)
#define DM_RAM                             //Drivemovement object (8800 Bytes)



//new networking
#define NETWORKBUFFER_RAM           /*RAM0*/        //ideally this is in AHB RAM to avoid extra copy.


/*
//networking objects (old networking)
//Following Total 4916Bytes (without LWip)
#define CS_RAM                      RAM1   //ConnectionState
#define NT_RAM                      RAM1   //NetworkTransaction
#define NTOB_RAM                    RAM1   //Output Buffer in NetworkTransaction
#define WEBSERVER_RAM               RAM1   //Webserver
//Webserver Interpreter Objects
#define FTP_RAM                     RAM1
#define HTTP_RAM                    RAM1
#define TELNET_RAM                  RAM1
#define HTTPOS_RAM                  RAM1 // HTTP GCode OutputStack
*/
#endif
