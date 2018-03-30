/* 
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science. 
 * All rights reserved.  
 *  
 * Redistribution and use in source and binary forms, with or without modification,  
 * are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution. 
 * 3. The name of the author may not be used to endorse or promote products 
 *    derived from this software without specific prior written permission.  
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED  
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF  
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT  
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS  
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN  
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY  
 * OF SUCH DAMAGE. 
 * 
 * This file is part of the lwIP TCP/IP stack. 
 *  
 * Author: Adam Dunkels <adam@sics.se> 
 * 
 */ 
#ifndef __CC_H__ 
#define __CC_H__ 

#include <stdint.h>
#include <stdio.h>
//#include <stddef.h> /* for size_t */

#include "LPC/lpc_mem_defs.h" // for our Defines for AHB RAM


/* ARM/LPC17xx is little endian only */
#if !defined(BYTE_ORDER) || (BYTE_ORDER != LITTLE_ENDIAN && BYTE_ORDER != BIG_ENDIAN)
#ifdef BYTE_ORDER
#undef BYTE_ORDER
#endif
#define BYTE_ORDER LITTLE_ENDIAN
#endif

/* Use LWIP error codes */
#define LWIP_PROVIDE_ERRNO

//#if defined(__arm__) && defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6010050)
//    /* Keil uVision4 tools */
//    #define PACK_STRUCT_BEGIN __packed
//    #define PACK_STRUCT_STRUCT
//    #define PACK_STRUCT_END
//    #define PACK_STRUCT_FIELD(fld) fld
//    #define ALIGNED(n)  __align(n)
//#elif defined (__IAR_SYSTEMS_ICC__) 
//    /* IAR Embedded Workbench tools */
//    #define PACK_STRUCT_BEGIN __packed
//    #define PACK_STRUCT_STRUCT
//    #define PACK_STRUCT_END
//    #define PACK_STRUCT_FIELD(fld) fld
//    #define IAR_STR(a) #a
//    #define ALIGNED(n) _Pragma(IAR_STR(data_alignment= ## n ##))
//#else 
//    /* GCC tools (CodeSourcery) */
//    #define PACK_STRUCT_BEGIN
//    #define PACK_STRUCT_STRUCT __attribute__ ((__packed__))
//    #define PACK_STRUCT_END
//    #define PACK_STRUCT_FIELD(fld) fld
//    #define ALIGNED(n)  __attribute__((aligned (n)))
//#endif 

/* Provide Thumb-2 routines for GCC to improve performance */
#if defined(TOOLCHAIN_GCC) && defined(__thumb2__)
    #define MEMCPY(dst,src,len)     thumb2_memcpy(dst,src,len)
    #define LWIP_CHKSUM             thumb2_checksum
    /* Set algorithm to 0 so that unused lwip_standard_chksum function
       doesn't generate compiler warning */
    #define LWIP_CHKSUM_ALGORITHM   0

    void* thumb2_memcpy(void* pDest, const void* pSource, size_t length);
    uint16_t thumb2_checksum(const void* pData, int length);
#else
    /* Used with IP headers only */
    #define LWIP_CHKSUM_ALGORITHM   1
#endif


#ifdef LWIP_DEBUG
//for RRF
extern void debugPrintf(const char *, ...);
#define LWIP_PLATFORM_DIAG(x)   {debugPrintf x;}
#define LWIP_PLATFORM_ASSERT(x) {debugPrintf("Assertion \"%s\" failed at line %d in %s\n", x, __LINE__, __FILE__); while(1);}

#endif //end LWIP_DEBUG

#if TRACE_TO_ASCII_HEX_DUMP
#define TRACE_TO_ASCII_HEX_DUMPF(prefix, len, data) trace_to_ascii_hex_dump(prefix, len, data)
void trace_to_ascii_hex_dump(char* prefix, int len, char *data);
#else
#define TRACE_TO_ASCII_HEX_DUMPF(prefix, len, data) ((void)0)
#endif

//#include "cmsis.h"
#define LWIP_PLATFORM_HTONS(x)      __REV16(x)
#define LWIP_PLATFORM_HTONL(x)      __REV(x)

/* Define the memory area for the lwip's memory pools */

//SD:: Added externs to relocate Seperate Memory Pools to AHB1
extern uint8_t LWIP_STATICRAM memp_memory_TCP_PCB_LISTEN_base[];
extern uint8_t LWIP_STATICRAM memp_memory_PBUF_base[];
extern uint8_t LWIP_STATICRAM memp_memory_REASSDATA_base[];
extern uint8_t LWIP_STATICRAM memp_memory_TCP_SEG_base[];
extern uint8_t LWIP_STATICRAM memp_memory_SYS_TIMEOUT_base[];
extern uint8_t LWIP_STATICRAM memp_memory_IGMP_GROUP_base[];
extern uint8_t LWIP_STATICRAM memp_memory_UDP_PCB_base[];
extern uint8_t LWIP_STATICRAM memp_memory_FRAG_PBUF_base[];
extern uint8_t LWIP_STATICRAM memp_memory_TCP_PCB_base[];
extern uint8_t LWIP_STATICRAM memp_memory_PBUF_POOL_base[];

extern uint8_t LWIP_STATICRAM memp_memory_NETCONN_base[];
extern uint8_t LWIP_STATICRAM memp_memory_NETBUF_base[];
extern uint8_t LWIP_STATICRAM memp_memory_TCPIP_MSG_INPKT_base[];
extern uint8_t LWIP_STATICRAM memp_memory_TCPIP_MSG_API_base[];
extern uint8_t LWIP_STATICRAM memp_memory_PPP_PCB_base[];
extern uint8_t LWIP_STATICRAM memp_memory_PPPOS_PCB_base[];



#endif /* __CC_H__ */ 
