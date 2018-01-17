/**************************************************************************//**
 * @file     mmu_VE_A9_MP.c
 * @brief    MMU Startup File for VE_A9_MP Device Series
 * @version  V1.01
 * @date     13 June 2014
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 - 2013 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

/*Memory map description from: DUI0447G_v2m_p1_trm.pdf 4.2.2 ARM Cortex-A Series memory map

                                                     Memory Type
0xffffffff |--------------------------|             ------------
           |       FLAG SYNC          |             Device Memory
0xfffff000 |--------------------------|             ------------
           |         Fault            |                Fault
0xfff00000 |--------------------------|             ------------
           |                          |                Normal
           |                          |
           |      Daughterboard       |
           |         memory           |
           |                          |
0x80505000 |--------------------------|             ------------
           |TTB (L2 Sync Flags   ) 4k |                Normal
0x80504C00 |--------------------------|             ------------
           |TTB (L2 Peripherals-B) 16k|                Normal
0x80504800 |--------------------------|             ------------
           |TTB (L2 Peripherals-A) 16k|                Normal
0x80504400 |--------------------------|             ------------
           |TTB (L2 Priv Periphs)  4k |                Normal
0x80504000 |--------------------------|             ------------
           |    TTB (L1 Descriptors)  |                Normal
0x80500000 |--------------------------|             ------------
           |           Heap           |                Normal
           |--------------------------|             ------------
           |          Stack           |                Normal
0x80400000 |--------------------------|             ------------
           |         ZI Data          |                Normal
0x80300000 |--------------------------|             ------------
           |         RW Data          |                Normal
0x80200000 |--------------------------|             ------------
           |         RO Data          |                Normal
           |--------------------------|             ------------
           |         RO Code          |              USH Normal
0x80000000 |--------------------------|             ------------
           |      Daughterboard       |                Fault
           |      HSB AXI buses       |
0x40000000 |--------------------------|             ------------
           |      Daughterboard       |                Fault
           |  test chips peripherals  |
0x2c002000 |--------------------------|             ------------
           |     Private Address      |            Device Memory
0x2c000000 |--------------------------|             ------------
           |      Daughterboard       |                Fault
           |  test chips peripherals  |
0x20000000 |--------------------------|             ------------
           |       Peripherals        |           Device Memory RW/RO
           |                          |              & Fault
0x00000000 |--------------------------|
*/

// L1 Cache info and restrictions about architecture of the caches (CCSIR register):
// Write-Through support *not* available
// Write-Back support available.
// Read allocation support available.
// Write allocation support available.

//Note: You should use the Shareable attribute carefully.
//For cores without coherency logic (such as SCU) marking a region as shareable forces the processor to not cache that region regardless of the inner cache settings.
//CA9-RTX uses LDREX/STREX instructions relying on Local monitors. Local monitors will be used only when the region gets cached, regions that are not cached will use the Global Monitor.
//Some A9 implementations does not include Global Monitors, so wrongly setting the attribute Shareable may cause STREX to fail.

//Recall: When the Shareable attribute is applied to a memory region that is not Write-Back, Normal memory, data held in this region is treated as Non-cacheable.
//When SMP bit = 0, Inner WB/WA Cacheable Shareable attributes are treated as Non-cacheable.
//When SMP bit = 1, Inner WB/WA Cacheable Shareable attributes are treated as Cacheable.


//Following MMU configuration is expected
//SCTLR.AFE == 1 (Simplified access permissions model - AP[2:1] define access permissions, AP[0] is an access flag)
//SCTLR.TRE == 0 (TEX remap disabled, so memory type and attributes are described directly by bits in the descriptor)
//Domain 0 is always the Client domain
//Descriptors should place all memory in domain 0

#include <stdint.h>
#include "VE_A9_MP.h"


//l2 table pointers
//----------------------------------------
#define PRIVATE_TABLE_L2_BASE_4k       (0x80504000) //Map 4k Private Address space
#define SYNC_FLAGS_TABLE_L2_BASE_4k    (0x80504C00) //Map 4k Flag synchronization
#define PERIPHERAL_A_TABLE_L2_BASE_64k (0x80504400) //Map 64k Peripheral #1 0x1C000000 - 0x1C00FFFFF
#define PERIPHERAL_B_TABLE_L2_BASE_64k (0x80504800) //Map 64k Peripheral #2 0x1C100000 - 0x1C1FFFFFF

//--------------------- PERIPHERALS -------------------
#define PERIPHERAL_A_FAULT (0x00000000 + 0x1c000000) //0x1C000000-0x1C00FFFF (1M)
#define PERIPHERAL_B_FAULT (0x00100000 + 0x1c000000) //0x1C100000-0x1C10FFFF (1M)

//--------------------- SYNC FLAGS --------------------
#define FLAG_SYNC     0xFFFFF000
#define F_SYNC_BASE   0xFFF00000  //1M aligned

//Import symbols from linker
extern uint32_t Image$$VECTORS$$Base;
extern uint32_t Image$$RW_DATA$$Base;
extern uint32_t Image$$ZI_DATA$$Base;
extern uint32_t Image$$TTB$$ZI$$Base;

static uint32_t Sect_Normal;     //outer & inner wb/wa, non-shareable, executable, rw, domain 0, base addr 0
static uint32_t Sect_Normal_Cod; //outer & inner wb/wa, non-shareable, executable, ro, domain 0, base addr 0
static uint32_t Sect_Normal_RO;  //as Sect_Normal_Cod, but not executable
static uint32_t Sect_Normal_RW;  //as Sect_Normal_Cod, but writeable and not executable
static uint32_t Sect_Device_RO;  //device, non-shareable, non-executable, ro, domain 0, base addr 0
static uint32_t Sect_Device_RW;  //as Sect_Device_RO, but writeable

/* Define global descriptors */
static uint32_t Page_L1_4k  = 0x0;  //generic
static uint32_t Page_L1_64k = 0x0;  //generic
static uint32_t Page_4k_Device_RW;  //Shared device, not executable, rw, domain 0
static uint32_t Page_64k_Device_RW; //Shared device, not executable, rw, domain 0

void create_translation_table(void)
{
    mmu_region_attributes_Type region;

    //Create 4GB of faulting entries
    __TTSection (&Image$$TTB$$ZI$$Base, 0, 4096, DESCRIPTOR_FAULT);

    /*
     * Generate descriptors. Refer to VE_A9_MP.h to get information about attributes
     *
     */
    //Create descriptors for Vectors, RO, RW, ZI sections
    section_normal(Sect_Normal, region);
    section_normal_cod(Sect_Normal_Cod, region);
    section_normal_ro(Sect_Normal_RO, region);
    section_normal_rw(Sect_Normal_RW, region);
    //Create descriptors for peripherals
    section_device_ro(Sect_Device_RO, region);
    section_device_rw(Sect_Device_RW, region);
    //Create descriptors for 64k pages
    page64k_device_rw(Page_L1_64k, Page_64k_Device_RW, region);
    //Create descriptors for 4k pages
    page4k_device_rw(Page_L1_4k, Page_4k_Device_RW, region);


    /*
     *  Define MMU flat-map regions and attributes
     *
     */

    //Define Image
    __TTSection (&Image$$TTB$$ZI$$Base, (uint32_t)&Image$$VECTORS$$Base, 1, Sect_Normal_Cod);
    __TTSection (&Image$$TTB$$ZI$$Base, (uint32_t)&Image$$RW_DATA$$Base, 1, Sect_Normal_RW);
    __TTSection (&Image$$TTB$$ZI$$Base, (uint32_t)&Image$$ZI_DATA$$Base, 1, Sect_Normal_RW);

    //all DRAM executable, rw, cacheable - applications may choose to divide memory into ro executable
    __TTSection (&Image$$TTB$$ZI$$Base, (uint32_t)&Image$$TTB$$ZI$$Base, 2043, Sect_Normal);

    //--------------------- PERIPHERALS -------------------
    __TTSection (&Image$$TTB$$ZI$$Base, 0x00000000UL    , 1023, Sect_Normal_RW);
    __TTSection (&Image$$TTB$$ZI$$Base, 0xFFFEC000UL    , 64  , Sect_Device_RW);
    __TTSection (&Image$$TTB$$ZI$$Base, VE_A9_MP_SRAM_BASE      , 64, Sect_Device_RW);
    __TTSection (&Image$$TTB$$ZI$$Base, VE_A9_MP_VRAM_BASE      , 32, Sect_Device_RW);
    __TTSection (&Image$$TTB$$ZI$$Base, VE_A9_MP_ETHERNET_BASE  , 16, Sect_Device_RW);
    __TTSection (&Image$$TTB$$ZI$$Base, VE_A9_MP_USB_BASE       , 16, Sect_Device_RW);


    /* Set location of level 1 page table
    ; 31:14 - Translation table base addr (31:14-TTBCR.N, TTBCR.N is 0 out of reset)
    ; 13:7  - 0x0
    ; 6     - IRGN[0] 0x0 (Inner WB WA)
    ; 5     - NOS     0x0 (Non-shared)
    ; 4:3   - RGN     0x1 (Outer WB WA)
    ; 2     - IMP     0x0 (Implementation Defined)
    ; 1     - S       0x0 (Non-shared)
    ; 0     - IRGN[1] 0x1 (Inner WB WA) */
    __set_TTBR0(((uint32_t)&Image$$TTB$$ZI$$Base) | 9);

    /* Set up domain access control register
    ; We set domain 0 to Client and all other domains to No Access.
    ; All translation table entries specify domain 0 */
    __set_DACR(1);
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
