/**************************************************************************//**
 * @file     system_VE_A9_MP.c
 * @brief    CMSIS Device System Source File for
 *           ARMCA9 Device Series
 * @version  V1.00
 * @date     24 July 2013
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2011 - 2013 ARM LIMITED

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


#include <stdint.h>
#include "VE_A9_MP.h"

extern void PendSV_Handler(uint32_t);
extern void OS_Tick_Handler(uint32_t);
extern void task_accum_func(void);
extern void tracking(void);
extern void $Super$$main(void);

__asm void FPUEnable(void);

/**
 * Initialize the cache.
 *
 * @param  none
 * @return none
 *
 * @brief Initialise caches. Requires PL1, so implemented as an SVC in case threads are USR mode.
 */
#pragma push
#pragma arm

void __svc(1) InitMemorySubsystem(void);
void __SVC_1(void) {

    /* This SVC is specific for reset where data / tlb / btac may contain undefined data, therefore before
     * enabling the cache you must invalidate the instruction cache, the data cache, TLB, and BTAC.
     * You are not required to invalidate the main TLB, even though it is recommended for safety
     * reasons. This ensures compatibility with future revisions of the processor. */

    /* Invalidate undefined data */
    __ca9u_inv_tlb_all();
    __v7_inv_icache_all();
    __v7_inv_dcache_all();
    __v7_inv_btac();

    /* Don't use this function during runtime since caches may contain valid data. For a correct cache maintenance you may need to execute a clean and
     * invalidate in order to flush the valid data to the next level cache.
     */
    //__enable_mmu();//!!!!temporary flat mapping;

    /* After MMU is enabled and data has been invalidated, enable caches and BTAC */
    __enable_caches();
    __enable_btac();

    /* If present, you may also need to Invalidate and Enable L2 cache here */
}
#pragma pop

IRQHandler IRQTable[] = {
    0, //IRQ 0
    0, //IRQ 1
    0, //IRQ 2
    0, //IRQ 3
    0, //IRQ 4
    0, //IRQ 5
    0, //IRQ 6
    0, //IRQ 7
    0, //IRQ 8
    0, //IRQ 9
    0, //IRQ 10
    0, //IRQ 11
    0, //IRQ 12
    0, //IRQ 13
    0, //IRQ 14
    0, //IRQ 15
    0, //IRQ 16
    0, //IRQ 17
    0, //IRQ 18
    0, //IRQ 19
    0, //IRQ 20
    0, //IRQ 21
    0, //IRQ 22
    0, //IRQ 23
    0, //IRQ 24
    0, //IRQ 25
    0, //IRQ 26
    0, //IRQ 27
    0, //IRQ 28
    0, //IRQ 29
    0, //IRQ 30
    0, //IRQ 31
    0, //IRQ 32
    0, //IRQ 33
    0, //IRQ 34
    0, //IRQ 35
    0, //IRQ 36
    0, //IRQ 37
    0, //IRQ 38
    0, //IRQ 39
    0, //IRQ 40
	0, //IRQ 41
	0, //IRQ 42
	0, //IRQ 43
	0, //IRQ 44
	0, //IRQ 45
	0, //IRQ 46
	0, //IRQ 47
	0, //IRQ 48
	0, //IRQ 49
	0, //IRQ 50
	0, //IRQ 51
	0, //IRQ 52
	0, //IRQ 53
	0, //IRQ 54
	0, //IRQ 55
	0, //IRQ 56
	0, //IRQ 57
	0, //IRQ 58
	0, //IRQ 59
	0, //IRQ 50
	0, //IRQ 61
	0, //IRQ 62
	0, //IRQ 63
	0, //IRQ 64
	0, //IRQ 65
	0, //IRQ 66
	0, //IRQ 67
	0, //IRQ 68
	0, //IRQ 69
	0, //IRQ 70
	0, //IRQ 71
	0, //IRQ 72
	0, //IRQ 73
	0, //IRQ 74
	0, //IRQ 75
	0, //IRQ 76
	0, //IRQ 77
	0, //IRQ 78
	0, //IRQ 79
	0  //IRQ 80
};
uint32_t IRQCount = sizeof IRQTable / 4;

uint32_t InterruptHandlerRegister (IRQn_Type irq, IRQHandler handler)
{
    if (irq < IRQCount) {
        IRQTable[irq] = handler;
        return 0;
    }
    else {
        return 1;
    }
}

uint32_t InterruptHandlerUnregister (IRQn_Type irq)
{
    if (irq < IRQCount) {
        IRQTable[irq] = 0;
        return 0;
    }
    else {
        return 1;
    }
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
/*       do not use global variables because this function is called before
         reaching pre-main. RW section may be overwritten afterwards.          */
    GIC_Enable();
}

void $Sub$$main(void)
{
    InterruptHandlerRegister(SGI0_IRQn      , PendSV_Handler);
    InterruptHandlerRegister(PrivTimer_IRQn , OS_Tick_Handler);
    InterruptHandlerRegister(tacking_IRQ	, tracking);
    InitMemorySubsystem();
    $Super$$main(); //Call main
}

//Fault Status Register (IFSR/DFSR) definitions
#define FSR_ALIGNMENT_FAULT                  0x01   //DFSR only. Fault on first lookup
#define FSR_INSTRUCTION_CACHE_MAINTAINANCE   0x04   //DFSR only - async/external
#define FSR_SYNC_EXT_TTB_WALK_FIRST          0x0c   //sync/external
#define FSR_SYNC_EXT_TTB_WALK_SECOND         0x0e   //sync/external
#define FSR_SYNC_PARITY_TTB_WALK_FIRST       0x1c   //sync/external
#define FSR_SYNC_PARITY_TTB_WALK_SECOND      0x1e   //sync/external
#define FSR_TRANSLATION_FAULT_FIRST          0x05   //MMU Fault - internal
#define FSR_TRANSLATION_FAULT_SECOND         0x07   //MMU Fault - internal
#define FSR_ACCESS_FLAG_FAULT_FIRST          0x03   //MMU Fault - internal
#define FSR_ACCESS_FLAG_FAULT_SECOND         0x06   //MMU Fault - internal
#define FSR_DOMAIN_FAULT_FIRST               0x09   //MMU Fault - internal
#define FSR_DOMAIN_FAULT_SECOND              0x0b   //MMU Fault - internal
#define FSR_PERMISION_FAULT_FIRST            0x0f   //MMU Fault - internal
#define FSR_PERMISION_FAULT_SECOND           0x0d   //MMU Fault - internal
#define FSR_DEBUG_EVENT                      0x02   //internal
#define FSR_SYNC_EXT_ABORT                   0x08   //sync/external
#define FSR_TLB_CONFLICT_ABORT               0x10   //sync/external
#define FSR_LOCKDOWN                         0x14   //internal
#define FSR_COPROCESSOR_ABORT                0x1a   //internal
#define FSR_SYNC_PARITY_ERROR                0x19   //sync/external
#define FSR_ASYNC_EXTERNAL_ABORT             0x16   //DFSR only - async/external
#define FSR_ASYNC_PARITY_ERROR               0x18   //DFSR only - async/external

void CDAbtHandler(uint32_t DFSR, uint32_t DFAR, uint32_t LR) {
    uint32_t FS = (DFSR & (1 << 10)) >> 6 | (DFSR & 0x0f); //Store Fault Status

    switch(FS) {
        //Synchronous parity errors - retry
        case FSR_SYNC_PARITY_ERROR:
        case FSR_SYNC_PARITY_TTB_WALK_FIRST:
        case FSR_SYNC_PARITY_TTB_WALK_SECOND:
            return;

        //Your code here. Value in DFAR is invalid for some fault statuses.
        case FSR_ALIGNMENT_FAULT:
        case FSR_INSTRUCTION_CACHE_MAINTAINANCE:
        case FSR_SYNC_EXT_TTB_WALK_FIRST:
        case FSR_SYNC_EXT_TTB_WALK_SECOND:
        case FSR_TRANSLATION_FAULT_FIRST:
        case FSR_TRANSLATION_FAULT_SECOND:
        case FSR_ACCESS_FLAG_FAULT_FIRST:
        case FSR_ACCESS_FLAG_FAULT_SECOND:
        case FSR_DOMAIN_FAULT_FIRST:
        case FSR_DOMAIN_FAULT_SECOND:
        case FSR_PERMISION_FAULT_FIRST:
        case FSR_PERMISION_FAULT_SECOND:
        case FSR_DEBUG_EVENT:
        case FSR_SYNC_EXT_ABORT:
        case FSR_TLB_CONFLICT_ABORT:
        case FSR_LOCKDOWN:
        case FSR_COPROCESSOR_ABORT:
        case FSR_ASYNC_EXTERNAL_ABORT: //DFAR invalid
        case FSR_ASYNC_PARITY_ERROR:   //DFAR invalid
        default:
            while(1);
    }
}

void CPAbtHandler(uint32_t IFSR, uint32_t IFAR, uint32_t LR) {
    uint32_t FS = (IFSR & (1 << 10)) >> 6 | (IFSR & 0x0f); //Store Fault Status

    switch(FS) {
        //Synchronous parity errors - retry
        case FSR_SYNC_PARITY_ERROR:
        case FSR_SYNC_PARITY_TTB_WALK_FIRST:
        case FSR_SYNC_PARITY_TTB_WALK_SECOND:
            return;

        //Your code here. Value in IFAR is invalid for some fault statuses.
        case FSR_SYNC_EXT_TTB_WALK_FIRST:
        case FSR_SYNC_EXT_TTB_WALK_SECOND:
        case FSR_TRANSLATION_FAULT_FIRST:
        case FSR_TRANSLATION_FAULT_SECOND:
        case FSR_ACCESS_FLAG_FAULT_FIRST:
        case FSR_ACCESS_FLAG_FAULT_SECOND:
        case FSR_DOMAIN_FAULT_FIRST:
        case FSR_DOMAIN_FAULT_SECOND:
        case FSR_PERMISION_FAULT_FIRST:
        case FSR_PERMISION_FAULT_SECOND:
        case FSR_DEBUG_EVENT: //IFAR invalid
        case FSR_SYNC_EXT_ABORT:
        case FSR_TLB_CONFLICT_ABORT:
        case FSR_LOCKDOWN:
        case FSR_COPROCESSOR_ABORT:
        default:
            while(1);
    }
}

//returns amount to decrement lr by
//this will be 0 when we have emulated the instruction and simply want to execute the next instruction
//this will be 2 when we have performed some maintenance and want to retry the instruction in thumb (state == 2)
//this will be 4 when we have performed some maintenance and want to retry the instruction in arm (state == 4)
uint32_t CUndefHandler(uint32_t opcode, uint32_t state, uint32_t LR) {
    const int THUMB = 2;
    const int ARM = 4;
    //Lazy VFP/NEON initialisation and switching
    if ((state == ARM   && ((opcode & 0x0C000000)) >> 26 == 0x03) ||
        (state == THUMB && ((opcode & 0xEC000000)) >> 26 == 0x3B)) {
        if (((opcode & 0x00000E00) >> 9) == 5) { //fp instruction?
            FPUEnable();
            return state;
        }
    }

    //Add code here for other Undef cases
    while(1);
}

#pragma push
#pragma arm
//Critical section, called from undef handler, so systick is disabled
__asm void FPUEnable(void) {
        ARM

        //Permit access to VFP registers by modifying CPACR
        MRC     p15,0,R1,c1,c0,2
        ORR     R1,R1,#0x00F00000
        MCR     p15,0,R1,c1,c0,2

        //Ensure that subsequent instructions occur in the context of VFP access permitted
        ISB

        //Enable VFP
        VMRS    R1,FPEXC
        ORR     R1,R1,#0x40000000
        VMSR    FPEXC,R1

        //Initialise VFP registers to 0
        MOV     R2,#0
        VMOV    D0, R2,R2
        VMOV    D1, R2,R2
        VMOV    D2, R2,R2
        VMOV    D3, R2,R2
        VMOV    D4, R2,R2
        VMOV    D5, R2,R2
        VMOV    D6, R2,R2
        VMOV    D7, R2,R2
        VMOV    D8, R2,R2
        VMOV    D9, R2,R2
        VMOV    D10,R2,R2
        VMOV    D11,R2,R2
        VMOV    D12,R2,R2
        VMOV    D13,R2,R2
        VMOV    D14,R2,R2
        VMOV    D15,R2,R2

        //Initialise FPSCR to a known state
        VMRS    R2,FPSCR
        LDR     R3,=0x00086060 //Mask off all bits that do not have to be preserved. Non-preserved bits can/should be zero.
        AND     R2,R2,R3
        VMSR    FPSCR,R2

        BX      LR
}
#pragma pop
