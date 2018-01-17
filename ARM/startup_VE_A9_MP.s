;/**************************************************************************//**
; * @file     startup_VE_A9_MP.s
; * @brief    CMSIS Core Device Startup File for
; *           VE_A9_MP Device Series
; * @version  V1.00
; * @date     2 Aug 2013
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2011 - 2013 ARM LIMITED
;
;   All rights reserved.
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions are met:
;   - Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;   - Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;   - Neither the name of ARM nor the names of its contributors may be used
;     to endorse or promote products derived from this software without
;     specific prior written permission.
;   *
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
;   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;   ---------------------------------------------------------------------------*/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/

ICCIAR_OFFSET   EQU     0x010C
ICCEOIR_OFFSET  EQU     0x0110
ICDISER0_OFFSET EQU     0x1100
ICDICER0_OFFSET EQU     0x1180

Mode_USR        EQU     0x10
Mode_FIQ        EQU     0x11
Mode_IRQ        EQU     0x12
Mode_SVC        EQU     0x13
Mode_ABT        EQU     0x17
Mode_UND        EQU     0x1B
Mode_SYS        EQU     0x1F

I_Bit           EQU     0x80            ; when I bit is set, IRQ is disabled
F_Bit           EQU     0x40            ; when F bit is set, FIQ is disabled
T_Bit           EQU     0x20            ; when T bit is set, core is in Thumb state

 

Sect_Normal     EQU     0x00005c06 ;outer & inner wb/wa, non-shareable, executable, rw, domain 0, base addr 0
Sect_Normal_Cod EQU     0x0000dc06 ;outer & inner wb/wa, non-shareable, executable, ro, domain 0, base addr 0
Sect_Normal_RO  EQU     0x0000dc16 ;as Sect_Normal_Cod, but not executable
Sect_Normal_RW  EQU     0x00005c16 ;as Sect_Normal_Cod, but writeable and not executable
Sect_SO         EQU     0x00000c12 ;strongly-ordered (therefore shareable), not executable, rw, domain 0, base addr 0
Sect_Device_RO  EQU     0x00008c12 ;device, non-shareable, non-executable, ro, domain 0, base addr 0
Sect_Device_RW  EQU     0x00000c12 ;as Sect_Device_RO, but writeable
Sect_Fault      EQU     0x00000000 ;this translation will fault (the bottom 2 bits are important, the rest are ignored)

RAM_BASE        EQU     0x80000000
VRAM_BASE       EQU     0x18000000
SRAM_BASE       EQU     0x2e000000
ETHERNET        EQU     0x1a000000
CS3_PERIPHERAL_BASE EQU 0x1c000000

; <h> Stack Configuration
;   <o> Stack Size (in Bytes, per mode) <0x0-0xFFFFFFFF:8>
; </h>

UND_Stack_Size  EQU     0x00000100
SVC_Stack_Size  EQU     0x00000100
ABT_Stack_Size  EQU     0x00000100
FIQ_Stack_Size  EQU     0x00000000
IRQ_Stack_Size  EQU     0x00000100
USR_Stack_Size  EQU     0x00000100

ISR_Stack_Size  EQU     (UND_Stack_Size + SVC_Stack_Size + ABT_Stack_Size + \
                         FIQ_Stack_Size + IRQ_Stack_Size)

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   USR_Stack_Size
__initial_sp    SPACE   ISR_Stack_Size

Stack_Top


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                ARM


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, CODE, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       LDR     PC, Reset_Addr            ; Address of Reset Handler
                LDR     PC, Undef_Addr            ; Address of Undef Handler
                LDR     PC, SVC_Addr              ; Address of SVC Handler
                LDR     PC, PAbt_Addr             ; Address of Prefetch Abort Handler
                LDR     PC, DAbt_Addr             ; Address of Data Abort Handler
                NOP                               ; Reserved Vector
                LDR     PC, IRQ_Addr              ; Address of IRQ Handler
                LDR     PC, FIQ_Addr              ; Address of FIQ Handler
__Vectors_End

__Vectors_Size 	EQU 	__Vectors_End - __Vectors

Reset_Addr      DCD     Reset_Handler
Undef_Addr      DCD     Undef_Handler
SVC_Addr        DCD     SVC_Handler
PAbt_Addr       DCD     PAbt_Handler
DAbt_Addr       DCD     DAbt_Handler
IRQ_Addr        DCD     IRQ_Handler
FIQ_Addr        DCD     FIQ_Handler

                AREA    |.text|, CODE, READONLY

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                ; Put any cores other than 0 to sleep
                MRC     p15, 0, R0, c0, c0, 5     ; Read MPIDR
                ANDS    R0, R0, #3
goToSleep
                WFINE
                BNE     goToSleep

                MRC     p15, 0, R0, c1, c0, 0       ; Read CP15 System Control register
                BIC     R0, R0, #(0x1 << 12)        ; Clear I bit 12 to disable I Cache
                BIC     R0, R0, #(0x1 <<  2)        ; Clear C bit  2 to disable D Cache
                BIC     R0, R0, #0x1                ; Clear M bit  0 to disable MMU
                BIC     R0, R0, #(0x1 << 11)        ; Clear Z bit 11 to disable branch prediction
                MCR     p15, 0, R0, c1, c0, 0       ; Write value back to CP15 System Control register
                ISB

; Set Vector Base Address Register (VBAR) to point to this application's vector table
                LDR     R0, =__Vectors
                MCR     p15, 0, R0, c12, c0, 0

;  Setup Stack for each exceptional mode
                LDR     R0, =Stack_Top

;  Enter Undefined Instruction Mode and set its Stack Pointer
                MSR     CPSR_C, #Mode_UND:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #UND_Stack_Size

;  Enter Abort Mode and set its Stack Pointer
                MSR     CPSR_C, #Mode_ABT:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #ABT_Stack_Size

;  Enter FIQ Mode and set its Stack Pointer
                MSR     CPSR_C, #Mode_FIQ:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #FIQ_Stack_Size

;  Enter IRQ Mode and set its Stack Pointer
                MSR     CPSR_C, #Mode_IRQ:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #IRQ_Stack_Size

;  Enter Supervisor Mode and set its Stack Pointer
                MSR     CPSR_C, #Mode_SVC:OR:I_Bit:OR:F_Bit
                MOV     SP, R0

;  Enter System Mode to complete initialization and enter kernel
                MSR     CPSR_C, #Mode_SYS:OR:I_Bit:OR:F_Bit
                MOV     SP, R0

                ISB

                IMPORT  create_translation_table
                BL      create_translation_table

;  USR/SYS stack pointer will be set during kernel init

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BLX     R0

                ENDP

Undef_Handler\
                PROC
                EXPORT  Undef_Handler             [WEAK]
                IMPORT  CUndefHandler
                SRSFD   SP!, #Mode_UND
                PUSH    {R0-R4, R12}              ; Save APCS corruptible registers to UND mode stack

                MRS     R0, SPSR
                TST     R0, #T_Bit                ; Check mode
                MOVEQ   R1, #4                    ; R1 = 4 ARM mode
                MOVNE   R1, #2                    ; R1 = 2 Thumb mode
                SUB     R0, LR, R1
                LDREQ   R0, [R0]                  ; ARM mode - R0 points to offending instruction
                BEQ     undef_cont

                ;Thumb instruction
                ;Determine if it is a 32-bit Thumb instruction
                LDRH    R0, [R0]
                MOV     R2, #0x1c
                CMP     R2, R0, LSR #11
                BHS     undef_cont                ;16-bit Thumb instruction

                ;32-bit Thumb instruction. Unaligned - we need to reconstruct the offending instruction.
                LDRH    R2, [LR]
                ORR     R0, R2, R0, LSL #16
undef_cont
                MOV     R2, LR                    ; Set LR to third argument

                AND     R12, SP, #4               ; Ensure stack is 8-byte aligned
                SUB     SP, SP, R12               ; Adjust stack
                PUSH    {R12, LR}                 ; Store stack adjustment and dummy LR

                ;R0 Offending instruction
                ;R1 =2 (Thumb) or =4 (ARM)
                BL      CUndefHandler

                POP     {R12, LR}                 ; Get stack adjustment & discard dummy LR
                ADD     SP, SP, R12               ; Unadjust stack

                LDR     LR, [SP, #24]             ; Restore stacked LR and possibly adjust for retry
                SUB     LR, LR, R0
                LDR     R0, [SP, #28]             ; Restore stacked SPSR
                MSR     SPSR_CXSF, R0
                POP     {R0-R4, R12}              ; Restore stacked APCS registers
                ADD     SP, SP, #8                ; Adjust SP for already-restored banked registers
                MOVS    PC, LR
                ENDP

PAbt_Handler\
                PROC
                EXPORT  PAbt_Handler              [WEAK]
                IMPORT  CPAbtHandler
                SUB     LR, LR, #4                ; Pre-adjust LR
                SRSFD   SP!, #Mode_ABT            ; Save LR and SPRS to ABT mode stack
                PUSH    {R0-R4, R12}              ; Save APCS corruptible registers to ABT mode stack
                MRC     p15, 0, R0, c5, c0, 1     ; IFSR
                MRC     p15, 0, R1, c6, c0, 2     ; IFAR

                MOV     R2, LR                    ; Set LR to third argument

                AND     R12, SP, #4               ; Ensure stack is 8-byte aligned
                SUB     SP, SP, R12               ; Adjust stack
                PUSH    {R12, LR}                 ; Store stack adjustment and dummy LR

                BL      CPAbtHandler

                POP     {R12, LR}                 ; Get stack adjustment & discard dummy LR
                ADD     SP, SP, R12               ; Unadjust stack

                POP     {R0-R4, R12}              ; Restore stack APCS registers
                RFEFD   SP!                       ; Return from exception
                ENDP


DAbt_Handler\
                PROC
                EXPORT  DAbt_Handler              [WEAK]
                IMPORT  CDAbtHandler
                SUB     LR, LR, #8                ; Pre-adjust LR
                SRSFD   SP!, #Mode_ABT            ; Save LR and SPRS to ABT mode stack
                PUSH    {R0-R4, R12}              ; Save APCS corruptible registers to ABT mode stack
                CLREX                             ; State of exclusive monitors unknown after taken data abort
                MRC     p15, 0, R0, c5, c0, 0     ; DFSR
                MRC     p15, 0, R1, c6, c0, 0     ; DFAR

                MOV     R2, LR                    ; Set LR to third argument

                AND     R12, SP, #4               ; Ensure stack is 8-byte aligned
                SUB     SP, SP, R12               ; Adjust stack
                PUSH    {R12, LR}                 ; Store stack adjustment and dummy LR

                BL      CDAbtHandler

                POP     {R12, LR}                 ; Get stack adjustment & discard dummy LR
                ADD     SP, SP, R12               ; Unadjust stack

                POP     {R0-R4, R12}              ; Restore stacked APCS registers
                RFEFD   SP!                       ; Return from exception
                ENDP

FIQ_Handler\
                PROC
                EXPORT  FIQ_Handler               [WEAK]
                B       .
                ENDP

SVC_Handler\
                PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP

IRQ_Handler\
                PROC
                EXPORT  IRQ_Handler                [WEAK]
                IMPORT  IRQCount
                IMPORT  IRQTable
                IMPORT  IRQNestLevel

                ;prologue
                SUB     LR, LR, #4                  ; Pre-adjust LR
                SRSFD   SP!, #Mode_SVC              ; Save LR_IRQ and SPRS_IRQ to SVC mode stack
                CPS     #Mode_SVC                   ; Switch to SVC mode, to avoid a nested interrupt corrupting LR on a BL
                PUSH    {R0-R3, R12}                ; Save remaining APCS corruptible registers to SVC stack

                AND     R1, SP, #4                  ; Ensure stack is 8-byte aligned
                SUB     SP, SP, R1                  ; Adjust stack
                PUSH    {R1, LR}                    ; Store stack adjustment and LR_SVC to SVC stack

                LDR     R0, =IRQNestLevel           ; Get address of nesting counter
                LDR     R1, [R0]
                ADD     R1, R1, #1                  ; Increment nesting counter
                STR     R1, [R0]

                ;identify and acknowledge interrupt
                MRC     p15, 4, R1, c15, c0, 0      ; Read CBAR
                LDR     R0, [R1, #ICCIAR_OFFSET]    ; Read ICCIAR (GIC CPU Interface register)
                DSB                                 ; Ensure that interrupt acknowledge completes before re-enabling interrupts
                LDR     R2, =IRQCount               ; Read number of IRQs
                LDR     R2, [R2]
                CMP     R0, R2                      ; Clean up and return if no handler
                BHS     ret_irq                     ; In a single-processor system, spurious interrupt ID 1023 does not need any special handling
                LDR     R2, =IRQTable               ; Get address of handler
                LDR     R2, [R2, R0, LSL #2]
                CMP     R2, #0                      ; Clean up and return if handler address is 0
                BEQ     ret_irq
                PUSH    {R0,R1}

                CPSIE   i                           ; Now safe to re-enable interrupts
                BLX     R2                          ; Call handler. R0 will be IRQ number
                CPSID   i                           ; Disable interrupts again

                ;write EOIR (GIC CPU Interface register)
                POP     {R0,R1}
                DSB                                 ; Ensure that interrupt source is cleared before we write the EOIR
                MRC     p15, 4, R1, c15, c0, 0      ; Read CBAR
ret_irq
                ;epilogue
                STR     R0, [R1, #ICCEOIR_OFFSET]

                LDR     R0, =IRQNestLevel           ; Get address of nesting counter
                LDR     R1, [R0]
                SUB     R1, R1, #1                  ; Decrement nesting counter
                STR     R1, [R0]

                POP     {R1, LR}                    ; Get stack adjustment and restore LR_SVC
                ADD     SP, SP, R1                  ; Unadjust stack

                POP     {R0-R3,R12}                 ; Restore stacked APCS registers
                RFEFD   SP!                         ; Return from exception
                ENDP


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + USR_Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ENDIF


                END
