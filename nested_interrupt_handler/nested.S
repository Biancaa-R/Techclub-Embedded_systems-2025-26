;Function : Modify the registers so we can see the values
;Directives
		PRESERVE8 ;Sets the width of the stack to 8 bytes
		THUMB ; tells the assembler that we are coding in thumb
;vector table mapped to address0 at reset ,linker requires __vectors to be exported
		AREA RESET,DATA,READONLY
		;Area designates the following as package
		;RESET :Sets up a new memory region reset
		;DATA tells the assembler the code is code
		;READONLY makes sure WE dont write over our own program
		EXPORT __Vectors
__Vectors DCD 0x20002000 ; stack pointer when the stack is empty
		DCD Reset_Handler;reset vector
		ALIGN
;reset needs reset handler and it needs to be exported
		AREA MYCODE ,CODE ,READONLY
		ENTRY
		EXPORT		Reset_Handler
Reset_Handler 
		Maskmd      EQU 0x1f ;processor mode masking
			;[4:0] bits in the CPSR :Clearing
		SVC32md     EQU 0x13 ;SVC mode -->value
		I_Bit       EQU 0x80; Setting up of the IRQ bit
		;For setiing the IRQ interrupt number
		;Assigning of frames like the task frames
		
		;Defines the offsets of the registers saved on the stack during an interrupt
		FRAME_R0    EQU 0x00
		FRAME_R1    EQU FRAME_RO+4
		FRAME_R2    EQU FRAME_R1+4
		FRAME_R3    EQU FRAME_R2+4
		FRAME_R4    EQU FRAME_R3+4
		FRAME_R5    EQU FRAME_R4+4
		FRAME_R6    EQU FRAME_R5+4
		FRAME_R7    EQU FRAME_R6+4
		FRAME_R8    EQU FRAME_R7+4
		FRAME_R9    EQU FRAME_R8+4
		FRAME_R10   EQU FRAME_R9+4
		FRAME_R11   EQU FRAME_R10+4
		FRAME_R12   EQU FRAME_R11+4	
		FRAME_PSR   EQU FRAME_R12+4 //Program staus register
		FRAME_LR    EQU FRAME_PSR+4 //link register
		FRAME_PC    EQU FRAME_LR+4
		FRAME_SIZE  EQU FRAME_PC+4 ;Total size of frames?
		;These define offsets for registers saved on the stack during an interrupt. They form a task context frame.
		;Assigning the different frames and the associated registers
IRQ_Entry ;
		SUB  R14,R14,#4
		;To allign it for returning
		//reducing the r14
		//Link register
		STMDB r13!,{r0-r3,r12-r14}
		;save context
		;<service interrupt>
		BL read_RescheduleFlag
		;To check if reschedule is needed
		CMP R0,#0 ;Compare if it is still 0
		LDMNEIA r13!,{r0-r3,r12,pc} ;else return
		;Restore the registers and return from IRQ
		;Zero flag is clear --> 0 ,If Z is not 1
		MRS R2,spsr ;copy the spsr value
		MOV R0,R13 ;Copy r13 IRQ ->SP
		;stack pointer value
		ADD R13,R14,#6*4 ;Reset tyhe handler stack
		MRS R1,cpsr 
		BIC R1,R1,#Maskmd ;Clear the mask value
		ORR R1,R1,#SVC32md ;Oring the SVC mode?
		MSR cpsr_c ,R1; change to SVC -->changed cpsr
		;callee saved regs
		SUB R13,R13,#FRAME_SIZE -FRAME_R4; Making space
		STMIA R13,{R4-R11};Going to R4 space
		;Restore R4 to R9
		LDMIA R0, {R4-R9};Restore R4 to R9 
		;Initially saved SP
		;Enable the irq again
		BIC R1,R1 ,#I_Bit
		MSR cpsr_c,R1 //enable IRq  (after switching modes and saving context)
		STMDB R13! ,{R4-R7}
		STR R2,[R13,#FRAME_PSR] ;save the frame psr values
		STR R8,[R13,#FRAME_R12] ;Save the value of R12
		;Save the PC value
		STR R9,[R13,#FRAME_PC]
		STR R14,[R13,#FRAME_LR]
		;Complete interrupt service routine
		LDMIA R13!,{R0-R12,R14}
		MSR spsr_cxsf ,r14;restoring the value of spsr
		LDMIA R13! ,{R14,PC}
		;Uses a stack frame structure all registers are addded except r13
		B			Reset_Handler
		ALIGN
		END
		