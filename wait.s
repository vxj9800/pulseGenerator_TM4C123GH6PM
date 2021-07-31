; Author: Vatsal Asitkumar Joshi
; Date: Oct 2nd, 2019
; This is an assembly code for waitMicroSeconds() function.
;
; "If you are done writing the code, now is a good time to debug it."
;

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

	.def waitMicroSeconds

;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

; Blocking function that returns only when SW1 is pressed
waitMicroSeconds:	SUB		R0, #1					; ms*1						;
					MOV		R1, #6					; ms*1						; The number moved to r1 is nLoop
REDO:				SUB		R1, #1					; ms*nLoop*1				; nLoop = 6
					NOP								; ms*nLoop*nNOP1			; nNOP1 = 1
					CBZ		R1, GOBACK				; ms*((nLoop-1)*1+1*3)		;
					NOP								; ms*(nLoop-1)*nNOP2		; nNOP2 = 1
					B		REDO					; ms*(nLoop-1)*2			;
GOBACK:				CBZ		R0, ENDCOUNTER			; (ms-1)*1+1*3				;
					B		waitMicroSeconds		; (ms-1)*2					;
ENDCOUNTER:			BX		LR						; nNOP3						;

.endm
