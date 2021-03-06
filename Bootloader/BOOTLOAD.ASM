;*************************************************************************
;*									 *
;*				Generic AVR Bootloader			 *
;*                                                                       *
;*                      Author: Peter Dannegger                          *
;*									 *
;*************************************************************************
.nolist
;			select the appropriate include file:
;.include "tn13def.inc"
;.include "tn2313def.inc"
;.include "tn25def.inc"
;.include "tn261def.inc"
;.include "tn44def.inc"
.include "tn45def.inc"
;.include "tn461def.inc"
;.include "m48def.inc"
;.include "tn84def.inc"
;.include "tn85def.inc"
;.include "tn861def.inc"

;			set the SecondBootStart fuse on these AVRs:
;.include "m8def.inc"
;.include "m8515def.inc"
;.include "m8535def.inc"
;.include "m88def.inc"
;.include "m16def.inc"
;.include "m162def.inc"
;.include "m168def.inc"

;			set the FirstBootStart fuse on these AVRs:
;.include "m32def.inc"
;.include "m64def.inc"
;.include "m644def.inc"
;.include "m128def.inc"
;.include "m1281def.inc"
;.include "m2561def.inc"


;			remove comment sign to exclude API-Call:
;			only on ATmega >= 8kB supported
;.equ	APICALL		= 0

;			remove comment sign to exclude Watchdog trigger:
;.equ   WDTRIGGER	= 0

;			remove comment sign to exclude CRC:
;.equ	CRC		= 0	

;			remove comment sign to exclude Verify:
;.equ	VERIFY		= 0

;-------------------------------------------------------------------------
;                               Port definitions
;-------------------------------------------------------------------------
;			set both lines equal for inverted onewire mode:

.equ    STX_PORT        = PORTB
.equ    STX             = PB2

.equ    SRX_PORT        = PORTB
.equ    SRX             = PB2
;-------------------------------------------------------------------------
.include "fastload.inc"
;-------------------------------------------------------------------------
