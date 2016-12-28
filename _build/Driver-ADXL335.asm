;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.3 #9775 (Mac OS X i386)
;--------------------------------------------------------
	.module main
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _isr_vector38
	.globl _isr_vector66
	.globl _isprint
	.globl _io_write
	.globl _io_read
	.globl _io_write_buffer
	.globl _io_read_buffer
	.globl _uart_init
	.globl _uart_set_baudrate
	.globl _uart_write
	.globl _uart_read
	.globl _uart_print
	.globl _uart_read_line
	.globl _uart_disable_interrupts
	.globl _uart_enable_interrupts
	.globl _ppi_init
	.globl _ppi_set_portc_bit
	.globl _ppi_clear_portc_bit
	.globl _delay_10us
	.globl _delay_100us
	.globl _delay_ms
	.globl _putchar
	.globl _getchar
	.globl _system_init
	.globl _configPPI
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
_PPI_PORTA	=	0x0060
_PPI_PORTB	=	0x0061
_PPI_PORTC	=	0x0062
_PPI_CTRL	=	0x0063
_URRBR	=	0x0070
_URTHR	=	0x0070
_URIER	=	0x0071
_URIIR	=	0x0072
_URLCR	=	0x0073
_URLSR	=	0x0075
_URMCR	=	0x0074
_URMSR	=	0x0076
_URDLL	=	0x0070
_URDLM	=	0x0071
_PORTA	=	0x0060
_PORTB	=	0x0061
_PORTC	=	0x0062
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _DATA
___ret_aux:
	.ds 1
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _INITIALIZED
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area _DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area _HOME
	.area _GSINIT
	.area _GSFINAL
	.area _GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area _HOME
	.area _HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area _CODE
;/opt/SMZ80_SDK/V1/include/smz80.h:230: void io_write(char port_addr, char data) {
;	---------------------------------
; Function io_write
; ---------------------------------
_io_write::
;/opt/SMZ80_SDK/V1/include/smz80.h:240: __endasm;
	ld	ix, #2
	add	ix,sp
	ld	c, (ix)
	inc	ix
	ld	a,(ix)
	out	(c), a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:243: char io_read(char port_addr) {
;	---------------------------------
; Function io_read
; ---------------------------------
_io_read::
;/opt/SMZ80_SDK/V1/include/smz80.h:251: __endasm;
	LD	IX, #2
	ADD	IX,SP
	LD	C, (IX)
	IN	A,(C)
	LD	(___ret_aux),A
;/opt/SMZ80_SDK/V1/include/smz80.h:252: return __ret_aux;
	ld	iy,#___ret_aux
	ld	l,0 (iy)
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:254: void io_write_buffer(char port_addr, char* buffer_out, char count) {
;	---------------------------------
; Function io_write_buffer
; ---------------------------------
_io_write_buffer::
;/opt/SMZ80_SDK/V1/include/smz80.h:269: __endasm;
	LD	IX, #2
	ADD	IX,SP
	LD	C, (IX)
	INC	IX
	LD	L,(IX)
	INC	IX
	LD	H,(IX)
	INC	IX
	LD	B,(IX)
	OTIR
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:271: void io_read_buffer(char port_addr, char* buffer_in, char count) {
;	---------------------------------
; Function io_read_buffer
; ---------------------------------
_io_read_buffer::
;/opt/SMZ80_SDK/V1/include/smz80.h:286: __endasm;
	LD	IX, #2
	ADD	IX,SP
	LD	C, (IX)
	INC	IX
	LD	L,(IX)
	INC	IX
	LD	H,(IX)
	INC	IX
	LD	B,(IX)
	INIR
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:289: void uart_init(const uart_cfg_t *uart_config) {
;	---------------------------------
; Function uart_init
; ---------------------------------
_uart_init::
	push	ix
	ld	ix,#0
	add	ix,sp
;/opt/SMZ80_SDK/V1/include/smz80.h:290: uart_set_baudrate(uart_config->baudrate);
	ld	c,4 (ix)
	ld	b,5 (ix)
	ld	a,(bc)
	ld	d,a
	push	bc
	push	de
	inc	sp
	call	_uart_set_baudrate
	inc	sp
	pop	bc
;/opt/SMZ80_SDK/V1/include/smz80.h:291: URIER = uart_config->interrupt;
	ld	l, c
	ld	h, b
	ld	de, #0x0004
	add	hl, de
	ld	a,(hl)
	out	(_URIER),a
;/opt/SMZ80_SDK/V1/include/smz80.h:292: URLCR = (uart_config->stop_bits) | (uart_config->parity) | (uart_config->word_length);
	ld	l, c
	ld	h, b
	inc	hl
	ld	e,(hl)
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	ld	a,(hl)
	or	a, e
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	inc	hl
	ld	c,(hl)
	or	a, c
	out	(_URLCR),a
	pop	ix
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:294: void uart_set_baudrate(const uart_baudrate_t baudrate) {
;	---------------------------------
; Function uart_set_baudrate
; ---------------------------------
_uart_set_baudrate::
;/opt/SMZ80_SDK/V1/include/smz80.h:295: URLCR |= BV(UDLAB);
	in	a,(_URLCR)
	set	7, a
	out	(_URLCR),a
;/opt/SMZ80_SDK/V1/include/smz80.h:296: URDLL = baudrate;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URDLL),a
;/opt/SMZ80_SDK/V1/include/smz80.h:297: URDLM = ((uint16_t)baudrate)>>8;
	ld	a, #0x00
	out	(_URDLM),a
;/opt/SMZ80_SDK/V1/include/smz80.h:298: URLCR &= ~BV(UDLAB);
	in	a,(_URLCR)
	and	a, #0x7f
	out	(_URLCR),a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:300: void uart_write(char c) {
;	---------------------------------
; Function uart_write
; ---------------------------------
_uart_write::
;/opt/SMZ80_SDK/V1/include/smz80.h:301: while( !(URLSR & BV(UTHRE)))
00101$:
	in	a,(_URLSR)
	and	a, #0x20
	jr	NZ,00103$
;/opt/SMZ80_SDK/V1/include/smz80.h:302: NOP();    
	NOP
	jr	00101$
00103$:
;/opt/SMZ80_SDK/V1/include/smz80.h:303: URTHR = c;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URTHR),a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:305: char uart_read() {
;	---------------------------------
; Function uart_read
; ---------------------------------
_uart_read::
;/opt/SMZ80_SDK/V1/include/smz80.h:306: while(!(URLSR & BV(UDR))) 
00101$:
	in	a,(_URLSR)
	rrca
	jr	C,00103$
;/opt/SMZ80_SDK/V1/include/smz80.h:307: NOP();
	NOP
	jr	00101$
00103$:
;/opt/SMZ80_SDK/V1/include/smz80.h:308: return URRBR;
	in	a,(_URRBR)
	ld	l,a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:310: void uart_print(const char* str) {
;	---------------------------------
; Function uart_print
; ---------------------------------
_uart_print::
;/opt/SMZ80_SDK/V1/include/smz80.h:311: while(*str)       
	pop	de
	pop	bc
	push	bc
	push	de
00101$:
	ld	a,(bc)
	or	a, a
	ret	Z
;/opt/SMZ80_SDK/V1/include/smz80.h:312: putchar(*str++); // envía el siguiente caracter. 
	ld	e,a
	inc	bc
	ld	d,#0x00
	push	bc
	push	de
	call	_putchar
	pop	af
	pop	bc
	jr	00101$
;/opt/SMZ80_SDK/V1/include/smz80.h:314: int uart_read_line(char* str) {
;	---------------------------------
; Function uart_read_line
; ---------------------------------
_uart_read_line::
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
;/opt/SMZ80_SDK/V1/include/smz80.h:315: int n=0;
	ld	bc,#0x0000
;/opt/SMZ80_SDK/V1/include/smz80.h:317: while(n<MAXLINE-1 && (c=getchar()) != '\n' && c !='\r') {
00111$:
	ld	a,c
	sub	a, #0xff
	ld	a,b
	rla
	ccf
	rra
	sbc	a, #0x83
	jp	NC,00113$
	push	bc
	call	_getchar
	pop	bc
	ld	a,l
	ld	e,a
	sub	a, #0x0a
	jp	Z,00113$
	ld	a,e
	sub	a, #0x0d
	jr	Z,00113$
;/opt/SMZ80_SDK/V1/include/smz80.h:321: putchar(c);
	ld	-2 (ix),e
	ld	-1 (ix),#0x00
;/opt/SMZ80_SDK/V1/include/smz80.h:318: if(c == 0x7F || c==0x08) {
	ld	a,e
	cp	a,#0x7f
	jr	Z,00105$
	sub	a, #0x08
	jr	NZ,00106$
00105$:
;/opt/SMZ80_SDK/V1/include/smz80.h:319: if(n>0){
	xor	a, a
	cp	a, c
	sbc	a, b
	jp	PO, 00149$
	xor	a, #0x80
00149$:
	jp	P,00111$
;/opt/SMZ80_SDK/V1/include/smz80.h:320: str[--n]='\0';
	dec	bc
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;/opt/SMZ80_SDK/V1/include/smz80.h:321: putchar(c);
	push	bc
	pop	de
	pop	hl
	push	hl
	push	de
	push	hl
	call	_putchar
	ld	hl, #0x0020
	ex	(sp),hl
	call	_putchar
	pop	af
	pop	de
	pop	hl
	push	hl
	push	de
	push	hl
	call	_putchar
	pop	af
	pop	bc
	jr	00111$
00106$:
;/opt/SMZ80_SDK/V1/include/smz80.h:327: if(isprint(c)) {
	push	bc
	push	de
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_isprint
	pop	af
	pop	de
	pop	bc
	ld	a,h
	or	a,l
	jr	Z,00111$
;/opt/SMZ80_SDK/V1/include/smz80.h:328: str[n++]=c;
	push	bc
	pop	iy
	inc	bc
	push	bc
	ld	c,4 (ix)
	ld	b,5 (ix)
	add	iy, bc
	pop	bc
	ld	0 (iy), e
;/opt/SMZ80_SDK/V1/include/smz80.h:329: putchar(c);
	push	bc
	pop	de
	pop	hl
	push	hl
	push	de
	push	hl
	call	_putchar
	pop	af
	pop	bc
	jp	00111$
00113$:
;/opt/SMZ80_SDK/V1/include/smz80.h:332: str[n]='\0';     
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;/opt/SMZ80_SDK/V1/include/smz80.h:333: putchar('\n');
	push	bc
	ld	hl,#0x000a
	push	hl
	call	_putchar
	pop	af
;/opt/SMZ80_SDK/V1/include/smz80.h:334: return n;
	pop	hl
	ld	sp, ix
	pop	ix
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:336: void uart_disable_interrupts() {
;	---------------------------------
; Function uart_disable_interrupts
; ---------------------------------
_uart_disable_interrupts::
;/opt/SMZ80_SDK/V1/include/smz80.h:337: URIER = 0;
	ld	a,#0x00
	out	(_URIER),a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:339: void uart_enable_interrupts(uart_interrupt_t int_cfg) {
;	---------------------------------
; Function uart_enable_interrupts
; ---------------------------------
_uart_enable_interrupts::
;/opt/SMZ80_SDK/V1/include/smz80.h:340: URIER = int_cfg;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URIER),a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:344: void ppi_init(const ppi_cfg_t *ppi_config) {
;	---------------------------------
; Function ppi_init
; ---------------------------------
_ppi_init::
	push	ix
	ld	ix,#0
	add	ix,sp
;/opt/SMZ80_SDK/V1/include/smz80.h:345: PPI_CTRL = 0x80 | ppi_config->mode | (ppi_config->pcl_dir << PCPCL) | (ppi_config->pch_dir << PCPCH) | (ppi_config->pa_dir << PCPA) | (ppi_config->pb_dir << PCPB);
	ld	e,4 (ix)
	ld	d,5 (ix)
	ld	a,(de)
	set	7, a
	ld	c,a
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	or	a, c
	ld	c,a
	push	de
	pop	iy
	ld	a,4 (iy)
	rlca
	rlca
	rlca
	and	a,#0xf8
	or	a, c
	ld	c,a
	ld	l, e
	ld	h, d
	inc	hl
	ld	a,(hl)
	rlca
	rlca
	rlca
	rlca
	and	a,#0xf0
	or	a, c
	ld	c,a
	ex	de,hl
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a, a
	or	a, c
	out	(_PPI_CTRL),a
	pop	ix
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:347: void ppi_set_portc_bit(const char bit) {
;	---------------------------------
; Function ppi_set_portc_bit
; ---------------------------------
_ppi_set_portc_bit::
;/opt/SMZ80_SDK/V1/include/smz80.h:348: PPI_CTRL = 1 | bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	set	0, a
	out	(_PPI_CTRL),a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:350: void ppi_clear_portc_bit(const char bit) {
;	---------------------------------
; Function ppi_clear_portc_bit
; ---------------------------------
_ppi_clear_portc_bit::
;/opt/SMZ80_SDK/V1/include/smz80.h:351: PPI_CTRL = bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	out	(_PPI_CTRL),a
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:354: void delay_10us(){
;	---------------------------------
; Function delay_10us
; ---------------------------------
_delay_10us::
;/opt/SMZ80_SDK/V1/include/smz80.h:363: __endasm;
	EXX
	EX	AF,AF'
	LD	B,#0x2
	    LOOP_10:
	DJNZ	LOOP_10
	EX	AF,AF'
	EXX
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:365: void delay_100us(){
;	---------------------------------
; Function delay_100us
; ---------------------------------
_delay_100us::
;/opt/SMZ80_SDK/V1/include/smz80.h:374: __endasm;
	EXX
	EX	AF,AF'
	LD	B,#0x3A
	    LOOP_100:
	DJNZ	LOOP_100
	EX	AF,AF'
	EXX
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:376: void delay_ms(int ms) {
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms::
	push	ix
	ld	ix,#0
	add	ix,sp
;/opt/SMZ80_SDK/V1/include/smz80.h:379: while(ms--)
	ld	c,4 (ix)
	ld	b,5 (ix)
00102$:
	ld	e, c
	ld	d, b
	dec	bc
	ld	a,d
	or	a,e
	jr	Z,00108$
;/opt/SMZ80_SDK/V1/include/smz80.h:380: for(i=0;i<0x106;i++)
	ld	de,#0x0106
00107$:
;/opt/SMZ80_SDK/V1/include/smz80.h:381: __asm__("nop");
	nop
	ex	de,hl
	dec	hl
	ld	e, l
;/opt/SMZ80_SDK/V1/include/smz80.h:380: for(i=0;i<0x106;i++)
	ld	a,h
	ld	d,a
	or	a,l
	jr	NZ,00107$
	jr	00102$
00108$:
	pop	ix
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:383: int putchar(int c) {
;	---------------------------------
; Function putchar
; ---------------------------------
_putchar::
;/opt/SMZ80_SDK/V1/include/smz80.h:385: if(c=='\n')
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	sub	a, #0x0a
	jr	NZ,00102$
	ld	a,1 (iy)
	or	a, a
	jr	NZ,00102$
;/opt/SMZ80_SDK/V1/include/smz80.h:386: uart_write('\r');
	ld	a,#0x0d
	push	af
	inc	sp
	call	_uart_write
	inc	sp
00102$:
;/opt/SMZ80_SDK/V1/include/smz80.h:387: uart_write(c);
	ld	hl, #2+0
	add	hl, sp
	ld	b, (hl)
	push	bc
	inc	sp
	call	_uart_write
	inc	sp
;/opt/SMZ80_SDK/V1/include/smz80.h:389: return c;
	pop	bc
	pop	hl
	push	hl
	push	bc
	ret
;/opt/SMZ80_SDK/V1/include/smz80.h:391: char getchar() {
;	---------------------------------
; Function getchar
; ---------------------------------
_getchar::
;/opt/SMZ80_SDK/V1/include/smz80.h:394: return uart_read();
	jp  _uart_read
;main.c:6: ISR_NMI() {
;	---------------------------------
; Function isr_vector66
; ---------------------------------
_isr_vector66::
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:7: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	retn
;main.c:9: ISR_INT_38() {
;	---------------------------------
; Function isr_vector38
; ---------------------------------
_isr_vector38::
	ei
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:10: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	reti
;main.c:22: int main() {
;	---------------------------------
; Function main
; ---------------------------------
_main::
;main.c:24: system_init();
	call	_system_init
;main.c:26: while(TRUE) {
00102$:
;main.c:27: PORTC = 0x00;
	ld	a,#0x00
	out	(_PORTC),a
;main.c:28: URTHR = PORTA;
	in	a,(_PORTA)
	out	(_URTHR),a
;main.c:29: delay_ms(25);
	ld	hl,#0x0019
	push	hl
	call	_delay_ms
	pop	af
;main.c:32: PORTC = 0x01;
	ld	a,#0x01
	out	(_PORTC),a
;main.c:33: URTHR = PORTA;
	in	a,(_PORTA)
	out	(_URTHR),a
;main.c:34: delay_ms(25);
	ld	hl,#0x0019
	push	hl
	call	_delay_ms
	pop	af
	jr	00102$
;main.c:42: void system_init() {
;	---------------------------------
; Function system_init
; ---------------------------------
_system_init::
	push	af
	push	af
	dec	sp
;main.c:44: uartConfig.baudrate = UART_BAUDRATE_9600;
	ld	hl,#0x0000
	add	hl,sp
	ld	(hl),#0x1a
;main.c:45: uartConfig.stop_bits = UART_STOP_BITS_1;
	ld	hl,#0x0000
	add	hl,sp
	ld	c,l
	ld	b,h
	ld	e, c
	ld	d, b
	inc	de
	xor	a, a
	ld	(de),a
;main.c:46: uartConfig.parity = UART_PARITY_NONE;
	ld	e, c
	ld	d, b
	inc	de
	inc	de
	xor	a, a
	ld	(de),a
;main.c:47: uartConfig.word_length = UART_WORD_LENGTH_8;
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	inc	hl
	ld	(hl),#0x03
;main.c:48: uartConfig.interrupt = UART_INTERRUPT_NONE;
	ld	hl,#0x0004
	add	hl,bc
	ld	(hl),#0x00
;main.c:49: uart_init(&uartConfig);
	push	bc
	call	_uart_init
	pop	af
;main.c:50: configPPI();
	call	_configPPI
	pop	af
	pop	af
	inc	sp
	ret
;main.c:60: void configPPI() {
;	---------------------------------
; Function configPPI
; ---------------------------------
_configPPI::
;main.c:62: PPI_CTRL = 0x92;
	ld	a,#0x92
	out	(_PPI_CTRL),a
	ret
	.area _CODE
	.area _INITIALIZER
	.area _CABS (ABS)
