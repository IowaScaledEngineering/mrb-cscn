/* CTC Siding Control Node - v1.0                                    */
/* File: ctcnode.c  Version: 1.0   Modified: 13-May-2008             */
/*  Copyright 2008 Nathan D. Holmes (maverick@drgw.net)              */
/*  Licensed under the GPL v2                                        */
/*  See http://www.ndholmes.com/pmwiki.php/Electronics/MRBus         */
/*   for more information                                            */

/* **** VERSION HISTORY ****

  13-May-2008:  Initial work to change over for PIC16F887 and BoostC 6.x  

  08-Mar-2003:  v0.9 - Modified to work with C2C v5.00
                 NOTE: Don't actually use v5.00 or 5.01 - serious bugs
                 exist.  I also haven't tested it on 5.04 yet.              
                Includes switching over to header file constants.   
                Also modified to use UMRBD v1.0
                Fixed condition where transmit buffer could be corrupted by
                 rapid-fire packets.
  
  28-Mar-2003:  v0.99 - Fixed lockup bug that resulted from bus_countdown
                 being set outside the range of ticks
                 
*/


/* Target PIC Selection - Uncomment the correct define */
//#define TARGET_PIC16F87x
#define TARGET_PIC16F88x
#pragma CLOCK_FREQ 20000000

#include <system.h>
#include "../core/mrbus.h"
#include "../core/mrbus.c"

char e_pnts_usig;
char e_pnts_lsig;
char e_main_sig;
char e_side_sig;

char w_pnts_usig;
char w_pnts_lsig;
char w_main_sig;
char w_side_sig;

char occupancy;
char v_occupancy;
char clearance;
char turnouts;

char manual_lockout;


char old_e_pnts_usig;
char old_e_pnts_lsig;
char old_e_main_sig;
char old_e_side_sig;

char old_w_pnts_usig;
char old_w_pnts_lsig;
char old_w_main_sig;
char old_w_side_sig;

char old_occupancy;
char old_clearance;
char old_turnouts;

/* Device specific defines */
#define E_MAIN_SIG_RED      7
#define E_MAIN_SIG_GREEN    6
#define E_SIDE_SIG_RED      5
#define E_SIDE_SIG_GREEN    4
#define E_PNTS_USIG_RED     3
#define E_PNTS_USIG_GREEN   2
#define E_PNTS_LSIG_RED     1
#define E_PNTS_LSIG_GREEN   0

char E_SIG@0x06;  /* Port B */

#define W_MAIN_SIG_RED      7
#define W_MAIN_SIG_GREEN    6
#define W_SIDE_SIG_RED      5
#define W_SIDE_SIG_GREEN    4
#define W_PNTS_USIG_RED     3
#define W_PNTS_USIG_GREEN   2
#define W_PNTS_LSIG_RED     1
#define W_PNTS_LSIG_GREEN   0

char W_SIG@0x08;  /* Port D */


#define RED              0x04
#define BL_YELLOW        0x03
#define YELLOW           0x02
#define GREEN            0x01

#define E_TURNOUT porte
#define E_PNTS_STATUS_MASK  0x01
#define E_PNTS_MAIN         1
#define E_PNTS_DIVERGING    2

#define E_TURNOUT_BTTNS porta
#define E_PNTS_BTTN_MAIN  0x10
#define E_PNTS_BTTN_DIV   0x20

#define W_TURNOUT portc
#define W_PNTS_STATUS_MASK  0x01
#define W_PNTS_MAIN         1
#define W_PNTS_DIVERGING    3

#define W_TURNOUT_BTTNS portc
#define W_PNTS_BTTN_MAIN  0x10
#define W_PNTS_BTTN_DIV   0x20


#define E_CLEAR_NONE		0x00
#define E_CLEAR_EAST		0x01
#define E_CLEAR_WEST		0x02

#define W_CLEAR_NONE		0x00
#define W_CLEAR_EAST		0x10
#define W_CLEAR_WEST		0x20

#define E_ADJOIN			0x80
#define E_APPROACH			0x40
#define W_ADJOIN			0x20
#define W_APPROACH			0x10
#define CTC_MAIN			0x08
#define CTC_SIDING			0x04
#define E_OS_SECT			0x02
#define W_OS_SECT			0x01

#define E_VOCC				0x02
#define W_VOCC				0x01


/* EEPROM Configuration Addresses */
#define EE_DEVICE_ADDR    0x00

#define EE_E_APRCH_ADDR   0x10
#define EE_E_APRCH2_ADDR  0x11
#define EE_E_ADJ_ADDR     0x12
#define EE_W_APRCH_ADDR   0x13
#define EE_W_APRCH2_ADDR  0x14
#define EE_W_ADJ_ADDR     0x15

#define EE_E_APRCH_PKT    0x20
#define EE_E_APRCH2_PKT   0x21
#define EE_E_ADJ_PKT      0x22
#define EE_W_APRCH_PKT    0x23
#define EE_W_APRCH2_PKT   0x24
#define EE_W_ADJ_PKT      0x25

#define EE_E_APRCH_BITBYTE    0x30
#define EE_E_APRCH2_BITBYTE   0x31
#define EE_E_ADJ_BITBYTE      0x32
#define EE_W_APRCH_BITBYTE    0x33
#define EE_W_APRCH2_BITBYTE   0x34
#define EE_W_ADJ_BITBYTE      0x35



void PktHandler(void);
void signal_output(void);
void east_os_section_signals(void);
void west_os_section_signals(void);
void CodeCTCRoute(char e_turn, char e_clear, char w_turn, char w_clear);

char EepromRead(char addr)
{
	eeadr = addr;
	set_bit(eecon1, RD);
	return(eedata);
}


char EepromWrite(char addr, char data)
{
	char temp_status;
	eeadr = addr;
	eedata = data;
	clear_bit(eecon1, 7); //EEPGD
	set_bit(eecon1, 2);	// WREN
	
	asm
	{
		bcf		_intcon, GIE
		;bsf		STATUS, RP0
		;bsf		STATUS, RP1
		movlw	0x55
		movwf	_pie2
		movlw	0xAA
		movwf	_pie2
		bsf		_pie1, 1
		bsf		_intcon, GIE
	}
	while(eecon1 & 0x02)
	{
		asm clrwdt
	}
    clear_bit( eecon1, WREN );
    
	return(EepromRead(addr));
}

void interrupt(void)
{
	char i;

	if ((intcon & T0IF_MASK) && (intcon & T0IE_MASK))
	{
		ticks++;
		if (ticks == 100) 
		{
			ticks = 0;
			secs++;
		}
		
		if (manual_lockout != 0)
		{
			manual_lockout = manual_lockout - 1;
		}
		clear_bit(intcon, T0IF);
	}

	if ((pir1 & RCIF_MASK) && (pie1 & RCIE_MASK))
	{
		mrbus_activity = 1;
		if (rcsta & RC_ERR_MASK)
		{
			/* Handle framing errors - these are likely arbitration bytes */
			rx_index = rcreg;
			clear_bit(rcsta, CREN);
			rx_index = 0; /* Reset receive buffer */
			set_bit(rcsta, CREN);
		} else {
			/* Receive Routine */
			rx_input_buffer[rx_index++] = rcreg;

			if ((rx_index > 4) && ((rx_index == rx_input_buffer[PKT_LEN]) || (rx_index >=  BUFFER_SIZE) ))
			{
				rx_index = 0;

				if((state & RX_PKT_READY) == 0)
				{
					if (rx_input_buffer[PKT_LEN] > BUFFER_SIZE) rx_input_buffer[PKT_LEN] = BUFFER_SIZE;
					
					for(i=0; i < rx_input_buffer[PKT_LEN]; i++)
					{
						rx_buffer[i] = rx_input_buffer[i];
					}
					state = state | RX_PKT_READY;
					mrbus_activity = 2;
				}
			}
			if (rx_index >= BUFFER_SIZE)
			{
				rx_index = BUFFER_SIZE-1;
			}

		}
	}

	if ((pir1 & TXIF_MASK) && (pie1 & TXIE_MASK))
	{
		/* Transmit Routine */
		clear_bit(pir1, TXIF);
		if (tx_index >= BUFFER_SIZE || tx_index >= tx_buffer[PKT_LEN])
		{
			while(!(txsta & TRMT_MASK));
			clear_bit(pie1, TXIE);
			state = state & (~TX_BUF_ACTIVE);
			clear_bit(RS485PORT, RS485TXEN);
		} else {
			txreg = tx_buffer[tx_index++];
		}
	}
}

void main(void)
{
	char i;
	char bus_countdown;
	char changed=0;
	
	char button_scan=0x33;
	char old_button_scan = 0x33;
	
	e_pnts_usig = RED;
	e_pnts_lsig = RED;
	e_main_sig = RED;
	e_side_sig = RED;

	w_pnts_usig = RED;
	w_pnts_lsig = RED;
	w_main_sig = RED;
	w_side_sig = RED;
	
	occupancy = 0;
	v_occupancy = 0;
	clearance = 0;
	turnouts = 0;
	state = 0x00;

	porte = 0x00;
	portc = 0x00;	

	trisa = 0xFF;
	trisb = 0x00;
	trisc = 0x31;
	trisd = 0x00;
	trise = 0x01;
	pie1  = 0x00;
	pir1  = 0x00;
	
	portc = 0x01;
	pie2  = 0x00;
	pir2  = 0x00;

	manual_lockout = 0;

	eecon1 = 0x00;

	#ifdef TARGET_PIC16F87x
	eeadrh = 0;
	adcon0 = 0x00;
	adcon1 = 0x07;
	#endif

	#ifdef TARGET_PIC16F88x
	eeadrh = 0;
	ansel = 0;
	anselh = 0;
	#endif

	porta = 0x00;
	portb = 0x00;

	dev_addr = EepromRead(EE_DEVICE_ADDR);
	
	MRBus_USART_Setup();

	/* Timer 0 Setup */
	asm {
		clrwdt
		bsf _status, RP0
		movlw H'07'
		movwf _option_reg
		bcf _status, RP0
	}

	intcon = 0xE0;
	clearance = E_CLEAR_NONE + W_CLEAR_NONE;

	/* Set past status to current status */
	old_e_pnts_usig = e_pnts_usig;
	old_e_pnts_lsig = e_pnts_lsig;
	old_e_main_sig = e_main_sig;
	old_e_side_sig = e_side_sig;
	old_w_pnts_usig = w_pnts_usig;
	old_w_pnts_lsig = w_pnts_lsig;
	old_w_main_sig = w_main_sig;
	old_w_side_sig = w_side_sig;
	old_occupancy = occupancy;
	old_clearance = clearance;
	old_turnouts = turnouts;
	
	changed = 0;	
	
	while(1)
	{
		if (state & RX_PKT_READY) PktHandler();

		turnouts = (E_TURNOUT & E_PNTS_STATUS_MASK) | ((W_TURNOUT & W_PNTS_STATUS_MASK)<<1);
		occupancy = (occupancy & 0xF0) | (porta & 0x0F);

		if (manual_lockout == 0)
		{
			/* Scan buttons every 1/8th of a second */
			button_scan = ((E_TURNOUT_BTTNS & 0x30)>>4) | (W_TURNOUT_BTTNS & 0x30);
			
			if (old_button_scan != button_scan)
			{
				i = button_scan ^ old_button_scan;
				i = i & old_button_scan;			
				
				/* East Turnout to the main */
				if (i & 0x01)
				{
					set_bit(E_TURNOUT, E_PNTS_MAIN);
					clearance = ((clearance & 0xF0) | E_CLEAR_NONE);
					clear_bit(E_TURNOUT, E_PNTS_MAIN);
				} else if (i & 0x02) {
					set_bit(E_TURNOUT, E_PNTS_DIVERGING);
					clearance = ((clearance & 0xF0) | E_CLEAR_NONE);
					clear_bit(E_TURNOUT, E_PNTS_DIVERGING);
				}
				
				/* West Turnout to the main */
				if (i & 0x10)
				{
					set_bit(W_TURNOUT, W_PNTS_MAIN);
					clearance = ((clearance & 0x0F) | W_CLEAR_NONE);
					clear_bit(W_TURNOUT, W_PNTS_MAIN);
				} else if (i & 0x20) {
					set_bit(W_TURNOUT, W_PNTS_DIVERGING);
					clearance = ((clearance & 0x0F) | W_CLEAR_NONE);
					clear_bit(W_TURNOUT, W_PNTS_DIVERGING);
				}
			
				old_button_scan = button_scan;
				manual_lockout = 50;
			}
			
		}
		
		if (occupancy & E_OS_SECT)
		{
			clearance = ((clearance & 0xF0) | E_CLEAR_NONE);
		}

		if (occupancy & W_OS_SECT)
		{
			clearance = ((clearance & 0x0F) | W_CLEAR_NONE);
		}

		/* Calculate signal aspects */
		east_os_section_signals();
		west_os_section_signals();
		
		/* Signal output logic */
		signal_output();

		asm clrwdt;

		/* Test if something changed from the last time
		   around the loop - we need to send an update 
		   packet if it did */
		   
		if ((old_e_pnts_usig != e_pnts_usig) ||
		(old_e_pnts_lsig != e_pnts_lsig) ||
		(old_e_main_sig != e_main_sig) ||
		(old_e_side_sig != e_side_sig) ||
		(old_w_pnts_usig != w_pnts_usig) ||
		(old_w_pnts_lsig != w_pnts_lsig) ||
		(old_w_main_sig != w_main_sig) ||
		(old_w_side_sig != w_side_sig) ||
		(old_occupancy != occupancy) ||
		(old_turnouts != turnouts) ||
		(old_clearance != clearance))
		{
			/* Something Changed - time to update */
			old_e_pnts_usig = e_pnts_usig;
			old_e_pnts_lsig = e_pnts_lsig;
			old_e_main_sig = e_main_sig;
			old_e_side_sig = e_side_sig;
			old_w_pnts_usig = w_pnts_usig;
			old_w_pnts_lsig = w_pnts_lsig;
			old_w_main_sig = w_main_sig;
			old_w_side_sig = w_side_sig;
			old_occupancy = occupancy;
			old_clearance = clearance;
			old_turnouts = turnouts;
			
			/* Set changed such that a packet gets sent */
			changed = 1;
		}
	
		if ((changed || secs >= 2) && !(state & (TX_BUF_ACTIVE | TX_PKT_READY)))
		{
			tx_buffer[PKT_SRC] = EepromRead(EE_DEVICE_ADDR);
			tx_buffer[PKT_DEST] = 0xFF;
			tx_buffer[PKT_LEN] = 13;			
			tx_buffer[PKT_TYPE] = 'S';

			tx_buffer[6] = (e_side_sig & 0x0F) | ((e_main_sig <<4) & 0xF0);
			tx_buffer[7] = (e_pnts_lsig & 0x0F) | ((e_pnts_usig <<4) & 0xF0);

			tx_buffer[8] = (w_side_sig & 0x0F) | ((w_main_sig <<4) & 0xF0);
			tx_buffer[9] = (w_pnts_lsig & 0x0F) | ((w_pnts_usig <<4) & 0xF0);

			tx_buffer[10] = occupancy;
			tx_buffer[11] = 0x00;
			tx_buffer[12] = 0x00;
			
			if ((clearance & 0x0F) == E_CLEAR_EAST)
			{
				tx_buffer[11] = tx_buffer[11] | 0x01;
			} else if ((clearance & 0x0F) == E_CLEAR_WEST) {
				tx_buffer[11] = tx_buffer[11] | 0x02;
			} else {
				tx_buffer[11] = tx_buffer[11] | 0x04;
			}

			if ((clearance & 0xF0) == W_CLEAR_EAST)
			{
				tx_buffer[12] = tx_buffer[12] | 0x01;
			} else if ((clearance & 0xF0) == W_CLEAR_WEST) {
				tx_buffer[12] = tx_buffer[12] | 0x02;
			} else {
				tx_buffer[12] = tx_buffer[12] | 0x04;
			}			

			if (E_TURNOUT & E_PNTS_STATUS_MASK)
			{
				tx_buffer[11] = tx_buffer[11] | 0x10;
			} else {
				tx_buffer[11] = tx_buffer[11] | 0x20;
			}

			if ((e_pnts_lsig == RED) && (e_pnts_usig == RED))
			{
				tx_buffer[11] = tx_buffer[11] | 0x40;
			}

			
			if (W_TURNOUT & W_PNTS_STATUS_MASK)
			{
				tx_buffer[12] = tx_buffer[12] | 0x10;
			} else {
				tx_buffer[12] = tx_buffer[12] | 0x20;
			}

			if ((w_pnts_lsig == RED) && (w_pnts_usig == RED))
			{
				tx_buffer[12] = tx_buffer[12] | 0x40;
			}

			secs = 0;
			changed = 0;
			
			clear_bit(intcon, GIE);
			while (intcon & 0x80);
			state = state | TX_PKT_READY;
			set_bit(intcon, GIE);
		}

			
		while(state & TX_PKT_READY)
		{
			if (state & RX_PKT_READY) PktHandler();
			i = MRBus_XmitPacket();
			if (i)
			{
				bus_countdown = ticks+2;
				/* Note:  Ticks is in centiseconds, range 0-99.  99+2 won't ever match. */
				if (bus_countdown >= 100) bus_countdown = bus_countdown - 100;
				
				while (ticks != bus_countdown && mrbus_activity != 2)
				{
					asm clrwdt;
					if (state & RX_PKT_READY) PktHandler();
				}
			} else {
				clear_bit(intcon, GIE);
				while (intcon & 0x80);
				state = state & (~TX_PKT_READY);	
				set_bit(intcon, GIE);
				set_bit(pie1, TXIE);			
			}
		}
	}
}

void east_os_section_signals(void)
{
	/* Default case - everything's red */
	e_pnts_usig = RED;
	e_pnts_lsig = RED;
	e_main_sig = RED;
	e_side_sig = RED;	

	if ((clearance & 0x0F) == E_CLEAR_EAST) 
	{
		/* Cleared from frog end to outbound main */
		if (E_TURNOUT & E_PNTS_STATUS_MASK)
		{
			/* Lined for main */
			if ((E_ADJOIN | E_OS_SECT) & occupancy)
			{
				e_main_sig = RED;
			} else if (E_APPROACH & occupancy) {
				e_main_sig = YELLOW;
			} else {
				e_main_sig = GREEN;
			}
			
		} else {
			/* Lined for siding */
			if ((E_ADJOIN | E_OS_SECT) & occupancy)
			{
				e_side_sig = RED;
			} else if (E_APPROACH & occupancy) {
				e_side_sig = YELLOW;
			} else {
				e_side_sig = GREEN;
			}
		}

	} else if ((clearance & 0x0F) == E_CLEAR_WEST) {

		/* Cleared from points end to other end of siding */

		if (E_TURNOUT & E_PNTS_STATUS_MASK)
		{
			/* Lined for main */
			if ((CTC_MAIN | E_OS_SECT) & occupancy)
			{
				e_pnts_usig = RED;
			} else if (((W_ADJOIN | W_OS_SECT) & occupancy) || 
				!(W_TURNOUT & W_PNTS_STATUS_MASK) ||
				!((clearance & 0xF0) == W_CLEAR_WEST)) {
				e_pnts_usig = YELLOW;
			} else {
				e_pnts_usig = GREEN;
			}				
			
		} else {
			/* Lined for siding */
			if ((CTC_SIDING | E_OS_SECT) & occupancy)
			{
				e_pnts_lsig = RED;
			} else if (((W_ADJOIN | W_OS_SECT) & occupancy) || 
				(W_TURNOUT & W_PNTS_STATUS_MASK) ||
				!((clearance & 0xF0) == W_CLEAR_WEST)) {
				e_pnts_lsig = YELLOW;
			} else {
				/* Change this to GREEN if highball is allowed into the siding */
				e_pnts_lsig = YELLOW;
			}					
		}
	}
}

void west_os_section_signals(void)
{
	w_pnts_usig = RED;
	w_pnts_lsig = RED;
	w_main_sig = RED;
	w_side_sig = RED;	

	if ((clearance & 0xF0) == W_CLEAR_WEST) 
	{
		/* Cleared from frog end to outbound main */
		if (W_TURNOUT & W_PNTS_STATUS_MASK)
		{
			/* Lined for main */
			if ((W_ADJOIN | W_OS_SECT) & occupancy)
			{
				w_main_sig = RED;
			} else if (W_APPROACH & occupancy) {
				w_main_sig = YELLOW;
			} else {
				w_main_sig = GREEN;
			}
			
		} else {
			/* Lined for siding */
			if ((W_ADJOIN | W_OS_SECT) & occupancy)
			{
				w_side_sig = RED;
			} else if (W_APPROACH & occupancy) {
				w_side_sig = YELLOW;
			} else {
				w_side_sig = GREEN;
			}
		}

	} else if ((clearance & 0xF0) == W_CLEAR_EAST) {
		
		/* Cleared from points end to other end of siding */

		if (W_TURNOUT & W_PNTS_STATUS_MASK)
		{
			/* Lined for main */
			if ((CTC_MAIN | W_OS_SECT) & occupancy)
			{
				w_pnts_usig = RED;
			} else if (((E_ADJOIN | E_OS_SECT) & occupancy) || 
				!(E_TURNOUT & E_PNTS_STATUS_MASK) ||
				!((clearance & 0x0F) == E_CLEAR_EAST)) {
				w_pnts_usig = YELLOW;
			} else {
				w_pnts_usig = GREEN;
			}				
			
		} else {
			/* Lined for siding */
			if ((CTC_SIDING | W_OS_SECT) & occupancy)
			{
				w_pnts_lsig = RED;
			} else if (((E_ADJOIN | E_OS_SECT) & occupancy) || 
				(E_TURNOUT & E_PNTS_STATUS_MASK) ||
				!((clearance & 0x0F) == E_CLEAR_EAST)) {
				w_pnts_lsig = YELLOW;
			} else {
				/* Change this to GREEN if highball is allowed into the siding */
				w_pnts_lsig = YELLOW;
			}					
		}
	
	}	
}

void signal_output(void)
{
	/* West Interlocking Signal Output Logic */

	if (GREEN == w_side_sig)
	{
		clear_bit(W_SIG, W_SIDE_SIG_RED);			
		set_bit(W_SIG, W_SIDE_SIG_GREEN);			
	} else if (YELLOW == w_side_sig) {
		set_bit(W_SIG, W_SIDE_SIG_RED);			
		set_bit(W_SIG, W_SIDE_SIG_GREEN);			
	} else if (RED == w_side_sig) {
		set_bit(W_SIG, W_SIDE_SIG_RED);			
		clear_bit(W_SIG, W_SIDE_SIG_GREEN);			
	} else {
		clear_bit(W_SIG, W_SIDE_SIG_RED);			
		clear_bit(W_SIG, W_SIDE_SIG_GREEN);	
	}

	if (GREEN == w_main_sig)
	{
		clear_bit(W_SIG, W_MAIN_SIG_RED);			
		set_bit(W_SIG, W_MAIN_SIG_GREEN);			
	} else if (YELLOW == w_main_sig) {
		set_bit(W_SIG, W_MAIN_SIG_RED);			
		set_bit(W_SIG, W_MAIN_SIG_GREEN);			
	} else if (RED == w_main_sig) {
		set_bit(W_SIG, W_MAIN_SIG_RED);			
		clear_bit(W_SIG, W_MAIN_SIG_GREEN);			
	} else {
		clear_bit(W_SIG, W_MAIN_SIG_RED);			
		clear_bit(W_SIG, W_MAIN_SIG_GREEN);	
	}

	if (GREEN == w_pnts_usig)
	{
		clear_bit(W_SIG, W_PNTS_USIG_RED);			
		set_bit(W_SIG, W_PNTS_USIG_GREEN);			
	} else if (YELLOW == w_pnts_usig) {
		set_bit(W_SIG, W_PNTS_USIG_RED);			
		set_bit(W_SIG, W_PNTS_USIG_GREEN);			
	} else if (RED == w_pnts_usig) {
		set_bit(W_SIG, W_PNTS_USIG_RED);			
		clear_bit(W_SIG, W_PNTS_USIG_GREEN);			
	} else {
		clear_bit(W_SIG, W_PNTS_USIG_RED);			
		clear_bit(W_SIG, W_PNTS_USIG_GREEN);	
	}		

	if (GREEN == w_pnts_lsig)
	{
		clear_bit(W_SIG, W_PNTS_LSIG_RED);			
		set_bit(W_SIG, W_PNTS_LSIG_GREEN);			
	} else if (YELLOW == w_pnts_lsig) {
		set_bit(W_SIG, W_PNTS_LSIG_RED);			
		set_bit(W_SIG, W_PNTS_LSIG_GREEN);			
	} else if (RED == w_pnts_lsig) {
		set_bit(W_SIG, W_PNTS_LSIG_RED);			
		clear_bit(W_SIG, W_PNTS_LSIG_GREEN);			
	} else {
		clear_bit(W_SIG, W_PNTS_LSIG_RED);			
		clear_bit(W_SIG, W_PNTS_LSIG_GREEN);	
	}

	/* East Interlocking Signal Output Logic */
	
	if (GREEN == e_side_sig)
	{
		clear_bit(E_SIG, E_SIDE_SIG_RED);			
		set_bit(E_SIG, E_SIDE_SIG_GREEN);			
	} else if (YELLOW == e_side_sig) {
		set_bit(E_SIG, E_SIDE_SIG_RED);			
		set_bit(E_SIG, E_SIDE_SIG_GREEN);			
	} else if (RED == e_side_sig) {
		set_bit(E_SIG, E_SIDE_SIG_RED);			
		clear_bit(E_SIG, E_SIDE_SIG_GREEN);			
	} else {
		clear_bit(E_SIG, E_SIDE_SIG_RED);			
		clear_bit(E_SIG, E_SIDE_SIG_GREEN);	
	}

	if (GREEN == e_main_sig)
	{
		clear_bit(E_SIG, E_MAIN_SIG_RED);			
		set_bit(E_SIG, E_MAIN_SIG_GREEN);			
	} else if (YELLOW == e_main_sig) {
		set_bit(E_SIG, E_MAIN_SIG_RED);			
		set_bit(E_SIG, E_MAIN_SIG_GREEN);			
	} else if (RED == e_main_sig) {
		set_bit(E_SIG, E_MAIN_SIG_RED);			
		clear_bit(E_SIG, E_MAIN_SIG_GREEN);			
	} else {
		clear_bit(E_SIG, E_MAIN_SIG_RED);			
		clear_bit(E_SIG, E_MAIN_SIG_GREEN);	
	}


	if (GREEN == e_pnts_usig)
	{
		clear_bit(E_SIG, E_PNTS_USIG_RED);			
		set_bit(E_SIG, E_PNTS_USIG_GREEN);			
	} else if (YELLOW == e_pnts_usig) {
		set_bit(E_SIG, E_PNTS_USIG_RED);			
		set_bit(E_SIG, E_PNTS_USIG_GREEN);			
	} else if (RED == e_pnts_usig) {
		set_bit(E_SIG, E_PNTS_USIG_RED);			
		clear_bit(E_SIG, E_PNTS_USIG_GREEN);			
	} else {
		clear_bit(E_SIG, E_PNTS_USIG_RED);			
		clear_bit(E_SIG, E_PNTS_USIG_GREEN);	
	}
		

	if (GREEN == e_pnts_lsig)
	{
		clear_bit(E_SIG, E_PNTS_LSIG_RED);			
		set_bit(E_SIG, E_PNTS_LSIG_GREEN);			
	} else if (YELLOW == e_pnts_lsig) {
		set_bit(E_SIG, E_PNTS_LSIG_RED);			
		set_bit(E_SIG, E_PNTS_LSIG_GREEN);			
	} else if (RED == e_pnts_lsig) {
		set_bit(E_SIG, E_PNTS_LSIG_RED);			
		clear_bit(E_SIG, E_PNTS_LSIG_GREEN);			
	} else {
		clear_bit(E_SIG, E_PNTS_LSIG_RED);			
		clear_bit(E_SIG, E_PNTS_LSIG_GREEN);	
	}

	
}

void PktHandler(void)
{
	char i, j, k, bitnum, byte, mask;

	CRC16_High=0;
	CRC16_Low=0;

	/*************** PACKET FILTER ***************/

	/* Loopback Test - did we send it? */
	if (rx_buffer[PKT_SRC] == dev_addr) 
	{
		goto	PktIgnore;
	}

	/* Destination Test - is this for us? */
	if (rx_buffer[PKT_DEST] != 0xFF && rx_buffer[PKT_DEST] != dev_addr) 
	{
		goto	PktIgnore;
	}

	/* CRC16 Test - is the packet intact? */
	for(i=0; i<rx_buffer[PKT_LEN]; i++)
	{
		if ((i != PKT_CRC_H) && (i != PKT_CRC_L)) MRBus_CRC16_Update(rx_buffer[i]);
	}

	if ((CRC16_High != rx_buffer[PKT_CRC_H]) || (CRC16_Low != rx_buffer[PKT_CRC_L])) goto PktError;


	/*************** END PACKET FILTER ***************/

	/* Check for common stuff - debug assist packets, pings, etc. */
	if ('Z' == rx_buffer[PKT_TYPE])
	{
		tx_buffer[PKT_SRC] = EepromRead(EE_DEVICE_ADDR);
		tx_buffer[PKT_DEST] = rx_buffer[PKT_SRC];
		tx_buffer[PKT_LEN] = 7;			
		tx_buffer[PKT_TYPE] = 'z';
		fsr = rx_buffer[6];
		tx_buffer[6] = INDF;			
		state = state | TX_PKT_READY;
		goto PktIgnore;

	} else if ('W' == rx_buffer[PKT_TYPE]) {

		tx_buffer[PKT_DEST] = rx_buffer[PKT_SRC];
		tx_buffer[PKT_LEN] = 7;			
		tx_buffer[PKT_TYPE] = 'w';
		tx_buffer[6] = EepromWrite(rx_buffer[6], rx_buffer[7]);
		tx_buffer[PKT_SRC] = EepromRead(EE_DEVICE_ADDR);
		dev_addr = EepromRead(EE_DEVICE_ADDR);
		state = state | TX_PKT_READY;
		goto PktIgnore;
	} else if ('R' == rx_buffer[PKT_TYPE]) {

		tx_buffer[PKT_SRC] = EepromRead(EE_DEVICE_ADDR);
		tx_buffer[PKT_DEST] = rx_buffer[PKT_SRC];
		tx_buffer[PKT_LEN] = 7;			
		tx_buffer[PKT_TYPE] = 'r';
		tx_buffer[6] = EepromRead(rx_buffer[6]);			
		state = state | TX_PKT_READY;
		goto PktIgnore;
	} else if ('C' == rx_buffer[PKT_TYPE]) {
		/* Actually do CTC clearing */
		CodeCTCRoute(rx_buffer[6], rx_buffer[7], 0, 0);
		goto PktIgnore;
	} else if ('B' == rx_buffer[PKT_TYPE]) {
		/* Actually do CTC clearing */
		CodeCTCRoute(0, 0, rx_buffer[6], rx_buffer[7]);
		goto PktIgnore;
	}

	/*************** PACKET SUCCESS - PROCESS HERE ***************/
	for (i = 0; i<6; i++)
	{
		byte = EepromRead(i+EE_E_APRCH_ADDR);
		if (rx_buffer[PKT_SRC] == byte)
		{
			byte = EepromRead(i+EE_E_APRCH_PKT);
			if (rx_buffer[PKT_TYPE] != byte) continue;
	
			/* BITBYTE is computed as follows:
				x = bit = 0-7
				y = byte = byte in data stream (6 is first data byte)
				xxxyyyy
			*/
				
				
			byte = EepromRead(i+EE_E_APRCH_BITBYTE);
			bitnum = byte & 0xE0;
			bitnum = (bitnum>>5) & 0x07;
			byte = byte & 0x1F;

			mask = 0x01;
			
			for (j=0; j<bitnum; j++) mask = mask<<1;		

			bitnum = rx_buffer[byte] & mask;

			/* 
			#define E_ADJOIN			0x80
			#define E_APPROACH		0x40
			#define W_ADJOIN			0x20
			#define W_APPROACH		0x10
			#define CTC_MAIN			0x08
			#define CTC_SIDING		0x04
			#define E_OS_SECT			0x02
			#define W_OS_SECT			0x01
			*/

			if (bitnum)
			{
				switch(i+EE_E_APRCH_ADDR)
				{
					case EE_E_APRCH_ADDR:
						occupancy = occupancy | E_APPROACH;
						break;
	
					case EE_E_APRCH2_ADDR:
						occupancy = occupancy | E_APPROACH;
						break;
	
					case EE_E_ADJ_ADDR:
						occupancy = occupancy | E_ADJOIN;
						break;
						
					case EE_W_APRCH_ADDR:
						occupancy = occupancy | W_APPROACH;
						break;
	
					case EE_W_APRCH2_ADDR:
						occupancy = occupancy | W_APPROACH;
						break;
	
					case EE_W_ADJ_ADDR:
						occupancy = occupancy | W_ADJOIN;
						break;
					default:
						break;
	
				}
			} else {
				switch(i+EE_E_APRCH_ADDR)
				{
					case EE_E_APRCH_ADDR:
						occupancy = occupancy & (~E_APPROACH);
						break;
	
					case EE_E_APRCH2_ADDR:
						occupancy = occupancy & (~E_APPROACH);					
						break;
	
					case EE_E_ADJ_ADDR:
						occupancy = occupancy & (~E_ADJOIN);
						break;
						
					case EE_W_APRCH_ADDR:
						occupancy = occupancy & (~W_APPROACH);
						break;
	
					case EE_W_APRCH2_ADDR:
						occupancy = occupancy & (~W_APPROACH);
						break;
	
					case EE_W_ADJ_ADDR:
						occupancy = occupancy & (~W_ADJOIN);
						break;
					default:
						break;
	
				}
			}			
		}
	}
	state = state & (~RX_PKT_READY);
	return;


PktError:

	/*************** PACKET ERROR ***************/

PktIgnore:
	state = state & (~RX_PKT_READY);
	return;
}

void CodeCTCRoute(char e_turn, char e_clear, char w_turn, char w_clear)
{
	if (!(occupancy & E_OS_SECT))
	{
		if ('M' == e_turn)
		{
			set_bit(E_TURNOUT, E_PNTS_MAIN);
		} else if ('D' == e_turn) {
			set_bit(E_TURNOUT, E_PNTS_DIVERGING);
		}
	}

	if (!(occupancy & W_OS_SECT))
	{
		if ('M' == w_turn)
		{
			set_bit(W_TURNOUT, W_PNTS_MAIN);
		} else if ('D' == w_turn) {
			set_bit(W_TURNOUT, W_PNTS_DIVERGING);
		}
	}
	
	if ('E' == e_clear && !(occupancy & E_OS_SECT))
	{
			clearance = (clearance & 0xF0) | E_CLEAR_EAST;
	} else if ('W' == e_clear && !(occupancy & E_OS_SECT)) {
			clearance = (clearance & 0xF0) | E_CLEAR_WEST;
	} else if (0 != e_clear) {
			clearance = (clearance & 0xF0) | E_CLEAR_NONE;
	}

	if ('E' == w_clear && !(occupancy & W_OS_SECT))
	{
			clearance = (clearance & 0x0F) | W_CLEAR_EAST;
	} else if ('W' == w_clear && !(occupancy & W_OS_SECT)) {
			clearance = (clearance & 0x0F) | W_CLEAR_WEST;
	} else if (0 != w_clear) {
			clearance = (clearance & 0x0F) | W_CLEAR_NONE;
	}

	clear_bit(E_TURNOUT, E_PNTS_MAIN);
	clear_bit(E_TURNOUT, E_PNTS_DIVERGING);
	clear_bit(W_TURNOUT, W_PNTS_MAIN);
	clear_bit(W_TURNOUT, W_PNTS_DIVERGING);


}

