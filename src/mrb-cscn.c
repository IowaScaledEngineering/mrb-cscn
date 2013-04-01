/*************************************************************************
Title:    MRBus AVR Template
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "mrbus.h"
#include "avr-i2c-master.h"

extern uint8_t mrbus_activity;
extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;

uint8_t mrbus_dev_addr = 0;
uint8_t pkt_count = 0;
volatile uint8_t events = 0;

#define EVENT_READ_HS_INPUTS  0x01
#define EVENT_READ_LS_INPUTS  0x02
#define EVENT_WRITE_OUTPUTS   0x04

#define E_CONTROLPOINT  0x01
#define W_CONTROLPOINT  0x02

#define CLEARANCE_NONE		0x00
#define CLEARANCE_EAST		0x01
#define CLEARANCE_WEST		0x02

#define POINTS_MAIN_SAFE      'M'
#define POINTS_REVERSE_SAFE   'D'
#define POINTS_MAIN_FORCE     'm'
#define POINTS_REVERSE_FORCE  'd'
#define POINTS_UNAFFECTED     'X'

#define OCC_CTC_MAIN				0x08
#define OCC_CTC_SIDING			0x04
#define OCC_E_OS_SECT			0x02
#define OCC_W_OS_SECT			0x01
#define XOCC_E_ADJOIN			0x04
#define XOCC_E_APPROACH			0x02
#define XOCC_E_APPROACH2		0x01
#define XOCC_W_ADJOIN			0x10
#define XOCC_W_APPROACH			0x20
#define XOCC_W_APPROACH2		0x40


#define E_PNTS_BTTN_NORMAL  0x01
#define E_PNTS_BTTN_REVERSE 0x02
#define W_PNTS_BTTN_NORMAL  0x04
#define W_PNTS_BTTN_REVERSE 0x08

#define E_PNTS_CNTL         0x10
#define W_PNTS_CNTL         0x20


#define EE_E_APRCH_ADDR       0x10
#define EE_E_APRCH2_ADDR      0x11
#define EE_E_ADJ_ADDR         0x12
#define EE_W_APRCH_ADDR       0x13
#define EE_W_APRCH2_ADDR      0x14
#define EE_W_ADJ_ADDR         0x15

#define EE_E_APRCH_PKT        0x20
#define EE_E_APRCH2_PKT       0x21
#define EE_E_ADJ_PKT          0x22
#define EE_W_APRCH_PKT        0x23
#define EE_W_APRCH2_PKT       0x24
#define EE_W_ADJ_PKT          0x25

#define EE_E_APRCH_BITBYTE    0x30
#define EE_E_APRCH2_BITBYTE   0x31
#define EE_E_ADJ_BITBYTE      0x32
#define EE_W_APRCH_BITBYTE    0x33
#define EE_W_APRCH2_BITBYTE   0x34
#define EE_W_ADJ_BITBYTE      0x35

#define EE_E_APRCH_SUBTYPE    0x40
#define EE_E_APRCH2_SUBTYPE   0x41
#define EE_E_ADJ_SUBTYPE      0x42
#define EE_W_APRCH_SUBTYPE    0x43
#define EE_W_APRCH2_SUBTYPE   0x44
#define EE_W_ADJ_SUBTYPE      0x45




// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

uint8_t ticks;
uint16_t decisecs=0;
uint16_t update_decisecs=10;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}
}

// End of 100Hz timer

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	switch(mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		case 'A':
			// PING packet
			mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 6;
			mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
			mrbus_state |= MRBUS_TX_PKT_READY;
			goto PktIgnore;

		case 'C':
			// CTC Command
			// There are two forms of this - the 8 byte packet (old school) and the 9 byte packet (new)
			// Old school implied the east control point.  New specifies the CP number in the first byte
			if (8 == mrbus_rx_buffer[MRBUS_PKT_LEN])
				CodeCTCRoute(E_CONTROLPOINT, mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
			else
				CodeCTCRoute(mrbus_rx_buffer[6], mrbus_rx_buffer[7], mrbus_rx_buffer[8]);
			goto PktIgnore;

		case 'B':
			// CTC Command
			// B is the old school variant of C for the western control point
			CodeCTCRoute(W_CONTROLPOINT, mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
			goto PktIgnore;

		case 'T':
			if (1 == mrbus_rx_buffer[6])
				CodeCTCRoute(E_CONTROLPOINT, mrbus_rx_buffer[6], CLEARANCE_NONE);
			else if (2 == mrbus_rx_buffer[6])
				CodeCTCRoute(W_CONTROLPOINT, mrbus_rx_buffer[6], CLEARANCE_NONE);		
			goto PktIgnore;

		case 'W':
			// EEPROM WRITE Packet
			mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
			mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
			eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
			mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
			mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
			if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
				mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_state |= MRBUS_TX_PKT_READY;
			goto PktIgnore;

		case 'R':
			// EEPROM READ Packet
			mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
			mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
			mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
			mrbus_tx_buffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
			mrbus_state |= MRBUS_TX_PKT_READY;
			goto PktIgnore;

		case 'V':
			// Version
			mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 16;
			mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'v';
			mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRED;
			mrbus_tx_buffer[7]  = 0; // Software Revision
			mrbus_tx_buffer[8]  = 0; // Software Revision
			mrbus_tx_buffer[9]  = 0; // Software Revision
			mrbus_tx_buffer[10]  = 1; // Hardware Major Revision
			mrbus_tx_buffer[11]  = 0; // Hardware Minor Revision
			mrbus_tx_buffer[12] = 'C';
			mrbus_tx_buffer[13] = 'S';
			mrbus_tx_buffer[14] = 'C';
			mrbus_tx_buffer[15] = 'N';
			mrbus_state |= MRBUS_TX_PKT_READY;
			goto PktIgnore;

		case 'X':
			// Reset
			cli();
			wdt_reset();
			MCUSR &= ~(_BV(WDRF));
			WDTCSR |= _BV(WDE) | _BV(WDCE);
			WDTCSR = _BV(WDE);
			while(1);  // Force a watchdog reset, hopefully
			sei();
			break;
	}


	/*************** NOT A PACKET WE EXPLICITLY UNDERSTAND, TRY BIT/BYTE RULES ***************/
	for (i=0; i<6; i++)
	{
		uint8_t byte, bitset=0;
		
		if (mrbus_rx_buffer[MRBUS_PKT_SRC] != eeprom_read_byte((uint8_t*)i+EE_E_APRCH_ADDR))
			continue;

		if (mrbus_rx_buffer[MRBUS_PKT_TYPE] != eeprom_read_byte((uint8_t*)i+EE_E_APRCH_PKT))
			continue;
		
		byte = eeprom_read_byte((uint8_t*)i+EE_E_APRCH_SUBTYPE);
		if ((0xFF != byte) && (mrbus_rx_buffer[MRBUS_PKT_SUBTYPE] != byte))
			continue;

		/* BITBYTE is computed as follows:
			x = bit = 0-7
			y = byte = byte in data stream (6 is first data byte)
			xxxyyyy
		*/
		byte = eeprom_read_byte((uint8_t*)i+EE_E_APRCH_BITBYTE);
		bitset = mrbus_rx_buffer[(byte & 0x1F)] & (1<<((byte>>5) & 0x07));

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

		switch(i + EE_E_APRCH_ADDR)
		{
			case EE_E_APRCH_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_E_APPROACH;
				else
					ext_occupancy &= ~(XOCC_E_APPROACH);
				break;

			case EE_E_APRCH2_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_E_APPROACH2;
				else
					ext_occupancy &= ~(XOCC_E_APPROACH2);
				break;
				
			case EE_E_ADJ_ADDR
				if (bitset)
					ext_occupancy |= XOCC_E_ADJOIN;
				else
					ext_occupancy &= ~(XOCC_E_ADJOIN);
				break;
			
			case EE_W_APRCH_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_W_APPROACH;
				else
					ext_occupancy &= ~(XOCC_W_APPROACH);
				break;

			case EE_W_APRCH2_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_W_APPROACH2;
				else
					ext_occupancy &= ~(XOCC_W_APPROACH2);
				break;
				
			case EE_W_ADJ_ADDR
				if (bitset)
					ext_occupancy |= XOCC_W_ADJOIN;
				else
					ext_occupancy &= ~(XOCC_W_ADJOIN);
				break;
		}
	}
	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
	return;	
}

/* 0x00-0x04 - input registers */
/* 0x08-0x0C - output registers */
/* 0x18-0x1C - direction registers - 0 is output, 1 is input */

#define I2C_RESET         0
#define I2C_OUTPUT_ENABLE 1
#define I2C_IRQ           2


#define I2C_XIO1_ADDRESS 0x40

void xioInitialize()
{
	uint8_t i2cBuf[8];

//	DDRB |= _BV(I2C_RESET) | _BV(I2C_OUTPUT_ENABLE);
//	PORTB &= ~(_BV(I2C_OUTPUT_ENABLE));
//	PORTB |= _BV(I2C_RESET);

	DDRB = 0x03;
	PORTB = 0x01;

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x18;  // 0x80 is auto-increment
	i2cBuf[2] = 0;
	i2cBuf[3] = 0;
	i2cBuf[4] = 0;
	i2cBuf[5] = 0xC0;
	i2cBuf[6] = 0xFF;
	i2c_transmit(i2cBuf, 7, 1);
	while(i2c_busy());
}


void init(void)
{
	// FIXME:  Do any initialization you need to do here.
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
	wdt_reset();
	wdt_disable();

	pkt_count = 0;

	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	mrbus_dev_addr = 0x20;
}

uint8_t xio1Outputs[4];
uint8_t xio1Inputs[2];

void xioOutputWrite()
{
	uint8_t i2cBuf[8];
	uint8_t i;
	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x08;  // 0x80 is auto-increment
	for(i=0; i<sizeof(xio1Outputs); i++)
		i2cBuf[2+i] = xio1Outputs[i];

	i2c_transmit(i2cBuf, 2+sizeof(xio1Outputs), 1);
}

void xioInputRead()
{
	uint8_t i2cBuf[8];
	uint8_t i;
	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x03;  // 0x80 is auto-increment, 0x03 is the first register with inputs
	i2c_transmit(i2cBuf, 2, 0);
	
	i2cBuf[0] = I2C_XIO1_ADDRESS | 0x01;
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());
	i2c_receive(i2cBuf, 3);
	xio1Inputs[0] = i2cBuf[1];
	xio1Inputs[1] = i2cBuf[2];	
}

uint8_t debounced_inputs[2] = {0,0}, old_debounced_inputs[2] = {0,0};
uint8_t clearance=0, old_clearance=0;
uint8_t occupancy=0, old_occupancy=0;
uint8_t ext_occupancy=0, old_ext_occupancy=0;
uint8_t clock_a[2] = {0,0}, clock_b[2] = {0,0};

void SetTurnout(uint8_t controlPoint, uint8_t points)
{
	if (POINTS_UNAFFECTED == points)
		return;
		
	switch(controlPoint)
	{
		case E_CONTROLPOINT:
			if (POINTS_REVERSE_FORCE == points || (POINTS_REVERSE_SAFE == points && !(occupancy & E_OS_SECT)))
				xio1Output[3] |= E_PNTS_CNTL;
			else if (POINTS_MAIN_FORCE == points || (POINTS_MAIN_SAFE == points && !(occupancy & E_OS_SECT)))
				xio1Output[3] &= ~(E_PNTS_CNTL);
			break;

		case W_CONTROLPOINT:
			if (POINTS_REVERSE_FORCE == points || (POINTS_REVERSE_SAFE == points && !(occupancy & W_OS_SECT)))
				xio1Output[3] |= W_PNTS_CNTL;
			else if (POINTS_MAIN_FORCE == points || (POINTS_MAIN_SAFE == points && !(occupancy & W_OS_SECT)))
				xio1Output[3] &= ~(W_PNTS_CNTL);
			break;
	}
}

void SetClearance(uint8_t controlPoint, uint8_t newClear)
{
	if (CLEARANCE_NONE != newClear 
		&& CLEARANCE_EAST  != newClear 
		&& CLEARANCE_WEST != newClear)
		return;

	switch(controlPoint)
	{
		case E_CONTROLPOINT:
			clearance &= 0xF0;
			clearance |= newClear;
			break;
		case W_CONTROLPOINT:
			clearance &= 0x0F;
			clearance |= newClear<<4;
			break;
	}
}

void CodeCTCRoute(uint8_t controlPoint, uint8_t newClear, uint8_t newPoints)
{
	SetClearance(controlPoint, newClear);
	SetTurnout(controlPoint, newPoints);						
}



void signalsToOuputs()
{



}

int main(void)
{
	uint8_t changed = 0;
	uint8_t i;
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();

	sei();	
	i2c_master_init();
	xioInitialize();

	while (1)
	{
		wdt_reset();
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();

		if(events & (EVENT_READ_INPUTS))
		{
			uint8_t delta;
			xioInputRead();
			for(i=0; i<2; i++)
			{
				// Vertical counter debounce courtesy 
				delta = xio1Inputs[i] ^ debounced_inputs[i];
				clock_a[i] ^= clock_b[i];
				clock_b[i]  = ~(clock_b[i]);
				clock_a[i] &= delta;
				clock_b[i] &= delta;
				debounced_inputs[i] ^= ~(~delta | clock_a[i] | clock_b[i]);
			}
		}			

		// Control Point Logic

		// Manual Control Logic
		if (1) // FIXME: add provision for LC lockout
		{
			uint8_t delta = (debounced_inputs[1] ^ old_debounced_inputs[1]) & (E_PNTS_BTTN_NORMAL | E_PNTS_BTTN_REVERSE);
			delta &= old_debounced_inputs[1];
			
			// Any bit in delta that's a 1 now corresponds to a switch going low in this cycle
			if (delta & E_PNTS_BTTN_NORMAL)
				CodeCTCRoute(E_CONTROLPOINT, POINTS_MAIN_FORCE, CLEARANCE_NONE);
			else if (delta & E_PNTS_BUTTON_REVERSE)
				CodeCTCRoute(E_CONTROLPOINT, POINTS_REVERSE_FORCE, CLEARANCE_NONE);
				
			if (delta & W_PNTS_BTTN_NORMAL)
				CodeCTCRoute(W_CONTROLPOINT, POINTS_MAIN_FORCE, CLEARANCE_NONE);
			else if (delta & W_PNTS_BUTTON_REVERSE)
				CodeCTCRoute(W_CONTROLPOINT, POINTS_REVERSE_FORCE, CLEARANCE_NONE);
		}

		// Get the physical occupancy inputs from debounced
		occupancy &= 0xF0;
		occupancy |= 0x0F & (debounced_inputs[1]>>4);

		if (occupancy & OCC_E_OS_SECT)
			setClearance(E_CONTROLPOINT, CLEAR_NONE);

		if (occupancy & OCC_W_OS_SECT)
			setClearance(W_CONTROLPOINT, CLEAR_NONE);


		// Calculate signal aspects
		east_os_section_signals();
		west_os_section_signals();
		
		// Vital Logic



		// Send output
		if (events & EVENT_WRITE_OUTPUTS)
		{
			signalsToOutputs();
			xioOutputWrite();
			events &= ~(EVENT_WRITE_OUTPUTS);
		}


		// Test if something changed from the last time
		// around the loop - we need to send an update 
		//   packet if it did 
		   
		if ((old_e_pnts_usig != e_pnts_usig) ||
		(old_e_pnts_lsig != e_pnts_lsig) ||
		(old_e_main_sig != e_main_sig) ||
		(old_e_side_sig != e_side_sig) ||
		(old_w_pnts_usig != w_pnts_usig) ||
		(old_w_pnts_lsig != w_pnts_lsig) ||
		(old_w_main_sig != w_main_sig) ||
		(old_w_side_sig != w_side_sig) ||
		(old_occupancy != occupancy) ||
		(old_ext_occupancy != ext_occupancy) ||
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
			old_ext_occupancy = ext_occupancy;
			old_clearance = clearance;
			old_turnouts = turnouts;
			
			/* Set changed such that a packet gets sent */
			changed = 1;
		}
		else if (decisecs >= update_decisecs)
			changed = 1;

		if (changed)
			decisecs = 0;

		/* If we need to send a packet and we're not already busy... */
		if (changed && !(mrbus_state & (MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY)))
		{
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 7;			
			mrbus_tx_buffer[5] = 'S';
			mrbus_tx_buffer[6] = 0x00;
			mrbus_tx_buffer[7] = 0x00;
			mrbus_state |= MRBUS_TX_PKT_READY;
			changed = 0;
		}


		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			uint8_t bus_countdown;

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
				if (mrbus_state & MRBUS_RX_PKT_READY) 
					PktHandler();
			}
		}
	}
}



