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
#include <string.h>
#include <util/delay.h>

#include "mrbus.h"
#include "avr-i2c-master.h"

void PktHandler(void);

extern volatile uint8_t mrbus_activity;
extern volatile uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern volatile uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern volatile uint8_t mrbus_state;

uint8_t mrbus_dev_addr = 0;
volatile uint8_t events = 0;

#define EVENT_READ_INPUTS    0x01
#define EVENT_WRITE_OUTPUTS  0x02
#define EVENT_I2C_ERROR      0x40
#define EVENT_BLINKY         0x80

#define E_CONTROLPOINT  0x01
#define W_CONTROLPOINT  0x02

#define CLEARANCE_NONE		0x00
#define CLEARANCE_EAST		0x01
#define CLEARANCE_WEST		0x02

#define POINTS_NORMAL_SAFE    'M'
#define POINTS_REVERSE_SAFE   'D'
#define POINTS_NORMAL_FORCE   'm'
#define POINTS_REVERSE_FORCE  'd'
#define POINTS_UNAFFECTED     'X'

#define OCC_CTC_MAIN				0x08
#define OCC_CTC_SIDING			0x04
#define OCC_E_OS_SECT			0x02
#define OCC_W_OS_SECT			0x01

#define OCC_VIRT_E_ADJOIN     0x10
#define OCC_VIRT_E_APPROACH   0x20
#define OCC_VIRT_W_ADJOIN     0x40
#define OCC_VIRT_W_APPROACH   0x80

#define XOCC_E_ADJOIN			0x01
#define XOCC_E_APPROACH			0x02
#define XOCC_E_APPROACH2		0x04
#define XOCC_E_TUMBLE         0x08
#define XOCC_W_ADJOIN			0x10
#define XOCC_W_APPROACH			0x20
#define XOCC_W_APPROACH2		0x40
#define XOCC_W_TUMBLE         0x80


#define E_PNTS_BTTN_NORMAL  0x01
#define E_PNTS_BTTN_REVERSE 0x02
#define W_PNTS_BTTN_NORMAL  0x04
#define W_PNTS_BTTN_REVERSE 0x08

#define E_PNTS_CNTL         0x10
#define W_PNTS_CNTL         0x20
#define E_PNTS_STATUS       0x40
#define W_PNTS_STATUS       0x80

// EEPROM Location Definitions

#define EE_E_APRCH_ADDR       0x10
#define EE_E_APRCH2_ADDR      0x11
#define EE_E_ADJ_ADDR         0x12
#define EE_W_APRCH_ADDR       0x13
#define EE_W_APRCH2_ADDR      0x14
#define EE_W_ADJ_ADDR         0x15
#define EE_E_TUMBLE_ADDR      0x16
#define EE_W_TUMBLE_ADDR      0x17

#define EE_E_APRCH_PKT        0x20
#define EE_E_APRCH2_PKT       0x21
#define EE_E_ADJ_PKT          0x22
#define EE_W_APRCH_PKT        0x23
#define EE_W_APRCH2_PKT       0x24
#define EE_W_ADJ_PKT          0x25
#define EE_E_TUMBLE_PKT       0x26
#define EE_W_TUMBLE_PKT       0x27

#define EE_E_APRCH_BITBYTE    0x30
#define EE_E_APRCH2_BITBYTE   0x31
#define EE_E_ADJ_BITBYTE      0x32
#define EE_W_APRCH_BITBYTE    0x33
#define EE_W_APRCH2_BITBYTE   0x34
#define EE_W_ADJ_BITBYTE      0x35
#define EE_E_TUMBLE_BITBYTE   0x36
#define EE_W_TUMBLE_BITBYTE   0x37

#define EE_E_APRCH_SUBTYPE    0x40
#define EE_E_APRCH2_SUBTYPE   0x41
#define EE_E_ADJ_SUBTYPE      0x42
#define EE_W_APRCH_SUBTYPE    0x43
#define EE_W_APRCH2_SUBTYPE   0x44
#define EE_W_ADJ_SUBTYPE      0x45
#define EE_E_TUMBLE_SUBTYPE   0x46
#define EE_W_TUMBLE_SUBTYPE   0x47

uint8_t debounced_inputs[2], old_debounced_inputs[2];
uint8_t clearance, old_clearance;
uint8_t occupancy, old_occupancy;
uint8_t ext_occupancy, old_ext_occupancy;
uint8_t turnouts, old_turnouts;
uint8_t clock_a[2] = {0,0}, clock_b[2] = {0,0};
uint8_t signalHeads[8], old_signalHeads[8];

uint8_t PktDirToClearance(uint8_t pktDir)
{
	switch(pktDir)
	{
		case 'E':
			return(CLEARANCE_EAST);
		case 'W':
			return(CLEARANCE_WEST);
		default:
			return(CLEARANCE_NONE);
	}

}

// Aspect Definitions

#define ASPECT_LUNAR     0x07
#define ASPECT_FL_RED    0x06
#define ASPECT_FL_GREEN  0x05
#define ASPECT_RED       0x04
#define ASPECT_FL_YELLOW 0x03
#define ASPECT_YELLOW    0x02
#define ASPECT_GREEN     0x01
#define ASPECT_OFF       0x00

// Signal Head Definitions

#define SIG_E_PNTS_UPPER 0
#define SIG_E_PNTS_LOWER 1
#define SIG_E_MAIN       2
#define SIG_E_SIDING     3
#define SIG_W_PNTS_UPPER 4
#define SIG_W_PNTS_LOWER 5
#define SIG_W_MAIN       6
#define SIG_W_SIDING     7

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
uint8_t blinkyCounter = 0;
volatile uint8_t buttonLockout=5;

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
	if (ticks & 0x01)
		events |= EVENT_READ_INPUTS;

	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;

		if (++blinkyCounter > 5)
		{
			events ^= EVENT_BLINKY;
			blinkyCounter = 0;
		}

		if (buttonLockout != 0)
			buttonLockout--;

		events |= EVENT_WRITE_OUTPUTS;
	}
}

// End of 100Hz timer

// Bus voltage monitor

volatile uint16_t busVoltageAccum=0;
volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

ISR(ADC_vect)
{
	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/3) / 5 * 1024
        //So multiply by 150, divide by 1024, or multiply by 75 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 75) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}


void init(void)
{
	uint8_t i;
	// Clear watchdog
	MCUSR = 0;
	wdt_reset();
	wdt_disable();

	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x46;  // AVCC reference; ADC6 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	busVoltageAccum = 0;
	busVoltageCount = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

	// Initialize signals and such
	for(i=0; i<sizeof(debounced_inputs); i++)
		debounced_inputs[i] = old_debounced_inputs[i] = 0;
	
	for(i=0; i<sizeof(signalHeads); i++)
		signalHeads[i] = old_signalHeads[i] = ASPECT_RED;

	clearance = old_clearance = 0;
	occupancy = old_occupancy = 0;
	ext_occupancy = old_ext_occupancy = 0;
	turnouts = old_turnouts = 0;
}

uint8_t xio1Outputs[4];
uint8_t xio1Inputs[2];

/* 0x00-0x04 - input registers */
/* 0x08-0x0C - output registers */
/* 0x18-0x1C - direction registers - 0 is output, 1 is input */

#define I2C_RESET         0
#define I2C_OUTPUT_ENABLE 1
#define I2C_IRQ           2
#define I2C_XIO1_ADDRESS 0x4E

void xioInitialize()
{
	uint8_t i2cBuf[8];

	events |= EVENT_I2C_ERROR;

	PORTB &= ~(_BV(I2C_RESET) | _BV(I2C_OUTPUT_ENABLE));
	DDRB |= _BV(I2C_RESET) | _BV(I2C_OUTPUT_ENABLE);
	PORTB &= ~(_BV(I2C_OUTPUT_ENABLE));
	PORTB |= _BV(I2C_RESET);

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x18;  // 0x80 is auto-increment
	i2cBuf[2] = 0;
	i2cBuf[3] = 0;
	i2cBuf[4] = 0;
	i2cBuf[5] = 0xC0;
	i2cBuf[6] = 0xFF;
	i2c_transmit(i2cBuf, 7, 1);
	while(i2c_busy());

	if (I2C_NO_STATE == i2c_state)
	{
		buttonLockout = 5;
		events &= ~(EVENT_I2C_ERROR);
	}
}

void xioOutputWrite()
{
	uint8_t i2cBuf[8];
	uint8_t i;

	while(i2c_busy());
	if (i2c_state != I2C_NO_STATE)
		events |= EVENT_I2C_ERROR;

	if (events & EVENT_I2C_ERROR)
		return;

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x08;  // 0x80 is auto-increment
	for(i=0; i<sizeof(xio1Outputs); i++)
		i2cBuf[2+i] = xio1Outputs[i];

	i2c_transmit(i2cBuf, 2+sizeof(xio1Outputs), 1);
}

void xioInputRead()
{
	uint8_t i2cBuf[4];

	if (events & EVENT_I2C_ERROR)
		return;

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x03;  // 0x80 is auto-increment, 0x03 is the first register with inputs
	i2c_transmit(i2cBuf, 2, 0);
	i2cBuf[0] = I2C_XIO1_ADDRESS | 0x01;
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());
	i2c_receive(i2cBuf, 3);

	if (i2c_state != I2C_NO_STATE)
		// In the event of a read hose-out, don't put crap in the input buffer
		events |= EVENT_I2C_ERROR;
	else
	{
		xio1Inputs[0] = i2cBuf[1];
		xio1Inputs[1] = i2cBuf[2];	
	}
}

void SetTurnout(uint8_t controlPoint, uint8_t points)
{
	if (POINTS_UNAFFECTED == points)
		return;
		
	switch(controlPoint)
	{
		case E_CONTROLPOINT:
			if (POINTS_REVERSE_FORCE == points || (POINTS_REVERSE_SAFE == points && !(occupancy & OCC_E_OS_SECT)))
			{
				turnouts |= E_PNTS_CNTL;
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				xio1Outputs[3] |= E_PNTS_CNTL;
			}
			else if (POINTS_NORMAL_FORCE == points || (POINTS_NORMAL_SAFE == points && !(occupancy & OCC_E_OS_SECT)))
			{
				turnouts &= ~(E_PNTS_CNTL);
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				xio1Outputs[3] &= ~(E_PNTS_CNTL); 
			}
			break;

		case W_CONTROLPOINT:
			if (POINTS_REVERSE_FORCE == points || (POINTS_REVERSE_SAFE == points && !(occupancy & OCC_W_OS_SECT)))
			{
				turnouts |= W_PNTS_CNTL;
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				xio1Outputs[3] |= W_PNTS_CNTL;
			}
			else if (POINTS_NORMAL_FORCE == points || (POINTS_NORMAL_SAFE == points && !(occupancy & OCC_W_OS_SECT)))
			{
				turnouts &= ~(W_PNTS_CNTL);
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				xio1Outputs[3] &= ~(W_PNTS_CNTL);
			}
			break;
	}
}

uint8_t GetClearance(uint8_t controlPoint)
{
	switch(controlPoint)
	{
		case E_CONTROLPOINT:
			return(clearance & 0x0F);

		case W_CONTROLPOINT:
			return((clearance>>4) & 0x0F);
	}

	return(CLEARANCE_NONE);
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
			if (CLEARANCE_NONE != newClear)
			{
				if (XOCC_E_TUMBLE & ext_occupancy)
					break;
				if (OCC_E_OS_SECT & occupancy)
					break;

				// FIXME: logic to prevent the CTC ends lining into each other
			}
			clearance &= 0xF0;
			clearance |= newClear;
			break;

		case W_CONTROLPOINT:
			if (CLEARANCE_NONE != newClear)
			{
				if (XOCC_W_TUMBLE & ext_occupancy)
					break;
				if (OCC_W_OS_SECT & occupancy)
					break;	

				// FIXME: logic to prevent the CTC ends lining into each other
			}
			
			clearance &= 0x0F;
			clearance |= newClear<<4;
			break;
	}
}

void CodeCTCRoute(uint8_t controlPoint, uint8_t newPoints, uint8_t newClear)
{
	SetClearance(controlPoint, newClear);
	SetTurnout(controlPoint, newPoints);						
}


// SignalsToOutputs is responsible for converting the signal head aspects in the
// signalHeads[] array to the physical wires on the XIO.
// Thus, it's hardware configuration dependent.

/* ASPECT_LUNAR     0x07
 ASPECT_FL_RED    0x06
 ASPECT_FL_GREEN  0x05
 ASPECT_RED       0x04
 ASPECT_FL_YELLOW 0x03
 ASPECT_YELLOW    0x02
 ASPECT_GREEN     0x01
 ASPECT_OFF       0x00 */

typedef struct
{
	const uint8_t signalHead;
	const uint8_t redByte;
	const uint8_t redMask;
	const uint8_t greenByte;
	const uint8_t greenMask;
} SignalPinDefinition;

const SignalPinDefinition sigPinDefs[8] = 
{
	{SIG_W_PNTS_UPPER, 0, _BV(0), 0, _BV(2)},
	{SIG_W_PNTS_LOWER, 0, _BV(3), 0, _BV(5)},
	{SIG_W_MAIN      , 0, _BV(6), 1, _BV(0)},
	{SIG_W_SIDING    , 1, _BV(1), 1, _BV(3)},
	{SIG_E_PNTS_UPPER, 1, _BV(4), 1, _BV(6)},
	{SIG_E_PNTS_LOWER, 1, _BV(7), 2, _BV(1)},
	{SIG_E_MAIN      , 2, _BV(2), 2, _BV(4)},
	{SIG_E_SIDING    , 2, _BV(5), 2, _BV(7)}
};

void SignalsToOutputs()
{
	uint8_t sigDefIdx;
	for(sigDefIdx=0; sigDefIdx<sizeof(sigPinDefs)/sizeof(SignalPinDefinition); sigDefIdx++)
	{
		uint8_t redByte = sigPinDefs[sigDefIdx].redByte;
		uint8_t redMask = sigPinDefs[sigDefIdx].redMask;
		uint8_t greenByte = sigPinDefs[sigDefIdx].greenByte;
		uint8_t greenMask = sigPinDefs[sigDefIdx].greenMask;

		xio1Outputs[redByte] &= ~(redMask);
		xio1Outputs[greenByte] &= ~(greenMask);

		switch(signalHeads[sigPinDefs[sigDefIdx].signalHead])
		{
			case ASPECT_OFF:
				break;
		
			case ASPECT_GREEN:
				xio1Outputs[greenByte] |= greenMask;
				break;
		
			case ASPECT_FL_GREEN:
				if (events & EVENT_BLINKY)
					xio1Outputs[greenByte] |= greenMask;
				break;

			case ASPECT_YELLOW:
				xio1Outputs[redByte] |= redMask;
				xio1Outputs[greenByte] |= greenMask;		
				break;
		
			case ASPECT_FL_YELLOW:
				if (events & EVENT_BLINKY)
				{
					xio1Outputs[redByte] |= redMask;
					xio1Outputs[greenByte] |= greenMask;
				}
				break;
		
		
			case ASPECT_RED:
			case ASPECT_LUNAR: // Can't display, so make like red
			default:
				xio1Outputs[redByte] |= redMask;			
				break;

			case ASPECT_FL_RED:
				if (events & EVENT_BLINKY)
					xio1Outputs[redByte] |= redMask;
				break;
		}
	}
}

static inline void vitalLogic()
{
	uint8_t eastTurnoutLocked = (!(((turnouts & (E_PNTS_STATUS))?1:0) ^ ((turnouts & (E_PNTS_CNTL))?1:0)));
	uint8_t westTurnoutLocked = (!(((turnouts & (W_PNTS_STATUS))?1:0) ^ ((turnouts & (W_PNTS_CNTL))?1:0)));	
	uint8_t eastCleared = CLEARANCE_NONE;
	uint8_t westCleared = CLEARANCE_NONE;	

	// Start out with a safe default - everybody red
	signalHeads[SIG_E_PNTS_UPPER] = ASPECT_RED;
	signalHeads[SIG_E_PNTS_LOWER] = ASPECT_RED;
	signalHeads[SIG_E_MAIN] = ASPECT_RED;
	signalHeads[SIG_E_SIDING] = ASPECT_RED;

	signalHeads[SIG_W_PNTS_UPPER] = ASPECT_RED;
	signalHeads[SIG_W_PNTS_LOWER] = ASPECT_RED;
	signalHeads[SIG_W_MAIN] = ASPECT_RED;
	signalHeads[SIG_W_SIDING] = ASPECT_RED;

	// Drop clearance if we see occupancy
	if (occupancy & OCC_E_OS_SECT)
		SetClearance(E_CONTROLPOINT, CLEARANCE_NONE);
	if (occupancy & OCC_W_OS_SECT)
		SetClearance(W_CONTROLPOINT, CLEARANCE_NONE);

	eastCleared = GetClearance(E_CONTROLPOINT);
	westCleared = GetClearance(W_CONTROLPOINT);
	
	if (eastTurnoutLocked && CLEARANCE_EAST == eastCleared)
	{
		// Eastbound clearance at the east control point means frog->points movement direction
		uint8_t head = (turnouts & (E_PNTS_STATUS))?SIG_E_SIDING:SIG_E_MAIN;
		
		if (XOCC_E_ADJOIN & ext_occupancy || OCC_E_OS_SECT & occupancy)
			signalHeads[head] = ASPECT_RED;
		else if (XOCC_E_APPROACH & ext_occupancy)
			signalHeads[head] = ASPECT_YELLOW;
		else if (XOCC_E_APPROACH2 & ext_occupancy)
			signalHeads[head] = ASPECT_FL_YELLOW;
		else
			signalHeads[head] = ASPECT_GREEN;
	}
	else if (eastTurnoutLocked && CLEARANCE_WEST == eastCleared)
	{
		// Westbound clearance at the east control point means points->frog movement direction	
		if(turnouts & (E_PNTS_STATUS))
		{
			// Lined to siding
			if ((OCC_CTC_SIDING | OCC_E_OS_SECT) & occupancy)
				signalHeads[SIG_E_PNTS_LOWER] = ASPECT_RED;
			else if ( (XOCC_W_ADJOIN & ext_occupancy || OCC_W_OS_SECT & occupancy)
				|| !(turnouts & W_PNTS_STATUS)
				|| !westTurnoutLocked
				|| westCleared != CLEARANCE_WEST)
				signalHeads[SIG_E_PNTS_LOWER] = ASPECT_YELLOW;
			else
				// Change this to GREEN if highball is allowed into the siding
				signalHeads[SIG_E_PNTS_LOWER] = ASPECT_YELLOW;
		} else {
			// Lined to mainline
			if ((OCC_CTC_MAIN | OCC_E_OS_SECT) & occupancy)
				signalHeads[SIG_E_PNTS_UPPER] = ASPECT_RED;
			else if ((XOCC_W_ADJOIN & ext_occupancy) || (OCC_W_OS_SECT & occupancy)
				|| (turnouts & W_PNTS_STATUS)
				|| !westTurnoutLocked
				|| westCleared != CLEARANCE_WEST)
				signalHeads[SIG_E_PNTS_UPPER] = ASPECT_YELLOW;
			else if (XOCC_W_APPROACH & ext_occupancy)
				signalHeads[SIG_E_PNTS_UPPER] = ASPECT_FL_YELLOW;
			else
				signalHeads[SIG_E_PNTS_UPPER] = ASPECT_GREEN;
		}
	}
	// The else case is that the turnout isn't locked up or we're not cleared
	// Good news - the signals are already defaulted to red


	if (westTurnoutLocked && CLEARANCE_WEST == westCleared)
	{
		// Eastbound clearance at the east control point means frog->points movement direction
		uint8_t head = (turnouts & (W_PNTS_STATUS))?SIG_W_SIDING:SIG_W_MAIN;
		
		if (XOCC_W_ADJOIN & ext_occupancy || OCC_W_OS_SECT & occupancy)
			signalHeads[head] = ASPECT_RED;
		else if (XOCC_W_APPROACH & ext_occupancy)
			signalHeads[head] = ASPECT_YELLOW;
		else if (XOCC_W_APPROACH2 & ext_occupancy)
			signalHeads[head] = ASPECT_FL_YELLOW;
		else
			signalHeads[head] = ASPECT_GREEN;
	}
	else if (westTurnoutLocked && CLEARANCE_EAST == westCleared)
	{
		// Westbound clearance at the east control point means points->frog movement direction	
		if(turnouts & (W_PNTS_STATUS))
		{
			// Lined to siding
			if ((OCC_CTC_SIDING | OCC_W_OS_SECT) & occupancy)
				signalHeads[SIG_W_PNTS_LOWER] = ASPECT_RED;
			else if ( (XOCC_W_ADJOIN & ext_occupancy || OCC_W_OS_SECT & occupancy)
				|| !(turnouts & E_PNTS_STATUS)
				|| !eastTurnoutLocked
				|| eastCleared != CLEARANCE_EAST)
				signalHeads[SIG_W_PNTS_LOWER] = ASPECT_YELLOW;
			else
				// Change this to GREEN if highball is allowed into the siding
				signalHeads[SIG_W_PNTS_LOWER] = ASPECT_YELLOW;
		} else {
			// Lined to mainline
			if ((OCC_CTC_MAIN | OCC_W_OS_SECT) & occupancy)
				signalHeads[SIG_W_PNTS_UPPER] = ASPECT_RED;
			else if ((XOCC_E_ADJOIN & ext_occupancy) || (OCC_E_OS_SECT & occupancy)
				|| (turnouts & E_PNTS_STATUS)
				|| !eastTurnoutLocked
				|| eastCleared != CLEARANCE_EAST)
				signalHeads[SIG_W_PNTS_UPPER] = ASPECT_YELLOW;
			else if (XOCC_E_APPROACH & ext_occupancy)
				signalHeads[SIG_W_PNTS_UPPER] = ASPECT_FL_YELLOW;
			else
				signalHeads[SIG_W_PNTS_UPPER] = ASPECT_GREEN;
		}
	}

			
	// Clear virtual occupancies
	occupancy &= ~(OCC_VIRT_E_APPROACH | OCC_VIRT_E_ADJOIN | OCC_VIRT_W_APPROACH | OCC_VIRT_W_ADJOIN);

	// Calculate east CP virtual occupancies
	if(!(((turnouts & (E_PNTS_STATUS))?1:0) ^ ((turnouts & (E_PNTS_CNTL))?1:0)))
	{
		// Turnout is properly lined one way or the other
		if ((turnouts & E_PNTS_STATUS) && (ASPECT_RED == signalHeads[SIG_W_SIDING]))
			occupancy |= OCC_VIRT_E_APPROACH;
		else if ((!(turnouts & E_PNTS_STATUS)) && (ASPECT_RED == signalHeads[SIG_W_MAIN]))
			occupancy |= OCC_VIRT_E_APPROACH;
	
		if (ASPECT_RED == signalHeads[SIG_E_PNTS_LOWER] && ASPECT_RED == signalHeads[SIG_E_PNTS_UPPER])
			occupancy |= OCC_VIRT_E_ADJOIN;
	} else {
		// East Control Point improperly lined, trip virtual occupancy
			occupancy |= OCC_VIRT_E_APPROACH | OCC_VIRT_E_ADJOIN;
	}			

	// Calculate west CP virtual occupancies
	if(!(((turnouts & (W_PNTS_STATUS))?1:0) ^ ((turnouts & (W_PNTS_CNTL))?1:0)))
	{
		// Turnout is properly lined one way or the other
		if ((turnouts & W_PNTS_STATUS) && (ASPECT_RED == signalHeads[SIG_E_SIDING]))
			occupancy |= OCC_VIRT_W_APPROACH;
		else if ((!(turnouts & W_PNTS_STATUS)) && (ASPECT_RED == signalHeads[SIG_E_MAIN]))
			occupancy |= OCC_VIRT_W_APPROACH;
	
		if (ASPECT_RED == signalHeads[SIG_W_PNTS_LOWER] && ASPECT_RED == signalHeads[SIG_W_PNTS_UPPER])
			occupancy |= OCC_VIRT_W_ADJOIN;
	} else {
		// West Control Point improperly lined, trip virtual occupancy
			occupancy |= OCC_VIRT_W_APPROACH | OCC_VIRT_W_ADJOIN;
	}	
}


int main(void)
{
	uint8_t changed = 0;
	uint8_t i;
	// Application initialization
	init();

	wdt_enable(WDTO_1S);

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

		// The EVENT_I2C_ERROR flag gets set if a read or write fails for some reason
		// I'm going to assume it's because the I2C bus went heywire, and we need to do
		// a very solid reset on things.  No I2C stuff will happen until this gets cleared
		
		if (events & EVENT_I2C_ERROR)
			xioInitialize();

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


			// Manual Control Logic
			// Note the lockouts based on time since last button
			if (0 == buttonLockout) 
			{
				uint8_t delta = (debounced_inputs[1] ^ old_debounced_inputs[1]) & (E_PNTS_BTTN_NORMAL | E_PNTS_BTTN_REVERSE | W_PNTS_BTTN_NORMAL | W_PNTS_BTTN_REVERSE);
				delta &= old_debounced_inputs[1];
			
				if (0 != delta)
				{
					buttonLockout = 5;
					// Any bit in delta that's a 1 now corresponds to a switch going low in this cycle
					if (delta & E_PNTS_BTTN_NORMAL)
						CodeCTCRoute(E_CONTROLPOINT, POINTS_NORMAL_FORCE, CLEARANCE_NONE);
					else if (delta & E_PNTS_BTTN_REVERSE)
						CodeCTCRoute(E_CONTROLPOINT, POINTS_REVERSE_FORCE, CLEARANCE_NONE);
				
					if (delta & W_PNTS_BTTN_NORMAL)
						CodeCTCRoute(W_CONTROLPOINT, POINTS_NORMAL_FORCE, CLEARANCE_NONE);
					else if (delta & W_PNTS_BTTN_REVERSE)
						CodeCTCRoute(W_CONTROLPOINT, POINTS_REVERSE_FORCE, CLEARANCE_NONE);
				}
			}
		
			// Get the physical occupancy inputs from debounced
			occupancy &= 0xF0;
			occupancy |= 0x0F & (debounced_inputs[1]>>4);
			turnouts &= ~(E_PNTS_STATUS | W_PNTS_STATUS);
			turnouts |= debounced_inputs[0] & (E_PNTS_STATUS | W_PNTS_STATUS);

			old_debounced_inputs[0] = debounced_inputs[0];
			old_debounced_inputs[1] = debounced_inputs[1];

			events &= ~(EVENT_READ_INPUTS);
		}			

		// Vital Logic
		vitalLogic();
	
		// Send output
		if (events & EVENT_WRITE_OUTPUTS)
		{
			SignalsToOutputs();
			xioOutputWrite();
			events &= ~(EVENT_WRITE_OUTPUTS);
		}

		// Test if something changed from the last time
		// around the loop - we need to send an update 
		//   packet if it did 
/*		if (0 != memcmp(signalHeads, old_signalHeads, sizeof(signalHeads))
			|| old_turnouts != turnouts
			|| old_clearance != clearance
			|| old_occupancy != occupancy)
		{
			// Something Changed - time to update
			for(i=0; i<sizeof(signalHeads); i++)
				signalHeads[i] = old_signalHeads[i];

			old_turnouts = turnouts;
			old_clearance = clearance;
			old_occupancy = occupancy;
			old_ext_occupancy = ext_occupancy;
			// Set changed such that a packet gets sent
			changed = 1;
		}
		else*/ if (decisecs >= update_decisecs)
			changed = 1;

		if (changed)
		{
			if (decisecs > update_decisecs)
				decisecs -= update_decisecs;
			else
				decisecs = 0;
		}
		/* If we need to send a packet and we're not already busy... */

/*
Byte
6 - East Frog Signals
 4:7 - Siding Signal
 0:3 - Main Signal
7 - East Points Signal
 4:7 - E Points Signal Upper
 0:3 - E Points Signal Lower
8 - West Frog Signals
 4:7 - Siding Signal
 0:3 - Main Signal
9 - West Points Signal
 4:7 - W Points Signal Upper
 0:3 - W Points Signal Lower

10 - Occupancy 1
 7 - E Adjoining
 6 - E Adjacent
 5 - W Adjoining
 4 - W Adjacent
 3 - Main
 2 - Siding
 1 - East OS Section
 0 - West OS Section

11: East OS Status
 7 - E OS Adjacent Virtual Occ
 6 - E OS Virtual Occ
 5 - E points lined Siding
 4 - E points lined Mainline
 3 - Undefined
 2 - E OS not cleared
 1 - E OS cleared Westbound
 0 - E OS cleared Eastbound
 
 */

#define MRB_STATUS_CP_CLEARED_EAST   0x01
#define MRB_STATUS_CP_CLEARED_WEST   0x02
#define MRB_STATUS_CP_CLEARED_NONE   0x04
#define MRB_STATUS_CP_SWITCH_NORMAL  0x10
#define MRB_STATUS_CP_SWITCH_REVERSE 0x20
#define MRB_STATUS_CP_VOCC_ADJOIN    0x40
#define MRB_STATUS_CP_VOCC_APPROACH  0x80

		if (changed && !(mrbus_state & (MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY)))
		{
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 13;			
			mrbus_tx_buffer[5] = 'S';

			mrbus_tx_buffer[6] = ((signalHeads[SIG_E_SIDING]<<4) & 0xF0) | (signalHeads[SIG_E_MAIN] & 0x0F);
			mrbus_tx_buffer[7] = ((signalHeads[SIG_E_PNTS_UPPER]<<4) & 0xF0) | (signalHeads[SIG_E_PNTS_LOWER] & 0x0F);
			mrbus_tx_buffer[8] = ((signalHeads[SIG_W_SIDING]<<4) & 0xF0) | (signalHeads[SIG_W_MAIN] & 0x0F);
			mrbus_tx_buffer[9] = ((signalHeads[SIG_W_PNTS_UPPER]<<4) & 0xF0) | (signalHeads[SIG_W_PNTS_LOWER] & 0x0F);

			mrbus_tx_buffer[10] = (occupancy & 0x0F) | (turnouts & 0xF0);
			
			switch(GetClearance(E_CONTROLPOINT))
			{
				case CLEARANCE_EAST:
					mrbus_tx_buffer[11] = MRB_STATUS_CP_CLEARED_EAST;
					break;
				case CLEARANCE_WEST:
					mrbus_tx_buffer[11] = MRB_STATUS_CP_CLEARED_WEST;
					break;
				case CLEARANCE_NONE:
				default:
					mrbus_tx_buffer[11] = MRB_STATUS_CP_CLEARED_NONE;
					break;
			}

			if (turnouts & E_PNTS_STATUS)  // Low is normal, high is reverse
				mrbus_tx_buffer[11] |= MRB_STATUS_CP_SWITCH_REVERSE;
			else
				mrbus_tx_buffer[11] |= MRB_STATUS_CP_SWITCH_NORMAL;
				
			if (occupancy & OCC_VIRT_E_ADJOIN)
				mrbus_tx_buffer[11] |= MRB_STATUS_CP_VOCC_ADJOIN;
			if (occupancy & OCC_VIRT_E_APPROACH)
				mrbus_tx_buffer[11] |= MRB_STATUS_CP_VOCC_APPROACH;

			switch(GetClearance(W_CONTROLPOINT))
			{
				case CLEARANCE_EAST:
					mrbus_tx_buffer[12] = MRB_STATUS_CP_CLEARED_EAST;
					break;
				case CLEARANCE_WEST:
					mrbus_tx_buffer[12] = MRB_STATUS_CP_CLEARED_WEST;
					break;
				case CLEARANCE_NONE:
				default:
					mrbus_tx_buffer[12] = MRB_STATUS_CP_CLEARED_NONE;
					break;
			}

			if (turnouts & W_PNTS_STATUS)  // Low is normal, high is reverse
				mrbus_tx_buffer[12] |= MRB_STATUS_CP_SWITCH_REVERSE;
			else
				mrbus_tx_buffer[12] |= MRB_STATUS_CP_SWITCH_NORMAL;

			if (occupancy & OCC_VIRT_W_ADJOIN)
				mrbus_tx_buffer[12] |= MRB_STATUS_CP_VOCC_ADJOIN;
			if (occupancy & OCC_VIRT_W_APPROACH)
				mrbus_tx_buffer[12] |= MRB_STATUS_CP_VOCC_APPROACH;

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
				CodeCTCRoute(E_CONTROLPOINT, mrbus_rx_buffer[6], PktDirToClearance(mrbus_rx_buffer[7]));
			else
				CodeCTCRoute(mrbus_rx_buffer[6], mrbus_rx_buffer[7], PktDirToClearance(mrbus_rx_buffer[8]));
			goto PktIgnore;

		case 'B':
			// CTC Command
			// B is the old school variant of C for the western control point
			CodeCTCRoute(W_CONTROLPOINT, mrbus_rx_buffer[6], PktDirToClearance(mrbus_rx_buffer[7]));
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
	for (i=0; i<8; i++)
	{
		uint8_t byte, bitset=0;
		
		if (mrbus_rx_buffer[MRBUS_PKT_SRC] != eeprom_read_byte((uint8_t*)(i+EE_E_APRCH_ADDR)))
			continue;

		if (mrbus_rx_buffer[MRBUS_PKT_TYPE] != eeprom_read_byte((uint8_t*)(i+EE_E_APRCH_PKT)))
			continue;
		
		byte = eeprom_read_byte((uint8_t*)(i+EE_E_APRCH_SUBTYPE));
		if ((0xFF != byte) && (mrbus_rx_buffer[MRBUS_PKT_SUBTYPE] != byte))
			continue;

		/* BITBYTE is computed as follows:
			x = bit = 0-7
			y = byte = byte in data stream (6 is first data byte)
			xxxyyyy
		*/
		byte = eeprom_read_byte((uint8_t*)(i+EE_E_APRCH_BITBYTE));
		bitset = mrbus_rx_buffer[(byte & 0x1F)] & (1<<((byte>>5) & 0x07));

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
				
			case EE_E_ADJ_ADDR:
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
				
			case EE_W_ADJ_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_W_ADJOIN;
				else
					ext_occupancy &= ~(XOCC_W_ADJOIN);
				break;

			case EE_W_TUMBLE_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_W_TUMBLE;
				else
					ext_occupancy &= ~(XOCC_W_TUMBLE);
				break;

			case EE_E_TUMBLE_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_E_TUMBLE;
				else
					ext_occupancy &= ~(XOCC_E_TUMBLE);
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


