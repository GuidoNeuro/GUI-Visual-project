/////////////////////////////////////////////
//
// CPaPiC.h: interface for Class PaPiC.
//
// Copyright (C) 2007 Zsolt Pöcze 
//

/*/////////////////////////////////////////////////////////////////////////////////

	This file is a part of Class PaPiC : Parallel Port Pin Control Library

	Version 1.0

	Written by Zsolt Pöcze
	
	Contact: pocze_zsolt@hotmail.com

	This code may be used in compiled form in any way you desire. This
	file may be redistributed unmodified by any means PROVIDING it is 
	not sold for profit without the authors written consent, and 
	providing that this notice and the authors name is included. If 
	the source code in this file is used in any commercial application 
	then a simple email would be nice.

	This file is provided "as is" with no expressed or implied warranty.
	The author accepts no liability if it causes any damage to your
	computer, causes your pet to fall ill, increases baldness and so forth.
	
	Please use and enjoy. Please let me know of any bugs/mods/improvements 
	that you have found/implemented and I will fix/incorporate them into this
	file. 

	Thanks

		Jeremy Elson and Al Hooton

	the PARAPIN package has been developed by.

//////////////////////////////////////////////////////////////////////////////////*/

#if !defined(C_PAPIC_)
#define C_PAPIC_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

typedef int lp_pin;

#define LPT1 0x3010 // for pin_init_user
#define LPT2 0x278 // for pin_init_user

#define LP_INPUT	0 // for pin_mode 
#define LP_OUTPUT	1 // for pin_mode

// This definitions map parallel port pin numbers to register bit numbers.
// - shifts of 0..7   represent bits 0..7 of port address + 0 (data register)
// - shifts of 8..11 represent bits 0..3 of port address + 2 (control register)
// - shifts of 12..16  represent bits 0..4 of port address + 1 (status register)

// pin 18 - pin 25 are not contollable
const lp_pin LP_PIN01 = 0x100;
const lp_pin LP_PIN02 = 0x1;
const lp_pin LP_PIN03 = 0x2;
const lp_pin LP_PIN04 = 0x4;
const lp_pin LP_PIN05 = 0x8;
const lp_pin LP_PIN06 = 0x10;
const lp_pin LP_PIN07 = 0x20;
const lp_pin LP_PIN08 = 0x40;
const lp_pin LP_PIN09 = 0x80;
const lp_pin LP_PIN10 = 0x8000;
const lp_pin LP_PIN11 = 0x10000;
const lp_pin LP_PIN12 = 0x4000;
const lp_pin LP_PIN13 = 0x2000;
const lp_pin LP_PIN14 = 0x200;
const lp_pin LP_PIN15 = 0x1000;
const lp_pin LP_PIN16 = 0x400;
const lp_pin LP_PIN17 = 0x800;


// pins that can act as input 
#define LP_INPUT_PINS (LP_PIN10 | LP_PIN11 | LP_PIN12 | LP_PIN13 | LP_PIN15 )

// pins that can act as output 
#define LP_OUTPUT_PINS ( LP_PIN01 | LP_PIN02 | LP_PIN03 | LP_PIN04 | LP_PIN05 | LP_PIN06 | \
		LP_PIN07 | LP_PIN08 | LP_PIN09 | LP_PIN14 | LP_PIN16 | LP_PIN17 )

#define LP_CLEAR	0 // for change_pin 
#define LP_SET		1 // for change_pin 

// Pin-to-register assignments in array form, so that they can be
// accessed using LP_PIN[pin-number]

static const int LP_PIN[] = {
  0,        // "pin 0" - not controllable 
  LP_PIN01,
  LP_PIN02,
  LP_PIN03,
  LP_PIN04,
  LP_PIN05,
  LP_PIN06,
  LP_PIN07,
  LP_PIN08,
  LP_PIN09,
  LP_PIN10,
  LP_PIN11,
  LP_PIN12,
  LP_PIN13,
  LP_PIN14,
  LP_PIN15,
  LP_PIN16,
  LP_PIN17
};

class CPaPiC  
{

protected:

	// LPT port register addresses
	int port_data;
	int port_control;
	int port_status;

	// masks of pins that we currently allow input and output on
	lp_pin lp_input_pins;
	lp_pin lp_output_pins;

public:
	CPaPiC();
	virtual ~CPaPiC();

	void pin_init_user(lp_pin lp_base);

	void pin_input_mode(lp_pin pins);

	void pin_output_mode(lp_pin pins); 

	void pin_mode(lp_pin pins, int mode);

	void set_pin(lp_pin pins);

	void clear_pin(lp_pin pins);

	void change_pin(lp_pin pins, int mode);

	void invert_pin(lp_pin pins);

	void pin_pattern(lp_pin pins);

	lp_pin pin_is_set(lp_pin pins);

};

#endif // !defined(C_PAPIC_)

//
// CPaPiC.h: interface for Class PaPiC.
//
// Copyright (C) 2007 Zsolt Pöcze 
//
////////////////////////////////////////