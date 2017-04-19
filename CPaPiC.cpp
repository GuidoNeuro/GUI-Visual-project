
//////////////////////////////////////////////////
//
// CPaPiC.cpp: implementation of the CPaPiC class.
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

#include "stdafx.h"
#include "PaPiC.h"
#include "CPaPiC.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

short _stdcall Inp32(short portaddr);
void _stdcall Out32(short portaddr, short datum);

static const int data_reg = 0x0;
static const int status_reg = 0x1;
static const int control_reg = 0x2;

// The state of some pins is inverted relative to the state of their
// corresponding register bits.  These masks correct the bits that
// need to be corrected.

static const lp_pin positive_mask_control = 0xB; // 1011
static const lp_pin positive_mask_status = 0x10; // 10000

static const lp_pin negative_mask_control = positive_mask_control ^ 0x1FFFF; // 11111111111110100

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CPaPiC::CPaPiC()
{
	int control = 0;
	pin_init_user(LPT1);
	pin_input_mode(LP_INPUT_PINS);
	pin_output_mode(LP_OUTPUT_PINS);
	Out32(control, Inp32(control) & 0xDF);
}

CPaPiC::~CPaPiC()
{

}

//////////////////////////////////////////////////////////////////////
// Other Member Functions
//////////////////////////////////////////////////////////////////////


// set port address
void CPaPiC::pin_init_user(lp_pin lp_base){

	port_data = lp_base + data_reg;
	port_control = lp_base + control_reg;
	port_status = lp_base + status_reg;

	Out32(port_control, Inp32(port_control) & 0xDF);

	lp_input_pins = 0;
	lp_output_pins = 0;

}

// set output pins high
void CPaPiC::set_pin(lp_pin pins){

	int new_reg;

	// make sure the user is only trying to set an output pin
	pins &= lp_output_pins;

	new_reg = Inp32( port_data );
 
	new_reg |= pins & 0xFF;
	
	Out32(port_data, new_reg );   

	new_reg = Inp32( port_control );

	pins = (pins >> 8) & 0xF;

	new_reg |= pins;

	new_reg &= (pins & positive_mask_control) ^ 0x1FFFF;

	Out32(port_control, new_reg);  
}

// set output pins low
void CPaPiC::clear_pin(lp_pin pins){
	
	int new_reg;

	// make sure the user is only trying to set an output pin 
	pins &= lp_output_pins;

	new_reg = Inp32( port_data );
 
	new_reg = new_reg & ( ( pins & 0xFF ) ^ 0xFF);

	Out32(port_data, new_reg );

	new_reg = Inp32( port_control );

	pins = (pins >> 8) & 0xF;

	new_reg |= pins & positive_mask_control;

	new_reg &= (pins & negative_mask_control) ^ 0x1FFFF;

	Out32(port_control, new_reg);  

}

// set output pins high or low
void CPaPiC::change_pin(lp_pin pins, int mode){

	// make sure the user is only trying to set an output pin 
	pins &= lp_output_pins;

	 if (mode == LP_SET)
		 set_pin(pins);
	 else if (mode == LP_CLEAR)
		clear_pin(pins);

}

// invert output pins
void CPaPiC::invert_pin(lp_pin pins){

	// make sure the user is only trying to set an output pin
	pins &= lp_output_pins;

	int new_reg;

	new_reg = Inp32( port_data );
 
	new_reg ^= pins & 0xFF;

	Out32(port_data, new_reg );   

	new_reg = Inp32( port_control );

	new_reg ^= ((pins >> 8) & 0xF);

	Out32(port_control, new_reg);  
}

// set output pattern: given pins will be high, the others low
void CPaPiC::pin_pattern(lp_pin pins){

	// make sure the user is only trying to set an output pin
	pins &= lp_output_pins;

	int new_reg;

	new_reg = Inp32( port_data );
 
	new_reg = pins & 0xFF;

	Out32(port_data, new_reg );   

	new_reg = Inp32( port_control );

	new_reg = (new_reg & 0xF0) | ((pins >> 8) & 0xF) ^ positive_mask_control;

	Out32(port_control, new_reg);  
}


// check input pins
lp_pin CPaPiC::pin_is_set(lp_pin pins){

	int  status_reg, control_reg, data_reg;

	data_reg = Inp32( port_data );

	control_reg = Inp32( port_control );

	status_reg = Inp32( port_status );
	
	status_reg = (status_reg >> 3) & (pins >> 12) & 0x1F;

	return ((status_reg^positive_mask_status) << 12) | 
			(data_reg ^ 0xFF) | (((control_reg ^ negative_mask_control) & 0xF) << 8);

}

// function to change pins to input mode
void CPaPiC::pin_input_mode(lp_pin pins)
{
	lp_input_pins = pins & LP_INPUT_PINS; 
}

// function to change pins to output mode 
void CPaPiC::pin_output_mode(lp_pin pins)
{
	lp_output_pins = pins & LP_OUTPUT_PINS; 
}

// another interface to pin_input_mode and pin_output_mode 
void CPaPiC::pin_mode(lp_pin pins, int mode)
{
  if (mode == LP_INPUT)
    pin_input_mode(pins);
  if (mode == LP_OUTPUT)
    pin_output_mode(pins);
}

//

//
// CPaPiC.cpp: implementation of the CPaPiC class.
//
// Copyright (C) 2007 Zsolt Pöcze 
//
///////////////////////////////////////////////////
