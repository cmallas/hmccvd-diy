// ****************************************************************************
//
//	hmccvd-diy
//
//	v0.00
//
//	Open Source Firmware-Clone/DIY-Firmware
//	for HomeMatic Radio Controlled Valve Drive HM-CC-VD
//
//	enables direct control of the valve position via LAN Config Tool
//	in order to run custom room temperature control algorithm on a server
//
//	(c) Christian Mallas <christianmallas@gmx.de>
//
//	Licensed under GNU GPL
//
// ****************************************************************************
//	
//	I/O pin map:
//	
//	PA0 = COM0 (LCD)
//	PA1 = COM1 (LCD)
//	PA2 = COM2 (LCD)
//	PA3 = COM3 (LCD)
//	PA4 = SEG0 (LCD)
//	PA5 = SEG1 (LCD)
//	PA6 = SEG2 (LCD)
//	PA7 = SEG3 (LCD)
//	
//	PB0 = SS# (SPI-TRX)
//	PB1 = SCK (SPI-TRX)
//	PB2 = MOSI (SPI-TRX)
//	PB3 = MISO (SPI-TRX)
//	PB4 = PCINT12 (Pushbutton)
//	PB5 = G1H (T2 of H bridge)
//	PB6 = G2H (T5 of H bridge)
//	
//	PD0 = unused? (SPI-TRX)
//	PD1 = INT0 (SPI-TRX)
//	
//	PE6 = GP Input (photo transistor)
//	PE7 = GP output (IR diode)
//	
//	PF0 = ADC0 (batt voltage measurement)
//	PF1 = GP Output (batt voltage measurement enable#/disable)
//	PF6 = G1L (T3 of H bridge)
//	PF7 = G2L (T4 of H bridge)
//	
//	PG2 = SEG4 (LCD)
//	PG5 = RESET#
//	
//	remaining I/O pins unused/unconnected
//	
// ****************************************************************************
//	
//	LCD mapping:
//	
//	SEG0 on COM0: 1a (segment a of left 7-seg figure)
//	SEG1 on COM0: 1b
//	SEG2 on COM0: 2a (segment a of right 7-seg figure)
//	SEG3 on COM0: 2b
//	SEG4 on COM0: -
//	
//	SEG0 on COM1: 1f
//	SEG1 on COM1: 1g
//	SEG2 on COM1: 2f
//	SEG3 on COM1: 2g
//	SEG4 on COM1: airwaves symbol
//	
//	SEG0 on COM2: 1e
//	SEG1 on COM2: 1c
//	SEG2 on COM2: 2e
//	SEG3 on COM2: 2c
//	SEG4 on COM2: percent symbol
//	
//	SEG0 on COM3: 1d
//	SEG1 on COM3: -
//	SEG2 on COM3: 2d
//	SEG3 on COM3: -
//	SEG4 on COM3: battery symbol
//	
// ****************************************************************************
//
//	Revision history
//
//	version 0.00:
//	- very beginning: safe DDRn and PORTn config!
//	- drive LC Display as debugging output (and later on valve pos display)
//	- switch LCD clk src to 32kHz xtal
//	- use pushbutton
// 
//	to do:
//	- motor drive, speed detection, end stop detection
//	- wireless comm
//	- behave like original f/w PLUS set valve pos by LAN config tool
//	- sleep mode for TRX
//	- sleep mode (Power-save) for uC/wake up by RTC
//	- battery voltage measurement
//
//	wishlist/further goodies:
//	- establish encrypted wireless comm
//	- save key into EEPROM
//	- OSCCAL = 0x00; (saves even a little bit more power)
//	- reduce LCD drive time to reduce power consumption
//	- UART interface for debugging/wardriving purpose
//	- draw schematic and add to f/w folder
//	- ...?
//
// ****************************************************************************


// ##### libraries ############################################################

#include <avr/io.h>
#include <avr/sleep.h>
#ifndef F_CPU
#define F_CPU 1000000
#endif
#include <util/delay.h>
#include <avr/interrupt.h>


// ##### preprocessor definitions #############################################

#define boolean unsigned char
#define TRUE 0xFF
#define FALSE 0x00

#define START_32KHZ_TOSC ASSR = 0b00001000
#define ACTIVATE_BUTTON {PCMSK1 = 0b00010000; EIMSK |= (1 << 7);}

#define LCD_1MHZ {LCDCRA &= ~(1 << 7); LCDCRB &= ~(1 << 7); LCDFRR = 0b01100000; LCDCRA |= 1 << 7;}
#define LCD_32KHZ {LCDCRA &= ~(1 << 7); LCDCRB |= 1 << 7; LCDFRR = 0b00010000; LCDCRA |= 1 << 7;}

#define MOTOR_FW {}
#define MOTOR_BACK {}
#define MOTOR_OFF {}


// ##### typedefs #############################################################

typedef unsigned char byte;


// ##### global variables #####################################################

//	LCDDR0  = - | - | - |    -     | 2b | 2a | 1b | 1a 
//	LCDDR5  = - | - | - | airwaves | 2g | 2f | 1g | 1f
//	LCDDR10 = - | - | - | percent  | 2c | 2e | 1c | 1e
//	LCDDR15 = - | - | - | battery  | -  | 2d | -  | 1d
//
//	- - - - 2b 2a 1b 1a | - - - air 2g 2f 1g 1f | - - - perct 2c 2e 1c 1e | - - - bat - 2d - 1d
//
//								     LCDDR0  |   LCDDR5  |  LCDDR10  |  LCDDR15
const byte lcd_figure1[18][4] = {{0b00000011, 0b00000001, 0b00000011, 0b00000001}, // '0'
								 {0b00000010, 0b00000000, 0b00000010, 0b00000000}, // '1'
								 {0b00000011, 0b00000010, 0b00000001, 0b00000001}, // '2'
								 {0b00000011, 0b00000010, 0b00000010, 0b00000001}, // '3'
								 {0b00000010, 0b00000011, 0b00000010, 0b00000000}, // '4'
								 {0b00000001, 0b00000011, 0b00000010, 0b00000001}, // '5'
								 {0b00000001, 0b00000011, 0b00000011, 0b00000001}, // '6'
								 {0b00000011, 0b00000000, 0b00000010, 0b00000000}, // '7'
								 {0b00000011, 0b00000011, 0b00000011, 0b00000001}, // '8'
								 {0b00000011, 0b00000011, 0b00000010, 0b00000001}, // '9'
								 {0b00000011, 0b00000011, 0b00000011, 0b00000000}, // 'A'
								 {0b00000000, 0b00000011, 0b00000011, 0b00000001}, // 'b'
								 {0b00000001, 0b00000001, 0b00000001, 0b00000001}, // 'C'
								 {0b00000010, 0b00000010, 0b00000011, 0b00000001}, // 'd'
								 {0b00000001, 0b00000011, 0b00000001, 0b00000001}, // 'E'
								 {0b00000001, 0b00000011, 0b00000001, 0b00000000}, // 'F'
								 {0b00000010, 0b00000011, 0b00000011, 0b00000000}, // 'H'
								 {0b00000000, 0b00000000, 0b00000000, 0b00000000}}; // ' '

//								     LCDDR0  |   LCDDR5  |  LCDDR10  |  LCDDR15
const byte lcd_figure2[18][4] = {{0b00001100, 0b00000100, 0b00001100, 0b00000100}, // '0'
								 {0b00001000, 0b00000000, 0b00001000, 0b00000000}, // '1'
								 {0b00001100, 0b00001000, 0b00000100, 0b00000100}, // '2'
								 {0b00001100, 0b00001000, 0b00001000, 0b00000100}, // '3'
								 {0b00001000, 0b00001100, 0b00001000, 0b00000000}, // '4'
								 {0b00000100, 0b00001100, 0b00001000, 0b00000100}, // '5'
								 {0b00000100, 0b00001100, 0b00001100, 0b00000100}, // '6'
								 {0b00001100, 0b00000000, 0b00001000, 0b00000000}, // '7'
								 {0b00001100, 0b00001100, 0b00001100, 0b00000100}, // '8'
								 {0b00001100, 0b00001100, 0b00001000, 0b00000100}, // '9'
								 {0b00001100, 0b00001100, 0b00001100, 0b00000000}, // 'A'
								 {0b00000000, 0b00001100, 0b00001100, 0b00000100}, // 'b'
								 {0b00000100, 0b00000100, 0b00000100, 0b00000100}, // 'C'
								 {0b00001000, 0b00001000, 0b00001100, 0b00000100}, // 'd'
								 {0b00000100, 0b00001100, 0b00000100, 0b00000100}, // 'E'
								 {0b00000100, 0b00001100, 0b00000100, 0b00000000}, // 'F'
								 {0b00000000, 0b00000100, 0b00000100, 0b00000000}, // 'I'
								 {0b00000000, 0b00000000, 0b00000000, 0b00000000}}; // ' '


const byte lcd_percent[4] 	= {0x00, 0x00, 		 0b00010000, 0x00};
const byte lcd_airwaves[4] 	= {0x00, 0b00010000, 0x00, 		 0x00};
const byte lcd_battery[4] 	= {0x00, 0x00, 		 0x00, 		 0b00010000};


// ##### prototypes ###########################################################

void init_io();
void init_lcd();
//void init_tmr();
//void init_btn();
//void init_trx();

void lcd_put(byte fig1, byte fig2, boolean percent, boolean airwaves, boolean battery);

// ##### interrupt service routines ###########################################

ISR(PCINT1_vect) // pushbutton
{
	// software debouncing
	_delay_ms(30);
	if((PINB & (1 << 4)) == 0)
	{
		// pushbutton demo
		lcd_put(16, 16, FALSE, FALSE, FALSE); // say "HI"
		_delay_ms(750);
	}
}


// ##### int main() ###########################################################

int main()
{
	init_io();
	init_lcd();
	lcd_put(16, 16, FALSE, FALSE, FALSE); // say "HI"
	START_32KHZ_TOSC; // start 32kHz osc
	ACTIVATE_BUTTON;
	sei(); // global interrupt enable
	_delay_ms(3000);
	LCD_32KHZ; // switch LCD clock src to 32kHz osc

	while(1)
	{
		// LCD demo
		int i=0;
		for(i=0; i<18; i++)
		{
			boolean temp_batt=FALSE;
			boolean temp_air=FALSE;
			boolean temp_percent=FALSE;
			
			if(i & 0x01) temp_batt=TRUE;
			if(i & 0x02) temp_air=TRUE;
			if(i & 0x04) temp_percent=TRUE;

			lcd_put(i,i,temp_percent, temp_air, temp_batt);
			_delay_ms(1000);
		}
	}
}


// ##### other functions ######################################################

void init_io()
{	
	PORTB = 0b11111111; // drive H bridge to state 00 (ST3 = LOW, ST4 = LOW)
						// by driving 	G1H = HIGH (T2 inhibiting)
						//				G1L = HIGH (T3 conducting)
						//				G2H = HIGH (T5 inhibiting)
						//				G2L = HIGH (T4 conducting)
	DDRB = 0b01100111;
	PORTF = 0b11111110; 
	DDRF = 0b11000010;

	PORTA = 0b00000000;
	DDRA = 0b11111111; // pins connected to LCD driven LOW
	PORTC = 0b11111111;
	DDRC = 0b00000000;
	PORTD = 0b11111111;
	DDRD = 0b00000000;
	PORTE = 0b11111111;
	DDRE = 0b10000000;
	PORTG = 0b11111011;
	DDRG = 0b00000100;
}

void init_lcd()
{	
	// config for 1MHz internal RC osc.
	LCDCRB = 0b00110000; // system clock; 1/3 Bias; 1/4 Duty; <=13 SEG per COM
	LCDFRR = 0b01100000; // Prescaler clk_LCD/2048; divide by 1
	LCDCCR = 0b00011111; // max drive time; 3.35V contrast voltage
	LCDCRA = 0b11000000; // Enable; Low Power Waveform
}

void lcd_put(byte fig1, byte fig2, boolean percent, boolean airwaves, boolean battery)
{
	LCDDR0 = lcd_figure1[fig1][0] | lcd_figure2[fig2][0] | (lcd_percent[0] & percent) | (lcd_airwaves[0] & airwaves) | (lcd_battery[0] & battery);
	LCDDR5 = lcd_figure1[fig1][1] | lcd_figure2[fig2][1] | (lcd_percent[1] & percent) | (lcd_airwaves[1] & airwaves) | (lcd_battery[1] & battery);
	LCDDR10 = lcd_figure1[fig1][2] | lcd_figure2[fig2][2] | (lcd_percent[2] & percent) | (lcd_airwaves[2] & airwaves) | (lcd_battery[2] & battery);
	LCDDR15 = lcd_figure1[fig1][3] | lcd_figure2[fig2][3] | (lcd_percent[3] & percent) | (lcd_airwaves[3] & airwaves) | (lcd_battery[3] & battery);
}
