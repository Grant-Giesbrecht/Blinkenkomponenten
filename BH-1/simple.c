/*******************************************************************************
 Firmware for the BlinkenHerz-1 clock module

 Assumes 20-bit counter

 Created by Grant M Giesbrecht
 14-3-2021
 -------------------------------------------------------------------------------
 Changes from v1:
 	* Now preset mode is on or off, not on1, on2, or off. Freq dial scrolls btwn
	  five options.
	* Added lock
	* Added 50DC circuit bypass
	* Added option headers
	* Added freq range indicators
	* Added frequency display
	* Removed 2nd error indicator
	* Addded clear error signal?
	* Improved hysteresis algorithm?
	* Cleaned up comments
	* Various bug fixes

Control descriptions:
	NOTE: Frequency lock control and force-no-hysteresis option are conflicting,
	lock control has higher priority.
*******************************************************************************/

// NOTE: Look for comments flagged with 'ATTENTION' to find lines that need to be fixed

#include <avr/io.h>
#include <math.h>
#define F_CPU 1e6
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "display.h"
#include <util/delay.h>

#define BOOL int
#define TRUE 1
#define FALSE 0

//******************************************************************************
//				ARDUINO TIMER DEFINITIONS


// #define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
// #define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
//
// // the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// // the overflow handler is called every 256 ticks.
// #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
//
// // the whole number of milliseconds per timer0 overflow
// #define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
//
// // the fractional number of milliseconds per timer0 overflow. we shift right
// // by three to fit these numbers into a byte. (for the clock speeds we care
// // about - 8 and 16 MHz - this doesn't lose precision.)
// #define FRACT_INC ((int(MICROSECONDS_PER_TIMER0_OVERFLOW) % 1000) >> 3)
// #define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

// #if defined(TIM0_OVF_vect)
// ISR(TIM0_OVF_vect)
// #else
// ISR(TIMER0_OVF_vect)
// #endif
// {
// 	// copy these to local variables so they can be stored in registers
// 	// (volatile variables must be read from memory on every access)
// 	unsigned long m = timer0_millis;
// 	unsigned char f = timer0_fract;
//
// 	m += MILLIS_INC;
// 	f += FRACT_INC;
// 	if (f >= FRACT_MAX) {
// 		f -= FRACT_MAX;
// 		m += 1;
// 	}
//
// 	timer0_fract = f;
// 	timer0_millis = m;
// 	timer0_overflow_count++;
// }
//
// unsigned long millis()
// {
// 	unsigned long m;
// 	uint8_t oldSREG = SREG;
//
// 	// disable interrupts while we read timer0_millis or we might get an
// 	// inconsistent value (e.g. in the middle of a write to timer0_millis)
// 	cli();
// 	m = timer0_millis;
// 	SREG = oldSREG;
//
// 	return m;
// }

//
//******************************************************************************


//***************************** PIN DEFINITIONS ******************************//

#define PIN_MANUAL_CTRL PB0
#define PIN_XN_CTRL PB1
#define PIN_DATA PB2			//For both Div-N shift registers and optional display shift register
#define PIN_LOCK_IND PB3
#define PIN_KHZ_IND PB4
#define PIN_MHZ_IND PB5
#define PIN_UPLOADING_N PB6
#define PIN_DC_CORRECTOR PB7

#define PIN_DIAL_CTRL PC0
#define PIN_PRESET_CTRL PC1
#define PIN_DIVN_BYPASS PC2
#define PIN_OSC_SELECT PC3

#define PIN_FORCE_50DC PD0 //Header option - forces BH-1 to use duty cycle corrector
#define PIN_NOHYST PD1 //Header option - disables hysteresis and continuously updates freq-divider value
#define PIN_SLOWDISP PD2 //Header option - forces display to update slowly
#define PIN_DISP_DCLK PD3
#define PIN_LOCK_CTRL PD4
#define PIN_DCLK PD5
#define PIN_RCLK PD6			//For both Div-N shift registers and optional display shift register
#define PIN_ERROR PD7

//*************************** HARDWARE PARAMETERS ****************************//

#define LO_FREQ 4e6 //Speed of local oscillator
#define LO2_FREQ 7e9 //Speed of secondary local oscillator
#define DIVN_MAX_RATIO 1.04858e6 //Max division ratio of divide by N

#define DISP_BUFF_SIZE 3

//************************** CONFIGURATION OPTIONS ***************************//

#define PERMITTED_MAX_F 4e6 //Maximum frequency BH allowed to generate (must be equal or less than LO_FREQ)
#define PERMITTED_MIN_F -1 //Mimimum frequency BH allowed to generate (must be greater than or equal to LO_FREQ/2^20/2). If set to -1, minimum frequency is set to minumum allowed by hardware.

#define XN_MULTIPLIER 18 //Amount by which to increase clock speed when xN on and Continous mode
#define HYST_FACTOR .05 // %/100 by which exponent of freq can change without forcing an update
#define PERMITTED_MIN_DC 10 //Minimum duty cycle allowed in normal operation (%)
#define DISP_UPDATE_T 200 //Amount of time that must ellapse before the readbout is updated (mS)

#define PRESET1_ALPHA 6
#define PRESET1_BETA 1e3
#define PRESET1_GAMMA 100e3
#define PRESET1_DELTA 1e6
#define PRESET1_EPSILON 2e6

//************************ FUNCTION DEFINITIONS ******************************//

void readControls(int* manual_mode, int* xN__mode, float* dial_value, int* preset_mode, int* lock_f, int* opt_force_50dc, int* opt_nohyst, int* opt_slowdisp);
float getDivRatio(int manual_mode, int xN__mode, int dial_value);
void updateCounterIfChanges(float* divRatio, float* divRatioLast);

void InterpretControls(int manual_mode, int xN__mode, float dial_value, int preset_mode, float& fLast, int lock_f, int opt_force_50dc, int opt_nohyst, int opt_slowdisp);
void updateDivN(long int N);
void binArray( unsigned long int n ,int* array, unsigned int numBits);
uint16_t readADC();
float dialToFrequency(float dial_value);
void displayFrequency(float f, uint8_t* displayBuffer);
void shiftDataOut(uint8_t* displayBuffer, long int buf_size, int reverse=FALSE);

typedef struct{
	long int N; //Divisor start/ program value
	float f; //output freq
	float f_req; //Requested freq
	float delta; //Error btw reqursted and provided freq
	int en50DC; //Enable 50% DC circuit
	int osc_idx; //Which oscillator to use
	float dc; //Duty cycle
	int bypass_divn; //Bypass Div-N
} synth_freq;

void getClosestFrequency(synth_freq* sf, int opt_force_50dc);
float DivisorToFrequency(long int N, float f_osc, int enable_50DC);
long int FrequencyToDivisor(float f, float f_osc, int enable_50DC);
void calcDutyCycle(synth_freq* sf);

synth_freq main_sf;

/*******************************************************************************
/																			   /
*******************************************************************************/
int main(){

	//Declare Variables
	int manual_mode = FALSE;
	int xN__mode = FALSE;
	float dial_value = 0;
	int preset_mode = 0; //Preset modes: 0: no preset, 1: preset-1 2: preset-2, 3: preset-3
	int lock_f = FALSE;
	int opt_force_50dc;
	int opt_nohyst;
	int opt_slowdisp;
	uint8_t displayBuffer[20];


	float fLast = -1;


	//******** SET DATA DIRECTION BITS

	//Register B
	DDRB &= ~(1 << PIN_MANUAL_CTRL); //B0, Input
	DDRB &= ~(1 << PIN_XN_CTRL); //B1, Input
	DDRB |= 1 << PIN_DATA; //B2, Output
	DDRB |= 1 << PIN_LOCK_IND; //B3, Output
	DDRB |= 1 << PIN_KHZ_IND; //B4, Output
	DDRB |= 1 << PIN_MHZ_IND; //B5, Output
	DDRB |= 1 << PIN_UPLOADING_N; //B6, Output
	DDRB |= 1 << PIN_DC_CORRECTOR; //B7, Output

	//Register C
	DDRC &= ~(1 << PIN_DIAL_CTRL); //PC0, Input
	DDRC &= ~(1 << PIN_PRESET_CTRL); //PC1, Input
	DDRC |= 1 << PIN_DIVN_BYPASS; //PC2, Output
	DDRC |= 1 << PIN_OSC_SELECT; //PC3, Output

	//Register D
	DDRD &= !(1 << PIN_FORCE_50DC); //Input
	DDRD &= !(1 << PIN_NOHYST); //Input
	DDRD &= !(1 << PIN_SLOWDISP); //Input
	DDRD |= 1 << PIN_DISP_DCLK; //Output
	DDRD |= 1 << PIN_DCLK; //, Output
	DDRD |= 1 << PIN_RCLK; //, Output
	DDRD |= 1 << PIN_ERROR; //, Output
	DDRD &= ~(1 << PIN_LOCK_CTRL); //, Output

	//******** SET OUTPUT STATES

	//Register B
	PORTB |= 1 << PIN_MANUAL_CTRL; //B0, Pullup resistor ON
	PORTB |= 1 << PIN_XN_CTRL; //B1, Pullup resistor ON
	PORTB &= ~(1 << PIN_DATA); //B2, Set LOW
	PORTB &= ~(1 << PIN_LOCK_IND); //B3, Set LOW
	PORTB &= ~(1 << PIN_KHZ_IND); //B4, Set LOW
	PORTB &= ~(1 << PIN_KHZ_IND); //B5, Set LOW
	PORTB &= ~(1 << PIN_UPLOADING_N); //B6, OFF
	PORTB |= 1 << PIN_DC_CORRECTOR; //B7, ON

	//Register C
	PORTC &= ~(1 << PIN_DIAL_CTRL); //C0, Pullup resistor OFF
	PORTC &= ~(1 << PIN_PRESET_CTRL); //C1, Pullup resistor OFF
	PORTC &= ~(1 << PIN_DIVN_BYPASS); //PC2, OFF (Use Div-N)
	PORTC &= ~(1 << PIN_OSC_SELECT); //PC3, OFF (Use Primary osc)
	// PORTC |= 1 << PC3;

	//Register D
	PORTD &= ~(1 << PIN_FORCE_50DC); //Pullup OFF
	PORTD &= ~(1 << PIN_NOHYST); //Pullup OFF
	PORTD &= ~(1 << PIN_SLOWDISP); //Pullup OFF
	PORTD &= ~(1 << PIN_DISP_DCLK); // Set low
	PORTD &= ~(1 << PIN_DCLK); //, Set LOW
	PORTD &= ~(1 << PIN_RCLK); //, Set LOW
	PORTD &= ~(1 << PIN_ERROR); //, Set LOW
	PORTD &= ~(1 << PIN_LOCK_CTRL); //, Set LOW

	//************** CONFIGURE ADC

	PRR &= ~(1 << PRADC); //Power On ADC (configured to off by default in power reduction register)
	// ADMUX &= ~((1 << REFS0) | (1 << REFS1)); //Set reference voltage to VCC
	ADMUX |= (1 << REFS0);
	// ADMUX &= ~(1 << PC0) //Set ADC channel to PC0. This is the default case so I've commented it out
	ADCSRA |= ((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2)); //Set ADC clock divider to 8 for ADC frequenct of 125 khz.
	ADCSRA |= (1 << ADEN); //Enable ADC

	// unsigned long last_update_time = millis();
	float f_displayed = -1;

	double count = 456;

	while (TRUE){

		// last_update_time = millis();
		count *= 10.0;		//Note: if you don't multiply by '10.0' and instead do '10' it will cause rounding errors!
		if (count > 10e6){
			count = 10;
		}

		// count = 12300;

		//Get updated values from the controls
		// readControls(&manual_mode, &xN__mode, &dial_value, &preset_mode, &lock_f, &opt_force_50dc, &opt_nohyst, &opt_slowdisp);

		//Update Divide-by-N Circuit if big enough change
		// InterpretControls(manual_mode, xN__mode, dial_value, preset_mode, fLast, lock_f, opt_force_50dc, opt_nohyst, opt_slowdisp);

		// if (millis() - last_update_time > DISP_UPDATE_T && f_displayed != main_sf.f){
		// 	f_displayed = main_sf.f;

		// displayBuffer[2] = 0b01100000;
		// displayBuffer[1] = 0b11011011;
		// displayBuffer[0] = 0b11110010;

		displayFrequency(count, displayBuffer);
		// }

		PORTB |=  (1 << PIN_LOCK_IND); //Uploading light ON

		_delay_ms(1e3);

		PORTB &=  ~(1 << PIN_LOCK_IND); //Uploading light ON

		_delay_ms(1e3);

	}

}


void displayFrequency(float f, uint8_t* displayBuffer){

	float range = 1;

	signed int dp_idx = -1;

	// Check which frequency units to use
	if (f >= 1e6){
		range = 1e6;
		PORTB &= ~(1 << PIN_KHZ_IND); // OFF
		PORTB |= 1 << PIN_MHZ_IND; // ON
	}else if (f >= 1e3){
		range = 1e3;
		PORTB &= ~(1 << PIN_MHZ_IND); // OFF
		PORTB |= 1 << PIN_KHZ_IND; // ON

	}else{
		PORTB &= ~(1 << PIN_KHZ_IND); // OFF
		PORTB &= ~(1 << PIN_MHZ_IND); // OFF
	}

	// Check how many decimal places fit
	if (f/range < 10){
		dp_idx = 2;
	}else if (f/range < 100){
		dp_idx = 1;
	}else{
		dp_idx = 0;
	}

	// Populate display buffer
	fto7a(displayBuffer, DISP_BUFF_SIZE, f/range, dp_idx);

	// At this point, whatever is in the display buffer wil be displayed. Index
	// zero is the left-most (highest value) digit.

	// displayBuffer[0] = 0b01100000;
	// displayBuffer[1] = 0b11011011;
	// displayBuffer[2] = 0b11110010;

	// At this point, the data is formatted s.t. higher value bits correspond to
	// earlier letters. so 0b10000000 illuminates 'A' only, and 0b00000001 only
	// illuminates the decimal point.

	// Send display data to shift registers on CIP-1
	shiftDataOut(displayBuffer, DISP_BUFF_SIZE, FALSE);

}

void shiftDataOut(uint8_t* displayBuffer, long int buf_size, int reverse){

	PORTD &= ~(1 << PIN_DISP_DCLK); //, Set LOW

	if (reverse == TRUE){

		//Scan over each byte (letter on display)
		for (long int i = 0; i < buf_size; i++){

			//Scan over each bit (segment on letter)
			for (long int b = 0; b < 8 ; b++){

				// Check if segment is set
				if (displayBuffer[i] & (1 << b)){ //Set
					PORTB |= (1 << PIN_DATA);
				}else{ //Unset
					PORTB &= ~(1 << PIN_DATA);
				}

				PORTD |= (1 << PIN_DISP_DCLK); //Clock high
				PORTD &= ~(1 << PIN_DISP_DCLK); // Clock low

			}

		}

	}else{

		//Scan over each byte (letter on display)
		for (long signed int i = buf_size-1; i >= 0; i--){

			//Scan over each bit (segment on letter)
			for (long int b = 0; b < 8 ; b++){

				// Check if segment is set
				if (displayBuffer[i] & (1 << b)){ //Set
					PORTB |= (1 << PIN_DATA);
				}else{ //Unset
					PORTB &= ~(1 << PIN_DATA);
				}

				PORTD |= (1 << PIN_DISP_DCLK); //Clock high
				PORTD &= ~(1 << PIN_DISP_DCLK); // Clock low

			}

		}

	}

	PORTD &= ~(1 << PIN_RCLK); // Clock low
	PORTD |= (1 << PIN_RCLK); //Clock high
	PORTD &= ~(1 << PIN_RCLK); // Clock low

}
