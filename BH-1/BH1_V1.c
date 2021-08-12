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

#define BOOL int
#define TRUE 1
#define FALSE 0

//******************************************************************************
//				ARDUINO TIMER DEFINITIONS


#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((int(MICROSECONDS_PER_TIMER0_OVERFLOW) % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

#if defined(TIM0_OVF_vect)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}

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
void displayFrequency(float f, char* buffer);
void shiftDataOut(char* buffer, long int buf_size, int reverse=FALSE);

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

	char displayBuffer[DISP_BUFF_SIZE];

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
	DDRC &= !(1 << PIN_FORCE_50DC); //Input
	DDRC &= !(1 << PIN_NOHYST); //Input
	DDRC &= !(1 << PIN_SLOWDISP); //Input
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

	unsigned long last_update_time = millis();
	float f_displayed = -1;

	double count = 100;

	while (TRUE){

		last_update_time = millis();

		main_sf.f = count * 10;

		//Get updated values from the controls
		// readControls(&manual_mode, &xN__mode, &dial_value, &preset_mode, &lock_f, &opt_force_50dc, &opt_nohyst, &opt_slowdisp);

		//Update Divide-by-N Circuit if big enough change
		// InterpretControls(manual_mode, xN__mode, dial_value, preset_mode, fLast, lock_f, opt_force_50dc, opt_nohyst, opt_slowdisp);

		// if (millis() - last_update_time > DISP_UPDATE_T && f_displayed != main_sf.f){
		// 	f_displayed = main_sf.f;
		// displayFrequency(main_sf.f, displayBuffer);
		// }

		PORTB |=  (1 << PIN_LOCK_IND); //Uploading light ON

		while ( millis() - last_update_time < 1e3){
			continue;
		}

		PORTB &=  ~(1 << PIN_LOCK_IND); //Uploading light ON

		while ( millis() - last_update_time < 2e3){
			continue;
		}

	}

}

/*******************************************************************************
Reads pin values and updates control variables from them.
*******************************************************************************/
void readControls(int* manual_mode, int* xN_mode, float* dial_value, int* preset_mode, int* lock_f, int* opt_force_50dc, int* opt_nohyst, int* opt_slowdisp){

	//******************** READ TOGGLE SWITCHES ******************************//

	//Read manual control
	if (PINB & (1 << PIN_MANUAL_CTRL)){
		*manual_mode = 1;
	}else{
		*manual_mode = 0;
	}

	//Read lock control, update lock light
	if (PIND & (1 << PIN_LOCK_CTRL)){
		*lock_f = 1;
		PORTB |=  (1 << PIN_LOCK_IND); //Uploading light ON
	}else{
		*lock_f = 0;
		PORTB &=  ~(1 << PIN_LOCK_IND); //Uploading light OFF
	}

	//Read xN_ mode control
	if (PINB & (1 << PIN_XN_CTRL)){
		*xN_mode = 1;
	}else{
		*xN_mode = 0;
	}

	//*********************** READ OPTION HEADERS ****************************//

	//Read force 50% DC option
	if (PIND & (1 << PIN_FORCE_50DC)){
		*opt_force_50dc = 1;
	}else{
		*opt_force_50dc = 0;
	}

	//Read force no hysteresis option
	if (PIND & (1 << PIN_NOHYST)){
		*opt_nohyst = 1;
	}else{
		*opt_nohyst = 0;
	}

	//Read force slow display update option
	if (PIND & (1 << PIN_SLOWDISP)){
		*opt_slowdisp = 1;
	}else{
		*opt_slowdisp = 0;
	}

	//************************* READ ANALOG CONTROLS *************************//

		// ************** READ FREQUENCY DIAL CONTROL ************* //

	//Read Frequency Dial Control
	//
	// thanks for the great tutorial:
	// https://mansfield-devine.com/speculatrix/2018/10/avr-basics-reading-analogue-input/
	//
	ADMUX &= 0b11110000; //Set ADC to read PC0 (Frequency Dial)
	uint16_t f_dial_adc_raw = readADC(); //Read value from ADC
	float f_dial_adc = (float)f_dial_adc_raw;

	*dial_value = f_dial_adc;
	// f_dial_adc = float(100);

	// f_dial_adc = 500;
	// *dial_value = f_dial_adc*800.0;
	// *dial_value = 400e3;

	// *dial_value = 1e3;



		// ****************** READ PRESETS CONTROL **************** //

	ADMUX &= 0b11110000; ADMUX |= (1 << MUX1); //Set ADC to read PC1 (Preset selector)
	uint16_t preset_selector_adc = readADC(); //Read value from ADC

	//Check preset setting
	if (preset_selector_adc < 341){
		*preset_mode = 0;
	}else if (preset_selector_adc < 682){
		*preset_mode = 1;
	}else{
		*preset_mode = 2;
	}

}

/*******************************************************************************

*******************************************************************************/
uint16_t readADC(){

	uint16_t val = 0;
	ADCSRA |= (1 << ADSC); // Write bit to 1 to start conversion.
	// It returns automatically to 0 when conversion is complete.
	while((ADCSRA & (1 << ADSC))) {
	    continue;
	}

	return ADCW;
	// Analogue value should now be in ADCL (low byte) and ADCH (high byte).
	// val = ADCL; // read low byte first
	// val += (ADCH << 8); // then read high byte
	// val = 1023 - val; //Flip val
	//
	// return val;
}





/*******************************************************************************
Takes the control states measured by 'readControls()' and interprets the state,
determining the desired frequency, and how to drive each subcircuit to achieve
the desired operation.

--------------------------------------------------------------------------------
Controls:
	- Manual/Auto Clock Toggle: Typically auto. Manual lets you cycle through
		clocks with push button. Auto, clock runs continuously.
	- xN_: Multiplies each clock trigger by 18 (however many mu-steps per phi).
		In auto mode, it increases the base clock speed (set by the dial) by this
		multiplier (18). In manual mode, each clock cycle button press triggers
		18 cycles (one full phi instead of a mu).
	- Dial: Sets the base clock speed of mu-clock.

*******************************************************************************/
void InterpretControls(int manual_mode, int xN_mode, float dial_value, int preset_mode, float& fLast, int lock_f, int opt_force_50dc, int opt_nohyst, int opt_slowdisp){

	float ratio;
	float freq_requested;

	float fLast_exp;
	float hystMin;
	float hystMax;

	preset_mode = 0;
	manual_mode = 0;

	if (preset_mode == 0){	//Preset mode is set to None/Live

		if (manual_mode){ //Manual/auto switch in MANUAL MODE

			//NOTE: In this mode, xN-control is ignored by the MCU. Only controls the trigger counter.

			//Calculate frequency
			freq_requested = dialToFrequency(dial_value);

		}else{ //Manual/auto switch in AUTO/CONTINOUS MODE

			//Calculate frequency
			freq_requested = dialToFrequency(dial_value);

			//If xN is on, multiply freqency by 'N'
			if (xN_mode) freq_requested *= float(XN_MULTIPLIER);

		}
	}else if(preset_mode == 1 || preset_mode == 2){ //Set to preset 1

		//In this mode, xN is ignored by the MCU. Only controls the trigger
		//counter.

		if (dial_value < 205){
			freq_requested = PRESET1_ALPHA;
		}else if(dial_value < 410){
			freq_requested = PRESET1_BETA;
		}else if(dial_value < 614){
			freq_requested = PRESET1_GAMMA;
		}else if(dial_value < 819){
			freq_requested = PRESET1_DELTA;
		}else{
			freq_requested = PRESET1_EPSILON;
		}

	}

	//Determine closest frequency to requested frequency that can be provided
	synth_freq local_sf;
	local_sf.f_req = freq_requested;
	getClosestFrequency(& local_sf, opt_force_50dc);

	//Calculate hysteresis thresholds
	if (fLast < 0){
		hystMax = -1;
		hystMin = -1;
	}else{
		fLast_exp = log10(fLast/float(XN_MULTIPLIER)); //Exponent (ie. linear with raw dial value)
		hystMin = pow(10, fLast_exp*(1-float(HYST_FACTOR)))*float(XN_MULTIPLIER); //Min freq
		hystMax = pow(10, fLast_exp*(1+float(HYST_FACTOR)))*float(XN_MULTIPLIER); //Max freq
	}

	//Update divide by N counter if past hysteresis bounds
	if (( opt_nohyst || (local_sf.f > hystMax || local_sf.f < hystMin)) && (lock_f == 0)){

		main_sf = local_sf;

		//Select locatl oscillator
		// if (main_sf.osc_idx == 1){
			PORTC |= (1 << PIN_OSC_SELECT); //PC3, ON (Use secondary osc)
			PORTC &= ~(1 << PIN_OSC_SELECT); //PC3, ON (Use secondary osc)
		// 	PORTC &= ~(1 << PIN_OSC_SELECT); //PC3, OFF (Use Primary osc)
		// }else{
			// PORTC &= ~(1 << PIN_OSC_SELECT); //PC3, OFF (Use Primary osc)
		// }

		//Select state of divider bypass MUX
		main_sf.bypass_divn = 0;
		if (main_sf.bypass_divn == 1){
			PORTC |= (1 << PIN_DIVN_BYPASS); //PC3, ON (Use oscillator directly)
		}else{
			PORTC &= ~(1 << PIN_DIVN_BYPASS); //PC3, OFF (Use Divider)
		}

		//Select state of duty cycle corrector MUX
		if (main_sf.en50DC == 1){
			DDRB |= 1 << PIN_DC_CORRECTOR; //B7, enable DC corrector
		}else{
			DDRB &= ~(1 << PIN_DC_CORRECTOR); //B7, disable DC Corrector
		}



		updateDivN(main_sf.N);

		// updateDivN(round(1048576 - 4e6/main_sf.f/2));
		fLast = freq_requested;
	}

}

/*******************************************************************************
Uploads the integer 'N' to the divide by N circuit
*******************************************************************************/
void updateDivN(long int N){

	PORTB |= (1 << PIN_DATA); //Set data line low
	PORTB &= ~(1 << PIN_DATA); //Set data line low

	PORTB |=  (1 << PIN_UPLOADING_N); //Uploading light ON

	int bits[20];
	binArray(N, bits, 20);

	//Loop through data to upload to Div-N Circuit (MSB first)
	for (signed int i = 19; i >= 0 ; i--){

		//Make sure data clock low
		PORTD &= ~(1 << PIN_DCLK);

		//Set data bit
		if (bits[i]){ //is 1
			PORTB |=  (1 << PIN_DATA);
		}else{
			PORTB &= ~(1 << PIN_DATA);
		}

		//Set data clock high
		PORTD |= (1 << PIN_DCLK);
	}

	//Set data clock low
	PORTD &= ~(1 << PIN_DCLK);

	//Cycle register clock
	PORTD &= ~(1 << PIN_RCLK);
	PORTD |=  (1 << PIN_RCLK);
	PORTD &= ~(1 << PIN_RCLK);

	PORTB &= ~(1 << PIN_UPLOADING_N); //Uploading light OFF

	PORTB &= ~(1 << PIN_DATA); //Set data line low
}


/*******************************************************************************
Converts an integer to binary and saves it to the array 'array'

Arguments:
	n - number to convert
	array - array in which to save number. idx=0 -> LSB, idx=max -> MSB
	numBits - number of bits in array

*******************************************************************************/
void binArray( unsigned long int n ,int* array, unsigned int numBits){

	unsigned long int x = n;

	for (unsigned int i = 0 ; i < numBits ; i++){
		if (0b1 & x){
			array[i] = 1;
		}else{
			array[i] = 0;
		}
		x = x >> 1;
	}


}

/*******************************************************************************
Converts a value representing a position of the frequency control knob to a
frequency value.


*******************************************************************************/
float dialToFrequency(float dial_value){

	//Calculate minimum and maximum frequency
	float max_f = PERMITTED_MAX_F/XN_MULTIPLIER;
	float min_f = PERMITTED_MIN_F;
	float hardware_min = LO_FREQ/2/pow(2, 20);

	//Verify that programmed frequency limits do not exceed hardware limitations
	if (max_f > LO_FREQ){
		max_f = LO_FREQ;
	}
	if (min_f < hardware_min){
		min_f = hardware_min;
	}

	//Calculate frequncy scaling parameters
	float alpha = log10(max_f);		// alpha = 5.346;
	float beta  = log10(min_f);		// beta = 1;
	float m = (alpha-beta)/1024;	// m = 0.0042449097;
	float epsilon = ((float)dial_value)*m+beta;		// epsilon = 3.1224548273;

	//Calculate frequency
	float freq = (float)pow(10, epsilon);

	//Check frequency is within bounds. If it isn't, that indicates a flaw in
	//this firmware.
	if (freq > max_f){
		freq = max_f;
		PORTD |= (1 << PIN_ERROR); // Turn on ERROR indicator
	}else if(freq < min_f){
		freq = min_f;
		PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
	}

	return freq;
}

/*******************************************************************************
Given a requested frequency, it determines the optimal local oscillator, if the
50% duty cycle corrector should be enabled, and what the output frequency will
be. Populates 'freq', 'N', and 'enable_50DC' with the optimum configuration.
*******************************************************************************/
void getClosestFrequency(synth_freq* sf, int opt_force_50dc){

	//Create synthesized frequency structs
	synth_freq LO1_50DC;
	synth_freq LO1;
	synth_freq LO2_50DC;
	synth_freq LO2;
	synth_freq LO1_Direct;
	synth_freq LO2_Direct;

	//Initialize their circuit enable states
	LO1_50DC.en50DC = 1;
	LO1.en50DC = 0;
	LO2_50DC.en50DC = 1;
	LO2.en50DC = 0;
	LO1_50DC.osc_idx = 0;
	LO1.osc_idx = 0;
	LO2_50DC.osc_idx = 1;
	LO2.osc_idx = 1;

	LO1_50DC.bypass_divn = 0;
	LO1.bypass_divn = 0;
	LO2_50DC.bypass_divn = 0;
	LO2.bypass_divn = 0;

	LO1_50DC.f_req = sf->f_req;
	LO1.f_req = sf->f_req;
	LO2_50DC.f_req = sf->f_req;
	LO2.f_req = sf->f_req;



	long int N; //Divisor start/ program value
	float f; //output freq
	float f_req; //Requested freq
	float delta; //Error btw reqursted and provided freq
	int en50DC; //Enable 50% DC circuit
	int osc_idx; //Which oscillator to use
	float dc; //Duty cycle
	int bypass_divn; //Bypass Div-N

	if (sf->N > 1048574 || sf->N < 0){
		sf->N = 1048574;
	}

	//Create bypass states
	LO1_Direct.N = sf->N;
	LO1_Direct.f = float(LO_FREQ);
	LO1_Direct.f_req = sf->f_req;
	LO1_Direct.osc_idx = 0;
	LO1_Direct.dc = 50;
	LO1_Direct.bypass_divn = 1;

	//Create bypass states
	LO2_Direct.N = sf->N;
	LO2_Direct.f = float(LO2_FREQ);
	LO2_Direct.f_req = sf->f_req;
	LO2_Direct.osc_idx = 1;
	LO2_Direct.dc = 50;
	LO2_Direct.bypass_divn = 1;


	long int natural_50DC_val = 1048575;

	// With each permutation of oscillator and DC corrector circuit, calculate optimum divisor
	LO1_50DC.N = FrequencyToDivisor(sf->f_req, LO_FREQ, 1);
	LO1.N = FrequencyToDivisor(sf->f_req, LO_FREQ, 0);
	if (LO2_FREQ > 1){
		LO2_50DC.N = FrequencyToDivisor(sf->f_req, LO2_FREQ, 1);
		LO2.N = FrequencyToDivisor(sf->f_req, LO2_FREQ, 0);
	}


	// Calculate frequency from optimum divisor for each permutation
	LO1_50DC.f = DivisorToFrequency(LO1_50DC.N, LO_FREQ, 1);
	LO1.f = DivisorToFrequency(LO1.N, LO_FREQ, 0);
	if (LO2_FREQ > 1){
		LO2_50DC.f = DivisorToFrequency(LO2_50DC.N, LO2_FREQ, 1);
		LO2.f = DivisorToFrequency(LO2.N, LO2_FREQ, 0);
	}

	// Calculate errors for each permutation
	LO1_50DC.delta = fabs(sf->f_req - LO1_50DC.f);
	LO1.delta = fabs(sf->f_req - LO1.f);
	if (LO2_FREQ > 1){
		LO2_50DC.delta = fabs(sf->f_req - LO2_50DC.f);
		LO2.delta = fabs(sf->f_req - LO2.f);
	}else{
		//If no secondary oscillator available, ensure it isn't selected by
		//setting it's error to the maximum value, the local oscillator.
		LO2_50DC.delta = (float)LO_FREQ;
		LO2.delta = (float)LO_FREQ;
	}
	LO1_Direct.delta = fabs(sf->f_req - LO1_Direct.f);
	LO2_Direct.delta = fabs(sf->f_req - LO2_Direct.f);
	if (LO2_FREQ < 0){
		LO2_Direct.delta = float(LO_FREQ);
	}

	//Get duty cycle
	calcDutyCycle(&LO1_50DC);
	calcDutyCycle(&LO1);
	calcDutyCycle(&LO2_50DC);
	calcDutyCycle(&LO2);

	// Interpret arguments to determine acceptable permutations
	synth_freq out = LO1_50DC;

	//Find permutation with minimum error

	float min_dc = float(PERMITTED_MIN_DC);
	if (opt_force_50dc == 1){
		min_dc = 49; //Set to 49 to protect against rounding errors
	}

	if ((LO1.dc >= min_dc) && LO1.delta < out.delta) out = LO1;
	if ((LO2.dc >= min_dc) && LO2.delta < out.delta) out = LO2;
	if (LO2_50DC.delta < out.delta) out = LO2_50DC;
	if (LO2_Direct.delta < out.delta) out = LO2_Direct;
	if (LO1_Direct.delta < out.delta) out = LO1_Direct;

	// if (opt_force_50dc == 1){
	//
	// 	//Enable LO1 for comparison if natuarally 50% DC
	// 	if (LO1.N == natural_50DC_val){
	// 		if (LO1.delta < out.delta) out = LO1;
	// 	}
	//
	// 	//Enable LO2 for comparison if natuarally 50% DC
	// 	if (LO2.N == natural_50DC_val){
	// 		if (LO2.delta < out.delta) out = LO2;
	// 	}
	//
	// 	//Check if LO2_50DC
	// 	if (LO2_50DC.delta < out.delta) out = LO2_50DC;
	//
	// }else{
	//
	// 	//Find permutation with minimum error
	// 	if (LO1.delta < out.delta) out = LO1;
	// 	if (LO2.delta < out.delta) out = LO2;
	// 	if (LO2_50DC.delta < out.delta) out = LO2_50DC;
	//
	// }

	sf->f = out.f;
	sf->N = out.N;
	sf->en50DC = out.en50DC;
	sf->osc_idx = out.osc_idx;
	sf->dc = out.dc;
	sf->bypass_divn = out.bypass_divn;

}

/*******************************************************************************
Given a requested frequency, finds the N value delivering it most closely.
Implements Eq. 155,2.
*******************************************************************************/
long int FrequencyToDivisor(float f, float f_osc, int enable_50DC){

	float dc_mult = 1;
	if (enable_50DC == 1) dc_mult = 2;

	long int N = round(1048576 - f_osc/(f*dc_mult));

	if (N > 1048574) N = 1048574;

	return N;
}

float DivisorToFrequency(long int N, float f_osc, int enable_50DC){

	float dc_mult = 1;
	if (enable_50DC == 1) dc_mult = 2;

	return f_osc/(dc_mult*(1048576 - N));
}



void calcDutyCycle(synth_freq* sf){

	//Check if 50% duty cycle circuit is on
	if (sf->en50DC){
		sf->dc = 50;
		return;
	}

	//Else calculate duty cycle from N
	sf->dc = 100/(1048576-sf->N);

}

void displayFrequency(float f, char* buffer){

	float range = 1;

	signed int dp_idx = -1;

	// Check which frequency units to use
	if (f > 1e6){
		range = 1e6;
		PORTB &= ~(1 << PIN_KHZ_IND); // OFF
		PORTB |= 1 << PIN_MHZ_IND; // ON
	}else if (f > 1e3){
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
	fto7a(buffer, DISP_BUFF_SIZE, f/range, dp_idx);

	// Send display data to shift registers on CIP-1
	shiftDataOut(buffer, DISP_BUFF_SIZE);

}

void shiftDataOut(char* buffer, long int buf_size, int reverse){

		#define PIN_DISP_DCLK PD3
		#define PIN_DATA PB2
		#define PIN_RCLK PD6

		PORTD &= ~(1 << PIN_DCLK); //, Set LOW

		if (reverse == FALSE){

			//Scan over each byte (letter on display)
			for (long int i = 0; i < buf_size; i++){

				//Scan over each bit (segment on letter)
				for (long int b = 0; b < 8 ; b++){

					// Check if segment is set
					if (buffer[i] & (1 << b)){ //Set
						PORTB |= (1 << PIN_DATA);
					}else{ //Unset
						PORTB &= ~(1 << PIN_DATA);
					}

					PORTD |= (1 << PIN_DCLK); //Clock high
					PORTD &= ~(1 << PIN_DCLK); // Clock low

				}

			}

		}else{

			//Scan over each byte (letter on display)
			for (long signed int i = buf_size-1; i >= 0; i--){

				//Scan over each bit (segment on letter)
				for (long int b = 0; b < 8 ; b++){

					// Check if segment is set
					if (buffer[i] & (1 << b)){ //Set
						PORTB |= (1 << PIN_DATA);
					}else{ //Unset
						PORTB &= ~(1 << PIN_DATA);
					}

					PORTD |= (1 << PIN_DCLK); //Clock high
					PORTD &= ~(1 << PIN_DCLK); // Clock low

				}

			}

		}


}
