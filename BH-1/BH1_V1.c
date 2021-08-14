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

#define ENABLE_ALT_OSC 0

#define DIVN_MAX_RATIO 1.04858e6 //Max division ratio of divide by N

#define DISP_BUFF_SIZE 3

//************************** CONFIGURATION OPTIONS ***************************//

#define PERMITTED_MAX_F 4e6 //Maximum frequency BH allowed to generate (must be equal or less than LO_FREQ)
#define PERMITTED_MIN_F -1 //Mimimum frequency BH allowed to generate (must be greater than or equal to LO_FREQ/2^20/2). If set to -1, minimum frequency is set to minumum allowed by hardware.

#define XN_MULTIPLIER 18 //Amount by which to increase clock speed when xN on and Continous mode
#define HYST_FACTOR .05 // %/100 by which exponent of freq can change without forcing an update
#define PERMITTED_MIN_DC 10 //Minimum duty cycle allowed in normal operation (%)
#define DISP_UPDATE_T 50 //Amount of time that must ellapse before the readout is updated (mS)
#define DISP_UPDATE_T_SLOW 1e3 //Amount of time that must ellapse before the readout is updated (mS)

#define PRESET1_ALPHA 6
#define PRESET1_BETA 1e3
#define PRESET1_GAMMA 100e3
#define PRESET1_DELTA 1e6
#define PRESET1_EPSILON 2e6

//************************ FUNCTION DEFINITIONS ******************************//

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

typedef struct{
	int manual_mode;
	int mult_mode;
	float dial_value;
	int preset;
	int lock_f;
	int opt_force_50dc;
	int opt_nohyst;
	int opt_slowdisp;
	float last_freq;

	float time_disp_update;
}ctrl_state;

void readControls(ctrl_state& ctrl);
float getDivRatio(ctrl_state& ctrl);
void updateCounterIfChanges(float* divRatio, float* divRatioLast);

void InterpretControls(ctrl_state& ctrl, synth_freq& sf);
void updateDivN(long int N);
void binArray( unsigned long int n ,int* array, unsigned int numBits);
uint16_t readADC();
float dialToFrequency(float dial_value);
void displayFrequency(float f, uint8_t* displayBuffer);
void shiftDataOut(uint8_t* displayBuffer, long int buf_size, int reverse=FALSE);



void getClosestFrequency(synth_freq* sf, int opt_force_50dc);
float DivisorToFrequency(long int N, float f_osc, int enable_50DC);
long int FrequencyToDivisor(float f, float f_osc, int enable_50DC);
void calcDutyCycle(synth_freq* sf);

/*******************************************************************************
/																			   /
*******************************************************************************/
int main(){

	//Initailize display buffer
	uint8_t displayBuffer[DISP_BUFF_SIZE];

	//Initialize main synthesized frequency
	synth_freq main_sf;

	//Initilaize control state
	ctrl_state ctrl;
	ctrl.manual_mode = FALSE;
	ctrl.mult_mode = FALSE;
	ctrl.dial_value = 0;
	ctrl.preset = 0; //Preset modes: 0: no preset, 1: preset-1 2: preset-2, 3: preset-3
	ctrl.lock_f = FALSE;
	ctrl.opt_force_50dc = FALSE;
	ctrl.opt_nohyst = FALSE;
	ctrl.opt_slowdisp = FALSE;
	ctrl.last_freq = -1;


	//Settings for timer
	TCNT0 = 0;
	TCCR0B |= (0<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK0 = (1<<TOIE0);
	sei();

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

	unsigned long last_update_time = millis();
	float f_displayed = -1;

	while (TRUE){

		// Get updated values from the controls
		readControls(ctrl);

		// //Update Divide-by-N Circuit if big enough change
		InterpretControls(ctrl, main_sf);

		// sendFreqToHardware();

		if ( millis() - last_update_time >= ctrl.time_disp_update){// && f_displayed != main_sf.f){

			// ************* THE NUMBER HERE WILL BE DISPLAYED CORRECTLY ******

			//DEBUG OPTION: Can enable this block to print alternate parameters
			// if (ctrl.lock_f){
			// 	displayFrequency(main_sf.f, displayBuffer);
			// }else{
			// 	displayFrequency(main_sf.N, displayBuffer);
			// }
			// displayFrequency(millis() - last_update_time, displayBuffer);

			displayFrequency(main_sf.f, displayBuffer);

			f_displayed = main_sf.f;

			last_update_time = millis();
		}



	}

}

/*******************************************************************************
Reads pin values and updates control variables from them.
*******************************************************************************/
void readControls(ctrl_state& ctrl){

	//******************** READ TOGGLE SWITCHES ******************************//

	//Read manual control
	if (PINB & (1 << PIN_MANUAL_CTRL)){
		ctrl.manual_mode = 1;
	}else{
		ctrl.manual_mode = 0;
	}

	//Read lock control, update lock light
	if (PIND & (1 << PIN_LOCK_CTRL)){
		ctrl.lock_f = 1;
		PORTB |=  (1 << PIN_LOCK_IND); //Uploading light ON
	}else{
		ctrl.lock_f = 0;
		PORTB &=  ~(1 << PIN_LOCK_IND); //Uploading light OFF
	}

	//Read xN_ mode control
	if (PINB & (1 << PIN_XN_CTRL)){
		ctrl.mult_mode = 1;
	}else{
		ctrl.mult_mode = 0;
	}

	//*********************** READ OPTION HEADERS ****************************//

	//Read force 50% DC option
	if (PIND & (1 << PIN_FORCE_50DC)){
		ctrl.opt_force_50dc = 1;
	}else{
		ctrl.opt_force_50dc = 0;
	}

	//Read force no hysteresis option
	if (PIND & (1 << PIN_NOHYST)){
		ctrl.opt_nohyst = 1;
	}else{
		ctrl.opt_nohyst = 0;
	}

	//Read force slow display update option
	if (PIND & (1 << PIN_SLOWDISP)){
		ctrl.opt_slowdisp = 1;
		ctrl.time_disp_update = DISP_UPDATE_T_SLOW;
	}else{
		ctrl.opt_slowdisp = 0;
		ctrl.time_disp_update = DISP_UPDATE_T;
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

	ctrl.dial_value = f_dial_adc;


	// ****************** READ PRESETS CONTROL **************** //

	//Check preset setting
	if (PINC & (1 << PIN_PRESET_CTRL) ){
		ctrl.preset = TRUE;
	}else{
		ctrl.preset = FALSE;
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
Given a requested frequency, finds the N value delivering it most closely.
Implements Eq. 155,2.
*******************************************************************************/
long int FrequencyToDivisor(synth_freq* sf){

	if (sf->bypass_divn){
		return 1;
	}

	float f_osc;
	if (sf->osc_idx == 0){
		f_osc = LO_FREQ;
	}else{
		f_osc = LO2_FREQ;
	}


	float dc_mult = 1;
	if (sf->en50DC == 1) dc_mult = 2;

	long int N = round(1048576 - f_osc/(sf->f_req * dc_mult));

	if (N > 1048574) N = 1048574;

	return N;
}

float DivisorToFrequency(synth_freq* sf){

	float f_osc;
	if (sf->osc_idx == 0){
		f_osc = LO_FREQ;
	}else{
		f_osc = LO2_FREQ;
	}

	if (sf->bypass_divn){
		return f_osc;
	}

	float dc_mult = 1;
	if (sf->en50DC == 1) dc_mult = 2;

	return f_osc/(dc_mult*(1048576 - sf->N));
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

/*******************************************************************************
Given a requested frequency, it determines the optimal local oscillator, if the
50% duty cycle corrector should be enabled, and what the output frequency will
be. Populates 'freq', 'N', and 'enable_50DC' with the optimum configuration.
*******************************************************************************/
void getClosestFrequency(synth_freq* sf, int opt_force_50dc){

	//************** CONFIGURE SF STRUCTS *************************************

	//Create array of SF options
	synth_freq all_sf[6];

	// LO=PRIME, DC=50%, BYPASS=FALSE
	all_sf[0].en50DC = TRUE;
	all_sf[0].osc_idx = 0;
	all_sf[0].bypass_divn = FALSE;

	// LO=PRIME, DC=?, BYPASS=FALSE
	all_sf[1].en50DC = FALSE;
	all_sf[1].osc_idx = 0;
	all_sf[1].bypass_divn = FALSE;

	// LO=SECOND, DC=50%, BYPASS=FALSE
	all_sf[2].en50DC = TRUE;
	all_sf[2].osc_idx = ENABLE_ALT_OSC;
	all_sf[2].bypass_divn = FALSE;

	// LO=SECOND, DC=?, BYPASS=FALSE
	all_sf[3].en50DC = FALSE;
	all_sf[3].osc_idx = ENABLE_ALT_OSC;
	all_sf[3].bypass_divn = FALSE;

	// LO=PRIME, DC=?, BYPASS=TRUE
	all_sf[4].en50DC = FALSE;
	all_sf[4].osc_idx = 0;
	all_sf[4].bypass_divn = TRUE;

	// LO=SECOND, DC=?, BYPASS=TRUE
	all_sf[5].en50DC = FALSE;
	all_sf[5].osc_idx = ENABLE_ALT_OSC;
	all_sf[5].bypass_divn = TRUE;

	//****************** GLOABL OPERATIONS ************************************

	//Globally...
	for (int si = 0 ; si < 6 ; si++){

		// Set requested frequency
		all_sf[si].f_req = sf->f_req;

		// Calculate closest divisor
		all_sf[si].N = FrequencyToDivisor(&all_sf[si]);

		// Calculate frequency from divisor
		all_sf[si].f = DivisorToFrequency(&all_sf[si]);

		// Calculate error
		all_sf[si].delta = fabs(sf->f_req - all_sf[si].f);

		// Calculate duty cycle
		calcDutyCycle(&all_sf[si]);
	}

	//******************** SELECT OPTIMAL CONFIGURATION ************************

	// Determine duty cycle tolerance
	float min_dc = float(PERMITTED_MIN_DC);
	if (opt_force_50dc == 1){
		min_dc = 49; //Set to 49 to protect against rounding errors
	}

	// Scan over all options, select option with lowest error
	synth_freq best = all_sf[0];
	for (int si = 1; si < 6 ; si++){

		// Skip configurations which do not meet the DC requirements
		if (all_sf[si].dc < min_dc) continue;

		// Check if struct beats current best
		if (all_sf[si].delta < best.delta){
			best = all_sf[si];
		}

	}

	// Transfer best SF struct's parameters to the original struct
	sf->f = best.f;
	sf->N = best.N;
	sf->en50DC = best.en50DC;
	sf->osc_idx = best.osc_idx;
	sf->dc = best.dc;
	sf->bypass_divn = best.bypass_divn;
}

void InterpretControls(ctrl_state& ctrl, synth_freq& sf){

	float ratio;
	float freq_requested;

	float fLast_exp;
	float hystMin;
	float hystMax;

	// ctrl.preset_mode = 0;
	// ctrl.manual_mode = 0;

	if (ctrl.preset == FALSE){	//Preset mode is set to OFF/ANALOG

		if (ctrl.manual_mode){ //Manual/auto switch in MANUAL MODE

			//NOTE: In this mode, xN-control is ignored by the MCU. Only controls the trigger counter.

			//Calculate frequency
			freq_requested = dialToFrequency(ctrl.dial_value);

		}else{ //Manual/auto switch in AUTO/CONTINOUS MODE

			//Calculate frequency
			freq_requested = dialToFrequency(ctrl.dial_value);

			//If multiplier is on, multiply freqency by 'N'
			if (ctrl.mult_mode) freq_requested *= float(XN_MULTIPLIER);

		}
	}else{ //Preset mode is ON

		//In this mode, xN is ignored by the MCU. Only controls the trigger
		//counter.

		if (ctrl.dial_value < 205){
			freq_requested = PRESET1_ALPHA;
		}else if(ctrl.dial_value < 410){
			freq_requested = PRESET1_BETA;
		}else if(ctrl.dial_value < 614){
			freq_requested = PRESET1_GAMMA;
		}else if(ctrl.dial_value < 819){
			freq_requested = PRESET1_DELTA;
		}else{
			freq_requested = PRESET1_EPSILON;
		}

	}

	//Determine closest frequency to requested frequency that can be provided
	synth_freq local_sf;
	local_sf.f_req = freq_requested;
	getClosestFrequency(&local_sf, ctrl.opt_force_50dc);

	//Calculate hysteresis thresholds
	if (ctrl.last_freq < 0){
		hystMax = -1;
		hystMin = -1;
	}else{
		fLast_exp = log10(ctrl.last_freq/float(XN_MULTIPLIER)); //Exponent (ie. linear with raw dial value)
		hystMin = pow(10, fLast_exp*(1-float(HYST_FACTOR)))*float(XN_MULTIPLIER); //Min freq
		hystMax = pow(10, fLast_exp*(1+float(HYST_FACTOR)))*float(XN_MULTIPLIER); //Max freq
	}

	//Update divide by N counter if past hysteresis bounds
	if (( ctrl.opt_nohyst || local_sf.f > hystMax || local_sf.f < hystMin) && (ctrl.lock_f == 0)){

		sf = local_sf;

		//Select local oscillator
		if (sf.osc_idx == 1){
			PORTC |= (1 << PIN_OSC_SELECT); //PC3, ON (Use secondary osc)
		}else{
			PORTC &= ~(1 << PIN_OSC_SELECT); //PC3, OFF (Use Primary osc)
		}

		//Select state of divider bypass MUX
		// sf.bypass_divn = 0;
		if (sf.bypass_divn == 1){
			PORTC |= (1 << PIN_DIVN_BYPASS); //PC3, ON (Use oscillator directly)
		}else{
			PORTC &= ~(1 << PIN_DIVN_BYPASS); //PC3, OFF (Use Divider)
		}

		//Select state of duty cycle corrector MUX
		if (sf.en50DC == 1){
			DDRB |= 1 << PIN_DC_CORRECTOR; //B7, enable DC corrector
		}else{
			DDRB &= ~(1 << PIN_DC_CORRECTOR); //B7, disable DC Corrector
		}


		// ************ THE DIV.N VALUE HERE WORKS ************************** // DEBUG
		updateDivN(sf.N);
		// ****************************************************************** //


		// updateDivN(round(1048576 - 4e6/main_sf.f/2));
		ctrl.last_freq = local_sf.f;
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

/*******************************************************************************
Uploads the integer 'N' to the divide by N circuit
*******************************************************************************/
void updateDivN(long int N){

	// PORTB |= (1 << PIN_DATA); //Set data line low
	PORTB &= ~(1 << PIN_DATA); //Set data line low
	PORTD &= ~(1 << PIN_RCLK);

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
