/*
 Firmware for the BlinkenHerz-1 clock module

 Assumes 20-bit counter

 Created by Grant M Giesbrecht
 22-2-2021
 */

// NOTE: Look for comments flagged with 'ATTENTION' to find lines that need to be fixed

#include <avr/io.h>
#include <math.h>
#define F_CPU 1e6
#include <util/delay.h>
#include <avr/interrupt.h>

#define BOOL int
#define TRUE 1
#define FALSE 0

#define PIN_MANUAL_CTRL PB0
#define PIN_XN_CTRL PB1
#define PIN_DATA PB2
//
#define PIN_UPLOADING_N PB6

#define PIN_DIAL_CTRL PC0
#define PIN_PRESET_CTRL PC1

#define PIN_ERROR2 PD4
#define PIN_DCLK PD5
#define PIN_RCLK PD6
#define PIN_ERROR PD7


#define PERMITTED_MAX_F 4e6 //Maximum frequency BH allowed to generate (must be equal or less than LO_FREQ)
#define PERMITTED_MIN_F -1 //Mimimum frequency BH allowed to generate (must be greater than or equal to LO_FREQ/2^20/2). If set to -1, minimum frequency is set to minumum allowed by hardware.

#define LO_FREQ 4e6 //Speed of local oscillator
#define DIVN_MAX_RATIO 1.04858e6 //Max division ratio of divide by N
#define XN_MULTIPLIER 18 //Amount by which to increase clock speed when xN on and Continous mode
#define HYST_FACTOR .05 // %/100 by which exponent of freq can change without forcing an update

#define PRESET1_LOW_F 6 //xN OFF
#define PRESET1_HIGH_F 1e3 //xN ON

#define PRESET2_LOW_F 100e3 //xN OFF
#define PRESET2_HIGH_F 1e6 //xN ON

void updateControls(int* manual_mode, int* xNmu__mode, float* dial_value, int* preset_mode);
float getDivRatio(int manual_mode, int xNmu__mode, int dial_value);
void updateCounterIfChanges(float* divRatio, float* divRatioLast);

void updateDivNIf(int manual_mode, int xNmu__mode, float dial_value, int preset_mode, float& fLast);
void updateDivN(long int N);
void binArray( unsigned long int n ,int* array, unsigned int numBits);
uint16_t readADC();

int main(){

	//Declare Variables
	int manual_mode = FALSE;
	int xNmu__mode = FALSE;
	float dial_value = 0;
	int preset_mode = 0; //Preset modes: 0: no preset, 1: preset-1 2: preset-2, 3: preset-3

	float fLast = -1;


	//******** SET DATA DIRECTION BITS

	//Register B
	DDRB &= ~(1 << PIN_MANUAL_CTRL); //B0, Input
	DDRB &= ~(1 << PIN_XN_CTRL); //B1, Input
	DDRB |= 1 << PIN_DATA; //B2, Output
	DDRB |= 1 << PIN_UPLOADING_N; //B5, Output

	//Register C
	DDRC &= ~(1 << PIN_DIAL_CTRL); //PC0, Input

	//Register D
	DDRD |= 1 << PIN_DCLK; //, Output
	DDRD |= 1 << PIN_RCLK; //, Output
	DDRD |= 1 << PIN_ERROR; //, Output
	DDRD |= 1 << PIN_ERROR2; //, Output

	//******** SET OUTPUT STATES

	//Register B
	PORTB |= 1 << PIN_MANUAL_CTRL; //B0, Pullup resistor ON
	PORTB |= 1 << PIN_XN_CTRL; //B1, Pullup resistor ON
	PORTB &= ~(1 << PIN_DATA); //B2, Set LOW
	PORTB &= ~(1 << PIN_UPLOADING_N); //B6, Output

	//Register C
	PORTC &= ~(1 << PIN_DIAL_CTRL); //C0, Pullup resistor OFF

	//Register D
	PORTD &= ~(1 << PIN_DCLK); //, Set LOW
	PORTD &= ~(1 << PIN_RCLK); //, Set LOW
	PORTD &= ~(1 << PIN_ERROR); //, Set LOW
	PORTD &= ~(1 << PIN_ERROR2); //, Set LOW

	//************** CONFIGURE ADC

	PRR &= ~(1 << PRADC); //Power On ADC (configured to off by default in power reduction register)
	// ADMUX &= ~((1 << REFS0) | (1 << REFS1)); //Set reference voltage to VCC
	ADMUX |= (1 << REFS0);
	// ADMUX &= ~(1 << PC0) //Set ADC channel to PC0. This is the default case so I've commented it out
	ADCSRA |= ((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2)); //Set ADC clock divider to 8 for ADC frequenct of 125 khz.
	ADCSRA |= (1 << ADEN); //Enable ADC


	while (TRUE){

		//Get updated values from the controls
		updateControls(&manual_mode, &xNmu__mode, &dial_value, &preset_mode);

		// if (dial_value > 222e3){
		// 	PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
		// 	PORTD &= ~(1 << PIN_ERROR); // Turn on ERROR indicator
		// }else if (dial_value < 1){
		// 	PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
		// 	PORTD &= ~(1 << PIN_ERROR2); // Turn on ERROR indicator
		// }else{
		// 	PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
		// 	PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
		// }

		//Update Divide-by-N Circuit if big enough change
		updateDivNIf(manual_mode, xNmu__mode, dial_value, preset_mode, fLast);
	}

}

/*
Reads pin values and updates control variables from them.
*/
void updateControls(int* manual_mode, int* xNmu_mode, float* dial_value, int* preset_mode){

	//Read manual control
	if (PINB & (1 << PIN_MANUAL_CTRL)){
		*manual_mode = 1;
	}else{
		*manual_mode = 0;
	}

	//Read xNmu_ mode control
	if (PINB & (1 << PIN_XN_CTRL)){
		*xNmu_mode = 1;
	}else{
		*xNmu_mode = 0;
	}

	// *preset_mode = 0;
	// *dial_value = 10e3;

	//Read Dial Control
	//
	// thanks for the great tutorial:
	// https://mansfield-devine.com/speculatrix/2018/10/avr-basics-reading-analogue-input/
	//
	ADMUX &= 0b11110000; //Set ADC to read PC0 (Frequency Dial)
	uint16_t f_dial_adc_raw = readADC(); //Read value from ADC
	// f_dial_adc_raw = 500;
	float f_dial_adc = (float)f_dial_adc_raw;
	// f_dial_adc = 10;

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
	// f_dial_adc = 500;
	float alpha = log10(max_f);
	// alpha = 5.346;
	float beta  = log10(min_f);
	// beta = 1;
	float m = (alpha-beta)/1024;
	// m = 0.0042449097;
	float epsilon = ((float)f_dial_adc)*m+beta;
	// epsilon = 3.1224548273;

	//Calculate frequency
	float freq = (float)pow(10, epsilon);
	// freq = 1325.7;
	// freq = 10e3;
	if (freq > max_f){
		*dial_value = max_f;
		PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
		PORTD &= ~(1 << PIN_ERROR); // Turn on ERROR indicator
		// *dial_value = 10e3;
	}else if(freq < min_f){
		*dial_value = min_f;
		PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
		PORTD &= ~(1 << PIN_ERROR2); // Turn on ERROR indicator
		// *dial_value = 100;
	}else{
		*dial_value = freq;
		// PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
		// PORTD |= (1 << PIN_ERROR); // Turn on ERROR indicator
		// *dial_value = 1e3;
	}
	// f_dial_adc = float(100);

	// f_dial_adc = 500;
	// *dial_value = f_dial_adc*800.0;
	// *dial_value = 400e3;

	// *dial_value = 1e3;





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

/*
Takes conotrol values and calculates the required N-value for the divide by N
counter. This effectively defines the purpose of the controls.



--------------------------------------------------------------------------------
Controls:
	- Manual/Auto Clock Toggle: Typically auto. Manual lets you cycle through
		clocks with push button. Auto, clock runs continuously.
	- xNmu_: Multiplies each clock trigger by 18 (however many mu-steps per phi).
		In auto mode, it increases the base clock speed (set by the dial) by this
		multiplier (18). In manual mode, each clock cycle button press triggers
		18 cycles (one full phi instead of a mu).
	- Dial: Sets the base clock speed of mu-clock.

*/
void updateDivNIf(int manual_mode, int xNmu_mode, float dial_value, int preset_mode, float& fLast){

	// if (dial_value > 222e3){
	// 	PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
	// 	PORTD &= ~(1 << PIN_ERROR); // Turn on ERROR indicator
	// }else if (dial_value < 1e3){
	// 	PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
	// 	PORTD &= ~(1 << PIN_ERROR2); // Turn on ERROR indicator
	// }else{
	// 	PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
	// 	PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
	// }

	float ratio;
	float freq;

	float fLast_exp;
	float hystMin;
	float hystMax;

	preset_mode = 0;
	manual_mode = 0;

	if (preset_mode == 0){
		if (manual_mode){ //Manual/auto switch in MANUAL MODE

			//In this mode, xNmu_ is ignored by the MCU. Only controls the trigger
			// counter.

			freq = dial_value;
			// freq = 100;

			ratio = round(float(LO_FREQ)/freq);
			if (ratio > DIVN_MAX_RATIO || ratio < 1){
				PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
				return;
			}

			if (fLast < 0){
				hystMax = -1;
				hystMin = -1;
			}else{
				fLast_exp = log10(fLast); //Exponent (ie. linear with raw dial value)
				hystMin = pow(10, fLast_exp*(1-float(HYST_FACTOR))); //Min freq
				hystMax = pow(10, fLast_exp*(1+float(HYST_FACTOR))); //Max freq
			}


		}else{ //Manual/auto switch in AUTO/CONTINOUS MODE

			// dial_value = 1e3;

			// freq = 22e3;
			if (xNmu_mode){
				// dial_value = 22e3;
				// freq = dial_value *2;
				freq = dial_value*float(XN_MULTIPLIER);
				// freq = 400e3;
				// PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
				// // PORTD &= ~(1 << PIN_ERROR); // Turn on ERROR indicator
			}else{
				// dial_value = 22e3;
				freq = dial_value;
				// freq = 22e3;
				// PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
				// // PORTD &= ~(1 << PIN_ERROR2); // Turn on ERROR indicator
			}
			// freq = 723e3;

			// if (freq > 4e6){
			// 	PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
			// }else if (freq < 10){
			// 	PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
			// }

			// ratio = round(float(LO_FREQ)/(freq));
			// if (ratio > DIVN_MAX_RATIO || ratio < 1){
			// 	if (ratio > DIVN_MAX_RATIO) PORTD |= 1 << PIN_ERROR2; // Turn on ERROR indicator
			// 	if (ratio < 1) PORTD |= 1 << PIN_ERROR; // Turn on ERROR indicator
			//
			// 	return;
			// }

			if (fLast < 0){
				hystMax = -1;
				hystMin = -1;
			}else{
				fLast_exp = log10(fLast/float(XN_MULTIPLIER)); //Exponent (ie. linear with raw dial value)
				hystMin = pow(10, fLast_exp*(1-float(HYST_FACTOR)))*float(XN_MULTIPLIER); //Min freq
				hystMax = pow(10, fLast_exp*(1+float(HYST_FACTOR)))*float(XN_MULTIPLIER); //Max freq
			}
		}
	}else if(preset_mode == 1){
		if (manual_mode){
			freq = PRESET1_LOW_F;
			freq = 700e3;
		}else{
			if (xNmu_mode){
				freq = PRESET1_HIGH_F;
				freq = 700e3;
			}else{
				freq = PRESET1_LOW_F;
				freq = 700e3;
			}
		}
	}else if (preset_mode == 2){
		if (manual_mode){
			freq = PRESET2_LOW_F;
			freq = 700e3;
		}else{
			if (xNmu_mode){
				freq = PRESET2_HIGH_F;
				freq = 700e3;
			}else{
				freq = PRESET2_LOW_F;
				freq = 700e3;
			}
		}
	}

	// freq = 1e3;

	//Update divide by N counter if past hysteresis bounds
	if (freq > hystMax || freq < hystMin){
		// updateDivN(round(float(LO_FREQ)/freq));
		float mannaz = 1048576;
		float fehu = mannaz - 4e6/freq/2;
		// fehu = 1048566;
		updateDivN(round(fehu));
		fLast = freq;

	}

}

/*
Uploads the integer 'N' to the divide by N circuit
*/
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


/*
Converts an integer to binary and saves it to the array 'array'

Arguments:
	n - number to convert
	array - array in which to save number. idx=0 -> LSB, idx=max -> MSB
	numBits - number of bits in array

*/
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
