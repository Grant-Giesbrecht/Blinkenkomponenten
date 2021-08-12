#include <string.h>
#include <stdlib.h>

/*
Functions to aide in drivind seven segment displays.

Returned Data Format

* Active LOW

7-Segment Display Bit Index Assignments:
	Index   Segment Name
	0       B
	1       C
	2       D
	3       E
	4       F
	5       A
	6       G
	7       DP

*/

/*
Segments to 7-Segent Data

Accepts a C-string of segments to enable. Returns the byte for one 7-segment
display with all specified segments enabled. Acceptable input is a null
terminated character array. If a character A-G is present, the corresponding
segment will be activated in the return byte. Use character 'p' to indicate the
decimal point.
*/
uint8_t sto7(char* insegs){

	uint8_t x = 0b00000000;

	// //For each character in the input list, activate the specified segment
	// for (long int i = 0 ; i < strlen(insegs); i++){
	// 	if (insegs[i] == 'A'){
	// 		x &= ~(1 << 5);
	// 	}else if (insegs[i] == 'B'){
	// 		x &= ~(1 << 0);
	// 	}else if (insegs[i] == 'C'){
	// 		x &= ~(1 << 1);
	// 	}else if (insegs[i] == 'D'){
	// 		x &= ~(1 << 2);
	// 	}else if (insegs[i] == 'E'){
	// 		x &= ~(1 << 3);
	// 	}else if (insegs[i] == 'F'){
	// 		x &= ~(1 << 4);
	// 	}else if (insegs[i] == 'G'){
	// 		x &= ~(1 << 6);
	// 	}else if (insegs[i] == 'P'){
	// 		x &= ~(1 << 7);
	// 	}
	// }

	//For each character in the input list, activate the specified segment
	for (long int i = 0 ; i < strlen(insegs); i++){
		if (insegs[i] == 'A'){
			x |= (1 << 7);
		}else if (insegs[i] == 'B'){
			x |= (1 << 6);
		}else if (insegs[i] == 'C'){
			x |= (1 << 5);
		}else if (insegs[i] == 'D'){
			x |= (1 << 4);
		}else if (insegs[i] == 'E'){
			x |= (1 << 3);
		}else if (insegs[i] == 'F'){
			x |= (1 << 2);
		}else if (insegs[i] == 'G'){
			x |= (1 << 1);
		}else if (insegs[i] == 'P'){
			x |= (1 << 0);
		}
	}

	return x;
}

/*
Int to 7-Segment Data

Converts an unsigned int to a byte for one 7-segment display with the correct
segments enabled to display that int. Accetable inputs are integers from 0 to 9.
If 'dp' is true (ie. 1), a decimal point is added.
*/
uint8_t ito7(uint8_t x, int dp){

	char s[20];

	switch(x){
	  case(0):
		strcpy(s, "ABCDEF");
		break;
	  case(1):
		strcpy(s, "BC");
		break;
	  case(2):
		strcpy(s, "ABGED");
		break;
	  case(3):
		strcpy(s, "ABGCD");
		break;
	  case(4):
		strcpy(s, "FBGC");
		break;
	  case(5):
		strcpy(s, "AFGCD");
		break;
	  case(6):
		strcpy(s, "AFGECD");
		break;
	  case(7):
		strcpy(s, "ABC");
		break;
	  case(8):
		strcpy(s, "ABCDEFG");
		break;
	  case(9):
		strcpy(s, "ABFGCD");
		break;
	  default:
		strcpy(s, "AFGED"); //'E'
		break;
	}

	if (dp == 1){
		strncat(s, "P", 1);
	}

	return sto7(s);
}

/*
Float to 7-Segment Data Array

Accepts a float 'inval', and converts each of its digits to 7-Segment data which
it saves in 'out_buf'. 'out_buf' must be at least as large as buf_size because
the entire buffer will be written.

ARGUMENTS:
	out_buf - output data buffer (populated with 7-Segment Data bytes)
	buf_size - length of 'out_buf'
	inval - Float value to convert to 7-segment display code
	dp_idx - Index at which to place decimal point. Make negative to surpress
		decimal point. Make 0 to show zero decimal places, 1 for 1, and so on
*/
bool fto7a(uint8_t* out_buf, long int buf_size, float inval, signed int dp_idx){

	//Convert to an integer, scaled up by 10^(dp) so correct no. decimals are kept
	int x = (int)round(inval * pow(10, dp_idx));

	int has_dp;

	//Loop through each cell of output buffer
	for (long int i = 0 ; i < buf_size ; i++){

		//Determine if decimal point belongs at current index
		has_dp = (dp_idx == (buf_size - 1 - i));

		//Populate output register
		// out_buf[i] = ito7( int(round(x/pow(10, buf_size-i-1)))%10, has_dp );
		out_buf[i] = ito7( int(floor(x/pow(10, buf_size-i-1)))%10, has_dp );

	}

	return true;
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
