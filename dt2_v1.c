/*
 Firmware for DT-2 R2 flash memeory controller.

 Created by Grant M Giesbrecht
 8.8.2021
 */

// NOTE: Look for comments flagged with 'ATTENTION' to find lines that need to be fixed

#include <avr/io.h>
#define F_CPU 1e6
#include <util/delay.h>
#include <avr/interrupt.h>

#define BOOL int
#define TRUE 1
#define FALSE 0

//************************ CREATE MODE DEFINITIONS ********************

#define OP_STANDBY 0
#define OP_READ 1
#define OP_WRITE 2
#define OP_ERASE_SEC 3
#define OP_ERASE_CHIP 4

//*********************** DEFINE PIN LAYOUT ***************************

#define PIN_ADDR_SDP_ENABLE PB0
#define PIN_ADDR_SDP_VALUE PB1
#define PIN_DATA_SDP_VALUE PB2
#define PIN_ERROR_LIGHT PB3
#define PIN_DATA_SDP_OVRRD_1 PB4
#define PIN_DATA_SDP_OVRRD_2 PB5

#define PIN_DATA_OUTBUF_CLK PD0 //Data out buffer clock
#define PIN_ADDR_BUFF_ENABLE PD1
#define PIN_TRIGGER_OPERATION PD2
#define PIN_CLEAR_PAUSE PD3
#define PIN_DATA_SDP_OVVRD_ENABLE PD4
#define PIN_DATA_BUFF_ENABLE PD5
#define PIN_WRITE_LIGHT PD6
#define PIN_READ_LIGHT PD7

#define PIN_DATA_SDP_ENABLE PC0

#define PIN_OPERATION_CODE_0 PC2
#define PIN_OPERATION_CODE_1 PC3
#define PIN_WRITE_ENABLE PC4
#define PIN_OUTPUT_ENABLE PC5

//******************* DEFINE SDP CODE VALUES *********************

#define SDP_ADDR_5555 0
#define SDP_ADDR_2AAA 1

#define SDP_DATA_AA 2
#define SDP_DATA_55 3
#define SDP_DATA_A0 4
#define SDP_DATA_80 5
#define SDP_DATA_30 6
#define SDP_DATA_10 7

//******************* DECLARE SUBROUTINES ************************

void read_byte();
void byte_program();
void sector_erase();
void chip_erase();
void sdp_code_gen_addr(int value);
void sdp_code_gen_data(int value);

//******************* DECLARE GLOBAL VARIABLES ************************

//State variables

BOOL pause_active = FALSE;
short mode = OP_STANDBY; //Specifies what mode the FMC is in.

//*********************** DEFINE MAIN LOOP *************************

int main(){

	//*************************** CONFIGURE INPUTS ****************************

	//******** SET DATA DIRECTION BITS

	//Register B
	DDRB |= 1 << PIN_ADDR_SDP_ENABLE; //B0
	DDRB |= 1 << PIN_ADDR_SDP_VALUE; //B1
	DDRB |= 1 << PIN_DATA_SDP_VALUE; //B2
	DDRB |= 1 << PIN_ERROR_LIGHT; //B3
	DDRB |= 1 << PIN_DATA_SDP_OVRRD_1; //B4
	DDRB |= 1 << PIN_DATA_SDP_OVRRD_2; //B5
	//B6 Reserved for crystal
	//B7 Reserved for crystal

	//Register C
	DDRC |= 1 << PIN_DATA_SDP_ENABLE; //C0
	//C1 N.C.
	DDRC |= 0 << PIN_OPERATION_CODE_0; //C2
	DDRC |= 0 << PIN_OPERATION_CODE_1; //C3
	DDRC |= 1 << PIN_WRITE_ENABLE; //C4
	DDRC |= 1 << PIN_OUTPUT_ENABLE; //C5
	//C6 N.C.

	//Register D
	DDRD |= 1 << PIN_DATA_OUTBUF_CLK; //D0
	DDRD |= 1 << PIN_ADDR_BUFF_ENABLE; //D1
	DDRD |= 0 << PIN_TRIGGER_OPERATION; //D2
	DDRD |= 0 << PIN_CLEAR_PAUSE; //D3
	DDRD |= 1 << PIN_DATA_SDP_OVVRD_ENABLE; //D4
	DDRD |= 1 << PIN_DATA_BUFF_ENABLE; //D5
	DDRD |= 1 << PIN_WRITE_LIGHT; //D6
	DDRD |= 1 << PIN_READ_LIGHT; //D7

	//******** SET OUTPUT STATES

	//Register B
	PORTB |= 1 << PIN_ADDR_SDP_ENABLE; //B0
	PORTB |= 1 << PIN_ADDR_SDP_VALUE; //B1
	PORTB |= 1 << PIN_DATA_SDP_VALUE; //B2
	PORTB |= 0 << PIN_ERROR_LIGHT; //B3
	PORTB |= 1 << PIN_DATA_SDP_OVRRD_1; //B4
	PORTB |= 1 << PIN_DATA_SDP_OVRRD_2; //B5
	//B6 Reserved for Crystal
	//B7 Reserved for Crystal

	//Register C
	PORTC |= 1 << PIN_DATA_SDP_ENABLE; //C0
	//C1 N.C.
	PORTC |= 0 << PIN_OPERATION_CODE_0; //C2
	PORTC |= 0 << PIN_OPERATION_CODE_1; //C3
	PORTC |= 1 << PIN_WRITE_ENABLE; //C4
	PORTC |= 1 << PIN_OUTPUT_ENABLE; //C5
	//C6 N.C.

	//Register D
	PORTD |= 0 << PIN_DATA_OUTBUF_CLK; //D0
	PORTD |= 1 << PIN_ADDR_BUFF_ENABLE; //D1
	PORTD |= 1 << PIN_TRIGGER_OPERATION; //D2
	PORTD |= 0 << PIN_CLEAR_PAUSE; //D3
	PORTD |= 1 << PIN_DATA_SDP_OVVRD_ENABLE; //D4
	PORTD |= 1 << PIN_DATA_BUFF_ENABLE; //D5
	PORTD |= 0 << PIN_WRITE_LIGHT; //D6
	PORTD |= 0 << PIN_READ_LIGHT; //D7

	//************************* CONFIGURE INTERRUPTS **************************

	EICRA |= 1 << ISC01 | 0 << ISC00; //Configure EXINT0 for falling edge trigger
	EICRA |= 1 << ISC11 | 0 << ISC10; //Configure EXINT1 for falling edge trigger
	EIMSK |= 1 << INT0; //Enable external interrupt 0
//	EIMSK |= 1 << INT1; //Enable external interrupt 1

	sei(); //Enable interrupts

	//************************** DECLARE STATE VARIABLES **************************

	while (1==1){

		//Hold - wait for interrupt to trigger an operation subroutine

//		PORTD |= 1 << PIN_WRITE_LIGHT;
//		PORTD &= ~(1 << PIN_READ_LIGHT);
//		PORTD &= ~(1 << PIN_WRITE_LIGHT);
//		PORTD |= 1 << PIN_READ_LIGHT;

	}

	return 0;
}

//***************** DEFINE INTERRUPT SERVICE ROUTINE *****************

ISR(INT0_vect){ //ISR_NOBLOCK Allows interrupts to be called from within the ISR


	//Check to see if FMC is in standby mode, else ignore trigger
	if (mode != OP_STANDBY) return;

	//Read values on command pins, set mode & call operation subroutine
	if (PINC & (1<<PC2)){ //If OP_CODE_0 is HIGH
		if (PINC & (1<<PC3)){ //If OP_CODE_1 is HIGH
			mode = OP_ERASE_SEC;
			sector_erase();
		}else{ //If OP_CODE_1 is LOW
			mode = OP_WRITE;
			byte_program();
		}
	}else{ //If OP_CODE_0 is LOW
		if (PINC & (1<<PC3)){ //If OP_CODE_1 is HIGH
			mode = OP_READ;
			read_byte();
		}else{ //If OP_CODE_1 is LOW
			mode = OP_ERASE_CHIP;
			chip_erase();
		}
	}

}

//ISR(INT1_vect){
//
//	PORTD |= 1 << PIN_WRITE_LIGHT;
//	pause_active = FALSE;
//
//	return;
//}

//**************************** DEFINE OPERATION SUBROUTINES ***************************

/*
 Reads a byte from flash memory and outputs to the computer's main bus.

 Trigger:
  * Triggered by a low pulse on INT0
  * Pauses when data output to main bus while PD3 is HIGH. Proceeds on PD3 LOW.

 Requirements:
 ADDRESS BUFFERS: Must contain the first two bytes of the address to read

 Results:
 MAIN BUS: Data value is output to main bus. (NOTE: Bus must be clear!)
 */
void read_byte(){

	pause_active = TRUE;

	//Activate read indicator
	PORTD |= 1 << PIN_READ_LIGHT; //Turn READ_LIGHT -> HIGH

	//Operation 1
	PORTD &= ~(1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUFF_ENABLE -> LOW (ON)

	//Operation 2
	PORTC |= 1 << PIN_WRITE_ENABLE; //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 3
	PORTC &= ~(1 << PIN_OUTPUT_ENABLE); //Turn OUTPUT_ENABLE -> LOW (ON)

	//Operation 4
	PORTD |= (1 << PIN_DATA_OUTBUF_CLK); //Turn DATA_TO_BUS_ENABLE -> HIGH (ON)
	PORTD &= ~(1 << PIN_DATA_OUTBUF_CLK); //Turn DATA_TO_BUS_ENABLE -> LOW (OFF)

	//Operation 5
	//Continuously scans PIN_CLEAR_PAUSE, passes when low
	while (TRUE){
		if ((~PIND & (1 << PIN_CLEAR_PAUSE))){
			break;
		}
	}

	//Operation 7
	PORTC |= 1 << PIN_OUTPUT_ENABLE; //Turn OUTPUT_ENABLE -> HIGH (OFF)

	//Operation 8
	PORTD |= 1 << PIN_ADDR_BUFF_ENABLE; //Turn ADDR_BUFF_ENABLE -> HIGH (OFF)

	//Remove read indication
	PORTD &= ~(1 << PIN_READ_LIGHT); //Turn READ_LIGHT -> LOW

	//Reset mode to standby
	mode = OP_STANDBY;

	return;
}

/*
 Writes a byte to flash memory

 Requirements:
 ADDRESS BUFFERS: Must contain the first two bytes of the address to write
 DATA BUFFER: Must contain the byte to write

 Results:
 FLASH MEMORY: Writes the byte to the address in flash memory
 */
void byte_program(){

	//Activate write indicator
	PORTD  |= (1 << PIN_WRITE_LIGHT);

	//Operation 1
	PORTC |= 1 << PIN_OUTPUT_ENABLE; //Turn OUTPUT_ENABLE -> HIGH (OFF)

	//Operation 2
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 3
	PORTD |= (1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUFF_ENABLE -> HIGH (OFF)

	//Operation 4
	PORTD |= (1 << PIN_DATA_BUFF_ENABLE); //Turn DATA_BUFF_ENABLE -> HIGH (OFF)

	//Operation 5
	PORTB &= ~(1 << PIN_ADDR_SDP_ENABLE); //Turn ADDR_SDP_ENABLE -> LOW (ON)

	//Operation 6
	PORTC &= ~(1 << PIN_DATA_SDP_ENABLE); //Turn DATA_SDP_ENABLE -> LOW (ON)

	//Operations 7-10
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to 5555
	sdp_code_gen_data(SDP_DATA_AA); //Set data SDP to AA
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 11-14
	sdp_code_gen_addr(SDP_ADDR_2AAA); //Set addr. SDP to 2AAA
	sdp_code_gen_data(SDP_DATA_55); //Set data SDP to 55
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 15-18
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to 5555
	sdp_code_gen_data(SDP_DATA_A0); //Set data SDP to A0
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 19
	PORTB |= (1 << PIN_ADDR_SDP_ENABLE); //Turn ADDR_SDP_ENABLE -> HIGH (OFF)

	//Operation 20
	PORTC |= (1 << PIN_DATA_SDP_ENABLE); //Turn DATA_SDP_ENABLE -> HIGH (OFF)

	//Operation 21
	PORTD &= ~(1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUFF_ENABLE -> LOW (ON)

	//Operation 22
	PORTD &= ~(1 << PIN_DATA_BUFF_ENABLE); //Turn DATA_BUFF_ENABLE -> LOW (ON)

	//Operation 23
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)

	//Operation 24
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 25
	PORTD |= (1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUFF_ENABLE -> HIGH (OFF)

	//Operation 26
	PORTD |= (1 << PIN_DATA_BUFF_ENABLE); //Turn DATA_BUFF_ENABLE -> HIGH (OFF)

	//Remove write indication
	PORTD &= ~(1 << PIN_WRITE_LIGHT);

	mode = OP_STANDBY;
}

void sector_erase(){

	//Activate write indicator
	PORTD  |= (1 << PIN_WRITE_LIGHT);

	//Operation 1
	PORTC |= 1 << PIN_OUTPUT_ENABLE; //Turn OUTPUT_ENABLE -> HIGH (OFF)

	//Operation 2
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 3
	PORTD |= (1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUFF_ENABLE -> HIGH (OFF)

	//Operation 4
	PORTD |= (1 << PIN_DATA_BUFF_ENABLE); //Turn DATA_BUFF_ENABLE -> HIGH (OFF)

	//Operation 5
	PORTB &= ~(1 << PIN_ADDR_SDP_ENABLE); //Turn ADDR_SDP_ENABLE -> LOW (ON)

	//Operation 6
	PORTC &= ~(1 << PIN_DATA_SDP_ENABLE); //Turn DATA_SDP_ENABLE -> LOW (ON)

	//Operations 7-10
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_AA); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 8-14
	sdp_code_gen_addr(SDP_ADDR_2AAA); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_55); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 15-18
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_80); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 19-22
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_AA); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 23-26
	sdp_code_gen_addr(SDP_ADDR_2AAA); //Set addr. SDP to 2AAA
	sdp_code_gen_data(SDP_DATA_55); //Set data SDP to 55
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 27
	sdp_code_gen_data(SDP_DATA_30); //Set data SDP to 30

	//Operation 28
	PORTB |= (1 << PIN_ADDR_SDP_ENABLE); //Turn ADDR_SDP_ENABLE -> HIGH (OFF)

	//Operation 29
	PORTD &= ~(1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUF_ENABLE -> LOW (ON)

	//Operation 30
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)

	//Operation 31
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 32
	PORTC |= (1 << PIN_DATA_SDP_ENABLE); //Turn DATA_SDP_ENABLE -> HIGH (OFF)

	//Operation 33
	PORTD |= (1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUF_ENABLE -> HIGH (OFF)

	//Operation 34
	_delay_ms(25); //25 ms pause for sector erase to complete

	//Remove write indication
	PORTD &= ~(1 << PIN_WRITE_LIGHT);

	mode = OP_STANDBY;
}

void chip_erase(){

	//Activate write indicator
	PORTD  |= (1 << PIN_WRITE_LIGHT);

	//Operation 1
	PORTC |= 1 << PIN_OUTPUT_ENABLE; //Turn OUTPUT_ENABLE -> HIGH (OFF)

	//Operation 2
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 3
	PORTD |= (1 << PIN_ADDR_BUFF_ENABLE); //Turn ADDR_BUFF_ENABLE -> HIGH (OFF)

	//Operation 4
	PORTD |= (1 << PIN_DATA_BUFF_ENABLE); //Turn DATA_BUFF_ENABLE -> HIGH (OFF)

	//Operation 5
	PORTB &= ~(1 << PIN_ADDR_SDP_ENABLE); //Turn ADDR_SDP_ENABLE -> LOW (ON)

	//Operation 6
	PORTC &= ~(1 << PIN_DATA_SDP_ENABLE); //Turn DATA_SDP_ENABLE -> LOW (ON)

	//Operations 7-10
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_AA); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 11-14
	sdp_code_gen_addr(SDP_ADDR_2AAA); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_55); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 15-18
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_80); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 19-22
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_AA); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 23-26
	sdp_code_gen_addr(SDP_ADDR_2AAA); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_55); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operations 27-30
	sdp_code_gen_addr(SDP_ADDR_5555); //Set addr. SDP to
	sdp_code_gen_data(SDP_DATA_10); //Set data SDP to
	PORTC &= ~(1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> LOW (ON)
	PORTC |= (1 << PIN_WRITE_ENABLE); //Turn WRITE_ENABLE -> HIGH (OFF)

	//Operation 31
	PORTB |= (1 << PIN_ADDR_SDP_ENABLE); //Turn ADDR_SDP_ENABLE -> HIGH (OFF)

	//Operation 32
	PORTC |= (1 << PIN_DATA_SDP_ENABLE); //Turn DATA_SDP_ENABLE -> HIGH (OFF)

	//Operation 33
	_delay_ms(100); //100 ms pause for chip erase to complete

	//Remove write indication
	PORTD &= ~(1 << PIN_WRITE_LIGHT);

	mode = OP_STANDBY;
}

//**************************** DEFINE SDP CODE GENERATOR SUBROUTINES ***************************

/*
 Commands address hardware-SDP generator to generate the specified SDP code.

 Arguments:
 	value - SDP code to generate

 Void return.
 */
void sdp_code_gen_addr(int value){

	switch (value) {
		case SDP_ADDR_2AAA:
			PORTB &= ~(1 << PB0); //Pin 14 LOW
			PORTB &= ~(1 << PB1); //Pin 15 LOW
			break;
		case SDP_ADDR_5555:
			PORTB &= ~(1 << PB0); //Pin 14 LOW
			PORTB |= (1 << PB1); //Pin 15 HIGH
			break;
		default:
			PORTB |= 1 << PIN_ERROR_LIGHT; //Set error light
			break;
	}

}

/*
 Commands data hardware-SDP generator to generate the specified SDP code.

 Arguments:
 	value - SDP code to generate

 Void return.
 */
void sdp_code_gen_data(int value){
	switch (value) {
		case SDP_DATA_10:
			PORTB |= (1 << PB4); //Pin B4
			PORTB &= ~(1 << PB5); //Pin B5
			PORTB &= ~(1 << PB2); //Pin B2
			PORTD &= ~(1 << PD4); //Pin D4
			PORTC &= ~(1 << PC0); //Pin C0
			break;
		case SDP_DATA_30:
			PORTB |= (1 << PB4); //Pin B4
			PORTB |= (1 << PB5); //Pin B5
			PORTB &= ~(1 << PB2); //Pin B2
			PORTD &= ~(1 << PD4); //Pin D4
			PORTC &= ~(1 << PC0); //Pin C0
			break;
		case SDP_DATA_80:
			PORTB &= ~(1 << PB4); //Pin B4
			PORTB &= ~(1 << PB5); //Pin B5
			PORTB &= ~(1 << PB2); //Pin B2
			PORTD &= ~(1 << PD4); //Pin D4
			PORTC &= ~(1 << PC0); //Pin C0
			break;
		case SDP_DATA_A0:
			PORTB &= ~(1 << PB4); //Pin B4
			PORTB |= (1 << PB5); //Pin B5
			PORTB &= ~(1 << PB2); //Pin B2
			PORTD &= ~(1 << PD4); //Pin D4
			PORTC &= ~(1 << PC0); //Pin C0
			break;
		case SDP_DATA_55:
			PORTB &= ~(1 << PB4); //Pin B4
			PORTB &= ~(1 << PB5); //Pin B5
			PORTB |= (1 << PB2); //Pin B2
			PORTD |= (1 << PD4); //Pin D4
			PORTC &= ~(1 << PC0); //Pin C0
			break;
		case SDP_DATA_AA:
			PORTB &= ~(1 << PB4); //Pin B4
			PORTB &= ~(1 << PB5); //Pin B5
			PORTB &= ~(1 << PB2); //Pin B2
			PORTD |= (1 << PD4); //Pin D4
			PORTC &= ~(1 << PC0); //Pin C0
			break;
		default:
			PORTB |= 1 << PIN_ERROR_LIGHT; //Set error light
			break;
	}
}
