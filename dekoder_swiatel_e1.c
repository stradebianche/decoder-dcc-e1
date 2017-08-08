/*
 * dekoder_swiatel_e1.c
 *
 * Created: 2017-06-28 20:24:53
 * Author : Fartek
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


static void decode( uint8_t bit );

static void decode_msg( void );

static void programme_cv( void );


/* Command variables */
#define MAX_MESSAGE 6  
volatile unsigned char message[MAX_MESSAGE];
volatile uint8_t is_new_msg = 0;
volatile uint8_t new_msg_size = 0;
typedef enum state {DCC_WF_PREAMBLE, DCC_WF_ZERO, DCC_WF_BYTE, DCC_WF_END} state;

/* EEPROM CV storing values */
uint8_t EEMEM ee_cv1_short_address = 3;
uint8_t EEMEM ee_cv17_long_address_msb;
uint8_t EEMEM ee_cv18_long_address_lsb;
uint8_t EEMEM ee_cv29_config_data;

/* CV registers numbers definitions */
#define CV_1_PRIMARY_ADDRESS		0
#define CV_17_LONG_ADDRESS_MSB		16
#define CV_18_LONG_ADDRESS_LSB		17
#define CV_29_CONFIG_DATA			28

/* Current decoder address */
volatile uint16_t current_address = 3;
volatile uint8_t ext_addr_enable = 0;

typedef enum decoder_state { SERVICE_MODE_ENABLED, SERVICE_MODE_DISABLED, OPERATIONS_MODE } decoder_state;

uint8_t received_cv_operation, received_cv_number, received_cv_value;

/* System counter */
uint8_t service_mode_tick = 0;
uint8_t tick_count = 0; 

volatile uint8_t dimming_enable = 0;
volatile uint8_t pwm_fill = 7;
volatile uint8_t light_up_fill = 0;
volatile uint8_t light_down_fill = 0;
volatile uint8_t light_up_enable = 0;
volatile uint8_t light_shut_down_enable = 0;
volatile uint8_t dimming_exit = 0;
/* Dimme */
volatile uint8_t pwm_cnt = 0;
volatile uint16_t t_cnt = 0;


/* Function prototype */


int main( void ) {

	/* Load current address from memory */
	current_address = eeprom_read_byte(&ee_cv1_short_address);
	current_address = 3;

	/* Output pins */
	DDRB |= 0xff;
	DDRD = (1<<PIND3) | (1<<PIND4);

	/* Input pins */
	DDRA &= ~(1 << PINA0);	/* pcint8 */
	DDRD &= ~(1 << PIND2);  /* int0 */

	/* Init interrupt */
	MCUCR |= (1 << ISC00);
	GIMSK |= (1 << INT0);

	/* Init dcc base timer - 1us per tick */
	TCCR0B |= (1 << CS01);
	TCNT0 = 0;

	/* Init 1ms control timer */
	TCCR1B |= (1 << CS10)| (1 << CS11) | (1 << WGM12);
	OCR1AL = 124;
	/* Only for debug */
	//OCR1AL = 60;
	//OCR1AH = 255;
	TIMSK |= (1 << OCIE1A);

	sei();

    while (1) {

		if( is_new_msg ) {
			decode_msg();
			is_new_msg = 0;	
		}
    }
}


/* A moze dac odwolanie do tablicy adresow do programowania? */
static void programme_cv( void ) {

	/* Write bit */
	if( received_cv_operation == 0b11 ) {

		/* Programing primary address */
		if( received_cv_number == CV_1_PRIMARY_ADDRESS ) {
			cli();
			eeprom_write_byte(&ee_cv1_short_address, received_cv_value);
			sei();
		}

		else if( received_cv_number == CV_17_LONG_ADDRESS_MSB ) {
			cli();
			eeprom_write_byte(&ee_cv17_long_address_msb, received_cv_value);
			sei();
		}

		else if( received_cv_number == CV_18_LONG_ADDRESS_LSB ) {
			cli();
			eeprom_write_byte(&ee_cv18_long_address_lsb, received_cv_value);
			sei();
		}

		else if( received_cv_number == CV_29_CONFIG_DATA ) {
			cli();
			eeprom_write_byte(&ee_cv29_config_data, received_cv_value);
			sei();
		}
	}

	/* Bit manipulation */
	else if( received_cv_operation == 0b10 ) {
		//if( received_cv_value == 28 )
			PORTB |= (1 << PINB4);
	}
}





/* Decodes dcc message */
static void decode_msg(void) {
	
	static uint8_t sm_enabled = 0;
	static uint8_t sm_received = 0;

	/* Check CRC by calculating XOR */
	uint8_t crc_xor = 0;

	for( int i = 0; i < (new_msg_size - 1); i++ )
		crc_xor ^= message[i];

	if( crc_xor != message[new_msg_size - 1] )
		return;


	/* Service mode routine */
	if( sm_enabled ) {

		/* Check if service mode time expired */
		if( service_mode_tick > 20 ) {
			sm_enabled = 0;
			sm_received = 0;
			return;
		}
		
		/* Reset message - enter service mode */
		if( message[0] == 0 ) {
			if( message[1] == 0 ) {
				sm_enabled = 1;
				service_mode_tick = 0;
				return;
			}
		}

		/* Service mode instruction received */
		else if ( (message[0] >= 112) && (message[0] <= 127) ) {

			/* Direct CV programming - uses 4 bytes of data */
			if( new_msg_size == 4 ) {

				service_mode_tick = 0;
				
				if( sm_received ) {
					/*Handle second message - verify correctness */
					uint8_t temp_cc;
					uint16_t temp_cv_adr;

					temp_cc = ( message[0] & 0b00001100 );
					temp_cc = temp_cc >> 2;

					temp_cv_adr = ( (message[0] & 0b00000011) << 8 ) | message[1];

					if( (temp_cc == received_cv_operation) && (temp_cv_adr == received_cv_number) && (message[2] == received_cv_value) ) {
						programme_cv();
					}

					sm_received = 0;
				}

				else {
					/*Handle first programming message */
					sm_received = 1;

					received_cv_operation = ( message[0] & 0b00001100 );
					received_cv_operation = received_cv_operation >> 2;
						
					received_cv_number = ( (message[0] & 0b00000011) << 8 ) | message[1] ;
						
					received_cv_value = message[2];	
				}
			}
			/* Other programming modes not supported */
			return;
		}
	}


	/* Disable service mode */
	sm_enabled = 0;
	sm_received = 0;


	/* Broadcast message */
	if( message[0] == 0 ) {
		if( message[1] == 0 ) {
			sm_enabled = 1;
			service_mode_tick = 0;
		}
	}


	//Combine 2digit with 4digit address

	/* Two digit address */
	else if( message[0] <= 127 ) { 
		
		/* Function group one */
		if( message[0] == current_address ) {


		}


				if( ( message[0] == current_address ) && ( message[1] & 128 ) ) {


/*
 *		This should be like
 *		decode msg -> 
 *		run specific function ie
 *		function_group_one( data );
 *
 */
					static uint8_t prev_val;

					/* Odebrano funkcje F0 */
					if( ( message[1] & (1 << 4) ) ) {
						
						if( !prev_val ) {
							light_up_fill = 0;
							light_shut_down_enable = 0;
							t_cnt = 0;
							pwm_cnt = 0;
							light_up_enable = 1;
							prev_val = 1;
						}
			
					}

					/* Kasowanie funkcji F0 */
					else if( !(message[1] & (1 << 4) ) ) {

						if( prev_val ) {
							light_down_fill = pwm_fill;
							light_up_enable = 0;
							//dimming_enable = 0;
							dimming_exit = 1;
							prev_val = 0;
							//t_cnt = 0;
							//pwm_cnt = 0;
							//light_shut_down_enable = 1;
						}
					}


					if( ( message[1] & (1 << 0) ) ) {
						pwm_fill = 10;

					}

					else if( !(message[1] & (1 << 0) ) ) {
						pwm_fill = 6;
					} 
					
				}
	}

	/* Idle packet handler */
	else {
		asm volatile ( "nop" );
	}
}




/* 1 ms timer */
ISR(TIMER1_COMPA_vect) {

	/* Light UP routine */
	if( light_up_enable ) {

		if( pwm_cnt < light_up_fill )
			PORTB |= (1<< PINB4);
		else
			PORTB &= ~(1<<PINB4);

		if( light_up_fill >= pwm_fill) {
			light_up_enable = 0;
			light_up_fill = 0;
			dimming_enable = 1;
		}

		if(t_cnt > 20) {		//mo¿e zajsæ losowosæ przy starcie???
			light_up_fill++;
			t_cnt = 0;
		}
	}


	/* Light shut down routine */
	if( light_shut_down_enable ) {

		if( pwm_cnt >= light_down_fill )
			PORTB &= ~(1<<PINB4);
		else
			PORTB |= (1<< PINB4);

		if( light_down_fill < 1) {
			light_shut_down_enable = 0;
			light_down_fill = 0;
			PORTB &= ~(1<<PINB4);
		}

		if(t_cnt > 20) {
			light_down_fill--;
			t_cnt = 0;
		}
	}

	/* Dimming lights */
	if ( dimming_enable ) {

		if( pwm_cnt < pwm_fill )
			PORTB |= (1<< PINB4);
		else
			PORTB &= ~(1<<PINB4);
	}

	pwm_cnt++;
	if(pwm_cnt > 10) {

		if( dimming_exit ) {
			dimming_exit = 0;
			pwm_cnt = 0;
			dimming_enable = 0;
			light_shut_down_enable = 1;
		}

		pwm_cnt = 0;
	}	
		

	t_cnt++;
	//tick_count++;
	service_mode_tick++;
}
 


/* Bit decoding routine */
static void decode( uint8_t bit ) {

	static uint8_t bit_count, byte_count, recv_byte;
	static state t_state = DCC_WF_PREAMBLE;

	bit_count++;

	/* Wait for preamble to be completed */
	if( t_state == DCC_WF_PREAMBLE ) {

		if( bit ) {
			if( bit_count >= 10 )
				t_state = DCC_WF_ZERO;
		}
		else 
			bit_count = 0;
	}

	/* Wait for zero as a new byte starter */
	else if( t_state == DCC_WF_ZERO ) {

		if( bit ) {
		}
		else {
			bit_count = 0;
			byte_count = 0;
			recv_byte = 0;
			t_state = DCC_WF_BYTE;
		}
	}

	/* Wait for new data byte to be completed */
	else if( t_state == DCC_WF_BYTE ) {

		uint8_t temp_byte;
		temp_byte = ( recv_byte << 1 );

		if( bit ) {
			temp_byte |= 1;
		}

		recv_byte = temp_byte;

		if( bit_count == 8 ) {

			if( byte_count == MAX_MESSAGE ) {
				t_state = DCC_WF_PREAMBLE;
			}
			else {
				message[byte_count++] = recv_byte;
				t_state = DCC_WF_END;
			}
		}
	}

	/* Wait for ending 'one' */
	else if( t_state == DCC_WF_END ) {

		if( bit ) {
			t_state = DCC_WF_PREAMBLE;
			bit_count = 1;
			new_msg_size = byte_count;
			is_new_msg = 1;
		}
		else {
			t_state = DCC_WF_BYTE;
			bit_count = 0;
			recv_byte = 0;
		}
	}

	/* By default go to PREAMBLE state */
	else 
		t_state = DCC_WF_PREAMBLE;
}

/* Bit recognition interrupt routine */
ISR( INT0_vect ) {

	static int t_time;
	t_time = TCNT0;

	if( PIND & (1 << PIND2) ) {

		TCNT0 = 0;

	} else {

		if( ( t_time > 52 ) && ( t_time < 64 ) )
			decode( 1 );

		else if( t_time > 90 )
			decode( 0 );
	}
}
