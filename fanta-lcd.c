/**** 
FANTACHIP LCD CONTROLLER 16x2
Code designed for atmega8-16PU

- Easy to use string command interface
- Connects to your microcontroller over serial interface
- Offload the LCD interface work from your main controller to this one

Design notes: 
- Fast interrupt-driven state machine architecture

License: GPL

How to use: 
- Upload the firmware to an ATMega8 chip
- Connect the lcd as specified below
- Use the TX/RX lines to send data to the controller. 

Pinout diagram of the flashed atmega: 

           ATMega8
				-----------
 Reset -|1      28|-
   RXD -|         |-
   TXD -|         |- D7
       -|         |- D6  
       -|         |- D5
       -|         |- D4
   VCC -|         |-
   GND -|         |-
 XTAL1 -|         |-
 XTAL2 -|         |-
    RW -|         |-
    RS -|         |-
     E -|14     15|-
        -----------

Issues: 
- If you want to use higher baud rate, use an external crystal
  with 1Mhz internal clock it's difficult to keep higher baud rates accurate.
   - you will need to update the F_CPU variable accordingly. 
 
**/

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#define LCD_CMD_PORT PORTD
#define LCD_CMD_DDR DDRD
#define LCD_RW_PIN 	PD5
#define LCD_RS_PIN 	PD6
#define LCD_E_PIN 	PD7

#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR 	DDRC
#define LCD_DATA_IN		PINC
#define LCD_D7_PIN 	PC3
#define LCD_D6_PIN 	PC2
#define LCD_D5_PIN 	PC1
#define LCD_D4_PIN 	PC0

#define LCD_DATA_PINS (_BV(LCD_D7_PIN) | _BV(LCD_D6_PIN) | _BV(LCD_D5_PIN) | _BV(LCD_D4_PIN))
#define LCD_CTRL_PINS (_BV(LCD_RW_PIN) | _BV(LCD_RS_PIN))

#define LCD_READ_NIBBLE() \
			(((LCD_DATA_IN & _BV(LCD_D7_PIN)) << (7 - LCD_D7_PIN) | \
			(LCD_DATA_IN & _BV(LCD_D6_PIN)) << (7 - LCD_D6_PIN) | \
			(LCD_DATA_IN & _BV(LCD_D5_PIN)) << (7 - LCD_D5_PIN) | \
			(LCD_DATA_IN & _BV(LCD_D4_PIN)) << (7 - LCD_D4_PIN)) & 0x0f)
#define LCD_PLACE_NIBBLE(n) \
			(LCD_DATA_PORT = (LCD_DATA_PORT & ~LCD_DATA_PINS) | \
			(((n & 0x0f) >> 3) << 	LCD_D7_PIN) | \
			(((n & 0x0f) >> 2) << 	LCD_D6_PIN) | \
			(((n & 0x0f) >> 1) << 	LCD_D5_PIN) | \
			(((n & 0x0f)) 		 << 	LCD_D4_PIN))
			
#define CLR_BIT(r, b) r &= ~(1 << b)
#define SET_BIT(r, b) r |= (1 << b)

#define LCD_CLOCK_HIGH() SET_BIT(LCD_CMD_PORT, LCD_E_PIN)
#define LCD_CLOCK_LOW() CLR_BIT(LCD_CMD_PORT, LCD_E_PIN)
#define LCD_SET_RW() 		SET_BIT(LCD_CMD_PORT, LCD_RW_PIN)
#define LCD_CLEAR_RW() 	CLR_BIT(LCD_CMD_PORT, LCD_RW_PIN)
#define LCD_SET_RS() 		SET_BIT(LCD_CMD_PORT, LCD_RS_PIN)
#define LCD_CLEAR_RS() 	CLR_BIT(LCD_CMD_PORT, LCD_RS_PIN)

#define STATE_TRANSMIT 		0
#define STATE_WAIT_READY 	4
#define STATE_READY 			10

static volatile uint16_t LCDDR = 0;
static volatile uint8_t RTMP = 0;
static volatile uint8_t LCDSR = 0; 
static volatile uint8_t DRVSR = STATE_READY; // bit 3,2,1,0 = step, bit 4 readstatus

#define DRV_STATE() (DRVSR & 0x0f)
#define DRV_STATE_TRANS(s) (DRVSR = (DRVSR & 0xf0) | (s & 0x0f))

// lcd status register bits
#define LCDBSY 8

#define LCD_DATA 0x200

inline static void LCD_SEND(uint16_t b) {
	while(DRV_STATE() != STATE_READY);
	SET_BIT(TIFR, TOV0);
	TCNT0 = 0;
	LCDDR = b;
	DRV_STATE_TRANS(STATE_TRANSMIT);
	return;
}

ISR(TIMER0_OVF_vect){
	if(DRV_STATE() == STATE_TRANSMIT){
		if(LCDDR & 0x200) LCD_SET_RS();
		else LCD_CLEAR_RS();
		if(LCDDR & 0x100) LCD_SET_RW();
		else LCD_CLEAR_RW();
		
		LCD_CLOCK_HIGH();
		LCD_PLACE_NIBBLE(LCDDR >> 4);
		DRV_STATE_TRANS(1);
	} else if(DRV_STATE() == 1){
		LCD_CLOCK_LOW();
		DRV_STATE_TRANS(2);
	} else if(DRV_STATE() == 2){
		LCD_CLOCK_HIGH();
		LCD_PLACE_NIBBLE(LCDDR);
		DRV_STATE_TRANS(3);
	} else if(DRV_STATE() == 3){
		LCD_CLOCK_LOW();
		DRV_STATE_TRANS(STATE_WAIT_READY);
	} else if(DRV_STATE() == STATE_WAIT_READY){ // now we need to loop until the LCD is ready 
		LCD_CMD_DDR &= ~LCD_DATA_PINS; // put data pins into input mode
		LCD_CLEAR_RS();
		LCD_SET_RW();
		LCD_CLOCK_HIGH();
		DRV_STATE_TRANS(5);
	} else if (DRV_STATE() == 5){
		RTMP = LCD_READ_NIBBLE() << 4;
		DRV_STATE_TRANS(6);
	} else if(DRV_STATE() == 6) {
		LCD_CLOCK_LOW();
		DRV_STATE_TRANS(7);
	} else if(DRV_STATE() == 7){
		LCD_CLOCK_HIGH();
		DRV_STATE_TRANS(8);
	} else if(DRV_STATE() == 8){
		DRVSR = RTMP | LCD_READ_NIBBLE();
		if(DRVSR & _BV(LCDBSY))
			DRV_STATE_TRANS(STATE_WAIT_READY); // if display is not ready then keep reading status until it is ready
		else
			DRV_STATE_TRANS(9); // otherwise we transfer into the ready state
		LCD_CLOCK_LOW();
	} else if(DRV_STATE() == 9){ // we are ready
		LCD_CMD_DDR 	|= LCD_DATA_PINS; 				// set data bus to outputs
		LCD_CMD_PORT 	&= ~LCD_CTRL_PINS; // clear rs/rw 
		DRV_STATE_TRANS(STATE_READY);
	}
}

#define UART_TX_READY() (UCSRA & (1<<UDRE))
#define UART_RX_READY() (UCSRA & (1<<RXC))
#define BAUD 2400UL

static volatile uint8_t RXSR = 0; 

// fired whenever we receive a byte
ISR(USART_RXC_vect){
 // not used at the moment
}

int main(){
	// setup timer
	TCCR0 = _BV(CS00);
  SET_BIT(TIMSK, TOIE0);
  
	UBRRL = (uint8_t)((F_CPU / (16UL * BAUD)) - 1) ;
	UBRRH = (uint8_t)(((F_CPU / (16UL * BAUD)) - 1 )>>8);
	UCSRC=(1<<URSEL)|(3<<UCSZ0); // Async, 8 N 1
	UCSRB	=(1<<RXEN)|(1<<TXEN);
  UCSRB |= (1 << RXCIE); 
  
	_delay_ms(30); // allow lcd to initialize
	
	//setup IO ports
	LCD_CMD_DDR 	= LCD_CTRL_PINS | _BV(LCD_E_PIN); // set ctrl pins as outputs
	LCD_DATA_DDR 	= LCD_DATA_PINS; 

	LCD_CMD_PORT 	&= ~(LCD_CTRL_PINS | _BV(LCD_E_PIN)); // set all ctrl lines to zero
	LCD_DATA_PORT &= ~LCD_DATA_PINS; // set all data lines to zero
	
	//allow all lines to stabilize
	_delay_us(0.3);	

	// LCD init sequence for 4 bit mode
	LCD_CLOCK_HIGH();
	LCD_DATA_PORT = 0x2; 
	_delay_us(1);
	LCD_CLOCK_LOW();
	_delay_us(1);
	
	// extra cycle to even out E cycles
	LCD_CLOCK_HIGH();
	_delay_us(1);
	LCD_CLOCK_LOW();
	_delay_us(1);
	
	// go into status loop state to wait until LCD is ready
	DRV_STATE_TRANS(STATE_WAIT_READY);
	
	sei(); // the loop will start running after this
	
	// setup the lcd 
	LCD_SEND(0b00001111);	//Display On, blink, U-line
	LCD_SEND(0b00101100);			//function set 4-bit,2 line 5x7 dot format
	LCD_SEND(0x01); // clear
	LCD_SEND(0x80); // position 0
	
	uint64_t c = 0;
	uint8_t x_pos = 0; 
	
	uint8_t state = 0;
	enum state_t{
		STATE_START = 0,
		STATE_COMMAND_START,
		STATE_GOTO_1,
		STATE_GOTO_2,
		STATE_STORE_1,
		STATE_STORE_2,
		STATE_RECALL_1,
		STATE_CURSOR
	}; 
	uint16_t RTMP = 0; 
	
	#define STORE_BUF_SIZE 9
	#define STORE_BUF_WIDTH 40
	
	char stored_strings[STORE_BUF_SIZE][STORE_BUF_WIDTH];
	uint8_t store_adr = 0, store_adr_x = 0;
	uint8_t line = 0; 
	
	while(1){
		if(!UART_RX_READY()) continue; 
		uint8_t b = UDR;
		if(b == '\\')
			state = STATE_COMMAND_START; 
		if(state == STATE_START){ // normal forwarding
			if(b == '\r' || b == '\n') {
				//LCD_SEND(0x80 | (line << 6)); // goto 0
				line ^= (1 << 6);
				while(!UART_TX_READY());
				UDR = '\r';
				while(!UART_TX_READY());
				UDR = '\n';
			} else {
				LCD_SEND(LCD_DATA | b);
				x_pos++;
				UDR = b; // echo byte
			}
			if(x_pos > 16){
				//LCD_SEND(0x18); // shift
			}
			if(x_pos == 39){
				//LCD_SEND(0xc0);
				x_pos = 0;
			}
		} else if(state == STATE_COMMAND_START){ // command mode
			if(b == 'c') {
				LCD_SEND(0x01); // clear
				LCD_SEND(0x02);
				state = STATE_START;
				x_pos = 0;
			} else if(b == 'g'){ // goto
				state = STATE_GOTO_1; 
			} else if(b == 's'){ // store a string
				state = STATE_STORE_1; 
			} else if(b == 'r'){
				state = STATE_RECALL_1; 
			} else if(b == 'x'){
				state = STATE_CURSOR; 
			} else if(b == 'h'){
				LCD_SEND(0x18); // shift left
			} else if(b == 'l'){
				LCD_SEND(0x1c); // shift right
			}
		} else if(state == STATE_GOTO_1){
			RTMP = ((uint16_t) b) << 8;
			state = STATE_GOTO_2;
		} else if(state == STATE_GOTO_2){
			RTMP |= b;
			uint8_t line = ((RTMP & 0xff00) >> 8) % 2;
			uint8_t col = (RTMP & 0xff);
			LCD_SEND(0x02);
			LCD_SEND(0x80 | ((line * 0x40) + col));
			state = STATE_START;
		} else if(state == STATE_STORE_1){
			store_adr = b % STORE_BUF_SIZE;
			store_adr_x = 0;
			state = STATE_STORE_2; 
		} else if(state == STATE_STORE_2){
			stored_strings[store_adr][store_adr_x++] = b;
			if(b == 0x01 || b == 0x00 || store_adr_x == STORE_BUF_WIDTH){
				stored_strings[store_adr][store_adr_x-1] = 0;
				state = STATE_START;
			}
		} else if(state == STATE_RECALL_1){
			if(b < STORE_BUF_SIZE && b > 0){
				char *ch = stored_strings[b];
				while(*ch && (ch - stored_strings[b]) < STORE_BUF_WIDTH){
					LCD_SEND(LCD_DATA | *ch);
					ch++;
				}
			}
			state = STATE_START;
		} else if(state == STATE_CURSOR){
			if(b == '0')
				LCD_SEND(0xc); // cursor off
			else if(b == '1')
				LCD_SEND(0xc | 1); // blink on
			else if(b == '2')
				LCD_SEND(0xc | 2); // blink and cursor on
			else if(b == '3')
				LCD_SEND(0xc | 3); // only cursor on
			state = STATE_START;
		}
		c++;
	}
}
