/*
* ScramblesTheDeathDealer.c
*
*	PORTB
*	0
*	1	CE pin on NRF24L01+
*	2	SS(CS) pin on NRF24L01+
*	3	MOSI pin on NRF24L01+
*	4	MISO pin on NRF24L01+
*	5	SCK pin on NRF24L01+
*	6
*	7
*
*	PORTC
*	0	Motor analog stick X
*	1	Motor analog stick Y
*	2	(Servo control stick X)
*	3	(Servo control stick Y)
*	4
*	5
*	6	RESET
*
*	PORTD
*	0	DEBUG RX
*	1	DEBUT TX
*	2	Left motor direction
*	3	Right motor direction
*	4
*	5	Left motor PWM
*	6	Right motor PWM
*	7
*
* Created: 10/10/2013 12:46:50 AM
*  Author: poop
*/

/*
*STILL TODO:
*  SCRAMBLES:
*	MAKE FUNCTIONS FOR DRIVING
*	SERVO CONTROLS
*	NRF24L01 COMMUNICATION
*	MAKE CIRCUIT BOARD
*	???
*
*  CONTROL:
*	NRF24L01 COMMUNICATION
*	HACK OPEN CHEAP CONTROLLER
*	MAKE CIRCUIT BOARD
*
*
*/

#define F_CPU 16000000
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((16000000 / (USART_BAUDRATE * 16UL)))-1)

#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01.h"

void motorControl(int m1Duty, char m1Dir , int m2Duty, char m2Dir);
long map(long x, long in_min, long in_max, int out_min, int out_max);
void enableADC(void);
uint16_t readADC(uint8_t channel);
void readJoysticks (int* xVal, int* yVal, char* leftDir, char* rightDir);
long mapRev(long x, long in_min, long in_max, int out_min, int out_max);
void startADC(void);
void stopADC(void);
void driveForward(int dutym1, int dutym2);
void driveBackward(int dutym1, int dutym2);
void turnLeft(int dutym1, int dutym2);
void turnRight(int dutym1, int dutym2);
void setupSPI(void);
void enableSPI(void);
void disableSPI(void);
char write_read_SPI(char cData);
uint8_t GetReg(uint8_t reg);
void writeReg(uint8_t reg, uint8_t val);

volatile long adc0Rd, adc1Rd;
int m1,m2;
char m1d, m2d;
int diagPrint = 1;

int main(void)
{
	//Setup SPI pins PB1 CE, PB2 SS(CS) OUTPUT, PB3 MOSI OUTPUT, PB4 MISO INPUT, PB5 SCK OUTPUT
	DDRB |= (1<<DDB1)|(1<<DDB2)|(1 << DDB3)|(1 << DDB5);
	//	DDRB &= ~(1 << DDB4);
	//Enable PD2 and PD3 for direction and PD5 and PD6 for PWM
	DDRD |= (1 << DDD2)|(1 << DDD3)|(1 << DDD5)|(1 << DDD6);
	//Enable PC0 for motor analog stick X and PC1 for motor analog stick Y
	DDRC |= (1 << DDC0)|(1 << DDC1);
	
	UBRR0L = BAUD_PRESCALE;
	UBRR0H = (BAUD_PRESCALE >> 8);
	//enable transmit and receive lines
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	//8 data bits, 1 stop bits
	UCSR0C = (3 << UCSZ00);
	
	while ((UCSR0A & (1 << UDRE0)) == 0) {};
	UDR0 = ' ';
	while ((UCSR0A & (1 << UDRE0)) == 0) {};

	setupSPI();

	//enableADC();
	_delay_ms(10);

	while (1)
	{
		uint8_t stat = GetReg(STATUS);
		if(diagPrint == 1)
		{
			while ((UCSR0A & (1 << UDRE0)) == 0) {};
			UDR0 = stat;
			while ((UCSR0A & (1 << UDRE0)) == 0) {};
		}
		while(stat != 0x0E){};
		writeReg(RF_CH,0x07);
		while ((UCSR0A & (1 << UDRE0)) == 0) {};
		UDR0 = (GetReg(RF_CH));
		while ((UCSR0A & (1 << UDRE0)) == 0) {};
		writeReg(RF_SETUP, 0x06);
		while ((UCSR0A & (1 << UDRE0)) == 0) {};
		UDR0 = (GetReg(RF_SETUP));
		while ((UCSR0A & (1 << UDRE0)) == 0) {};
		_delay_ms(50);
	}
	//while(1)
	//{
	//	readJoysticks(&m1,&m2,&m1d,&m2d);
	//	motorControl(m1, m1d, m2, m2d);
	//}
}

uint8_t GetReg(uint8_t reg)
{
	//CSN low
	PORTB &= ~(1 << DDB2);
	//Set R_Register
	write_read_SPI(R_REGISTER + reg);
	//Send dummy receive register value
	reg = write_read_SPI(NOP);
	//CSN high
	PORTB |= (1 << DDB2);
	//return registry value that was read
	return reg;
}

void writeReg(uint8_t reg, uint8_t val)
{
	//CSN low
	PORTB &= ~(1 << DDB2);
	
	//Write register value
	SPDR = reg;
	//Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	
	//Write Value to go into register
	SPDR = val;
	//Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	
	//CSN high
	PORTB |= (1 << DDB2);
}

void setupSPI(void)
{
	//Enable SPI, Master, set clock rate
	SPCR |= (1<<SPE)|(1<<MSTR);
	
	//SETBIT(PORTB, 2);
	PORTB |= (1<<DDB2);
	//CLEARBIT(PORTB, 1);
	if (PORTB & (1 << DDB1))
	{
		PORTB &= ~(1<<DDB1);
	}
}

void enableSPI(void)
{
	SPCR |= (1<<SPE);
}

void disableSPI(void)
{
	SPCR &= ~(1<<SPE);
}

char write_read_SPI(char cData)
{
	//Start transmission
	SPDR = cData;
	//Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	
	return SPDR;
}

long map(long x, long in_min, long in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long mapRev(long x, long in_min, long in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_min - out_max) / (in_max - in_min) + out_min;
}

void enableADC(void)
{
	//AVCC set to AREF voltage
	ADMUX |= (1 << REFS0)|(1 << REFS1);
	//Disable digital inputs on ADC lines
	DIDR0 |= (1 << ADC0D)|(1 << ADC1D)|(1 << ADC2D)|(1 << ADC3D)|(1 << ADC4D)|(1 << ADC5D);
	//128 prescaller and enable the ADC
	ADCSRA |= (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0)|(1 << ADEN);
}

void startADC(void)
{
	ADCSRA |= (1 << ADEN);
}

void stopADC(void)
{
	ADCSRA &= ~(1 << ADEN);
}

//channel 0 - 8
uint16_t readADC(uint8_t channel)
{
	//case statement for selecting the channel
	//This sets up ADMUX registers
	switch (channel)
	{
		case 0:
		if(ADMUX & (1 << MUX0))
		{
			ADMUX &= ~(1 << MUX0);
		}
		if(ADMUX & (1 << MUX1))
		{
			ADMUX &= ~(1 << MUX1);
		}
		if(ADMUX & (1 << MUX2))
		{
			ADMUX &= ~(1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 1:
		if(ADMUX & ~(1 << MUX0))
		{
			ADMUX |= (1 << MUX0);
		}
		if(ADMUX & (1 << MUX1))
		{
			ADMUX &= ~(1 << MUX1);
		}
		if(ADMUX & (1 << MUX2))
		{
			ADMUX &= ~(1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 2:
		if(ADMUX & (1 << MUX0))
		{
			ADMUX &= ~(1 << MUX0);
		}
		if(ADMUX & ~(1 << MUX1))
		{
			ADMUX |= (1 << MUX1);
		}
		if(ADMUX & (1 << MUX2))
		{
			ADMUX &= ~(1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 3:
		if(ADMUX & ~(1 << MUX0))
		{
			ADMUX |= (1 << MUX0);
		}
		if(ADMUX & ~(1 << MUX1))
		{
			ADMUX |= (1 << MUX1);
		}
		if(ADMUX & (1 << MUX2))
		{
			ADMUX &= ~(1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 4:
		if(ADMUX & (1 << MUX0))
		{
			ADMUX &= ~(1 << MUX0);
		}
		if(ADMUX & (1 << MUX1))
		{
			ADMUX &= ~(1 << MUX1);
		}
		if(ADMUX & ~(1 << MUX2))
		{
			ADMUX |= (1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 5:
		if(ADMUX & ~(1 << MUX0))
		{
			ADMUX |= (1 << MUX0);
		}
		if(ADMUX & (1 << MUX1))
		{
			ADMUX &= ~(1 << MUX1);
		}
		if(ADMUX & ~(1 << MUX2))
		{
			ADMUX |= (1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 6:
		if(ADMUX & (1 << MUX0))
		{
			ADMUX &= ~(1 << MUX0);
		}
		if(ADMUX & ~(1 << MUX1))
		{
			ADMUX |= (1 << MUX1);
		}
		if(ADMUX & ~(1 << MUX2))
		{
			ADMUX |= (1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 7:
		if(ADMUX & ~(1 << MUX0))
		{
			ADMUX |= (1 << MUX0);
		}
		if(ADMUX & ~(1 << MUX1))
		{
			ADMUX |= (1 << MUX1);
		}
		if(ADMUX & ~(1 << MUX2))
		{
			ADMUX |= (1 << MUX2);
		}
		if(ADMUX & (1 << MUX3))
		{
			ADMUX &= ~(1 << MUX3);
		}
		break;
		
		case 8:
		if(ADMUX & (1 << MUX0))
		{
			ADMUX &= ~(1 << MUX0);
		}
		if(ADMUX & (1 << MUX1))
		{
			ADMUX &= ~(1 << MUX1);
		}
		if(ADMUX & (1 << MUX2))
		{
			ADMUX &= ~(1 << MUX2);
		}
		if(ADMUX & ~(1 << MUX3))
		{
			ADMUX |= (1 << MUX3);
		}
		break;
		
		default:
		break;
	}
	
	//Start conversion
	ADCSRA |= (1 << ADSC);
	//WAIT FOR IT!!!
	while(ADCSRA & (1 << ADSC));
	//BOOM!
	return ADC;
}

//m1Duty and m2Duty should be between 0 and 255
void motorControl(int m1Duty, char m1Dir , int m2Duty, char m2Dir)
{
	//Set PWM value
	OCR0A = (m1Duty);
	OCR0B = (m2Duty);
	
	//setup pwm
	TCCR0A |= (1 << COM0A1)|(1 << COM0B1);
	TCCR0A |= (1 << WGM01)|(1 << WGM00);
	
	//Start that SHIT!
	TCCR0B |= (1 << CS00)|(1 << CS02);
	
	//Set direction for motor 1
	switch (m1Dir)
	{
		case 0:
		PORTD &= ~_BV(PD2);
		break;
		case 1:
		PORTD |= _BV(PD2);
		default:
		break;
	}
	
	//Set direction for motor 2
	switch (m2Dir)
	{
		case 0:
		PORTD &= ~_BV(PD3);
		break;
		case 1:
		PORTD |= _BV(PD3);
		break;
		default:
		break;
	}
}


void readJoysticks (int* xVal, int* yVal, char* leftDir, char* rightDir)
{
	//variables
	long ch0Rd, ch1Rd;
	
	//read joystick channels, they need to be read twice
	//with delay because the ADC MUX isn't good for high speed
	startADC();
	ch0Rd = readADC(0);
	stopADC();
	_delay_ms(10);
	//	ch0Rd = readADC(0);
	//	_delay_ms(10);
	startADC();
	ch1Rd = readADC(1);
	stopADC();
	_delay_ms(10);
	//	ch1Rd = readADC(1);
	//	_delay_ms(10);
	
	//map the adc reads and determine direction
	if (ch0Rd < 512)
	{
		*xVal = mapRev(ch0Rd, 0, 511, 0, 255);
		*leftDir = 0;
	}
	else
	{
		*xVal = map(ch0Rd, 512, 1024, 0, 255);
		*leftDir = 1;
	}
	
	if (ch1Rd < 512)
	{
		*yVal = mapRev(ch1Rd, 0, 511, 0, 255);
		*rightDir = 0;
	}
	else
	{
		*yVal = map(ch1Rd, 512, 1024, 0, 255);
		*rightDir = 1;
	}
}

void driveForward(int dutym1, int dutym2)
{
	char dir1 = 1;
	char dir2 = 1;
	motorControl(dutym1, dir1, dutym2, dir2);
}

void driveBackward(int dutym1, int dutym2)
{
	char dir1 = 0;
	char dir2 = 0;
	motorControl(dutym1, dir1, dutym2, dir2);
}

void turnLeft(int dutym1, int dutym2)
{
	char dir1 = 0;
	char dir2 = 1;
	motorControl(dutym1, dir1, dutym2, dir2);
}

void turnRight(int dutym1, int dutym2)
{
	char dir1 = 1;
	char dir2 = 0;
	motorControl(dutym1, dir1, dutym2, dir2);
}