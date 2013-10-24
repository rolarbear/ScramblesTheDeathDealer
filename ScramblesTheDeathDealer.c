/*
* ScramblesTheDeathDealer.c
*
*
*
* Created: 10/10/2013 12:46:50 AM
*  Author: poop
*/

#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>

void motorControl(int m1Duty, char m1Dir , int m2Duty, char m2Dir);
long map(long x, long in_min, long in_max, char out_min, char out_max);
void enableADC(void);
uint16_t readADC(uint8_t channel);
void readJoysticks (int* xVal, int* yVal, char* leftDir, char* rightDir);
long mapRev(long x, long in_min, long in_max, char out_min, char out_max);

const int dlyTime = 1000;
long adc0Rd, adc1Rd;
int m1,m2;
char m1d, m2d;

int main(void)
{
	//Enable PD0 and PD6 for direction and PB1 and PB2 for duty cycle
	DDRD |= (1 << DDD0)|(1 << DDD1)|(1 << DDD5)|(1 << DDD6);
	DDRC |= (1 << DDC0)|(1 << DDC1);
	
	enableADC();
	_delay_ms(10);
	
	while(1)
	{
		
		readJoysticks(&m1,&m2,&m1d,&m2d);
		
		motorControl(m1, m1d, m2, m2d);
	}
}

long map(long x, long in_min, long in_max, char out_min, char out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long mapRev(long x, long in_min, long in_max, char out_min, char out_max)
{
	return (x - in_min) * (out_min - out_max) / (in_max - in_min) + out_min;
}

void enableADC(void)
{
	//AVCC set to AREF voltage
	ADMUX |= (1 << REFS0);
	//128 prescaller and enable the ADC
	ADCSRA |= (1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0)|(1 << ADEN);
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
		PORTD &= ~_BV(PD0);
		break;
		case 1:
		PORTD |= _BV(PD0);
		default:
		break;
	}
	
	//Set direction for motor 2
	switch (m2Dir)
	{
		case 0:
		PORTD &= ~_BV(PD1);
		break;
		case 1:
		PORTD |= _BV(PD1);
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
	ch0Rd = readADC(0);
	_delay_ms(10);
	ch0Rd = readADC(0);
	_delay_ms(10);
	ch1Rd = readADC(1);
	_delay_ms(10);
	ch1Rd = readADC(1);
	_delay_ms(10);
	
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