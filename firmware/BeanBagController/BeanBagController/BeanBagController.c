/*
 * BeanBagController.c
 *
 * Created: 8/5/2014 6:49:43 PM
 *  Author: Ryan
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 1000000
#include <util/delay.h>

#define FLASH_CYCLES		15
#define FLASH_PERIOD_MS		100

#define FLASH_ON_TIME_MS	(FLASH_PERIOD_MS / 2)
#define FLASH_OFF_TIME_MS	FLASH_PERIOD_MS

uint8_t	led_low_level_pwm = 200;	//change this to set off to dim/completely off
uint32_t watchdog_ticks_since_flash = 0;		//8-sec ticks
uint8_t bLowPowerMode = 0;


//PB0 - NSHTD boost converter enable (active high)
//PB1 - NC
//PB2 - receiver power
//PB3 - reset pin

void flash_leds(void);
void prepare_to_sleep(void);
void do_on_wakeup(void);
void enable_active_mode(void);
void enable_emitter(void);


int main(void)
{
	
	DDRA = (1<<PA6)| (1<<PA2) | (1<<PA7);
	PORTA = (1<<PA2);		
	
	//enable boost converter
	DDRB = (1<<PB0);
	PORTB = (1<<PB0);
		
	PCMSK0 = (1<<PCINT0)|(1<<PCINT1);
	enable_active_mode();
	WDTCSR = (1<<WDIE)|(1<<WDP3)|(1<<WDP0);	//enabled watchdog interrupt, prescaler = 1024, 128kHz clk = 8-sec tick
	sei();
		
    while(1)
    {   		
		if(watchdog_ticks_since_flash > 1)
		{
			bLowPowerMode = 1;
			prepare_to_sleep();
		}
		else
		{			
			enable_pwm();
		}
		
		set_sleep_mode(SLEEP_MODE_IDLE);
		cli();
		if (bLowPowerMode)
		{
			sleep_enable();
			sleep_bod_disable();
			sei();
			sleep_cpu();
			sleep_disable();
			do_on_wakeup();						
		}
		sei();
    }
}

void flash_leds(void)
{
	watchdog_ticks_since_flash = 0;
	bLowPowerMode = 0;
	for(uint8_t flashes = 0; flashes < FLASH_CYCLES; flashes++)
	{
		OCR1A = 0;	//100% PWM
		_delay_ms(FLASH_ON_TIME_MS);
		OCR1A = 200;
		_delay_ms(FLASH_OFF_TIME_MS);			
	}	
}

void enable_active_mode(void)
{
	//clear flags?
	enable_emitter();
	enable_pwm();
	GIMSK = (1<<PCIE0);
}

void prepare_to_sleep(void)
{
	GIMSK = 0;		//disable pin-change interrupts
	TCCR1A = 0;		//disable PWM
	TCCR1B = 0;
	PORTA = 0x00;	//shut off LEDs
}

void do_on_wakeup(void)
{
	PORTA = (1<<PA2);
	_delay_ms(1);
	if( (PINA & ( (1<<PINA0) | (1<<PINA1) )) !=  ( (1<<PINA0) | (1<<PINA1) ) )
	{
		enable_active_mode();
		flash_leds();
	}	
}

void enable_pwm(void)
{
	//configure timer 1 for Fast PWM Mode (Mode 15)
	TCCR1A = (1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (0 << COM1B0)| (0<<WGM11) | (1<<WGM10);
	OCR1A = 0x080;
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 <<CS12) | (1 << CS11) | (1<<CS10);	//prescaler = 64
}

void enable_emitter(void)
{	
	//100us period	
	OCR0A = 100;
	//10us pulse width
	OCR0B = 10;
	//fast PWM mode, set OC0B/P7/emitter clear on compare match, set at BOTTOM	
	TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00);
	//enable clock with no prescaling (1MHz)
	TCCR0B = (1 << WGM02)  | (0 << CS02) | (0 << CS01) | (1 << CS00);
}

ISR(PCINT0_vect)
{
	if( (PINA & ( (1<<PINA0) | (1<<PINA1) )) !=  ( (1<<PINA0) | (1<<PINA1) ) )
	{
		enable_pwm();
		flash_leds();				
	}	
}

ISR(WATCHDOG_vect)
{
	//PINA = (1<<PINA6);
	watchdog_ticks_since_flash++;
}

