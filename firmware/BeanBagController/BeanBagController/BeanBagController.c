/*
 * BeanBagController.c
 *
 * Created: 8/5/2014 6:49:43 PM
 *  Author: Ryan
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdbool.h>

//configurable parameters
#define MAIN_LOOP_PERIOD_MS				1

#define RX_WARMUP_MS					10		//milliseconds to wait for analog circuitry to settle

#define FLASH_CYCLES					10		//flash pattern when the beam is broken
#define FLASH_ON_TIME_MS				50
#define FLASH_OFF_TIME_MS				75
#define SLEEP_SIG_CYCLES				3		//flash pattern before going to sleep
#define SLEEP_SIG_ON_TIME_MS			750		
#define SLEEP_SIG_OFF_TIME_MS			750
#define STARTUP_SIG_CYCLES				5
#define STARTUP_SIG_DELAY_MS			75

#define EMITTER_PERIOD_US				100		//max = 255
#define EMITTER_WIDTH_US				10		//max = EMITTER_PERIOD_US

#define AMBIENT_LIGHT_SAMPLE_PERIOD_MS	10
#define AMBIENT_LIGHT_SCALE_FACTOR		256		//required for filter to work without floating point
#define AMBIENT_LIGHT_THRESH			500		//enable night mode when ambient "darkness" rises above this value (0 to 1023)
#define AMBIENT_LIGHT_HYST				200		//0 to 1023
#define AMBIENT_LIGHT_FILTER_ALPHA		255		//y[0] = (1-alpha)*x[0] + alpha*y[-1]

#define NIGHT_MODE_PWM_LEVEL			20		//PWM duty cycle out of 255
#define NIGHT_MODE_MIN_LEVEL			1
#define NIGHT_MODE_STEP					1
#define NIGHT_MODE_BREATH_PERIOD_MS		20
#define NIGHT_MODE_MAX_LEVEL			60

#define IDLE_TIME_TIL_SLEEP_SECS		1000
#define RECHECK_PERIOD_ON_WAKEUP_MS		10		//period to wait between checks in ms
#define RECHECK_COUNT_ON_WAKEUP			10		//number of times that a obstacle must be seen before waking up

#define DETECT_DELAY_MS					100

//fixed and derived parameters
#define F_CPU							1000000	//main clock frequency, F_CPU must be defined before including delay.h
#include <util/delay.h>
#define AMBIENT_LIGHT_MAIN_LOOP_PERIODS	(AMBIENT_LIGHT_SAMPLE_PERIOD_MS / MAIN_LOOP_PERIOD_MS)
#define NIGHT_MODE_BREATH_MAIN_LOOP_PERIODS	(NIGHT_MODE_BREATH_PERIOD_MS / MAIN_LOOP_PERIOD_MS)
#define WDT_TICK_PERIOD_SECS			8
#define IDLE_WDT_TICKS_TIL_SLEEP		(IDLE_TIME_TIL_SLEEP_SECS / WDT_TICK_PERIOD_SECS)

//PORTA pin definitions
#define RX1_MASK			(1<<PA0)
#define RX2_MASK			(1<<PA1)
#define RX3_MASK			(1<<PA2)
#define PHOTO_RES_MASK		(1<<PA3)
#define LED_STRIP_DRV_MASK	(1<<PA6)
#define EMITTER_DRV_MASK	(1<<PA7)
//PORTB pin definitions
#define BOOST_NSHTD_MASK	(1<<PB0)
#define RX_VDD_MASK			(1<<PB2)

uint32_t watchdog_ticks_since_flash = 0;		//8-sec ticks

bool bSleepPending = 0;
bool bNightModeEnabled = 0;
bool bIsNightModeLevelIncreasing = true;
volatile bool bFlashPending = 1;
uint8_t uiNightModeLevel = NIGHT_MODE_PWM_LEVEL;
uint8_t uiLastInputState = ( RX1_MASK | RX2_MASK | RX3_MASK );
volatile uint8_t uiTimeoutRemainingRx = 0;


uint32_t ambientLightLevel = 0;					//current ambient light level = (0 - 1023) * AMBIENT_LIGHT_SCALE_FACTOR
//uint32_t ambientLightWarmUpRemaining = AMBIENT_LIGHT_WARMUP_CYCLES;

//forward declarations
void flash_led_strip(void);
void flash_sleep_signal(void);
void flash_startup_sequence(void);
void prepare_to_sleep(bool);
void do_on_wakeup(void);
void configure_ddr(void);
void enable_active_mode(void);
void sample_ambient_light(void);
bool is_beam_blocked(void);
uint8_t count_set_bits(uint8_t, uint8_t);

void enable_flash_pwm(void);
void disable_flash_pwm(void);
void enable_emitter(void);
void disable_emitter(void);
void enable_rx_vdd(void);
void disable_rx_vdd(void);
void enable_boost(void);
void disable_boost(void);

int main(void)
{	
	uint32_t main_loop_iterations = 0;	//59-day rollover with a main loop period of 1 ms
	
	configure_ddr();
	PCMSK0 = (1<<PCINT0)|(1<<PCINT1)|(1<<PCINT2);
	enable_active_mode();
	WDTCSR = (1<<WDIE)|(1<<WDP3)|(1<<WDP0);	//enabled watchdog interrupt, prescaler = 1024, 128kHz clk = 8-sec tick	
	flash_startup_sequence();
	sei();
			
    while(1)
    {   
		_delay_ms(MAIN_LOOP_PERIOD_MS);		
		cli();
		if(uiTimeoutRemainingRx)
		{
			uiTimeoutRemainingRx--;
		}
		else if(~uiLastInputState & (PINA & ( RX1_MASK | RX2_MASK | RX3_MASK )))
		{			
			//check to see if blocked beam is now clear
			bFlashPending = 1;
			uiLastInputState = (PINA & ( RX1_MASK | RX2_MASK | RX3_MASK ));
		}
		
		if(bFlashPending)
		{	
			disable_emitter();
			flash_led_strip();
			enable_emitter();			
			GIFR = (1<<PCIF0);		//clear pin change flags triggered by flash			
			bFlashPending = 0;
		}
		sei();
		
		if(main_loop_iterations % AMBIENT_LIGHT_MAIN_LOOP_PERIODS == 0)
		{
			sample_ambient_light();
			if(bNightModeEnabled)
				enable_flash_pwm();				
			else
				disable_flash_pwm();				
		}
		
		if(bNightModeEnabled && main_loop_iterations % NIGHT_MODE_BREATH_MAIN_LOOP_PERIODS == 0)
		{
			if(bIsNightModeLevelIncreasing)
			{
				if(uiNightModeLevel + NIGHT_MODE_STEP <= NIGHT_MODE_MAX_LEVEL)				
					uiNightModeLevel += NIGHT_MODE_STEP;
				else
					bIsNightModeLevelIncreasing = false;				
			}
			else
			{
				if(uiNightModeLevel >= NIGHT_MODE_MIN_LEVEL + NIGHT_MODE_STEP)
					uiNightModeLevel -= NIGHT_MODE_STEP;
				else
					bIsNightModeLevelIncreasing = true;
			}
		}
				
		if(watchdog_ticks_since_flash > IDLE_WDT_TICKS_TIL_SLEEP)
		{			
			prepare_to_sleep(!bSleepPending);	//only flash sleep signal if there has been an event since the last time we woke up
			bSleepPending = 1;			
		}
						
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		cli();
		if (bSleepPending)
		{
			sleep_enable();
			sleep_bod_disable();
			//interrupts must be enabled before going to sleep or it will never come out
			sei();
			sleep_cpu();
			sleep_disable();
			do_on_wakeup();						
		}
		main_loop_iterations++;
		sei();
    }
}

void configure_ddr(void)
{
	DDRA = LED_STRIP_DRV_MASK | EMITTER_DRV_MASK;
	DDRB = BOOST_NSHTD_MASK | RX_VDD_MASK;
	//enable pull-ups
	PORTA = (RX1_MASK | RX2_MASK | RX3_MASK);
}

void flash_led_strip(void)
{
	watchdog_ticks_since_flash = 0;
	bSleepPending = 0;	
	disable_flash_pwm();
	for(uint8_t flashes = 0; flashes < FLASH_CYCLES; flashes++)
	{
		//OCR1A = 0;	//100% PWM
		PORTA |= LED_STRIP_DRV_MASK;
		_delay_ms(FLASH_ON_TIME_MS);
		PORTA &= ~LED_STRIP_DRV_MASK;
		//OCR1A = 200;
		_delay_ms(FLASH_OFF_TIME_MS);			
	}
	if(bNightModeEnabled)
	{
		enable_flash_pwm();
	}
	
}

void flash_sleep_signal(void)
{	
	for(uint8_t flashes = 0; flashes < SLEEP_SIG_CYCLES; flashes++)
	{		
		PORTA |= LED_STRIP_DRV_MASK;
		_delay_ms(SLEEP_SIG_ON_TIME_MS);
		PORTA &= ~LED_STRIP_DRV_MASK;		
		_delay_ms(SLEEP_SIG_OFF_TIME_MS);
	}
}

void flash_startup_sequence(void)
{
	for(uint8_t flashes = 0; flashes < STARTUP_SIG_CYCLES; flashes++)
	{
		disable_emitter();
		_delay_ms(STARTUP_SIG_DELAY_MS);
		enable_emitter();
		_delay_ms(STARTUP_SIG_DELAY_MS);
		PORTA |= LED_STRIP_DRV_MASK;
		_delay_ms(STARTUP_SIG_DELAY_MS);
		PORTA &= ~LED_STRIP_DRV_MASK;
		_delay_ms(STARTUP_SIG_DELAY_MS);			
	}

	for(uint8_t flashes = 0; flashes < STARTUP_SIG_CYCLES; flashes++)
	{		
		while(uiNightModeLevel + NIGHT_MODE_STEP <= NIGHT_MODE_MAX_LEVEL)
		{
			uiNightModeLevel += NIGHT_MODE_STEP;
			enable_flash_pwm();
			_delay_ms(NIGHT_MODE_BREATH_PERIOD_MS / 4);
		}
		while(uiNightModeLevel >= NIGHT_MODE_MIN_LEVEL + NIGHT_MODE_STEP)
		{
			uiNightModeLevel -= NIGHT_MODE_STEP;
			enable_flash_pwm();
			_delay_ms(NIGHT_MODE_BREATH_PERIOD_MS / 4);
		}
	}
	disable_flash_pwm();			
}

void enable_active_mode(void)
{
	enable_rx_vdd();
	enable_boost();
	//wait warm up time?	
	enable_emitter();
	GIMSK = (1<<PCIE0);
}

void prepare_to_sleep(bool bFlashSleepSignal)
{	
	ADCSRA = 0;		//disable ADC
	GIMSK = 0;		//disable pin-change interrupts
	GIFR = (1<<PCIF0);		//clear pin change flags trigger by flash
	//PORTA = 0x00;	//shut off LEDs
	disable_flash_pwm();
	disable_emitter();
	disable_rx_vdd();
	if(bFlashSleepSignal)
		flash_sleep_signal();
	disable_boost();
	
}

void do_on_wakeup(void)
{	
	bool bWakeup = true;
	enable_boost();
	enable_rx_vdd();
	enable_emitter();
	_delay_ms(RX_WARMUP_MS);
	
	for (uint8_t i = 0; bWakeup && i < RECHECK_COUNT_ON_WAKEUP; i++)	
	{
		bWakeup = is_beam_blocked();
		_delay_ms(RECHECK_PERIOD_ON_WAKEUP_MS);
	}
	
	if(bWakeup)
	{
		enable_active_mode();
		bFlashPending = true;
	}
	else
		bSleepPending = true;	
}

void enable_flash_pwm(void)
{
	//configure timer 1 for Fast PWM Mode, 8-bit
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0)| (0<<WGM11) | (1<<WGM10);
	//OCR1A = 0x080;
	OCR1A = uiNightModeLevel;
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 <<CS12) | (1 << CS11) | (1<<CS10);	//prescaler = 64
}

void disable_flash_pwm(void)
{
	TCCR1A = 0;		//disable PWM
	TCCR1B = 0;
}

void enable_emitter(void)
{	
	//100us period	
	OCR0A = EMITTER_PERIOD_US;
	//10us pulse width
	OCR0B = EMITTER_WIDTH_US;
	//fast PWM mode, set OC0B/P7/emitter clear on compare match, set at BOTTOM	
	TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00);
	//enable clock with no prescaling (1MHz)
	TCCR0B = (1 << WGM02)  | (0 << CS02) | (0 << CS01) | (1 << CS00);
	DDRB |= EMITTER_DRV_MASK;
}

void disable_emitter(void)
{
	TCCR0A = 0;
	TCCR0B = 0;
	DDRB &= ~(1 << EMITTER_DRV_MASK);
}

void enable_rx_vdd(void)
{
	PORTB |= RX_VDD_MASK;
}

void disable_rx_vdd(void)
{
	PORTB &= ~RX_VDD_MASK;
}

void enable_boost(void)
{
	PORTB |= BOOST_NSHTD_MASK;
}

void disable_boost(void)
{
	PORTB &= ~BOOST_NSHTD_MASK;
}

void sample_ambient_light(void)
{	
	uint32_t newValue;
	//use internal 1.1V ref, input = CH3
	ADMUX = (1 << REFS1) | (0 << REFS0) | (3 << MUX0);
	//free-running trigger
	ADCSRB = 0;
	//digital input disable
	DIDR0 = (1 << ADC3D);
	//enable ADC, start conversion, prescaler = 16, adcclk = 1 MHz / 8
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (4 << ADPS0);
	//wait for conversion to complete
	while(!(ADCSRA & (1<<ADIF)));
	newValue = ADC;
	ambientLightLevel = ((AMBIENT_LIGHT_SCALE_FACTOR - AMBIENT_LIGHT_FILTER_ALPHA) * (newValue * AMBIENT_LIGHT_SCALE_FACTOR) + AMBIENT_LIGHT_FILTER_ALPHA * ambientLightLevel) / AMBIENT_LIGHT_SCALE_FACTOR;
	bNightModeEnabled = (ambientLightLevel / AMBIENT_LIGHT_SCALE_FACTOR) > (bNightModeEnabled ? (AMBIENT_LIGHT_THRESH - AMBIENT_LIGHT_HYST) : AMBIENT_LIGHT_THRESH);	
}

bool is_beam_blocked(void)
{
	//input is low when blocked
	return (PINA & ( RX1_MASK | RX2_MASK | RX3_MASK )) !=  ( ( RX1_MASK | RX2_MASK | RX3_MASK ) );	
};

uint8_t count_set_bits(uint8_t byte, uint8_t max_bits)
{
	uint8_t bits_set = 0;
	for(uint8_t i = 0; i < max_bits; i++)
	{
		if(byte	& (1 << i))
			bits_set++;		
	}
}

ISR(PCINT0_vect)
{
	if(uiTimeoutRemainingRx == 0)
	{	
		uint8_t input_state = (PINA & ( RX1_MASK | RX2_MASK | RX3_MASK ));
		/*
		//check to see if blocked beam is now clear
		if (~uiLastInputState & input_state)
		{
			bFlashPending = 1;
		}
		*/
		//check if new beams have been blocked
		if(uiLastInputState & ~input_state)
		{
			uiTimeoutRemainingRx = DETECT_DELAY_MS;
			uiLastInputState = input_state;
		}		
	}

/*
	if(is_beam_blocked())
	{		
		bFlashPending = 1;
	}	
*/
}

ISR(WATCHDOG_vect)
{	
	watchdog_ticks_since_flash++;	
}

