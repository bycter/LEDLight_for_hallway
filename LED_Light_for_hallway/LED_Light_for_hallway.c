#define F_CPU 9600000L

#include <avr/io.h>
#include <util/delay.h>


enum DEV_MODE{
	CHECKING,
	GLOW,
	LIGHT,
	FADE	
	};

#define PWM_OUT PORTB0
#define PHOTO_SENSOR PORTB2
#define SENSOR1 PORTB3
#define SENSOR2 PORTB4

#define PHOTO_SENSOR_FLAG

#define PWM_MAX 200
#define PWM_MIN 30

/* 
T(s) = T_light(s) + T_fade(s)
T_light(s) = LIGHT_TIME * 1000 / PAUSE
T_fade(s) = PWM_MAX * PAUSE * 1000
*/
#define LIGHT_TIME 5 // in seconds
#define PAUSE 20 // in ms
#define DELAY_COUNTS LIGHT_TIME * 1000 / PAUSE

///Global variables
uint8_t g_state;
///uint8_t g_flags[5];


//Fuctions prototypes
void initMC(void);
void initPort(void);
uint8_t CheckPort(uint8_t);
void StartPWM(void);
void StopPWM(void);
void ResetDelay(void);


void initMC(void)
{
// Crystal Oscillator division factor: 1
CLKPR=0x80;
CLKPR=0x00;

// Timer/Counter 0 Stopped
// Interrupt disable
TIMSK0=0x00;
TCCR0A=0x00;
TCCR0B=0x00;
TCNT0=0x00;

// External Interrupt(s) initialization
// INT0: Off
GIMSK &= ~(1<<INT0); /// ISR INT0 Off
 


// Analog Comparator Off
ACSR=0x80;

// Watchdog Timer initialization
MCUSR = 1<<WDRF; // Clear WDRF in MCUSR
WDTCR |= (1<<WDCE) | (1<<WDE);
WDTCR = 0x00;
}

void initPort(void)
{
DDRB = (1<<DDB0); // 0b000001 PB5,1 - IN
PORTB = (1<<SENSOR1) | (1<<SENSOR2) | (1<<PHOTO_SENSOR); // PB2-PB4 - pull-up

asm("nop");
}

int main(void)
{
initMC();
initPort();

uint8_t pwm = 0;
unsigned short delay = 0;
g_state = CHECKING;
uint8_t pwm_state = 1;


//asm("sei");
while (1)
	{
	switch (g_state)
		{
		case CHECKING:
			if (!CheckPort(PHOTO_SENSOR)) 
				{
				if(CheckPort(SENSOR1) || CheckPort(SENSOR2))
					{
					g_state = GLOW;
					StartPWM(); 
					OCR0A = pwm = PWM_MIN;
					}
				}
		break;

		case GLOW:
			pwm++;
			//if (CheckPort(PHOTO_SENSOR)) g_state = FADE;
			if (pwm > PWM_MAX)
				{
				pwm = PWM_MAX;
				g_state = LIGHT;
				delay = 0;
				}
		break;

		case LIGHT:
			delay++;
			if (CheckPort(SENSOR1) || CheckPort(SENSOR2)) delay = 0; ///!!!!!!!!!!!!!!!!!
			//if (CheckPort(PHOTO_SENSOR)) g_state = FADE;
			if (delay > DELAY_COUNTS)
				{
				g_state = FADE;
				delay = 0;
				}
		break;

		case FADE:
			if (pwm_state) 
				{
				pwm--;
				pwm_state = 0;
				}
			else 
				{
				pwm_state = 1;
				if (!CheckPort(PHOTO_SENSOR))
					{
					if (CheckPort(SENSOR1) || CheckPort(SENSOR2)) 
						{
						g_state = GLOW;
						}
					}
				}

			if (pwm < PWM_MIN)
				{
				StopPWM();
				g_state = CHECKING;
				pwm = 0;
				}
		break;
		}
	OCR0A = pwm;
	_delay_ms(PAUSE);
	}
}


uint8_t CheckPort(uint8_t port)
{
uint8_t count = 0;
uint8_t i;
for (i = 0; i < 16; i++)
  {
  if ((PINB&(1<<port))==0) count++;
  }
if (count > 10) return 1;
else return 0;
}

void StartPWM(void) 
{
TCCR0A =  (1<<COM0A1)  | (1<<WGM01) | (1<<WGM00);
TCNT0 = 0;
OCR0A = 0;
TCCR0B |= (1<<CS00); 
}

void StopPWM(void)
{
TCCR0B=0x00;
TCCR0A=0x00; 
}

