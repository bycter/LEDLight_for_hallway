/*
 * LED_Light_for_hallway.c
 *
 * Created: 13.01.2017
 *  Author: bycter
 */ 


#define F_CPU 9600000L

#include "bits_macros.h"
#include <avr/io.h>
#include <util/delay.h>


enum DEV_MODE{
	CHECKING,
	GLOW,
	LIGHT,
	FADE
};

#define PWM_OUT          PORTB0
#define PHOTO_SENSOR_LED PORTB1
#define PHOTO_SENSOR     PORTB2
#define SENSOR1          PORTB3
#define SENSOR2          PORTB4

#define PWM_MAX          200
#define PWM_MIN          30

// States of devices
#define ON			         1
#define OFF			         0

/*
T(s) = T_light(s) + T_fade(s)
T_light(s) = LIGHT_TIME * 1000 / PAUSE
T_fade(s) = PWM_MAX * PAUSE * 1000
*/
#define LIGHT_TIME 5 // in seconds
#define PAUSE 20 // in ms
#define DELAY_COUNTS LIGHT_TIME * 1000 / PAUSE

#if DELAY_COUNTS > 255
	#error "DELAY_COUNTS must be less than 256! Check values of LIGHT_TIME and PAUSE."
#endif

///Global variables
uint8_t g_state;

//Fuctions prototypes
void initMC(void);
uint8_t CheckPort(uint8_t);
//uint8_t CheckPhotoSensor(void);
void StartPWM(void);
void StopPWM(void);

void initMC(){
  
	/*	;=======================================================================
		�������������� ������� �����
                                          DEFINE            DDR   PORT
			PB5	(RESET/ADC0/dW/PCINT5)          ---               0     0
			PB4	(ADC2/PCINT4)                   SENSOR2           IN    pull-up
			PB3	(ADC3/CLKI/PCINT3)              SENSOR1           IN    pull-up
			PB2	(SCK/ADC1/T0/PCINT2)            PHOTO_SENSOR      IN    hi-z
			PB1	(MISO/AIN1/OC0B/INT0/PCINT1)    PHOTO_SENSOR_LED  OUT   1
			PB0	(MOSI/AIN0/OC0A/PCINT0)         PWM_OUT           OUT   0
	*/
  DDRB =	(0<<DDB4) |		(0<<DDB3) |		(0<<DDB2) |		(1<<DDB1) |		(1<<DDB0);
  PORTB =	(1<<PORTB4) |	(1<<PORTB3) |	(0<<PORTB2) |	(1<<PORTB1) |	(0<<PORTB0);

  asm("nop");

  //  ���������� ��� �������� �������
  TCCR0A =	0x00;
	TCCR0B =	0x00;
  TCNT0 =   0x00;
  OCR0A =   0x00;
  
  /************************************************************************/
  /*  ���������� ����������                                               */
  /************************************************************************/
  //  ����������� � AIN0 ����������� ��� = 1.1V
  ACSR = (0<<ACD) | (1<<ACBG) | (0<<ACI) | (0<<ACIE) | (0<<ACIS1) | (0<<ACIS0);
	//ACSR = (1<<ACD);
	ADCSRA &= ~(0<<ADEN); // ��������� ���
	ADCSRB |= (1<<ACME);	//����������� �������������� ��� � ����������� 
	ADMUX = (0<<MUX1) | (1<<MUX0); // �������������� ����� AIN1 -> ADC1 (PB1 -> PB2)

}

int main(void)
{
	initMC();

	uint8_t pwm = 0;
	unsigned short delay = 0;
	g_state = CHECKING;

	uint8_t counter = 0;  
  // Variable pwm_state allow to do FADE time twice long than GLOW time
	uint8_t pwm_state = ON;
  volatile int photoSensorState = 0;

	//asm("sei");
	while (1)
	{
		//SetBitVal(PORTB, PHOTO_SENSOR_LED, BitIsSet(ACSR, ACO));
    if (counter == 250)
		{
			counter = 0;
			if (BitIsSet(ACSR, ACO))
			{
				ClearBit(PORTB, PHOTO_SENSOR_LED); // �������� LED
				photoSensorState = 1; // � �������� ������
			} else {
				SetBit(PORTB, PHOTO_SENSOR_LED); // ��������� LED
				photoSensorState = 0; // � �������� �����
			}
		}
    
    switch (g_state)
		{
			case CHECKING:
			  if (!photoSensorState) // ���� ����� � ���� �������� - � ����� GLOW
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
        if (pwm >= PWM_MAX)
			  {
  			  pwm = PWM_MAX;
  			  g_state = LIGHT;
  			  delay = 0;
			  }
			  break;

			case LIGHT:
			  delay++;
			  if (CheckPort(SENSOR1) || CheckPort(SENSOR2)) delay = 0;
				if (photoSensorState) g_state = FADE; // ���� ������ - � ����� FADE
			  if (delay > DELAY_COUNTS)
			  {
				  g_state = FADE;
			  }
			  break;

			case FADE:
			  if (pwm_state == ON) // ������ ����� IF ������ � ��� ���� ���������, ��� �����������
			  {
				  pwm--;
				  pwm_state = OFF;
			  }
			  else
			  {
				  pwm_state = ON;
				  if (!photoSensorState) // ���� ����� � ���� �������� - � ����� GLOW
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
			  }
			  break;
		}

		OCR0A = pwm;
		_delay_ms(PAUSE);
		counter++;
	}
}


uint8_t CheckPort(uint8_t port)
{
	uint8_t count = 0;
	uint8_t i;
	for (i = 0; i < 16; i++)
	{
		if (BitIsClear(PINB, port)) count++;
	}
	if (count > 10) return 1;
	else return 0;
}

/* This function check Photo Sensor
    It return OFF - when dark, ON - when led light don't need
    Comparator settings see initMC()
*/
uint8_t CheckPhotoSensor(void)
{
  uint8_t count = 0;
  uint8_t i;
  for (i = 0; i < 16; i++)
  {
	  if (BitIsSet(ACSR, ACO)) count++;
  }
  if (count > 10) return 1;
  else return 0; 
}

/**************************************************************************/
/*  ������ ����������� �� Fast PWM                                        */
/*  ����� PB0 �����: �������� ��� ����������, ��������� ����� OC0A = MAX  */
/*  ������� ��� ������������, �������_CPU/256 = 37.5 ���                  */
/**************************************************************************/
void StartPWM(void)
{
  TCCR0A = (1<<COM0A1) |	(0<<COM0A0) |	(1<<WGM01) |	(1<<WGM00);
	TCCR0B = (1<<CS00);
  TCNT0 = 0;
	OCR0A = 0;
}

void StopPWM(void)
{
  TCCR0A=0x00;
	TCCR0B=0x00;
}

