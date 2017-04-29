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
#include <avr/interrupt.h>


#define PWM_OUT          PORTB0
#define PHOTO_SENSOR_LED PORTB1
#define PHOTO_SENSOR     PORTB2
#define MOVE_SENSOR1     PORTB3
#define MOVE_SENSOR2     PORTB4

#define PWM_MAX          200
#define PWM_MIN          30

// States of devices
#define ON			         1
#define OFF			         0

// Set the common delay time in ms
#define PAUSE 20

// Задаем время свечения подсветки
#define LIGHT_TIME 6 // in seconds, MAX = 1310s

// Задаем периодичность проверки фотодатчика
#define PHOTO_SENSOR_TIME 2 // in seconds, MAX = 1310s


#define LIGHT_TIME_COUNTS LIGHT_TIME * 1000UL / PAUSE
#define PHOTO_SENSOR_TIME_COUNTS PHOTO_SENSOR_TIME * 1000UL / PAUSE

#if LIGHT_TIME_COUNTS > 65535
	#error "LIGHT_TIME_COUNTS must be less than 65536! Check values of LIGHT_TIME and PAUSE."
#endif

#if PHOTO_SENSOR_TIME > 65535
#error "PHOTO_SENSOR_TIME_COUNTS must be less than 65536! Check values of PHOTO_SENSOR_TIME and PAUSE."
#endif

enum DEV_MODE{
  CHECKING,
  GLOW,
  LIGHT,
  FADE
};

///Global variables
volatile uint8_t g_AIN_state = 0;

//Fuctions prototypes
void initMC(void);
uint8_t CheckPort(uint8_t);
//uint8_t CheckPhotoSensor(void);
void StartPWM(void);
void StopPWM(void);

inline void initMC(){
  
	/*	;=======================================================================
		Альтернативные функции пинов
                                          DEFINE            DDR   PORT
			PB5	(RESET/ADC0/dW/PCINT5)          ---               0     0
			PB4	(ADC2/PCINT4)                   MOVE_SENSOR2      IN    pull-up
			PB3	(ADC3/CLKI/PCINT3)              MOVE_SENSOR1      IN    pull-up
			PB2	(SCK/ADC1/T0/PCINT2)            PHOTO_SENSOR      IN    hi-z
			PB1	(MISO/AIN1/OC0B/INT0/PCINT1)    PHOTO_SENSOR_LED  OUT   1
			PB0	(MOSI/AIN0/OC0A/PCINT0)         PWM_OUT           OUT   0
	*/
  DDRB =	(0<<DDB4) |		(0<<DDB3) |		(0<<DDB2) |		(1<<DDB1) |		(1<<DDB0);
  PORTB =	(1<<PORTB4) |	(1<<PORTB3) |	(0<<PORTB2) |	(1<<PORTB1) |	(0<<PORTB0);

  asm("nop");

  // Таймер остановлен
  TCCR0A =	0x00;
	TCCR0B =	0x00;
  
  /************************************************************************/
  /*  Настраваем компаратор                                               */
  /************************************************************************/
  /*
    Компаратор включен
    Подключение к AIN0 внутреннего ИОН = 1.1V (ACBG)
    Включено прерывание (ACIE) по любому изменению компаратора (ACIS[1:0])
    Включен мультиплексор (ACME)
    АЦП выключен (ADEN)
    Переназначение входа компаратора AIN1(PB1) -> на вход АЦП ADC1(PB2)
  */
  ACSR = (0<<ACD) | (1<<ACBG) | (1<<ACIE) | (0<<ACIS1) | (0<<ACIS0);
	ADCSRA &= ~(0<<ADEN); // Выключаем АЦП
	ADCSRB |= (1<<ACME);	//Подключение мультиплексора АЦП к компаратору 
	ADMUX = (0<<MUX1) | (1<<MUX0); // Переназначение входа AIN1 -> ADC1 (PB1 -> PB2)

}

ISR(ANA_COMP_vect){
  if (BitIsSet(ACSR, ACO))
  {
    g_AIN_state = 1; // В прихожей светло
  } else 
  {
    g_AIN_state = 0; // В прихожей темно
  }
  
}

int main(void)
{
	initMC();

  uint8_t programState;
  uint16_t lightTime_counts = 0; // max counts = 65535
  uint16_t checkPhotoSensor_counts = 0; // max counts = 65535
	
	programState = CHECKING;
  // Variable pwm_state allow to do FADE time twice long than GLOW time
	uint8_t pwm_state = ON;
  uint8_t photoSensorState = 0;
  uint8_t pwm = 0;
  
	asm("sei");
  
	while (1)
	{
    if (checkPhotoSensor_counts > PHOTO_SENSOR_TIME_COUNTS)
		{
      checkPhotoSensor_counts = 0;
			if (g_AIN_state)  // Проверям переменную из прерывания компаратора
			{
  			ClearBit(PORTB, PHOTO_SENSOR_LED); // включить LED
  			photoSensorState = 1; // В прихожей светло
  		} else 
      {
  			SetBit(PORTB, PHOTO_SENSOR_LED); // выключить LED
  			photoSensorState = 0; // В прихожей темно
			}
		}
    
    switch (programState)
		{
			case CHECKING:
			  if (!photoSensorState) // если темно и есть движение - в режим GLOW
			  {
				  if(CheckPort(MOVE_SENSOR1) || CheckPort(MOVE_SENSOR2))
				  {
					  programState = GLOW;
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
  			  programState = LIGHT;
  			  lightTime_counts = 0;
			  }
			  break;

			case LIGHT:
			  lightTime_counts++;
			  if (CheckPort(MOVE_SENSOR1) || CheckPort(MOVE_SENSOR2)) lightTime_counts = 0;
				if (photoSensorState) programState = FADE; // если светло - в режим FADE
			  if (lightTime_counts > LIGHT_TIME_COUNTS)
			  {
				  programState = FADE;
			  }
			  break;

			case FADE:
			  if (pwm_state == ON) // засчет этого IF тухнем в два раза медленнее, чем разгораемся
			  {
				  pwm--;
				  pwm_state = OFF;
			  }
			  else
			  {
				  pwm_state = ON;
				  if (!photoSensorState) // если темно и есть движение - в режим GLOW
				  {
					  if (CheckPort(MOVE_SENSOR1) || CheckPort(MOVE_SENSOR2))
					  {
					    programState = GLOW;
			      }
				  }
			  }

			  if (pwm < PWM_MIN)
			  {
				  StopPWM();
				  programState = CHECKING;
			  }
			  break;
		}

		OCR0A = pwm;
		_delay_ms(PAUSE);
		checkPhotoSensor_counts++;
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
/*  Таймер настраиваем на Fast PWM                                        */
/*  Вывод PB0 режим: сбросить при совпадении, установка когда OC0A = MAX  */
/*  Частота ШИМ максимальная, частота_CPU/256 = 37.5 кГц                  */
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

