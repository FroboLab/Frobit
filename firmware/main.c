/****************************************************************************
# Frobit RoboCard interface
# Copyright (c) 2012-2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
# File: main.c
# Project: Frobit RoboCard interface
# Platform: RoboCard v1.11 http://www.robocard.org
# Microcontroller: ATmega168p
# Author: Kjeld Jensen <kjeld@frobomind.org>
# Created:  2012-08-15
# Modified: 2013-02-04 Migrated to the BSD license
****************************************************************************/
/* includes */
#include <avr/interrupt.h>
#include <stdlib.h>
#include "rcdef.h"
#include "wheel.h"
#include "rcserial.h"
#include "rcnmea.h"

/***************************************************************************/
/* defines */

#define false				0
#define true				1

/* defines for the timer1 interrupt */
#define INT1_1MS_CNT			2000 /* 1ms */
#define FLIPBIT				PC3
#define FLIPBIT_PORT			PORTC
#define FLIPBIT_DDR			DDRC

/* system state (only the highest status is transmitted through NMEA) */
#define STATE_OK			1
#define STATE_NMEA_ERR			2 /* error occured while receiving the latest nmea packet */
#define STATE_WATCHDOG			3 /* no validated nmea packets received for the past 0.2 seconds */
#define STATE_LOWBAT			4 /* battery voltage too low */

#define VOLTAGE_MIN_DEFAULT		682 /* minimum voltage */

/* signal led defines */
#define LED_STATE_OFF			0
#define LED_STATE_ON			1
#define LED_DELAY			4 /* times the cycle */

/* adc defines */
#define ADC_NUM				1 /* number of used ADC's */
#define ADC_VOLT			0 /* Voltage divider connected at ADC0 (PC0) */

/* NMEA defines */
#define NMEA_WD_TOUT			2 /* 1/10 [s] without receiving ok NMEA before timeout */

/***************************************************************************/
/* global and static variables */

/* timer1 and scheduler variables */
volatile unsigned char t1ms;
unsigned short t1ms_cnt;

/* system variables */
char state;

/* user interface variables*/
char led_state;
char led_signal;
char led_count;
char but1;

/* NMEA variables */
unsigned short nmea_wd_timeout; /* NMEA watchdog timeout [ms] */
unsigned short nmea_wd; /* NMEA watchdog counter */
unsigned short pfbst_interval; /* PFBST interval (20-10000) [ms] */
short nmea_ticks_l, nmea_ticks_r;

/* ADC variables (10 bit [0;1023]) */
volatile unsigned short adc_data[ADC_NUM]; /* ADC data variables */
volatile unsigned char adc_ch; /* current adc channel */
unsigned char adc_ports[ADC_NUM]; /* maps to ADC ports */

unsigned short voltage;
unsigned short voltage_min;
unsigned char battery_low_warning;

/* wheel variables */
extern char pid_enable;
extern short pid_interval; /* 1-1000 [ms] */
short pid_rate; /* Hz */


/***************************************************************************/
void sched_init(void)
{
	/* timer 1 interrupt init */
	t1ms = 0;
	t1ms_cnt = 0;
        TIMSK1 = BV(OCIE1A); 
        TCCR1B = BV(CS11) | BV(WGM12); /* clk/8, Clear Timer on Compare Match (OCR1A) */  
        OCR1A = INT1_1MS_CNT;
	PB_OUT (FLIPBIT_DDR, FLIPBIT); /* set 1ms flipbit as output */
}
/***************************************************************************/
/*ISR(SIG_OUTPUT_COMPARE1A) */
ISR (TIMER1_COMPA_vect)
{
	t1ms++;
	PB_FLIP (FLIPBIT_PORT, FLIPBIT); /* time to flip the flip bit */
}
/***************************************************************************/
/* ADC interrupt handler */
ISR (ADC_vect)
{
	adc_data[adc_ch] = ((ADCL) | ((ADCH)<<8)); /* read value */
	if (++adc_ch >= ADC_NUM) /* go to next adc channel */
		adc_ch = 0;
	ADMUX = adc_ports[adc_ch]; /* ref. is AREF pin */
	ADCSRA |= (1<<ADSC);  /* request a new adc conversion */
}
/***************************************************************************/
void adc_init (void)
{
	adc_ch = 0;
	adc_ports[0] = ADC_VOLT;

	ADCSRA = BV(ADEN); /* enable ADC conversion */
	ADCSRA |= (BV(ADPS2) | BV(ADPS1) | BV(ADPS0)); /* div by 128 presc. */
	ADCSRA |= BV(ADIE); /* interrupt enable */
	ADMUX = adc_ports[adc_ch]; /* Voltage reference is AREF) */
	ADCSRA |= BV(ADSC); /* request ADC conversion */
}
/***************************************************************************/
void voltage_update(void)
{
	if (spd_set_l == 0 && spd_set_r == 0) /* only test when standing still */
	{
		if (battery_low_warning == false && voltage < voltage_min)
			battery_low_warning = true;
		else if (battery_low_warning == true && voltage >= voltage_min)
			battery_low_warning = false;
	}
}
/***************************************************************************/
void led_update(void)
{
	/* led_state = state; */
	switch (led_state) {
		case LED_STATE_ON:
			led_state = LED_STATE_OFF;
			RC_LED_OFF;
			break;

		case LED_STATE_OFF:
			led_count++;
			if (led_count <= led_signal) {
				RC_LED_ON;
				led_state = LED_STATE_ON;
			}
			else if (led_count > led_signal + LED_DELAY) {
				led_count = 0;
			}
			break;
	}
}
/***************************************************************************/
void led_init(void)
{
	RC_LED_INIT;
	led_count = 0;
	led_signal = 1;

	led_state = LED_STATE_OFF;
}
/***************************************************************************/
void button_update(void)
{
	but1 = PB_IS_HIGH (PIND, PIND7); /* button enabled if logic zero */
}
/***************************************************************************/
void button_init(void)
{
	PB_PULL_UP (PORTD, PD7); /* enable pull-up resistor */
	button_update();
}
/***************************************************************************/
void nmea_init(void)
{
	nmea_reset();
	nmea_wd = 0xffff; /* set watchdog timeout at init */

	tx[0] = '$'; /* send first boot message */
	tx[1] = 'P';
	tx[2] = 'F';
	tx[3] = 'B';
	tx[4] = 'H';
	tx[5] = 'I';
	tx[6] = ',';
	tx[7] = '1';
	tx[8] = ',';
	tx[9] = '1';
	tx_len = 10;
	nmea_tx();

	tx[4] = 'S'; /* prepare for status messages */
	tx[5] = 'T';
	tx_len = 7;

	nmea_ticks_l = 0;
	nmea_ticks_r = 0;
}
/***************************************************************************/
void nmea_rx_parse(void)
{
	if (rx[3] == 'C' && rx[4] == 'T') /* control */
	{
		nmea_wd = 0; /* reset watchdog timeout */
		rx_ite = 5; /* jump to first value */
		spd_set_l = nmea_rx_next_val()/pid_rate;
		if (rx_ite != -1)
			spd_set_r = nmea_rx_next_val()/pid_rate;
	}
	else if (rx[3] == 'C' && rx[4] == 'P') /* Communication Parameters */
	{
		rx_ite = 5; /* jump to first value */
		pfbst_interval = nmea_rx_next_val();
		if (rx_ite != -1)
			nmea_wd_timeout = nmea_rx_next_val();
	}
	else if (rx[3] == 'S' && rx[4] == 'P') /* System Parameters */
	{
		rx_ite = 5; /* jump to first value */
		voltage_min = nmea_rx_next_val();
	}
	else if (rx[3] == 'W' && rx[4] == 'P') /* Wheel Parameters */
	{
		rx_ite = 5; /* jump to first value */
		pid_enable = nmea_rx_next_val();
		if (pid_enable == TRUE)
		{ 
			long Kp_l, Ki_l, Kd_l, Kp_r, Ki_r, Kd_r;
			pid_interval = nmea_rx_next_val();
			pid_rate = 1000/pid_interval; /* always remember after setting pid_interval */
			if (rx_ite != -1)
			{
				Kp_l = nmea_rx_next_val();
				if (rx_ite != -1)
				{
					Ki_l = nmea_rx_next_val();
					if (rx_ite != -1)
					{
						Kd_l = nmea_rx_next_val();
						if (rx_ite != -1)
						{
							Kp_r = nmea_rx_next_val();
							if (rx_ite != -1)
							{
								Ki_r = nmea_rx_next_val();
								if (rx_ite != -1)
								{
									Kd_r = nmea_rx_next_val();
									motor_set_param (WHEEL_LEFT, pid_interval, Kp_l, Ki_l, Kd_l);
									motor_set_param (WHEEL_RIGHT, pid_interval, Kp_r, Ki_r, Kd_r);
								}
							}
						}
					}
				}
			}
		}
	}
}
/***************************************************************************/
void nmea_tx_status(void)
{
	short ticks_l, ticks_r;
	tx_len = 7; /* keep the NMEA message prefix */

	nmea_tx_append_ushort (state);
	ticks_l = nmea_ticks_l;
	nmea_ticks_l = 0;
	ticks_r = nmea_ticks_r;
	nmea_ticks_r = 0;
	nmea_tx_append_short (ticks_l);
	nmea_tx_append_short (ticks_r);
	nmea_tx_append_ushort (voltage); /* battery voltage [0;1023] */
	tx_len--; /* delete the last comma */
	nmea_tx();
}
/***************************************************************************/
void state_update(void)
{
	if (battery_low_warning == true)
		state = STATE_LOWBAT;
	else if (nmea_wd > NMEA_WD_TOUT)
		state = STATE_WATCHDOG; 
	else if (nmea_err != 0)
	{
		state = STATE_NMEA_ERR;
		nmea_err = 0;
	}
	else
		state = STATE_OK;

	led_signal = state; /* RoboCard LED flashes state number */
}
/***************************************************************************/
void sched_update (void)
{
	t1ms_cnt++;
	if (t1ms_cnt == 10000)
		t1ms_cnt = 0;
	
	if (t1ms_cnt % pid_interval == 0) /* motor controller update */
	{
		if (pid_enable)
			wheel_update_pid(); /* update PID motor controller */
		else
			wheel_update_open_loop(); /* update open loop motor controller */
	}

	if (t1ms_cnt % pfbst_interval == 0) /* send $PFBST */
	{
		nmea_tx_status();
	}

	/* each 10 ms */
	if (t1ms_cnt % 10 == 0) /* each 10 ms */
	{
		if (t1ms_cnt % 20 == 0) /* each 20 ms */
		{
		}

		if (t1ms_cnt % 50 == 0) /* each 50 ms */
		{
		}

		if (t1ms_cnt % 100 == 0) /* each 100 ms */
		{
			if (nmea_wd_timeout)
				nmea_wd++; /* increase watchdog timeout */
			else
				nmea_wd = 0;
			voltage = adc_data[0]; /* request voltage measurement */
			state_update();
		}
		if (t1ms_cnt % 200 == 0) /* each 200 ms */
		{
			button_update();		
			led_update();
			voltage_update();
		}
	}
}
/***************************************************************************/
int main(void)
{
	sched_init(); /* initialize the scheduler */
	led_init(); /* initialize led */
	button_init(); /* initialize button */
	adc_init(); /* initialize ADC (battery voltage measurement) */
	serial_init(); /* initialize serial communication */
	nmea_init(); /* initialize nmea protocol handler */
	wheel_init(); /* initialize encoders, PWM output, PID etc. */

	pid_interval = 100; /* default PID update interval 100ms */ 
	pid_rate = 1000/pid_interval; /* always remember after setting pid_interval */
	pfbst_interval = 100; /* send $PFBST at 100 ms interval */
	nmea_wd_timeout = 1; /* set PFBCT watchdog timeout to 100ms */
	nmea_wd = NMEA_WD_TOUT+1; /* make sure we begin in watchdog timeout state */
	voltage_min = VOLTAGE_MIN_DEFAULT;
	battery_low_warning = false;
	state_update();

	sei(); /* enable interrupts */
	for (;;) /* go into an endless loop */
	{
		/* motor_update(); */

		if (t1ms != 0) /* if the interrupt has timed out after 10ms */
		{
			t1ms --;
			sched_update(); /* run the scheduler */
		}
		else
		{
			nmea_rx_update();
		}
	}
	return 0; /* just for the principle as we never get here */
}
/***************************************************************************/
