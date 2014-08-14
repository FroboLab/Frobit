/****************************************************************************
# Frobit RoboCard interface
# Copyright (c) 2012-2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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
# File: wheel.c
# Project: Frobit RoboCard interface
# Platform: RoboCard v1.11 http://www.robocard.org
# Microcontroller: ATmega168p
# Author: Kjeld Jensen <kjeld@frobomind.org>
# Created:  2012-08-15 Kjeld Jensen
# Modified: 2013-02-04 Kjeld Jensen, Migrated to the BSD license
# Modified: 2014-04-17 Kjeld Jensen, FrobitV2 updates
****************************************************************************/
/* includes */
#include <avr/interrupt.h>
#include "wheel.h"
#include "rcdef.h"
#include "pid_ctrl_int.h"

/***************************************************************************/
/* defines */
#define STATE_NO_ERR		2 /* maximum system state value before shutting down motors */

#define M_PWM_MAX			255 /* 8 bit PWM */

/* Motor defines (left) */
#define M_L_PWM			OCR0A /* OC0A is 8 bit */
#define M_L			PD6
#define M_L_DDR			DDRD							
#define M_L_PWM_ON		TCCR0A |= BV(COM0A1) /* Clear OC0A on Compare Match */
#define M_L_PWM_OFF		TCCR0A &= ~BV(COM0A1) /* Normal port operation, OC0A disconnected */
#define M_L_C			PD3							
#define M_L_C_PORT		PORTD							
#define M_L_C_DDR		DDRD							
#define M_L_D			PB5						
#define M_L_D_PORT		PORTB							
#define M_L_D_DDR		DDRB							

/* Motor defines (right) */
#define M_R_PWM			OCR0B /* OC0B is 8 bit */
#define M_R			PD5
#define M_R_DDR			DDRD							
#define M_R_PWM_ON		TCCR0A |= BV(COM0B1) /* Clear OC0B on Compare Match */
#define M_R_PWM_OFF		TCCR0A &= ~BV(COM0B1) /* Normal port operation, OC0B disconnected */
#define M_R_C			PB3							
#define M_R_C_PORT		PORTB							
#define M_R_C_DDR		DDRB							
#define M_R_D			PB2						
#define M_R_D_PORT		PORTB							
#define M_R_D_DDR		DDRB							

/* Encoder defines */
#define ENC_L_A			PB4 /* int4 */
#define ENC_L_A_PORT	PINB
#define ENC_L_B			PB1
#define ENC_L_B_PORT	PINB

#define ENC_R_A			PC4 /* int12 */
#define ENC_R_A_PORT	PINC
#define ENC_R_B			PC5
#define ENC_R_B_PORT	PINC

#define TICKS_BUF_LEN	10

/* PID control */

/* Low level motor control */
#define PROP_STEP		1 /* max change per. motor_update() call */

/***************************************************************************/
/* global and static variables */
extern char state;

/* actuator variables */
unsigned char set_cmd;
short vel_set_l, vel_set_r; /* [ticks/s] */
long prop_l, prop_r;
short pwm_l, pwm_r;

/* PID variables */
char pid_enable;
short pid_rate; /* [Hz] */
short pid_interval; /* 1-1000 [ms] */
pid_int_t pid_l, pid_r;
static long feed_forward;

/* status variables */
volatile long ticks_l, ticks_r;
static long pid_ticks_l, pid_ticks_r;
static short ticks_l_buf[TICKS_BUF_LEN];
static short ticks_r_buf[TICKS_BUF_LEN];

/***************************************************************************/
void encoder_init(void)
{
	/* initialize interrupts for external quadrature encoders */
	PCICR |= BV(PCIE0) | BV(PCIE1); /* enable int0 (pin change 0-7) and int1 (pin change 8-14) */
	EICRA |= BV(ISC00) | BV(ISC10); /* int on both rising and falling edge */
	
	PCMSK0 |= BV(PCINT4); /* enable pin change INT4 (PB4) Left  */
	PCMSK1 |= BV(PCINT12); /* enable pin change INT12 (PC4) Right */

	PB_PULL_UP (ENC_L_A_PORT, ENC_L_A); /* pull-up required according to datasheet */
	PB_PULL_UP (ENC_L_B_PORT, ENC_L_B);
	PB_PULL_UP (ENC_R_A_PORT, ENC_R_A);
	PB_PULL_UP (ENC_R_B_PORT, ENC_R_B);

	/* reset variables */
	ticks_l = 0;
	pid_ticks_l = 0;
	ticks_r = 0;
	pid_ticks_r = 0;
}
/***************************************************************************/
ISR (PCINT0_vect)
{
	if (ENC_L_A_PORT & (1<<ENC_L_A)) /* if ch A is high */
	{
		if (ENC_L_B_PORT & (1<<ENC_L_B)) /* if ch B is high */
			ticks_l++;
		else /* if ch B is low */
			ticks_l--;
	}
	else /* if ch A is low */
	{
		if (ENC_L_B_PORT & (1<<ENC_L_B)) /* if ch B is high */
			ticks_l--;
		else /* if ch B is low */
			ticks_l++;
	}
}
/***************************************************************************/
ISR (PCINT1_vect)
{
	if (ENC_R_A_PORT & (1<<ENC_R_A)) /* if ch A is high */
	{
		if (ENC_R_B_PORT & (1<<ENC_R_B)) /* if ch B is high */
			ticks_r--;
		else /* if ch B is low */
			ticks_r++;
	}
	else /* if ch A is low */
	{
		if (ENC_R_B_PORT & (1<<ENC_R_B)) /* if ch B is high */
			ticks_r++;
		else /* if ch B is low */
			ticks_r--;
	}
}
/***************************************************************************/
void motor_set_param(long dT, long Kp, long Ki, long Kd, long i_max, long feed_fwd)
{
	feed_forward = feed_fwd;

	pid_l.dT = dT;
	pid_l.Kp = Kp;
	pid_l.Ki = Ki;
	pid_l.Kd = Kd;
	pid_l.integral_max = i_max;
	pid_l.integral_factor = 10000;
	pid_l.derivative_factor = 100;
	pid_int_init (&pid_l); /* initialize PID controller (left) */

	pid_r.dT = dT;
	pid_r.Kp = Kp;
	pid_r.Ki = Ki;
	pid_r.Kd = Kd;
	pid_r.integral_max = i_max;
	pid_r.integral_factor = 10000;
	pid_r.derivative_factor = 100;
	pid_int_init (&pid_r); /* initialize PID controller (right) */
}
/***************************************************************************/
void motor_init(void)
{
	TCCR0A = BV(WGM00)|BV(WGM01);  /* 10 bit phase correct PWM */
	TCCR0B = BV(CS00); /* prescaler clk_io/1 works for 16 MHz */

	/* left motor */
	PB_OUT (M_L_C_DDR, M_L_C); /* set pin connected to L298 in3 as output */
	PB_OUT (M_L_D_DDR, M_L_D); /* set pin connected to L298 in4 as output */
	PB_OUT (M_L_DDR, M_L); /* set pin connected to L298 Enable B (PWM 0A) as output */
	M_L_PWM_OFF;
	pwm_l = 0;

	/* right motor */
	PB_OUT (M_R_C_DDR, M_R_C); /* set pin connected to L298 in1 as output */
	PB_OUT (M_R_D_DDR, M_R_D); /* set pin connected to L298 in2 as output */
	PB_OUT (M_R_DDR, M_R); /* set pin connected to L298 Enable A (PWM 0B) as output */
	M_R_PWM_OFF;
	pwm_r = 0;
}
/***************************************************************************/
void wheel_init (void)
{
	pid_enable = TRUE; /* default enable PID */

	encoder_init(); /* initialize tacho encoders */
	motor_init(); /* initialize PWM */
}
/***************************************************************************/
static void motor_update (void)
{
	if (state <= STATE_NO_ERR)
	{
		if (pwm_l >= 0) 
		{
			PB_LOW (M_L_C_PORT, M_L_C);
			PB_HIGH (M_L_D_PORT, M_L_D);
			M_L_PWM_ON;
			M_L_PWM = pwm_l;
		}
		else
		{
			PB_HIGH (M_L_C_PORT, M_L_C);
			PB_LOW (M_L_D_PORT, M_L_D);
			M_L_PWM_ON;
			M_L_PWM = -pwm_l;
		}

		if (pwm_r >= 0) 
		{
			PB_HIGH (M_R_C_PORT, M_R_C);
			PB_LOW (M_R_D_PORT, M_R_D);
			M_R_PWM_ON;
			M_R_PWM = pwm_r;
		}
		else
		{
			PB_LOW (M_R_C_PORT, M_R_C);
			PB_HIGH (M_R_D_PORT, M_R_D);
			M_R_PWM_ON;
			M_R_PWM = -pwm_r;
		} 
	}
	else
	{
		M_L_PWM_OFF;
		M_R_PWM_OFF;
		pwm_l = 0;
		pwm_r = 0;
	}
}
/***************************************************************************/
void wheel_update_ticks_buffers (void) /* asways 50 hz */
{
	short i;

	for (i=TICKS_BUF_LEN-1; i>0; i--)
	{
		ticks_l_buf[i] = ticks_l_buf[i-1];
		ticks_r_buf[i] = ticks_r_buf[i-1];
	}

	ticks_l_buf[0] = ticks_l - pid_ticks_l;
	pid_ticks_l = ticks_l;

	ticks_r_buf[0] = ticks_r - pid_ticks_r;
	pid_ticks_r = ticks_r;		
}
/***************************************************************************/
short wheel_calc_vel (short *buf)
{
	short sum, ticks;

	sum = buf[0]+buf[1]+ buf[2];
	if (sum > 20 || sum < -20)
	{
		ticks = (15*buf[0]+10*buf[1]+5*buf[2]);
	}
	else
	{
		sum += (buf[3]+buf[4]);
		if (sum > 20 || sum < -20)
		{
			ticks = (10*buf[0]+8*buf[1]+6*buf[2]+4*buf[3]+2*buf[4]);
		}
		else
		{
			ticks = (7*buf[0]+6*buf[1]+5*buf[2]+4*buf[3]+3*buf[4]+2*buf[5]+1*buf[6]+1*buf[7]+1*buf[8]);
		}
	}
	return ticks;
}
/***************************************************************************/
void wheel_update_pid (void)
{
	if (vel_set_l != 0 && state <= STATE_NO_ERR) /* if the velocity is set to zero */
	{
		pid_l.setpoint = vel_set_l*30/50;
		pid_l.measured = wheel_calc_vel(ticks_l_buf); 
		pid_int_update (&pid_l); 

		prop_l += pid_l.output/50;

		if (vel_set_l > 0) 
			pwm_l = feed_forward + prop_l;
		else
			pwm_l = -feed_forward + prop_l;
		
		if (pwm_l > M_PWM_MAX)
			pwm_l = M_PWM_MAX;
		else if (pwm_l <-M_PWM_MAX)
			pwm_l = -M_PWM_MAX;
	}
	else
	{
		prop_l = 0;
		pwm_l = 0;
		pid_int_init (&pid_l); 		
	}


	if (vel_set_r != 0 && state <= STATE_NO_ERR) /* if the velocity is set to zero */
	{
		pid_r.setpoint = vel_set_r*30/50;
		pid_r.measured = wheel_calc_vel(ticks_r_buf); 
		pid_int_update (&pid_r); 

		prop_r += pid_r.output/50;

		if (vel_set_r > 0) 
			pwm_r = feed_forward + prop_r;
		else
			pwm_r = -feed_forward + prop_r;
		
		if (pwm_r > M_PWM_MAX)
			pwm_r = M_PWM_MAX;
		else if (pwm_r <-M_PWM_MAX)
			pwm_r = -M_PWM_MAX;
	}
	else
	{
		prop_r = 0;
		pwm_r = 0;
		pid_int_init (&pid_r); 		
	}

	motor_update(); 
}
/***************************************************************************/
void wheel_update_open_loop (void)
{
	pwm_l = vel_set_l; 
	pwm_r = vel_set_r;
	motor_update();
}
/***************************************************************************/
