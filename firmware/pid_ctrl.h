/****************************************************************************
# PID controller
# Copyright (c) 2007-2013, Kjeld Jensen <kj@kjen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name RoboCard nor the
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
# File: pid_ctrl.h
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2007-03-21
# Modified: 2013-02-12 Kjeld Jensen, updated algorithm, added integer version, added BSD license.
****************************************************************************/

/* PID data record */
typedef struct
{
	/* initialization parameters */
	double Kp; /* proportional gain */
	double Ki; /* integrational gain */
	double Kd; /* deriative gain */
	double dT; /* time interval [ms] */
	double integral_min; /* minimum integral */
	double integral_max; /* maximum integral */
	/* update input */
	double setpoint; /* setpoint */
	double measured; /* measured value */
	/* internal vars */
	double error;
	double error_prev; /* error previous step */
	double integral;
	double derivative; 
	/* update output */
	double output; /* controller output */
} pid_t;

/* PID data record (integer version) */
typedef struct
{
	/* initialization parameters */
	long Kp; /* proportional gain */
	long Ki; /* integrational gain */
	long Kd; /* deriative gain */
	long dT; /* time interval [ms] */
	long integral_min; /* minimum integral */
	long integral_max; /* maximum integral */
	/* update input */
	long setpoint; /* setpoint */
	long measured; /* measured value */
	/* internal vars */
	long error;
	long error_prev; /* error previous step */
	long integral;
	long derivative; 
	/* update output */
	long output; /* controller output */
} pid_int_t;

/***************************************************************************/
/* function prototypes */
void pid_init (pid_t *pid);
void pid_update (pid_t *pid);

void pid_int_init (pid_int_t *pid);
void pid_int_update (pid_int_t *pid);

/***************************************************************************/
