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
# File: pid_ctrl.c
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2007-03-21 Kjeld Jensen
# Modified: 2013-02-12 Kjeld Jensen, updated algorithm, added integer version, added BSD license.
****************************************************************************/
#include "pid_ctrl.h"


/***************************************************************************/
void pid_init (pid_t *pid)
{
	pid->error_prev = 0;
	pid->integral = 0;
}
/***************************************************************************/
void pid_update (pid_t *pid)
{
	pid->error = pid->setpoint - pid->measured; /* calc error */

	pid->integral += pid->error * pid->dT; /* integrate error over time */
	if (pid->integral > pid->integral_max)  /* keep integral within min and max */
		pid->integral = pid->integral_max;
	else if (pid->integral < pid->integral_min)
		pid->integral = pid->integral_min;

	pid->derivative = (pid->error - pid->error_prev)/pid->dT; /* error change */

	pid->output = pid->Kp*pid->error + pid->Ki*pid->integral + pid->Kd*pid->derivative;
	pid->error_prev  = pid->error; /* save err for next iteration */
}
/***************************************************************************/
void pid_int_init (pid_int_t *pid)
{
	pid->error_prev = 0;
	pid->integral = 0;
}
/***************************************************************************/
void pid_int_update (pid_int_t *pid)
{
	pid->error = pid->setpoint - pid->measured; /* calc error */

	pid->integral += pid->error * pid->dT; /* integrate error over time */
	if (pid->integral > pid->integral_max)  /* keep integral within min and max */
		pid->integral = pid->integral_max;
	else if (pid->integral < pid->integral_min)
		pid->integral = pid->integral_min;

	pid->derivative = (pid->error - pid->error_prev)/pid->dT; /* error change */

	pid->output = pid->Kp*pid->error + pid->Ki*pid->integral + pid->Kd*pid->derivative; 

	pid->error_prev  = pid->error; /* save err for next iteration */
}
/***************************************************************************/
