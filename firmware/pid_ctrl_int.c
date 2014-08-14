/****************************************************************************
# PID controller
# Copyright (c) 2007-2014, Kjeld Jensen <kj@kjen.dk>
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
# File: pid_ctrl.c
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2007-03-21 Kjeld Jensen
# Modified: 2013-02-12 Kjeld Jensen, updated algorithm, added integer version, added BSD license.
# Modified: 2013-11-20 KJ, changed (integer) integral integral minimum to - integral maximum
#                          added (integer) internal state variables for P,I, D output (for debug)
#                          fixed I and D bug in integer version.
# Modified: 2014-04-17 Kjeld Jensen, removed floating point version
****************************************************************************/
#include "pid_ctrl_int.h"

/***************************************************************************/
void pid_int_init (pid_int_t *pid)
{
	pid->error = 0;
	pid->error_prev = 0;
	pid->integral_state = 0;
	pid->output_p = 0;
	pid->output_i = 0;
	pid->output_d = 0;
	pid->output = 0;
}
/***************************************************************************/
void pid_int_update (pid_int_t *pid)
{
	pid->error = pid->setpoint - pid->measured; /* calc error */

	pid->integral_state += pid->error * pid->dT; /* integrate error over time */

	pid->integral = pid->integral_state/pid->integral_factor;

	if (pid->integral > pid->integral_max)  /* keep integral within +/- max */
		pid->integral = pid->integral_max;
	else if (pid->integral < -pid->integral_max)
		pid->integral = -pid->integral_max;

	pid->derivative = (pid->error - pid->error_prev)*pid->derivative_factor/pid->dT; /* error change */

	pid->output_p = pid->Kp*pid->error;
	pid->output_i = pid->Ki*pid->integral;
	pid->output_d = pid->Kd*pid->derivative;

	pid->output = pid->output_p + pid->output_i + pid->output_d; 

	pid->error_prev  = pid->error; /* save err for next iteration */
}
/***************************************************************************/
