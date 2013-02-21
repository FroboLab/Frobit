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
# File: wheel.h
# Project: Frobit RoboCard interface
# Platform: RoboCard v1.11 http://www.robocard.org
# Microcontroller: ATmega168p
# Author: Kjeld Jensen <kjeld@frobomind.org>
# Created:  2012-08-29 Kjeld Jensen
# Modified: 2013-02-04 Kjeld Jensen, Migrated to the BSD license
****************************************************************************/

#ifndef _WHEEL_H
#define _WHEEL_H

/***************************************************************************/
/* defines */

#define WHEEL_LEFT		0
#define WHEEL_RIGHT		1

/***************************************************************************/
/* shared variables */

/* actuator variables */
extern short spd_set_l;
extern short spd_set_r;

/* status variables */
extern short ticks_l;
extern short ticks_r;
extern short spd_l;
extern short spd_r;

/***************************************************************************/
/* function prototypes */

void wheel_init(void);
void wheel_update_pid (void);
void wheel_update_open_loop (void);
void motor_set_param(char wheel, long dT, long Kp, long Ki, long Kd);

/***************************************************************************/
#endif
