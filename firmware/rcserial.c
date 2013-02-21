/****************************************************************************
# RoboCard Serial Communication Library
# Copyright (c) 2009-2013, Kjeld Jensen <kj@kjen.dk>
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
# File: rcserial.c
# Project: RoboCard Serial Communication Library
# Platform: RoboCard v1.0 http://www.robocard.dk
# Microcontroller: ATmega88PA
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2009-12-08 Kjeld Jensen
# Modified: 2011-02-21 Kjeld Jensen, released under the MIT license
# Modified  2011-03-14 Kjeld Jensen, added serial_rx_flush()
# Modified  2011-12-18 Kjeld Jensen, added serial_tx_idle
# Modified  2013-01-15 Kjeld Jensen, added support for double speed baud rates
# Modified  2013-02-04 Kjeld Jensen, migrated to the BSD license 
# Modified  2013-02-18 Kjeld Jensen, added rx interrupt routine
****************************************************************************/
/* includes */

#include <avr/io.h>
#include <avr/interrupt.h>

/***************************************************************************/
/* defines for serial communication */

/* #define DOUBLE_SPEED_MODE */

#define FOSC 16000000	/* oscillator frequency [Hz] */
#define BAUD 57600		/* baud rate */

#ifdef DOUBLE_SPEED_MODE
	#define UBRR (FOSC/BAUD/8 - 1)
#else
	#define UBRR (FOSC/BAUD/16 - 1)
#endif

/***************************************************************************/
#define INB_MAX		20

unsigned char inb[INB_MAX];
short inb_head, inb_tail;

/***************************************************************************/
void serial_init(void)
{
	/* enable tx and rx */
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);

	/* set baud rate */
	UBRR0H = (unsigned char) ((UBRR)>>8);
	UBRR0L = (unsigned char) (UBRR); /* remember the ()! */

	/* asynchronous 8N1 */
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
	
	/* enable double speed mode if #DOUBLE_SPEED_MODE is set */
#ifdef DOUBLE_SPEED_MODE
	UCSR0A |= U2X0;
#endif

	/* enable rx interrupt */
	inb_head = 0;
	inb_tail = 0;
	UCSR0B |= (1 << RXCIE0);
}
/***************************************************************************/
ISR (USART_RX_vect)
{
	inb_head++;
	if (inb_head == INB_MAX)
		inb_head = 0;
	if (inb_head != inb_tail) /* do not add if buffer overrun */
		inb[inb_head] = UDR0;
}
/***************************************************************************/
void serial_tx (unsigned char c)
{
	/* wait for an empty transmit buffer */
	while ( !(UCSR0A & (1<<UDRE0))) /* check Data Register Empty bit */
		;
	UDR0 = c; /* fill Data Register */
}
/***************************************************************************/
void serial_tx_string (char *s)
{
	while (*s != 0)
	{
		/* wait for an empty transmit buffer */
		while (!(UCSR0A & (1<<UDRE0))) /* check Data Register Empty bit */
			;
		UDR0 = *s; /* fill Data Register */
		s++; /* go to next char in s */
	}
}
/***************************************************************************/
unsigned char serial_tx_idle (void)
{
	/* test if no transmission is in progress */
	return (UCSR0A & (1<<TXC0)); /* check Transmit Complete bit */
}
/***************************************************************************/
unsigned char serial_rx_avail (void)
{
	/* return true if there is a character in the input buffer */
	return (inb_head != inb_tail); 
}
/***************************************************************************/
unsigned char serial_rx (void)
{
	/* return next char in buffer */
	inb_tail++;
	if (inb_tail == INB_MAX)
		inb_tail = 0;
	
	return (inb[inb_tail]);
}
/***************************************************************************/
void serial_rx_flush (void)
{
	unsigned char c;
	while (UCSR0A & (1<<RXC0))
		c = UDR0;
}
/***************************************************************************/
