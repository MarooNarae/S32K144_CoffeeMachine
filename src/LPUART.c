/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016 - 2018, NXP.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "device_registers.h"	/* include peripheral declarations S32K144 */
#include "LPUART.h"

void LPUART1_init(void) {
	PCC->PCCn[PCC_LPUART1_INDEX] &= ~PCC_PCCn_CGC_MASK; /* Ensure clk disabled for config */
	PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_PCS(0b001) | PCC_PCCn_CGC_MASK; /* Enable clock for LPUART1 regs */
	LPUART1->BAUD = 0x0F000034; /* Initialize for 9600 baud, 1 stop: */
	LPUART1->CTRL = 0x002C0000; /* Enable transmitter & receiver, no parity, 8 bit char: */
}

void LPUART1_transmit_char(char send) {    /* Function to Transmit single Char */
	while((LPUART1->STAT & LPUART_STAT_TDRE_MASK)>>LPUART_STAT_TDRE_SHIFT==0);
	/* Wait for transmit buffer to be empty */
	LPUART1->DATA=send;              /* Send data */
}

void LPUART1_transmit_string(char data_string[])  {  /* Function to Transmit whole string */
	uint32_t i=0;
	while(data_string[i] != '\0')  {           /* Send chars one at a time */
		LPUART1_transmit_char(data_string[i]);
		i++;
	}
}

char LPUART1_receive_char(void) {    /* Function to Receive single Char */
	char receive;
	while((LPUART1->STAT & LPUART_STAT_RDRF_MASK)>>LPUART_STAT_RDRF_SHIFT==0);
	/* Wait for received buffer to be full */
	receive= LPUART1->DATA;            /* Read received data*/
	return receive;
}

void LPUART1_receive_and_echo_char(void)  {  /* Function to echo received char back */
	char send = LPUART1_receive_char();        /* Receive Char */
	LPUART1_transmit_char(send);               /* Transmit same char back to the sender */
	LPUART1_transmit_char('\n');               /* New line */
	LPUART1_transmit_char('\r');               /* Return */
}
