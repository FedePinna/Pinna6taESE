/* Copyright 2016, XXXXXX
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "examen.h"       /* <= own header */
#include "switch.h"
#include "timer.h"
#include "led.h"
#include "adc.h"
#include "dac.h"
#include "uart.h"
#include "stdio.h"


/*==================[macros and definitions]=================================*/

#define TO_VALUE_DAC(a)  a*RES_DAC
#define TO_VALUE_ADC(a)  ((float)a)/RES_ADC

#define GAIN1 1.2
#define GAIN2 1.1
#define GAIN3 1.
#define GAIN4 0.9
#define GAIN5 0.8

#define OFFSET1 0.2
#define OFFSET2 0.1
#define OFFSET3	0.0
#define OFFSET4 -0.1
#define OFFSET5	0.2


/*==================[internal data declaration]==============================*/

volatile uint16_t count;
volatile uint16_t count_dac;
volatile uint32_t value_dac;
volatile uint16_t value_adc;


/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */


void delay(int32_t count){

	int32_t i;

	for(i=0;i<=count;i++){
		asm("nop");
	}
}


volatile struct signal signal_cal;
struct signal signal_ent;

void counter(){

	count_dac++;
	count++;

	if(count_dac<=signal_cal.duty_cycle){
		value_dac=(uint32_t)signal_cal.vmax;
	}else{
		value_dac=(uint32_t)signal_cal.vmin;
	}
	if(count_dac==signal_cal.period){
		count_dac=0;
	}

	dacUpdate(value_dac);

	value_adc = adcGetValue(CH1);

	signal_ent.vins=(TO_VALUE_ADC(value_adc))*signal_ent.gain + signal_ent.offset;
	signal_ent.vmax= MAX(signal_ent.vmax,signal_ent.vins);
	signal_ent.vmin= MIN(signal_ent.vmin,signal_ent.vins);


}



int main(void)
{
   /* perform the needed initialization here */

	ledsInit();

	/***********señal calibracion*********/

	signal_cal.vpp =1.0;
	signal_cal.offset=1.5;
	signal_cal.period = 100;
	signal_cal.duty_cycle=50;

	signal_cal.vmax = (signal_cal.vpp/2)+signal_cal.offset;
	signal_cal.vmin = signal_cal.offset-(signal_cal.vpp/2);

	signal_cal.vmax = TO_VALUE_DAC(signal_cal.vmax);
	signal_cal.vmin = TO_VALUE_DAC(signal_cal.vmin);



	/***********************************/

	signal_ent.gain = 1;
	signal_ent.offset = 0.5;
	signal_ent.vmin = 3.3;
	signal_ent.vmax = 0;

	dacInit();
	adcInit(CH1);
	timerInit(1,&counter);
	switchsInit();
	UART_Init();


	while(1){

		if(count==1000){
			count=0;
		//	UART_SendChar();
		}


	}
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

