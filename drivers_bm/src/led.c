/* Copyright 2016, XXXXXXXXX  
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

/** \brief Blinking Bare Metal driver led
 **
 **
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal LED Driver
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

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif
#include "led.h"


/*==================[macros and definitions]=================================*/


#define PORT0 0
#define PORT1 1
#define PORT2 2
#define PORT5 5

#define OUTPUT 1
#define INPUT 0

#define LED1_PIN 14
#define LED2_PIN 11
#define LED3_PIN 12
#define LED0_R_PIN 0
#define LED0_G_PIN 1
#define LED0_B_PIN 2

#define LED1_BIT 14
#define LED2_BIT 11
#define LED3_BIT 12
#define LED0_R_BIT 0
#define LED0_G_BIT 1
#define LED0_B_BIT 2

#define PACKLEDS 2
/*
#define LED1_MASK (1<<LED1_PIN)
#define LED2_MASK (1<<LED2_PIN)
#define LED3_MASK (1<<LED3_PIN)
#define LED0_R_MASK (1<<LED0_R_PIN)
#define LED0_G_MASK (1<<LED0_G_PIN)
#define LED0_B_MASK (1<<LED0_B_PIN)
*/
#define LED1_MASK (1<<LED1_BIT)
#define LED2_MASK (1<<LED2_BIT)
#define LED3_MASK (1<<LED3_BIT)
#define LED0_R_MASK (1<<LED0_R_BIT)
#define LED0_G_MASK (1<<LED0_G_BIT)
#define LED0_B_MASK (1<<LED0_B_BIT)

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

void ledsInit(){

	Chip_GPIO_Init(LPC_GPIO_PORT);

	Chip_SCU_PinMux(PACKLEDS,LED1_PIN,MD_PLN,FUNC0);
	Chip_SCU_PinMux(PACKLEDS,LED2_PIN,MD_PLN,FUNC0);
	Chip_SCU_PinMux(PACKLEDS,LED3_PIN,MD_PLN,FUNC0);
	Chip_SCU_PinMux(PACKLEDS,LED0_R_PIN,MD_PLN,FUNC4);
	Chip_SCU_PinMux(PACKLEDS,LED0_G_PIN,MD_PLN,FUNC4);
	Chip_SCU_PinMux(PACKLEDS,LED0_B_PIN,MD_PLN,FUNC4);

	Chip_GPIO_SetDir(LPC_GPIO_PORT,PORT0,LED1_MASK,OUTPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,PORT1,LED2_MASK|LED3_MASK|LED3_MASK,OUTPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,PORT5,LED0_R_MASK|LED0_G_MASK|LED0_B_MASK,OUTPUT);

}

void ledON(uint8_t led){

	switch (led) {
		case LED_YELLOW:
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,PORT0,LED1_BIT);
			break;
		case LED_RED:
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,PORT1,LED2_BIT);
			break;
		case LED_GREEN:
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,PORT1,LED3_BIT);
			break;
		case LED_RGB_RED:
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,PORT5,LED0_R_BIT);
			break;
		case LED_RGB_GREEN:
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,PORT5,LED0_G_BIT);
			break;
		case LED_RGB_BLUE:
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,PORT5,LED0_B_BIT);
			break;

		default:
			break;
	}

}


void ledOFF(uint8_t led){

	switch (led) {
		case LED_YELLOW:
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,PORT0,LED1_BIT);
			break;
		case LED_RED:
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,PORT1,LED2_BIT);
			break;
		case LED_GREEN:
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,PORT1,LED3_BIT);
			break;
		case LED_RGB_RED:
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,PORT5,LED0_R_BIT);
			break;
		case LED_RGB_GREEN:
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,PORT5,LED0_G_BIT);
			break;
		case LED_RGB_BLUE:
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,PORT5,LED0_B_BIT);
			break;
		default:
			break;
	}


}


void ledToggle(uint8_t led){

	switch (led) {
		case LED_YELLOW:
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,PORT0,LED1_BIT);
			break;
		case LED_RED:
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,PORT1,LED2_BIT);
			break;
		case LED_GREEN:
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,PORT1,LED3_BIT);
			break;
		case LED_RGB_RED:
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,PORT5,LED0_R_BIT);
			break;
		case LED_RGB_GREEN:
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,PORT5,LED0_G_BIT);
			break;
		case LED_RGB_BLUE:
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,PORT5,LED0_B_BIT);
			break;
		default:
			break;
	}

}



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




/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

