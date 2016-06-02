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
#include "switch.h"


/*==================[macros and definitions]=================================*/


#define PORT0 0
#define PORT1 1
#define PORT2 2
#define PORT5 5

#define OUTPUT 1
#define INPUT 0

#define PACKSWITCHS 1

#define SW1_PIN 0
#define SW2_PIN 1
#define SW3_PIN 2
#define SW4_PIN 6

#define SW1_BIT 4
#define SW2_BIT 8
#define SW3_BIT 9
#define SW4_BIT 9

#define SW1_MASK (1<<SW1_BIT)
#define SW2_MASK (1<<SW2_BIT)
#define SW3_MASK (1<<SW3_BIT)
#define SW4_MASK (1<<SW4_BIT)

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

void switchsInit(){

	Chip_SCU_PinMux(PACKSWITCHS,SW1_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(PACKSWITCHS,SW2_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(PACKSWITCHS,SW3_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(PACKSWITCHS,SW4_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);

	Chip_GPIO_SetDir(LPC_GPIO_PORT,PORT0,SW1_MASK,INPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,PORT0,SW2_MASK,INPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,PORT0,SW3_MASK,INPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT,PORT1,SW4_MASK,INPUT);
}

uint8_t switchGetStatus(uint8_t button){

	uint8_t state_button;

	switch (button) {
		case TEC1:
			state_button=Chip_GPIO_ReadPortBit(LPC_GPIO_PORT,PORT0,SW1_BIT);
			break;
		case TEC2:
			state_button=Chip_GPIO_ReadPortBit(LPC_GPIO_PORT,PORT0,SW2_BIT);
			break;
		case TEC3:
			state_button=Chip_GPIO_ReadPortBit(LPC_GPIO_PORT,PORT0,SW3_BIT);
			break;
		case TEC4:
			state_button=Chip_GPIO_ReadPortBit(LPC_GPIO_PORT,PORT1,SW4_BIT);
			break;

		default:
			break;
	}
	return state_button;

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

