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
#include "ledRGB.h"


/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/
#define COUNT_BASE 1
#define COUNT_RESET 0
#define COUNT_MAX 25



/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

volatile uint8_t count;
volatile struct RGBcolor color={0,0,0};
struct RGBcolor palettecolor[11]={
		{25,0,0}, 	//red
		{0,25,0}, 	//green
		{0,0,25}, 	//blue
		{25,15,0}, 	//yellow
		{25,0,25}, 	//pink
		{25,25,25}, //blank
		{0,20,25}, 	//cyan
		{25,3,1}, 	//orange
		{15,5,1}, 	//brown
		{11,0,11}, //violet
		{11,20,0}, //
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void counter();

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


void counter(){

	count++;

	count%=COUNT_MAX;

	if(count>=color.red) ledOFF(LED_RGB_RED); else ledON(LED_RGB_RED);

	if(count>=color.blue) ledOFF(LED_RGB_BLUE); else ledON(LED_RGB_BLUE);

	if(count>=color.green) ledOFF(LED_RGB_GREEN); else ledON(LED_RGB_GREEN);


}

void ledRGBInit(){

	count=COUNT_RESET;
	ledsInit();
	timerInit(COUNT_BASE,&counter);
}

void ledRGBsetColorPalette(uint8_t rgb_palette){

	color.red = palettecolor[rgb_palette].red;
	color.blue = palettecolor[rgb_palette].blue;
	color.green = palettecolor[rgb_palette].green;

}

void ledRGBsetColor(struct RGBcolor rgb){

	if(rgb.red>COUNT_MAX) rgb.red=0;
	if(rgb.green>COUNT_MAX)rgb.green = 0;
	if(rgb.blue>COUNT_MAX) rgb.blue = 0;

	color.red = rgb.red;
	color.blue = rgb.blue;
	color.green = rgb.green;

}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

