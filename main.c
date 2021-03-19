/** NOTES **
** Helpful Links
**		Understanding the Tiva DMA
**			- https://sites.google.com/site/luiselectronicprojects/tutorials/tiva-tutorials/tiva-dma/understanding-the-tiva-dma
**
**		Blink with DMA
**			- https://sites.google.com/site/luiselectronicprojects/tutorials/tiva-tutorials/tiva-dma/blink-with-dma
**
**		ADC Internal Temperature Sensor Implementation
**			- http://shukra.cedt.iisc.ernet.in/edwiki/EmSys:TM4C123_Temperature_sensor
**
**		ADC Implementation
**			- https://microcontrollerslab.com/adc-tm4c123g-tiva-c-launchpad-measure-analog-voltage-signal/
**
**		Timer Interrupt Implementation
**			- https://microcontrollerslab.com/timer-interrupt-tm4c123-generate-delay-with-timer-interrupt-service-routine/
**
**		UDMA SSI TX TRANSFER ON TM4C129
**			- https://e2e.ti.com/support/microcontrollers/other/f/other-microcontrollers-forum/397656/udma-ssi-tx-transfer-on-tm4c129
**
**		uDMA to SSI Transfer - Am I missing something?
**			- https://e2e.ti.com/support/microcontrollers/other/f/other-microcontrollers-forum/439993/udma-to-ssi-transfer---am-i-missing-something
*/

#include <stdint.h>
#include "bsp.h"
#include "TM4C123GH6PM.h"
#include <stdio.h>

int main(void){	
	// Initalize protocols
	BSP_init();
	
	while(1){	
		// Do something
	}
}
