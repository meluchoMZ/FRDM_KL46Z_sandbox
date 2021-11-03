// Embedded Systems, 2021
// Miguel Blanco Godón
#include "MKL46Z4.h"


int main(void)
{
	SIM->COPC = 0;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[5] = PORT_PCR_MUX(1);
	GPIOD->PDDR |= (1 << 5);
	GPIOD->PSOR = (1 << 5);
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[29] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1 << 29);
	GPIOE->PSOR = (1 << 29);
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[12] = PORT_PCR_MUX(1);
	PORTC->PCR[12] |= 0x3;
	GPIOC->PDDR &= ~(1 << 12);
	GPIOD->PTOR = (1 << 5);

	while (1) {
		if ((GPIOC->PDIR & (1 << 12)) == 0) {
			GPIOD->PTOR = (1 << 5);
			GPIOE->PTOR = (1 << 29);
			while ((GPIOC->PDIR & (1 << 12)) == 0);
		}

	}

	return 0;
}
