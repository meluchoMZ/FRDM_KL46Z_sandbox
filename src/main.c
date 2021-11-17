// Simple automata implementation
// Embedded Systems, 2021
// Miguel Blanco Godón

#include "MKL46Z4.h"

void init_green_led(void)
{
	SIM->COPC = 0x0;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[5] = PORT_PCR_MUX(1);
	GPIOD->PDDR |= (1 << 5);
	GPIOD->PSOR = (1 << 5);
}

void init_red_led(void)
{
	SIM->COPC = 0x0;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[29] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1 << 29);
	GPIOE->PSOR = (1 << 29);
}

void enable_sw3(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[12] = PORT_PCR_MUX(1);
	PORTC->PCR[12] |= (1 << 16);
	PORTC->PCR[12] |= (1 << 19);
	PORTC->PCR[12] |= 0x3;
	GPIOC->PDDR &= ~(1 << 12);
}

void enable_sw1(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[3] = PORT_PCR_MUX(1);
	PORTC->PCR[3] |= (1 << 16);
	PORTC->PCR[3] |= (1 << 19);
	PORTC->PCR[3] |= 0x3;
	GPIOC->PDDR &= ~(1 << 3);
}

void enable_gpio_interrupt(void)
{
	NVIC_SetPriority(31, 1);
	NVIC_EnableIRQ(31);
}

void PORTDIntHandler(void)
{
	volatile static uint8_t d1 = 0x0;
	volatile static uint8_t d2 = 0x0;
	uint8_t s;

	if ((PORTC->PCR[3] & (1 << 24)) == (1 << 24)) {
		// interupt on SW1
		PORTC->ISFR |= (1 << 3);
		d1 = (d1+0x1)%2;
	}
	if ((PORTC->PCR[12] & (1 << 24)) == (1 << 24)) {
		// interrupt on SW3
		PORTC->ISFR |= (1 << 12);
		d2 = (d2+0x1)%2;
	}
	s = d1 + d2;
	switch (s)
	{
		case 0x0: 
			GPIOD->PCOR |= (1 << 5);
			GPIOE->PSOR |= (1 << 29);
			break;
		default:
			GPIOD->PSOR |= (1 << 5);
			GPIOE->PCOR |= (1 << 29);
			break;
	}
}

int main(void)
{
	init_green_led();
	init_red_led();
	enable_sw1();
	enable_sw3();
	enable_gpio_interrupt();
	GPIOD->PTOR = (1 << 5);
	while (1)
		;
	return 0;
}
