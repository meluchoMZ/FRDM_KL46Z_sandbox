// Digital metronome 
// Embedded Systems, 2021
// Miguel Blanco Godón

#include "MKL46Z4.h"
#include "lcd.h"

#define TRUE 0
#define FALSE 1
#define TIMEOUT 1000
#define REF 60000 // 60*1000

volatile static uint8_t speed = 60, mustTick = FALSE;
volatile static unsigned long timeout = TIMEOUT;

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
	// enables and set priority for button irq
	NVIC_SetPriority(31, 1);
	NVIC_EnableIRQ(31);
}


void display(void)
{
	// cleans lcd
	LCD->AR = LCD_AR_BLANK_MASK;
	// restarts lcd
	lcd_ini();
	lcd_display_dec(speed);
}
void PORTDIntHandler(void)
{
	// disables IRQ (software button debouncing)
	NVIC_DisableIRQ(31);
	// checks which button was pressed
	// updates speed consequently
	if ((PORTD->PCR[4] & (1 << 24)) == (1 << 24)) {
		PORTD->ISFR |= (1 << 4);
		speed--;
	}
	if ((PORTD->PCR[5] & (1 << 24)) == (1 << 24)) {
		PORTD->ISFR |= (1 << 5);
		speed++;
	}
	if ((PORTD->PCR[6] & (1 << 24)) == (1 << 24)) {
		PORTD->ISFR |= (1 << 6);
		speed -= 10;
	}
	if ((PORTD->PCR[7] & (1 << 24)) == (1 << 24)) {
		PORTD->ISFR |= (1 << 7);
		speed += 10;
	}
	// establishes speed upper and lower boundaries
	if (speed > 240) 
		speed = 240;
	if (speed < 30) 
		speed = 30;

	// updates display
	display();


	// button pressed feedback 
	GPIOE->PCOR |= (1 << 29);
	for (int i = 1; i < 250000; i++);
	GPIOE->PSOR |= (1 << 29);
	// assures there is not any active button IRQ
	PORTD->ISFR |= (1 << 4);
	PORTD->ISFR |= (1 << 5);
	PORTD->ISFR |= (1 << 6);
	PORTD->ISFR |= (1 << 7);
	// reenables IRQ
	NVIC_EnableIRQ(31);
}

void enableButtons(void)
{
	SIM->COPC = 0x0;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	// PTD5
	PORTD->PCR[5] = PORT_PCR_MUX(1) | PORT_PCR_PE(1);
	PORTD->PCR[5] |= (1 << 16);
	PORTD->PCR[5] |= (1 << 19);
	PORTD->PCR[5] |= 0x3;
	GPIOD->PDDR &= ~(1 << 5);

	// PTD7
	PORTD->PCR[7] = PORT_PCR_MUX(1) | PORT_PCR_PE(1);
	PORTD->PCR[7] |= (1 << 16);
	PORTD->PCR[7] |= (1 << 19);
	PORTD->PCR[7] |= 0x3;
	GPIOD->PDDR &= ~(1 << 7);
	
	// PTD6
	PORTD->PCR[6] = PORT_PCR_MUX(1) | PORT_PCR_PE(1);
	PORTD->PCR[6] |= (1 << 16);
	PORTD->PCR[6] |= (1 << 19);
	PORTD->PCR[6] |= 0x3;
	GPIOD->PDDR &= ~(1 << 6);

	// PTl4
	PORTD->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_PE(1);
	PORTD->PCR[4] |= (1 << 16);
	PORTD->PCR[4] |= (1 << 19);
	PORTD->PCR[4] |= 0x3;
	GPIOD->PDDR &= ~(1 << 4);
}

void enableOutput(void)
{
	// PTE20 -> BUZZER & LED
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[20] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1 << 20);
	GPIOE->PCOR |= (1 << 20);
}


void irclk_ini()
{
	MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
	MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

void SysTickIntHandler(void)
{
	if (timeout == TRUE) {
		timeout = REF/speed;
		// sets flag to ring the buzzer
		mustTick = TRUE;
	} else {
		timeout--;
	}
}

void configureSystemClock(void)
{
	SysTick->CTRL = 0;
	// 48 kHz tick
	SysTick->LOAD = (DEFAULT_SYSTEM_CLOCK/1000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x7;
}

int main(void)
{
	configureSystemClock();
	init_green_led();
	init_red_led();
	irclk_ini();
	lcd_ini();
	display();
	enableButtons();
	enableOutput();
	enable_gpio_interrupt();
	
	while (1) {
		if (mustTick == TRUE) {
			// writes to connected buzzer & led
			GPIOE->PSOR |= (1 << 20);
			for (int i = 0; i < 100000; i++);
			GPIOE->PCOR |= (1 << 20);
			mustTick = FALSE;
		} 
	}
	return 0;
}

