#include "MKL46Z4.h"
#include "lcd.h"

// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// RIGHT (SW1) = PTC3 (pin 73)
// LEFT (SW2) = PTC12 (pin 88)

// Enable IRCLK (Internal Reference Clock)
// see Chapter 24 in MCU doc
#define GREEN 0
#define RED 1
#define UNDEFINED 255
#define LOCKED 0
#define UNLOCKED 1


// vars
volatile static uint8_t lock = LOCKED;
volatile static uint8_t hits = 0x0;
volatile static uint8_t misses = 0x0;
volatile static uint8_t ledStatus = UNDEFINED;



void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++);
}

// RIGHT_SWITCH (SW1) = PTC3
void sw1_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1);
  PORTC->PCR[3] |= (1 << 16);
  PORTC->PCR[3] |= (1 << 19);
  PORTC->PCR[3] |= 0x3;
  GPIOC->PDDR &= ~(1 << 3);
}

// LEFT_SWITCH (SW2) = PTC12
void sw2_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[12] |= PORT_PCR_MUX(1);
  PORTC->PCR[12] |= (1 << 16);
  PORTC->PCR[12] |= (1 << 19);
  PORTC->PCR[12] |= 0x3;
  GPIOC->PDDR &= ~(1 << 12);
}

int sw1_check()
{
  return( !(GPIOC->PDIR & (1 << 3)) );
}

int sw2_check()
{
  return( !(GPIOC->PDIR & (1 << 12)) );
}

// RIGHT_SWITCH (SW1) = PTC3
// LEFT_SWITCH (SW2) = PTC12
void sws_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);
}

// LED_GREEN = PTD5
void led_green_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}

// LED_RED = PTE29
void led_red_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}

void led_red_toggle(void)
{
  GPIOE->PTOR = (1 << 29);
}

// LED_RED = PTE29
// LED_GREEN = PTD5
void leds_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR = (1 << 29);
}

// Hit condition: (else, it is a miss)
// - Left switch matches red light
// - Right switch matches green light

void switchLeds (int green)
{
	if (green == 0)	{
		GPIOD->PCOR |= (1 << 5);
		GPIOE->PSOR |= (1 << 29);
		ledStatus = GREEN;
	} else {
		GPIOD->PSOR |= (1 << 5);
		GPIOE->PCOR |= (1 << 29);
		ledStatus = RED;
	}
}

void enableInterrupts(void)
{
	NVIC_SetPriority(31,1);
	NVIC_EnableIRQ(31);
}

void disableInterrupts(void)
{
	NVIC_DisableIRQ(31);
}
	
void PORTDIntHandler(void)
{
	if ((PORTC->PCR[3] & (1 << 24)) == (1 << 24)) {
		// interrupt on sw1
		PORTC->ISFR |= (1 << 3);
		if (ledStatus == GREEN) {
			hits++;
		} else {
			misses++;
		}
	}

	if ((PORTC->PCR[12] & (1 << 24)) == (1 << 24)) {
		// interrupt on sw2
		PORTC->ISFR |= (1 << 12);
		if (ledStatus == RED) {
			hits++;
		} else {
			misses++;
		}
	}
	// unlocking loop so that the "game" can continue
	lock = UNLOCKED;
}

int main(void)
{
  leds_ini();

  sw1_ini();
  sw2_ini();

  irclk_ini(); // Enable internal ref clk to use by LCD

  lcd_ini();
  enableInterrupts();

  // 'Random' sequence :-)
  volatile unsigned int sequence = 0x32B14D98,
    index = 0;

  while (index < 32) {
  	lcd_display_time(hits,misses);
	lock = LOCKED;
    if (sequence & (1 << index)) { //odd
      //
      // Switch on green led
	    switchLeds(GREEN);
      //
    } else { //even
      //
      // Switch on red led
	    switchLeds(RED);
      //
    }
    while (lock == LOCKED)
    {
	    ;
    }
    index++;
  }

  // Stop game and show blinking final result in LCD: hits:misses
  lcd_display_time(hits,misses);
  disableInterrupts();
  LCD->AR = LCD_AR_BLINK(1);
  // shutting down leds
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR |= (1 << 29);
  //

  while (1) {
  }

  return 0;
}
