// Embedded systems, 2021
// Miguel Blanco Godón


#include "MKL46Z4.h"
#include "lcd.h"

#define GREEN 0
#define RED 1
#define INVALID 0
#define VALID 1
#define DOT 0
#define DASH 1
#define S 5
#define O 0
#define NONE 255
#define SO 50
#define SOS 55
#define TRUE 0
#define FALSE 1
#define TIMEOUT_DEFAULT 10000



volatile static uint8_t isSOS = FALSE;
volatile static uint8_t arDefault;
volatile static long timeout = TIMEOUT_DEFAULT;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

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

void sws_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);
}

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


void enableInterrupts(void)
{
	NVIC_SetPriority(31,1);
	NVIC_EnableIRQ(31);
}

void disableInterrupts(void)
{
	NVIC_DisableIRQ(31);
}

void SysTickIntHandler(void)
{
	if (timeout == TRUE) {
		GPIOE->PCOR |= (1 << 29);
		for (int i = 0; i < 100000; i++);
		GPIOE->PSOR |= (1 << 29);
		// request system request
		SCB->AIRCR = (0x5FA<<SCB_AIRCR_VECTKEY_Pos)|SCB_AIRCR_SYSRESETREQ_Msk;
	} else {
		timeout--;
	}
}


void PORTDIntHandler(void)
{
	uint8_t validInput = INVALID;
	uint8_t input;
	volatile static uint8_t state;
	uint8_t nextState = 0x00;
	volatile static uint8_t lcdValue = NONE;

	timeout = TIMEOUT_DEFAULT; // resets "watchdog" timer

	if ((PORTC->PCR[3] & (1 << 24)) == (1 << 24)) {
		PORTC->ISFR |= (1 << 3);
		validInput = VALID;
		input = DOT;
		GPIOD->PCOR |= (1 << 5);
		for (int i = 0; i < 100000; i++);
		GPIOD->PSOR |= (1 << 5);
	}

	if ((PORTC->PCR[12] & (1 << 24)) == (1 << 24)) {
		PORTC->ISFR |= (1 << 12);
		validInput = (validInput == VALID) ? INVALID : VALID;
		input = DASH;
		GPIOD->PCOR |= (1 << 5);
		for (int i = 0; i < 300000; i++);
		GPIOD->PSOR |= (1 << 5);
	}

	if (validInput == INVALID) {
		return;
	}
	switch (state)
	{
		// S(0)
		case 0x01:
			nextState = (input == DOT) ? 0x02 : 0x00;
			if (input == DASH)
				lcdValue = NONE;
			break;
		// S(1)
		case 0x02:
			nextState = 0x00;
			if (input == DASH) {
				lcdValue = NONE;
				break;
			}
			if (lcdValue == SO) {
				lcdValue = SOS;
				break;
			}
			lcdValue = S;
			break;
		// O(0)
		case 0x04:
			nextState = (input == DASH) ? 0x05 : 0x00;
			if (input == DOT || lcdValue == SO)
				lcdValue = NONE;
			break;
		// O(1)
		case 0x05:
			nextState = 0x00; 
			if (input == DOT) {
				lcdValue = NONE;
				break;
			}
			lcdValue = (lcdValue == S) ? SO : NONE;
			break;
		// initial state
		default:
			nextState = (input == DOT) ? 0x01 : 0x04;
			if (lcdValue == SO && input == DASH)
				lcdValue = NONE;
			if (lcdValue == S && input == DOT)
				lcdValue = NONE;
			break;
	}
	switch (lcdValue)
	{
		case S:
  			//LCD->AR = LCD_AR_BLANK_MASK; 
			//LCD->AR = arDefault;
			lcd_ini();
			lcd_set(S, 4);
			break;
		case SO:
  			//LCD->AR = LCD_AR_BLANK_MASK; 
			//LCD->AR = arDefault;
			lcd_ini();
			lcd_set(S, 3);
			lcd_set(O, 4);
			break;
		case SOS:
  			//LCD->AR = LCD_AR_BLANK_MASK; 
			//LCD->AR = arDefault;
			lcd_ini();
			lcd_set(S, 2);
			lcd_set(O, 3);
			lcd_set(S, 4);
			isSOS = TRUE;
			break;
		default:
  			LCD->AR = LCD_AR_BLANK_MASK; 
			break;
	}
	state = nextState;
}

int main(void)
{
  leds_ini();

  sw1_ini();
  sw2_ini();

  irclk_ini(); // Enable internal ref clk to use by LCD

  lcd_ini();
  arDefault = LCD->AR;
  enableInterrupts();

  SysTick_Config(0x2000U);

  while (isSOS == FALSE) 
	  ;
  
  GPIOD->PSOR |= (1 << 5);
  GPIOE->PCOR |= (1 << 29);
  disableInterrupts();
  LCD->AR = LCD_AR_BLINK(1);
  return 0;
}
