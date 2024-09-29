#include "MKL25Z4.h"                    // Device header

unsigned int counter = 0;
unsigned int current_led = 0;

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))

static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__asm("NOP");
		nof--;
	}
}

void InitGPIO(void)
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
	
	PTD->PDOR |= MASK(BLUE_LED);
	PTB->PDOR |= MASK(RED_LED);
	PTB->PDOR |= MASK(GREEN_LED);
}

void incrementLed(void){
	
	switch(current_led){
		case 0 :
			PTD->PDOR |= MASK(BLUE_LED);
			PTB->PDOR &= ~MASK(RED_LED);
			current_led++;
			break;
		case 1 :
			PTB->PDOR |= MASK(RED_LED);
			PTB->PDOR &= ~MASK(GREEN_LED);
			current_led++;
			break;
		case 2 : 
			PTB->PDOR |= MASK(GREEN_LED);
			PTD->PDOR &= ~MASK(BLUE_LED);
			current_led = 0;
			break;
	}
}

main(void)
{
	SystemCoreClockUpdate();
	InitGPIO();
	while(1)
	{
		incrementLed();
		delay(0x80000);
	}
}
