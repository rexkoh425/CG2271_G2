#include "MKL25Z4.h" 

#define SW_POS 6
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1

unsigned int volatile current_led = 0;
int volatile led_control = 1;
int volatile count = 0;

#define MASK(x) (1 << (x))
void incrementLed(void);

void PORTD_IRQHandler(){
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	led_control ^= 1;
	PORTD->ISFR = 0xffffffff;
	count ++;
}

static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__asm("NOP");
		nof--;
	}
}

void initLED(){
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
			PTB->PDOR &= ~MASK(RED_LED); //on red
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

void initSwitch(void)
{
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK);
	
	PORTD->PCR[SW_POS] |= (PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0x0a));
	PTD->PDDR &= ~MASK(SW_POS);
	NVIC_EnableIRQ(PORTD_IRQn);
}


int main(void)
{
	initSwitch();
	initLED();
	
	while(1)
	{
		if(led_control){
			incrementLed();
			delay(0x8000);
			led_control = 0;
		}
	}
}
