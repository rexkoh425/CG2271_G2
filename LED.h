#include "MKL25Z4.h"
#include "Board_LED.h"
#include "cmsis_os2.h"


#define MASK(x) (1 << (x))
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1

volatile uint8_t ISR_error;

/* Initialise LED for GPIO Pins
*/
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

void LED_Turn_On(void) {
	PTB->PDOR &= ~MASK(RED_LED);
}

void LED_Turn_Off(void) {
  PTB->PDOR |= MASK(RED_LED);
}

void ledOn(void *argument) {
  for(;;){
		  LED_Turn_On();
		  osDelay(2000);
      LED_Turn_Off();
      osDelay(2000);
  }
}