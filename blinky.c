#include "MKL25Z4.h"
#include "Board_LED.h"
#include "PWMmusic.h"
#include "PWMmotors.h"
#include "LED.h"
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "LED_frontback.h"

#define MASK(x) (1 << (x))
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define BAUD_RATE 9600
#define PTE23_Pin 23 //RX

volatile uint8_t receive_data = 0;
volatile uint8_t ISR_error = 0;
volatile uint8_t running_state = 0;
uint8_t isFrontLEDOn = 0;


void Init_UART2() {
	uint32_t divisor,bus_clock;
	// enable clock to UART and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	// connect UART to pins for PTE22, PTE23
	PORTE->PCR[PTE23_Pin] = PORT_PCR_MUX(4);
	// ensure tx and rx are disabled before configuration
	UART2->C2 &= ~ UARTLP_C2_RE_MASK;
	// Set baud rate to 4800 baud
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock/(BAUD_RATE*16);
	UART2->BDH = UART_BDH_SBR(divisor>>8);
	UART2->BDL = UART_BDL_SBR(divisor);
	// No parity, 8 bits, two stop bits, other settings;
	UART2->C1 = UART2->S2 = UART2->C3 = 0;
	// Enable transmitter and receiver
	UART2->C2 = UART_C2_RE_MASK;
	
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);	
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK;
	
	
}


void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		receive_data = UART2->D;
	}
	if (UART2->S1 &  (UART_S1_OR_MASK |
										UART_S1_NF_MASK |
										UART_S1_FE_MASK |
										UART_S1_PF_MASK)) {
											//Handle error
											ISR_error = 1;
											
										} 
										
}

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

void ESP_response(uint8_t rx_data){
	switch(rx_data) {
		case 0x31:
			running_state = 1;
			goLeft();
		  break;
		case 0x32:
			running_state = 1;
			goRight();
		  break;
		case 0x33:
			running_state = 1;
			goForward();
		  break;
		case 0x34:
			running_state = 1;
			goBackward();
		  break;
		default:
			running_state = 0;
			stop();
		  break;
	}
}

void ESPresponse(void *argument) {
	for(;;) {
		ESP_response(receive_data);
	}
}

void playMusic(void *argument){
    for(;;){
        setBuzzerFrequency(WEE_FREQUENCY);
        osDelay(500);
        setBuzzerFrequency(WOO_FREQUENCY);
        osDelay(500);
    }
}

//When running, turn on the Front LEDs 1 by 1
//When not running, turn on all Front LEDs
void ledFront(void *argument){
    for(;;){
        if(running_state == 1){
			isFrontLEDOn = 1;
            PTE->PDOR |= MASK(PTE5_Pin);
            PTE->PDOR &= ~MASK(PTE5_Pin);
            osDelay(100);
            PTE->PDOR |= MASK(PTE4_Pin);
            PTE->PDOR &= ~MASK(PTE4_Pin);
            osDelay(100);
            PTE->PDOR |= MASK(PTE3_Pin);
            PTE->PDOR &= ~MASK(PTE3_Pin);
            osDelay(100);
            PTE->PDOR |= MASK(PTE2_Pin);
            PTE->PDOR &= ~MASK(PTE2_Pin);
            osDelay(100);
            PTB->PDOR |= MASK(PTB11_Pin);
            PTB->PDOR &= ~MASK(PTB11_Pin);
            osDelay(100);
            PTB->PDOR |= MASK(PTB10_Pin);
            PTB->PDOR &= ~MASK(PTB10_Pin);
            osDelay(100);
            PTB->PDOR |= MASK(PTB9_Pin);
            PTB->PDOR &= ~MASK(PTB9_Pin);
            osDelay(100);
            PTB->PDOR |= MASK(PTB8_Pin);
            PTB->PDOR &= ~MASK(PTB8_Pin);
            osDelay(100);
        } else {
			if(!isFrontLEDOn){
				PTE->PDOR |= MASK(PTE5_Pin);
            	PTE->PDOR |= MASK(PTE4_Pin);
            	PTE->PDOR |= MASK(PTE3_Pin);
            	PTE->PDOR |= MASK(PTE2_Pin);
            	PTB->PDOR |= MASK(PTB11_Pin);
            	PTB->PDOR |= MASK(PTB10_Pin);
            	PTB->PDOR |= MASK(PTB9_Pin);
            	PTB->PDOR |= MASK(PTB8_Pin);
				isFrontLEDOn = 1;
			}
        }
    }
}

void ledBack (void *argument){
    for(;;){
        if(running_state == 1){
            PTE->PDOR |= MASK(PTE0_Pin);
            osDelay(500);
            PTE->PDOR &= ~MASK(PTE0_Pin);
            osDelay(500);
        } else {
            PTE->PDOR |= MASK(PTE0_Pin);
            osDelay(250);
            PTE->PDOR &= ~MASK(PTE0_Pin);
            osDelay(250);
        }
    }
}

int main()
{
	// setup code for UART, RGB LED, etc.
	SystemCoreClockUpdate();
	Init_UART2();
	initLED();
	initPWM_buzzer();
	initPWM_motors();
	initLED_frontback();
	
	
	osKernelInitialize();	// Initialize CMSIS-RTOS
  	//osThreadNew(ledOn , NULL, NULL);    // Create application main thread
	osThreadNew(ESPresponse, NULL, NULL); // Thread to run motors
	osThreadNew(playMusic, NULL, NULL); // Thread to play music
	osThreadNew(ledFront, NULL, NULL); // Thread to run front LEDs
	osThreadNew(ledBack, NULL, NULL); // Thread to run back LEDs
  	osKernelStart();
	while(1);
}
