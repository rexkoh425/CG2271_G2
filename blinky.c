#include "MKL25Z4.h"
//#include "cmsis_os.h"
#include "Board_LED.h"
#include "PWMmusic.h"
#include "PWMmotors.h"
#include "LED.h"
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define MASK(x) (1 << (x))
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define BAUD_RATE 9600

volatile uint8_t receive_data;
volatile uint8_t ISR_error = 0;


void Init_UART2() {
  uint32_t divisor,bus_clock;
  // enable clock to UART and Port E
  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  // connect UART to pins for PTE22, PTE23
  PORTE->PCR[22] = PORT_PCR_MUX(4);
  PORTE->PCR[23] = PORT_PCR_MUX(4);
  // ensure tx and rx are disabled before configuration
  UART2->C2 &= ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
  // Set baud rate to 4800 baud
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
  divisor = bus_clock/(BAUD_RATE*16);
  UART2->BDH = UART_BDH_SBR(divisor>>8);
  UART2->BDL = UART_BDL_SBR(divisor);
  // No parity, 8 bits, two stop bits, other settings;
  UART2->C1 = UART2->S2 = UART2->C3 = 0;
  // Enable transmitter and receiver
  UART2->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
	/*	
	NVIC_SetPriority(UART2_IRQn, 128); 
	NVIC_ClearPendingIRQ(UART2_IRQn); 
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
	UART2->C2 |= UART_C2_RIE_MASK;
	*/
	
}

/*
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
*/

void UART2_Transmit_Poll(uint8_t data) {
  // wait until transmit data register is empty
  while (!(UART2->S1 & UART_S1_TDRE_MASK))
  ;
  UART2->D = data;
}
uint8_t UART2_Receive_Poll(void) {
  // wait until receive data register is full
  while (!(UART2->S1 & UART_S1_RDRF_MASK))
  ;
	receive_data = UART2->D;
  return receive_data;
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
			goLeft();
		  break;
		case 0x32:
			goRight();
		  break;
		case 0x33:
			goForward();
		  break;
		case 0x34:
			goBackward();
		  break;
		default:
			stop();
		  break;
	}
}

int main()
{
  // setup code for UART, RGB LED, etc.
  Init_UART2();
  initLED();
  uint8_t rx_data;
	
	SystemCoreClockUpdate();
	//initPWM();
	initPWM_motors();
	//TPM1_C0V = 7500;
	//TPM1_C1V = 7500;
	
	/*	osKernelInitialize();	// Initialize CMSIS-RTOS
  osThreadNew(ledOn , NULL, NULL);    // Create application main thread
  osKernelStart(); 
	*/
  
  while(1)
  {
    rx_data = UART2_Receive_Poll();
    ESP_response(rx_data);
  }
}
