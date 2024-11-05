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

volatile uint8_t ISR_error = 0;
volatile uint8_t running_state = 0;
volatile uint8_t ending = 0;
volatile uint8_t receiveddatafromESP = 0;
volatile uint8_t receiveddataQueue = 0;
volatile uint8_t received_data = 0;


uint8_t isFrontLEDOn = 0;

osThreadId_t esp_thread;


// Define notes as frequencies in Hz
#define NOTE_E5  659
#define NOTE_D5  587
#define NOTE_FS4 370
#define NOTE_GS4 415
#define NOTE_C5  523
#define NOTE_B4  494
#define NOTE_D4  294
#define NOTE_E4  330

// Define note durations in ms
#define WHOLE_NOTE 500
#define HALF_NOTE (WHOLE_NOTE / 2)
#define QUARTER_NOTE (WHOLE_NOTE / 4)
#define EIGHTH_NOTE (WHOLE_NOTE / 8)

#define Q_SIZE (32)
	typedef struct {
	unsigned char Data[Q_SIZE];
	unsigned int Head; // points to oldest data element
	unsigned int Tail; // points to next free space
	unsigned int Size; // quantity of elements in queue
} Q_T;

Q_T rx_q;

void Q_Init(Q_T * q) {
	unsigned int i;
	for (i=0; i<Q_SIZE; i++)
	q->Data[i] = 0; // to simplify our lives when debugging
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}
int Q_Empty(Q_T * q) {
	return q->Size == 0;
}
int Q_Full(Q_T * q) {
	return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T * q, unsigned char d) {
	// What if queue is full?
	if (!Q_Full(q)) {
	q->Data[q->Tail++] = d;
	q->Tail %= Q_SIZE;
	q->Size++;
	return 1; // success
	} else
	return 0; // failure
}
unsigned char Q_Dequeue(Q_T * q) {
	// Must check to see if queue is empty before dequeueing
	unsigned char t=0;
	if (!Q_Empty(q)) {
	t = q->Data[q->Head];
	q->Data[q->Head++] = 0; // to simplify debugging
	q->Head %= Q_SIZE;
	q->Size--;
	}
	return t;
}


void playEndingTone() {
    setBuzzerFrequency(NOTE_E5); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_D5); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_FS4); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_GS4); osDelay(HALF_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);

    setBuzzerFrequency(NOTE_C5); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_B4); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_D4); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_E4); osDelay(HALF_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);

    setBuzzerFrequency(NOTE_E5); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_D5); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_FS4); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_GS4); osDelay(HALF_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);

    setBuzzerFrequency(NOTE_C5); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_E4); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_B4); osDelay(QUARTER_NOTE); setBuzzerFrequency(0); osDelay(EIGHTH_NOTE);
    setBuzzerFrequency(NOTE_GS4); osDelay(HALF_NOTE); setBuzzerFrequency(0); osDelay(HALF_NOTE);
}

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
		//if (!Q_Full(&rx_q)) {
	//	receiveddatafromESP = UART2->D; 
		//	Q_Enqueue(&rx_q, UART2->D);
	//	}
		received_data = UART2->D;
	//	osThreadFlagsSet(esp_thread,0x0001);
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

// 0x3n -> motor direction
// 0x1n -> motor speed
// 0x9n -> buzzer tone
void ESP_response(uint8_t rx_data){
	switch(rx_data) {
		case 0x11:
			motor_speed = 1;
		  break;
		case 0x12:
			motor_speed = 0.5;
		  break;
		case 0x31:
			running_state = 1;
			ending = 0;
			goLeft();
		  break;
		case 0x32:
			running_state = 1;
			ending = 0;
			goRight();
		  break;
		case 0x33:
			running_state = 1;
			ending = 0;
			goForward();
		  break;
		case 0x34:
			running_state = 1;
			ending = 0;
			goBackward();
		  break;
		case 0x35:
			running_state = 1;
			ending = 0;
			moveLeftForward();
		  break;
		case 0x36:
			running_state = 1;
			ending = 0;
			moveRightForward();
		  break;
		case 0x37:
			running_state = 1;
			ending = 0;
			moveLeftBackward();
		  break;
		case 0x38:
			running_state = 1;
			ending = 0;
			moveRightBackward();
		  break;
		case 0x39:
			running_state = 1;
			ending = 0;
			goLeftFull();
		  break;
		case 0x3a:
			running_state = 1;
			ending = 0;
			goRightFull();
		  break;
		case 0x98:
			ending = 0;
			break;
		case 0x99:
			ending = 1;
	    break;
		default:
			running_state = 0;
			stop();
			ending = 0;
		  break;
	}
}

void ESPresponse(void *argument) {
	for(;;) {
		
	//	if(!Q_Empty(&rx_q)){
		//	receiveddataQueue = Q_Dequeue(&rx_q);
		//  osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
			ESP_response(received_data);
	//	} else if (Q_Full(&rx_q)){
		//ESP_response(0xFF);
	//	}
	}
}

void playMusic(void *argument){
    for(;;){
				if(ending == 0){
					setBuzzerFrequency(WEE_FREQUENCY);
					osDelay(500);
					setBuzzerFrequency(WOO_FREQUENCY);
					osDelay(500);
				}else{
					playEndingTone();
				}
    }
}

//When running, turn on the Front LEDs 1 by 1
//When not running, turn on all Front LEDs
void ledFront(void *argument){
    for(;;){
        if(running_state == 1){
						isFrontLEDOn = 0;
						PTE->PDOR &= ~MASK(PTE2_Pin);
						PTE->PDOR &= ~MASK(PTE3_Pin);
						PTE->PDOR &= ~MASK(PTE4_Pin);
						PTE->PDOR &= ~MASK(PTE5_Pin);
						PTB->PDOR &= ~MASK(PTB8_Pin);
						PTB->PDOR &= ~MASK(PTB9_Pin);
						PTB->PDOR &= ~MASK(PTB10_Pin);
						PTB->PDOR &= ~MASK(PTB11_Pin);
					
            PTE->PDOR |= MASK(PTE5_Pin);
            osDelay(200);
						PTE->PDOR &= ~MASK(PTE5_Pin);
            PTE->PDOR |= MASK(PTE4_Pin);
            osDelay(200);
						PTE->PDOR &= ~MASK(PTE4_Pin);
            PTE->PDOR |= MASK(PTE3_Pin);
            osDelay(200);
						PTE->PDOR &= ~MASK(PTE3_Pin);
            PTE->PDOR |= MASK(PTE2_Pin);
            osDelay(200);
						PTE->PDOR &= ~MASK(PTE2_Pin);
            PTB->PDOR |= MASK(PTB11_Pin);
            osDelay(200);
					  PTB->PDOR &= ~MASK(PTB11_Pin);
            PTB->PDOR |= MASK(PTB10_Pin);
            osDelay(200);
						PTB->PDOR &= ~MASK(PTB10_Pin);
            PTB->PDOR |= MASK(PTB9_Pin);
            osDelay(200);
						PTB->PDOR &= ~MASK(PTB9_Pin);
            PTB->PDOR |= MASK(PTB8_Pin);
            osDelay(200);
						PTB->PDOR &= ~MASK(PTB8_Pin);
        } else if(!isFrontLEDOn){
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
 // osThreadNew(ledOn , NULL, NULL);    // Create application main thread
	osThreadNew(ESPresponse, NULL, NULL); // Thread to run motors
	osThreadNew(playMusic, NULL, NULL); // Thread to play music
	osThreadNew(ledFront, NULL, NULL); // Thread to run front LEDs
	osThreadNew(ledBack, NULL, NULL); // Thread to run back LEDs
  osKernelStart();
	while(1);
}
