#include "MKL25Z4.h"
#include "cmsis_os2.h"

#define PTE5_Pin 5
#define PTE4_Pin 4
#define PTE3_Pin 3
#define PTE2_Pin 2
#define PTE0_Pin 0
#define PTB11_Pin 11
#define PTB10_Pin 10
#define PTB9_Pin 9
#define PTB8_Pin 8

#define MASK(x) (1 << (x))

void initLED_frontback(void){
    //SCGC5 used to toggle the clock for PORTE and PORTB
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    //MUX is used to set alternate mode to mode 1 to use the right GPIO
    PORTE->PCR[PTE5_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE5_Pin] |= PORT_PCR_MUX(1);
    PORTE->PCR[PTE4_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE4_Pin] |= PORT_PCR_MUX(1);
    PORTE->PCR[PTE3_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE3_Pin] |= PORT_PCR_MUX(1);
    PORTE->PCR[PTE2_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE2_Pin] |= PORT_PCR_MUX(1);
    
    PORTB->PCR[PTB11_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PTB11_Pin] |= PORT_PCR_MUX(1);
    PORTB->PCR[PTB10_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PTB10_Pin] |= PORT_PCR_MUX(1);
    PORTB->PCR[PTB9_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PTB9_Pin] |= PORT_PCR_MUX(1);
    PORTB->PCR[PTB8_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PTB8_Pin] |= PORT_PCR_MUX(1);

    // Set Data Direction Registers for PortB and PortE to be outputs
    PTB->PDDR |= (MASK(PTB11_Pin) | MASK(PTB10_Pin) | MASK(PTB9_Pin) | MASK(PTB8_Pin));
    PTE->PDDR |= (MASK(PTE5_Pin) | MASK(PTE4_Pin) | MASK(PTE3_Pin) | MASK(PTE2_Pin));

    PORTE->PCR[PTE0_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE0_Pin] |= PORT_PCR_MUX(1);

    PTE-> PDDR |= MASK(PTE0_Pin);

    PORTE->PCR[PTE0_Pin] |= PORT_PCR_PE(1);
    PORTE->PCR[PTE0_Pin] |= PORT_PCR_PS(1);
}

