#include "MKL25Z4.h"

#define PTE20_Pin 20
#define PTE21_Pin 21
#define PTE29_Pin 29
#define PTE30_Pin 30
	
/* Using PTE20,21,29,30 for motors
PTE20 -> TPM1_CH0
PTE21 -> TPM1_CH1
PTE29 -> TPM0_CH2
PTE30 -> TPM0_CH3
on the motor driver, A for left side, B for right side
AIN1 for PTE20, AIN2 for PTE21
BIN1 for PTE29, BIN2 for PTE30
*/
void initPWM_motors(void){	
	  //SCGC5 used to toggle the clock for PORTE
		SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	  
	  //MUX is used to set alternate mode to mode 3 to use TPM2_CH0 etc
	  PORTE->PCR[PTE20_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE20_Pin] |= PORT_PCR_MUX(3);
	
	  PORTE->PCR[PTE21_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE21_Pin] |= PORT_PCR_MUX(3);
	
		PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(3);
	
		PORTE->PCR[PTE30_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE30_Pin] |= PORT_PCR_MUX(3);
	  
	  //SCGC6 used to toggle the clock for TPM1, TPM0, TPM2
	  SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK;
    
	  //SOPT2 used to select clock source for TPM
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	  //MOD is the value to count up to, defines the frequency
	  TPM0->MOD = 7500;
    //SC CMOD value of 1 sets it to increase for every clock edge selected
		//PS is the prescaler, 7 is 128
    TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
    TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));

    //SC CPWMS sets to up counting mode
		TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
    //C0SC MSB 1 sets to edge-aligned PWM, ELSB 1 sets to High-true pulses (clear Output on match, set Output on reload)
		//TROUBLESHOOTING HEADACHE #1: TPMx_CnSC, x is the TPM number, n is the channel number.
    TPM0_C2SC &= ~(TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK);
    TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		TPM0_C3SC &= ~(TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK);
    TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		TPM1->MOD = 7500;

    TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
    TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));

    TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

    TPM1_C0SC &= ~(TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK);
    TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
		
		TPM1_C1SC &= ~(TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK);
    TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void goForward (){
		TPM1_C0V = 7500;
		TPM1_C1V = 0;
		TPM0_C2V = 7500;
		TPM0_C3V = 0;	
}

void stop() {
		TPM1_C0V = 0;
		TPM1_C1V = 0;
		TPM0_C2V = 0;
		TPM0_C3V = 0;	
}

void goBackward () {
		TPM1_C0V = 0;
		TPM1_C1V = 7500;
		TPM0_C2V = 0;
		TPM0_C3V = 7500;	
}

void goLeft() {
	  TPM1_C0V = 0;
		TPM1_C1V = 7500;
		TPM0_C2V = 7500;
		TPM0_C3V = 0;	
}

void goRight() {
		TPM1_C0V = 7500;
		TPM1_C1V = 0;
		TPM0_C2V = 0;
		TPM0_C3V = 7500;	
}